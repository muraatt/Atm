"""Joint design variables must change production flight, not just a budget table."""
from datetime import datetime, timezone
from types import SimpleNamespace
import casadi as ca
import numpy as np
import pytest
from pydantic import ValidationError

from atmosphere.models import default_scenario, StageMass, StageMassBounds, Burn, FlightPlan, Scenario
from atmosphere.sizing import apply_stage_masses, mass_bounds, seed_masses, masses
from atmosphere.dynamics import Simulation, NumericalFailure
from atmosphere.collocation import CollocationProblem
from atmosphere.derivatives import FrameLookup, AtmosphereLookup, node_function
from atmosphere.earth import R_E, V_SCALE, geodetic_to_ecef
from atmosphere.mission import seed_from_ascent, MissionTopology


@pytest.fixture(scope='module')
def sim():
    s=default_scenario();s.vehicle.optimize_stage_masses=True
    s.environment.model='standard';s.launch.epoch=datetime(2026,10,5,tzinfo=timezone.utc)
    s.constraints.max_mission_duration_s=5000;s.constraints.vertical_ascent_s=0
    return Simulation(s)


def test_structural_bounds_and_fixed_engine_payload(sim):
    original=sim.scenario.model_dump(mode='json')
    values=seed_masses(sim.scenario);values[1].propellant_mass_kg*=.6
    sized=apply_stage_masses(sim.scenario,values)
    assert sized.vehicle.initial_mass_kg<sim.scenario.vehicle.initial_mass_kg
    assert sized.vehicle.payload_mass_kg==sim.scenario.vehicle.payload_mass_kg
    for before,after in zip(sim.scenario.vehicle.stages,sized.vehicle.stages):
        for key in ('engine_id','engine_count','vacuum_thrust_n','isp_vacuum_s','isp_sea_level_s','ignition_limit'):
            assert getattr(before,key)==getattr(after,key)
    assert sim.scenario.model_dump(mode='json')==original
    values[0].dry_mass_kg=1
    with pytest.raises(ValueError,match='bounds'):apply_stage_masses(sim.scenario,values)


def test_seed_respects_structure_even_if_input_is_outside_bounds(sim):
    s=sim.scenario.model_copy(deep=True)
    s.vehicle.stages[1].mass_bounds=StageMassBounds(dry_mass_min_kg=1000,dry_mass_max_kg=5000,
        propellant_min_kg=1000,propellant_max_kg=200000,hardware_mass_kg=2000,tank_structure_ratio=.05)
    sized=apply_stage_masses(s,seed_masses(s))
    upper=sized.vehicle.stages[1]
    assert upper.propellant_mass_kg<=60000 and upper.dry_mass_kg>=2000+.05*upper.propellant_mass_kg


def test_solid_masses_and_profile_are_fixed(sim):
    s=sim.scenario.model_copy(deep=True);s.vehicle.stages[0].engine_type='solid'
    values=masses(s);values[0].propellant_mass_kg*=.9
    with pytest.raises(ValueError,match='Solid motor'):apply_stage_masses(s,values)


def test_sizing_requires_mission_solver(sim):
    data=sim.scenario.model_dump(mode='json');data['constraints']['solver_method']='legacy'
    with pytest.raises(ValidationError,match='Stage mass'):Scenario.model_validate(data)


@pytest.mark.parametrize('stage_index',[0,1])
@pytest.mark.parametrize('steering_reference',['air_relative','inertial_velocity'])
def test_mass_derivatives_match_independent_production(sim,stage_index,steering_reference):
    plan=FlightPlan(guidance_frame='local_orbital',burns=[Burn(stage_index=0,duration_s=10)])
    owner=SimpleNamespace(sim=sim,seed_plan=plan,interrupted=None,design_enabled=True,
        dry_scales=np.array([s.dry_mass_kg for s in sim.scenario.vehicle.stages]),
        fuel_scales=np.array([s.propellant_mass_kg for s in sim.scenario.vehicle.stages]))
    owner.scale=lambda i:np.r_[[R_E]*3,[V_SCALE]*3,owner.fuel_scales[i]]
    owner.frame_lookup=FrameLookup(owner);owner.atmosphere_lookup=AtmosphereLookup(owner)
    function=node_function(owner,stage_index,True,False,steering_reference)
    time=100.;r=sim.frames.matrix(time)@geodetic_to_ecef(28.5,-80.6,35000)
    state=np.r_[r,[500.,1500.,1200.],owner.fuel_scales[stage_index]*.3]
    factors=np.array([1.2,1.3,.8,1.1])
    point=np.r_[state/owner.scale(stage_index),time/1000,.2,.1,.8,.5,.4,factors]
    def independent(z):
        n=len(owner.dry_scales)
        design=[StageMass(dry_mass_kg=float(d),propellant_mass_kg=float(p)) for d,p in zip(z[13:13+n]*owner.dry_scales,z[13+n:]*owner.fuel_scales)]
        sized=sim.for_design(design);sized._guidance_frame='local_orbital'
        burn=Burn(stage_index=stage_index,duration_s=1,steering_reference=steering_reference,
                  angle_fractions=(z[8],)*3,clock_angles_rad=(z[9],)*3,throttles=(z[10],)*3)
        derivative=sized.forces(z[7]*1000,z[:7]*owner.scale(stage_index),stage_index,burn,z[7]*1000-z[12],z[11]*180)[0]
        return derivative*1000/owner.scale(stage_index)
    np.testing.assert_allclose(np.asarray(function(point)).ravel()[:7],independent(point),rtol=1e-10,atol=1e-10)
    symbol=ca.MX.sym('design_state',len(point))
    jac=np.asarray(ca.Function('design_jac',[symbol],[ca.jacobian(function(symbol),symbol)])(point))[:7,13:]
    numeric=np.column_stack([(independent(point+np.eye(len(point))[i]*1e-5)-independent(point-np.eye(len(point))[i]*1e-5))/2e-5 for i in range(13,len(point))])
    np.testing.assert_allclose(jac,numeric,rtol=2e-5,atol=1e-7)
    assert np.linalg.norm(jac[3:6,1])>0 # The upper dry mass is always carried.
    assert (np.linalg.norm(jac[3:6,3])>0)==(stage_index==0) # Upper fuel is carried by the booster.


def test_transcription_and_replay_use_same_loaded_design(sim):
    values=seed_masses(sim.scenario);values[1].propellant_mass_kg*=.6
    plan=FlightPlan(stage_masses=values,guidance_frame='local_orbital',burns=[Burn(stage_index=0,duration_s=30),Burn(stage_index=1,duration_s=30)])
    problem=CollocationProblem(sim,plan,4)
    decoded=problem.decode(np.asarray(problem.w0))
    assert decoded.stage_masses==values
    result=sim.run(decoded,strict=True)
    assert result['stage_sizing']['bounds_verified']
    assert result['input_scenario']['vehicle']['stages'][1]['propellant_mass_kg']==65000
    assert result['scenario']['vehicle']['stages'][1]['propellant_mass_kg']==39000
    for row in result['propellant_ledger']:
        assert row['initial_kg']==pytest.approx(row['burned_kg']+row['remaining_kg']+row['discarded_kg'],abs=1e-5)
    assert result['series'][0]['mass_kg']==pytest.approx(3000+14000+220000+3500+39000)
    reset=seed_from_ascent(sim.scenario,MissionTopology((1,2)),decoded)
    assert reset.stage_masses==values


def test_invalid_sized_candidate_is_rejected_before_replay(sim):
    plan=FlightPlan(stage_masses=[StageMass(dry_mass_kg=1,propellant_mass_kg=1)]*2,burns=[Burn(stage_index=0,duration_s=1),Burn(stage_index=1,duration_s=1)])
    with pytest.raises(NumericalFailure,match='bounds'):sim.run(plan)


def test_no_liftoff_input_still_searches_sized_designs(sim,monkeypatch):
    import atmosphere.optimization as optimization
    import atmosphere.mission_solver as mission_solver
    from atmosphere.dynamics import SolverDeadline
    s=sim.scenario.model_copy(deep=True);s.vehicle.stages[0].vacuum_thrust_n/=4
    s.constraints.solver_budget_s=5;s.constraints.refinement_levels=1;s.constraints.max_topologies=1
    calls=[]
    monkeypatch.setattr(optimization,'warm_start',lambda *a,**k:(None,None))
    def attempted(sim,plan,mesh):
        calls.append(plan.stage_masses)
        raise SolverDeadline('search_time_limit')
    monkeypatch.setattr(mission_solver,'CollocationProblem',attempted)
    result=mission_solver.optimize_mission(s)
    assert calls and calls[0] is not None
    assert result['optimizer']['termination_reason']!='no_liftoff'
    assert result['input_scenario']==s.model_dump(mode='json')
