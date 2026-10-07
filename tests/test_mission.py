"""Mission structure, steering, transcription and independent forward checks."""
from datetime import datetime, timezone
import numpy as np
import pytest
from pydantic import ValidationError

from atmosphere.models import Burn, FlightPlan, Scenario, Stage, TargetOrbit, Vehicle, default_scenario
from atmosphere.mission import mission_topologies, seed_plan, seed_from_ascent, seed_from_transfer, time_to_apogee, MissionTopology, resample_burn, phase_manifest
from atmosphere.dynamics import Simulation, Cancelled, SolverDeadline, NumericalFailure, thrust_angle_limit, thrust_direction, validate_plan
from atmosphere.collocation import CollocationProblem, NodeFunction, hermite_simpson_defect
from atmosphere.earth import MU, R_E, G0, unit


def orbital_case(inclination=3):
    scenario=Scenario(vehicle=Vehicle(payload_mass_kg=1000,stages=[Stage(dry_mass_kg=1000,propellant_mass_kg=10000,
        vacuum_thrust_n=200000,isp_sea_level_s=450,isp_vacuum_s=450,ignition_limit=2,space_thrust_velocity_angle_deg=180)]),
        target=TargetOrbit(perigee_altitude_m=400000,apogee_altitude_m=600000,inclination_deg=inclination,
                          altitude_tolerance_m=2000,angle_tolerance_deg=.1))
    scenario.launch.epoch=datetime(2026,10,5,tzinfo=timezone.utc)
    scenario.environment.model='standard';scenario.constraints.max_mission_duration_s=10000
    scenario.constraints.vertical_ascent_s=0
    sim=Simulation(scenario);r=R_E+400000;v=np.sqrt(MU/r)
    sim.initial=np.array([r,0,0,0,v,0,10000.])
    speed=np.sqrt(MU*(2/r-2/(r+R_E+600000)))
    dv=np.array([0,speed*np.cos(np.radians(inclination))-v,speed*np.sin(np.radians(inclination))])
    forward=unit(sim.initial[3:6]-sim.frames.earth_velocity(0,sim.initial[:3]))
    la,lo,_=sim.frames.geographic(0,sim.initial[:3]);la,lo=np.radians([la,lo])
    up=sim.frames.matrix(0)@np.array([np.cos(la)*np.cos(lo),np.cos(la)*np.sin(lo),np.sin(la)])
    down=unit(-up+np.dot(up,forward)*forward);normal=unit(np.cross(forward,down));direction=unit(dv)
    angle=np.arccos(np.clip(np.dot(direction,forward),-1,1))/np.pi
    clock=np.arctan2(np.dot(direction,normal),np.dot(direction,down))
    duration=12000*(1-np.exp(-np.linalg.norm(dv)/(G0*450)))/scenario.vehicle.stages[0].mass_flow_kg_s
    plan=FlightPlan(guidance_frame='local_orbital',burns=[Burn(stage_index=0,duration_s=duration,angle_fractions=(angle,)*3,
                              clock_angles_rad=(clock,)*3,throttles=(1.,)*3)])
    return sim,plan


def test_topologies_include_distributed_restarts_and_respect_capabilities():
    s=default_scenario();s.vehicle.stages[0].ignition_limit=2;s.vehicle.stages[1].ignition_limit=3
    s.constraints.max_topologies=24
    options,total=mission_topologies(s)
    assert total==6 and len(options)==6
    assert (2,2) in [t.counts for t in options] and (2,3) in [t.counts for t in options]
    for topology in options:validate_plan(s,seed_plan(s,topology))
    s.constraints.max_topologies=2
    assert [t.counts for t in mission_topologies(s)[0]]==[(1,1),(2,2)]


def test_solid_motor_never_gets_restart_or_mutable_throttle():
    s=default_scenario();s.vehicle.stages[0]=Stage(engine_type='solid',solid_burn_curve=[(0,0),(.4,1),(1,.5)])
    for topology in mission_topologies(s)[0]:
        assert topology.counts[0]==1
        plan=seed_plan(s,topology,9);validate_plan(s,plan)
        assert plan.burns[0].duration_s==s.vehicle.stages[0].full_burn_s
        assert set(plan.burns[0].throttles)=={1}


def test_high_perigee_portfolio_prioritizes_a_legal_arrival_burn():
    s=default_scenario()
    s.target=TargetOrbit(perigee_altitude_m=20200000,apogee_altitude_m=20200000)
    s.vehicle.stages[0].ignition_limit=1
    s.vehicle.stages[1].ignition_limit=3
    s.constraints.max_topologies=1
    options,total=mission_topologies(s)
    assert total==3 and options[0].counts==(1,2)
    validate_plan(s,seed_plan(s,options[0]))
    s.vehicle.stages[1].ignition_limit=1
    options,total=mission_topologies(s)
    assert total==1 and options[0].counts==(1,1)


def test_high_arrival_plane_mismatch_prioritizes_available_three_burn_structure():
    from atmosphere.mission import needs_node_transfer
    s=default_scenario();s.launch.latitude_deg=28.6;s.launch.azimuth_sectors_deg=[]
    s.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,inclination_deg=0)
    s.vehicle.stages[0].ignition_limit=1;s.vehicle.stages[1].ignition_limit=3
    s.constraints.max_topologies=1
    assert needs_node_transfer(s)
    options,total=mission_topologies(s)
    assert total==3 and options[0].counts==(1,3)
    s.vehicle.stages[1].ignition_limit=2
    assert mission_topologies(s)[0][0].counts==(1,2)
    s.target.inclination_deg=None
    assert not needs_node_transfer(s)


def test_arrival_normal_seeds_requested_raan_without_requiring_inclination():
    from atmosphere.mission import arrival_normal
    target=TargetOrbit(perigee_altitude_m=None,inclination_deg=None,raan_deg=80)
    angle=np.radians(30)
    normal=arrival_normal(target,np.array([1.,0,0]),np.array([0,np.cos(angle),np.sin(angle)]))
    assert np.degrees(np.arccos(normal[2]))==pytest.approx(30)
    assert np.degrees(np.arctan2(normal[0],-normal[1]))==pytest.approx(80)
    assert target.inclination_deg is None


def test_node_aligned_seed_positions_finite_plane_change_without_new_targets():
    from atmosphere.mission import seed_from_parking
    base,ascent=orbital_case(0)
    s=base.scenario;s.constraints.max_mission_duration_s=50000
    s.vehicle.stages[0].ignition_limit=3
    s.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,inclination_deg=0)
    sim=Simulation(s)
    angle=np.radians(28.6);radius=R_E+250000
    sim.initial=np.r_[np.array([0,radius*np.cos(angle),radius*np.sin(angle)]),
                      np.array([-np.sqrt(MU/radius),0,0]),10000.]
    ascent.burns[0].duration_s=.001;ascent.burns[0].angle_fractions=(0.,)*3
    original=s.model_dump()
    candidate=seed_from_parking(sim,MissionTopology((3,)),ascent)
    validate_plan(s,candidate)
    result=sim.run(candidate,strict=True)
    assert s.model_dump()==original
    assert len(candidate.burns)==3
    assert candidate.burns[1].coast_before_s>100
    assert candidate.burns[-1].steering_reference=='inertial_velocity'
    assert max(candidate.burns[-1].angle_fractions)-min(candidate.burns[-1].angle_fractions)>.01
    assert result['orbit']['bound']
    assert result['orbit']['inclination_deg']<.2
    assert all(result['constraint_checks'].values())
    with pytest.raises(ValueError,match='three liquid'):
        seed_from_parking(sim,MissionTopology((2,)),ascent)


def test_parking_seed_does_not_add_intermediate_target_constraints():
    s=default_scenario();s.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,inclination_deg=0)
    s.vehicle.stages[-1].ignition_limit=3;s.constraints.max_topologies=24
    ascent=seed_plan(s,mission_topologies(s)[0][0])
    ascent.burns[-1].duration_s*=.2
    before=s.target.model_dump()
    topology=next(t for t in mission_topologies(s)[0] if t.counts==(1,3))
    plan=seed_from_ascent(s,topology,ascent,parking=True)
    validate_plan(s,plan)
    assert len(plan.burns)==4 and plan.burns[-1].coast_before_s>10000
    assert s.target.model_dump()==before


def test_apogee_estimate_accounts_for_already_elapsed_transfer_time():
    rp=R_E+200000;ra=R_E+35786000;a=(rp+ra)/2;e=(ra-rp)/(ra+rp)
    anomaly=1.2;r=a*(1-e*np.cos(anomaly));n=np.sqrt(MU/a**3)
    # Eccentric anomaly parameterization in an inertial orbital plane.
    position=a*np.array([np.cos(anomaly)-e,np.sqrt(1-e*e)*np.sin(anomaly),0])
    velocity=np.sqrt(MU*a)/r*np.array([-np.sin(anomaly),np.sqrt(1-e*e)*np.cos(anomaly),0])
    expected=(np.pi-anomaly+e*np.sin(anomaly))/n
    assert time_to_apogee(position,velocity)==pytest.approx(expected,rel=1e-12)
    assert expected<np.pi/n


@pytest.mark.parametrize('cone',[15,90,180])
def test_inertial_arrival_keeps_real_velocity_cone(cone):
    sim,plan=orbital_case(0);stage=sim.scenario.vehicle.stages[0]
    stage.space_thrust_velocity_angle_deg=cone
    r=R_E+35786000
    sim.initial=np.array([r,0,0,0,1600.,0,10000.])
    plan.burns[0].steering_reference='inertial_velocity'
    plan.burns[0].angle_fractions=(0.,)*3
    derivative,row=sim.forces(0,sim.initial,0,plan.burns[0],-100,90)
    assert row['thrust_velocity_angle_deg']<=cone+1e-8
    if cone==180:assert derivative[4]>0  # Inertial prograde at transfer apogee.
    else:assert row['thrust_velocity_angle_deg']==pytest.approx(cone,abs=1e-8)
    locked=sim.initial.copy();locked[3:6]=sim.frames.earth_velocity(0,locked[:3])
    _,row=sim.forces(0,locked,0,plan.burns[0],-100,90)
    assert row['thrust_velocity_angle_deg']==0


def test_transfer_seed_preserves_ascent_and_adds_legal_inertial_arrival():
    sim,ascent=orbital_case(0)
    sim.scenario.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,inclination_deg=0)
    sim.scenario.constraints.max_mission_duration_s=50000
    sim.scenario.vehicle.stages[0].ignition_limit=3
    rp=R_E+200000;ra=R_E+35786000
    terminal=np.array([rp,0,0,0,np.sqrt(MU*(2/rp-2/(rp+ra))),0,9000.])
    target=sim.scenario.target.model_dump()
    plan=seed_from_transfer(sim,MissionTopology((3,)),ascent,terminal,1000)
    validate_plan(sim.scenario,plan)
    assert len(plan.burns)==3
    assert sum(b.duration_s for b in plan.burns[:-1])==pytest.approx(ascent.burns[0].duration_s)
    assert plan.burns[-1].coast_before_s>15000
    assert plan.burns[-1].steering_reference=='inertial_velocity'
    assert plan.burns[-1].duration_s>0
    assert sim.scenario.target.model_dump()==target
    assert phase_manifest(sim.scenario,plan)[-1]['steering_reference']=='inertial_velocity'


def test_radial_ascent_steering_keeps_commanded_heading():
    up=np.array([1.,0,0]);east=np.array([0.,1,0])
    # Tiny gravity-driven north velocity must not choose the launch plane.
    direction,_=thrust_direction(np.array([100.,0,.001]),up,east,.1,0)
    assert direction[1]>.09 and abs(direction[2])<1e-4


def test_old_control_arrays_migrate_and_refine_without_changing_commands():
    burn=Burn(stage_index=0,duration_s=100,angle_fractions=(.2,.4,.8),clock_angles_rad=(0,.1,.2),throttles=(.4,.7,1))
    refined=resample_burn(burn,9)
    for key in ('angle_fractions','clock_angles_rad','throttles'):
        old=getattr(burn,key);new=getattr(refined,key)
        assert np.interp(np.linspace(0,1,101),np.linspace(0,1,9),new)==pytest.approx(np.interp(np.linspace(0,1,101),[0,.5,1],old))
    with pytest.raises(ValidationError):Burn(stage_index=0,duration_s=1,angle_fractions=(0,1))


def test_steering_capability_inherits_and_blends_only_when_explicit():
    stage=Stage(max_thrust_velocity_angle_deg=7)
    assert thrust_angle_limit(stage,400000)==7
    stage.space_thrust_velocity_angle_deg=180
    assert thrust_angle_limit(stage,90000)==7
    assert thrust_angle_limit(stage,110000)==pytest.approx(93.5)
    assert thrust_angle_limit(stage,120000)==180


def test_normal_steering_does_not_flip_when_velocity_crosses_launch_heading():
    up=np.array([1.,0,0]);heading=np.array([0.,1,0]);directions=[]
    for normal_speed in (-1e-4,0,1e-4,1.,100.):
        velocity=np.array([0.,7500.,normal_speed])
        direction,_=thrust_direction(velocity,up,heading,np.pi/2,np.pi/2)
        directions.append(direction)
        assert direction[2]>.999
    assert np.dot(directions[0],directions[2])>1-1e-9


def test_arrival_window_and_final_coast_are_rechecked_by_forward_simulation():
    sim,plan=orbital_case(0)
    sim.scenario.target=TargetOrbit(perigee_altitude_m=None,inclination_deg=0,arrival_time_max_s=500)
    plan.final_coast_s=600
    result=sim.run(plan,strict=True)
    assert not result['constraint_checks']['arrival_window'] and result['status']=='Target Not Reached'
    assert any(e['event']=='Arrival Coast' for e in result['events'])
    ledger=result['propellant_ledger'][0]
    expected=sim.scenario.vehicle.stages[0].mass_flow_kg_s*plan.burns[0].duration_s
    assert ledger['burned_kg']==pytest.approx(expected,abs=1e-5)
    assert ledger['initial_kg']==pytest.approx(ledger['burned_kg']+ledger['remaining_kg'],abs=1e-5)
    assert phase_manifest(sim.scenario,plan)[-1]['kind']=='arrival_coast'


def test_invalid_arrival_window_is_rejected():
    with pytest.raises(ValidationError):TargetOrbit(arrival_time_min_s=100,arrival_time_max_s=10)


def test_quadrature_defect_reference_exact_for_cubic_state():
    # y=t^3, y'=3t^2: Simpson integration is exact, unlike trapezoidal.
    assert hermite_simpson_defect([0],[8],[0],[3],[12],2)==pytest.approx([0])


def test_transfer_coast_mesh_refinement_reduces_production_trajectory_defects():
    sim,plan=orbital_case(0)
    sim.scenario.constraints.max_mission_duration_s=20000
    rp,ra=R_E+200000,R_E+20200000
    semimajor=(rp+ra)/2
    sim.initial=np.array([rp,0.,0.,0.,np.sqrt(MU*(2/rp-1/semimajor)),0.,10000.])
    plan.burns[0].duration_s=.001
    plan.final_coast_s=np.pi*np.sqrt(semimajor**3/MU)
    reference=sim.run(plan,strict=True)['orbit']
    sim.scenario.target=TargetOrbit(perigee_altitude_m=reference['perigee_altitude_m'],
        apogee_altitude_m=reference['apogee_altitude_m'],arrival_phase_deg=reference['arrival_phase_deg'])
    defects=[]
    for mesh in (12,24):
        problem=CollocationProblem(sim,plan,mesh)
        values=np.asarray(problem.constraint_function(problem.w0)).ravel()
        defects.append(np.max(np.maximum(problem.gl-values,0)+np.maximum(values-problem.gu,0)))
    assert defects[1]<defects[0]/4
    assert defects[1]<3e-4


def test_coast_resolution_rejects_unresolved_multirevolution_intervals():
    base,plan=orbital_case(0)
    base.scenario.constraints.max_mission_duration_s=20000
    sim=Simulation(base.scenario)
    radius=R_E+400000
    sim.initial=np.array([radius,0.,0.,0.,np.sqrt(MU/radius),0.,10000.])
    plan.burns[0].duration_s=.001
    plan.final_coast_s=4*np.pi*np.sqrt(radius**3/MU)
    reference=sim.run(plan,strict=True)
    orbit=reference['orbit']
    sim.scenario.target=TargetOrbit(perigee_altitude_m=orbit['perigee_altitude_m'],
        apogee_altitude_m=orbit['apogee_altitude_m'],arrival_phase_deg=orbit['arrival_phase_deg'])
    advances=[]
    for mesh in (8,64):
        problem=CollocationProblem(sim,plan,mesh)
        advances.append(float(np.max(np.asarray(problem.coast_resolution_function(problem.w0)))))
    assert reference['summary']['arrival_time_s']>=plan.final_coast_s
    assert advances[0]>.5
    assert advances[1]<.5


def test_invalid_terminal_state_has_finite_infeasibility_residuals():
    sim,plan=orbital_case(0);problem=CollocationProblem(sim,plan,4)
    for state in (np.zeros(7),np.array([1,0,0,0,0,0,.5]),np.full(7,np.nan)):
        output=np.asarray(problem.terminal(state)).ravel()
        assert np.all(np.isfinite(output))
        assert np.all(output[-2:]<0)


def test_objective_polishing_requires_connected_flight_not_just_terminal_match():
    sim,plan=orbital_case(3)
    reference=sim.run(plan,strict=True)
    orbit=reference['orbit']
    sim.scenario.target=TargetOrbit(perigee_altitude_m=orbit['perigee_altitude_m'],apogee_altitude_m=orbit['apogee_altitude_m'],inclination_deg=orbit['inclination_deg'])
    problem=CollocationProblem(sim,plan,8)
    assert problem.feasible(problem.w0)
    disconnected=problem.w0.copy()
    # The final target error is unchanged, but an intermediate state no longer
    # connects to its adjacent intervals.
    disconnected[problem.state_slices[0].start]+=.01
    assert np.array_equal(disconnected[problem.final_slice],problem.w0[problem.final_slice])
    assert not problem.feasible(disconnected)


def test_feasibility_search_retains_consistent_bound_seed_and_stops_early():
    sim,plan=orbital_case(3)
    reference=sim.run(plan,strict=True)
    orbit=reference['orbit']
    sim.scenario.target=TargetOrbit(perigee_altitude_m=orbit['perigee_altitude_m'],apogee_altitude_m=orbit['apogee_altitude_m'],inclination_deg=orbit['inclination_deg'])
    problem=CollocationProblem(sim,plan,8)
    candidate,stats=problem.solve(lambda value:None,100,60)
    assert problem.feasibility_ready
    assert problem.feasible(problem.last_x)
    assert stats['return_status']=='User_Requested_Stop'
    assert stats['iter_count']==0  # no default 1-percent displacement of the seed
    assert sim.run(candidate,strict=True)['status']=='Target Achieved'


def test_solid_collocation_node_uses_full_burn_profile_fraction():
    sim,plan=orbital_case(0)
    sim.scenario.vehicle.stages[0]=Stage(engine_type='solid',dry_mass_kg=1000,propellant_mass_kg=10000,
        vacuum_thrust_n=200000,isp_sea_level_s=450,isp_vacuum_s=450,solid_burn_curve=[(0,.2),(.3,1),(1,.4)])
    stage=sim.scenario.vehicle.stages[0]
    plan=FlightPlan(burns=[Burn(stage_index=0,duration_s=stage.full_burn_s,angle_fractions=(0,)*3)])
    problem=CollocationProblem(sim,plan,4)
    node=NodeFunction(problem,0,True,True)
    z=np.r_[sim.initial/problem.scale(0),stage.full_burn_s*.3/1000,0,0,1,.5,.3]
    output=np.asarray(node(z)).reshape(-1)
    assert output[6]*stage.propellant_mass_kg/1000==pytest.approx(-stage.mass_flow_kg_s,rel=1e-9)


@pytest.mark.parametrize('stop',[Cancelled,SolverDeadline])
def test_collocation_preserves_cancellation_and_deadline(stop):
    sim,plan=orbital_case(0);problem=CollocationProblem(sim,plan,4)
    count=0
    def check():
        nonlocal count
        count+=1
        if count>20:raise stop()
    sim.check=check
    with pytest.raises(stop):problem.solve(lambda v:None,100,20)


def test_combined_orbit_raise_and_plane_change_passes_independent_replay():
    """An actual finite-burn NLP solve, not an impulsive maneuver substituted as flight."""
    sim,plan=orbital_case(3)
    problem=CollocationProblem(sim,plan,8)
    candidate,stats=problem.solve(lambda value:None,40,60)
    result=sim.run(candidate,strict=True)
    assert result['status']=='Target Achieved',result['target_comparison']
    assert all(result['constraint_checks'].values())
    assert result['metadata']['verified_with_strict_integration']
    assert result['orbit']['inclination_deg']==pytest.approx(3,abs=.1)
    assert result['summary']['propellant_burned_kg']>0
    assert len(candidate.burns)==1


def test_ascent_initialization_stops_on_a_continuous_target_seed():
    from atmosphere.optimization import warm_start
    sim,_=orbital_case(0)
    stage=sim.scenario.vehicle.stages[0];stage.propellant_mass_kg=100
    sim.initial[6]=100
    plan=FlightPlan(guidance_frame='local_orbital',burns=[Burn(stage_index=0,duration_s=.85*stage.full_burn_s,
        angle_fractions=(.25,.1,.025))])
    reference=sim.run(plan,strict=True)
    orbit=reference['orbit']
    sim.scenario.target=TargetOrbit(perigee_altitude_m=orbit['perigee_altitude_m'],apogee_altitude_m=orbit['apogee_altitude_m'],inclination_deg=orbit['inclination_deg'])
    candidate,result=warm_start(sim,lambda value:None,'local_orbital',stop_on_target=True)
    assert sim.warm_evaluations==1 and sim.warm_stopped_on_target
    assert result['status']=='Target Achieved'
    assert sim.run(candidate,strict=True)['status']=='Target Achieved'


def test_strict_replay_rejects_chattering_at_the_low_speed_guidance_switch():
    sim,_=orbital_case(0)
    position=np.array([R_E+35786000,0.,0.])
    state=np.r_[position,sim.frames.earth_velocity(0,position)+[0,2.,0],10000.]
    burn=Burn(stage_index=0,duration_s=5,angle_fractions=(1.,)*3)
    sim._guidance_frame='local_orbital'
    with pytest.raises(NumericalFailure,match='Integration stalled'):
        sim.integrate(state,0,5,0,burn,strict=True)
