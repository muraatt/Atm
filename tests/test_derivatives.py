"""Compare accelerated values and derivatives to independent production physics."""
from datetime import datetime, timezone
from types import SimpleNamespace

import casadi as ca
import numpy as np
import pytest

from atmosphere.models import Burn, FlightPlan, default_scenario
from atmosphere.dynamics import Simulation
from atmosphere.environment import MSISAtmosphere, StandardAtmosphere
from atmosphere.earth import R_E, V_SCALE, geodetic_to_ecef
from atmosphere.derivatives import AtmosphereLookup, FrameLookup, node_function


def air_vector(state):
    return np.r_[state.density_kg_m3, state.pressure_pa, state.sound_speed_m_s or 1., state.wind_ecef_m_s]


@pytest.fixture(scope='module', params=['standard', 'msis'])
def provider(request):
    s = default_scenario()
    s.environment.solar_mode = 'nominal'
    s.launch.epoch = datetime(2026, 10, 5, tzinfo=timezone.utc)
    return StandardAtmosphere() if request.param == 'standard' else MSISAtmosphere(s.environment, s.launch)


@pytest.mark.parametrize('height', [-250., 3150., 25730., 85123., 115700., 414600., 1000001.])
def test_provider_derivatives_match_independent_queries(provider, height):
    point = np.array([37.43, -75.26, height, 1783.2])
    state, analytic = provider.sample_with_derivatives(*point)
    np.testing.assert_array_equal(air_vector(state), air_vector(provider.sample(*point)))
    numeric = np.empty((6, 4))
    for i, step in enumerate([1e-4, 1e-4, .05, .1]):
        delta = np.eye(4)[i]*step
        numeric[:, i] = (air_vector(provider.sample(*(point+delta)))-air_vector(provider.sample(*(point-delta))))/(2*step)
    np.testing.assert_allclose(analytic, numeric, rtol=2e-5, atol=2e-9)


@pytest.fixture(scope='module')
def accelerated_nodes():
    s = default_scenario();s.environment.solar_mode='nominal'
    s.launch.epoch=datetime(2026,10,5,tzinfo=timezone.utc)
    s.launch.azimuth_sectors_deg=[(350,120),(160,200)]
    s.constraints.max_mission_duration_s=20000
    s.vehicle.stages[0].space_thrust_velocity_angle_deg=160
    s.vehicle.stages[0].cd_table=[(0,.2),(.8,.35),(1.2,.5),(5,.15)]
    sim=Simulation(s);plan=FlightPlan(guidance_frame='local_orbital',burns=[Burn(stage_index=0,duration_s=100)])
    owner=SimpleNamespace(sim=sim,seed_plan=plan,interrupted=None)
    owner.scale=lambda i:np.r_[[R_E]*3,[V_SCALE]*3,s.vehicle.stages[i].propellant_mass_kg]
    owner.frame_lookup=FrameLookup(owner);owner.atmosphere_lookup=AtmosphereLookup(owner)
    sim._guidance_frame=plan.guidance_frame
    return owner,{(powered,first):node_function(owner,0,powered,first) for powered,first in [(True,False),(True,True),(False,False)]}


@pytest.mark.parametrize('height,time,powered,first', [(0,0,True,True),(3000,12,True,True),(35000,137.21,True,False),
    (110000,223.1,True,False),(400000,351.3,True,False),(35786000,6003.23,False,False),
    (-200,137.21,True,False)])
def test_accelerated_nodes_match_production_forces(accelerated_nodes,height,time,powered,first):
    owner,functions=accelerated_nodes;sim=owner.sim
    position=sim.frames.matrix(time)@geodetic_to_ecef(37.9,-75.43,height)
    if height==0:
        position=sim.initial[:3].copy()
    velocity=sim.frames.earth_velocity(time,position)+(np.zeros(3) if height==0 else np.array([731.,1412.,821.]))
    state=np.r_[position,velocity,sim.scenario.vehicle.stages[0].propellant_mass_kg*.63]
    command=np.array([.24,.53,.8]);azimuth=84.;fraction=.47
    z=np.r_[state/owner.scale(0),time/1000,command,azimuth/180,fraction]
    burn=Burn.model_construct(stage_index=0,duration_s=1,coast_before_s=0,angle_fractions=(command[0],)*2,
                             clock_angles_rad=(command[1],)*2,throttles=(command[2],)*2) if powered else None
    derivative,row=sim.forces(time,state,0,burn,0 if first else time-fraction,azimuth)
    clearance,_=sim.surface_clearance(time,position)
    expected=np.r_[derivative*1000/owner.scale(0),clearance/R_E,row['dynamic_pressure_pa']/100000,row['load_factor_g']/10,sim.departure_margin([row])/180]
    np.testing.assert_allclose(np.asarray(functions[(powered,first)](z)).ravel(),expected,rtol=2e-10,atol=2e-11)


def test_node_jacobian_matches_central_production_differences(accelerated_nodes):
    owner,functions=accelerated_nodes;time=137.21
    position=owner.sim.frames.matrix(time)@geodetic_to_ecef(37.9,-75.43,35000)
    state=np.r_[position,[731.,1412.,821.],100000.]
    point=np.r_[state/owner.scale(0),time/1000,.24,.53,.8,84/180,.47]
    function=functions[(True,False)]
    symbol=ca.MX.sym('z',13)
    analytic=np.asarray(ca.Function('node_jacobian',[symbol],[ca.jacobian(function(symbol),symbol)])(point))
    numeric=np.empty((11,13))
    for i in range(13):
        delta=np.eye(13)[i]*1e-6
        numeric[:,i]=(np.asarray(function(point+delta)).ravel()-np.asarray(function(point-delta)).ravel())/2e-6
    assert np.all(np.isfinite(analytic))
    np.testing.assert_allclose(analytic,numeric,rtol=2e-4,atol=2e-6)


def test_pad_jacobian_is_finite(accelerated_nodes):
    owner,functions=accelerated_nodes
    point=np.r_[owner.sim.initial/owner.scale(0),0,.2,0,1,.5,0]
    symbol=ca.MX.sym('pad',13);function=functions[(True,True)]
    jac=np.asarray(ca.Function('pad_jac',[symbol],[ca.jacobian(function(symbol),symbol)])(point))
    assert np.all(np.isfinite(jac))


@pytest.mark.parametrize('height,angle',[(35000,.35),(400000,.08),(35786000,0.),(35786000,.2)])
def test_inertial_steering_values_and_jacobian_match_production(accelerated_nodes,height,angle):
    owner,_=accelerated_nodes;sim=owner.sim;time=1500.
    position=sim.frames.matrix(time)@geodetic_to_ecef(37.9,-75.43,height)
    velocity=np.cross(sim.frames._angular[0],position)*.6+np.array([100.,200.,80.])
    state=np.r_[position,velocity,100000.]
    point=np.r_[state/owner.scale(0),time/1000,angle,.43,.8,84/180,.47]
    function=node_function(owner,0,True,False,'inertial_velocity')
    def independent(z):
        y=z[:7]*owner.scale(0);t=z[7]*1000
        burn=Burn(stage_index=0,duration_s=1,steering_reference='inertial_velocity',
                  angle_fractions=(z[8],)*2,clock_angles_rad=(z[9],)*2,throttles=(z[10],)*2)
        derivative,row=sim.forces(t,y,0,burn,t-z[12],z[11]*180)
        clearance,_=sim.surface_clearance(t,y[:3])
        return np.r_[derivative*1000/owner.scale(0),clearance/R_E,row['dynamic_pressure_pa']/100000,
                     row['load_factor_g']/10,sim.departure_margin([row])/180]
    np.testing.assert_allclose(np.asarray(function(point)).ravel(),independent(point),rtol=2e-9,atol=3e-10)
    symbol=ca.MX.sym('inertial_node',13)
    jac=np.asarray(ca.Function('inertial_jac',[symbol],[ca.jacobian(function(symbol),symbol)])(point))
    numeric=np.empty((11,13))
    for i in range(13):
        delta=np.eye(13)[i]*1e-6
        if i==8 and angle==0:
            numeric[:,i]=(-3*independent(point)+4*independent(point+delta)-independent(point+2*delta))/2e-6
        else:numeric[:,i]=(independent(point+delta)-independent(point-delta))/2e-6
    assert np.all(np.isfinite(jac))
    np.testing.assert_allclose(jac,numeric,rtol=3e-4,atol=3e-6)


@pytest.mark.parametrize('latitude',[0.,90.,-90.])
def test_equatorial_and_polar_pad_derivatives_are_finite(latitude):
    s=default_scenario();s.environment.model='standard'
    s.launch.latitude_deg=latitude;s.launch.longitude_deg=0
    s.launch.azimuth_sectors_deg=[(0,360)]
    s.constraints.max_mission_duration_s=5000
    sim=Simulation(s)
    owner=SimpleNamespace(sim=sim,seed_plan=SimpleNamespace(guidance_frame='local_orbital'),interrupted=None)
    owner.scale=lambda i:np.r_[[R_E]*3,[V_SCALE]*3,s.vehicle.stages[i].propellant_mass_kg]
    owner.frame_lookup=FrameLookup(owner);owner.atmosphere_lookup=AtmosphereLookup(owner)
    function=node_function(owner,0,True,True)
    point=np.r_[sim.initial/owner.scale(0),0,.2,0,1,.5,0]
    symbol=ca.MX.sym('pole',13)
    jac=np.asarray(ca.Function('pole_jac',[symbol],[ca.jacobian(function(symbol),symbol)])(point))
    assert np.all(np.isfinite(jac))


@pytest.mark.parametrize('fraction',[.07,.3,.77,1.])
def test_solid_profile_accelerated_node_matches_production(fraction):
    s=default_scenario();s.environment.model='standard';s.constraints.max_mission_duration_s=5000
    s.vehicle.stages[0].engine_type='solid';s.vehicle.stages[0].solid_burn_curve=[(0,.2),(.3,1),(.8,.6),(1,.4)]
    sim=Simulation(s);sim._guidance_frame='local_orbital'
    owner=SimpleNamespace(sim=sim,seed_plan=SimpleNamespace(guidance_frame='local_orbital'),interrupted=None)
    owner.scale=lambda i:np.r_[[R_E]*3,[V_SCALE]*3,s.vehicle.stages[i].propellant_mass_kg]
    owner.frame_lookup=FrameLookup(owner);owner.atmosphere_lookup=AtmosphereLookup(owner)
    function=node_function(owner,0,True,False)
    burn=Burn(stage_index=0,duration_s=s.vehicle.stages[0].full_burn_s,angle_fractions=(.2,)*3,clock_angles_rad=(.1,)*3)
    time=1000+fraction*burn.duration_s
    position=sim.frames.matrix(time)@geodetic_to_ecef(32.4,-80.2,400000)
    state=np.r_[position,[710.,1412.,821.],100000.]
    expected=sim.forces(time,state,0,burn,1000,84)[0]
    point=np.r_[state/owner.scale(0),time/1000,.2,.1,1,84/180,fraction]
    actual=np.asarray(function(point)).ravel()[:7]*owner.scale(0)/1000
    np.testing.assert_allclose(actual,expected,rtol=2e-10,atol=2e-8)

