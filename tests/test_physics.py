from datetime import datetime, timezone

import numpy as np
import pytest
from pydantic import ValidationError
from scipy.integrate import solve_ivp

from atmosphere.dynamics import Simulation, thrust_direction, validate_plan
from atmosphere.earth import EarthFrames, G0, MU, R_E, ecef_to_geodetic, geodetic_to_ecef, gravity, orbital_elements, orbit_curve, targeted_elements
from atmosphere.environment import MSISAtmosphere, StandardAtmosphere
from atmosphere.models import Burn, EnvironmentConfig, FlightPlan, LaunchSite, Stage, TargetOrbit, default_scenario
from atmosphere.optimization import MultipleShooting, templates
from atmosphere.terrain import tile_name


@pytest.mark.parametrize('lat,lon,h',[(0,0,0),(28.5,-80.6,-30),(31.5,35.5,-400),(90,180,500),(-90,-180,0),(72,179.999,3000)])
def test_geodetic_round_trip(lat,lon,h):
    actual=ecef_to_geodetic(geodetic_to_ecef(lat,lon,h))
    assert actual[0]==pytest.approx(lat,abs=1e-9)
    assert actual[2]==pytest.approx(h,abs=1e-5)
    if abs(lat)<90:
        assert actual[1]==pytest.approx(lon,abs=1e-9)


def test_initial_earth_rotation_and_astropy_frames():
    launch=LaunchSite(epoch=datetime(2026,10,4,tzinfo=timezone.utc),latitude_deg=0,longitude_deg=0)
    frames=EarthFrames(launch.epoch,10000)
    state=frames.initial_state(launch)
    assert np.linalg.norm(state[3:6])==pytest.approx(465.101,abs=.02)
    assert frames.geographic(0,state[:3])==pytest.approx((0,0,0),abs=1e-6)
    fixed=frames.matrix(9000).T@(frames.matrix(9000)@geodetic_to_ecef(60,130,500))
    assert np.allclose(fixed,geodetic_to_ecef(60,130,500),atol=1e-7)


def test_unpowered_two_body_conservation():
    r0=R_E+400000
    state=np.array([r0,0,0,0,np.sqrt(MU/r0),0])
    period=2*np.pi*np.sqrt(r0**3/MU)
    result=solve_ivp(lambda t,y:np.r_[y[3:6],gravity(y[:3],include_j2=False)],(0,period*3),state,method='DOP853',rtol=1e-11,atol=1e-7)
    r,v=result.y[:3],result.y[3:6]
    energy=np.sum(v*v,axis=0)/2-MU/np.linalg.norm(r,axis=0)
    h=np.cross(r.T,v.T)
    assert np.max(abs((energy-energy[0])/energy[0]))<1e-9
    assert np.max(np.linalg.norm(h-h[0],axis=1)/np.linalg.norm(h[0]))<1e-9


def test_vacuum_rocket_equation():
    stage=Stage(dry_mass_kg=1000,propellant_mass_kg=5000,vacuum_thrust_n=100000,isp_vacuum_s=350,isp_sea_level_s=350)
    initial_mass=6000
    end=stage.full_burn_s*.8
    sol=solve_ivp(lambda t,y:[stage.vacuum_thrust_n/y[1],-stage.mass_flow_kg_s],(0,end),[0,initial_mass],method='DOP853',rtol=1e-11,atol=1e-8)
    expected=G0*350*np.log(initial_mass/sol.y[1,-1])
    assert sol.y[0,-1]==pytest.approx(expected,rel=1e-10)


def test_standard_reference_atmosphere():
    provider=StandardAtmosphere()
    sea=provider.sample(0,0,0,0)
    assert sea.temperature_k==pytest.approx(288.15)
    assert sea.pressure_pa==pytest.approx(101325)
    assert sea.density_kg_m3==pytest.approx(1.225,rel=2e-5)
    # 11 km geopotential expressed as geometric height.
    at11=provider.sample(0,0,11000*6356766/(6356766-11000),0)
    assert at11.temperature_k==pytest.approx(216.65,abs=.001)
    assert at11.pressure_pa==pytest.approx(22632.1,rel=1e-4)
    vacuum=provider.sample(0,0,1000001,0)
    assert vacuum.density_kg_m3==0 and vacuum.temperature_k is None


def test_msis_interpolation_matches_reference_at_grid_nodes():
    import pymsis
    config=EnvironmentConfig(solar_mode='nominal')
    launch=LaunchSite(latitude_deg=30,longitude_deg=20,epoch=datetime(2026,10,4,tzinfo=timezone.utc))
    provider=MSISAtmosphere(config,launch)
    raw=pymsis.calculate([np.datetime64('2026-10-04')],[20],[30],[100],f107s=[150],f107as=[150],aps=[[4]*7],version=2.1).reshape(11)
    actual=provider.sample(30,20,100000,0)
    assert actual.density_kg_m3==pytest.approx(float(raw[0]),rel=1e-6)
    assert actual.temperature_k==pytest.approx(float(raw[10]),rel=1e-6)
    assert actual.pressure_pa>0 and actual.sound_speed_m_s>0
    assert provider.sample(0,0,1000001,0).temperature_k is None


@pytest.mark.parametrize('speed',[0,.5,1,50,8000])
def test_thrust_cone_and_zero_speed(speed):
    up=np.array([1.,0,0]);heading=np.array([0.,1,0])
    relative=np.array([speed,0,0])
    direction,angle=thrust_direction(relative,up,heading,np.radians(15),.7)
    assert np.linalg.norm(direction)==pytest.approx(1)
    if speed<1:
        assert np.allclose(direction,up) and angle==0
    else:
        assert np.degrees(np.arccos(np.dot(relative/speed,direction)))<=15.000001


def test_solid_curve_preserves_propellant_impulse():
    stage=Stage(engine_type='solid',solid_burn_curve=[(0,0),(.1,1),(.8,.8),(1,0)])
    assert stage.full_burn_s*stage.mass_flow_kg_s*stage.curve_mean==pytest.approx(stage.propellant_mass_kg)
    with pytest.raises(ValidationError):
        Stage(engine_type='solid',ignition_limit=2)


def test_validation_rejects_inconsistent_mass_and_target():
    scenario=default_scenario().model_dump(mode='json')
    scenario['vehicle']['expected_initial_mass_kg']=1
    with pytest.raises(ValidationError):
        type(default_scenario()).model_validate(scenario)
    with pytest.raises(ValidationError):
        TargetOrbit(perigee_altitude_m=300000,apogee_altitude_m=200000)
    with pytest.raises(ValidationError):
        TargetOrbit(inclination_deg=0,raan_deg=45)
    with pytest.raises(ValidationError):
        TargetOrbit(apogee_altitude_m=200000,argument_of_periapsis_deg=30)


@pytest.mark.parametrize('inclination',[0,28.5,98,180])
def test_orbit_elements_and_singular_phase(inclination):
    target=TargetOrbit(inclination_deg=inclination)
    curve=orbit_curve({'bound':True,'semi_major_axis_m':R_E+200000,'eccentricity':0.,'inclination_deg':inclination,'raan_deg':30,'argument_of_periapsis_deg':0})
    r=np.array(curve[0]);rnext=np.array(curve[1])
    tangent=rnext-r; tangent-=np.dot(tangent,r/np.linalg.norm(r))*r/np.linalg.norm(r);tangent/=np.linalg.norm(tangent)
    v=tangent*np.sqrt(MU/np.linalg.norm(r))
    orbit=targeted_elements(r,v,target)
    assert orbit['inclination_deg']==pytest.approx(inclination,abs=1e-6)
    assert orbit['perigee_altitude_m']==pytest.approx(200000,abs=.001)
    assert orbit['apogee_altitude_m']==pytest.approx(200000,abs=.001)
    assert orbit['phase_definition']==('True longitude' if inclination in (0,180) else 'Argument of latitude')


def test_etopo_index_direction_dateline_and_poles():
    name,row,col=tile_name(28.5,-80.6,'surface')
    assert name=='ETOPO_2022_v1_15s_N30W090_surface.nc'
    assert row==3240 and col==2256
    for lat,lon in [(90,180),(-90,-180),(0,0),(-15,179.9)]:
        name,row,col=tile_name(lat,lon,'geoid')
        assert 0<=row<3600 and 0<=col<3600


def test_stage_accounting_and_restarts():
    scenario=default_scenario();scenario.environment.model='standard'
    sim=Simulation(scenario)
    plan=FlightPlan(burns=[Burn(stage_index=0,duration_s=120,angle_fractions=(0,0,0)),
                          Burn(stage_index=1,duration_s=20,angle_fractions=(0,0,0)),
                          Burn(stage_index=1,duration_s=15,coast_before_s=10,angle_fractions=(0,0,0))])
    result=sim.run(plan,strict=True)
    assert sum(r['initial_kg'] for r in result['propellant_ledger'])==pytest.approx(sum(r['burned_kg']+r['remaining_kg']+r['discarded_kg'] for r in result['propellant_ledger']),abs=1e-5)
    assert all(r['active_propellant_kg']>=0 for r in result['series'])
    assert len([e for e in result['events'] if e['event']=='Ignition'])==3
    assert len([e for e in result['events'] if e['event']=='Stage Separation'])==1
    assert result['constraint_checks']['thrust_angle']
    assert result['metadata']['verified_with_strict_integration']
    with pytest.raises(ValueError):
        validate_plan(scenario,FlightPlan(burns=plan.burns+[plan.burns[-1]]))


def test_no_liftoff_and_impact_are_separate():
    scenario=default_scenario();scenario.environment.model='standard'
    scenario.vehicle.stages[0].vacuum_thrust_n=1000
    plan=FlightPlan(burns=[Burn(stage_index=i,duration_s=10) for i in range(2)])
    result=Simulation(scenario).run(plan)
    assert result['status']=='No Liftoff'
    assert result['summary']['propellant_burned_kg']==0
    scenario.vehicle.stages[0].vacuum_thrust_n=4200000
    scenario.constraints.vertical_ascent_s=0
    result=Simulation(scenario).run(FlightPlan(burns=[Burn(stage_index=i,duration_s=150,angle_fractions=(.8,.8,.8)) for i in range(2)]))
    assert result['status']=='Impact'


def test_multiple_shooting_seed_is_continuous_including_separation_coast():
    scenario=default_scenario();scenario.environment.model='standard'
    problem=MultipleShooting(Simulation(scenario),templates(scenario)[0],.25)
    value=problem.evaluate(problem.seed)
    assert np.max(abs(value['eq']))<1e-7


def test_hohmann_geo_coast_and_circularization_reference():
    rp,ra=R_E+200000,R_E+35786000
    a=(rp+ra)/2
    speed=np.sqrt(MU*(2/rp-1/a))
    coast=np.pi*np.sqrt(a**3/MU)
    state=np.array([rp,0,0,0,speed,0])
    sol=solve_ivp(lambda t,y:np.r_[y[3:6],gravity(y[:3],include_j2=False)],(0,coast),state,method='DOP853',rtol=1e-11,atol=1e-6)
    r,v=sol.y[:3,-1],sol.y[3:6,-1]
    assert np.linalg.norm(r)==pytest.approx(ra,rel=1e-9)
    h=np.cross(r,v);transverse=np.cross(h/np.linalg.norm(h),r/np.linalg.norm(r))
    orbit=orbital_elements(r,transverse*np.sqrt(MU/np.linalg.norm(r)))
    assert orbit['perigee_altitude_m']==pytest.approx(35786000,abs=.1)
    assert orbit['apogee_altitude_m']==pytest.approx(35786000,abs=.1)
