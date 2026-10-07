"""Development experiment: initialize a node-aligned parking/transfer mission."""
import json
import time
import numpy as np
from scipy.optimize import brentq

from verify_system_matrix import ROOT, scenario_for
from atmosphere.optimization import warm_start
from atmosphere.models import TargetOrbit, Burn, FlightPlan
from atmosphere.sizing import seed_masses
from atmosphere.dynamics import Simulation, SolverDeadline
from atmosphere.mission import MissionTopology, seed_from_ascent, seed_from_transfer
from atmosphere.earth import MU, unit
from atmosphere.screening import departure_options


if __name__=='__main__':
    s=scenario_for('geo-plane-change',900);start=time.monotonic()
    def check():
        if time.monotonic()-start>150:raise SolverDeadline()
    source=Simulation(s,check=check);masses=seed_masses(s)
    sim=source.for_design(masses)
    target=sim.scenario.target
    insertion=departure_options(s.launch.latitude_deg,target,s.launch.azimuth_sectors_deg)[2]
    sim.scenario.target=TargetOrbit(perigee_altitude_m=200000,apogee_altitude_m=200000,inclination_deg=insertion,
                                  altitude_tolerance_m=5000,angle_tolerance_deg=.3)
    def progress(value):print(json.dumps(value),flush=True)
    ascent,fit=warm_start(sim,progress,'local_orbital',stop_on_target=True)
    sim.scenario.target=target
    sim.check=lambda:None;source.check=lambda:None
    if ascent is None:raise RuntimeError('No parking ascent candidate')
    result=sim.run(ascent,strict=False,record=False)
    state=sim._last_state.copy();t=result['summary']['arrival_time_s']
    if not result['orbit']['bound'] or result['orbit']['perigee_altitude_m']<100000:raise RuntimeError('Parking seed is not safely bound')
    period=2*np.pi*np.sqrt(result['orbit']['semi_major_axis_m']**3/MU)
    normal=np.array([0.,0.,1.])
    def at(dt):return sim.integrate(state,t,float(dt),1,azimuth=ascent.launch_azimuth_deg)
    times=np.linspace(1,period,25);previous=0.;previous_value=float(np.dot(normal,state[:3]));roots=[]
    for dt in times:
        value=float(np.dot(normal,at(dt).state[:3]))
        if value*previous_value<0:
            roots.append(brentq(lambda d:float(np.dot(normal,at(d).state[:3])),previous,dt,xtol=.001))
        previous=dt;previous_value=value
    plan=seed_from_ascent(sim.scenario,MissionTopology((1,3)),ascent,parking=True)
    injection=plan.burns[-2]
    node=next(root for root in roots if root>injection.duration_s/2)
    injection.coast_before_s=node-injection.duration_s/2
    injection.steering_reference='inertial_velocity'
    prefix=plan.model_copy(update={'burns':plan.burns[:-1]})
    transfer=sim.run(prefix,strict=False,record=False)
    arrival=seed_from_transfer(sim,MissionTopology((1,3)),ascent,sim._last_state.copy(),transfer['summary']['arrival_time_s']).burns[-1]
    candidate=FlightPlan(stage_masses=masses,launch_azimuth_deg=ascent.launch_azimuth_deg,
                         guidance_frame=ascent.guidance_frame,burns=[*prefix.burns,arrival])
    sim.check=lambda:None
    out=ROOT/'data/system-validation/plane-seed-probe';out.mkdir(parents=True,exist_ok=True)
    replay=source.run(candidate,strict=True)
    (out/'input.json').write_text(s.model_dump_json(indent=2))
    (out/'flight-plan.json').write_text(candidate.model_dump_json(indent=2))
    (out/'result.json').write_text(json.dumps(replay,allow_nan=False))
    print(json.dumps({'parking':result['orbit'],'node_wait_s':node,'target_comparison':replay['target_comparison'],'phases':[(b.stage_index,b.duration_s,b.coast_before_s) for b in candidate.burns]}))
