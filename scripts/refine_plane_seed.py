"""Development experiment with production propagation and free node placement."""
import json
import numpy as np
from scipy.optimize import least_squares

from verify_system_matrix import ROOT
from atmosphere.models import Scenario, FlightPlan
from atmosphere.dynamics import Simulation, thrust_direction
from atmosphere.earth import G0, unit, R_E
from atmosphere.mission import MissionTopology, time_to_apogee, seed_from_transfer
from atmosphere.restoration import SegmentReplayCache


if __name__=='__main__':
    root=ROOT/'data/system-validation/plane-seed-probe'
    s=Scenario.model_validate_json((root/'input.json').read_text())
    p=FlightPlan.model_validate_json((root/'flight-plan.json').read_text())
    source=Simulation(s);base=source.for_design(p.stage_masses)
    cache=SegmentReplayCache();sim=cache.simulation(base,base.scenario,base.check)
    prefix=p.model_copy(update={'stage_masses':None,'burns':p.burns[:-1]})
    initial=prefix.burns[-1];target=R_E+s.target.apogee_altitude_m
    def flight(x):
        candidate=prefix.model_copy(deep=True)
        candidate.burns[-1].coast_before_s=initial.coast_before_s+x[0]*50
        candidate.burns[-1].duration_s=initial.duration_s+x[1]
        r=sim.run(candidate,strict=False,record=False)
        y=sim._last_state.copy();t=r['summary']['arrival_time_s']
        coast=time_to_apogee(y[:3],y[3:6])
        segment=sim.integrate(y,t,coast,1,azimuth=p.launch_azimuth_deg)
        return candidate,segment,y,t
    def residual(x):
        _,seg,_,_=flight(x)
        r=seg.state[:3]
        values=[(np.linalg.norm(r)-target)/1000,r[2]/1000]
        print(json.dumps({'x':x.tolist(),'errors_km':values}),flush=True)
        return values
    fit=least_squares(residual,[0.,0.],bounds=([-4,-5],[4,5]),diff_step=2e-4,x_scale='jac',max_nfev=20,gtol=1e-5,ftol=1e-7,xtol=1e-7)
    candidate,seg,y,t=flight(fit.x)
    direct=prefix.model_copy(update={'burns':prefix.burns[:2]})
    arrival=seed_from_transfer(sim,MissionTopology((1,3)),direct,y,t).burns[-1]
    v=seg.state[3:6];r=seg.state[:3];tm=seg.time
    def basis(vv,rr,tt):
        lat,lon,_=sim.frames.geographic(tt,rr);lat,lon=np.radians([lat,lon]);rot=sim.frames.matrix(tt)
        up=rot@np.array([np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)])
        north=rot@np.array([-np.sin(lat)*np.cos(lon),-np.sin(lat)*np.sin(lon),np.cos(lat)])
        east=rot@np.array([-np.sin(lon),np.cos(lon),0])
        heading=np.cos(np.radians(p.launch_azimuth_deg))*north+np.sin(np.radians(p.launch_azimuth_deg))*east
        radial,_=thrust_direction(vv,up,heading,np.pi/2,0,p.guidance_frame)
        lateral,_=thrust_direction(vv,up,heading,np.pi/2,np.pi/2,p.guidance_frame)
        return up,heading,radial,lateral
    up,heading,_,_=basis(v,r,tm)
    direction,_=thrust_direction(v,up,heading,arrival.angle_fractions[0]*np.pi,arrival.clock_angles_rad[0],p.guidance_frame)
    stage=sim.scenario.vehicle.stages[1];mass=sim.base_mass(1)+seg.state[6];mdot=stage.mass_flow_kg_s*stage.throttle_max
    angles=[];clocks=[]
    for f in np.linspace(0,1,len(arrival.throttles)):
        dt=f*arrival.duration_s;vf=v+direction*(stage.isp_vacuum_s*G0*np.log(mass/(mass-mdot*dt)));rf=r+.5*(v+vf)*dt
        _,_,radial,lateral=basis(vf,rf,tm+dt)
        angles.append(float(np.arctan2(np.linalg.norm(np.cross(unit(vf),direction)),np.dot(unit(vf),direction))/np.pi))
        clocks.append(float(np.arctan2(np.dot(direction,lateral),np.dot(direction,radial))))
    arrival.angle_fractions=tuple(angles);arrival.clock_angles_rad=tuple(np.unwrap(clocks))
    candidate=FlightPlan(stage_masses=p.stage_masses,launch_azimuth_deg=p.launch_azimuth_deg,
                         guidance_frame=p.guidance_frame,burns=[*candidate.burns,arrival])
    replay=source.run(candidate,strict=True)
    (root/'refined-flight-plan.json').write_text(candidate.model_dump_json(indent=2))
    (root/'refined-result.json').write_text(json.dumps(replay,allow_nan=False))
    print(json.dumps({'target':replay['target_comparison'],'status':replay['status'],'cache_hits':cache.hits}),flush=True)
