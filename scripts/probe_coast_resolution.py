"""Independent implicit HS coast accuracy study against production DOP853."""
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.optimize import root

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from atmosphere.models import Scenario,FlightPlan
from atmosphere.dynamics import Simulation
from atmosphere.earth import R_E,V_SCALE

if __name__=='__main__':
    folder=ROOT/'data/stage-sizing/tolerance-aware/geo'
    scenario=Scenario.model_validate_json((folder/'input.json').read_text())
    saved=json.loads((folder/'result.json').read_text())
    plan=FlightPlan.model_validate(saved['flight_plan'])
    sim=Simulation(scenario).for_design(plan.stage_masses)
    burn=plan.burns[-1]
    arrival_start=sum(b.duration_s+b.coast_before_s for b in plan.burns[:-1])+scenario.vehicle.stages[0].separation_delay_s
    row=min(saved['series'],key=lambda r:abs(r['time_s']-arrival_start))
    assert abs(row['time_s']-arrival_start)<1e-5
    initial=np.r_[row['position_gcrs_m'],row['velocity_gcrs_m_s'],row['active_propellant_kg']]
    reference=sim.integrate(initial,arrival_start,burn.coast_before_s,1,strict=True).state
    scale=np.r_[[R_E]*3,[V_SCALE]*3,scenario.vehicle.stages[1].propellant_mass_kg]
    outcomes=[]
    for segments in (12,24,48,96,192):
        began=time.monotonic();grid=(1-np.cos(np.pi*np.linspace(0,1,segments+1)))/2
        state=initial.copy();success=True
        for a,b in zip(grid[:-1],grid[1:]):
            t=arrival_start+a*burn.coast_before_s;dt=(b-a)*burn.coast_before_s
            left=state/scale;f0=sim.forces(t,state,1)[0]/scale
            def defect(right):
                f1=sim.forces(t+dt,right*scale,1)[0]/scale
                middle=(left+right)/2+dt*(f0-f1)/8
                fm=sim.forces(t+dt/2,middle*scale,1)[0]/scale
                return right-left-dt*(f0+4*fm+f1)/6
            # Independent physical guess selects the connected local root.
            guess=sim.integrate(state,t,dt,1,strict=True).state/scale
            fit=root(defect,guess,tol=1e-10)
            success &= np.linalg.norm(defect(fit.x),np.inf)<1e-9
            state=fit.x*scale
        item={'segments':segments,'connected_roots':bool(success),
              'position_error_m':float(np.linalg.norm(state[:3]-reference[:3])),
              'velocity_error_m_s':float(np.linalg.norm(state[3:6]-reference[3:6])),
              'wall_s':time.monotonic()-began}
        outcomes.append(item);print(json.dumps(item),flush=True)
    destination=ROOT/'data/diagnostics/coast-resolution.json'
    destination.write_text(json.dumps({'coast_duration_s':burn.coast_before_s,'comparison':'Independent implicit HS vs production strict DOP853; no optimizer target changes','outcomes':outcomes},indent=2),encoding='utf-8')
