"""Isolated diagnostic of a tolerance-aware plane residual at GEO arrival."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from verify_stage_sizing import benchmark
from atmosphere.models import FlightPlan
from atmosphere.dynamics import Simulation

if __name__=='__main__':
    source=(ROOT/'backend/atmosphere/restoration.py').read_text(encoding='utf-8')
    old='residual.extend(normal[:2]/np.radians(s.target.angle_tolerance_deg))'
    new='plane=normal[:2]/np.radians(s.target.angle_tolerance_deg)\n            residual.extend(plane*max(0,1-.8/max(1e-12,np.linalg.norm(plane))))'
    if old not in source:
        raise SystemExit('The angular-margin change is now in production. Use verify_stage_sizing.py for current validation; archived diagnostic results are retained.')
    destination=ROOT/'data/stage-sizing/plane-margin-probe';destination.mkdir(parents=True,exist_ok=True)
    module_path=destination/'restoration_diagnostic.py';module_path.write_text(source.replace(old,new),encoding='utf-8')
    spec=importlib.util.spec_from_file_location('atmosphere.restoration_diagnostic',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    s=benchmark('geo',1800);sim=Simulation(s)
    previous=json.loads((ROOT/'data/stage-sizing/transfer-arrival-probe/corrected.json').read_text(encoding='utf-8'))
    original=FlightPlan.model_validate(previous['flight_plan'])
    options=[]
    for sign in (-1,1):
        plan=original.model_copy(deep=True);burn=plan.burns[-1]
        radial=np.array(burn.angle_fractions)*np.cos(burn.clock_angles_rad)
        normal=np.array(burn.angle_fractions)*np.sin(burn.clock_angles_rad)+sign*.0005
        burn.angle_fractions=tuple(np.hypot(radial,normal));burn.clock_angles_rad=tuple(np.arctan2(normal,radial))
        result=sim.run(plan,strict=True)
        minimum=min(r['relative_speed_m_s'] for r in result['series'] if r['time_s']>10000 and r['thrust_n']>0)
        if result['orbit']['inclination_deg']<.08:options.append((minimum,plan,result))
    _,plan,before=max(options,key=lambda item:item[0])
    print(json.dumps({'before':before['target_comparison'],'minimum_relative_speed':max(o[0] for o in options)}),flush=True)
    corrected,report=module.restore_continuous(sim,plan,300,lambda v:print(json.dumps(v),flush=True) if v['iteration']%10==0 else None)
    after=sim.run(corrected,strict=True)
    (destination/'corrected.json').write_text(json.dumps(after,allow_nan=False),encoding='utf-8')
    (destination/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'status':after['status'],'comparison':after['target_comparison'],'report':report}),flush=True)
