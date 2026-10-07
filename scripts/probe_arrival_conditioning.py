"""Probe allowed arrival-plane offsets in an archived failed surface flight."""
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from atmosphere.models import Scenario,FlightPlan
from atmosphere.dynamics import Simulation
from atmosphere.restoration import restore_continuous

if __name__=='__main__':
    source=ROOT/'data/stage-sizing/tolerance-aware/geo'
    destination=ROOT/'data/stage-sizing/arrival-condition-probe';destination.mkdir(parents=True,exist_ok=True)
    scenario=Scenario.model_validate_json((source/'input.json').read_text())
    original=FlightPlan.model_validate(json.loads((source/'result.json').read_text())['flight_plan'])
    sim=Simulation(scenario);options=[]
    for offset in (-.0005,-.00025,.00025,.0005):
        plan=original.model_copy(deep=True);burn=plan.burns[-1]
        radial=np.array(burn.angle_fractions)*np.cos(burn.clock_angles_rad)
        normal=np.array(burn.angle_fractions)*np.sin(burn.clock_angles_rad)+offset
        burn.angle_fractions=tuple(np.hypot(radial,normal));burn.clock_angles_rad=tuple(np.arctan2(normal,radial))
        result=sim.run(plan,strict=True)
        minimum=min(r['relative_speed_m_s'] for r in result['series'] if r['time_s']>10000 and r['thrust_n']>0)
        item={'offset':offset,'inclination_deg':result['orbit']['inclination_deg'],'minimum_relative_speed_m_s':minimum,'target_comparison':result['target_comparison']}
        print(json.dumps(item),flush=True)
        if result['orbit']['inclination_deg']<.08:options.append((minimum,plan))
    minimum,plan=max(options,key=lambda item:item[0]);last=0
    def progress(value):
        global last
        if time.monotonic()-last>20:print(json.dumps(value),flush=True);last=time.monotonic()
    corrected,report=restore_continuous(sim,plan,240,progress)
    result=sim.run(corrected,strict=True)
    (destination/'result.json').write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
    (destination/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'status':result['status'],'target_comparison':result['target_comparison'],'report':report}),flush=True)
