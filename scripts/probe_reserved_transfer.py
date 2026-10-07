"""Diagnostic mass-balanced ascent/arrival seed, without editing user inputs."""
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from verify_stage_sizing import benchmark
from atmosphere.budget import mission_budget
from atmosphere.sizing import seed_masses,mass_bounds
from atmosphere.dynamics import Simulation,SolverDeadline
from atmosphere.models import TargetOrbit
from atmosphere.optimization import warm_start
from atmosphere.mission import seed_from_transfer,MissionTopology,resample_burn
from atmosphere.restoration import restore_continuous

if __name__=='__main__':
    s=benchmark('geo',1800);masses=seed_masses(s)
    reference=mission_budget(s)['tiers'][-1]
    for value,bound,row in zip(masses,mass_bounds(s),reference['stages']):
        value.propellant_mass_kg=float(np.clip(max(value.propellant_mass_kg,row['required_fuel_kg']),bound.propellant_min_kg,bound.propellant_max_kg))
        value.dry_mass_kg=max(value.dry_mass_kg,bound.hardware_mass_kg+bound.tank_structure_ratio*value.propellant_mass_kg)
    sim=Simulation(s);design=sim.for_design(masses)
    original=design.scenario.target
    design.scenario.target=TargetOrbit(perigee_altitude_m=200000,apogee_altitude_m=35786000,inclination_deg=0,
                                      altitude_tolerance_m=10000,angle_tolerance_deg=.5)
    start=time.monotonic();last=0
    def progress(v):
        global last
        if time.monotonic()-last>20:
            print(json.dumps({'elapsed_s':time.monotonic()-start,**v}),flush=True);last=time.monotonic()
    def check():
        if time.monotonic()-start>240:raise SolverDeadline('seed_probe')
    design.check=check
    warm,fit=warm_start(design,progress,'local_orbital',stop_on_target=True)
    design.check=lambda:None;design.scenario.target=original
    warm=warm.model_copy(update={'stage_masses':masses,'burns':[resample_burn(b,9) for b in warm.burns]})
    result=design.run(warm.model_copy(update={'stage_masses':None}),strict=True)
    seed=seed_from_transfer(design,MissionTopology((1,2)),warm,design._last_state,result['summary']['arrival_time_s'])
    before=sim.run(seed,strict=True)
    destination=ROOT/'data/stage-sizing/reserved-transfer-probe';destination.mkdir(parents=True,exist_ok=True)
    (destination/'seed.json').write_text(json.dumps(before,allow_nan=False),encoding='utf-8')
    corrected,report=restore_continuous(sim,seed,600,progress)
    after=sim.run(corrected,strict=True)
    (destination/'corrected.json').write_text(json.dumps(after,allow_nan=False),encoding='utf-8')
    (destination/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'status':after['status'],'comparison':after['target_comparison'],'report':report}),flush=True)
