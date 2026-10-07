"""Diagnose a finite arrival seed using an independently replayed ascent."""
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from verify_stage_sizing import benchmark
from atmosphere.models import FlightPlan
from atmosphere.dynamics import Simulation
from atmosphere.mission import MissionTopology,seed_from_transfer
from atmosphere.restoration import restore_continuous

if __name__=='__main__':
    s=benchmark('geo',1800)
    saved=json.loads((ROOT/'data/stage-sizing/joint-final/gto/result.json').read_text(encoding='utf-8'))
    ascent=FlightPlan.model_validate(saved['flight_plan'])
    sim=Simulation(s);design=sim.for_design(ascent.stage_masses)
    warm=design.run(ascent.model_copy(update={'stage_masses':None}),strict=True)
    seed=seed_from_transfer(design,MissionTopology((1,2)),ascent,design._last_state,warm['summary']['arrival_time_s'])
    before=sim.run(seed,strict=True)
    started=time.monotonic();last=0
    def progress(value):
        global last
        if time.monotonic()-last>20:
            print(json.dumps(value),flush=True);last=time.monotonic()
    corrected,report=restore_continuous(sim,seed,600,progress)
    after=sim.run(corrected,strict=True)
    dest=ROOT/'data/stage-sizing/transfer-arrival-probe';dest.mkdir(exist_ok=True,parents=True)
    for name,value in [('seed',before),('corrected',after),('report',report)]:
        (dest/f'{name}.json').write_text(json.dumps(value,allow_nan=False,indent=2),encoding='utf-8')
    print(json.dumps({'elapsed_s':time.monotonic()-started,'before':before['target_comparison'],
                      'after':after['target_comparison'],'status':after['status'],'report':report}),flush=True)
