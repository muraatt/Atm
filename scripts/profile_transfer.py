"""Profile production coast propagation without changing the physical model."""
import cProfile
import io
import json
from pathlib import Path
import pstats
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from atmosphere.models import Scenario,FlightPlan
from atmosphere.dynamics import Simulation
from atmosphere.mission import time_to_apogee

if __name__=='__main__':
    origin=ROOT/'data/stage-sizing/joint-final/gto'
    s=Scenario.model_validate_json((origin/'input.json').read_text(encoding='utf-8'))
    plan=FlightPlan.model_validate(json.loads((origin/'result.json').read_text(encoding='utf-8'))['flight_plan'])
    sim=Simulation(s).for_design(plan.stage_masses)
    warm=sim.run(plan.model_copy(update={'stage_masses':None}),strict=False,record=False)
    state=sim._last_state;duration=time_to_apogee(state[:3],state[3:6]);start=warm['summary']['arrival_time_s']
    profile=cProfile.Profile();started=time.monotonic();profile.enable()
    segment=sim.integrate(state,start,duration,1,strict=True,record=True)
    profile.disable();out=io.StringIO();pstats.Stats(profile,stream=out).sort_stats('cumulative').print_stats(22)
    destination=ROOT/'data/diagnostics/transfer-profile.txt'
    destination.write_text(f'Elapsed {time.monotonic()-started:.3f} s; duration {duration:.3f} s; status {segment.status}\n'+out.getvalue(),encoding='utf-8')
    print(destination.read_text(encoding='utf-8'),flush=True)
