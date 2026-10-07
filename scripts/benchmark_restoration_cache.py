"""Compare exact repeated shooting trials with and without segment reuse."""
import argparse
import json
import time
from pathlib import Path

from verify_system_matrix import ROOT, sources
from atmosphere.dynamics import Simulation
from atmosphere.models import Scenario, FlightPlan
from atmosphere.restoration import SegmentReplayCache


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',default='meo-fast')
    p.add_argument('--source-label',default='varied-v3');args=p.parse_args()
    origin=ROOT/'data/system-validation'/args.source_label/args.case
    s=Scenario.model_validate_json((origin/'input.json').read_text())
    plan=FlightPlan.model_validate_json((origin/'flight-plan.json').read_text())
    sim=Simulation(s).for_design(plan.stage_masses)
    plan=plan.model_copy(update={'stage_masses':None})
    candidates=[]
    for offset in (0,1e-5,-1e-5,2e-5):
        candidate=plan.model_copy(deep=True)
        candidate.burns[-1].duration_s*=1+offset;candidates.append(candidate)
    cache=SegmentReplayCache();timings={};outputs={};hashes=sources()
    for mode in ('uncached','cached'):
        runner=sim if mode=='uncached' else cache.simulation(sim,sim.scenario,sim.check)
        started=time.monotonic()
        outputs[mode]=[runner.run(candidate,strict=True) for candidate in candidates]
        timings[mode]=time.monotonic()-started
    identical=outputs['uncached']==outputs['cached']
    report={'case':args.case,'trials':len(candidates),'strict_results_identical':identical,
            'timings_s':timings,'speedup':timings['uncached']/timings['cached'],
            'cache_hits':cache.hits,'cache_misses':cache.misses,'source_unchanged':hashes==sources(),
            'scope':'Identical last-burn duration perturbations of one saved design; not total optimizer speedup.'}
    destination=ROOT/'data/system-validation/cache-benchmark';destination.mkdir(parents=True,exist_ok=True)
    (destination/f'{args.case}.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)
    raise SystemExit(0 if identical and report['source_unchanged'] else 1)
