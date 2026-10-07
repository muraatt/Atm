"""Audit saved flights against current production equations, without a new search."""
import argparse
import json
import time

from verify_system_matrix import ROOT, CASES, sources, audit
from atmosphere.dynamics import Simulation
from atmosphere.models import Scenario, FlightPlan


def replay(name, source_label, label):
    origin=ROOT/'data/system-validation'/source_label/name
    dest=ROOT/'data/system-validation'/label/name
    dest.mkdir(parents=True,exist_ok=True)
    before=sources();started=time.monotonic()
    scenario=Scenario.model_validate_json((origin/'input.json').read_text())
    result=json.loads((origin/'result.json').read_text())
    plan=FlightPlan.model_validate_json((origin/'flight-plan.json').read_text())
    flight=Simulation(scenario).run(plan,strict=True)
    outcome=audit(scenario,result,flight,before,sources())
    outcome.update(case=name,wall_s=time.monotonic()-started,
                   validation_mode='Independent strict replay of a saved optimizer candidate',
                   optimizer_source_label=source_label,
                   optimizer_source_hashes=json.loads((origin/'implementation.json').read_text()),
                   replay_source_hashes=before)
    (dest/'input.json').write_text(scenario.model_dump_json(indent=2),encoding='utf-8')
    (dest/'flight-plan.json').write_text(plan.model_dump_json(indent=2),encoding='utf-8')
    (dest/'independent-replay.json').write_text(json.dumps(flight,allow_nan=False),encoding='utf-8')
    (dest/'audit.json').write_text(json.dumps(outcome,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'case':name,'passed':outcome['passed'],'status':outcome['status']}),flush=True)
    return outcome


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--case',nargs='+',choices=list(CASES),required=True)
    p.add_argument('--source-label',required=True);p.add_argument('--label',required=True)
    args=p.parse_args()
    outcomes=[replay(name,args.source_label,args.label) for name in args.case]
    raise SystemExit(0 if all(r['passed'] for r in outcomes) else 1)
