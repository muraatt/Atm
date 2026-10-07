"""Fresh strict replay of archived surface designs against current physics."""
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from atmosphere.models import Scenario,FlightPlan
from atmosphere.dynamics import Simulation

if __name__=='__main__':
    destination=ROOT/'data/stage-sizing/current-physics-replay';destination.mkdir(parents=True,exist_ok=True)
    sources={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted((ROOT/'backend/atmosphere').glob('*.py'))}
    audits=[]
    for case,label in [('leo','joint-restored'),('meo','joint-restored'),('gto','joint-final')]:
        origin=ROOT/'data/stage-sizing'/label/case
        scenario=Scenario.model_validate_json((origin/'input.json').read_text(encoding='utf-8'))
        previous=json.loads((origin/'result.json').read_text(encoding='utf-8'))
        plan=FlightPlan.model_validate(previous['flight_plan'])
        result=Simulation(scenario).run(plan,strict=True)
        checks={'surface_start':abs(result['series'][0]['altitude_m'])<.1,
                'target':result['status']=='Target Achieved',
                'tolerances':all(r['within_tolerance'] for r in result['target_comparison']),
                'constraints':all(result['constraint_checks'].values()),
                'fuel_closure':all(abs(r['initial_kg']-r['burned_kg']-r['remaining_kg']-r['discarded_kg'])<.01 for r in result['propellant_ledger'])}
        (destination/f'{case}-replay.json').write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
        audit={'case':case,'passed':all(checks.values()),'checks':checks,'orbit':result['target_comparison']}
        audits.append(audit);print(json.dumps(audit),flush=True)
    (destination/'audit.json').write_text(json.dumps({'sources':sources,'audits':audits,
        'scope':'Current-physics replay of earlier optimized designs; not a rerun of the final optimization pipeline.'},indent=2),encoding='utf-8')
    sys.exit(0 if all(a['passed'] for a in audits) else 1)
