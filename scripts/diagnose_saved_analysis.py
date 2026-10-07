"""Isolated continuation/cold-search diagnostics; never overwrite a user job."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
from atmosphere.models import Scenario, FlightPlan
from atmosphere.dynamics import Simulation
from atmosphere.optimization import optimize
from atmosphere.restoration import restore_continuous


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--job', required=True)
    parser.add_argument('--mode', choices=('restore', 'search'), required=True)
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--iterations', type=int, default=250)
    parser.add_argument('--refinements', type=int, default=2)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    if len(args.job) != 32 or any(c not in '0123456789abcdef' for c in args.job):
        parser.error('Job must be a saved analysis identifier.')
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-' for c in args.label):
        parser.error('Label must contain lowercase letters, digits or hyphens.')
    origin = ROOT / 'data' / 'analyses' / args.job
    destination = ROOT / 'data' / 'diagnostics' / args.job / args.label
    destination.mkdir(parents=True, exist_ok=False)
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in origin.iterdir() if p.name in ('result.json', 'scenario.json')}
    original = Scenario.model_validate_json((origin / 'scenario.json').read_text())
    scenario = original.model_copy(deep=True)
    baseline = json.loads((origin / 'result.json').read_text())
    start = time.monotonic()
    last = -100.

    def progress(value):
        global last
        if time.monotonic() - last >= 20:
            print(json.dumps(value, allow_nan=False), flush=True)
            last = time.monotonic()

    if args.mode == 'restore':
        plan, info = restore_continuous(Simulation(scenario),
            FlightPlan.model_validate(baseline['flight_plan']), args.seconds, progress)
        result = Simulation(scenario).run(plan, strict=True, record=True)
    else:
        scenario.constraints.solver_budget_s = args.seconds
        scenario.constraints.max_iterations = args.iterations
        scenario.constraints.refinement_levels = args.refinements
        scenario = Scenario.model_validate(scenario.model_dump())
        result = optimize(scenario, progress=progress)
        info = result['optimizer']
        replay = Simulation(scenario).run(FlightPlan.model_validate(result['flight_plan']),
                                          strict=True, record=True)
        save(destination / 'independent-replay.json', replay)
        assert replay['status'] == result['status'], 'Independent replay status changed.'
    save(destination / 'input.json', scenario.model_dump(mode='json'))
    save(destination / 'result.json', result)
    save(destination / 'solver.json', info)
    unchanged = all(hashlib.sha256((origin / name).read_bytes()).hexdigest() == digest
                    for name, digest in hashes.items())
    assert unchanged, 'The original saved analysis changed.'
    assert scenario.target == original.target, 'Target or acceptance changed.'
    report = {'mode': args.mode, 'source_job': args.job, 'source_hashes': hashes,
              'original_job_unchanged': unchanged, 'target_and_tolerances_unchanged': True,
              'elapsed_s': time.monotonic() - start, 'status': result['status'],
              'target_comparison': result['target_comparison'],
              'constraint_checks': result['constraint_checks'], 'summary': result['summary']}
    save(destination / 'report.json', report)
    print(json.dumps(report, allow_nan=False), flush=True)
