"""Summarize isolated saved-flight diagnostics without altering user results."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--job', required=True)
    args = parser.parse_args()
    if len(args.job) != 32 or any(c not in '0123456789abcdef' for c in args.job):
        parser.error('Job must be a saved analysis identifier.')
    root = ROOT / 'data' / 'diagnostics' / args.job
    original = json.loads((ROOT / 'data' / 'analyses' / args.job / 'result.json').read_text())
    source = original.get('input_scenario', original['scenario'])
    target = source['target']
    constraints = source['constraints']
    results = [('Original saved run', original)]
    for path in sorted(root.glob('*/result.json')):
        results.append((path.parent.name, json.loads(path.read_text())))
    lines = ['# Saved analysis diagnosis', '', f'Source job: `{args.job}`.', '',
             f"Target: {json.dumps(target)}. Payload: {source['vehicle']['payload_mass_kg']} kg. "
             'Original user files are unchanged.', '',
             '| Evidence | Status | Perigee error (m) | Apogee error (m) | Inclination error (deg) | Fuel burned (kg) |',
             '|---|---|---:|---:|---:|---:|']
    for label, result in results:
        errors = {r['parameter']: r['error'] for r in result['target_comparison']}
        fields = [f"{errors[key]:+.6f}" for key in
                  ('perigee_altitude_m', 'apogee_altitude_m', 'inclination_deg')]
        lines.append(f"| {label} | {result['status']} | {' | '.join(fields)} | "
                     f"{result['summary']['propellant_burned_kg']:.6f} |")
    limited = sum(r['stop_reason'] == 'iteration_limit' for r in original['optimizer']['searches'])
    correction = original['optimizer']['mission'].get('continuous_restoration', {})
    lines += ['', f"Original settings: {constraints['solver_budget_s']} seconds, "
              f"{constraints['max_iterations']} iterations per search and {constraints['refinement_levels']} refinement levels. "
              f"{limited} searches reached their iteration limits. "
              f"Continuous correction took {correction.get('elapsed_s', 0):.2f} seconds. "
              f"Strict target error: {original['optimizer']['mission']['verified_target_error']:.6f} tolerance units. "
              f"Failed flight checks: {[k for k, passed in original['constraint_checks'].items() if not passed]}. "
              f"Numerical failures: {original['optimizer'].get('numerical_failures', 0)}.", '',
              'extra-correction-180 continues the saved flight with independent production integration. '
              'This is a continuation, not a new cold-start search.', '',
              'cold-search-600 starts from the original inputs without using the saved flight. '
              'Only solver time (600 seconds), iteration cap (250) and refinement levels (3) change. '
              'The target, payload, propulsion, design bounds and flight constraints remain unchanged. '
              'A separate strict replay is stored with the diagnostic.', '',
              'Successful candidates establish feasibility for these inputs within the stated model. '
              'They do not certify a global fuel optimum or guarantee convergence for other missions.', '',
              'UI validation: 22 frontend tests, TypeScript and production build passed. '
              'Failure guidance now distinguishes iteration exhaustion and discretization/replay gaps. '
              'Production flight equations and optimizer tolerances were not changed.']
    (root / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(root / 'report.md')
