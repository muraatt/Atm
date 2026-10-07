"""Summarize persisted audits without treating unfinished/missed cases as passes."""
import argparse
import json
from pathlib import Path

from verify_system_matrix import CASES,ROOT


def report(label):
    directory=ROOT/'data/system-validation'/label
    rows=[];complete=0;passed=0
    for name in CASES:
        path=directory/name/'audit.json'
        if not path.exists():
            rows.append(f'| {name} | Pending | — | — | — | — |')
            continue
        value=json.loads(path.read_text(encoding='utf-8'));complete+=1;passed+=int(value['passed'])
        errors={r['parameter']:r['error'] for r in value.get('target_comparison',[])}
        number=lambda key:'Free / undefined' if errors.get(key) is None else f'{errors[key]:+.6f}'
        state='PASS' if value['passed'] else 'FAIL'
        rows.append(f"| {name} | {state}: {value['status']} | {number('perigee_altitude_m')} | {number('apogee_altitude_m')} | {number('inclination_deg')} | {value['wall_s']:.1f} |")
    lines=[
        '# Varied surface-to-orbit system validation','',
        f'Completed: {complete}/{len(CASES)}. Passing complete optimizer + independent replay audits: {passed}/{len(CASES)}.','',
        'Original acceptance tolerances: 1,000 m perigee/apogee and 0.1° angles. Blank target parameters remain free. A search miss is a failed positive regression, not proof of physical impossibility.','',
        '| Case | Audit outcome | Perigee error (m) | Apogee error (m) | Inclination error (°) | Wall time (s) |',
        '|---|---|---:|---:|---:|---:|',*rows,'',
        'Each case contains the immutable original input, source hashes, optimizer result, independent strict DOP853 replay and audit. A passing audit requires target/flight constraints, source stability, launch coordinates/height, fixed payload/engine settings, mass and fuel closure, ignition limits, time ordering and model-domain reporting.','',
        'The matrix includes catalog departure sectors, polar/retrograde flight, partial targets, negative and elevated synthetic pad heights, manual MSIS and US76 environments, different engine configurations/payloads, fixed and optimized masses, and fuel/time objectives. Pad elevations/geoid values are explicit synthetic test inputs, not surveyed launch-pad elevations.','',
        'Engine thrust references are checked against the catalog. Missing Isp/vacuum thrust and hardware/tank assumptions are explicit study inputs; these cases are not manufacturer vehicle performance claims. Local search does not establish a global minimum.','',
        'Additional automated tests cover invalid inputs, no liftoff, impact, cancellation, restart recovery, unavailable solar data, strict result serialization, API lifecycle/exports and independently constructed Kepler states. Frontend checks cover target validation, engine selection, scenario signatures and ground-track geometry.','',
        'The isolated browser test verifies launch selection/import, two-stage engine profiles, fixed sizing mode, diagnostic analysis, results/charts, CSV download and Outdated labeling. Application user jobs and scenario storage are untouched by the test origin.','',
    ]
    for name in CASES:
        path=directory/name/'audit.json'
        if not path.exists():continue
        value=json.loads(path.read_text(encoding='utf-8'))
        if not value['passed']:
            failures=[key for key,ok in value.get('checks',{}).items() if not ok]
            lines.extend([f'## {name}: unresolved audit','',value.get('error') or ', '.join(failures),''])
    directory.mkdir(parents=True,exist_ok=True)
    output=directory/'report.md';output.write_text('\n'.join(lines),encoding='utf-8')
    print(f'{output}: {passed}/{len(CASES)} passed, {complete}/{len(CASES)} complete',flush=True)
    return passed==len(CASES)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--label',default='varied-json-fixed');args=parser.parse_args()
    report(args.label)
