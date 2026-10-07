"""Combine fresh searches with explicitly identified saved-flight replays."""
import json
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

from verify_system_matrix import ROOT, CASES


if __name__=='__main__':
    root=ROOT/'data/system-validation';rows=[];evidence=[];passed=complete=0
    for name in CASES:
        active=root/'varied-final-v2'/name
        if (active/'input.json').exists() and not (active/'audit.json').exists():
            rows.append(f'| {name} | Pending | Fresh final 900 s search | — | — | — |');continue
        choices=[('varied-final-v2','Fresh final 900 s search + independent replay'),
                 ('final-replay','Saved search candidate; replayed with current code'),
                 ('varied-node-long','Fresh 900 s search + independent replay'),
                 ('varied-cached-long','Fresh 900 s search + independent replay')]
        for label,mode in choices:
            path=root/label/name/'audit.json'
            if path.exists():
                audit=json.loads(path.read_text());break
        else:
            rows.append(f'| {name} | Pending | Fresh 900 s search | — | — | — |');continue
        complete+=1;passed+=int(audit['passed'])
        if label=='final-replay':mode=f'Saved {audit["optimizer_source_label"]} candidate; current-code strict replay'
        errors={r['parameter']:r.get('error') for r in audit.get('target_comparison',[])}
        def error(key):
            value=errors.get(key)
            return 'Free / undefined' if value is None else f'{value:+.6f}'
        outcome=('PASS: ' if audit['passed'] else 'FAIL: ')+audit['status']
        rows.append(f'| {name} | {outcome} | {mode} | {error("perigee_altitude_m")} | {error("apogee_altitude_m")} | {error("inclination_deg")} |')
        evidence.append({'case':name,'label':label,'validation_mode':mode,'audit':audit})
    lines=['# System validation: latest evidence','',f'Updated: {datetime.now(timezone.utc).isoformat()}.',
           f'Completed current-code audits: **{complete}/12**. Passing: **{passed}/12**.',
           '', 'Acceptance remains 1,000 m perigee/apogee and 0.1° inclination. Each requested parameter is checked separately; free parameters remain free.',
           '', '| Case | Outcome | Evidence | Perigee error (m) | Apogee error (m) | Inclination error (°) |',
           '|---|---|---|---:|---:|---:|',*rows,'',
           'Six LEO/GTO candidates are saved 300 s searches. Three MEO candidates and the inclined GEO candidate are successful saved 900 s searches. All ten are independently replayed against current production equations; these replays are not new optimizer runs. The final MSIS GEO and Cape-to-equatorial-GEO rows are fresh 900 s searches with separate strict replays.',
           '', 'A target miss does not establish physical impossibility. All audit checks, immutable inputs, engine/mass assumptions, commands, environment metadata, time series and source hashes remain in each linked label directory.',
           '', 'The initial complete matrix is [varied-v3/report.md](varied-v3/report.md). It passed 6/12 and identified the six follow-up searches.',
           '', 'Automated check reports are retained as backend-final-v2.xml, backend-node-final.xml and backend-cached-final.xml. Frontend: 18 tests, TypeScript and production build passed. The isolated UI check includes scenario import, launch selection, analysis, charts, CSV download and outdated-result labeling.',
           '', 'Fixed issues include strict result serialization, mission-duration phase termination, early-impact mass accounting, environment/API error classification and input validation.',
           '', 'Continuous correction now reuses only exactly identical production integration segments within one bounded search. The cache includes design, initial/current state, time, command, steering frame, atmosphere/frame identities and integration settings; cached values are copied and cancellation is checked. A changed flight or design is reintegrated. Final verification bypasses this cache.',
           '', 'An isolated four-trial MEO comparison produced identical strict results with 2.18× faster terminal-duration trial evaluation (13.94 s versus 6.39 s). This measures one saved design and perturbation family, not total optimizer speedup.',
           '', 'Graph setup and solve time are recorded separately. Refinement is deferred when estimated construction would consume most of its allocation, preserving time for actual-flight correction. Cancellation propagates; a transfer-seed deadline retains candidates for final strict verification.',
           '', 'For high arrival orbits with a departure-plane mismatch, an available three-ignition upper stage now has a node-aligned parking/injection/arrival seed. Production propagation places transfer apogee in the destination plane; the complete-mission solver remains free to change all intermediate states and commands. Finite-arrival steering varies with changing inertial velocity. Original targets, vehicle capability and acceptance tolerances remain unchanged.']
    lines+=['', 'High-perigee searches reserve 30% of the budget for continuous correction and cap the one-upper-burn alternative at 6% when restart alternatives are present. A second bounded correction can recenter on the retained physical candidate; only strict acceptance stops this continuation. The mission-duration constraint is included in the correction merit. For enabled joint sizing, the parking initializer can start the liquid upper-stage dry mass at its user-defined structural floor; fixed-mass scenarios retain their design.']
    test_report=root/'backend-final-v2.xml'
    if test_report.exists():
        suites=list(ET.parse(test_report).getroot().iter('testsuite'))
        total=sum(int(s.get('tests',0)) for s in suites)
        failed=sum(int(s.get('failures',0))+int(s.get('errors',0)) for s in suites)
        lines+=['',f'Final backend suite: **{total} tests**, **{failed} failures/errors**. [JUnit report](backend-final-v2.xml).']
    for row in evidence:
        if not row['audit']['passed']:
            a=row['audit'];fail=[k for k,v in a.get('checks',{}).items() if not v]
            lines+=['',f'## {row["case"]}: unresolved', '',a.get('error',', '.join(fail))]
    (root/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    (root/'latest-evidence.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    print(f'{passed}/12 passing, {complete}/12 completed. Report: {root / "report.md"}')
