"""Summarize completed, independently replayed surface mission audits."""
import argparse
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--label',default='angle-consistent')
    args=parser.parse_args();folder=ROOT/'data/stage-sizing'/args.label
    lines=['# Joint stage sizing — surface mission verification','',
        'Payload: 3,000 kg. Two stages. Engine identities/counts and propulsion ratings are fixed. Target tolerances: 1 km and 0.1 degrees. Each audit includes a fresh strict DOP853 replay from the original surface input.','',
        '| Mission | Audit | Perigee error (m) | Apogee error (m) | Inclination error (degrees) | Final initial mass (t) |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    complete=True;states={};completed=[]
    for case in ('leo','meo','gto','geo'):
        path=folder/case/'audit.json'
        if not path.exists():
            lines.append(f'| {case.upper()} | Running / pending audit | — | — | — | — |')
            complete=False;states[case]='pending';continue
        audit=json.loads(path.read_text(encoding='utf-8'))
        passed=audit['passed'] and not audit.get('source_files_changed_during_run',[])
        completed.append((case,audit,passed))
        complete&=passed;states[case]='passed' if passed else 'failed'
        errors={r['parameter']:r['error'] for r in audit['target_comparison']}
        def display(key):
            value=errors.get(key);return 'Undefined' if value is None else f'{value:.6f}'
        mass=audit['stage_sizing']['optimized_initial_mass_kg']/1000
        lines.append(f"| {case.upper()} | {'Passed' if passed else 'Not passed'} | {display('perigee_altitude_m')} | {display('apogee_altitude_m')} | {display('inclination_deg')} | {mass:.3f} |")
    lines.extend(['','Input initial mass: 473.000 t. A feasible final design can need more mass than the input design. The objective prices burned and discarded propellant; it is not a proof of the global minimum.','',
                  'A passing audit requires modified masses, fixed payload/engines, preserved original inputs, actual surface departure, all orbit and flight checks, original sizing bounds, strict independent replay, balanced propellant and unchanged source files during the run.','',
                  'Published engine thrust references are combined with explicitly assumed missing Isp/vacuum booster thrust and hardware/tank mass bounds. These are synthetic preliminary design benchmarks, not manufacturer launcher ratings.','',
                  f"Overall: {'All four final audits passed.' if complete else 'Verification remains in progress; do not mark the implementation complete.'}"])
    for case,audit,passed in completed:
        lines.extend(['',f'## {case.upper()} verification','',
            f"Result: **{'Passed' if passed else 'Not passed'}**. Status: {audit['status']}. Test wall time: {audit['wall_s']:.1f} s.",
            '',f"Evidence: [original inputs]({case}/input.json), [optimizer result]({case}/result.json), [independent strict replay]({case}/independent-replay.json), [audit]({case}/audit.json).",'',
            '| Stage | Original dry (t) | Optimized dry (t) | Original propellant (t) | Optimized propellant (t) |',
            '| --- | ---: | ---: | ---: | ---: |'])
        input_path=folder/case/'input.json'
        if input_path.exists():
            settings=json.loads(input_path.read_text(encoding='utf-8'))['constraints']
            lines.insert(len(lines)-2,f"Solver settings: {settings['solver_budget_s']:.0f} s budget; {settings['max_iterations']} iterations per search; {settings['mesh_segments']} initial mesh segments; {settings['refinement_levels']} refinement levels.")
            lines.insert(len(lines)-2,'')
        for stage in audit['stage_sizing']['stages']:
            masses=[stage[key]/1000 for key in ('original_dry_mass_kg','dry_mass_kg','original_propellant_mass_kg','propellant_mass_kg')]
            lines.append('| '+stage['stage']+' | '+' | '.join(f'{mass:.6f}' for mass in masses)+' |')
        failures=[name for name,value in audit['checks'].items() if not value]
        changes=audit.get('source_files_changed_during_run',[])
        lines.extend(['',f"Verified pad T/W: {audit['stage_sizing']['verified_initial_pad_twr']:.6f}; required minimum: {audit['stage_sizing']['minimum_initial_twr']:.6f}.",
            'Failed checks: '+(', '.join(failures) if failures else 'None')+'.',
            'Source changes during run: '+(', '.join(changes) if changes else 'None')+'.'])
        correction=audit.get('continuous_restoration',{})
        lines.extend(['',f"Continuous cost search stopped because: `{correction.get('cost_stop_reason','Not recorded')}`. The verified feasible design was retained and independently replayed.",
            'This is a local feasible design, not a certified global minimum. A fresh search can return a more expensive design than an archived feasible search.'])
    folder.mkdir(parents=True,exist_ok=True);(folder/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'complete':complete,'states':states,'report':str(folder/'report.md')}))
