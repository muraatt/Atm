"""Isolated full-mission merit regularization study; production is untouched."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from verify_stage_sizing import run
from atmosphere import mission_solver

if __name__=='__main__':
    destination=ROOT/'data/stage-sizing/conditioning-study'
    destination.mkdir(parents=True,exist_ok=True)
    source=(ROOT/'backend/atmosphere/collocation.py').read_text(encoding='utf-8')
    if 'self.conditioning_function=' in source:
        raise SystemExit('Velocity conditioning is now in production. Use verify_stage_sizing.py for current validation; this archived experiment must not add the merit twice.')
    marker='self.feasibility_objective=sum(x*x for x in other)'
    replacement=marker+'''
        # Soft merit tie-break only; never an added flight constraint.
        if self.analytic and self.final_coast_slice is None and self.seed_plan.burns[-1].steering_reference=='inertial_velocity':
            omega=self.frame_lookup(t)[4:]
            relative=v-ca.cross(omega,r)
            relative_speed=ca.sqrt(ca.dot(relative,relative)+1e-30)
            self.feasibility_objective+=10*ca.fmax(0,3-relative_speed)**2
'''
    assert marker in source
    module_path=destination/'collocation_diagnostic.py'
    module_path.write_text(source.replace(marker,replacement),encoding='utf-8')
    spec=importlib.util.spec_from_file_location('atmosphere.collocation_diagnostic',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    mission_solver.CollocationProblem=module.CollocationProblem
    args=SimpleNamespace(budget=1200,iterations=500,mesh=24,refinements=3,label='conditioning-study')
    (destination/'diagnostic-provenance.json').write_text(json.dumps({'production_modified':False,'diagnostic_module':str(module_path),'merit':'soft 3 m/s terminal relative-speed preference for inertial burns; no new flight constraints','note':'This is a development experiment, not a final production benchmark.'},indent=2),encoding='utf-8')
    sys.exit(0 if run('geo',args) else 1)
