"""Budget and cancellation behavior; flight physics has independent tests."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from atmosphere import mission_solver, optimization, restoration
from atmosphere.dynamics import SolverDeadline, Cancelled
from test_mission import orbital_case


@pytest.fixture
def driver(monkeypatch):
    sim,plan=orbital_case(1)
    reference=sim.run(plan,strict=True)
    s=sim.scenario;s.constraints.solver_budget_s=180
    s.constraints.max_topologies=1;s.constraints.refinement_levels=3
    clock=SimpleNamespace(now=0.)
    monkeypatch.setattr(mission_solver.time,'monotonic',lambda:clock.now)
    def factory(scenario,check):sim.check=check;return sim
    monkeypatch.setattr(mission_solver,'Simulation',factory)
    def run(candidate,strict=True,record=True):
        sim.check()
        result=deepcopy(reference);result['status']='Target Achieved'
        result['flight_plan']=candidate.model_dump()
        result['metadata']['verified_with_strict_integration']=strict
        return result
    monkeypatch.setattr(sim,'run',run)
    monkeypatch.setattr(optimization,'warm_start',lambda *a,**k:(None,None))
    monkeypatch.setattr(mission_solver,'analyze_consistency',lambda *a:{})
    return s,sim,plan,clock


def test_expensive_graph_refinement_retains_time_for_continuous_restoration(driver,monkeypatch):
    s,sim,plan,clock=driver
    constructed=[];restoration_time=[]
    class Graph:
        def __init__(self,*args):
            clock.now+=60;constructed.append(args[-1])
            self.iterations=1;self.invalid_candidates=0;self.best_x=None
            self.best_polish_x=None;self.last_x=None;self.feasibility_ready=False
        def solve(self,*args,**kwargs):
            clock.now+=20
            return plan,{'success':True,'return_status':'Solve_Succeeded'}
        def feasible(self,*args):return False
    monkeypatch.setattr(mission_solver,'CollocationProblem',Graph)
    def restore(sim,candidate,seconds,progress):
        restoration_time.append(seconds);return candidate,{}
    monkeypatch.setattr(restoration,'restore_continuous',restore)
    result=mission_solver.optimize_mission(s)
    assert len(constructed)==1
    assert restoration_time==[100]
    report=result['optimizer']['searches'][0]
    assert report['construction_elapsed_s']==60
    assert report['solve_elapsed_s']==20
    assert any('Refinement 2 deferred' in m for m in result['optimizer']['messages'])
    assert result['metadata']['verified_with_strict_integration']


@pytest.mark.parametrize('interruption',[SolverDeadline,Cancelled])
def test_transfer_seed_deadline_retains_candidates_but_cancellation_propagates(driver,monkeypatch,interruption):
    s,sim,plan,clock=driver
    s.target.perigee_altitude_m=20200000;s.target.apogee_altitude_m=20200000
    monkeypatch.setattr(optimization,'warm_start',lambda *a,**k:(plan,None))
    def seed(*args):
        if interruption is SolverDeadline:clock.now=180
        raise interruption()
    monkeypatch.setattr(mission_solver,'seed_from_transfer',seed)
    if interruption is Cancelled:
        with pytest.raises(Cancelled):mission_solver.optimize_mission(s)
    else:
        result=mission_solver.optimize_mission(s)
        assert result['optimizer']['termination_reason']=='total_budget'
        assert result['metadata']['verified_with_strict_integration']


def test_high_orbit_correction_recenters_only_after_strict_acceptance(driver,monkeypatch):
    s,sim,plan,clock=driver
    s.target.perigee_altitude_m=20200000;s.target.apogee_altitude_m=20200000
    class Graph:
        def __init__(self,*args):
            clock.now+=60;self.iterations=1;self.invalid_candidates=0
            self.best_x=None;self.best_polish_x=None;self.last_x=None;self.feasibility_ready=False
        def solve(self,*a,**k):clock.now+=20;return plan,{'success':True,'return_status':'Solve_Succeeded'}
        def feasible(self,*a):return False
    monkeypatch.setattr(mission_solver,'CollocationProblem',Graph)
    allocations=[]
    def correct(sim,candidate,seconds,progress):
        allocations.append(seconds);clock.now+=20
        return candidate,{'status':'Target Not Reached' if len(allocations)==1 else 'Target Achieved',
                          'evaluations':len(allocations),'elapsed_s':20}
    monkeypatch.setattr(restoration,'restore_continuous',correct)
    result=mission_solver.optimize_mission(s)
    # The fixture's non-strict replay claims success even on pass one. The
    # strict corrector status must control whether another pass is needed.
    assert allocations==[50,80]
    report=result['optimizer']['mission']['continuous_restoration']
    assert len(report['passes'])==2
    assert report['evaluations']==3
    assert report['elapsed_s']==40
