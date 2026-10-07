"""Regression tests for invalid search states and optimizer lifecycle failures."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from atmosphere import optimization
from atmosphere.dynamics import Cancelled, InvalidTrialState, NumericalFailure, Simulation, SolverDeadline
from atmosphere.earth import unit
from atmosphere.environment import EnvironmentError, MSISAtmosphere, StandardAtmosphere
from atmosphere.models import Burn, EnvironmentConfig, FlightPlan, default_scenario
from atmosphere.optimization import MultipleShooting, templates


@pytest.fixture
def scenario():
    value=default_scenario()
    value.environment.model='standard'
    value.constraints.solver_method='legacy'
    value.constraints.max_mission_duration_s=3000
    return value


@pytest.mark.parametrize('state,time',[
    ([-3034499.8597411127,-2865690.503764592,2535101.30481248,15810.731438028693,-15810.731438028695,-11759.474739090188,80000],22.39101345223649),
    ([-3389164.792415101,-3620577.5032691276,2876667.1397343227,15808.201661426798,-15807.874803140168,-12670.332995568882,80000],40.346015590030596),
])
def test_saved_underground_failures_stop_before_atmosphere(scenario,state,time,monkeypatch):
    sim=Simulation(scenario)
    def forbidden(*args): raise AssertionError('Underground trial reached atmosphere')
    monkeypatch.setattr(sim.atmosphere,'sample',forbidden)
    segment=sim.integrate(np.array(state),time,2654.8216826489947,1)
    assert segment.status=='Impact'
    assert segment.time==time
    assert np.array_equal(segment.state,state)


@pytest.mark.parametrize('component,value',[(0,np.nan),(3,np.inf),(6,np.nan)])
def test_nonfinite_state_rejected_before_provider_lookup(scenario,component,value,monkeypatch):
    sim=Simulation(scenario)
    state=sim.initial.copy();state[component]=value
    monkeypatch.setattr(sim.atmosphere,'sample',lambda *args:pytest.fail('Provider was queried'))
    with pytest.raises(InvalidTrialState,match='finite'):
        sim.integrate(state,0,10,0)


def test_below_sea_level_launch_is_not_rejected(scenario):
    scenario.launch.elevation_m=-400
    scenario.launch.geoid_height_m=-40
    sim=Simulation(scenario)
    assert sim.frames.geographic(0,sim.initial[:3])[2]==pytest.approx(-440,abs=1e-5)
    segment=sim.integrate(sim.initial.copy(),0,2,0,Burn(stage_index=0,duration_s=2))
    assert segment.status=='Complete'
    assert sim.surface_clearance(segment.time,segment.state[:3])[0]>0


def test_coast_time_limit_does_not_create_a_phantom_ignition(scenario):
    scenario.constraints.max_mission_duration_s=12
    scenario.vehicle.stages[0].separation_delay_s=0
    plan=FlightPlan(burns=[Burn(stage_index=0,duration_s=10),
                          Burn(stage_index=1,duration_s=2,coast_before_s=10)])
    result=Simulation(scenario).run(plan)
    assert result['status']=='Target Not Reached'
    assert result['termination_reason']=='Time Limit'
    assert not result['constraint_checks']['mission_duration']
    assert result['summary']['arrival_time_s']==pytest.approx(12)
    assert len([e for e in result['events'] if e['event']=='Ignition'])==1
    assert result['propellant_ledger'][1]['burned_kg']==0


def test_early_impact_mass_is_identical_with_and_without_series(scenario):
    plan=FlightPlan(burns=[Burn(stage_index=0,duration_s=2),Burn(stage_index=1,duration_s=2)])
    results=[]
    for record in (True,False):
        sim=Simulation(scenario)
        sim.initial[:3]+=1000*unit(sim.initial[:3])
        sim.initial[3:6]=sim.frames.earth_velocity(0,sim.initial[:3])-5000*unit(sim.initial[:3])
        results.append(sim.run(plan,record=record))
    assert all(r['status']=='Impact' for r in results)
    assert results[0]['summary']['final_mass_kg']==pytest.approx(results[1]['summary']['final_mass_kg'],abs=.01)
    expected=scenario.vehicle.initial_mass_kg-results[0]['summary']['propellant_burned_kg']
    assert results[1]['summary']['final_mass_kg']==pytest.approx(expected,abs=.01)
    import json
    for result in results:
        json.dumps(result,allow_nan=False)


def test_extreme_finite_state_is_rejected_without_overflow(scenario):
    sim=Simulation(scenario)
    state=sim.initial.copy();state[0]=1e200
    with np.errstate(over='raise',invalid='raise'):
        with pytest.raises(InvalidTrialState,match='integration domain'):
            sim.integrate(state,0,10,0)


def test_surface_extension_still_resolves_real_impact(scenario):
    sim=Simulation(scenario)
    state=sim.initial.copy();state[:3]+=unit(state[:3])*1000
    state[3:6]=sim.frames.earth_velocity(0,state[:3])-5000*unit(state[:3])
    segment=sim.integrate(state,0,5,0,strict=True)
    assert segment.status=='Impact'
    assert 0<segment.time<1
    assert sim.surface_clearance(segment.time,segment.state[:3])[0]==pytest.approx(-.1,abs=1e-4)


@pytest.mark.parametrize('model',['standard','msis'])
def test_atmosphere_rejects_nonfinite_and_deep_queries(scenario,model):
    provider=StandardAtmosphere() if model=='standard' else MSISAtmosphere(EnvironmentConfig(solar_mode='nominal'),scenario.launch)
    with pytest.raises(EnvironmentError,match='finite'): provider.sample(np.nan,0,100,0)
    with pytest.raises(EnvironmentError,match='surface-height'): provider.sample(0,0,-100000,0)


def test_bad_phase_boundary_has_finite_constraints_and_cannot_replace_best(scenario):
    problem=MultipleShooting(Simulation(scenario),templates(scenario)[0],.25)
    good=problem.evaluate(problem.seed)
    best=problem.best_x.copy()
    bad=problem.seed.copy();bad[problem.state_slices[0]][:3]*=.8
    failed=problem.evaluate(bad)
    assert not failed['valid']
    for key in ('errors','eq','ineq','terminal'):
        assert failed[key].shape==good[key].shape
        assert np.all(np.isfinite(failed[key]))
    assert np.min(failed['ineq'])<0
    assert np.array_equal(best,problem.best_x)
    assert problem.evaluate(problem.seed)['valid']


def test_integration_failure_is_recoverable_candidate_not_search_abort(scenario,monkeypatch):
    sim=Simulation(scenario)
    problem=MultipleShooting(sim,templates(scenario)[0],.25)
    problem.evaluate(problem.seed)
    best=problem.best_x.copy()
    integrate=sim.integrate
    calls=0
    def fail_once(*args,**kwargs):
        nonlocal calls
        calls+=1
        if calls==1: raise NumericalFailure('Injected integration failure')
        return integrate(*args,**kwargs)
    monkeypatch.setattr(sim,'integrate',fail_once)
    trial=problem.seed.copy();trial[0]-=.001
    failed=problem.evaluate(trial)
    assert not failed['valid']
    assert problem.numerical_failures==1
    assert np.array_equal(problem.best_x,best)
    trial[0]-=.001
    assert problem.evaluate(trial)['valid']


def test_nonfinite_optimizer_vector_returns_fixed_finite_residuals(scenario):
    problem=MultipleShooting(Simulation(scenario),templates(scenario)[0],.25)
    good=problem.evaluate(problem.seed)
    trial=problem.seed.copy();trial[0]=np.nan
    failed=problem.evaluate(trial)
    assert not failed['valid']
    for key in ('errors','eq','ineq','terminal'):
        assert np.all(np.isfinite(failed[key]))
        assert failed[key].shape==good[key].shape


@pytest.mark.parametrize('exception',[Cancelled,SolverDeadline])
def test_rejection_does_not_swallow_cancel_or_deadline(scenario,exception):
    sim=Simulation(scenario)
    problem=MultipleShooting(sim,templates(scenario)[0],.25)
    def stop(): raise exception()
    sim.check=stop
    with pytest.raises(exception): problem.evaluate(problem.seed)


@pytest.fixture
def lifecycle(scenario,monkeypatch):
    """Controlled optimizer/clock; physics itself is covered by the tests above."""
    sim=Simulation(scenario)
    reference=sim.run(FlightPlan(burns=[Burn(stage_index=i,duration_s=10) for i in range(2)]))
    reference['status']='Target Not Reached'
    scenario.target.apogee_altitude_m=3000000  # skip the low-orbit initial fit
    clock=SimpleNamespace(now=0.0)
    monkeypatch.setattr(optimization.time,'monotonic',lambda:clock.now)
    def factory(scenario,check):
        sim.check=check
        return sim
    monkeypatch.setattr(optimization,'Simulation',factory)
    def run(plan,strict=True,record=True):
        result=deepcopy(reference)
        result['flight_plan']=plan.model_dump()
        result['metadata']['verified_with_strict_integration']=strict
        return result
    monkeypatch.setattr(sim,'run',run)
    class Problem:
        def __init__(self,sim,specs,angle):
            self.seed=np.array([.25]);self.best_x=self.seed.copy();self.bounds=[(0,1)]
            self.best_merit=1.;self.invalid_candidates=0;self.numerical_failures=0;self.failure_reasons={}
        def plan(self,x):
            return FlightPlan(launch_azimuth_deg=123,burns=[Burn(stage_index=i,duration_s=10) for i in range(2)])
        def evaluate(self,x):
            return {'valid':True,'eq':np.array([1.]),'ineq':np.array([1.]),'terminal':np.array([-1.]),'errors':np.array([1.]),'objective':1.}
    monkeypatch.setattr(optimization,'MultipleShooting',Problem)
    return sim,clock,reference,run


def test_partial_time_limit_is_not_total_budget_exhaustion(scenario,lifecycle,monkeypatch):
    sim,clock,_,_=lifecycle
    calls=0
    def minimize(*args,**kwargs):
        nonlocal calls
        calls+=1
        if calls==1:
            clock.now=46  # first search receives 180 / 4 = 45 seconds
            sim.check()
        return SimpleNamespace(success=False,status=9,message='Iteration limit reached')
    monkeypatch.setattr(optimization,'minimize',minimize)
    result=optimization.optimize(scenario)
    assert result['optimizer']['searches'][0]['stop_reason']=='search_time_limit'
    assert result['optimizer']['searches'][1]['stop_reason']=='iteration_limit'
    assert not result['optimizer']['total_budget_exhausted']
    assert not result['consistency']['diagnostic']['optimizer_budget_exhausted']
    assert result['optimizer']['termination_reason']=='searches_completed'


def test_failed_strict_candidate_does_not_displace_baseline(scenario,lifecycle,monkeypatch):
    sim,_,_,run=lifecycle
    monkeypatch.setattr(optimization,'minimize',lambda *args,**kwargs:SimpleNamespace(success=False,status=9,message='Iteration limit reached'))
    def replay(plan,strict=True,record=True):
        if strict and plan.launch_azimuth_deg==123:
            raise NumericalFailure('Injected strict replay failure')
        return run(plan,strict,record)
    monkeypatch.setattr(sim,'run',replay)
    result=optimization.optimize(scenario)
    assert result['flight_plan']['launch_azimuth_deg']!=123
    assert result['optimizer']['verification_failures']==['Injected strict replay failure']
    assert result['metadata']['verified_with_strict_integration']
