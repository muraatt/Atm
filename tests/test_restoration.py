"""Continuous restoration preserves strict target/flight acceptance."""
from atmosphere.restoration import restore_continuous
from atmosphere.dynamics import Cancelled, NumericalFailure
import pytest
from test_mission import orbital_case


def test_segment_reuse_matches_uncached_flight_and_cannot_mutate_stored_state():
    from atmosphere.restoration import SegmentReplayCache
    import numpy as np
    sim,plan=orbital_case(1)
    cache=SegmentReplayCache()
    trial=cache.simulation(sim,sim.scenario,sim.check)
    expected=sim.run(plan,strict=True)
    first=trial.run(plan,strict=True)
    assert first==expected
    state=trial._last_state.copy()
    first['series'][0]['mass_kg']=-999
    trial._last_state[6]=-999
    again=trial.run(plan,strict=True)
    assert again==expected
    assert np.array_equal(trial._last_state,state)
    assert cache.hits>0
    changed=plan.model_copy(deep=True)
    changed.burns[0].duration_s*=1.001
    previous_misses=cache.misses
    assert trial.run(changed,strict=True)==sim.run(changed,strict=True)
    assert cache.misses>previous_misses


def test_segment_reuse_separates_design_and_integration_settings_and_checks_cancel():
    from atmosphere.restoration import SegmentReplayCache
    sim,plan=orbital_case(1)
    cache=SegmentReplayCache(capacity=2)
    trial=cache.simulation(sim,sim.scenario,sim.check)
    trial.run(plan,strict=True)
    changed=sim.scenario.model_copy(deep=True)
    changed.vehicle.payload_mass_kg+=100
    designed=cache.simulation(sim,changed,sim.check)
    independent=type(sim)(changed,frames=sim.frames,atmosphere=sim.atmosphere)
    independent.initial=sim.initial.copy()
    assert designed.run(plan,strict=True)==independent.run(plan,strict=True)
    previous_misses=cache.misses
    trial.run(plan,strict=False)
    assert cache.misses>previous_misses
    def cancel():raise Cancelled()
    stopped=cache.simulation(sim,sim.scenario,cancel)
    with pytest.raises(Cancelled):stopped.run(plan,strict=False)
    assert len(cache.entries)<=2


def test_continuous_restoration_repairs_a_finite_burn_without_relaxing_target():
    sim,plan=orbital_case(1)
    target=sim.scenario.target.model_dump()
    plan.burns[0].duration_s*=.985
    fixed=sim.scenario.vehicle.model_dump()
    restored,report=restore_continuous(sim,plan,30,lambda _:None)
    result=sim.run(restored,strict=True)
    assert result['status']=='Target Achieved',result['target_comparison']
    assert report['evaluations']>0
    assert all(row['within_tolerance'] for row in result['target_comparison'])
    assert all(result['constraint_checks'].values())
    assert sim.scenario.target.model_dump()==target
    assert sim.scenario.vehicle.model_dump()==fixed


def test_restoration_propagates_user_cancellation():
    sim,plan=orbital_case(1)
    def cancel():raise Cancelled()
    sim.check=cancel
    with pytest.raises(Cancelled):restore_continuous(sim,plan,30,lambda _:None)


def test_restoration_penalizes_a_truncated_mission_duration():
    sim,plan=orbital_case(0)
    sim.scenario.constraints.max_mission_duration_s=1.
    restored,report=restore_continuous(sim,plan,10,lambda _:None)
    result=sim.run(restored,strict=True)
    assert not result['constraint_checks']['mission_duration']
    assert result['status']=='Target Not Reached'
    assert report['score']>=1000


def test_cost_polish_failure_preserves_the_verified_feasible_plan(monkeypatch):
    sim,plan=orbital_case(1)
    reference=sim.run(plan,strict=True)
    from atmosphere.models import TargetOrbit
    sim.scenario.target=TargetOrbit(
        perigee_altitude_m=reference['orbit']['perigee_altitude_m'],
        apogee_altitude_m=reference['orbit']['apogee_altitude_m'],
        inclination_deg=reference['orbit']['inclination_deg'])
    def failed_trial(*args,**kwargs):
        raise NumericalFailure('Rejected numerical trial during cost polishing')
    monkeypatch.setattr('atmosphere.restoration.minimize',failed_trial)
    restored,report=restore_continuous(sim,plan,30,lambda _:None)
    assert report['cost_stop_reason']=='numerical_trial_rejected'
    assert sim.run(restored,strict=True)['status']=='Target Achieved'
    assert report['best_feasible_cost']==report['initial_feasible_cost']


def test_time_polishing_can_shorten_a_coast_and_rechecks_the_orbit():
    sim,plan=orbital_case(1)
    sim.scenario.objective='time'
    plan.final_coast_s=600
    reference=sim.run(plan,strict=True)
    from atmosphere.models import TargetOrbit
    sim.scenario.target=TargetOrbit(
        perigee_altitude_m=reference['orbit']['perigee_altitude_m'],
        apogee_altitude_m=reference['orbit']['apogee_altitude_m'],
        inclination_deg=reference['orbit']['inclination_deg'])
    restored,report=restore_continuous(sim,plan,30,lambda _:None)
    verified=sim.run(restored,strict=True)
    assert verified['status']=='Target Achieved'
    assert all(verified['constraint_checks'].values())
    assert restored.final_coast_s<plan.final_coast_s-10
    assert verified['summary']['arrival_time_s']<reference['summary']['arrival_time_s']-10
    assert report['cost_evaluations']>0
    assert report['best_feasible_cost']<report['initial_feasible_cost']


def test_joint_cost_steps_change_design_and_preserve_verified_target():
    from atmosphere.models import StageMass,StageMassBounds,TargetOrbit
    from atmosphere.sizing import apply_stage_masses
    sim,plan=orbital_case(0)
    stage=sim.scenario.vehicle.stages[0]
    stage.mass_bounds=StageMassBounds(dry_mass_min_kg=500,dry_mass_max_kg=1500,
        propellant_min_kg=8000,propellant_max_kg=12000,hardware_mass_kg=200,tank_structure_ratio=.02)
    sim.scenario.vehicle.optimize_stage_masses=True
    reference=sim.run(plan,strict=True)
    sim.scenario.target=TargetOrbit(perigee_altitude_m=reference['orbit']['perigee_altitude_m'],
                                  apogee_altitude_m=reference['orbit']['apogee_altitude_m'],inclination_deg=0)
    original=sim.scenario.model_dump()
    plan.stage_masses=[StageMass(dry_mass_kg=stage.dry_mass_kg,propellant_mass_kg=stage.propellant_mass_kg)]
    restored,report=restore_continuous(sim,plan,60,lambda _:None)
    # This orbital-only unit test preserves its initial state when applying
    # the new design, exactly as the restoration trial does.
    designed=apply_stage_masses(sim.scenario,restored.stage_masses)
    replay=type(sim)(designed,frames=sim.frames,atmosphere=sim.atmosphere)
    replay.initial[:6]=sim.initial[:6]
    verified=replay.run(restored.model_copy(update={'stage_masses':None}),strict=True)
    assert verified['status']=='Target Achieved'
    assert all(verified['constraint_checks'].values())
    assert sim.scenario.model_dump()==original
    assert any(abs(getattr(restored.stage_masses[0],key)-getattr(stage,key))>1
               for key in ('dry_mass_kg','propellant_mass_kg'))
    assert type(restored.launch_azimuth_deg) is float
    import json
    json.dumps(verified,allow_nan=False)
    assert report['best_feasible_cost']<report['initial_feasible_cost']-1e-6


def test_equatorial_plane_correction_uses_strict_inclination_acceptance():
    sim,plan=orbital_case(.11)
    sim.scenario.target.inclination_deg=0
    before=sim.run(plan,strict=True)
    assert before['orbit']['inclination_deg']>sim.scenario.target.angle_tolerance_deg
    restored,report=restore_continuous(sim,plan,30,lambda _:None)
    verified=sim.run(restored,strict=True)
    assert verified['status']=='Target Achieved',verified['target_comparison']
    assert verified['orbit']['inclination_deg']<=sim.scenario.target.angle_tolerance_deg
    assert sim.scenario.target.inclination_deg==0


def test_plane_correction_remains_continuous_with_a_zero_steering_endpoint():
    sim,plan=orbital_case(.11)
    angle=plan.burns[0].angle_fractions[0]
    plan.burns[0].angle_fractions=(angle,angle,0.)
    reference=sim.run(plan,strict=True)['orbit']
    from atmosphere.models import TargetOrbit
    sim.scenario.target=TargetOrbit(perigee_altitude_m=reference['perigee_altitude_m'],
        apogee_altitude_m=reference['apogee_altitude_m'],inclination_deg=0,angle_tolerance_deg=.02)
    assert reference['inclination_deg']>.02
    restored,report=restore_continuous(sim,plan,30,lambda _:None)
    verified=sim.run(restored,strict=True)
    assert verified['status']=='Target Achieved',verified['target_comparison']
    assert all(verified['constraint_checks'].values())
    assert restored.burns[0].angle_fractions[-1]==0
    assert report['evaluations']>1
