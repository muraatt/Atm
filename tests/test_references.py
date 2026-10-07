"""Replay saved, physically continuous insertion solutions independently."""
import json
from pathlib import Path

import pytest

from atmosphere.dynamics import Simulation
from atmosphere.models import FlightPlan, Scenario


@pytest.mark.parametrize('atmosphere',['msis','standard'])
def test_two_stage_leo_strict_reference(atmosphere):
    path=Path(__file__).resolve().parents[1]/'examples'/f'leo-{atmosphere}-reference.json'
    reference=json.loads(path.read_text(encoding='utf-8'))
    scenario=Scenario.model_validate(reference['scenario'])
    plan=FlightPlan.model_validate(reference['flight_plan'])
    result=Simulation(scenario).run(plan,strict=True,record=True)
    assert result['status']=='Target Achieved'
    assert result['metadata']['verified_with_strict_integration']
    # Worker/API persistence must accept the complete result without a custom
    # NumPy serializer, including every verification boolean.
    assert json.loads(json.dumps(result,allow_nan=False))['constraint_checks']==result['constraint_checks']
    assert all(row['within_tolerance'] for row in result['target_comparison'])
    assert all(result['constraint_checks'].values())
    assert 0<=result['summary']['max_thrust_angle_utilization']<=1+1e-6
    ledger=result['propellant_ledger']
    assert len(ledger)==2
    assert sum(row['initial_kg'] for row in ledger)==pytest.approx(
        sum(row['burned_kg']+row['remaining_kg']+row['discarded_kg'] for row in ledger),abs=1e-5)


def test_numpy_optimizer_commands_keep_native_json_verification_flags():
    import numpy as np
    path=Path(__file__).resolve().parents[1]/'examples'/'leo-standard-reference.json'
    reference=json.loads(path.read_text(encoding='utf-8'))
    scenario=Scenario.model_validate(reference['scenario'])
    scenario.target.arrival_time_min_s=0
    scenario.target.arrival_time_max_s=10000
    original=FlightPlan.model_validate(reference['flight_plan'])
    plan=original.model_copy(update={'launch_azimuth_deg':np.float64(original.launch_azimuth_deg)})
    result=Simulation(scenario).run(plan,strict=True)
    assert result['status']=='Target Achieved'
    assert all(type(value) is bool for value in result['constraint_checks'].values())
    json.dumps(result,allow_nan=False)


@pytest.mark.parametrize('limit',['dynamic_pressure','load_factor','earliest_arrival','latest_arrival'])
def test_feasible_orbit_is_rejected_when_requested_flight_limits_fail(limit):
    path=Path(__file__).resolve().parents[1]/'examples'/'leo-standard-reference.json'
    reference=json.loads(path.read_text(encoding='utf-8'))
    scenario=Scenario.model_validate(reference['scenario'])
    plan=FlightPlan.model_validate(reference['flight_plan'])
    baseline=reference['summary']
    expected=limit
    if limit=='dynamic_pressure':scenario.constraints.max_dynamic_pressure_pa=baseline['max_dynamic_pressure_pa']*.5
    elif limit=='load_factor':scenario.constraints.max_load_factor_g=baseline['max_load_factor_g']*.5
    elif limit=='earliest_arrival':
        scenario.target.arrival_time_min_s=baseline['arrival_time_s']+10;expected='arrival_window'
    else:
        scenario.target.arrival_time_max_s=baseline['arrival_time_s']-10;expected='arrival_window'
    result=Simulation(scenario).run(plan,strict=True)
    assert all(row['within_tolerance'] for row in result['target_comparison'])
    assert result['status']=='Target Not Reached'
    assert result['constraint_checks'][expected] is False
    assert all(value for name,value in result['constraint_checks'].items() if name!=expected)
    json.dumps(result,allow_nan=False)


def test_partial_target_strict_reference_keeps_apogee_and_inclination_free():
    path=Path(__file__).resolve().parents[1]/'examples'/'leo-partial-target-reference.json'
    reference=json.loads(path.read_text(encoding='utf-8'))
    scenario=Scenario.model_validate(reference['scenario'])
    result=Simulation(scenario).run(FlightPlan.model_validate(reference['flight_plan']),strict=True,record=True)
    assert scenario.target.apogee_altitude_m is None
    assert scenario.target.inclination_deg is None
    assert result['status']=='Target Achieved'
    assert [row['parameter'] for row in result['target_comparison']]==['perigee_altitude_m']
    assert result['orbit']['apogee_altitude_m']>500000
    assert all(result['constraint_checks'].values())


def test_sized_surface_geo_strict_reference_preserves_engines_and_fuel_ledger():
    path=Path(__file__).resolve().parents[1]/'examples'/'geo-stage-sizing-reference.json'
    reference=json.loads(path.read_text(encoding='utf-8'))
    scenario=Scenario.model_validate(reference['scenario'])
    before=scenario.model_dump()
    result=Simulation(scenario).run(FlightPlan.model_validate(reference['flight_plan']),strict=True)
    assert abs(result['series'][0]['altitude_m'])<.1
    assert result['status']=='Target Achieved'
    assert all(row['within_tolerance'] for row in result['target_comparison'])
    assert all(result['constraint_checks'].values())
    assert result['stage_sizing']['bounds_verified']
    json.dumps(result,allow_nan=False)
    assert scenario.model_dump()==before
    assert result['scenario']['vehicle']['payload_mass_kg']==scenario.vehicle.payload_mass_kg
    for original,sized in zip(scenario.vehicle.stages,result['scenario']['vehicle']['stages']):
        assert sized['engine_id']==original.engine_id
        assert sized['engine_count']==original.engine_count
        assert sized['vacuum_thrust_n']==original.vacuum_thrust_n
        assert sized['isp_vacuum_s']==original.isp_vacuum_s
    for row in result['propellant_ledger']:
        assert row['initial_kg']==pytest.approx(row['burned_kg']+row['remaining_kg']+row['discarded_kg'],abs=.01)


def test_surface_geo_plane_change_strict_reference_flies_all_three_upper_ignitions():
    reference=json.loads((Path(__file__).resolve().parents[1]/'examples'/'geo-cape-node-reference.json').read_text())
    scenario=Scenario.model_validate(reference['scenario'])
    plan=FlightPlan.model_validate(reference['flight_plan'])
    assert scenario.launch.latitude_deg>28
    assert scenario.target.inclination_deg==0
    assert scenario.target.altitude_tolerance_m==1000
    assert scenario.target.angle_tolerance_deg==.1
    result=Simulation(scenario).run(plan,strict=True)
    assert abs(result['series'][0]['altitude_m'])<.1
    assert result['series'][0]['latitude_deg']==pytest.approx(scenario.launch.latitude_deg,abs=1e-5)
    assert result['status']=='Target Achieved'
    assert all(row['within_tolerance'] for row in result['target_comparison'])
    assert all(result['constraint_checks'].values())
    upper=scenario.vehicle.stages[-1].name
    assert sum(e['event']=='Ignition' and e['stage']==upper for e in result['events'])==3
    assert plan.burns[-2].coast_before_s>1000
    assert plan.burns[-1].coast_before_s>15000
    for row in result['propellant_ledger']:
        assert row['initial_kg']==pytest.approx(row['burned_kg']+row['remaining_kg']+row['discarded_kg'],abs=.01)
    json.dumps(result,allow_nan=False)
