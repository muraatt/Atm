import math

import pytest

from atmosphere.consistency import analyze_consistency, ideal_capacity
from atmosphere.earth import G0, R_E
from atmosphere.models import TargetOrbit, default_scenario


def test_stage_delta_v_uses_upper_stage_and_payload_mass():
    scenario=default_scenario()
    rows=ideal_capacity(scenario)
    assert rows[0]['ignition_mass_kg']==305500
    assert rows[0]['burnout_mass_kg']==85500
    assert rows[1]['ignition_mass_kg']==71500
    assert rows[0]['ideal_delta_v_m_s']==pytest.approx(G0*320*math.log(305500/85500))
    assert rows[1]['ideal_delta_v_m_s']==pytest.approx(G0*360*math.log(71500/6500))


def test_heavy_payload_capacity_shortfall_is_separate_from_solver_failure():
    scenario=default_scenario();scenario.vehicle.payload_mass_kg=35000
    scenario.vehicle.stages[1].propellant_mass_kg=64000
    scenario.target=TargetOrbit(perigee_altitude_m=230000,apogee_altitude_m=290000,inclination_deg=28.5)
    report=analyze_consistency(scenario)
    assert report['ideal_delta_v_m_s']==pytest.approx(6785.6118414)
    assert report['energy_screen_delta_v_m_s']>report['ideal_delta_v_m_s']
    assert any(f['code']=='delta_v_shortfall' for f in report['findings'])
    assert not any(f['code']=='solver_budget' for f in report['findings'])
    ceiling=report['optimistic_payload_ceiling_kg']
    assert 1<ceiling<35000
    assert sum(r['ideal_delta_v_m_s'] for r in ideal_capacity(scenario,ceiling))==pytest.approx(report['energy_screen_delta_v_m_s'])


def test_free_target_parameters_use_only_the_implicit_orbital_floor():
    scenario=default_scenario()
    scenario.target=TargetOrbit(perigee_altitude_m=None,inclination_deg=28.5)
    floor=analyze_consistency(scenario)
    scenario.target=TargetOrbit(perigee_altitude_m=100000)
    assert analyze_consistency(scenario)['energy_screen_delta_v_m_s']==pytest.approx(floor['energy_screen_delta_v_m_s'])
    scenario.target=TargetOrbit(perigee_altitude_m=None,apogee_altitude_m=35786000)
    assert analyze_consistency(scenario)['energy_screen_delta_v_m_s']>floor['energy_screen_delta_v_m_s']
    assert scenario.target.perigee_altitude_m is None


def test_solid_motor_ignition_and_peak_liftoff_twr_are_distinct():
    scenario=default_scenario();stage=scenario.vehicle.stages[0]
    stage.engine_type='solid';stage.solid_burn_curve=[(0,0),(0.5,1),(1,0)]
    report=analyze_consistency(scenario)
    assert report['initial_twr']==0
    assert report['maximum_pad_twr']>1
    stage.vacuum_thrust_n=1000
    assert any(f['code']=='liftoff' for f in analyze_consistency(scenario)['findings'])


def test_result_diagnosis_distinguishes_descent_reserve_and_search_timeout():
    scenario=default_scenario()
    result={'status':'Target Not Reached','series':[{'altitude_m':40000,'position_gcrs_m':[R_E+40000,0,0],'velocity_gcrs_m_s':[-1000,4000,0]}],
            'orbit':{'perigee_altitude_m':-4000000},'summary':{'propellant_remaining_kg':9000},
            'propellant_ledger':[{'initial_kg':10000,'burned_kg':1000,'remaining_kg':9000,'discarded_kg':0}],
            'optimizer':{'iterations':0,'messages':['Total optimization budget reached'],'total_budget_exhausted':True}}
    report=analyze_consistency(scenario,result)
    codes={f['code'] for f in report['findings']}
    assert {'suborbital','descending','fuel_remaining','solver_budget'}<=codes
    assert report['diagnostic']['final_radial_speed_m_s']==-1000
    assert report['diagnostic']['propellant_balance_error_kg']==0
    result['status']='Target Achieved'
    assert not {'suborbital','descending','fuel_remaining','solver_budget'}&{f['code'] for f in analyze_consistency(scenario,result)['findings']}


def test_legacy_generic_budget_message_has_unknown_scope():
    scenario=default_scenario()
    result={'status':'Target Not Reached','optimizer':{'iterations':11,'elapsed_s':237,'messages':['Optimization budget reached']}}
    report=analyze_consistency(scenario,result)
    assert report['diagnostic']['optimizer_budget_exhausted'] is None
    assert any(f['code']=='legacy_search_limit' for f in report['findings'])
    assert not any(f['code']=='solver_budget' for f in report['findings'])
