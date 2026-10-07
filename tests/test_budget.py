import copy
import json
import math

import pytest
from pydantic import ValidationError
from atmosphere.budget import inverse_fuel, mission_budget, size_reference
from atmosphere.models import BudgetAssumptions, TargetOrbit, default_scenario
from atmosphere.earth import G0


def test_inverse_fuel_matches_rocket_equation():
    fuel = inverse_fuel(10000,1477.2819118954533,370)
    assert fuel == pytest.approx(5025.105107219476)
    assert G0*370*math.log((10000+fuel)/10000) == pytest.approx(1477.2819118954533)


def test_backward_sizing_carries_upper_fuel_and_protects_arrival_reserve():
    s = default_scenario()
    sized = size_reference(s,5000,8000,1500,300)
    rows = sized['stages'];last = rows[-1]
    final_dry = 5000+s.vehicle.stages[-1].dry_mass_kg
    reserve = inverse_fuel(final_dry,300,360)
    assert last['protected_reserve_kg'] == pytest.approx(reserve)
    assert last['arrival_fuel_kg'] == pytest.approx(inverse_fuel(final_dry+reserve,1500,360))
    assert last['ascent_fuel_kg'] == pytest.approx(inverse_fuel(final_dry+last['fuel_after_ascent_kg'],last['ascent_delta_v_m_s'],360))
    booster_final = 5000+sum(x.dry_mass_kg for x in s.vehicle.stages)+last['required_fuel_kg']
    assert rows[0]['ascent_fuel_kg'] == pytest.approx(inverse_fuel(booster_final,rows[0]['ascent_delta_v_m_s'],320))
    assert sized['total_required_fuel_kg'] == pytest.approx(sum(x['ascent_fuel_kg']+x['arrival_fuel_kg']+x['protected_reserve_kg'] for x in rows))


def test_losses_and_margin_increase_fuel_and_live_inputs_are_immutable():
    s=default_scenario();before=copy.deepcopy(s.model_dump())
    report=mission_budget(s)
    assert s.model_dump() == before
    fuel=[x['total_required_fuel_kg'] for x in report['tiers']]
    assert fuel[0]<fuel[1]<fuel[2]
    assert report['tiers'][-1]['protected_reserve_kg']>0
    sweep=[x['total_required_fuel_kg'] for x in report['payload_sweep']]
    assert sweep == sorted(sweep)
    s.budget=BudgetAssumptions(gravity_loss_m_s=0,drag_loss_m_s=0,steering_loss_m_s=0,contingency_percent=0)
    tiers=mission_budget(s)['tiers']
    assert all(t['total_required_fuel_kg']==pytest.approx(tiers[0]['total_required_fuel_kg']) for t in tiers)


def test_geo_prices_plane_change_without_double_counting():
    s=default_scenario();s.launch.latitude_deg=38;s.launch.azimuth_sectors_deg=[]
    s.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,inclination_deg=38)
    same_plane=mission_budget(s)
    s.target.inclination_deg=0
    geo=mission_budget(s)
    assert geo['reference']['plane_change_deg']==pytest.approx(38)
    assert geo['components'][1]['delta_v_m_s']>geo['coplanar_arrival_delta_v_m_s']
    assert geo['tiers'][1]['total_required_fuel_kg']>same_plane['tiers'][1]['total_required_fuel_kg']
    assert geo['tiers'][-1]['delta_v_m_s']==pytest.approx(sum(x['delta_v_m_s'] for x in geo['components']))


def test_partial_targets_and_unpriced_angles_are_explicit():
    s=default_scenario();s.target=TargetOrbit(perigee_altitude_m=None,inclination_deg=50,raan_deg=20,arrival_time_max_s=600)
    b=mission_budget(s)
    assert b['reference']['perigee_altitude_m']==100000
    assert s.target.perigee_altitude_m is None
    assert any('Advanced angles' in x for x in b['warnings'])
    assert any('Free apses' in x for x in b['warnings'])


def test_fixed_solids_ignition_and_local_shortfalls_are_reported():
    s=default_scenario();s.vehicle.stages[-1].engine_type='solid';s.vehicle.stages[-1].ignition_limit=1
    s.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,inclination_deg=0)
    s.vehicle.stages[-1].propellant_mass_kg=100
    b=mission_budget(s)
    assert not b['tiers'][-1]['stages'][-1]['resizable']
    assert any('Solid propellant' in x for x in b['warnings'])
    assert any('two ignitions' in x for x in b['warnings'])
    assert b['tiers'][-1]['stage_shortfall_kg']>0


def test_extreme_isp_produces_unavailable_finite_json():
    s=default_scenario()
    for stage in s.vehicle.stages:stage.isp_sea_level_s=stage.isp_vacuum_s=.001
    b=mission_budget(s)
    assert not b['tiers'][-1]['available']
    json.dumps(b,allow_nan=False)


@pytest.mark.parametrize('values',[{'gravity_loss_m_s':-1},{'contingency_percent':101},{'drag_loss_m_s':float('nan')}])
def test_invalid_planning_assumptions_are_rejected(values):
    with pytest.raises(ValidationError):BudgetAssumptions(**values)


def test_gto_onward_budget_does_not_invent_propellant():
    s=default_scenario();s.vehicle.payload_mass_kg=5000;s.vehicle.stages[-1].dry_mass_kg=5000;s.vehicle.stages[-1].isp_vacuum_s=370
    saved={'orbit':{'bound':True,'apogee_altitude_m':35784392.31583071,'semi_major_axis_m':24370293.349043693,'inclination_deg':38.03156563940657},
           'series':[{'stage_index':1,'active_propellant_kg':337.8144621465765}], 'summary':{'final_mass_kg':10337.814462146576}}
    before=copy.deepcopy(saved)
    b=mission_budget(s,saved)
    onward=b['onward_geo']
    assert onward['remaining_fuel_kg']==pytest.approx(337.8144621465765)
    assert onward['remaining_delta_v_m_s']==pytest.approx(120.5497454)
    assert onward['coplanar_fuel_kg']==pytest.approx(5025.1051072)
    assert onward['aligned_node_geo_fuel_kg']>onward['coplanar_fuel_kg']
    assert saved==before
