import copy
import pytest
from fastapi.testclient import TestClient
from atmosphere.main import app
from atmosphere.models import default_scenario, Scenario
from atmosphere.engines import CATALOG, validate_design

client=TestClient(app)


def selected():
    scenario=default_scenario()
    stage=scenario.vehicle.stages[1]
    stage.engine_id='rutherford-vac';stage.engine_count=2;stage.engine_catalog_version=CATALOG['version']
    stage.vacuum_thrust_n=51600;stage.isp_vacuum_s=343;stage.isp_sea_level_s=200
    return scenario


def test_engine_catalog_and_snapshot_roundtrip():
    response=client.get('/api/engines')
    assert response.status_code==200 and len(response.json()['engines'])==8
    scenario=selected()
    validate_design(scenario)
    checked=client.post('/api/scenarios/validate',json=scenario.model_dump(mode='json'))
    assert checked.status_code==200
    stage=checked.json()['scenario']['vehicle']['stages'][1]
    assert stage['engine_id']=='rutherford-vac' and stage['engine_count']==2
    assert stage['vacuum_thrust_n']==51600 and stage['dry_mass_kg']==3500
    assert scenario.vehicle.stages[1].mass_flow_kg_s==pytest.approx(2*25800/(9.80665*343))


@pytest.mark.parametrize('field,value',[('vacuum_thrust_n',25800),('isp_vacuum_s',350),('engine_id','missing-engine'),('engine_count',1.5)])
def test_catalog_mismatch_rejected(field,value):
    data=selected().model_dump(mode='json');data['vehicle']['stages'][1][field]=value
    assert client.post('/api/scenarios/validate',json=data).status_code==422


def test_two_stage_limit_enforced_without_losing_historical_reads():
    data=default_scenario().model_dump(mode='json')
    data['vehicle']['stages'].append(copy.deepcopy(data['vehicle']['stages'][1]))
    original=copy.deepcopy(data)
    assert len(Scenario.model_validate(data).vehicle.stages)==3
    for endpoint in ('/api/scenarios/validate','/api/analyses'):
        response=client.post(endpoint,json=data)
        assert response.status_code==422 and 'at most two' in response.json()['detail']
    assert client.post('/api/consistency',json=data).status_code==200
    assert data==original


def test_upper_engine_cannot_be_assigned_to_booster():
    data=selected().model_dump(mode='json')
    data['vehicle']['stages'].reverse()
    assert client.post('/api/scenarios/validate',json=data).status_code==422


def test_legacy_inputs_can_be_loaded_but_new_run_needs_engine_selection():
    data=default_scenario().model_dump(mode='json')
    assert client.post('/api/scenarios/validate',json=data).status_code==200
    response=client.post('/api/analyses',json=data)
    assert response.status_code==422 and 'Select a catalog engine' in response.json()['detail']


def test_published_booster_sea_level_thrust_is_reproduced():
    scenario=default_scenario();stage=scenario.vehicle.stages[0]
    stage.engine_id='merlin-sl';stage.engine_count=9
    stage.vacuum_thrust_n=9*950000;stage.isp_vacuum_s=311
    stage.isp_sea_level_s=311*845000/950000
    stage.throttle_min=.57
    validate_design(scenario)
    stage.isp_sea_level_s=200
    with pytest.raises(ValueError,match='sea-level thrust'):
        validate_design(scenario)


def test_merlin_vacuum_rating_is_checked_by_api():
    scenario=default_scenario();stage=scenario.vehicle.stages[1]
    stage.engine_id='merlin-vac';stage.engine_count=2
    stage.vacuum_thrust_n=2*981000;stage.isp_vacuum_s=348;stage.isp_sea_level_s=200
    stage.throttle_min=140679/220500
    validate_design(scenario)
    stage.vacuum_thrust_n=2*800000
    with pytest.raises(ValueError,match='catalog per-engine'):
        validate_design(scenario)


def test_unspecified_isp_is_context_only_not_a_vacuum_rating():
    engine=next(e for e in CATALOG['engines'] if e['id']=='rutherford-sl')
    assert engine['isp_unspecified_s']==311 and engine['isp_vacuum_s'] is None
    scenario=default_scenario();stage=scenario.vehicle.stages[0]
    stage.engine_id='rutherford-sl';stage.engine_count=9
    stage.vacuum_thrust_n=9*25000;stage.isp_vacuum_s=330
    stage.isp_sea_level_s=330*engine['sea_level_thrust_n']/25000
    validate_design(scenario)
    assert stage.mass_flow_kg_s==pytest.approx(9*25000/(9.80665*330))


def test_published_throttle_floor_is_enforced():
    scenario=default_scenario();stage=scenario.vehicle.stages[0]
    stage.engine_id='merlin-sl';stage.engine_count=9
    stage.vacuum_thrust_n=9*950000;stage.isp_vacuum_s=311
    stage.isp_sea_level_s=311*845000/950000
    stage.throttle_min=.57
    validate_design(scenario)
    stage.throttle_min=.3
    with pytest.raises(ValueError,match='minimum throttle'):
        validate_design(scenario)
