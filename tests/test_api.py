import pytest
from fastapi.testclient import TestClient

from atmosphere.main import app
from atmosphere.models import default_scenario

client=TestClient(app)


def test_globe_orientation_aligns_geography_with_production_launch_frame():
    import numpy as np
    from datetime import datetime, timezone
    from atmosphere.earth import EarthFrames, geodetic_to_ecef
    epoch=datetime(2026,10,6,12,tzinfo=timezone.utc)
    response=client.get('/api/earth/orientation',params={'epoch':epoch.isoformat()})
    assert response.status_code==200
    matrix=np.array(response.json()['itrs_to_gcrs'])
    assert matrix@matrix.T==pytest.approx(np.eye(3),abs=1e-12)
    assert np.linalg.det(matrix)==pytest.approx(1,abs=1e-12)
    pad=geodetic_to_ecef(28.60822681,-80.60428186,0)
    assert matrix@pad==pytest.approx(EarthFrames(epoch,1800).matrix(0)@pad,abs=1e-7)
    later=client.get('/api/earth/orientation',params={'epoch':'2026-10-06T18:00:00Z'}).json()
    assert np.linalg.norm(np.array(later['itrs_to_gcrs'])@pad-matrix@pad)>7e6


@pytest.mark.parametrize('epoch',['2026-10-06T12:00:00','not-a-date'])
def test_globe_orientation_requires_an_explicit_valid_timezone(epoch):
    assert client.get('/api/earth/orientation',params={'epoch':epoch}).status_code==422


def test_health_and_scenario_validation():
    assert client.get('/api/health').json()['status']=='ok'
    scenario=client.get('/api/defaults').json()
    response=client.post('/api/scenarios/validate',json=scenario)
    assert response.status_code==200
    assert response.json()['initial_mass_kg']==305500
    scenario['vehicle']['stages'][0]['propellant_mass_kg']=-1
    assert client.post('/api/scenarios/validate',json=scenario).status_code==422


def test_analysis_requires_resolved_launch_elevation():
    assert client.post('/api/analyses',json=default_scenario().model_dump(mode='json')).status_code==422


def test_consistency_endpoint_screens_without_starting_an_analysis():
    scenario=default_scenario().model_dump(mode='json')
    scenario['vehicle']['payload_mass_kg']=35000
    response=client.post('/api/consistency',json=scenario)
    assert response.status_code==200
    assert response.json()['optimistic_margin_m_s']<0
    assert response.json()['diagnostic'] is None


def test_budget_assumptions_roundtrip_and_preview_without_vehicle_changes():
    scenario=client.get('/api/defaults').json()
    original=scenario['vehicle'].copy()
    default=client.post('/api/consistency',json=scenario).json()['mission_budget']
    scenario['budget']['contingency_percent']=20
    response=client.post('/api/consistency',json=scenario)
    assert response.status_code==200
    budget=response.json()['mission_budget']
    assert budget['tiers'][-1]['total_required_fuel_kg']>default['tiers'][-1]['total_required_fuel_kg']
    assert scenario['vehicle']==original
    validated=client.post('/api/scenarios/validate',json=scenario)
    assert validated.status_code==200
    scenario['budget']['contingency_percent']=-1
    assert client.post('/api/consistency',json=scenario).status_code==422


def test_saved_result_diagnostics_do_not_rewrite_the_original_result(tmp_path,monkeypatch):
    import json
    from atmosphere.main import manager
    scenario=default_scenario().model_dump(mode='json')
    scenario['vehicle']['payload_mass_kg']=35000
    result={'scenario':scenario,'status':'Target Not Reached','series':[],
            'orbit':{'perigee_altitude_m':-4000000},'summary':{'propellant_remaining_kg':9000},
            'optimizer':{'iterations':0,'messages':['Optimization budget reached']}}
    original=json.dumps(result)
    path=tmp_path/'result.json';path.write_text(original,encoding='utf-8')
    monkeypatch.setattr(manager,'path',lambda job_id:tmp_path)
    response=client.get('/api/analyses/'+('a'*32)+'/diagnostics')
    assert response.status_code==200
    assert {'delta_v_shortfall','legacy_search_limit','fuel_remaining'}<={f['code'] for f in response.json()['findings']}
    assert response.json()['diagnostic']['optimizer_budget_exhausted'] is None
    assert path.read_text(encoding='utf-8')==original


def test_partial_target_site_recommendations_endpoint():
    scenario=default_scenario().model_dump(mode='json')
    response=client.post('/api/launch-sites/recommendations',json={'target':{'perigee_altitude_m':None,'inclination_deg':98},'launch':scenario['launch']})
    assert response.status_code==200 and len(response.json()['sites'])==24
    assert any(site['status']=='Direct reference match' for site in response.json()['sites'])


def test_atmosphere_preview_and_missing_solar_data(monkeypatch):
    scenario=default_scenario()
    scenario.environment.model='standard'
    response=client.post('/api/atmosphere/preview',json={'launch':scenario.launch.model_dump(mode='json'),'environment':scenario.environment.model_dump()})
    assert response.status_code==200 and len(response.json()['profile'])>100
    from pymsis import utils
    def unavailable(*args,**kwargs):
        raise ValueError('Unavailable test date')
    monkeypatch.setattr(utils,'get_f107_ap',unavailable)
    scenario.environment.model='msis'
    scenario.environment.solar_mode='automatic'
    response=client.post('/api/atmosphere/preview',json={'launch':scenario.launch.model_dump(mode='json'),'environment':scenario.environment.model_dump()})
    assert response.status_code==422
    assert 'nominal' in response.json()['detail']


def test_missing_jobs_and_path_traversal():
    assert client.get('/api/analyses/not-a-job').status_code==404
    assert client.get('/api/analyses/'+('a'*32)).status_code==404
    assert client.post('/api/analyses/not-a-job/cancel').status_code==404
    assert client.get('/api/unknown').status_code==404
    for suffix in ('result','diagnostics','export.csv'):
        assert client.get('/api/analyses/'+('a'*32)+'/'+suffix).status_code==404


@pytest.mark.parametrize('lat,lon',[(91,0),(0,-181),('nan',0),(0,'inf')])
def test_invalid_elevation_coordinates_are_input_errors(lat,lon):
    assert client.get('/api/elevation',params={'latitude_deg':lat,'longitude_deg':lon,'surface':'ocean'}).status_code==422


def test_real_api_worker_lifecycle_and_exports(tmp_path,monkeypatch):
    import time
    import json
    from atmosphere.jobs import JobManager
    from atmosphere import main
    from atmosphere.engines import CATALOG
    manager=JobManager(tmp_path)
    monkeypatch.setattr(main,'manager',manager)
    scenario=default_scenario()
    scenario.environment.model='standard'
    scenario.launch.elevation_resolved=True
    scenario.vehicle.stages[0].engine_id='merlin-sl'
    scenario.vehicle.stages[0].engine_count=1
    scenario.vehicle.stages[0].engine_catalog_version=CATALOG['version']
    scenario.vehicle.stages[0].vacuum_thrust_n=950000
    scenario.vehicle.stages[0].isp_vacuum_s=311
    scenario.vehicle.stages[0].isp_sea_level_s=311*845000/950000
    scenario.vehicle.stages[0].throttle_min=.57
    scenario.vehicle.stages[0].propellant_mass_kg=2e6
    scenario.vehicle.stages[1].engine_id='be-3u'
    scenario.vehicle.stages[1].engine_count=1
    scenario.vehicle.stages[1].engine_catalog_version=CATALOG['version']
    scenario.vehicle.stages[1].vacuum_thrust_n=890000
    scenario.vehicle.stages[1].throttle_min=.5
    original=scenario.model_dump(mode='json')
    try:
        response=client.post('/api/analyses',json=original)
        assert response.status_code==202,response.text
        job=response.json()['id']
        end=time.monotonic()+25
        while time.monotonic()<end:
            status=client.get('/api/analyses/'+job).json()
            if status['state'] in ('completed','failed','cancelled'):break
            time.sleep(.1)
        assert status['state']=='completed',status
        result=client.get('/api/analyses/'+job+'/result')
        assert result.status_code==200
        assert result.json()['status']=='No Liftoff'
        assert result.json()['metadata']['verified_with_strict_integration']
        assert json.loads((tmp_path/job/'scenario.json').read_text())==original
        assert client.get('/api/analyses/'+job+'/export.csv').status_code==200
        assert client.get('/api/analyses/'+job+'/diagnostics').status_code==200
        assert client.post('/api/analyses/'+job+'/cancel').json()['state']=='completed'
    finally:
        manager.close()


def test_result_and_csv_preserve_si_state_and_null_temperature(tmp_path,monkeypatch):
    import csv
    import io
    import json
    from atmosphere.main import manager
    monkeypatch.setattr(manager,'path',lambda job_id:tmp_path)
    result={'status':'Target Not Reached','series':[
        {'time_s':12.5,'altitude_m':1200000,'temperature_k':None,
         'position_gcrs_m':[1,2,3],'velocity_gcrs_m_s':[4,5,6]}]}
    (tmp_path/'result.json').write_text(json.dumps(result),encoding='utf-8')
    assert client.get('/api/analyses/'+('a'*32)+'/result').json()==result
    response=client.get('/api/analyses/'+('a'*32)+'/export.csv')
    assert response.status_code==200 and 'attachment' in response.headers['content-disposition']
    row=list(csv.DictReader(io.StringIO(response.text)))[0]
    assert row['time_s']=='12.5' and row['altitude_m']=='1200000'
    assert row['temperature_k']==''
    assert [float(row['position_'+axis+'_m']) for axis in 'xyz']==[1,2,3]
    assert [float(row['velocity_'+axis+'_m_s']) for axis in 'xyz']==[4,5,6]
