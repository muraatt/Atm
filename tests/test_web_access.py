import json
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from atmosphere.web_access import allowed_origins, normalize_origin, LocalEngineCORS

def test_web_origins_are_explicit_and_do_not_include_all_vercel_previews(tmp_path,monkeypatch):
    monkeypatch.delenv('ATMOSPHERE_WEB_ORIGINS',raising=False)
    (tmp_path/'data').mkdir()
    (tmp_path/'data'/'web-origins.json').write_text(json.dumps(['https://my-atm.vercel.app/']))
    origins=allowed_origins(tmp_path)
    assert 'https://my-atm.vercel.app' in origins
    assert 'https://other.vercel.app' not in origins
    assert '*' not in origins

@pytest.mark.parametrize('origin',['http://example.com','https://user:pass@example.com','https://example.com/path','https://example.com?token=x'])
def test_rejects_non_origin_configuration(origin):
    with pytest.raises(ValueError):normalize_origin(origin)

def test_local_network_preflight_requires_authorized_origin():
    app=FastAPI()
    app.add_middleware(LocalEngineCORS,allow_origins=['https://my-atm.vercel.app'],allow_methods=['GET','POST'],allow_headers=['Content-Type'])
    client=TestClient(app)
    headers={'Origin':'https://my-atm.vercel.app','Access-Control-Request-Method':'POST','Access-Control-Request-Headers':'content-type','Access-Control-Request-Private-Network':'true'}
    response=client.options('/api/analyses',headers=headers)
    assert response.status_code==200
    assert response.headers['Access-Control-Allow-Origin']=='https://my-atm.vercel.app'
    assert response.headers['Access-Control-Allow-Private-Network']=='true'
    response=client.options('/api/analyses',headers={**headers,'Origin':'https://other.vercel.app'})
    assert response.status_code==400
    assert 'Access-Control-Allow-Private-Network' not in response.headers

def test_exported_result_contains_earth_frame_without_rewriting_original(tmp_path,monkeypatch):
    from atmosphere.jobs import manager
    from atmosphere.main import app
    monkeypatch.setattr(manager,'root',tmp_path)
    folder=tmp_path/('a'*32);folder.mkdir()
    result={'scenario':{'launch':{'epoch':'2026-10-06T12:00:00Z'}},'summary':{'arrival_time_s':600},'metadata':{}}
    path=folder/'result.json';path.write_text(json.dumps(result));original=path.read_bytes()
    response=TestClient(app).get('/api/analyses/'+folder.name+'/result')
    assert response.status_code==200
    frame=response.json()['metadata']['earth_orientation']
    assert frame['epoch']=='2026-10-06T12:10:00+00:00'
    assert len(frame['itrs_to_gcrs'])==3
    assert path.read_bytes()==original
