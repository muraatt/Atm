from __future__ import annotations

import csv
import io
import json
from datetime import datetime, timezone, timedelta
from functools import lru_cache
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, File, Form, HTTPException, UploadFile, Query
from .web_access import LocalEngineCORS, allowed_origins
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel

from .environment import EnvironmentError, preview
from .consistency import analyze_consistency
from .jobs import manager
from .engines import CATALOG, validate_design
from .models import EnvironmentConfig, LaunchSite, Scenario, TargetOrbit, default_scenario
from .screening import recommend_sites
from .terrain import ROOT, TerrainError, elevation, register_local
from .earth import EarthFrames


@asynccontextmanager
async def lifespan(app):
    yield
    manager.close()


app=FastAPI(title="Atmosphere — Surface-to-Orbit Analysis",version="0.1.0",lifespan=lifespan)
app.add_middleware(LocalEngineCORS,allow_origins=allowed_origins(),
                   allow_methods=["GET","POST"],allow_headers=["Content-Type"])


@app.get("/api/health")
def health():
    return {"status":"ok","version":"0.1.0","application":"atmosphere"}


@lru_cache(maxsize=128)
def globe_orientation(epoch):
    frames = EarthFrames(epoch, 0)
    return {'epoch': epoch.isoformat(), 'itrs_to_gcrs': frames.matrix(0).tolist(),
            'warnings': frames.warnings}


@app.get('/api/earth/orientation')
def earth_orientation(epoch: datetime):
    if epoch.tzinfo is None:
        raise HTTPException(422, 'Earth orientation time must include a UTC offset.')
    return globe_orientation(epoch.astimezone(timezone.utc))


@app.get("/api/defaults")
def defaults():
    scenario=default_scenario()
    scenario.vehicle.optimize_stage_masses=True
    return scenario.model_dump(mode="json")


@app.get("/api/engines")
def engine_catalog():
    return CATALOG


@app.get("/api/elevation")
def get_elevation(latitude_deg:float=Query(ge=-90,le=90,allow_inf_nan=False),longitude_deg:float=Query(ge=-180,le=180,allow_inf_nan=False),surface:str="auto"):
    if surface not in ("auto","land","ocean"):
        raise HTTPException(422,"Surface must be auto, land, or ocean.")
    try:
        return elevation(latitude_deg,longitude_deg,surface)
    except TerrainError as exc:
        raise HTTPException(503,str(exc)) from exc


@app.post("/api/terrain/upload")
def upload_terrain(file:UploadFile=File(...),kind:str=Form(...)):
    if kind not in ("surface","geoid"):
        raise HTTPException(422,"Terrain kind must be surface or geoid.")
    content=file.file.read(200*1024*1024+1)
    if len(content)>200*1024*1024:
        raise HTTPException(413,"The local terrain upload limit is 200 MB per file.")
    try:
        return register_local(content,file.filename or "terrain.tif",kind)
    except TerrainError as exc:
        raise HTTPException(422,str(exc)) from exc


class PreviewRequest(BaseModel):
    launch:LaunchSite
    environment:EnvironmentConfig


@app.post("/api/atmosphere/preview")
def atmosphere_preview(request:PreviewRequest):
    try:
        return preview(request.environment,request.launch)
    except EnvironmentError as exc:
        raise HTTPException(422,str(exc)) from exc
    except Exception as exc:
        raise HTTPException(503,f"Atmosphere model could not be evaluated: {exc}") from exc


@app.post("/api/scenarios/validate")
def validate_scenario(scenario:Scenario):
    try:
        validate_design(scenario)
    except ValueError as exc:
        raise HTTPException(422,str(exc)) from exc
    return {"valid":True,"initial_mass_kg":scenario.vehicle.initial_mass_kg,"scenario":scenario.model_dump(mode="json")}


@app.post("/api/consistency")
def consistency_report(scenario:Scenario):
    return analyze_consistency(scenario)


class ScreeningRequest(BaseModel):
    target:TargetOrbit
    launch:LaunchSite


@app.post("/api/launch-sites/recommendations")
def launch_recommendations(request:ScreeningRequest):
    return recommend_sites(request.target,request.launch.epoch,request.launch)


@app.post("/api/analyses",status_code=202)
def start_analysis(scenario:Scenario):
    try:
        validate_design(scenario,require_engines=True)
    except ValueError as exc:
        raise HTTPException(422,str(exc)) from exc
    if not scenario.launch.elevation_resolved:
        raise HTTPException(422,"Resolve terrain elevation or explicitly accept a manual launch elevation before analysis.")
    try:
        return manager.start(scenario)
    except ValueError as exc:
        raise HTTPException(409,str(exc)) from exc


@app.get("/api/analyses/{job_id}")
def analysis_status(job_id:str):
    try:
        return manager.status(job_id)
    except FileNotFoundError as exc:
        raise HTTPException(404,str(exc)) from exc


@app.post("/api/analyses/{job_id}/cancel")
def cancel_analysis(job_id:str):
    try:
        return manager.cancel(job_id)
    except FileNotFoundError as exc:
        raise HTTPException(404,str(exc)) from exc


@app.get("/api/analyses/{job_id}/result")
def analysis_result(job_id:str):
    try:
        path=manager.path(job_id)/"result.json"
        if not path.exists():
            manager.status(job_id)
            raise HTTPException(409,"The result is not ready.")
        result=json.loads(path.read_text(encoding="utf-8"))
        if 'epoch' in result.get('scenario',{}).get('launch',{}) and 'arrival_time_s' in result.get('summary',{}):
            epoch=datetime.fromisoformat(result['scenario']['launch']['epoch'].replace('Z','+00:00'))+timedelta(seconds=result['summary']['arrival_time_s'])
            result.setdefault('metadata',{})['earth_orientation']=globe_orientation(epoch.astimezone(timezone.utc))
        return result
    except FileNotFoundError as exc:
        raise HTTPException(404,str(exc)) from exc


@app.get("/api/analyses/{job_id}/diagnostics")
def analysis_diagnostics(job_id:str):
    result=analysis_result(job_id)
    return analyze_consistency(Scenario.model_validate(result['scenario']),result)


@app.get("/api/analyses/{job_id}/export.csv")
def export_series(job_id:str):
    result=analysis_result(job_id)
    rows=result["series"]
    columns=[k for k in rows[0] if k not in ("position_gcrs_m","velocity_gcrs_m_s")] if rows else []
    columns+=['position_x_m','position_y_m','position_z_m','velocity_x_m_s','velocity_y_m_s','velocity_z_m_s']
    stream=io.StringIO(newline="")
    writer=csv.DictWriter(stream,fieldnames=columns)
    writer.writeheader()
    for row in rows:
        record={k:v for k,v in row.items() if k not in ("position_gcrs_m","velocity_gcrs_m_s")}
        record.update(zip(columns[-6:],row["position_gcrs_m"]+row["velocity_gcrs_m_s"]))
        writer.writerow(record)
    return Response(stream.getvalue(),media_type="text/csv",headers={"Content-Disposition":'attachment; filename="atmosphere-timeseries.csv"'})


# The production bundle is served by the same loopback server as the API.
DIST=ROOT/"frontend"/"dist"
@app.get("/{path:path}",include_in_schema=False)
def frontend(path:str):
    if path.startswith("api/"):
        raise HTTPException(404,"API endpoint not found.")
    requested=(DIST/path).resolve()
    if not requested.is_relative_to(DIST.resolve()):
        raise HTTPException(404,"File not found.")
    if requested.is_file():
        return FileResponse(requested)
    if (DIST/"index.html").exists():
        return FileResponse(DIST/"index.html")
    raise HTTPException(503,"Build the frontend or start the Vite development server.")
