from __future__ import annotations

import json
import math
import re
import threading
from functools import lru_cache
from pathlib import Path
from uuid import uuid4

import httpx
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "data" / "terrain"
NOAA = "https://www.ngdc.noaa.gov/thredds/dodsC/global/ETOPO2022/15s"
LAND_URL = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_land.geojson"
_lock = threading.RLock()


class TerrainError(ValueError):
    pass


def tile_name(lat: float, lon: float, kind: str) -> tuple[str, int, int]:
    north = min(90, (math.floor(lat/15)+1)*15)
    west = math.floor(((lon+180)%360-180)/15)*15
    name = f"ETOPO_2022_v1_15s_{'N' if north >= 0 else 'S'}{abs(north):02d}{'E' if west >= 0 else 'W'}{abs(west):03d}_{kind}.nc"
    # NetCDF latitude is ascending, even though its GDAL GeoTransform is north-up.
    row = min(3599, max(0, int((lat-(north-15))*240)))
    col = min(3599, max(0, int((((lon+180)%360-180)-west)*240)))
    return name, row, col


def _remote_cell(lat: float, lon: float, kind: str) -> float:
    DATA.mkdir(parents=True, exist_ok=True)
    name, row, col = tile_name(lat, lon, kind)
    cache = DATA / "cells" / f"{name}_{row}_{col}.json"
    if cache.exists():
        return float(json.loads(cache.read_text())["value"])
    folder = "15s_surface_elev_netcdf" if kind == "surface" else "15s_geoid_netcdf"
    url = f"{NOAA}/{folder}/{name}.ascii?z[{row}][{col}]"
    try:
        response = httpx.get(url, timeout=45, follow_redirects=True)
        response.raise_for_status()
        values = re.findall(r"^\[\d+\],\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)", response.text, flags=re.M)
        if not values:
            # Some DAP versions omit the row label for a one-cell Grid.
            match = re.search(r"z\.z(?:\[\d+\]){2}\s*\n(?:\[\d+\],\s*)?(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)", response.text)
            if not match:
                raise ValueError("Unrecognized NOAA response")
            values = [match.group(1)]
        value = float(values[0])
        if not np.isfinite(value) or value <= -99990:
            raise ValueError("Missing terrain sample")
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps({"value": value, "source": url}), encoding="utf-8")
        return value
    except Exception as exc:
        raise TerrainError("ETOPO elevation/geoid data could not be retrieved. Retry, upload local ETOPO files, or explicitly use a manual elevation.") from exc


@lru_cache(maxsize=1)
def _land_geometry():
    from shapely import from_geojson, prepare
    from shapely.ops import unary_union
    path = DATA / "natural_earth_10m_land.geojson"
    DATA.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        response = httpx.get(LAND_URL, timeout=90, follow_redirects=True)
        response.raise_for_status()
        # Parse before committing a download to the cache.
        json.loads(response.text)
        path.write_text(response.text, encoding="utf-8")
    features = json.loads(path.read_text(encoding="utf-8"))["features"]
    geom = unary_union([from_geojson(json.dumps(f["geometry"])) for f in features])
    prepare(geom)
    return geom


def _local_sample(lat, lon, kind):
    import rasterio
    from rasterio.warp import transform
    registry = DATA / "local.json"
    if not registry.exists():
        return None
    for item in reversed(json.loads(registry.read_text())):
        if item["kind"] != kind:
            continue
        try:
            with rasterio.open(DATA / item["file"]) as src:
                x, y = transform("EPSG:4326", src.crs, [lon], [lat])
                if not (src.bounds.left <= x[0] <= src.bounds.right and src.bounds.bottom <= y[0] <= src.bounds.top):
                    continue
                value = next(src.sample([(x[0],y[0])], masked=True))[0]
                if np.ma.is_masked(value) or not np.isfinite(value):
                    continue
                return float(value), f"Local {item['name']}", abs(src.res[0])*3600 if src.crs.is_geographic else None
        except Exception:
            continue
    return None


def register_local(content: bytes, name: str, kind: str) -> dict:
    import rasterio
    DATA.mkdir(parents=True, exist_ok=True)
    suffix = Path(name).suffix.lower()
    if suffix not in (".tif", ".tiff", ".nc"):
        raise TerrainError("Upload a GeoTIFF or NetCDF ETOPO surface/geoid file.")
    filename = f"{uuid4().hex}{suffix}"
    path = DATA / filename
    path.write_bytes(content)
    try:
        with rasterio.open(path) as src:
            if src.crs is None or src.count != 1:
                raise TerrainError("Terrain files must have one band and a geographic coordinate reference.")
            bounds = list(src.bounds)
    except Exception as exc:
        path.unlink(missing_ok=True)
        raise TerrainError("The uploaded file could not be read as a georeferenced single-band raster.") from exc
    entry = {"file": filename, "name": Path(name).name, "kind": kind, "bounds": bounds}
    with _lock:
        registry = DATA / "local.json"
        items = json.loads(registry.read_text()) if registry.exists() else []
        registry.write_text(json.dumps(items+[entry]), encoding="utf-8")
    return entry


def elevation(lat: float, lon: float, surface: str = "auto") -> dict:
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise TerrainError("Latitude / longitude are outside their valid ranges.")
    try:
        from shapely.geometry import Point
        land = bool(_land_geometry().covers(Point(lon, lat))) if surface == "auto" else surface == "land"
    except Exception as exc:
        raise TerrainError("Land/ocean classification is unavailable. Choose Land or Ocean explicitly and retry.") from exc
    local_z = _local_sample(lat, lon, "surface") if land else None
    local_n = _local_sample(lat, lon, "geoid")
    z = local_z[0] if local_z else (_remote_cell(lat, lon, "surface") if land else 0.0)
    geoid = local_n[0] if local_n else _remote_cell(lat, lon, "geoid")
    return {"latitude_deg": lat, "longitude_deg": lon, "elevation_m": z,
            "geoid_height_m": geoid, "ellipsoid_height_m": z+geoid,
            "surface": "land" if land else "ocean", "elevation_resolved": True,
            "elevation_source": local_z[1] if local_z else ("NOAA ETOPO 2022 surface + EGM2008 geoid" if land else "Ocean surface (0 m EGM2008) + ETOPO geoid"),
            "resolution_arcsec": local_z[2] if local_z else 15,
            "classification_source": "Natural Earth 1:10m land mask; shoreline classification can be overridden"}
