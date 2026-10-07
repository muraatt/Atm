import json

import numpy as np
import pytest
import rasterio
from rasterio.transform import from_origin

from atmosphere import terrain


def test_local_rasters_preserve_negative_land_and_convert_geoid(tmp_path,monkeypatch):
    monkeypatch.setattr(terrain,'DATA',tmp_path)
    for kind,value in [('surface',-420),('geoid',18)]:
        path=tmp_path/f'{kind}.tif'
        with rasterio.open(path,'w',driver='GTiff',height=2,width=2,count=1,dtype='float32',crs='EPSG:4326',transform=from_origin(35,32,.5,.5)) as dst:
            dst.write(np.full((2,2),value,dtype='float32'),1)
        terrain.register_local(path.read_bytes(),path.name,kind)
    land=terrain.elevation(31.5,35.5,'land')
    ocean=terrain.elevation(31.5,35.5,'ocean')
    assert land['elevation_m']==-420 and land['ellipsoid_height_m']==-402
    assert ocean['elevation_m']==0 and ocean['ellipsoid_height_m']==18


def test_remote_missing_data_is_explicit(tmp_path,monkeypatch):
    monkeypatch.setattr(terrain,'DATA',tmp_path)
    def unavailable(*args,**kwargs):
        raise ValueError('Offline test')
    monkeypatch.setattr(terrain.httpx,'get',unavailable)
    with pytest.raises(terrain.TerrainError,match='could not be retrieved'):
        terrain.elevation(0,0,'land')


def test_terrain_upload_rejects_non_raster(tmp_path,monkeypatch):
    monkeypatch.setattr(terrain,'DATA',tmp_path)
    with pytest.raises(terrain.TerrainError):
        terrain.register_local(b'not a raster','bad.tif','surface')
    assert not list(tmp_path.glob('*.tif'))
