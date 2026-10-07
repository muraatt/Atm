"""Build a compact offline globe layer from public-domain Natural Earth data."""
import json
from pathlib import Path
import urllib.request
import numpy as np
import shapely
from shapely.geometry import shape

ROOT = Path(__file__).resolve().parents[1]
SOURCE = 'https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/'


def lines(geometry):
    if geometry.geom_type in ('LineString', 'LinearRing'):
        return [[[round(x, 3), round(y, 3)] for x, y in geometry.coords]]
    if hasattr(geometry, 'geoms'):
        return [line for part in geometry.geoms for line in lines(part)]
    return []


if __name__ == '__main__':
    land_path = ROOT / 'data/terrain/natural_earth_10m_land.geojson'
    land = json.loads(land_path.read_text(encoding='utf-8'))
    # Natural Earth includes an explicit cartographic "Null island" marker at
    # zero latitude/longitude. It is not physical land and must not color the globe.
    geometry = shapely.union_all([shape(feature['geometry']) for feature in land['features']
                                  if feature['properties'].get('featurecla') != 'Null island'])
    latitudes = np.arange(-90, 91)
    longitudes = np.arange(-180, 181)
    lon, lat = np.meshgrid(longitudes, latitudes)
    mask = shapely.intersects_xy(geometry, lon, lat)
    border_path = ROOT / 'data/terrain/natural_earth_110m_borders.geojson'
    if not border_path.exists():
        request = urllib.request.Request(SOURCE + 'ne_110m_admin_0_boundary_lines_land.geojson',
                                         headers={'User-Agent': 'Atmosphere globe asset builder'})
        with urllib.request.urlopen(request, timeout=30) as response:
            border_path.write_bytes(response.read())
    borders = json.loads(border_path.read_text(encoding='utf-8'))
    payload = {'latitudeStepDeg': 1, 'longitudeStepDeg': 1,
        'landMask': [''.join('1' if point else '0' for point in row) for row in mask],
        'coastlines': lines(geometry.boundary.simplify(.15, preserve_topology=True)),
        'borders': [line for feature in borders['features']
                    for line in lines(shape(feature['geometry']).simplify(.1))],
        'source': 'Natural Earth', 'license': 'Public domain',
        'sourceUrl': 'https://www.naturalearthdata.com/',
        'landSourceUrl': SOURCE + 'ne_10m_land.geojson',
        'bordersSourceUrl': SOURCE + 'ne_110m_admin_0_boundary_lines_land.geojson'}
    output = ROOT / 'frontend/src/data/globe-map.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, separators=(',', ':')), encoding='utf-8')
    print(f'Globe asset: {output.stat().st_size:,} bytes; {mask.shape} surface grid.')
