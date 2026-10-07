from datetime import datetime, timezone
import numpy as np
import pytest
from pydantic import ValidationError

from atmosphere.dynamics import compare_target
from atmosphere.models import LaunchSite, TargetOrbit
from atmosphere.screening import azimuth_bounds, azimuth_margin, departure_options, recommend_sites


def test_partial_target_does_not_fill_omitted_constraints():
    target=TargetOrbit(perigee_altitude_m=200000)
    assert target.apogee_altitude_m is None and target.inclination_deg is None
    rows=compare_target({'perigee_altitude_m':200000,'apogee_altitude_m':35786000,'inclination_deg':98},target)
    assert len(rows)==1 and rows[0]['within_tolerance']
    with pytest.raises(ValidationError):TargetOrbit(perigee_altitude_m=None)
    assert len(compare_target({'inclination_deg':98},TargetOrbit(perigee_altitude_m=None,inclination_deg=98)))==1


def test_north_crossing_sectors_and_solver_bounds():
    assert azimuth_margin(0,[(350,10)])==10
    assert azimuth_margin(180,[(350,10)])<0
    launch=LaunchSite(azimuth_sectors_deg=[(350,10)])
    bounds,azimuth=azimuth_bounds(launch,5)
    assert bounds==(350,370) and azimuth==365
    with pytest.raises(ValidationError):LaunchSite(azimuth_sectors_deg=[(40,400)])


def test_screening_distinguishes_direct_maneuver_and_missing_range():
    result=recommend_sites(TargetOrbit(inclination_deg=98),datetime(2026,10,4,tzinfo=timezone.utc))
    sites={site['id']:site for site in result['sites']}
    assert sites['ll2-16']['status']=='Direct reference match'
    assert sites['ll2-87']['status']=='Maneuver required'
    assert sites['ll2-67']['status']=='Range data unavailable'
    assert sites['ll2-67']['azimuth_sectors_deg']==[]
    assert not sites['ll2-67']['range_verified']


def test_rotation_corrected_departure_and_plane_change():
    error,heading,inclination=departure_options(28.5,TargetOrbit(inclination_deg=51.6),[(35,120)])
    assert error==0 and inclination==51.6 and 35<=heading<=120
    inertial=np.degrees(np.arcsin(np.cos(np.radians(51.6))/np.cos(np.radians(28.5))))
    assert abs(heading-inertial)>.1
    error,_,_=departure_options(45,TargetOrbit(inclination_deg=0),[])
    assert error==pytest.approx(45)


def test_user_departure_sector_replaces_catalog_reference():
    launch=LaunchSite(launch_site_id='ll2-87',azimuth_sectors_deg=[(175,200)],range_source='User-defined departure sectors')
    result=recommend_sites(TargetOrbit(inclination_deg=98),datetime(2026,10,4,tzinfo=timezone.utc),launch)
    site=next(site for site in result['sites'] if site['id']=='ll2-87')
    assert site['azimuth_sectors_deg']==[(175,200)]
    assert site['status']=='Direct reference match'
