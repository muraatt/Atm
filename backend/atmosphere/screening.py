"""Preliminary geometric screening; reference envelopes are not range clearance."""
from datetime import timedelta
from pathlib import Path
import json

import numpy as np
from scipy.optimize import brentq

from .earth import EarthFrames, MU, R_E, geodetic_to_ecef

CATALOG=json.loads((Path(__file__).parent/'data'/'launch_sites.json').read_text(encoding='utf-8'))


def azimuth_margin(azimuth,sectors):
    if not sectors:
        return 180.0
    margins=[]
    for start,end in sectors:
        width=(end-start)%360
        if end-start==360:
            return 180.0
        offset=(azimuth-start)%360
        margins.append(min(offset,width-offset) if offset<=width else -min(offset-width,360-offset))
    return max(margins)


def seed_shape(target):
    # These numbers initialize a trajectory; they never become target constraints.
    perigee=target.perigee_altitude_m if target.perigee_altitude_m is not None else min(200000,target.apogee_altitude_m or 200000)
    apogee=target.apogee_altitude_m if target.apogee_altitude_m is not None else perigee
    return perigee,max(perigee,apogee)


def departure_options(latitude,target,sectors):
    rp,ra=seed_shape(target);vp=np.sqrt(MU*(2/(R_E+rp)-2/(2*R_E+rp+ra)))
    rotation=7.292115e-5*R_E*np.cos(np.radians(latitude))
    desired=target.inclination_deg
    candidates=[]
    if desired is not None:
        ratio=np.cos(np.radians(desired))/max(1e-12,np.cos(np.radians(latitude)))
        if abs(ratio)<=1+1e-10:
            inertial=np.degrees(np.arcsin(np.clip(ratio,-1,1)))
            for angle in (inertial,180-inertial):
                angle=np.radians(angle)
                azimuth=float(np.degrees(np.arctan2(vp*np.sin(angle)-rotation,vp*np.cos(angle)))%360)
                if azimuth_margin(azimuth,sectors)>=-1e-6:candidates.append((0.,azimuth,desired))
    if candidates:return min(candidates,key=lambda row:-rotation*np.sin(np.radians(row[1])))
    if sectors:
        headings=np.concatenate([np.linspace(start,end if end>start else end+360,361) for start,end in sectors])%360
    else:headings=np.linspace(0,360,721)
    for az in headings:
        # Relative speed magnitude is chosen to give the reference inertial speed.
        a=np.radians(az);vrel=-rotation*np.sin(a)+np.sqrt(max(0,vp**2-rotation**2*np.cos(a)**2))
        inclination=float(np.degrees(np.arccos(np.clip(np.cos(np.radians(latitude))*(vrel*np.sin(a)+rotation)/vp,-1,1))))
        candidates.append((abs(inclination-desired) if desired is not None else 0.,float(az),inclination))
    return min(candidates,key=lambda row:(round(row[0],8),-rotation*np.sin(np.radians(row[1]))))


def azimuth_bounds(launch,preferred):
    if not launch.azimuth_sectors_deg:return (-180.,360.),preferred
    arcs=[]
    for start,end in launch.azimuth_sectors_deg:
        end=end if end>start else end+360
        candidate=preferred+360 if preferred<start else preferred
        clamped=float(np.clip(candidate,start,end))
        arcs.append((abs(candidate-clamped),(start,end),clamped))
    _,bounds,initial=min(arcs,key=lambda row:row[0])
    return bounds,initial


def plane_windows(site,target,epoch,frames):
    if target.raan_deg is None or target.inclination_deg is None:return []
    inc,raan=np.radians([target.inclination_deg,target.raan_deg])
    normal=np.array([np.sin(inc)*np.sin(raan),-np.sin(inc)*np.cos(raan),np.cos(inc)])
    fixed=geodetic_to_ecef(site['latitude_deg'],site['longitude_deg'],0)
    def distance(t):return float(np.dot(normal,frames.matrix(t)@fixed)/R_E)
    grid=np.linspace(0,86400,145);values=[distance(t) for t in grid];roots=[]
    for left,right,a,b in zip(grid,grid[1:],values,values[1:]):
        if a*b<0:
            root=brentq(distance,left,right)
            roots.append((epoch+timedelta(seconds=root)).isoformat())
    return roots


def recommend_sites(target,epoch,launch_override=None):
    frames=EarthFrames(epoch,86400) if target.raan_deg is not None and target.inclination_deg is not None else None
    rp,ra=seed_shape(target);vp=np.sqrt(MU*(2/(R_E+rp)-2/(2*R_E+rp+ra)))
    results=[]
    for entry in CATALOG:
        site=entry.copy()
        if launch_override is not None and launch_override.launch_site_id==site['id'] and launch_override.range_source.startswith('User-defined'):
            site.update(azimuth_sectors_deg=launch_override.azimuth_sectors_deg,range_reference=launch_override.range_source,range_source_url=None)
        sectors=site['azimuth_sectors_deg']
        error,heading,insertion_inc=departure_options(site['latitude_deg'],target,sectors)
        rotation=7.292115e-5*R_E*np.cos(np.radians(site['latitude_deg']))*np.sin(np.radians(heading))
        penalty=2*vp*np.sin(np.radians(error)/2)
        verified=bool(sectors)
        status='Direct reference match' if verified and error<=target.angle_tolerance_deg else 'Maneuver required' if error>target.angle_tolerance_deg else 'Range data unavailable'
        results.append({**site,'status':status,'range_verified':verified,'recommended_azimuth_deg':heading,
            'reference_inclination_deg':insertion_inc,'minimum_plane_change_deg':error,
            'plane_change_estimate_m_s':float(penalty),'rotation_benefit_m_s':float(rotation),
            'ranking_cost_m_s':float(penalty-rotation+(0 if verified else 250)),
            'launch_windows_utc':plane_windows(site,target,epoch,frames) if frames else []})
    results.sort(key=lambda site:site['ranking_cost_m_s'])
    return {'sites':results,'note':'Preliminary ranking uses departure-sector geometry, Earth rotation and an impulsive plane-change estimate at the reference perigee speed. It is not a total launch delta-v or fuel estimate. Vehicle performance, doglegs, drop zones and current range approvals require mission-specific analysis. Unpublished envelopes remain unverified.',
            'window_note':'RAAN windows estimate launch-site plane crossings over the next 24 hours; ascent duration and plane maneuvers are handled by trajectory optimization.'}
