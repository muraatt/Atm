"""Independent Kepler states exercise requested elements and singular cases."""
import numpy as np
import pytest

from atmosphere.earth import MU,R_E,targeted_elements
from atmosphere.dynamics import compare_target
from atmosphere.models import TargetOrbit


@pytest.mark.parametrize('perigee,apogee,inclination',[
    (200e3,200e3,0),(550e3,550e3,97.4),(2000e3,2000e3,30),
    (20200e3,20200e3,55),(200e3,35786e3,28.6),(200e3,40000e3,120),
    (35786e3,35786e3,0),(35786e3,35786e3,10),(200e3,800e3,180),
])
def test_orbit_acceptance_against_independently_constructed_kepler_state(perigee,apogee,inclination):
    rp,ra=R_E+perigee,R_E+apogee
    e=(ra-rp)/(ra+rp);p=2*rp*ra/(rp+ra)
    circular=perigee==apogee;equatorial=inclination in (0,180)
    raan=0 if equatorial else 359.99
    omega,anomaly=42.5,317.49
    om,inc,w,nu=np.radians([raan,inclination,omega,anomaly])
    # Explicit active rotations, independent of the implementation's orbit
    # curve and element conversion helpers.
    rz=lambda a:np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    rx=np.array([[1,0,0],[0,np.cos(inc),-np.sin(inc)],[0,np.sin(inc),np.cos(inc)]])
    rotation=rz(om)@rx@rz(w)
    r=rotation@(p/(1+e*np.cos(nu))*np.array([np.cos(nu),np.sin(nu),0]))
    v=rotation@(np.sqrt(MU/p)*np.array([-np.sin(nu),e+np.cos(nu),0]))
    phase=(omega+anomaly)%360 if circular else anomaly
    target=TargetOrbit(perigee_altitude_m=perigee,apogee_altitude_m=apogee,inclination_deg=inclination,
        raan_deg=None if equatorial else raan,argument_of_periapsis_deg=None if circular else omega,
        arrival_phase_deg=phase,altitude_tolerance_m=1,angle_tolerance_deg=.001)
    actual=targeted_elements(r,v,target)
    comparison=compare_target(actual,target)
    assert actual['bound']
    assert all(row['within_tolerance'] for row in comparison),comparison
    assert max(abs(row['error']) for row in comparison if row['parameter'].endswith('_m'))<1e-5


def test_angular_acceptance_wraps_at_zero_without_freeing_supplied_angles():
    target=TargetOrbit(perigee_altitude_m=None,inclination_deg=30,raan_deg=.01,arrival_phase_deg=359.99)
    rows=compare_target({'inclination_deg':30,'raan_deg':359.99,'arrival_phase_deg':.01},target)
    assert {row['parameter'] for row in rows}=={'inclination_deg','raan_deg','arrival_phase_deg'}
    assert all(row['within_tolerance'] for row in rows)
    rows=compare_target({'inclination_deg':30,'raan_deg':359,'arrival_phase_deg':.01},target)
    assert not next(row for row in rows if row['parameter']=='raan_deg')['within_tolerance']
