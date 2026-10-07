from __future__ import annotations

import warnings
from datetime import datetime

import astropy.units as u
import numpy as np
from astropy.coordinates import CartesianRepresentation, EarthLocation, GCRS, ITRS
from astropy.time import Time
from astropy.utils import iers
from scipy.spatial.transform import Rotation, Slerp

MU = 3.986004418e14
R_E = 6378137.0
FLATTENING = 1 / 298.257223563
J2 = 1.08262668e-3
OMEGA = 7.2921150e-5
G0 = 9.80665
E2 = FLATTENING * (2 - FLATTENING)
V_SCALE = np.sqrt(MU / R_E)

# Bundled IERS values make runs reproducible and avoid downloads inside a solve.
iers.conf.auto_download = False
iers.conf.auto_max_age = None
iers.conf.iers_degraded_accuracy = "warn"


def geodetic_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> np.ndarray:
    lat, lon = np.radians([lat_deg, lon_deg])
    n = R_E / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.array([(n + height_m) * np.cos(lat) * np.cos(lon),
                     (n + height_m) * np.cos(lat) * np.sin(lon),
                     (n * (1 - E2) + height_m) * np.sin(lat)])


def ecef_to_geodetic(r: np.ndarray) -> tuple[float, float, float]:
    x, y, z = r
    p = np.hypot(x, y)
    if p < 1e-6:
        return float(np.sign(z) * 90), 0.0, float(abs(z) - R_E * (1 - FLATTENING))
    lat = np.arctan2(z, p * (1 - E2))
    for _ in range(5):
        n = R_E / np.sqrt(1 - E2 * np.sin(lat) ** 2)
        lat = np.arctan2(z + E2 * n * np.sin(lat), p)
    n = R_E / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    height = p * np.cos(lat) + z * np.sin(lat) - n * (1 - E2 * np.sin(lat) ** 2)
    return float(np.degrees(lat)), float(np.degrees(np.arctan2(y, x))), float(height)


def unit(x: np.ndarray) -> np.ndarray:
    return x / max(np.linalg.norm(x), 1e-15)


class EarthFrames:
    """Interpolated Astropy ITRS -> GCRS rotations; no Coriolis in GCRS dynamics."""

    def __init__(self, epoch: datetime, duration_s: float):
        self.epoch = epoch
        # A 60-second rotation table is much cheaper than per-RHS Astropy transforms.
        times = np.linspace(0, duration_s + 60, max(2, int(duration_s / 60) + 2))
        epochs = Time(epoch) + times * u.s
        matrices = np.empty((len(times), 3, 3))
        self.warnings: list[str] = []
        with warnings.catch_warnings(record=True) as caught:
            for axis in range(3):
                xyz = np.zeros((3, len(times)))
                xyz[axis] = 1
                transformed = ITRS(CartesianRepresentation(xyz * u.m), obstime=epochs).transform_to(GCRS(obstime=epochs))
                matrices[:, :, axis] = transformed.cartesian.xyz.to_value(u.m).T
        if caught:
            self.warnings = ["Earth orientation uses bundled IERS data; extrapolation may reduce time/frame accuracy."]
        self._slerp = Slerp(times, Rotation.from_matrix(matrices))
        rotations=Rotation.from_matrix(matrices)
        self._quaternions=rotations.as_quat()
        for i in range(1,len(times)):
            if np.dot(self._quaternions[i-1],self._quaternions[i])<0:
                self._quaternions[i]*=-1
        self._step=times[1]-times[0]
        self._angular=((rotations[1:]*rotations[:-1].inv()).as_rotvec()/self._step)
        self._last_t = None
        self._last_matrix = None

    def matrix(self, t: float) -> np.ndarray:
        if not np.isfinite(t):
            raise ValueError("Earth-frame time must be finite.")
        t = max(0.0, float(t))
        if t != self._last_t:
            index=min(len(self._quaternions)-2,int(t/self._step))
            fraction=(t-index*self._step)/self._step
            q=(1-fraction)*self._quaternions[index]+fraction*self._quaternions[index+1]
            self._last_matrix = Rotation.from_quat(q/np.linalg.norm(q)).as_matrix()
            self._last_t = t
        return self._last_matrix

    def earth_velocity(self, t: float, r: np.ndarray) -> np.ndarray:
        if not np.isfinite(t) or not np.all(np.isfinite(r)):
            raise ValueError("Earth-frame time and position must be finite.")
        index=min(len(self._angular)-1,max(0,int(t/self._step)))
        return np.cross(self._angular[index],r)

    def initial_state(self, launch) -> np.ndarray:
        fixed = geodetic_to_ecef(launch.latitude_deg, launch.longitude_deg,
                                 launch.elevation_m + launch.geoid_height_m)
        r = self.matrix(0) @ fixed
        v = self.earth_velocity(0, r)
        return np.r_[r, v]

    def geographic(self, t: float, r: np.ndarray) -> tuple[float, float, float]:
        return ecef_to_geodetic(self.matrix(t).T @ r)


def gravity(r: np.ndarray, rotation: np.ndarray | None = None, include_j2: bool = True) -> np.ndarray:
    radius = np.linalg.norm(r)
    a = -MU * r / radius ** 3
    if include_j2:
        fixed = r if rotation is None else rotation.T @ r
        z2 = (fixed[2] / radius) ** 2
        perturbation = 1.5 * J2 * MU * R_E ** 2 / radius ** 5 * fixed * np.array([5*z2-1, 5*z2-1, 5*z2-3])
        a += perturbation if rotation is None else rotation @ perturbation
    return a


def wrap_deg(x: float) -> float:
    return float((x + 180) % 360 - 180)


def orbital_elements(r: np.ndarray, v: np.ndarray) -> dict:
    radius, speed = np.linalg.norm(r), np.linalg.norm(v)
    h = np.cross(r, v)
    hn = np.linalg.norm(h)
    if hn < 1e-6:
        return {"bound": False, "eccentricity": 1.0, "perigee_altitude_m": -R_E,
                "apogee_altitude_m": None, "inclination_deg": 0.0,
                "raan_deg": None, "argument_of_periapsis_deg": None,
                "arrival_phase_deg": None, "phase_definition": "Undefined", "semi_major_axis_m": None}
    normal = h / hn
    evec = np.cross(v, h) / MU - r / radius
    ecc = np.linalg.norm(evec)
    energy = speed**2 / 2 - MU / radius
    a = -MU / (2 * energy) if abs(energy) > 1e-10 else None
    p = hn ** 2 / MU
    inc = np.degrees(np.arccos(np.clip(normal[2], -1, 1)))
    node = np.cross([0, 0, 1], h)
    equatorial = np.linalg.norm(node) / hn < 1e-8
    circular = ecc < 1e-7
    def angle(start, end):
        start, end = unit(start), unit(end)
        return float(np.degrees(np.arctan2(np.dot(np.cross(start, end), normal), np.dot(start, end))) % 360)
    raan = None if equatorial else float(np.degrees(np.arctan2(node[1], node[0])) % 360)
    arg = None if circular else angle(np.array([1, 0, 0]) if equatorial else node, evec)
    phase = angle((np.array([1, 0, 0]) if equatorial else node) if circular else evec, r)
    return {"bound": bool(energy < 0 and ecc < 1), "semi_major_axis_m": a,
            "eccentricity": float(ecc), "perigee_altitude_m": float(p/(1+ecc)-R_E),
            "apogee_altitude_m": float(p/(1-ecc)-R_E) if energy < 0 and ecc < 1 else None,
            "inclination_deg": float(inc), "raan_deg": raan, "argument_of_periapsis_deg": arg,
            "arrival_phase_deg": phase,
            "phase_definition": "True longitude" if circular and equatorial else ("Argument of latitude" if circular else "True anomaly")}


def orbit_curve(elements: dict, samples: int = 240) -> list[list[float]]:
    if not elements.get("bound"):
        return []
    a, ecc = elements["semi_major_axis_m"], elements["eccentricity"]
    inc, raan, arg = np.radians([elements["inclination_deg"], elements.get("raan_deg") or 0,
                                elements.get("argument_of_periapsis_deg") or 0])
    transform = Rotation.from_euler("ZXZ", [raan, inc, arg]).as_matrix()
    nu = np.linspace(0, 2*np.pi, samples)
    radius = a * (1-ecc**2) / (1+ecc*np.cos(nu))
    return (transform @ np.array([radius*np.cos(nu), radius*np.sin(nu), np.zeros(samples)])).T.tolist()


def targeted_elements(r: np.ndarray, v: np.ndarray, target) -> dict:
    elements=orbital_elements(r,v)
    h=unit(np.cross(r,v))
    if np.linalg.norm(h)<1e-8:
        return elements
    circular=(target.apogee_altitude_m is not None and target.perigee_altitude_m is not None and abs(target.apogee_altitude_m-target.perigee_altitude_m)<1e-3) or elements["eccentricity"]<1e-7
    equatorial=(target.inclination_deg is not None and min(target.inclination_deg,180-target.inclination_deg)<1e-6) or min(elements["inclination_deg"],180-elements["inclination_deg"])<1e-6
    def angle(start,end):
        start,end=unit(start),unit(end)
        return float(np.degrees(np.arctan2(np.dot(np.cross(start,end),h),np.dot(start,end)))%360)
    if circular:
        reference=np.array([1,0,0]) if equatorial else np.cross([0,0,1],h)
        elements['arrival_phase_deg']=angle(reference,r)
        elements['phase_definition']='True longitude' if equatorial else 'Argument of latitude'
    elif equatorial:
        evec=np.cross(v,np.cross(r,v))/MU-unit(r)
        elements['argument_of_periapsis_deg']=angle(np.array([1,0,0]),evec)
    return elements


def target_curve(target, actual: dict) -> list[list[float]]:
    rp,ra=R_E+(target.perigee_altitude_m if target.perigee_altitude_m is not None else max(100000,actual.get("perigee_altitude_m") or 200000)),R_E+(target.apogee_altitude_m if target.apogee_altitude_m is not None else max(100000,actual.get("apogee_altitude_m") or target.perigee_altitude_m or 200000))
    ra=max(rp,ra)
    return orbit_curve({"bound": True, "semi_major_axis_m": (rp+ra)/2, "eccentricity": (ra-rp)/(ra+rp),
                        "inclination_deg": target.inclination_deg if target.inclination_deg is not None else actual.get("inclination_deg",0),
                        "raan_deg": target.raan_deg if target.raan_deg is not None else actual.get("raan_deg"),
                        "argument_of_periapsis_deg": target.argument_of_periapsis_deg if target.argument_of_periapsis_deg is not None else actual.get("argument_of_periapsis_deg")})
