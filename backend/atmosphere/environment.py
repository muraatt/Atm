from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from typing import Protocol

import numpy as np
from scipy.interpolate import PchipInterpolator

from .models import EnvironmentConfig, LaunchSite


class EnvironmentError(ValueError):
    pass


def validate_query(latitude, longitude, height, elapsed):
    if not np.all(np.isfinite([latitude, longitude, height, elapsed])):
        raise EnvironmentError("Atmosphere coordinates, height and time must be finite.")
    if height < -1000:
        raise EnvironmentError("Atmosphere query is below the supported surface-height domain (-1,000 m).")


@dataclass
class AtmosphereState:
    density_kg_m3: float
    temperature_k: float | None
    pressure_pa: float
    sound_speed_m_s: float | None
    wind_ecef_m_s: np.ndarray


class AtmosphereProvider(Protocol):
    metadata: dict
    def sample(self, latitude_deg: float, longitude_deg: float, height_m: float, elapsed_s: float) -> AtmosphereState: ...


class StandardAtmosphere:
    """US76 hydrostatic lower layers (geopotential), with published upper tables."""
    def __init__(self):
        self.metadata = {"model": "U.S. Standard Atmosphere 1976", "wind": "Zero wind",
                         "source": "https://ntrs.nasa.gov/citations/19770009539",
                         "upper_model": "Log-density / temperature interpolation of US76 upper-atmosphere reference table",
                         "valid_height_m": [-500, 1000000]}
        self.bases = np.array([0, 11000, 20000, 32000, 47000, 51000, 71000, 84852])
        self.lapses = np.array([-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002])
        self.temps, self.pressures = [288.15], [101325.0]
        for k, lapse in enumerate(self.lapses):
            dh = self.bases[k+1]-self.bases[k]
            tb, pb = self.temps[-1], self.pressures[-1]
            tn = tb + lapse * dh
            pn = pb * np.exp(-9.80665*dh/(287.05287*tb)) if lapse == 0 else pb*(tb/tn)**(9.80665/(287.05287*lapse))
            self.temps.append(tn)
            self.pressures.append(pn)
        # Geometric km, density kg/m3, temperature K. Above 86 km pressure is ideal-gas
        # approximate; composition varies. Use MSIS for upper-atmosphere analysis.
        heights = np.array([86, 90, 100, 110, 120, 150, 200, 300, 400, 500, 600, 700, 800, 900, 1000])*1000
        densities = [6.958e-6,3.416e-6,5.604e-7,9.708e-8,2.222e-8,2.076e-9,2.541e-10,1.916e-11,2.803e-12,5.215e-13,1.137e-13,3.070e-14,1.136e-14,5.759e-15,3.561e-15]
        temps = [186.946,186.867,195.081,240,360,634.392,854.559,976.008,995.825,999.235,999.859,999.974,999.995,999.999,1000]
        self._logrho = PchipInterpolator(heights, np.log(densities))
        self._temperature = PchipInterpolator(heights, temps)
        self._logrho_derivative = self._logrho.derivative()
        self._temperature_derivative = self._temperature.derivative()

    def sample_with_derivatives(self, latitude_deg, longitude_deg, height_m, elapsed_s):
        """Exact piecewise derivatives of this provider, per degree/metre/second."""
        state = self.sample(latitude_deg, longitude_deg, height_m, elapsed_s)
        jac = np.zeros((6, 4))  # density, pressure, sound speed, ECEF wind
        if height_m > 1e6 or height_m < -500:
            return state, jac
        z = height_m
        if z >= 86000:
            log_density_gradient = float(self._logrho_derivative(z))
            temperature_gradient = float(self._temperature_derivative(z))
            density_gradient = state.density_kg_m3 * log_density_gradient
            pressure_gradient = 287.05287 * (density_gradient*state.temperature_k + state.density_kg_m3*temperature_gradient)
        else:
            h = 6356766*z/(6356766+z)
            k = min(6, max(0, np.searchsorted(self.bases, h, side='right')-1))
            geopotential_gradient = (6356766/(6356766+z))**2
            temperature_gradient = self.lapses[k]*geopotential_gradient
            pressure_gradient = -state.pressure_pa*9.80665/(287.05287*state.temperature_k)*geopotential_gradient
            density_gradient = state.density_kg_m3*(pressure_gradient/state.pressure_pa-temperature_gradient/state.temperature_k)
        jac[:3, 2] = [density_gradient, pressure_gradient, state.sound_speed_m_s*temperature_gradient/(2*state.temperature_k)]
        return state, jac

    def sample(self, latitude_deg, longitude_deg, height_m, elapsed_s):
        validate_query(latitude_deg, longitude_deg, height_m, elapsed_s)
        if height_m > 1e6:
            return AtmosphereState(0, None, 0, None, np.zeros(3))
        z = max(-500, height_m)
        if z >= 86000:
            rho, temp = float(np.exp(self._logrho(z))), float(self._temperature(z))
            pressure = rho * 287.05287 * temp
        else:
            h = 6356766*z/(6356766+z)
            k = min(6, max(0, np.searchsorted(self.bases, h, side="right")-1))
            dh, lapse = h-self.bases[k], self.lapses[k]
            temp = self.temps[k]+lapse*dh
            pressure = self.pressures[k] * (np.exp(-9.80665*dh/(287.05287*temp)) if lapse == 0 else (self.temps[k]/temp)**(9.80665/(287.05287*lapse)))
            rho = pressure/(287.05287*temp)
        return AtmosphereState(rho, temp, pressure, float(np.sqrt(1.4*287.05287*temp)), np.zeros(3))


class MSISAtmosphere:
    def __init__(self, config: EnvironmentConfig, launch: LaunchSite):
        import pymsis
        self.config, self.launch = config, launch
        self._pymsis = pymsis
        self._profiles = {}
        self._cells = {}
        if config.solar_mode == "automatic":
            try:
                from pymsis.utils import get_f107_ap
                f107, avg, ap = get_f107_ap(np.array([np.datetime64(launch.epoch.replace(tzinfo=None))]))
                self.f107, self.avg, self.ap = float(f107[0]), float(avg[0]), float(ap[0,0])
                if not np.isfinite(self.f107+self.avg+self.ap):
                    raise ValueError("Nonfinite solar indices")
            except Exception as exc:
                raise EnvironmentError("Solar indices are unavailable for this launch date. Select explicit nominal conditions or enter F10.7 / Ap manually.") from exc
        else:
            self.f107, self.avg, self.ap = (150.0, 150.0, 4.0) if config.solar_mode == "nominal" else (config.f107, config.f107_average, config.ap)
        self.metadata = {"model": "NRLMSIS 2.1", "wrapper_version": pymsis.__version__,
                         "solar_mode": config.solar_mode, "f107": self.f107, "f107_average": self.avg, "ap": self.ap,
                         "wind": "Zero wind", "valid_height_m": [0, 1000000],
                         "source": "https://ccmc.gsfc.nasa.gov/models/NRLMSIS~2.1/",
                         "note": "Climatological atmosphere, not a weather forecast. Vacuum above 1,000 km.",
                         "interpolation": "Latitude/longitude/time grid: 1 degree / 1 degree / 1 hour; log-density vertical PCHIP"}
        self._solar_days={launch.epoch.date().isoformat():(self.f107,self.avg,self.ap)}
        self.metadata['solar_indices_by_day']={launch.epoch.date().isoformat():{'f107':self.f107,'f107_average':self.avg,'ap':self.ap}}
        self.heights = np.unique(np.r_[np.arange(0, 22000, 1000), np.arange(22000, 100000, 2000), np.arange(100000, 310000, 10000), np.arange(310000, 1000001, 30000), 1000000])

    def _profile(self, lat: int, lon: int, hour: int):
        key = (lat, lon, hour)
        if key not in self._profiles:
            epoch = self.launch.epoch + timedelta(hours=hour)
            day=epoch.date().isoformat()
            if self.config.solar_mode=='automatic' and day not in self._solar_days:
                try:
                    from pymsis.utils import get_f107_ap
                    f107,avg,ap=get_f107_ap(np.array([np.datetime64(epoch.replace(tzinfo=None))]))
                    values=(float(f107[0]),float(avg[0]),float(ap[0,0]))
                    if not np.all(np.isfinite(values)):
                        raise ValueError('Missing solar indices')
                    self._solar_days[day]=values
                    self.metadata['solar_indices_by_day'][day]={'f107':values[0],'f107_average':values[1],'ap':values[2]}
                except Exception as exc:
                    raise EnvironmentError('Solar indices are unavailable for part of the mission. Select explicit nominal conditions or manual indices.') from exc
            f107,avg,ap=self._solar_days.get(day,(self.f107,self.avg,self.ap))
            date = np.datetime64(epoch.replace(tzinfo=None))
            data = self._pymsis.calculate([date], [lon], [lat], self.heights/1000,
                        f107s=[f107], f107as=[avg], aps=[[ap]*7], version=2.1).reshape(-1,11)
            if not np.all(np.isfinite(data[:,0])) or np.any(data[:,0] <= 0):
                raise EnvironmentError("MSIS returned invalid atmospheric density.")
            temp = data[:,10]
            # Number density determines pressure; anomalous oxygen is excluded.
            number_density = np.nansum(data[:,1:8], axis=1) + np.nan_to_num(data[:,9])
            pressure = np.maximum(number_density*1.380649e-23*temp, 1e-30)
            monatomic = np.nansum(data[:,[3,4,5,6,7]], axis=1)
            diatomic = np.nansum(data[:,[1,2,9]], axis=1)
            gamma = (2.5*monatomic+3.5*diatomic)/np.maximum(1.5*monatomic+2.5*diatomic,1e-30)
            sound = np.sqrt(gamma*pressure/data[:,0])
            self._profiles[key] = PchipInterpolator(self.heights, np.column_stack([np.log(data[:,0]),temp,np.log(pressure),sound]))
        return self._profiles[key]

    def _sample_interpolated(self, latitude_deg, longitude_deg, height_m, elapsed_s, derivatives=False):
        validate_query(latitude_deg, longitude_deg, height_m, elapsed_s)
        if height_m > 1e6:
            state = AtmosphereState(0, None, 0, None, np.zeros(3))
            return (state, np.zeros((6, 4))) if derivatives else state
        lat = float(np.clip(latitude_deg, -90, 90))
        lon = (longitude_deg+180)%360-180
        hour = max(0, elapsed_s)/3600
        low = [int(np.floor(lat)), int(np.floor(lon)), int(np.floor(hour))]
        fractions = [lat-low[0], lon-low[1], hour-low[2]]
        key=tuple(low)
        if key not in self._cells:
            self._cells[key]=np.asarray([self._profile(min(90,low[0]+i),(low[1]+j+180)%360-180,low[2]+k).c
                                       for i in (0,1) for j in (0,1) for k in (0,1)])
        height=max(0,height_m)
        interval=min(len(self.heights)-2,max(0,int(np.searchsorted(self.heights,height,side='right')-1)))
        delta=height-self.heights[interval]
        coefficients=self._cells[key][:,:,interval,:]
        interpolated=((coefficients[:,0]*delta+coefficients[:,1])*delta+coefficients[:,2])*delta+coefficients[:,3]
        a,b,c=fractions
        weights=np.array([(1-a)*(1-b)*(1-c),(1-a)*(1-b)*c,(1-a)*b*(1-c),(1-a)*b*c,
                          a*(1-b)*(1-c),a*(1-b)*c,a*b*(1-c),a*b*c])
        value=weights@interpolated
        if derivatives:
            da=np.array([-(1-b)*(1-c),-(1-b)*c,-b*(1-c),-b*c,(1-b)*(1-c),(1-b)*c,b*(1-c),b*c])
            db=np.array([-(1-a)*(1-c),-(1-a)*c,(1-a)*(1-c),(1-a)*c,-a*(1-c),-a*c,a*(1-c),a*c])
            dc=np.array([-(1-a)*(1-b),(1-a)*(1-b),-(1-a)*b,(1-a)*b,-a*(1-b),a*(1-b),-a*b,a*b])
            vertical=weights@((3*coefficients[:,0]*delta+2*coefficients[:,1])*delta+coefficients[:,2])
            gradient=np.column_stack([da@interpolated,db@interpolated,vertical,dc@interpolated/3600])
            if latitude_deg < -90 or latitude_deg > 90: gradient[:,0]=0
            if elapsed_s < 0: gradient[:,3]=0
            if height_m < 0: gradient[:,2]=0
        if height_m < 0:
            # MSIS lower limit is sea level; hydrostatic extension for below-sea-level land.
            value[0] += -height_m*9.80665/(287.05287*value[1])
            value[2] += -height_m*9.80665/(287.05287*value[1])
            if derivatives:
                extension_gradient=height_m*9.80665/(287.05287*value[1]**2)*gradient[1]
                extension_gradient[2]-=9.80665/(287.05287*value[1])
                gradient[0]+=extension_gradient;gradient[2]+=extension_gradient
        state = AtmosphereState(float(np.exp(value[0])), float(value[1]), float(np.exp(value[2])), float(value[3]), np.zeros(3))
        if not derivatives: return state
        jac=np.zeros((6,4))
        jac[0]=state.density_kg_m3*gradient[0]
        jac[1]=state.pressure_pa*gradient[2]
        jac[2]=gradient[3]
        return state,jac

    def sample(self, latitude_deg, longitude_deg, height_m, elapsed_s):
        return self._sample_interpolated(latitude_deg, longitude_deg, height_m, elapsed_s)

    def sample_with_derivatives(self, latitude_deg, longitude_deg, height_m, elapsed_s):
        """Differentiate the same cached 1° / 1 h PCHIP cells used in replay."""
        return self._sample_interpolated(latitude_deg, longitude_deg, height_m, elapsed_s, True)


def make_atmosphere(config: EnvironmentConfig, launch: LaunchSite) -> AtmosphereProvider:
    return StandardAtmosphere() if config.model == "standard" else MSISAtmosphere(config, launch)


def preview(config: EnvironmentConfig, launch: LaunchSite) -> dict:
    provider = make_atmosphere(config, launch)
    rows = []
    for height in np.unique(np.r_[np.linspace(max(0, launch.elevation_m), 100000, 100), np.linspace(110000, 1000000, 60)]):
        state = provider.sample(launch.latitude_deg, launch.longitude_deg, float(height), 0)
        rows.append({"altitude_m": float(height), "density_kg_m3": state.density_kg_m3,
                     "temperature_k": state.temperature_k, "pressure_pa": state.pressure_pa,
                     "sound_speed_m_s": state.sound_speed_m_s})
    return {"metadata": provider.metadata, "profile": rows}
