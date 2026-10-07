from __future__ import annotations

from datetime import datetime, timezone
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class LaunchSite(Model):
    launch_site_id: str | None = Field(default=None,max_length=80)
    azimuth_sectors_deg: list[tuple[float,float]] = Field(default_factory=list)
    range_source: str = "No departure-sector constraint supplied"
    latitude_deg: float = Field(default=28.5, ge=-90, le=90)
    longitude_deg: float = Field(default=-80.6, ge=-180, le=180)
    elevation_m: float = Field(default=0, ge=-500, le=9000)
    geoid_height_m: float = Field(default=0, ge=-150, le=150)
    elevation_source: str = "Manual sea-level launch"
    elevation_resolution_arcsec: float | None = Field(default=None, gt=0)
    surface: Literal["auto", "land", "ocean"] = "auto"
    elevation_resolved: bool = False
    epoch: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))

    @model_validator(mode="after")
    def utc(self):
        if any(not (0<=start<360 and 0<=end<=360) or start==end for start,end in self.azimuth_sectors_deg):
            raise ValueError("Azimuth sectors need distinct endpoints between 0 and 360 degrees; north-crossing sectors are supported.")
        if self.epoch.tzinfo is None:
            raise ValueError("Launch time must include a UTC offset.")
        self.epoch = self.epoch.astimezone(timezone.utc)
        return self


class EnvironmentConfig(Model):
    model: Literal["msis", "standard"] = "msis"
    solar_mode: Literal["automatic", "manual", "nominal"] = "automatic"
    f107: float = Field(default=150, gt=0, le=500)
    f107_average: float = Field(default=150, gt=0, le=500)
    ap: float = Field(default=4, ge=0, le=400)


class StageMassBounds(Model):
    dry_mass_min_kg: float = Field(gt=0)
    dry_mass_max_kg: float = Field(gt=0)
    propellant_min_kg: float = Field(gt=0)
    propellant_max_kg: float = Field(gt=0)
    hardware_mass_kg: float = Field(gt=0)
    tank_structure_ratio: float = Field(ge=0, le=1)

    @model_validator(mode="after")
    def ordered(self):
        if self.dry_mass_min_kg > self.dry_mass_max_kg or self.propellant_min_kg > self.propellant_max_kg:
            raise ValueError('Minimum stage masses must not exceed maximum masses.')
        if self.hardware_mass_kg+self.tank_structure_ratio*self.propellant_min_kg > self.dry_mass_max_kg:
            raise ValueError('Stage mass bounds contain no feasible hardware and tank design.')
        return self


class StageMass(Model):
    dry_mass_kg: float = Field(gt=0)
    propellant_mass_kg: float = Field(gt=0)


class Stage(Model):
    name: str = Field(default="Stage", min_length=1, max_length=80)
    engine_id: str | None = Field(default=None, max_length=80)
    engine_count: int = Field(default=1, ge=1, le=40)
    engine_catalog_version: str | None = Field(default=None, max_length=40)
    mass_bounds: StageMassBounds | None = None
    engine_type: Literal["liquid", "solid"] = "liquid"
    dry_mass_kg: float = Field(default=12000, gt=0)
    propellant_mass_kg: float = Field(default=180000, gt=0)
    vacuum_thrust_n: float = Field(default=3500000, gt=0)
    isp_sea_level_s: float = Field(default=285, gt=0)
    isp_vacuum_s: float = Field(default=320, gt=0)
    reference_area_m2: float = Field(default=10, gt=0)
    cd: float = Field(default=0.3, ge=0, le=5)
    cd_table: list[tuple[float, float]] = Field(default_factory=list)
    throttle_min: float = Field(default=0.4, gt=0, le=1)
    throttle_max: float = Field(default=1, gt=0, le=1)
    ignition_limit: int = Field(default=1, ge=1, le=4)
    separation_delay_s: float = Field(default=2, ge=0, le=300)
    max_thrust_velocity_angle_deg: float = Field(default=15, ge=0, le=90)
    space_thrust_velocity_angle_deg: float | None = Field(default=None, ge=0, le=180)
    # Normalized time 0..1 and thrust multiplier. Duration is derived from fuel.
    solid_burn_curve: list[tuple[float, float]] = Field(default_factory=list)

    @model_validator(mode="after")
    def consistent(self):
        if self.isp_sea_level_s > self.isp_vacuum_s:
            raise ValueError("Sea-level Isp cannot exceed vacuum Isp.")
        if self.throttle_min > self.throttle_max:
            raise ValueError("Minimum throttle cannot exceed maximum throttle.")
        for table, label in [(self.cd_table, "Cd"), (self.solid_burn_curve, "Solid burn")]:
            if table and (len(table) < 2 or any(b[0] <= a[0] for a, b in zip(table, table[1:]))):
                raise ValueError(f"{label} table needs at least two strictly increasing x values.")
            if any(x < 0 or y < 0 or not np.isfinite(x + y) for x, y in table):
                raise ValueError(f"{label} table values must be finite and nonnegative.")
        if self.solid_burn_curve:
            if self.solid_burn_curve[0][0] != 0 or self.solid_burn_curve[-1][0] != 1:
                raise ValueError("Solid burn curve must cover normalized time 0 to 1.")
            if self.curve_mean <= 0:
                raise ValueError("Solid burn curve must have positive impulse.")
        if self.engine_type == "solid" and self.ignition_limit != 1:
            raise ValueError("Solid motors permit exactly one ignition.")
        return self

    @property
    def mass_flow_kg_s(self) -> float:
        return self.vacuum_thrust_n / (9.80665 * self.isp_vacuum_s)

    @property
    def curve_mean(self) -> float:
        if not self.solid_burn_curve:
            return 1.0
        table = np.asarray(self.solid_burn_curve)
        return float(np.trapezoid(table[:, 1], table[:, 0]))

    @property
    def full_burn_s(self) -> float:
        return self.propellant_mass_kg / (self.mass_flow_kg_s * (self.curve_mean if self.engine_type == "solid" else 1))


class Vehicle(Model):
    optimize_stage_masses: bool = False
    minimum_initial_twr: float = Field(default=1.05, gt=1, le=5)
    payload_mass_kg: float = Field(default=3000, gt=0)
    expected_initial_mass_kg: float | None = Field(default=None, gt=0)
    stages: list[Stage] = Field(min_length=1, max_length=6)

    @property
    def initial_mass_kg(self) -> float:
        return self.payload_mass_kg + sum(s.dry_mass_kg + s.propellant_mass_kg for s in self.stages)

    @model_validator(mode="after")
    def mass_check(self):
        if self.expected_initial_mass_kg is not None:
            if abs(self.expected_initial_mass_kg - self.initial_mass_kg) > max(0.01, self.initial_mass_kg * 1e-6):
                raise ValueError("Expected initial mass does not match payload plus all stage masses.")
        return self


class Constraints(Model):
    vertical_ascent_s: float = Field(default=25, ge=0, le=300)
    max_dynamic_pressure_pa: float | None = Field(default=None, gt=0)
    max_load_factor_g: float | None = Field(default=None, gt=0)
    max_mission_duration_s: float = Field(default=172800, gt=1, le=2592000)
    solver_budget_s: float = Field(default=180, ge=5, le=1800)
    max_iterations: int = Field(default=60, ge=1, le=500)
    solver_method: Literal['mission', 'legacy'] = 'mission'
    control_nodes: int = Field(default=5, ge=3, le=25)
    mesh_segments: int = Field(default=8, ge=4, le=64)
    refinement_levels: int = Field(default=2, ge=1, le=3)
    max_topologies: int = Field(default=6, ge=1, le=24)


class TargetOrbit(Model):
    perigee_altitude_m: float | None = Field(default=200000, ge=100000, le=1e9)
    apogee_altitude_m: float | None = Field(default=None, ge=100000, le=1e9)
    inclination_deg: float | None = Field(default=None, ge=0, le=180)
    raan_deg: float | None = Field(default=None, ge=0, lt=360)
    argument_of_periapsis_deg: float | None = Field(default=None, ge=0, lt=360)
    arrival_phase_deg: float | None = Field(default=None, ge=0, lt=360)
    altitude_tolerance_m: float = Field(default=1000, gt=0, le=100000)
    angle_tolerance_deg: float = Field(default=0.1, gt=0, le=10)
    arrival_time_min_s: float | None = Field(default=None, ge=0)
    arrival_time_max_s: float | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def shape(self):
        if self.arrival_time_min_s is not None and self.arrival_time_max_s is not None and self.arrival_time_min_s > self.arrival_time_max_s:
            raise ValueError('Earliest arrival must not exceed latest arrival.')
        if all(getattr(self,key) is None for key in ('perigee_altitude_m','apogee_altitude_m','inclination_deg','raan_deg','argument_of_periapsis_deg','arrival_phase_deg')):
            raise ValueError("Specify at least one target-orbit parameter. Blank parameters remain unconstrained.")
        if self.apogee_altitude_m is not None and self.perigee_altitude_m is not None and self.apogee_altitude_m < self.perigee_altitude_m:
            raise ValueError("Apogee must be at least as high as perigee.")
        circular = self.apogee_altitude_m is not None and self.perigee_altitude_m is not None and abs(self.apogee_altitude_m - self.perigee_altitude_m) < 1e-3
        equatorial = self.inclination_deg is not None and min(self.inclination_deg, 180 - self.inclination_deg) < 1e-6
        if equatorial and self.raan_deg is not None:
            raise ValueError("RAAN is undefined for an equatorial orbit; leave it unconstrained.")
        if circular and self.argument_of_periapsis_deg is not None:
            raise ValueError("Argument of periapsis is undefined for a circular orbit.")
        return self


class BudgetAssumptions(Model):
    """Advisory planning inputs; these never modify flight dynamics or targets."""
    gravity_loss_m_s: float = Field(default=1800, ge=0, le=15000)
    drag_loss_m_s: float = Field(default=150, ge=0, le=15000)
    steering_loss_m_s: float = Field(default=150, ge=0, le=15000)
    contingency_percent: float = Field(default=10, ge=0, le=100)


class Scenario(Model):
    schema_version: Literal[1] = 1
    name: str = Field(default="Two-stage LEO study", max_length=100)
    launch: LaunchSite = Field(default_factory=LaunchSite)
    environment: EnvironmentConfig = Field(default_factory=EnvironmentConfig)
    vehicle: Vehicle
    constraints: Constraints = Field(default_factory=Constraints)
    target: TargetOrbit = Field(default_factory=TargetOrbit)
    objective: Literal["propellant", "time"] = "propellant"
    budget: BudgetAssumptions = Field(default_factory=BudgetAssumptions)

    @model_validator(mode='after')
    def sizing_method(self):
        if self.vehicle.optimize_stage_masses and self.constraints.solver_method != 'mission':
            raise ValueError('Stage mass optimization requires the complete-mission solver.')
        return self


class Burn(Model):
    stage_index: int = Field(ge=0)
    duration_s: float = Field(gt=0)
    coast_before_s: float = Field(default=0, ge=0)
    steering_reference: Literal['air_relative','inertial_velocity'] = 'air_relative'
    angle_fractions: tuple[float, ...] = (0.5, 0.2, 0.0)
    clock_angles_rad: tuple[float, ...] = (0, 0, 0)
    throttles: tuple[float, ...] = (1, 1, 1)

    @model_validator(mode="after")
    def controls(self):
        if not 2 <= len(self.angle_fractions) <= 129 or len(self.angle_fractions) != len(self.clock_angles_rad) or len(self.angle_fractions) != len(self.throttles):
            raise ValueError('Guidance and throttle need equally sized control arrays with 2 to 129 nodes.')
        if any(not 0 <= x <= 1 for x in self.angle_fractions + self.throttles):
            raise ValueError("Angle fractions and throttle must lie between 0 and 1.")
        return self


class FlightPlan(Model):
    stage_masses: list[StageMass] | None = None
    launch_azimuth_deg: float = 90
    burns: list[Burn] = Field(min_length=1)
    final_coast_s: float = Field(default=0, ge=0)
    # Old saved commands must replay in the frame in which they were designed.
    guidance_frame: Literal['launch_heading','local_orbital'] = 'launch_heading'


def default_scenario() -> Scenario:
    return Scenario(launch=LaunchSite(launch_site_id="ll2-87",latitude_deg=28.60822681,
                                    longitude_deg=-80.60428186,surface="land",azimuth_sectors_deg=[(35,120)],range_source="NASA Ames launch-azimuth reference (2013)"),vehicle=Vehicle(stages=[
        Stage(name="Booster", dry_mass_kg=14000, propellant_mass_kg=220000,
              vacuum_thrust_n=4200000, isp_sea_level_s=285, isp_vacuum_s=320,
              reference_area_m2=12, max_thrust_velocity_angle_deg=15),
        Stage(name="Upper stage", dry_mass_kg=3500, propellant_mass_kg=65000,
              vacuum_thrust_n=800000, isp_sea_level_s=310, isp_vacuum_s=360,
              reference_area_m2=7, ignition_limit=2, max_thrust_velocity_angle_deg=15),
    ]))
