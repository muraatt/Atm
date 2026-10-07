from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from typing import Callable

import numpy as np
from scipy.integrate import solve_ivp

from .earth import EarthFrames, G0, R_E, gravity, targeted_elements, orbit_curve, target_curve, unit, wrap_deg
from .environment import AtmosphereProvider, EnvironmentError, make_atmosphere
from .models import Burn, FlightPlan, Scenario
from .screening import azimuth_margin


class Cancelled(Exception):
    pass


class SolverDeadline(Exception):
    def __init__(self, scope="total_budget"):
        self.scope = scope
        super().__init__(scope)


class NumericalFailure(Exception):
    pass


class InvalidTrialState(NumericalFailure):
    """An optimizer trial lies outside the physical/numerical integration domain."""
    pass


@dataclass
class Segment:
    state: np.ndarray
    time: float
    status: str
    rows: list[dict]
    max_q: float = 0
    max_load: float = 0
    max_angle: float = 0
    min_height: float = 0
    min_departure_margin_deg: float = 180


def thrust_direction(relative_velocity, up, heading, angle_rad, clock_rad, frame='local_orbital'):
    if np.linalg.norm(relative_velocity) < 1:
        return up, 0.0
    forward = unit(relative_velocity)
    # Use a local orbital frame after liftoff. Projecting the launch heading
    # becomes singular when velocity aligns with it and can flip the normal
    # steering axis during a small plane change. The vertical projection stays
    # well conditioned for orbital flight; heading defines the radial limit.
    vertical_projection = -up + np.dot(up, forward)*forward
    heading_projection = heading - np.dot(heading, forward)*forward
    # Near radial ascent gravity/geodetic differences must not choose the
    # departure plane. Blend from the commanded heading to the orbital frame
    # over 5–15 degrees away from the vertical.
    blend=float(np.clip((np.linalg.norm(vertical_projection)-np.sin(np.radians(5)))/
                        (np.sin(np.radians(15))-np.sin(np.radians(5))),0,1))
    blend=blend*blend*(3-2*blend)
    mixed=(1-blend)*unit(heading_projection)+blend*unit(vertical_projection)
    if np.linalg.norm(mixed)<1e-8:
        mixed=vertical_projection if np.linalg.norm(vertical_projection)>1e-6 else heading_projection
    downrange=unit(mixed)
    if frame=='launch_heading':
        downrange=heading_projection
        if np.linalg.norm(downrange)<1e-6:downrange=vertical_projection
        downrange=unit(downrange)
    crossrange = unit(np.cross(forward, downrange))
    lateral = np.cos(clock_rad)*downrange + np.sin(clock_rad)*crossrange
    return unit(np.cos(angle_rad)*forward + np.sin(angle_rad)*lateral), angle_rad


def thrust_angle_limit(stage, height):
    """Explicit vehicle capability; blend atmospheric/space limits over 100–120 km."""
    if stage.space_thrust_velocity_angle_deg is None:
        return stage.max_thrust_velocity_angle_deg
    fraction = float(np.clip((height-100000)/20000, 0, 1))
    return (1-fraction)*stage.max_thrust_velocity_angle_deg + fraction*stage.space_thrust_velocity_angle_deg


def project_to_velocity_cone(direction,relative,up,heading,limit_rad):
    """Parameterize inertial steering while retaining the physical air cone."""
    if np.linalg.norm(relative)<1:return up,0.
    forward=unit(relative)
    angle=float(np.arctan2(np.linalg.norm(np.cross(forward,direction)),np.dot(forward,direction)))
    if angle<=limit_rad:return direction,angle
    lateral=direction-np.dot(direction,forward)*forward
    if np.linalg.norm(lateral)<1e-12:lateral=up-np.dot(up,forward)*forward
    if np.linalg.norm(lateral)<1e-12:lateral=heading-np.dot(heading,forward)*forward
    return unit(np.cos(limit_rad)*forward+np.sin(limit_rad)*unit(lateral)),limit_rad


def validate_plan(scenario: Scenario, plan: FlightPlan):
    counts = [0]*len(scenario.vehicle.stages)
    last_stage = 0
    if plan.burns[0].stage_index != 0 or plan.burns[0].coast_before_s != 0:
        raise ValueError("The first phase must ignite stage 1 on the launch pad.")
    for burn in plan.burns:
        idx = burn.stage_index
        if idx >= len(counts) or idx < last_stage or idx > last_stage+1:
            raise ValueError("Burn phases must follow the vehicle's stage order.")
        stage = scenario.vehicle.stages[idx]
        counts[idx] += 1
        if counts[idx] > stage.ignition_limit:
            raise ValueError("Flight plan exceeds a stage's ignition limit.")
        if stage.engine_type == "solid":
            if not np.isclose(burn.duration_s, stage.full_burn_s, rtol=1e-8) or any(x != 1 for x in burn.throttles):
                raise ValueError("Solid motors must follow their complete burn at fixed throttle.")
        elif any(x < stage.throttle_min or x > stage.throttle_max for x in burn.throttles):
            raise ValueError("Throttle command is outside the engine's allowed range.")
        last_stage = idx
    if not all(counts):
        raise ValueError("Each vehicle stage must have at least one burn.")


class Simulation:
    def __init__(self, scenario: Scenario, atmosphere: AtmosphereProvider | None = None,
                 check: Callable[[], None] | None = None, frames=None):
        self.scenario = scenario
        self.atmosphere = atmosphere or make_atmosphere(scenario.environment, scenario.launch)
        self.frames = frames or EarthFrames(scenario.launch.epoch, scenario.constraints.max_mission_duration_s)
        self.check = check or (lambda: None)
        self.initial = np.r_[self.frames.initial_state(scenario.launch), scenario.vehicle.stages[0].propellant_mass_kg]
        self._rhs_count = 0
        self._guidance_frame='launch_heading'

    def base_mass(self, stage_idx):
        return self.scenario.vehicle.payload_mass_kg + self.scenario.vehicle.stages[stage_idx].dry_mass_kg + sum(s.dry_mass_kg+s.propellant_mass_kg for s in self.scenario.vehicle.stages[stage_idx+1:])

    def for_design(self, stage_masses):
        if stage_masses is None:return self
        from .sizing import apply_stage_masses
        sized=apply_stage_masses(self.scenario,stage_masses)
        sim=Simulation(sized,self.atmosphere,self.check,frames=self.frames)
        sim.initial=self.initial.copy()
        sim.initial[6]*=sized.vehicle.stages[0].propellant_mass_kg/self.scenario.vehicle.stages[0].propellant_mass_kg
        return sim

    def surface_clearance(self, t, position):
        if not np.isfinite(t) or not np.all(np.isfinite(position)):
            raise InvalidTrialState("Nonfinite time or position in a trial trajectory.")
        height = self.frames.geographic(t, position)[2]
        floor = self.collision_floor(position)
        return height-floor, floor

    def collision_floor(self, position):
        distance = np.linalg.norm(position-self.initial[:3])
        return self.scenario.launch.elevation_m+self.scenario.launch.geoid_height_m if distance < 50000 else 0.0

    def validate_state(self, t, state):
        if np.shape(state) != (7,) or not np.isfinite(t) or t < 0 or not np.all(np.isfinite(state)):
            raise InvalidTrialState("Trial state must contain finite position, velocity, fuel and nonnegative time.")
        # hypot avoids overflow even for finite but absurd optimizer values.
        radius_limit = R_E+1e6*self.scenario.constraints.max_mission_duration_s
        if np.max(np.abs(state[:3])) > radius_limit or np.max(np.abs(state[3:6])) > 1e6:
            raise InvalidTrialState("Trial position or velocity is outside the integration domain.")
        radius = np.hypot.reduce(state[:3])
        speed = np.hypot.reduce(state[3:6])
        if not np.isfinite(radius) or not np.isfinite(speed) or radius < R_E*.5 or radius > radius_limit or speed > 1e6:
            raise InvalidTrialState("Trial position or velocity is outside the integration domain.")

    def forces(self, t, y, stage_idx, burn=None, burn_start=0, azimuth=90):
        self.validate_state(t, y)
        stage = self.scenario.vehicle.stages[stage_idx]
        rotation = self.frames.matrix(t)
        lat, lon, height = self.frames.geographic(t, y[:3])
        floor = self.collision_floor(y[:3])
        # RK internal stages may cross the surface before the impact root is
        # located. Extend atmosphere at the collision surface only for those
        # stages; integration never accepts an underground starting state.
        air = self.atmosphere.sample(lat, lon, max(height, floor), t)
        if np.shape(air.wind_ecef_m_s)!=(3,):
            raise EnvironmentError("Atmosphere wind must be a finite three-component vector.")
        air_values = [air.density_kg_m3, air.pressure_pa, *(air.wind_ecef_m_s)]
        if air.temperature_k is not None: air_values.append(air.temperature_k)
        if air.sound_speed_m_s is not None: air_values.append(air.sound_speed_m_s)
        if (np.shape(air.wind_ecef_m_s)!=(3,) or not np.all(np.isfinite(air_values))
            or air.density_kg_m3 < 0 or air.pressure_pa < 0
            or (air.temperature_k is not None and air.temperature_k<=0)
            or (air.sound_speed_m_s is not None and air.sound_speed_m_s<=0)):
            raise EnvironmentError("Atmosphere returned invalid density, pressure, temperature, sound speed or wind.")
        lat_rad, lon_rad = np.radians([lat,lon])
        up = rotation @ np.array([np.cos(lat_rad)*np.cos(lon_rad),np.cos(lat_rad)*np.sin(lon_rad),np.sin(lat_rad)])
        north = rotation @ np.array([-np.sin(lat_rad)*np.cos(lon_rad),-np.sin(lat_rad)*np.sin(lon_rad),np.cos(lat_rad)])
        east = rotation @ np.array([-np.sin(lon_rad),np.cos(lon_rad),0])
        heading = np.cos(np.radians(azimuth))*north + np.sin(np.radians(azimuth))*east
        # Astropy frame table supplies inertial Earth rotation velocity.
        relative = y[3:6] - self.frames.earth_velocity(t,y[:3]) - rotation @ air.wind_ecef_m_s
        speed = np.linalg.norm(relative)
        mach = speed/air.sound_speed_m_s if air.sound_speed_m_s else 0
        cd = float(np.interp(mach,*np.asarray(stage.cd_table).T)) if stage.cd_table else stage.cd
        q = 0.5*air.density_kg_m3*speed**2
        drag = -q*cd*stage.reference_area_m2*unit(relative)
        thrust, mdot, angle, throttle = 0.0, 0.0, 0.0, 0.0
        direction = up
        if burn is not None:
            fraction = np.clip((t-burn_start)/burn.duration_s,0,1)
            nodes = np.linspace(0, 1, len(burn.throttles))
            throttle = 1.0 if stage.engine_type == "solid" else float(np.interp(fraction,nodes,burn.throttles))
            shape = float(np.interp(fraction,*np.asarray(stage.solid_burn_curve).T)) if stage.engine_type == "solid" and stage.solid_burn_curve else 1.0
            mdot = stage.mass_flow_kg_s*throttle*shape
            isp = stage.isp_vacuum_s-(stage.isp_vacuum_s-stage.isp_sea_level_s)*np.clip(air.pressure_pa/101325,0,1)
            thrust = mdot*G0*isp
            inertial=burn.steering_reference=='inertial_velocity'
            requested_angle = (np.pi if inertial else np.radians(thrust_angle_limit(stage,height)))*np.interp(fraction,nodes,burn.angle_fractions)
            clock = np.interp(fraction,nodes,burn.clock_angles_rad)
            direction, angle = thrust_direction(y[3:6] if inertial else relative,up,heading,requested_angle,clock,self._guidance_frame)
            if stage_idx == 0 and burn_start == 0 and height < 100000 and t < self.scenario.constraints.vertical_ascent_s + 10 and speed >= 1:
                # Hold a vertical launch initially, but respect the same cone:
                # no undocumented exemption from the thrust/velocity limit.
                forward = unit(relative)
                toward_up = up-np.dot(up,forward)*forward
                deviation = np.arccos(np.clip(np.dot(forward,up),-1,1))
                allowed = np.radians(thrust_angle_limit(stage,height))
                vertical_command = forward if np.linalg.norm(toward_up)<1e-12 else unit(np.cos(min(allowed,deviation))*forward+np.sin(min(allowed,deviation))*unit(toward_up))
                blend=np.clip((t-self.scenario.constraints.vertical_ascent_s)/10,0,1)
                direction=unit((1-blend)*vertical_command+blend*direction)
                angle=float(np.arccos(np.clip(np.dot(forward,direction),-1,1)))
            if inertial:
                direction,angle=project_to_velocity_cone(direction,relative,up,heading,np.radians(thrust_angle_limit(stage,height)))
        mass = self.base_mass(stage_idx)+max(0,y[6])
        proper = (thrust*direction+drag)/mass
        acceleration = gravity(y[:3],rotation)+proper
        if burn is not None and stage_idx==0 and burn_start==0 and height <= self.scenario.launch.elevation_m+self.scenario.launch.geoid_height_m+.05 and speed<1 and abs(lat-self.scenario.launch.latitude_deg)<1e-4 and abs(wrap_deg(lon-self.scenario.launch.longitude_deg))<1e-4:
            surface_acc=np.cross(self.frames._angular[min(len(self.frames._angular)-1,int(t/self.frames._step))],self.frames.earth_velocity(t,y[:3]))
            support=max(0,float(np.dot(surface_acc-acceleration,up)))
            acceleration+=support*up
            proper+=support*up
        derivative = np.r_[y[3:6],acceleration,-mdot]
        if not np.all(np.isfinite(derivative)) or not np.isfinite(q):
            raise InvalidTrialState("Nonfinite forces in a trial trajectory.")
        return derivative, {
            "time_s":float(t),"stage_index":stage_idx,"phase":"Burn" if burn else "Coast",
            "latitude_deg":lat,"longitude_deg":lon,"altitude_m":height,
            "inertial_speed_m_s":float(np.linalg.norm(y[3:6])),"relative_speed_m_s":float(speed),
            "mass_kg":float(mass),"active_propellant_kg":float(max(0,y[6])),
            "thrust_n":float(thrust),"throttle":float(throttle),"density_kg_m3":air.density_kg_m3,
            "temperature_k":air.temperature_k,"pressure_pa":air.pressure_pa,
            "mach":float(mach) if air.sound_speed_m_s else None,"dynamic_pressure_pa":float(q),
            "load_factor_g":float(np.linalg.norm(proper)/G0),"thrust_velocity_angle_deg":float(np.degrees(angle)),
            "thrust_angle_limit_deg":thrust_angle_limit(stage,height),
            "position_gcrs_m":y[:3].tolist(),"velocity_gcrs_m_s":y[3:6].tolist(),
        }

    def departure_margin(self,rows):
        if not self.scenario.launch.azimuth_sectors_deg:return 180.
        margins=[180.]
        lat0,lon0=np.radians([self.scenario.launch.latitude_deg,self.scenario.launch.longitude_deg])
        for row in rows:
            if row['stage_index']!=0:continue
            lat,lon=np.radians([row['latitude_deg'],row['longitude_deg']]);dlon=lon-lon0
            north=np.cos(lat0)*np.sin(lat)-np.sin(lat0)*np.cos(lat)*np.cos(dlon)
            east=np.sin(dlon)*np.cos(lat)
            if np.hypot(north,east)*R_E>1000:
                bearing=float(np.degrees(np.arctan2(east,north))%360)
                margins.append(azimuth_margin(bearing,self.scenario.launch.azimuth_sectors_deg))
        return min(margins)

    def integrate(self,y,t,duration,stage_idx,burn=None,azimuth=90,strict=False,record=False):
        self.check()
        self.validate_state(t, y)
        if not np.isfinite(duration) or duration < 0 or not np.isfinite(azimuth):
            raise InvalidTrialState("Trial duration and heading must be finite; duration must be nonnegative.")
        clearance, _ = self.surface_clearance(t, y[:3])
        if clearance < -.1:
            return Segment(y.copy(),t,"Impact",[],min_height=clearance)
        if y[6] < -.1:
            raise InvalidTrialState("Trial propellant cannot be negative.")
        if burn is not None and y[6] <= 1e-5:
            return Segment(y.copy(),t,"Fuel Depleted",[])
        if duration <= 1e-7:
            return Segment(y.copy(),t,"Complete",[])
        requested_duration = duration
        duration = min(duration,self.scenario.constraints.max_mission_duration_s-t)
        if duration <= 0:
            return Segment(y.copy(),t,"Time Limit",[])
        start=t
        recent_rhs_times=deque(maxlen=2048)
        def rhs(time,state):
            self._rhs_count += 1
            if self._rhs_count % 16 == 0:
                self.check()
            recent_rhs_times.append(time)
            if (len(recent_rhs_times)==recent_rhs_times.maxlen and self._rhs_count%128==0 and
                    max(recent_rhs_times)-min(recent_rhs_times)<1e-3):
                # A retrograde command near the 1 m/s vertical-guidance switch
                # can chatter without advancing physical time. The strict
                # verifier has no search deadline; reject this stalled candidate
                # explicitly rather than looping indefinitely during replay.
                raise NumericalFailure('Integration stalled near a guidance or model discontinuity '
                                       f'at {time:.6f} s; this candidate could not be verified.')
            return self.forces(time,state,stage_idx,burn,start,azimuth)[0]
        def fuel(time,state):
            return state[6]
        fuel.terminal, fuel.direction = True, -1
        def impact(time,state):
            # Global fallback collision surface is the ellipsoid. Near the pad use
            # its resolved terrain/geoid; a separate full terrain-corridor model is
            # intentionally not implied by the launch-site DEM.
            return self.surface_clearance(time,state[:3])[0]+0.1
        impact.terminal, impact.direction = True, -1
        try:
            solution=solve_ivp(rhs,(t,t+duration),y,method="DOP853",rtol=2e-10 if strict else 2e-8,
                               atol=np.array([0.001]*3+[1e-6]*3+[1e-5]) if strict else np.array([.02]*3+[1e-5]*3+[.001]),
                               max_step=1.0 if strict and burn else (3.0 if burn else 90.0),
                               events=[fuel,impact] if burn else [impact],dense_output=True)
        except (Cancelled,SolverDeadline,EnvironmentError,NumericalFailure):
            raise
        except Exception as exc:
            raise NumericalFailure(f"Integration failed: {exc}") from exc
        if not solution.success:
            raise NumericalFailure(solution.message)
        end=float(solution.t[-1])
        state=solution.y[:,-1].copy()
        self.validate_state(end, state)
        if state[6] < 0 and state[6] > -0.1:
            state[6]=0
        impact_hit=bool(solution.t_events[-1].size)
        fuel_hit=bool(solution.t_events[0].size) if burn else False
        status="Impact" if impact_hit else ("Fuel Depleted" if fuel_hit else "Complete")
        if end >= self.scenario.constraints.max_mission_duration_s-1e-6 and duration < requested_duration-1e-6:
            status="Time Limit"
        spacing=1.0 if burn or (self.frames.geographic(end,state[:3])[2] < 120000) else max(5,(end-t)/500)
        sample_times=np.unique(np.r_[np.arange(t,end,spacing),end]) if record or strict else np.unique(np.r_[solution.t,np.linspace(t,end, min(80,max(2,int((end-t)/3)+1)))])
        if strict:
            # Replay checks include integrator nodes and commanded/profile
            # breakpoints, rather than relying only on one-second output rows.
            extra=solution.t
            if burn is not None:
                stage=self.scenario.vehicle.stages[stage_idx]
                fractions=np.r_[np.linspace(0,1,len(burn.throttles)),[x for x,y in stage.solid_burn_curve]]
                extra=np.r_[extra,start+burn.duration_s*fractions]
            sample_times=np.unique(np.r_[sample_times,extra[(extra>=t)&(extra<=end)]])
        rows=[self.forces(float(s),solution.sol(s),stage_idx,burn,start,azimuth)[1] for s in sample_times]
        return Segment(state,end,status,rows if record else [],
                       max((r["dynamic_pressure_pa"] for r in rows),default=0),
                       max((r["load_factor_g"] for r in rows),default=0),
                       max((r["thrust_velocity_angle_deg"] for r in rows),default=0),
                       min((r["altitude_m"] for r in rows),default=0),self.departure_margin(rows))

    def run(self,plan:FlightPlan,strict=True,record=True):
        if plan.stage_masses is not None:
            from .sizing import design_report
            try:sized=self.for_design(plan.stage_masses)
            except ValueError as exc:raise NumericalFailure(str(exc)) from exc
            result=sized.run(plan.model_copy(update={'stage_masses':None}),strict,record)
            result['input_scenario']=self.scenario.model_dump(mode='json')
            result['flight_plan']=plan.model_dump()
            result['stage_sizing']=design_report(self.scenario,sized.scenario)
            result['constraint_checks']['stage_mass_bounds']=True
            from .sizing import pad_reference
            pad_thrust,weight=pad_reference(sized)
            twr=pad_thrust*plan.burns[0].throttles[0]/(sized.scenario.vehicle.initial_mass_kg*weight)
            twr_ok=twr>=self.scenario.vehicle.minimum_initial_twr-1e-6
            result['constraint_checks']['sized_initial_twr']=bool(twr_ok)
            result['stage_sizing']['verified_initial_pad_twr']=float(twr)
            if not twr_ok and result['status']=='Target Achieved':result['status']='Target Not Reached'
            return result
        self._strict_run = strict
        self._guidance_frame=plan.guidance_frame
        validate_plan(self.scenario,plan)
        y,t=self.initial.copy(),0.0
        _,initial=self.forces(0,y,0,plan.burns[0],0,plan.launch_azimuth_deg)
        # Proper upward thrust at liftoff must exceed local gravity.
        acceleration,_=self.forces(0,y,0,plan.burns[0],0,plan.launch_azimuth_deg)
        up=unit(y[:3])
        first_stage=self.scenario.vehicle.stages[0]
        peak_shape=max((y for x,y in first_stage.solid_burn_curve),default=1) if first_stage.engine_type=='solid' else 1
        launch_isp=first_stage.isp_vacuum_s-(first_stage.isp_vacuum_s-first_stage.isp_sea_level_s)*np.clip(initial['pressure_pa']/101325,0,1)
        maximum_thrust=first_stage.vacuum_thrust_n*launch_isp/first_stage.isp_vacuum_s*peak_shape*(1 if first_stage.engine_type=='solid' else max(plan.burns[0].throttles))
        surface_acc=np.cross(self.frames._angular[0],self.frames.earth_velocity(0,y[:3]))
        effective_weight=float(np.dot(surface_acc-gravity(y[:3],self.frames.matrix(0)),up))
        if maximum_thrust/self.scenario.vehicle.initial_mass_kg <= effective_weight:
            return self.result(y,t,"No Liftoff",[initial],[],[],plan)
        rows,events,ledger=[],[],[]
        active=0
        status="Complete"
        for burn in plan.burns:
            self.check()
            idx=burn.stage_index
            if idx != active:
                stage=self.scenario.vehicle.stages[active]
                events.append({"time_s":t,"event":"Stage Separation","stage":stage.name,
                               "discarded_dry_mass_kg":stage.dry_mass_kg,"discarded_propellant_kg":float(y[6])})
                ledger.append({"stage":stage.name,"initial_kg":stage.propellant_mass_kg,
                               "burned_kg":float(stage.propellant_mass_kg-y[6]),"remaining_kg":0.0,"discarded_kg":float(y[6])})
                y[6]=self.scenario.vehicle.stages[idx].propellant_mass_kg
                delay=stage.separation_delay_s
                active=idx
                coast=self.integrate(y,t,delay,idx,azimuth=plan.launch_azimuth_deg,strict=strict,record=record)
                y,t=coast.state,coast.time
                rows.extend(coast.rows)
                if coast.status in ("Impact","Time Limit"):
                    status=coast.status
                    break
            if burn.coast_before_s:
                events.append({"time_s":t,"event":"Coast Start","stage":self.scenario.vehicle.stages[idx].name})
                coast=self.integrate(y,t,burn.coast_before_s,idx,azimuth=plan.launch_azimuth_deg,strict=strict,record=record)
                y,t=coast.state,coast.time
                rows.extend(coast.rows)
                if coast.status in ("Impact","Time Limit"):
                    status=coast.status
                    break
            events.append({"time_s":t,"event":"Ignition","stage":self.scenario.vehicle.stages[idx].name})
            segment=self.integrate(y,t,burn.duration_s,idx,burn,plan.launch_azimuth_deg,strict,record)
            y,t=segment.state,segment.time
            rows.extend(segment.rows)
            events.append({"time_s":t,"event":"Fuel Depletion" if segment.status=="Fuel Depleted" else "Cutoff","stage":self.scenario.vehicle.stages[idx].name})
            if segment.status=="Impact":
                status=segment.status
                break
            if t >= self.scenario.constraints.max_mission_duration_s-1e-5:
                status="Time Limit"
                break
        if status == 'Complete' and plan.final_coast_s:
            events.append({'time_s':t,'event':'Arrival Coast','stage':self.scenario.vehicle.stages[active].name})
            coast=self.integrate(y,t,plan.final_coast_s,active,azimuth=plan.launch_azimuth_deg,strict=strict,record=record)
            y,t=coast.state,coast.time
            rows.extend(coast.rows)
            if coast.status in ('Impact','Time Limit'): status=coast.status
        stage=self.scenario.vehicle.stages[active]
        ledger.append({"stage":stage.name,"initial_kg":stage.propellant_mass_kg,
                       "burned_kg":float(stage.propellant_mass_kg-y[6]),"remaining_kg":float(y[6]),"discarded_kg":0.0})
        for pending in self.scenario.vehicle.stages[active+1:]:
            ledger.append({"stage":pending.name,"initial_kg":pending.propellant_mass_kg,"burned_kg":0.0,
                           "remaining_kg":pending.propellant_mass_kg,"discarded_kg":0.0})
        return self.result(y,t,status,rows,events,ledger,plan,active)

    def result(self,y,t,status,rows,events,ledger,plan,active=0):
        self._last_state=y.copy()
        orbit=targeted_elements(y[:3],y[3:6],self.scenario.target)
        target=self.scenario.target
        comparisons=compare_target(orbit,target)
        limits=self.scenario.constraints
        termination_reason=status
        duration_ok=bool(status!='Time Limit' and t<=limits.max_mission_duration_s+1e-5)
        maxq=max((r["dynamic_pressure_pa"] for r in rows),default=0)
        maxload=max((r["load_factor_g"] for r in rows),default=0)
        q_ok=limits.max_dynamic_pressure_pa is None or maxq <= limits.max_dynamic_pressure_pa*(1+1e-6)
        g_ok=limits.max_load_factor_g is None or maxload <= limits.max_load_factor_g*(1+1e-6)
        angle_ok=all(r["thrust_velocity_angle_deg"] <= thrust_angle_limit(self.scenario.vehicle.stages[r["stage_index"]],r['altitude_m'])+1e-5 for r in rows)
        angle_utilization=max((r["thrust_velocity_angle_deg"]/max(1e-12,thrust_angle_limit(self.scenario.vehicle.stages[r["stage_index"]],r['altitude_m'])) for r in rows if r["thrust_n"]>0),default=0)
        arrival_ok=bool((target.arrival_time_min_s is None or t>=target.arrival_time_min_s-1e-5) and (target.arrival_time_max_s is None or t<=target.arrival_time_max_s+1e-5))
        azimuth_ok=bool(azimuth_margin(plan.launch_azimuth_deg,self.scenario.launch.azimuth_sectors_deg)>=-1e-5)
        corridor_ok=self.departure_margin(rows)>=-1e-5
        bound_ok=orbit['bound'] and orbit['perigee_altitude_m']>=100000-1e-3
        if status=="Complete":
            status="Target Achieved" if bound_ok and all(c["within_tolerance"] for c in comparisons) and q_ok and g_ok and angle_ok and azimuth_ok and corridor_ok and arrival_ok and duration_ok else "Target Not Reached"
        if status=="Time Limit":
            status="Target Not Reached"
        final=rows[-1] if rows else self.forces(t,y,active)[1]
        warnings=self.frames.warnings+[
            "3-DOF point-mass analysis: attitude, structural loads, heating and long-term orbit stability are not modeled.",
            "Collision checks use launch-site terrain within 50 km and the WGS84 ellipsoid elsewhere; elevated terrain along the full flight corridor is not resolved.",
            "J2 plus central Earth gravity only; Sun/Moon perturbations are not modeled for high or long-duration transfers.",
            "Atmospheric temperature is not vehicle skin temperature."]
        warnings.append("Departure-sector checks cover launch heading and first-stage ground-track bearing after 1 km downrange. They do not model operational drop zones, populated-area polygons or current range clearance.")
        if self.scenario.environment.model=="standard":
            warnings.append("US76 above 86 km uses an interpolated reference table and approximate composition; use MSIS for upper-atmosphere work.")
        if not ledger:
            ledger=[{"stage":s.name,"initial_kg":s.propellant_mass_kg,"burned_kg":0.0,"remaining_kg":s.propellant_mass_kg,"discarded_kg":0.0} for s in self.scenario.vehicle.stages]
        return {"status":status,"termination_reason":termination_reason,"orbit":orbit,"target_comparison":comparisons,
                "summary":{"arrival_time_s":t,"final_mass_kg":final["mass_kg"],"final_altitude_m":final["altitude_m"],
                           "final_inertial_speed_m_s":final["inertial_speed_m_s"],
                           "propellant_burned_kg":sum(l["burned_kg"] for l in ledger),
                           "propellant_remaining_kg":sum(l["remaining_kg"] for l in ledger),
                           "propellant_discarded_kg":sum(l["discarded_kg"] for l in ledger),
                           "max_dynamic_pressure_pa":maxq,"max_load_factor_g":maxload,
                           "max_thrust_velocity_angle_deg":max((r["thrust_velocity_angle_deg"] for r in rows),default=0),
                           "max_thrust_angle_utilization":angle_utilization},
                "constraint_checks":{"dynamic_pressure":q_ok,"load_factor":g_ok,"thrust_angle":angle_ok,"launch_azimuth":azimuth_ok,"departure_corridor":bool(corridor_ok),"bound_orbit_above_100_km":bool(bound_ok),'arrival_window':arrival_ok,'mission_duration':duration_ok},
                "propellant_ledger":ledger,"events":events,"series":rows,
                "actual_orbit_curve":orbit_curve(orbit),"target_orbit_curve":target_curve(target,orbit),
                "flight_plan":plan.model_dump(),"scenario":self.scenario.model_dump(mode="json"),
                "metadata":{"application_version":"0.1.0","units":"SI","frame":"GCRS (inertial), ITRS (Earth-fixed)",
                            "orbit_altitude_reference_m":R_E,"gravity":"WGS84 central + J2","integrator":"DOP853",
                            "atmosphere":self.atmosphere.metadata,"launch_elevation_source":self.scenario.launch.elevation_source,
                            "launch_elevation_resolution_arcsec":self.scenario.launch.elevation_resolution_arcsec,
                            "departure_sector_source":self.scenario.launch.range_source,"departure_sectors_deg":self.scenario.launch.azimuth_sectors_deg,
                            'guidance_frame':plan.guidance_frame,
                            "verified_with_strict_integration":getattr(self,"_strict_run",False)},"warnings":warnings}


def compare_target(orbit,target):
    rows=[]
    for name,wanted,tolerance,angular in [
        ("perigee_altitude_m",target.perigee_altitude_m,target.altitude_tolerance_m,False),
        ("apogee_altitude_m",target.apogee_altitude_m,target.altitude_tolerance_m,False),
        ("inclination_deg",target.inclination_deg,target.angle_tolerance_deg,False),
        ("raan_deg",target.raan_deg,target.angle_tolerance_deg,True),
        ("argument_of_periapsis_deg",target.argument_of_periapsis_deg,target.angle_tolerance_deg,True),
        ("arrival_phase_deg",target.arrival_phase_deg,target.angle_tolerance_deg,True),
    ]:
        if wanted is None:
            continue
        actual=orbit.get(name)
        error=None if actual is None else (wrap_deg(actual-wanted) if angular else actual-wanted)
        rows.append({"parameter":name,"target":wanted,"actual":actual,"error":error,
                     "tolerance":tolerance,"within_tolerance":error is not None and abs(error)<=tolerance})
    return rows
