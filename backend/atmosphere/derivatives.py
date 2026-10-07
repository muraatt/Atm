"""Differentiable production node equations and exact environment lookup slopes.

Astropy's existing quaternion table and the atmosphere provider's existing cells
are retained. No altitude-only atmosphere or spherical-Earth surrogate is used.
The separate NumPy/DOP853 implementation remains the acceptance authority.
"""
from itertools import count

import casadi as ca
import numpy as np

from .dynamics import Cancelled, SolverDeadline
from .environment import EnvironmentError
from .earth import E2, FLATTENING, G0, J2, MU, R_E, V_SCALE

_names = count()


class LookupJacobian(ca.Callback):
    def __init__(self, parent, name):
        ca.Callback.__init__(self)
        self.parent = parent
        self.construct(name, {})

    def get_n_in(self): return 2
    def get_n_out(self): return 1
    def get_sparsity_in(self, i):
        return ca.Sparsity.dense(self.parent.nin, 1) if i == 0 else ca.Sparsity(self.parent.nout, 1)
    def get_sparsity_out(self, i): return ca.Sparsity.dense(self.parent.nout, self.parent.nin)
    def eval(self, args): return [ca.DM(self.parent.evaluate(np.asarray(args[0]).ravel())[1])]


class ExactLookup(ca.Callback):
    def __init__(self, owner, nin, nout, name):
        ca.Callback.__init__(self)
        self.owner, self.nin, self.nout = owner, nin, nout
        self.last = None
        self.cached = None
        self.jacobians = []
        self.calls = 0
        self.construct(f'{name}_{next(_names)}', {})

    def get_n_in(self): return 1
    def get_n_out(self): return 1
    def get_sparsity_in(self, i): return ca.Sparsity.dense(self.nin, 1)
    def get_sparsity_out(self, i): return ca.Sparsity.dense(self.nout, 1)
    def has_jacobian(self): return True
    def get_jacobian(self, name, inames, onames, opts):
        jacobian = LookupJacobian(self, name)
        self.jacobians.append(jacobian)  # callbacks must outlive the CasADi graph
        return jacobian

    def evaluate(self, value):
        if self.owner.interrupted is not None:
            return self.fallback(), np.zeros((self.nout, self.nin))
        try:
            self.owner.sim.check()
            if self.last is None or not np.array_equal(value, self.last):
                self.cached = self.lookup(value)
                self.last = value.copy()
                self.calls += 1
            return self.cached
        except (Cancelled, SolverDeadline, EnvironmentError) as exc:
            self.owner.interrupted = exc
            return self.fallback(), np.zeros((self.nout, self.nin))

    def fallback(self): return np.zeros(self.nout)

    def eval(self, args): return [ca.DM(self.evaluate(np.asarray(args[0]).ravel())[0])]


class FrameLookup(ExactLookup):
    def __init__(self, owner): super().__init__(owner, 1, 7, 'earth_frame')

    def lookup(self, value):
        frames = self.owner.sim.frames
        t = max(0., value[0]*1000)
        index = min(len(frames._quaternions)-2, int(t/frames._step))
        fraction = (t-index*frames._step)/frames._step
        delta = frames._quaternions[index+1]-frames._quaternions[index]
        # Normalization and its derivative are performed in the symbolic graph,
        # exactly as EarthFrames.matrix normalizes its linear quaternion blend.
        quat = frames._quaternions[index]+fraction*delta
        omega = frames._angular[min(len(frames._angular)-1, index)]
        gradient = np.r_[delta*1000/frames._step, np.zeros(3)][:, None]
        if value[0] < 0: gradient[:] = 0
        return np.r_[quat, omega], gradient


class AtmosphereLookup(ExactLookup):
    def __init__(self, owner): super().__init__(owner, 4, 6, 'atmosphere')

    def fallback(self): return np.array([0., 0., 1., 0., 0., 0.])

    def lookup(self, value):
        air, jac = self.owner.sim.atmosphere.sample_with_derivatives(*value)
        result = np.r_[air.density_kg_m3, air.pressure_pa, air.sound_speed_m_s or 1., air.wind_ecef_m_s]
        if (not np.all(np.isfinite(result)) or not np.all(np.isfinite(jac)) or
                result[0] < 0 or result[1] < 0 or result[2] <= 0):
            raise EnvironmentError('Atmosphere returned invalid node values or derivatives.')
        return result, jac


def norm(vector): return ca.sqrt(ca.fmax(ca.dot(vector, vector), 1e-30))
def unit(vector): return vector/norm(vector)
def clip(value, low, high): return ca.fmin(high, ca.fmax(low, value))


def table(value, rows):
    """The same piecewise-linear interpolation and endpoint clamp as np.interp."""
    result = rows[0][1]
    for (a, av), (b, bv) in zip(rows, rows[1:]):
        result += (bv-av)*clip((value-a)/(b-a), 0, 1)
    return result


def rotation(quaternion):
    x, y, z, w = ca.vertsplit(unit(quaternion))
    return ca.vertcat(ca.horzcat(1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                      ca.horzcat(2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                      ca.horzcat(2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def geographic(fixed):
    x, y, z = ca.vertsplit(fixed)
    p = ca.sqrt(ca.fmax(x*x+y*y, 1e-30))
    lat = ca.atan2(z, p*(1-E2))
    for _ in range(5):
        n = R_E/ca.sqrt(1-E2*ca.sin(lat)**2)
        lat = ca.atan2(z+E2*n*ca.sin(lat), p)
    n = R_E/ca.sqrt(1-E2*ca.sin(lat)**2)
    height = p*ca.cos(lat)+z*ca.sin(lat)-n*(1-E2*ca.sin(lat)**2)
    polar = p < 1e-6
    return (ca.if_else(polar, ca.sign(z)*np.pi/2, lat),
            ca.if_else(polar, 0, ca.atan2(y, x)),
            ca.if_else(polar, ca.fabs(z)-R_E*(1-FLATTENING), height))


def node_function(owner, stage_index, powered, first=False,steering_reference='air_relative'):
    """13 inputs, 11 outputs, matching collocation.NodeFunction at valid states."""
    scenario = owner.sim.scenario
    stage = scenario.vehicle.stages[stage_index]
    sizing=getattr(owner,'design_enabled',False)
    z = ca.MX.sym('node', 13+(2*len(scenario.vehicle.stages) if sizing else 0))
    y = z[:7]*ca.DM(owner.scale(stage_index))
    t = ca.fmax(0, z[7]*1000)
    frame = owner.frame_lookup(z[7])
    rot = rotation(frame[:4]); omega = frame[4:]
    lat, lon, height = geographic(rot.T@y[:3])
    distance = norm(y[:3]-ca.DM(owner.sim.initial[:3]))
    launch_height = scenario.launch.elevation_m+scenario.launch.geoid_height_m
    floor = ca.if_else(distance < 50000, launch_height, 0.)
    air = owner.atmosphere_lookup(ca.vertcat(lat*180/np.pi, lon*180/np.pi, ca.fmax(height, floor), t))
    up = rot@ca.vertcat(ca.cos(lat)*ca.cos(lon), ca.cos(lat)*ca.sin(lon), ca.sin(lat))
    north = rot@ca.vertcat(-ca.sin(lat)*ca.cos(lon), -ca.sin(lat)*ca.sin(lon), ca.cos(lat))
    east = rot@ca.vertcat(-ca.sin(lon), ca.cos(lon), 0)
    heading = ca.cos(z[11]*np.pi)*north+ca.sin(z[11]*np.pi)*east
    earth_velocity = ca.cross(omega, y[:3])
    relative = y[3:6]-earth_velocity-rot@air[3:]
    speed = norm(relative)
    inertial=steering_reference=='inertial_velocity'
    guide_velocity=y[3:6] if inertial else relative
    forward = unit(guide_velocity)
    cd = table(speed/air[2], stage.cd_table) if stage.cd_table else stage.cd
    q = .5*air[0]*speed**2
    drag = -q*cd*stage.reference_area_m2*unit(relative)
    direction = up; thrust = 0.; mdot = 0.
    space_angle = stage.space_thrust_velocity_angle_deg
    angle_limit = stage.max_thrust_velocity_angle_deg
    if space_angle is not None:
        angle_limit += (space_angle-angle_limit)*clip((height-100000)/20000, 0, 1)
    if powered:
        fraction = clip(z[12], 0, 1)
        throttle = 1. if stage.engine_type == 'solid' else z[10]
        shape = table(fraction, stage.solid_burn_curve) if stage.engine_type == 'solid' and stage.solid_burn_curve else 1.
        mdot = stage.mass_flow_kg_s*throttle*shape
        isp = stage.isp_vacuum_s-(stage.isp_vacuum_s-stage.isp_sea_level_s)*clip(air[1]/101325, 0, 1)
        thrust = mdot*G0*isp
        vertical = -up+ca.dot(up, forward)*forward
        heading_projection = heading-ca.dot(heading, forward)*forward
        blend = clip((norm(vertical)-np.sin(np.radians(5)))/(np.sin(np.radians(15))-np.sin(np.radians(5))), 0, 1)
        blend = blend*blend*(3-2*blend)
        mixed = (1-blend)*unit(heading_projection)+blend*unit(vertical)
        mixed = ca.if_else(norm(mixed) < 1e-8, ca.if_else(norm(vertical) > 1e-6, vertical, heading_projection), mixed)
        down = unit(mixed)
        if owner.seed_plan.guidance_frame == 'launch_heading':
            down = unit(ca.if_else(norm(heading_projection) < 1e-6, vertical, heading_projection))
        normal = unit(ca.cross(forward, down))
        angle = (np.pi if inertial else angle_limit*np.pi/180)*z[8]
        lateral = ca.cos(z[9])*down+ca.sin(z[9])*normal
        direction = unit(ca.cos(angle)*forward+ca.sin(angle)*lateral)
        if first and stage_index == 0:
            vertical_forward=unit(relative)
            toward_up = up-ca.dot(up, vertical_forward)*vertical_forward
            # atan2 is equivalent to acos for unit vectors and has no infinite
            # derivative at exactly vertical flight.
            deviation = ca.atan2(norm(ca.cross(vertical_forward, up)), ca.dot(vertical_forward, up))
            allowed = angle_limit*np.pi/180
            command = ca.if_else(norm(toward_up) < 1e-12, vertical_forward,
                                 unit(ca.cos(ca.fmin(allowed, deviation))*vertical_forward+ca.sin(ca.fmin(allowed, deviation))*unit(toward_up)))
            blend = clip((t-scenario.constraints.vertical_ascent_s)/10, 0, 1)
            vertical_command = unit((1-blend)*command+blend*direction)
            direction = ca.if_else(ca.logic_and(height < 100000, t < scenario.constraints.vertical_ascent_s+10), vertical_command, direction)
        if inertial:
            direction=ca.if_else(norm(guide_velocity)<1,up,direction)
            actual_forward=unit(relative)
            actual_angle=ca.atan2(norm(ca.cross(actual_forward,direction)),ca.dot(actual_forward,direction))
            lateral=direction-ca.dot(direction,actual_forward)*actual_forward
            fallback=up-ca.dot(up,actual_forward)*actual_forward
            fallback=ca.if_else(norm(fallback)<1e-12,heading-ca.dot(heading,actual_forward)*actual_forward,fallback)
            lateral=ca.if_else(norm(lateral)<1e-12,fallback,lateral)
            allowed=angle_limit*np.pi/180
            projected=unit(ca.cos(allowed)*actual_forward+ca.sin(allowed)*unit(lateral))
            direction=ca.if_else(actual_angle>allowed,projected,direction)
        direction = ca.if_else(speed < 1, up, direction)
    if sizing:
        count=len(scenario.vehicle.stages)
        dry=z[13:13+count]*ca.DM(owner.dry_scales)
        fuel=z[13+count:]*ca.DM(owner.fuel_scales)
        base=scenario.vehicle.payload_mass_kg+ca.sum1(dry[stage_index:])+ca.sum1(fuel[stage_index+1:])
    else:base=owner.sim.base_mass(stage_index)
    mass = base+ca.fmax(0, y[6])
    proper = (thrust*direction+drag)/mass
    radius = norm(y[:3]); fixed = rot.T@y[:3]
    z2 = (fixed[2]/radius)**2
    perturbation = 1.5*J2*MU*R_E**2/radius**5*fixed*ca.vertcat(5*z2-1, 5*z2-1, 5*z2-3)
    acceleration = -MU*y[:3]/radius**3+rot@perturbation+proper
    if first and powered and stage_index == 0:
        support = ca.fmax(0, ca.dot(ca.cross(omega, earth_velocity)-acceleration, up))
        longitude_error = ca.fmod(lon*180/np.pi-scenario.launch.longitude_deg+540, 360)-180
        pad = ca.logic_and(height <= launch_height+.05, speed < 1)
        pad = ca.logic_and(pad, ca.fabs(lat*180/np.pi-scenario.launch.latitude_deg) < 1e-4)
        pad = ca.logic_and(pad, ca.fabs(longitude_error) < 1e-4)
        support = ca.if_else(pad, support, 0)
        acceleration += support*up; proper += support*up
    corridor = 180.
    if stage_index == 0 and scenario.launch.azimuth_sectors_deg:
        lat0, lon0 = np.radians([scenario.launch.latitude_deg, scenario.launch.longitude_deg])
        delta = lon-lon0
        north_bearing = np.cos(lat0)*ca.sin(lat)-np.sin(lat0)*ca.cos(lat)*ca.cos(delta)
        east_bearing = ca.sin(delta)*ca.cos(lat)
        bearing = ca.atan2(east_bearing, north_bearing)*180/np.pi
        margins = []
        for a, b in scenario.launch.azimuth_sectors_deg:
            if b-a == 360:
                margins = [180.]
                break
            width = (b-a)%360
            center = a+width/2
            error = ca.atan2(ca.sin((bearing-center)*np.pi/180), ca.cos((bearing-center)*np.pi/180))*180/np.pi
            margins.append(width/2-ca.fabs(error))
        corridor = margins[0]
        for margin in margins[1:]: corridor = ca.fmax(corridor, margin)
        corridor = ca.if_else(ca.sqrt(north_bearing**2+east_bearing**2+1e-30)*R_E > 1000, corridor, 180.)
    derivative = ca.vertcat(y[3:6], acceleration, -mdot)*1000/ca.DM(owner.scale(stage_index))
    result = ca.vertcat(derivative, (height-floor)/R_E, q/100000, norm(proper)/G0/10, corridor/180)
    return ca.Function(f'exact_node_{next(_names)}', [z], [result], {'cse': True})

