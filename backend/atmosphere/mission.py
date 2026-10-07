"""Vehicle-constrained mission structure, independent of target-orbit names."""
from dataclasses import dataclass
from itertools import product
import numpy as np

from .earth import MU, R_E
from .models import Burn, FlightPlan
from .screening import departure_options, seed_shape


@dataclass(frozen=True)
class MissionTopology:
    counts: tuple[int, ...]

    @property
    def stages(self):
        return tuple(i for i, count in enumerate(self.counts) for _ in range(count))

    @property
    def name(self):
        return ' / '.join(f'S{i+1}: {n} burn'+('s' if n>1 else '') for i,n in enumerate(self.counts))


def mission_topologies(scenario):
    """Enumerate combinations across ALL stages; select a bounded, diverse portfolio."""
    choices=[range(1,s.ignition_limit+1) if s.engine_type=='liquid' else (1,) for s in scenario.vehicle.stages]
    combinations=list(product(*choices))
    direct=tuple(1 for _ in choices)
    selected=[direct]
    # Reserve a distributed-restart structure before filling individual options.
    distributed=tuple(min(2,max(c)) for c in choices)
    if distributed != direct: selected.append(distributed)
    remaining=sorted((c for c in combinations if c not in selected),key=lambda c:(sum(c),tuple(-n for n in c[::-1])))
    selected.extend(remaining)
    # High final perigees generally benefit from a separated arrival burn.
    # Prioritize these legal structures before truncating the portfolio; keep
    # one-burn alternatives available, and never add ignition capability.
    if (scenario.target.perigee_altitude_m or 0)>2e6:
        preferred=3 if needs_node_transfer(scenario) and max(choices[-1])>=3 else 2
        selected.sort(key=lambda c:(c[-1]<preferred,abs(c[-1]-preferred),sum(c)))
    return [MissionTopology(c) for c in selected[:scenario.constraints.max_topologies]],len(combinations)


def needs_node_transfer(scenario):
    """A high arrival plane differs from the site's direct departure plane."""
    target=scenario.target
    if (target.perigee_altitude_m or 0)<=2e6:return False
    if target.raan_deg is not None:return True
    if target.inclination_deg is None:return False
    departure=departure_options(scenario.launch.latitude_deg,target,scenario.launch.azimuth_sectors_deg)[2]
    return abs(target.inclination_deg-departure)>target.angle_tolerance_deg


def arrival_normal(target,position,velocity):
    from .earth import unit
    normal=unit(np.cross(position,velocity))
    if target.inclination_deg is not None or target.raan_deg is not None:
        inclination=np.radians(target.inclination_deg) if target.inclination_deg is not None else np.arccos(np.clip(normal[2],-1,1))
        raan=np.radians(target.raan_deg) if target.raan_deg is not None else np.arctan2(normal[0],-normal[1])
        normal=np.array([np.sin(inclination)*np.sin(raan),-np.sin(inclination)*np.cos(raan),np.cos(inclination)])
    return normal


def resample_burn(burn, nodes):
    grid=np.linspace(0,1,nodes)
    return burn.model_copy(update={key:tuple(np.interp(grid,np.linspace(0,1,len(getattr(burn,key))),getattr(burn,key)))
                                   for key in ('angle_fractions','clock_angles_rad','throttles')})


def seed_plan(scenario, topology, nodes=None):
    """Physical seed families, never compulsory intermediate orbital targets."""
    nodes=nodes or scenario.constraints.control_nodes
    rp,ra=seed_shape(scenario.target)
    parking=R_E+min(rp,300000)
    parking_half=np.pi*np.sqrt(parking**3/MU)
    transfer_half=np.pi*np.sqrt(((parking+R_E+ra)/2)**3/MU)
    high_circular=rp>2e6
    _,az,insertion_inc=departure_options(scenario.launch.latitude_deg,scenario.target,scenario.launch.azimuth_sectors_deg)
    burns=[]
    for i,count in enumerate(topology.counts):
        stage=scenario.vehicle.stages[i]
        for j in range(count):
            fraction=1 if stage.engine_type=='solid' else (.97 if count==1 else (.60 if j==0 else .30/(count-1)))
            coast=0.
            if j:
                coast=transfer_half if high_circular and i==len(topology.counts)-1 and j==count-1 else (parking_half if i else 10.)
                coast=min(coast,scenario.constraints.max_mission_duration_s*.65/max(1,sum(topology.counts)-len(topology.counts)))
            angles=np.linspace(.25,.025,nodes) if i==0 and j==0 else np.full(nodes,.04)
            clocks=np.zeros(nodes)
            if j and scenario.target.inclination_deg is not None:
                # Both out-of-plane signs remain free; this only initializes one.
                clocks[:]=np.pi/2 if scenario.target.inclination_deg>insertion_inc else -np.pi/2
                limit=stage.space_thrust_velocity_angle_deg
                if limit and abs(scenario.target.inclination_deg-insertion_inc)>.1: angles[:]=min(.5,20/limit)
            burns.append(Burn(stage_index=i,duration_s=stage.full_burn_s*fraction/(1 if stage.engine_type=='solid' else stage.throttle_max),
                              coast_before_s=coast,angle_fractions=tuple(angles),clock_angles_rad=tuple(clocks),
                              throttles=(1.,)*nodes if stage.engine_type=='solid' else (stage.throttle_max,)*nodes))
    return FlightPlan(launch_azimuth_deg=az,burns=burns,guidance_frame='local_orbital')


def phase_manifest(scenario,plan):
    phases=[]
    t=0.;active=0
    for i,burn in enumerate(plan.burns):
        if burn.stage_index != active:
            delay=scenario.vehicle.stages[active].separation_delay_s
            phases.append({'kind':'separation','stage_index':active,'time_s':t,'duration_s':delay})
            t+=delay;active=burn.stage_index
        if burn.coast_before_s:
            phases.append({'kind':'coast','stage_index':active,'time_s':t,'duration_s':burn.coast_before_s})
            t+=burn.coast_before_s
        phases.append({'kind':'burn','stage_index':active,'time_s':t,'duration_s':burn.duration_s,'control_nodes':len(burn.throttles),
                       'steering_reference':burn.steering_reference})
        t+=burn.duration_s
    if plan.final_coast_s:phases.append({'kind':'arrival_coast','stage_index':active,'time_s':t,'duration_s':plan.final_coast_s})
    return phases


def seed_from_ascent(scenario,topology,ascent,parking=False):
    """Lift a continuous ascent into any legal restart structure.

    A parking-orbit seed is merely initialization; the NLP never receives a
    parking-orbit boundary constraint. All intermediate states remain free.
    """
    from .earth import G0
    if ascent.stage_masses is not None:
        from .sizing import apply_stage_masses
        scenario=apply_stage_masses(scenario,ascent.stage_masses)
    burns=[];last=len(scenario.vehicle.stages)-1
    _,ra=seed_shape(scenario.target)
    rp=R_E+200000;destination=R_E+ra;a=(rp+destination)/2
    transfer_coast=np.pi*np.sqrt(a**3/MU)
    for i,count in enumerate(topology.counts):
        original=resample_burn(ascent.burns[i],scenario.constraints.control_nodes)
        stage=scenario.vehicle.stages[i]
        if i==last and parking and count>=2:
            consumed=original.duration_s*stage.mass_flow_kg_s*np.mean(original.throttles)
            reserve=max(0,stage.propellant_mass_kg-consumed)
            mass=scenario.vehicle.payload_mass_kg+stage.dry_mass_kg+reserve
            injection_dv=max(0,np.sqrt(MU*(2/rp-1/a))-np.sqrt(MU/rp))
            injection_fuel=mass*(1-np.exp(-injection_dv/(G0*stage.isp_vacuum_s)))
            injection_duration=min(reserve*.85,injection_fuel)/stage.mass_flow_kg_s/stage.throttle_max
            circular_dv=max(0,np.sqrt(MU/destination)-np.sqrt(MU*(2/destination-1/a)))
            circular_fuel=max(0,mass-injection_fuel)*(1-np.exp(-circular_dv/(G0*stage.isp_vacuum_s)))
            circular_duration=max(.01,min(max(0,reserve-injection_fuel)*.95,circular_fuel)/stage.mass_flow_kg_s/stage.throttle_max)
            prograde={'angle_fractions':(0.,)*len(original.throttles),'clock_angles_rad':(0.,)*len(original.throttles),'throttles':(stage.throttle_max,)*len(original.throttles)}
            if count==2:
                burns.append(original.model_copy(update={'duration_s':original.duration_s+injection_duration}))
            else:
                burns.append(original)
                burns.append(original.model_copy(update={**prograde,'duration_s':max(.01,injection_duration),'coast_before_s':0.}))
            burns.append(original.model_copy(update={**prograde,'duration_s':circular_duration,'coast_before_s':min(transfer_coast,scenario.constraints.max_mission_duration_s*.7)}))
            while sum(b.stage_index==i for b in burns)<count:
                burns.append(original.model_copy(update={**prograde,'duration_s':.01,'coast_before_s':0.}))
        else:
            # Splitting a burn does not create free fuel or additional vehicles.
            for j in range(count):
                begin=j/count;end=(j+1)/count;grid=np.linspace(begin,end,len(original.throttles))
                controls={key:tuple(np.interp(grid,np.linspace(0,1,len(getattr(original,key))),getattr(original,key)))
                          for key in ('angle_fractions','clock_angles_rad','throttles')}
                burns.append(original.model_copy(update={**controls,'duration_s':original.duration_s/count,'coast_before_s':original.coast_before_s if j==0 else 0.}))
    return FlightPlan(stage_masses=ascent.stage_masses,launch_azimuth_deg=ascent.launch_azimuth_deg,burns=burns,guidance_frame=ascent.guidance_frame)


def time_to_apogee(position,velocity):
    """Forward osculating two-body coast estimate, not a flight constraint."""
    r=np.linalg.norm(position);energy=np.dot(velocity,velocity)/2-MU/r
    if energy>=0:raise ValueError('Transfer seed must have a bound orbit.')
    a=-MU/(2*energy)
    eccentricity=np.linalg.norm(np.cross(velocity,np.cross(position,velocity))/MU-position/r)
    n=np.sqrt(MU/a**3)
    if eccentricity<1e-7:return np.pi/n
    cosine=np.clip((1-r/a)/eccentricity,-1,1)
    sine=np.dot(position,velocity)/(eccentricity*np.sqrt(MU*a))
    anomaly=np.arctan2(sine,cosine)
    mean=anomaly-eccentricity*np.sin(anomaly)
    return float(((np.pi-mean)%(2*np.pi))/n)


def seed_from_transfer(sim,topology,ascent,terminal_state,terminal_time):
    """Preserve the ascent; initialize a legal finite arrival maneuver.

    The coast is propagated with production gravity and frames. Inertial
    steering avoids reversal of the Earth-relative velocity at high apogee;
    the production thrust cone still applies to every commanded direction.
    All coast times, controls and intermediate states remain NLP variables.
    """
    from .earth import G0
    from .dynamics import thrust_direction,unit
    scenario=sim.scenario;last=len(scenario.vehicle.stages)-1
    stage=scenario.vehicle.stages[last];count=topology.counts[last]
    if count<2 or stage.engine_type!='liquid':
        return seed_from_ascent(scenario,topology,ascent)
    # Split existing ascent controls without stretching them or adding fuel.
    split=MissionTopology((*topology.counts[:-1],count-1))
    seed=seed_from_ascent(scenario,split,ascent)
    coast=min(time_to_apogee(terminal_state[:3],terminal_state[3:6]),
              max(0,scenario.constraints.max_mission_duration_s-terminal_time)*.8)
    segment=sim.integrate(terminal_state,terminal_time,coast,last,azimuth=ascent.launch_azimuth_deg)
    if segment.status!='Complete':raise ValueError('Transfer coast seed terminated before arrival.')
    state=segment.state;position=state[:3];velocity=state[3:6];radius=np.linalg.norm(position)
    rp,ra=seed_shape(scenario.target);a=(2*R_E+rp+ra)/2
    speed=np.sqrt(max(0,MU*(2/radius-1/a)))
    normal=arrival_normal(scenario.target,position,velocity)
    desired=speed*unit(np.cross(normal,position))
    earth_velocity=sim.frames.earth_velocity(segment.time,position)
    if (scenario.target.inclination_deg is not None and scenario.target.raan_deg is None
            and min(scenario.target.inclination_deg,180-scenario.target.inclination_deg)<1e-6
            and np.linalg.norm(desired-earth_velocity)<1.5):
        # At a synchronous arrival, forcing an exact pole can create a zero
        # Earth-relative speed and activate the explicit vertical lock. Seed
        # an allowed plane inside the user's unchanged inclination tolerance.
        # The optimizer may change it freely; this is not a new target.
        pole=normal.copy();axis=unit(np.cross(unit(position),pole))
        tilt=np.radians(scenario.target.angle_tolerance_deg*.5)
        options=[speed*unit(np.cross(unit(pole*np.cos(tilt)+sign*axis*np.sin(tilt)),position)) for sign in (-1,1)]
        desired=max(options,key=lambda candidate:np.linalg.norm(candidate-earth_velocity))
    delta=desired-velocity;dv=np.linalg.norm(delta)
    mass=scenario.vehicle.payload_mass_kg+stage.dry_mass_kg+state[6]
    fuel=mass*(-np.expm1(-dv/(G0*stage.isp_vacuum_s)))
    duration=max(.01,min(state[6]*.98,fuel)/stage.mass_flow_kg_s/stage.throttle_max)
    rotation=sim.frames.matrix(segment.time)
    lat,lon,_=sim.frames.geographic(segment.time,position);lat,lon=np.radians([lat,lon])
    up=rotation@np.array([np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)])
    north=rotation@np.array([-np.sin(lat)*np.cos(lon),-np.sin(lat)*np.sin(lon),np.cos(lat)])
    east=rotation@np.array([-np.sin(lon),np.cos(lon),0])
    heading=np.cos(np.radians(ascent.launch_azimuth_deg))*north+np.sin(np.radians(ascent.launch_azimuth_deg))*east
    direction=unit(delta) if dv>1e-6 else unit(velocity)
    forward=unit(velocity)
    radial,_=thrust_direction(velocity,up,heading,np.pi/2,0,ascent.guidance_frame)
    lateral,_=thrust_direction(velocity,up,heading,np.pi/2,np.pi/2,ascent.guidance_frame)
    angle=np.arctan2(np.linalg.norm(np.cross(forward,direction)),np.dot(forward,direction))/np.pi
    clock=np.arctan2(np.dot(direction,lateral),np.dot(direction,radial))
    nodes=scenario.constraints.control_nodes
    arrival=Burn(stage_index=last,duration_s=duration,coast_before_s=coast,steering_reference='inertial_velocity',
                 angle_fractions=(float(angle),)*nodes,clock_angles_rad=(float(clock),)*nodes,throttles=(stage.throttle_max,)*nodes)
    return seed.model_copy(update={'burns':[*seed.burns,arrival]})


def seed_from_parking(sim,topology,ascent):
    """Initialize a legal node-aligned injection and finite plane/arrival burn.

    Waiting in the parking orbit lets transfer apogee lie in the destination
    plane. A local fit places that apogee using production propagation. These
    are seed choices only: no parking orbit, node or apogee boundary is imposed
    on the complete-mission NLP, and no extra ignition capability is added.
    """
    from scipy.optimize import brentq, least_squares
    from .earth import G0, unit
    from .dynamics import thrust_direction
    from .restoration import SegmentReplayCache
    last=len(sim.scenario.vehicle.stages)-1;stage=sim.scenario.vehicle.stages[last]
    if topology.counts[last]!=3 or stage.engine_type!='liquid':
        raise ValueError('Node-aligned parking initialization needs three liquid upper-stage ignitions.')
    trial=SegmentReplayCache().simulation(sim,sim.scenario,sim.check)
    parking=trial.run(ascent.model_copy(update={'stage_masses':None}),strict=False,record=False)
    if not parking['orbit']['bound'] or parking['orbit']['perigee_altitude_m']<100000:
        raise ValueError('Parking initialization did not reach a safe bound orbit.')
    state=trial._last_state.copy();terminal=parking['summary']['arrival_time_s']
    normal=arrival_normal(sim.scenario.target,state[:3],state[3:6])
    period=2*np.pi*np.sqrt(parking['orbit']['semi_major_axis_m']**3/MU)
    available=max(0,sim.scenario.constraints.max_mission_duration_s-terminal)
    if available<period:raise ValueError('Insufficient mission duration for node-aligned initialization.')
    def node_value(dt):
        segment=trial.integrate(state,terminal,float(dt),last,azimuth=ascent.launch_azimuth_deg)
        if segment.status!='Complete':raise ValueError('Parking coast initialization terminated early.')
        return float(np.dot(normal,segment.state[:3]))
    plan=seed_from_ascent(sim.scenario,topology,ascent,parking=True)
    injection=plan.burns[-2]
    if injection.duration_s<.1:raise ValueError('No useful injection fuel in parking initialization.')
    previous=0.;previous_value=float(np.dot(normal,state[:3]));node=None
    for dt in np.linspace(1,period,25):
        value=node_value(dt)
        if value*previous_value<0:
            root=brentq(node_value,previous,dt,xtol=.001)
            if root>injection.duration_s/2:
                node=root;break
        previous=dt;previous_value=value
    if node is None:raise ValueError('No destination-plane crossing found in parking initialization.')
    injection.coast_before_s=float(node-injection.duration_s/2)
    injection.steering_reference='inertial_velocity'
    prefix=plan.model_copy(update={'stage_masses':None,'burns':plan.burns[:-1]})
    _,apogee=seed_shape(sim.scenario.target);destination=R_E+apogee
    def propagate(x):
        candidate=prefix.model_copy(deep=True)
        candidate.burns[-1].coast_before_s=injection.coast_before_s+float(x[0])*50
        candidate.burns[-1].duration_s=injection.duration_s+float(x[1])
        result=trial.run(candidate,strict=False,record=False)
        if result['termination_reason']!='Complete' or not result['orbit']['bound']:
            raise ValueError('Node-aligned injection initialization did not complete in a bound orbit.')
        y=trial._last_state.copy();t=result['summary']['arrival_time_s']
        coast=time_to_apogee(y[:3],y[3:6])
        segment=trial.integrate(y,t,coast,last,azimuth=ascent.launch_azimuth_deg)
        if segment.status!='Complete':raise ValueError('Node-aligned transfer initialization terminated early.')
        return candidate,segment,y,t
    def residual(x):
        _,segment,_,_=propagate(x);position=segment.state[:3]
        return [(np.linalg.norm(position)-destination)/1000,np.dot(normal,position)/1000]
    # Bounds keep this small seed fit near the physical parking departure.
    duration_span=min(5,injection.duration_s*.25)
    coast_lower=max(-4,-injection.coast_before_s/50)
    fit=least_squares(residual,[0.,0.],bounds=([coast_lower,-duration_span],[4,duration_span]),
                      diff_step=2e-4,x_scale='jac',max_nfev=20,gtol=1e-5,ftol=1e-7,xtol=1e-7)
    candidate,segment,y,t=propagate(fit.x)
    arrival=seed_from_transfer(sim,topology,ascent,y,t).burns[-1]
    position=segment.state[:3];velocity=segment.state[3:6];arrival_time=segment.time
    def basis(v,r,time):
        lat,lon,_=sim.frames.geographic(time,r);lat,lon=np.radians([lat,lon]);rotation=sim.frames.matrix(time)
        up=rotation@np.array([np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)])
        north=rotation@np.array([-np.sin(lat)*np.cos(lon),-np.sin(lat)*np.sin(lon),np.cos(lat)])
        east=rotation@np.array([-np.sin(lon),np.cos(lon),0])
        heading=np.cos(np.radians(ascent.launch_azimuth_deg))*north+np.sin(np.radians(ascent.launch_azimuth_deg))*east
        radial,_=thrust_direction(v,up,heading,np.pi/2,0,ascent.guidance_frame)
        lateral,_=thrust_direction(v,up,heading,np.pi/2,np.pi/2,ascent.guidance_frame)
        return up,heading,radial,lateral
    up,heading,_,_=basis(velocity,position,arrival_time)
    direction,_=thrust_direction(velocity,up,heading,arrival.angle_fractions[0]*np.pi,arrival.clock_angles_rad[0],ascent.guidance_frame)
    mass=trial.base_mass(last)+segment.state[6];flow=stage.mass_flow_kg_s*stage.throttle_max
    angles=[];clocks=[]
    # Follow an approximately fixed inertial delta-v direction as velocity
    # changes. A constant angle to velocity rotates the intended maneuver and
    # can leave a large plane/energy error even with sufficient reserved fuel.
    for fraction in np.linspace(0,1,len(arrival.throttles)):
        dt=fraction*arrival.duration_s
        v=velocity+direction*(G0*stage.isp_vacuum_s*np.log(mass/(mass-flow*dt)))
        r=position+.5*(velocity+v)*dt
        _,_,radial,lateral=basis(v,r,arrival_time+dt)
        angles.append(float(np.arctan2(np.linalg.norm(np.cross(unit(v),direction)),np.dot(unit(v),direction))/np.pi))
        clocks.append(float(np.arctan2(np.dot(direction,lateral),np.dot(direction,radial))))
    arrival.angle_fractions=tuple(angles);arrival.clock_angles_rad=tuple(np.unwrap(clocks))
    return FlightPlan(stage_masses=ascent.stage_masses,launch_azimuth_deg=ascent.launch_azimuth_deg,
                      guidance_frame=ascent.guidance_frame,burns=[*candidate.burns,arrival])
