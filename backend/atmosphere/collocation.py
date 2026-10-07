"""Sparse Hermite–Simpson transcription of the production flight dynamics.

Production equations use automatic derivatives plus exact slopes of the existing
Astropy frame table and atmosphere interpolator. Custom atmosphere providers can
still use the guarded finite-difference node. Strict DOP853 replay is independent.
"""
from itertools import count
import casadi as ca
import numpy as np

from .dynamics import Cancelled, NumericalFailure, SolverDeadline, compare_target
from .environment import EnvironmentError
from .earth import MU, R_E, V_SCALE, targeted_elements
from .models import Burn, FlightPlan
from .screening import azimuth_bounds, seed_shape

_ids=count()


class NodeFunction(ca.Callback):
    def __init__(self,owner,stage,powered,first=False,steering_reference='air_relative'):
        ca.Callback.__init__(self)
        self.owner,self.stage,self.powered,self.first=owner,stage,powered,first
        self.steering_reference=steering_reference
        self.construct(f'node_{next(_ids)}',{'enable_fd':True,'fd_method':'central'})
    def get_n_in(self):return 1
    def get_n_out(self):return 1
    def get_sparsity_in(self,i):return ca.Sparsity.dense(13+(2*len(self.owner.sim.scenario.vehicle.stages) if self.owner.design_enabled else 0),1)
    def get_sparsity_out(self,i):return ca.Sparsity.dense(11,1)
    def eval(self,args):
        owner=self.owner
        if owner.interrupted is not None:return [ca.DM.zeros(11)]
        try:
            owner.sim.check()
            z=np.asarray(args[0]).reshape(-1)
            stage=owner.sim.scenario.vehicle.stages[self.stage]
            y=z[:7]*owner.scale(self.stage);t=max(0,z[7]*1000)
            control=z[8:11];az=z[11]*180;fraction=z[12]
            burn=None
            if self.powered:
                burn=Burn.model_construct(stage_index=self.stage,duration_s=stage.full_burn_s if stage.engine_type=='solid' else 1.,coast_before_s=0.,
                    angle_fractions=(control[0],)*2,clock_angles_rad=(control[1],)*2,throttles=(control[2],)*2,steering_reference=self.steering_reference)
            # Liquid commands are constant in this local node call. Solid curve
            # lookup uses the full phase fraction even at collocation midpoints.
            start=0. if self.first else t-fraction*(stage.full_burn_s if stage.engine_type=='solid' else 1.)
            if owner.design_enabled:
                count=len(owner.sim.scenario.vehicle.stages)
                # Custom-provider finite differences see the same mass model.
                saved=[(s.dry_mass_kg,s.propellant_mass_kg) for s in owner.sim.scenario.vehicle.stages]
                try:
                    for i,s in enumerate(owner.sim.scenario.vehicle.stages):
                        s.dry_mass_kg=float(z[13+i]*owner.dry_scales[i]);s.propellant_mass_kg=float(z[13+count+i]*owner.fuel_scales[i])
                    derivative,row=owner.sim.forces(t,y,self.stage,burn,start,az)
                finally:
                    for s,(dry,fuel) in zip(owner.sim.scenario.vehicle.stages,saved):s.dry_mass_kg=dry;s.propellant_mass_kg=fuel
            else:derivative,row=owner.sim.forces(t,y,self.stage,burn,start,az)
            clearance,_=owner.sim.surface_clearance(t,y[:3])
            corridor=owner.sim.departure_margin([row]) if self.stage==0 else 180.
            return [ca.DM(np.r_[derivative*1000/owner.scale(self.stage),clearance/R_E,
                               row['dynamic_pressure_pa']/100000,row['load_factor_g']/10,corridor/180])]
        except (Cancelled,SolverDeadline,EnvironmentError) as exc:
            owner.interrupted=exc
            return [ca.DM.zeros(11)]
        except (NumericalFailure,ValueError,FloatingPointError):
            owner.invalid_candidates+=1
            # An explicitly infeasible node, not an accepted simulated state.
            return [ca.DM([0]*7+[-10,10,10,-10])]


class TerminalFunction(ca.Callback):
    def __init__(self,owner):
        ca.Callback.__init__(self);self.owner=owner
        self.keys=[r['parameter'] for r in compare_target(targeted_elements(owner.sim.initial[:3],owner.sim.initial[3:6],owner.sim.scenario.target),owner.sim.scenario.target)]
        self.construct(f'terminal_{next(_ids)}',{'enable_fd':True,'fd_method':'central'})
    def get_n_in(self):return 1
    def get_n_out(self):return 1
    def get_sparsity_in(self,i):return ca.Sparsity.dense(7,1)
    def get_sparsity_out(self,i):return ca.Sparsity.dense(len(self.keys)+2,1)
    def numeric(self,z):
        y=np.asarray(z).reshape(-1)*self.owner.scale(len(self.owner.sim.scenario.vehicle.stages)-1)
        self.owner.sim.validate_state(0,y)
        if np.linalg.norm(np.cross(y[:3],y[3:6]))<1e-6:
            raise NumericalFailure('Trial orbital plane is undefined.')
        orbit=targeted_elements(y[:3],y[3:6],self.owner.sim.scenario.target)
        rows=compare_target(orbit,self.owner.sim.scenario.target)
        # Terminal errors in tolerance units. Circular orbit eccentricity uses
        # its nonsingular vector below in the feasibility objective.
        errors=[10000 if r['error'] is None else r['error']/r['tolerance'] for r in rows]
        result=np.r_[errors,(orbit['perigee_altitude_m']-100000)/R_E,1-orbit['eccentricity']]
        if not np.all(np.isfinite(result)):raise NumericalFailure('Nonfinite terminal orbital elements.')
        return result
    def eval(self,args):
        try:return [ca.DM(self.numeric(args[0]))]
        except (ValueError,FloatingPointError,NumericalFailure):return [ca.DM([10000]*len(self.keys)+[-10,-10])]


class IterationMonitor(ca.Callback):
    def __init__(self,owner,nx,ng,progress,stop_on_feasible=False):
        ca.Callback.__init__(self);self.owner,self.nx,self.ng,self.progress=owner,nx,ng,progress
        self.calls=0;self.offset=owner.iterations
        self.stop_on_feasible=stop_on_feasible
        self.construct(f'monitor_{next(_ids)}',{})
    def get_n_in(self):return ca.nlpsol_n_out()
    def get_n_out(self):return 1
    def get_name_in(self,i):return ca.nlpsol_out(i)
    def get_name_out(self,i):return 'stop'
    def get_sparsity_in(self,i):
        name=ca.nlpsol_out(i)
        return ca.Sparsity.dense(self.nx if name in ('x','lam_x') else self.ng if name in ('g','lam_g') else 1 if name=='f' else 0,1)
    def get_sparsity_out(self,i):return ca.Sparsity.scalar()
    def eval(self,args):
        owner=self.owner
        try:
            owner.sim.check()
            if owner.interrupted:raise owner.interrupted
            x=np.asarray(args[ca.nlpsol_out().index('x')]).reshape(-1)
            g=np.asarray(args[ca.nlpsol_out().index('g')]).reshape(-1)
            violation=np.maximum(owner.gl-g,0)+np.maximum(g-owner.gu,0)
            error=np.asarray(owner.terminal(x[owner.final_slice])).reshape(-1)[:len(owner.terminal.keys)]
            conditioning=float(owner.conditioning_function(x))
            merit=float(np.linalg.norm(error)+1e6*np.linalg.norm(violation)+np.sqrt(max(0,conditioning)))
            if np.all(np.isfinite(x)) and np.isfinite(merit) and merit<owner.best_merit:
                owner.best_x=x.copy();owner.best_merit=merit
            if (owner.polishing and np.all(np.isfinite(x)) and np.max(violation,initial=0)<=1e-7 and
                    np.max(np.abs(error),initial=0)<=.8+1e-7 and
                    np.all(x>=np.asarray(owner.wl)-1e-7) and np.all(x<=np.asarray(owner.wu)+1e-7)):
                cost=float(args[ca.nlpsol_out().index('f')])
                if np.isfinite(cost) and cost<owner.best_cost:
                    owner.best_cost=cost;owner.best_polish_x=x.copy()
            owner.last_x=x.copy();self.calls+=1;owner.iterations=self.offset+self.calls-1
            self.progress({'iteration':owner.iterations,'target_error_score':float(np.linalg.norm(error)),
                           'constraint_violation':float(np.max(violation,initial=0)),
                           'rejected_candidates':owner.invalid_candidates})
            if (self.stop_on_feasible and np.max(violation,initial=0)<=1e-7 and
                    np.max(np.abs(error),initial=0)<=.8+1e-7 and conditioning<=1e-7 and np.all(np.isfinite(x)) and
                    np.all(x>=np.asarray(owner.wl)-1e-7) and np.all(x<=np.asarray(owner.wu)+1e-7)):
                owner.feasibility_ready=True;owner.best_x=x.copy()
                return [1]
            return [0]
        except (Cancelled,SolverDeadline,EnvironmentError) as exc:
            owner.interrupted=exc
            return [1]


class CollocationProblem:
    def __init__(self,sim,plan,segments=8,analytic=True):
        self.sim,self.seed_plan,self.segments=sim,plan,segments
        self.sim._guidance_frame=plan.guidance_frame
        self.interrupted=None;self.invalid_candidates=0;self.iterations=0
        self.best_x=None;self.last_x=None;self.best_merit=np.inf
        self.polishing=False;self.best_cost=np.inf;self.best_polish_x=None
        self.feasibility_ready=False
        self.w=[];self.w0=[];self.wl=[];self.wu=[];self.g=[];self.gl=[];self.gu=[]
        self.callbacks=[];self.records=[];self.node_variables=0;self.state_slices=[]
        self.coast_advance=[]
        self.design_enabled=sim.scenario.vehicle.optimize_stage_masses
        self.design_slices=None
        self.dry_scales=np.array([s.dry_mass_kg for s in sim.scenario.vehicle.stages])
        self.fuel_scales=np.array([s.propellant_mass_kg for s in sim.scenario.vehicle.stages])
        self.seed_sim=sim.for_design(plan.stage_masses)
        self.seed_sim._guidance_frame=plan.guidance_frame
        self.analytic=analytic and hasattr(sim.atmosphere,'sample_with_derivatives')
        if self.analytic:
            from .derivatives import FrameLookup, AtmosphereLookup
            self.frame_lookup=FrameLookup(self);self.atmosphere_lookup=AtmosphereLookup(self)
            self.callbacks.extend([self.frame_lookup,self.atmosphere_lookup])
        self.build()
    def scale(self,stage):return np.r_[[R_E]*3,[V_SCALE]*3,self.sim.scenario.vehicle.stages[stage].propellant_mass_kg]
    def fuel_upper(self,stage):
        return float(self.mass_limits[stage].propellant_max_kg/self.fuel_scales[stage]) if self.design_enabled and self.sim.scenario.vehicle.stages[stage].engine_type=='liquid' else 1.
    def node_input(self,state,time,control,az,fraction):
        values=ca.vertcat(state,time,control,az,fraction)
        return ca.vertcat(values,self.design_vector) if self.design_enabled else values
    def fuel_path(self,state,stage):
        if self.design_enabled:self.constraint(state[6]-self.fuel_masses[stage]/self.fuel_scales[stage],-np.inf,0)
    def variable(self,name,shape,initial,lower=-np.inf,upper=np.inf):
        var=ca.MX.sym(f'{name}_{next(_ids)}',*shape)
        size=var.numel();begin=len(self.w0)
        self.w.append(ca.vec(var));self.w0.extend(np.asarray(initial).reshape(-1,order='F'))
        self.wl.extend(np.broadcast_to(lower,shape).reshape(-1,order='F'));self.wu.extend(np.broadcast_to(upper,shape).reshape(-1,order='F'))
        return var,slice(begin,begin+size)
    def constraint(self,expression,lower=0,upper=0):
        self.g.append(ca.vec(expression));size=expression.numel()
        self.gl.extend(np.broadcast_to(lower,(size,)));self.gu.extend(np.broadcast_to(upper,(size,)))
    def path(self,node,state):
        self.constraint(node[7],-.1/R_E,np.inf)
        # These algebraic safeguards retain a useful gradient for invalid nodes.
        self.constraint(ca.dot(state[:3],state[:3]),.98**2,np.inf)
        self.constraint(node[10],0,np.inf)
        limits=self.sim.scenario.constraints
        if limits.max_dynamic_pressure_pa is not None:self.constraint(node[8],-np.inf,limits.max_dynamic_pressure_pa/100000)
        if limits.max_load_factor_g is not None:self.constraint(node[9],-np.inf,limits.max_load_factor_g/10)
    def phase(self,start,t,stage,powered,duration,seed_duration,seed_state,controls,first=False):
        self.sim.check()
        reference=controls['burn'].steering_reference if powered else 'air_relative'
        if self.analytic:
            from .derivatives import node_function
            callback=node_function(self,stage,powered,first,reference)
        else:callback=NodeFunction(self,stage,powered,first,reference)
        self.callbacks.append(callback)
        # Include solid profile breakpoints so quadrature cannot skip a pulse.
        grid=np.linspace(0,1,self.segments+1)
        if not powered and seed_duration>0:
            # A transfer coast crosses the fast periapsis arc and then spends
            # much longer near apoapsis. Uniform time intervals under-resolve
            # periapsis even when their terminal algebraic state looks good.
            # Endpoint clustering keeps phase duration a free decision while
            # resolving either end of a general inbound/outbound transfer.
            grid=(1-np.cos(np.pi*grid))/2
        if powered:grid=np.unique(np.r_[grid,np.linspace(0,1,len(controls['seed'][0]))])
        if first and self.sim.frames.geographic(0,self.sim.initial[:3])[2]<120000:
            # Resolve liftoff and the vertical-to-turn transition; a uniform
            # coarse burn grid can otherwise miss both and disagree with replay.
            vertical=self.sim.scenario.constraints.vertical_ascent_s
            transition=np.clip(np.array([vertical-5,vertical,vertical+5,vertical+10])/max(.001,seed_duration),0,1)
            grid=np.unique(np.r_[grid,[.01,.025,.05,.075,.1],transition])
        if powered and self.sim.scenario.vehicle.stages[stage].engine_type=='solid':
            grid=np.unique(np.r_[grid,[x for x,y in self.sim.scenario.vehicle.stages[stage].solid_burn_curve]])
        states=[start];clock=0.;y=seed_state.copy()
        az=self.seed_plan.launch_azimuth_deg
        for j in range(1,len(grid)):
            self.sim.check()
            width=(grid[j]-grid[j-1])*seed_duration
            # Physical state guesses are useful, but never count as acceptance.
            try:
                if powered:
                    burn=controls['burn']
                    # Preserve full phase control/profile time when seeding.
                    from scipy.integrate import solve_ivp
                    def seed_rhs(tau,z):
                        self.sim.check()
                        return self.seed_sim.forces(controls['seed_t']+tau,z,stage,burn,controls['seed_t'],az)[0]
                    out=solve_ivp(seed_rhs,
                                  (clock,clock+width),y,rtol=1e-6,atol=1e-3,method='DOP853',max_step=max(1,min(30,width)))
                    if out.success:y=out.y[:,-1]
                else:
                    out=self.seed_sim.integrate(y,controls['seed_t']+clock,width,stage,azimuth=az)
                    y=out.state
            except NumericalFailure:pass
            clock+=width
            # Keep initial guesses in the bounded numerical domain only. The
            # original physical trajectory is replayed separately by the driver.
            if not np.all(np.isfinite(y)) or np.linalg.norm(y[:3])<.99*R_E:y=seed_state.copy()
            y[6]=max(1e-8,min(y[6],self.seed_sim.scenario.vehicle.stages[stage].propellant_mass_kg))
            rmax=max(3,(R_E+seed_shape(self.sim.scenario.target)[1])*2/R_E)
            state,sl=self.variable('state',(7,1),y/self.scale(stage),np.r_[[-rmax]*3,[-5]*3,0][:,None],np.r_[[rmax]*3,[5]*3,self.fuel_upper(stage)][:,None])
            self.node_variables+=1
            self.state_slices.append(sl)
            states.append(state)
        def control(fraction):
            if not powered:return ca.DM([0,0,0])
            index=min(len(controls['seed'][0])-2,int(fraction*(len(controls['seed'][0])-1)))
            blend=fraction*(len(controls['seed'][0])-1)-index
            return (1-blend)*controls['var'][:,index]+blend*controls['var'][:,index+1]
        for j in range(len(grid)-1):
            self.sim.check()
            a,b=grid[j:j+2];dt=duration*(b-a);left,right=states[j:j+2]
            f0=callback(self.node_input(left,t+duration*a,control(a),self.az,a))
            f1=callback(self.node_input(right,t+duration*b,control(b),self.az,b))
            middle=(left+right)/2+dt*(f0[:7]-f1[:7])/8
            fm=callback(self.node_input(middle,t+duration*(a+b)/2,control((a+b)/2),self.az,(a+b)/2))
            self.constraint(right-left-dt*(f0[:7]+4*fm[:7]+f1[:7])/6)
            if not powered:
                # A free coast duration must not stretch a fixed mesh over
                # unresolved periapsis passages or multiple revolutions.
                # Resolve both translational and gravitational time scales;
                # finer meshes permit longer phases without changing physics.
                for state in (left,middle,right):
                    radius=R_E*ca.sqrt(ca.dot(state[:3],state[:3])+1e-30)
                    speed=V_SCALE*ca.sqrt(ca.dot(state[3:6],state[3:6])+1e-30)
                    rate=ca.fmax(speed/radius,ca.sqrt(MU/radius**3))
                    advance=dt*1000*rate
                    self.coast_advance.append(advance);self.constraint(advance,0,.5)
            for node,state in ((f0,left),(fm,middle),(f1,right)):
                self.path(node,state);self.fuel_path(state,stage)
            self.constraint(middle[6],0,self.fuel_upper(stage))
        return states[-1],t+duration,y
    def build(self):
        scenario=self.sim.scenario
        self.mass_limits=[]
        if self.design_enabled:
            from .sizing import mass_bounds
            self.mass_limits=mass_bounds(scenario)
            dry_lower=[];dry_upper=[];fuel_lower=[];fuel_upper=[]
            for i,(stage,b) in enumerate(zip(scenario.vehicle.stages,self.mass_limits)):
                fixed=stage.engine_type=='solid'
                dry_lower.append(1. if fixed else b.dry_mass_min_kg/self.dry_scales[i]);dry_upper.append(1. if fixed else b.dry_mass_max_kg/self.dry_scales[i])
                fuel_lower.append(1. if fixed else b.propellant_min_kg/self.fuel_scales[i]);fuel_upper.append(1. if fixed else b.propellant_max_kg/self.fuel_scales[i])
            seed_dry=np.array([s.dry_mass_kg for s in self.seed_sim.scenario.vehicle.stages])/self.dry_scales
            seed_fuel=np.array([s.propellant_mass_kg for s in self.seed_sim.scenario.vehicle.stages])/self.fuel_scales
            dry,ds=self.variable('dry_masses',(len(dry_lower),1),seed_dry,np.array(dry_lower)[:,None],np.array(dry_upper)[:,None])
            fuel,fs=self.variable('fuel_loads',(len(fuel_lower),1),seed_fuel,np.array(fuel_lower)[:,None],np.array(fuel_upper)[:,None])
            self.design_slices=(ds,fs);self.design_vector=ca.vertcat(dry,fuel)
            self.dry_masses=dry*ca.DM(self.dry_scales);self.fuel_masses=fuel*ca.DM(self.fuel_scales)
            for i,(stage,b) in enumerate(zip(scenario.vehicle.stages,self.mass_limits)):
                if stage.engine_type=='liquid':self.constraint((self.dry_masses[i]-b.hardware_mass_kg-b.tank_structure_ratio*self.fuel_masses[i])/self.dry_scales[i],0,np.inf)
        bounds,az=azimuth_bounds(scenario.launch,self.seed_plan.launch_azimuth_deg)
        self.az,self.az_slice=self.variable('azimuth',(1,1),[az/180],bounds[0]/180,bounds[1]/180)
        start=ca.DM(self.sim.initial/self.scale(0))
        if self.design_enabled:start=ca.vertcat(start[:6],start[6]*self.fuel_masses[0]/self.fuel_scales[0])
        t=ca.MX(0);seed_t=0.;seed_y=self.seed_sim.initial.copy();active=0
        for i,burn in enumerate(self.seed_plan.burns):
            idx=burn.stage_index;stage=scenario.vehicle.stages[idx]
            if idx!=active:
                # State reset: continuous r/v, discard old reserve, load next fuel.
                start=ca.vertcat(start[:6],self.fuel_masses[idx]/self.fuel_scales[idx] if self.design_enabled else 1.)
                seed_y[6]=self.seed_sim.scenario.vehicle.stages[idx].propellant_mass_kg
                delay=scenario.vehicle.stages[active].separation_delay_s/1000
                if delay:
                    start,t,seed_y=self.phase(start,t,idx,False,delay,delay*1000,seed_y,{'seed_t':seed_t})
                    seed_t+=delay*1000
                active=idx
            coast_slice=None
            if i:
                duration,coast_slice=self.variable('coast',(1,1),[burn.coast_before_s/1000],0,scenario.constraints.max_mission_duration_s/1000)
                start,t,seed_y=self.phase(start,t,idx,False,duration,burn.coast_before_s,seed_y,{'seed_t':seed_t})
                seed_t+=burn.coast_before_s
            fixed=stage.full_burn_s/1000 if stage.engine_type=='solid' else None
            duration,duration_slice=self.variable('burn_duration',(1,1),[burn.duration_s/1000],fixed if fixed else .001,
                                                  fixed if fixed else min(stage.full_burn_s*self.fuel_upper(idx)/stage.throttle_min,scenario.constraints.max_mission_duration_s)/1000)
            controls=np.array([burn.angle_fractions,burn.clock_angles_rad,burn.throttles])
            lower=np.array([0,-np.pi,1 if stage.engine_type=='solid' else stage.throttle_min])[:,None]
            upper=np.array([1,np.pi,1 if stage.engine_type=='solid' else stage.throttle_max])[:,None]
            u,us=self.variable('controls',controls.shape,controls,lower,upper)
            if self.design_enabled and i==0:
                from .sizing import pad_reference
                pad_thrust,weight=pad_reference(self.sim)
                total_mass=scenario.vehicle.payload_mass_kg+ca.sum1(self.dry_masses)+ca.sum1(self.fuel_masses)
                self.constraint(pad_thrust*u[2,0]/(total_mass*weight),scenario.vehicle.minimum_initial_twr,np.inf)
            start,t,seed_y=self.phase(start,t,idx,True,duration,burn.duration_s,seed_y,{'var':u,'seed':controls,'burn':burn,'seed_t':seed_t},i==0)
            seed_t+=burn.duration_s
            self.records.append((idx,duration_slice,coast_slice,us,controls.shape))
        self.final_coast_slice=None
        if scenario.target.arrival_phase_deg is not None or scenario.target.arrival_time_min_s is not None:
            duration,self.final_coast_slice=self.variable('arrival_coast',(1,1),[self.seed_plan.final_coast_s/1000],0,scenario.constraints.max_mission_duration_s/1000)
            start,t,seed_y=self.phase(start,t,active,False,duration,self.seed_plan.final_coast_s,seed_y,{'seed_t':seed_t})
        # Explicit final variable gives the callback a stable state index.
        final,self.final_slice=self.variable('arrival',(7,1),seed_y/self.scale(active),np.r_[[-1000]*3,[-5]*3,0][:,None],np.r_[[1000]*3,[5]*3,self.fuel_upper(active)][:,None])
        self.constraint(final-start)
        self.constraint(t,0,scenario.constraints.max_mission_duration_s/1000)
        if scenario.target.arrival_time_min_s is not None:self.constraint(t,scenario.target.arrival_time_min_s/1000,np.inf)
        if scenario.target.arrival_time_max_s is not None:self.constraint(t,-np.inf,scenario.target.arrival_time_max_s/1000)
        self.terminal=TerminalFunction(self);self.callbacks.append(self.terminal)
        terminal=self.terminal(final)
        self.terminal_errors=terminal[:-2]
        # Stable circular shape residual: eccentricity VECTOR, not an arbitrary
        # periapsis direction or a derivative of |e| at zero eccentricity.
        r=final[:3]*R_E;v=final[3:6]*V_SCALE;h=ca.cross(r,v)
        eccentricity=ca.cross(v,h)/MU-r/ca.sqrt(ca.dot(r,r)+1e-12)
        # Differentiate the terminal bound-orbit safeguards directly as well.
        # The guarded angular callback remains for optional angles/acceptance,
        # but basic energy/shape searches need no terminal finite differences.
        eccentricity_norm=ca.sqrt(ca.fmax(ca.dot(eccentricity,eccentricity),1e-30))
        perigee=ca.dot(h,h)/MU/(1+eccentricity_norm)-R_E
        self.constraint(ca.vertcat((perigee-100000)/R_E,1-eccentricity_norm),0,np.inf)
        target=scenario.target
        other=[]
        if target.inclination_deg is not None:
            normal=h/ca.sqrt(ca.dot(h,h)+1e-12)
            inclination=ca.atan2(ca.sqrt(normal[0]**2+normal[1]**2+1e-16),normal[2])
            tolerance=np.radians(target.angle_tolerance_deg)
            if min(target.inclination_deg,180-target.inclination_deg)<1e-6:
                plane=normal[:2]/tolerance
                length=ca.sqrt(ca.dot(plane,plane)+1e-30)
                margin=np.sin(.8*tolerance)/tolerance
                other.extend(plane[j]*ca.fmax(0,1-margin/length) for j in range(2))
                pole=1 if target.inclination_deg<90 else -1
                other.append(1000*ca.fmax(0,-normal[2]*pole))
            else:
                error=(inclination-np.radians(target.inclination_deg))/tolerance
                other.append(ca.fmax(0,ca.fabs(error)-.8))
        excluded={'inclination_deg'}
        if target.perigee_altitude_m is not None and target.apogee_altitude_m is not None:
            p=ca.dot(h,h)/MU
            rp,ra=R_E+target.perigee_altitude_m,R_E+target.apogee_altitude_m
            shape=[(p-2*rp*ra/(rp+ra))/target.altitude_tolerance_m]
            if abs(rp-ra)<1e-3:shape.extend([eccentricity[j]*rp/target.altitude_tolerance_m for j in range(3)])
            else:shape.append((ca.sqrt(ca.dot(eccentricity,eccentricity)+1e-16)-(ra-rp)/(ra+rp))*R_E/target.altitude_tolerance_m)
            other.extend(shape);excluded.update(('perigee_altitude_m','apogee_altitude_m'))
        other.extend(self.terminal_errors[j] for j,k in enumerate(self.terminal.keys) if k not in excluded)
        # Smooth invariant residuals retain information even for trial escape
        # trajectories where an apogee does not exist. Avoid a log objective
        # whose gradient becomes weak far from the target.
        self.feasibility_objective=sum(x*x for x in other)
        # Inertial arrival at near-zero rotating-Earth speed can cross the
        # explicit 1 m/s vertical-lock discontinuity between sparse nodes.
        # Prefer an allowed plane/phase with a resolved velocity direction.
        # This is a soft search merit, not a new physical flight constraint;
        # fuel/time polishing and final acceptance retain their original rules.
        conditioning=ca.MX(0)
        if self.analytic and self.final_coast_slice is None and self.seed_plan.burns[-1].steering_reference=='inertial_velocity':
            omega=self.frame_lookup(t)[4:]
            relative=v-ca.cross(omega,r)
            relative_speed=ca.sqrt(ca.dot(relative,relative)+1e-30)
            conditioning=10*ca.fmax(0,3-relative_speed)**2
        self.feasibility_objective+=conditioning
        loaded=ca.sum1(self.fuel_masses) if self.design_enabled else sum(s.propellant_mass_kg for s in scenario.vehicle.stages)
        self.cost=(loaded-final[6]*scenario.vehicle.stages[-1].propellant_mass_kg)/scenario.vehicle.initial_mass_kg if scenario.objective=='propellant' else t
        self.x=ca.vertcat(*self.w);self.constraints=ca.vertcat(*self.g)
        self.cost_function=ca.Function(f'cost_{next(_ids)}',[self.x],[self.cost])
        self.conditioning_function=ca.Function(f'conditioning_{next(_ids)}',[self.x],[conditioning])
        self.coast_resolution_function=ca.Function(f'coast_resolution_{next(_ids)}',[self.x],[ca.vertcat(*self.coast_advance)])
        self.constraint_function=ca.Function(f'constraints_{next(_ids)}',[self.x],[self.constraints])
        self.w0=np.clip(self.w0,self.wl,self.wu)
        self.gl,self.gu=np.asarray(self.gl),np.asarray(self.gu)
    def feasible(self,x,tolerance=1e-7):
        values=np.asarray(self.constraint_function(x)).reshape(-1)
        self.sim.check()
        errors=np.asarray(self.terminal(x[self.final_slice])).reshape(-1)
        return (self.interrupted is None and np.all(np.isfinite(values)) and
                np.all(x>=np.asarray(self.wl)-tolerance) and np.all(x<=np.asarray(self.wu)+tolerance) and
                np.max(np.maximum(self.gl-values,0)+np.maximum(values-self.gu,0),initial=0)<=tolerance and
                np.max(np.abs(errors[:-2]),initial=0)<=.8+tolerance)
    def decode(self,x):
        x=np.asarray(x).reshape(-1);burns=[]
        for index,(stage,duration,coast,us,shape) in enumerate(self.records):
            controls=x[us].reshape(shape,order='F')
            burns.append(Burn(stage_index=stage,duration_s=max(.001,float(x[duration][0]*1000)),
                              steering_reference=self.seed_plan.burns[index].steering_reference,
                              coast_before_s=max(0,float(x[coast][0]*1000)) if coast else 0.,
                              angle_fractions=tuple(np.clip(controls[0],0,1)),clock_angles_rad=tuple(controls[1]),
                              throttles=tuple(np.clip(controls[2],self.sim.scenario.vehicle.stages[stage].throttle_min,self.sim.scenario.vehicle.stages[stage].throttle_max))))
        stage_masses=None
        if self.design_enabled:
            from .models import StageMass
            ds,fs=self.design_slices
            stage_masses=[StageMass(dry_mass_kg=float(d),propellant_mass_kg=float(p)) for d,p in zip(x[ds]*self.dry_scales,x[fs]*self.fuel_scales)]
        return FlightPlan(stage_masses=stage_masses,launch_azimuth_deg=float(x[self.az_slice][0]*180),burns=burns,guidance_frame=self.seed_plan.guidance_frame,
                          final_coast_s=max(0,float(x[self.final_coast_slice][0]*1000)) if self.final_coast_slice else 0.)
    def solve(self,progress,max_iterations,seconds,polish=False,options=None):
        self.sim.check()
        self.polishing=polish
        if polish and self.last_x is not None and self.feasible(self.last_x):
            self.best_polish_x=self.last_x.copy();self.best_cost=float(self.cost_function(self.last_x))
        g=self.constraints;gl=self.gl;gu=self.gu
        if polish:
            g=ca.vertcat(g,self.terminal_errors);gl=np.r_[gl,[-.8]*len(self.terminal.keys)];gu=np.r_[gu,[.8]*len(self.terminal.keys)]
        self.feasibility_ready=False
        monitor=IterationMonitor(self,self.x.numel(),g.numel(),progress,stop_on_feasible=not polish);self.callbacks.append(monitor)
        # Monitor uses bounds for both feasibility and objective phases.
        old_gl,old_gu=self.gl,self.gu;self.gl,self.gu=gl,gu
        solver_options={'print_time':False,'error_on_fail':False,'iteration_callback':monitor,
                          'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.max_iter':max_iterations,
                          'ipopt.max_wall_time':max(.01,seconds),'ipopt.hessian_approximation':'limited-memory',
                          'ipopt.mu_strategy':'monotone','ipopt.mu_init':1e-3,
                          # Default 0.01 pushes a zero normalized coast by 10 s
                          # and alters full-throttle/fuel seeds by 1 percent.
                          'ipopt.bound_push':1e-8,'ipopt.bound_frac':1e-8,
                          'ipopt.slack_bound_push':1e-8,'ipopt.slack_bound_frac':1e-8,
                          'ipopt.tol':1e-7,'ipopt.constr_viol_tol':1e-7,'ipopt.bound_relax_factor':0.}
        solver_options.update(options or {})
        solver=ca.nlpsol(f'mission_{next(_ids)}','ipopt',{'x':self.x,'f':self.cost if polish else self.feasibility_objective,'g':g},solver_options)
        try:
            out=solver(x0=self.last_x if polish and self.last_x is not None else self.w0,lbx=self.wl,ubx=self.wu,lbg=gl,ubg=gu)
            stats=solver.stats();x=np.asarray(out['x']).reshape(-1)
            if self.interrupted:raise self.interrupted
            self.last_x=x
            return self.decode(self.best_polish_x if polish and self.best_polish_x is not None else x),stats
        finally:self.gl,self.gu=old_gl,old_gu


def hermite_simpson_defect(y0,y1,f0,fm,f1,dt):
    """Shared reference formula, useful for independent quadrature tests."""
    return np.asarray(y1)-np.asarray(y0)-dt*(np.asarray(f0)+4*np.asarray(fm)+np.asarray(f1))/6
