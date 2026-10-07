"""Bounded continuous shooting restoration after collocation discretization.

This changes actual design/control decisions, never the target or acceptance
tolerances. Every candidate uses the independent production DOP853 equations.
"""
import time
from collections import OrderedDict
from copy import deepcopy
import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.linalg import qr
from .models import Burn, StageMass
from .sizing import apply_stage_masses, mass_bounds, pad_reference
from .dynamics import Simulation, NumericalFailure, SolverDeadline
from .earth import MU, R_E
from .screening import azimuth_bounds


class SegmentReplayCache:
    """Reuse identical strict integrations within one restoration search.

    A changed design, state, command, time, frame or integration setting is a
    different entry. The environment and Earth frames are shared and fixed by
    this search. Copies prevent separation from mutating stored fuel states.
    """
    def __init__(self, capacity=64):
        self.entries=OrderedDict();self.capacity=capacity
        self.hits=0;self.misses=0

    def simulation(self, source, scenario, check):
        cache=self
        design_key=scenario.model_dump_json()
        class CachedSimulation(Simulation):
            def integrate(self,y,t,duration,stage_idx,burn=None,azimuth=90,strict=False,record=False):
                self.check()
                key=(design_key,id(self.atmosphere),id(self.frames),self.initial.tobytes(),
                     np.asarray(y).tobytes(),float(t),float(duration),stage_idx,
                     burn.model_dump_json() if burn is not None else None,float(azimuth),
                     strict,record,self._guidance_frame)
                if key in cache.entries:
                    cache.hits+=1;cache.entries.move_to_end(key)
                    return deepcopy(cache.entries[key])
                segment=super().integrate(y,t,duration,stage_idx,burn,azimuth,strict,record)
                cache.misses+=1;cache.entries[key]=deepcopy(segment)
                if len(cache.entries)>cache.capacity:cache.entries.popitem(last=False)
                return segment
        trial=CachedSimulation(scenario,atmosphere=source.atmosphere,frames=source.frames,check=check)
        trial.initial=source.initial.copy()
        trial.initial[6]*=scenario.vehicle.stages[0].propellant_mass_kg/source.scenario.vehicle.stages[0].propellant_mass_kg
        return trial


def restore_continuous(sim,plan,seconds,progress):
    # Leave time for the driver's final candidate bookkeeping/replay.
    seconds=max(.01,seconds-5)
    start=time.monotonic();calls=0;best=plan;best_score=np.inf;best_result=None;best_x=None;best_reported_error=np.inf
    polishing=False;best_cost=np.inf;cache=None;cost_evaluations=0;newton_iterations=0
    s=sim.scenario;limits=mass_bounds(s);original=plan.stage_masses
    segment_cache=SegmentReplayCache()
    steering_span=.15
    x0=[];lo=[];hi=[];descriptors=[]
    def variable(kind,index,value,lower,upper):
        if upper-lower<1e-12:return
        descriptors.append((kind,index));x0.append(float(value));lo.append(float(lower));hi.append(float(upper))
    if original is not None:
        for i,(mass,stage,bound) in enumerate(zip(original,s.vehicle.stages,limits)):
            if stage.engine_type=='solid':continue
            variable('dry',i,1,max(.8,bound.dry_mass_min_kg/mass.dry_mass_kg),min(1.2,bound.dry_mass_max_kg/mass.dry_mass_kg))
            variable('fuel',i,1,max(.8,bound.propellant_min_kg/mass.propellant_mass_kg),min(1.2,bound.propellant_max_kg/mass.propellant_mass_kg))
    for i,burn in enumerate(plan.burns):
        stage=s.vehicle.stages[burn.stage_index]
        if stage.engine_type=='liquid':
            variable('duration',i,1,.9,1.1)
            variable('throttle',i,1,1+(stage.throttle_min-min(burn.throttles))/.1,
                     1+(stage.throttle_max-max(burn.throttles))/.1)
        if burn.coast_before_s>0:variable('coast',i,1,.9,1.1)
        # Cartesian cone offsets retain a plane-change derivative at zero
        # steering angle, where a polar clock angle has no physical effect.
        # Unit centers also give relative differences a meaningful step.
        variable('radial',i,1,0,2);variable('normal',i,1,0,2)
    if plan.final_coast_s>0:variable('final_coast',0,1,.9,1.1)
    bounds,az=azimuth_bounds(s.launch,plan.launch_azimuth_deg)
    variable('azimuth',0,az/180,max(bounds[0]/180,az/180-.02),min(bounds[1]/180,az/180+.02))
    def decode(x):
        burns=[b.model_copy(deep=True) for b in plan.burns]
        steering=[np.array([np.array(b.angle_fractions)*np.cos(b.clock_angles_rad),
                           np.array(b.angle_fractions)*np.sin(b.clock_angles_rad)]) for b in plan.burns]
        # Preserve isolated zero-angle endpoints. Creating a finite clock at
        # such an endpoint would rotate the interpolated polar command over
        # the preceding interval, despite an arbitrarily tiny cone offset.
        weights=[np.ones(len(b.angle_fractions)) if max(b.angle_fractions)<1e-7
                 else np.minimum(1,np.array(b.angle_fractions)/steering_span) for b in plan.burns]
        design=[m.model_copy() for m in original] if original is not None else None
        azimuth=plan.launch_azimuth_deg
        final_coast=plan.final_coast_s
        for (kind,index),value in zip(descriptors,x):
            if kind=='dry':design[index].dry_mass_kg=original[index].dry_mass_kg*value
            elif kind=='fuel':design[index].propellant_mass_kg=original[index].propellant_mass_kg*value
            elif kind=='azimuth':azimuth=value*180
            elif kind=='duration':burns[index].duration_s=plan.burns[index].duration_s*value
            elif kind=='coast':burns[index].coast_before_s=plan.burns[index].coast_before_s*value
            elif kind=='final_coast':final_coast=plan.final_coast_s*value
            elif kind=='throttle':burns[index].throttles=tuple(np.array(plan.burns[index].throttles)+(value-1)*.1)
            elif kind=='radial':steering[index][0]+=(value-1)*steering_span*weights[index]
            elif kind=='normal':steering[index][1]+=(value-1)*steering_span*weights[index]
        for burn,base,components in zip(burns,plan.burns,steering):
            magnitude=np.hypot(*components)
            burn.angle_fractions=tuple(np.clip(magnitude,0,1))
            burn.clock_angles_rad=tuple(np.where(magnitude<1e-12,base.clock_angles_rad,np.arctan2(components[1],components[0])))
        # model_copy assignments bypass Pydantic scalar conversion. Restore a
        # canonical command here: NumPy azimuths otherwise produce np.bool_
        # verification flags, which cannot be persisted by the job worker.
        candidate=plan.model_copy(update={'stage_masses':design,'burns':burns,'launch_azimuth_deg':azimuth,'final_coast_s':final_coast})
        return type(plan).model_validate(candidate.model_dump())
    def check():
        sim.check()
        if time.monotonic()-start>=seconds:raise SolverDeadline('continuous_restoration')
    circular=s.target.perigee_altitude_m is not None and s.target.apogee_altitude_m is not None and abs(s.target.perigee_altitude_m-s.target.apogee_altitude_m)<1e-3
    equatorial=s.target.inclination_deg is not None and min(s.target.inclination_deg,180-s.target.inclination_deg)<1e-6
    class Ready(Exception):pass
    def evaluate(x):
        nonlocal calls,best,best_score,best_result,best_x,best_cost,cache,cost_evaluations,best_reported_error
        check();candidate=decode(x)
        if cache is not None and np.array_equal(cache['x'],x):return cache['residual']
        sized=s
        if candidate.stage_masses is not None:
            sized=apply_stage_masses(s,candidate.stage_masses,check_bounds=False)
        trial=segment_cache.simulation(sim,sized,check)
        result=trial.run(candidate.model_copy(update={'stage_masses':None}),strict=True,record=True)
        comparisons=result['target_comparison'];residual=[]
        r,v=trial._last_state[:3],trial._last_state[3:6];h=np.cross(r,v)
        if circular:
            evec=np.cross(v,h)/MU-r/np.linalg.norm(r);radial=r/np.linalg.norm(r)
            transverse=np.cross(h/max(1e-12,np.linalg.norm(h)),radial)
            radius=R_E+s.target.perigee_altitude_m
            residual.extend([(np.dot(h,h)/MU-radius)/s.target.altitude_tolerance_m,
                np.dot(evec,radial)*radius/s.target.altitude_tolerance_m,np.dot(evec,transverse)*radius/s.target.altitude_tolerance_m])
        if equatorial:
            normal=h/max(1e-12,np.linalg.norm(h))
            pole=np.array([0.,0.,1. if s.target.inclination_deg<90 else -1.])
            # A scalar inclination is a norm at an equatorial target. Its
            # directional derivative becomes ambiguous exactly where the
            # solver must remove the remaining plane error.
            plane=normal[:2]/np.radians(s.target.angle_tolerance_deg)
            # The requested inclination has a tolerance, not an equality to
            # the pole. In synchronous flight exact zero can activate the
            # Earth-relative 1 m/s vertical lock unnecessarily. Preserve a
            # plane already inside the conservative acceptance margin.
            angular_tolerance=np.radians(s.target.angle_tolerance_deg)
            margin=np.sin(.8*angular_tolerance)/angular_tolerance
            residual.extend(plane*max(0,1-margin/max(1e-12,np.linalg.norm(plane))))
            residual.append(1000*max(0,-np.dot(normal,pole)))
        residual.extend(10000 if row['error'] is None else row['error']/row['tolerance'] for row in comparisons
            if not (circular and row['parameter'] in ('perigee_altitude_m','apogee_altitude_m'))
            and not (equatorial and row['parameter']=='inclination_deg'))
        valid=True
        if candidate.stage_masses is not None:
            for mass,bound,stage in zip(candidate.stage_masses,limits,s.vehicle.stages):
                violation=max(0,bound.hardware_mass_kg+bound.tank_structure_ratio*mass.propellant_mass_kg-mass.dry_mass_kg) if stage.engine_type=='liquid' else 0
                residual.append(1000*violation/max(1,stage.dry_mass_kg));valid&=violation<=1e-6
            thrust,weight=pad_reference(trial)
            twr=thrust*candidate.burns[0].throttles[0]/(trial.scenario.vehicle.initial_mass_kg*weight)
            residual.append(1000*max(0,s.vehicle.minimum_initial_twr-twr));valid&=twr>=s.vehicle.minimum_initial_twr-1e-6
        residual.append(1000 if result['status'] in ('Impact','No Liftoff') else 0)
        for key in ('dynamic_pressure','load_factor','thrust_angle','launch_azimuth','departure_corridor','arrival_window','mission_duration','bound_orbit_above_100_km'):
            residual.append(1000 if result['constraint_checks'].get(key,True) is False else 0)
        score=float(np.linalg.norm(residual));calls+=1
        if valid and all(result['constraint_checks'].values()):
            actual_error=np.linalg.norm([10000 if row['error'] is None else row['error']/row['tolerance'] for row in comparisons])
            best_reported_error=min(best_reported_error,float(actual_error))
        cost=(result['summary']['propellant_burned_kg']+result['summary']['propellant_discarded_kg'])/s.vehicle.initial_mass_kg if s.objective=='propellant' else result['summary']['arrival_time_s']/1000
        if (not polishing and valid and score<best_score) or (polishing and valid and result['status']=='Target Achieved' and cost<best_cost):
            best_score=score;best=candidate;best_result=result;best_x=np.array(x).copy()
            if result['status']=='Target Achieved':best_cost=cost
        cache={'x':np.array(x).copy(),'residual':np.asarray(residual),'result':result,'candidate':candidate,'trial':trial,'cost':cost}
        if polishing:cost_evaluations+=1
        if calls%5==0:progress({'phase':'Minimizing verified continuous mission cost' if polishing else 'Restoring continuous flight accuracy','iteration':calls,
            'target_error_score':score,'score_units':'target tolerances',
            'best_continuous_error':best_reported_error if np.isfinite(best_reported_error) else None,
            'feasible_flight_retained':best_result is not None and best_result['status']=='Target Achieved'})
        if not polishing and valid and result['status']=='Target Achieved':
            best=candidate;best_result=result;best_score=score;best_x=np.array(x).copy();best_cost=cost
            raise Ready()
        return np.asarray(residual)
    dimension=None
    def residual(x):
        nonlocal dimension
        try:
            values=evaluate(x);dimension=len(values);return values
        except NumericalFailure:
            if dimension is None:raise
            return np.full(dimension,10000.)
    def newton_restore(x):
        """Rank-revealing control correction with nonlinear propagation.

        A plane correction can create a second-order energy error larger than
        the starting plane residual. A monotone trust-region search then takes
        tiny steps. Permit a bounded predictor followed by an energy/shape
        correction, retaining the best valid flight independently throughout.
        Mass decisions remain available to the full fit and cost optimizer.
        """
        nonlocal newton_iterations
        values=residual(x)
        # Terminal controls first: their perturbations can reuse the exact
        # ascent and transfer prefix, rather than reintegrating it each time.
        indices=sorted((i for i,(kind,_) in enumerate(descriptors) if kind not in ('dry','fuel')),
                       key=lambda i:(descriptors[i][0]=='azimuth',-descriptors[i][1]))
        for _ in range(6):
            check();columns=[]
            for i in indices:
                step=2e-5*max(1,abs(x[i]))
                step=min(step,hi[i]-x[i]) if hi[i]-x[i]>=x[i]-lo[i] else -min(step,x[i]-lo[i])
                trial=x.copy();trial[i]+=step
                columns.append((residual(trial)-values)/step)
            jac=np.asarray(columns).T
            scales=np.linalg.norm(jac,axis=0)
            usable=scales>1e-10
            if not np.any(usable):break
            active=np.asarray(indices)[usable];matrix=jac[:,usable]
            _,triangular,pivots=qr(matrix/scales[usable],mode='economic',pivoting=True)
            diagonal=np.abs(np.diag(triangular));rank=int(np.sum(diagonal>max(1e-6,diagonal.max(initial=0)*1e-6)))
            if not rank:break
            selected=pivots[:rank];chosen=active[selected]
            delta=np.linalg.lstsq(matrix[:,selected],-values,rcond=1e-10)[0]
            fraction=min(1,.25/max(1e-12,np.max(np.abs(delta))))
            trial=x.copy();trial[chosen]+=fraction*delta;trial=np.clip(trial,lo,hi)
            if np.max(np.abs(trial-x))<1e-10:break
            values=residual(trial);x=trial;newton_iterations+=1
        return x
    try:
        if x0:
            newton_restore(np.asarray(x0))
            seed=best_x if best_x is not None else np.asarray(x0)
            least_squares(residual,seed,bounds=(lo,hi),diff_step=2e-5,x_scale='jac',max_nfev=100,ftol=1e-10,xtol=1e-10,gtol=1e-10)
    except (Ready,SolverDeadline):pass
    initial_cost=best_cost if np.isfinite(best_cost) else None
    cost_status='not_started'
    if best_result is not None and best_result['status']=='Target Achieved' and seconds-(time.monotonic()-start)>10:
        polishing=True;cache=None
        def objective(x):evaluate(x);return cache['cost']
        def inequalities(x):
            evaluate(x);result=cache['result'];candidate=cache['candidate'];trial=cache['trial']
            values=[]
            for row in result['target_comparison']:
                error=10000 if row['error'] is None else row['error']/row['tolerance']
                values.extend([.8-error,.8+error])
            if candidate.stage_masses is not None:
                for mass,bound,stage in zip(candidate.stage_masses,limits,s.vehicle.stages):
                    values.append((mass.dry_mass_kg-bound.hardware_mass_kg-bound.tank_structure_ratio*mass.propellant_mass_kg)/stage.dry_mass_kg if stage.engine_type=='liquid' else 1.)
                thrust,weight=pad_reference(trial)
                values.append(thrust*candidate.burns[0].throttles[0]/(trial.scenario.vehicle.initial_mass_kg*weight)-s.vehicle.minimum_initial_twr)
            summary=result['summary']
            values.extend([(result['orbit']['perigee_altitude_m']-100000)/R_E,1-result['orbit']['eccentricity'],(s.constraints.max_mission_duration_s-summary['arrival_time_s'])/1000])
            if s.constraints.max_dynamic_pressure_pa:values.append(1-summary['max_dynamic_pressure_pa']/s.constraints.max_dynamic_pressure_pa)
            if s.constraints.max_load_factor_g:values.append(1-summary['max_load_factor_g']/s.constraints.max_load_factor_g)
            values.append(trial.departure_margin(result['series'])/180)
            if s.target.arrival_time_min_s is not None:values.append((summary['arrival_time_s']-s.target.arrival_time_min_s)/1000)
            if s.target.arrival_time_max_s is not None:values.append((s.target.arrival_time_max_s-summary['arrival_time_s'])/1000)
            return np.asarray(values)
        try:
            if original is not None:
                # Small coupled design steps stay close to the verified
                # trajectory manifold. Large mass steps in a shooting NLP
                # otherwise create enormous orbit errors before the controls
                # have a chance to compensate. This local SQP uses the smooth
                # vector orbit residuals, including the equatorial plane.
                trust=.002
                for _ in range(4):
                    check();center=best_x.copy();base=residual(center)
                    base_cost=cache['cost'];margin=inequalities(center)
                    jac=[];gradient=[];constraint_jac=[]
                    for i in range(len(center)):
                        step=2e-5
                        if hi[i]-center[i]<step:step=-min(step,center[i]-lo[i])
                        trial=center.copy();trial[i]+=step
                        jac.append((residual(trial)-base)/step)
                        gradient.append((cache['cost']-base_cost)/step)
                        constraint_jac.append((inequalities(trial)-margin)/step)
                    jac=np.asarray(jac).T;gradient=np.asarray(gradient)
                    constraint_jac=np.asarray(constraint_jac).T
                    # Select independent smooth residual rows, rather than
                    # redundant zero-valued flight penalties.
                    norms=np.linalg.norm(jac,axis=1);usable=norms>1e-8
                    matrix=jac[usable]/norms[usable,None]
                    rhs=base[usable]/norms[usable]
                    _,triangular,pivots=qr(matrix.T,mode='economic',pivoting=True)
                    diagonal=np.abs(np.diag(triangular))
                    rank=int(np.sum(diagonal>max(1e-6,diagonal.max(initial=0)*1e-6)))
                    selected=pivots[:rank];matrix=matrix[selected];rhs=rhs[selected]
                    lower=np.maximum(np.asarray(lo)-center,-trust)
                    upper=np.minimum(np.asarray(hi)-center,trust)
                    constraints=[{'type':'ineq','fun':lambda d:margin+constraint_jac@d}]
                    if rank:constraints.append({'type':'eq','fun':lambda d:rhs+matrix@d})
                    step_fit=minimize(lambda d:float(gradient@d+.1*np.dot(d,d)),np.zeros(len(center)),
                        jac=lambda d:gradient+.2*d,method='SLSQP',bounds=list(zip(lower,upper)),
                        constraints=constraints,options={'maxiter':80,'ftol':1e-12})
                    if not step_fit.success:break
                    residual(center+step_fit.x)
                    if best_cost>=base_cost-1e-9:
                        trust*=.5
                    if trust<.00025:break
            fit=minimize(objective,best_x,method='SLSQP',bounds=list(zip(lo,hi)),constraints=[{'type':'ineq','fun':inequalities}],options={'eps':2e-5,'maxiter':60,'ftol':1e-7})
            cost_status=str(fit.message)
        except SolverDeadline:cost_status='time_allocation'
        except NumericalFailure:
            # A failed trial must not discard the independently verified
            # feasible design already retained by evaluate(). Cancellation
            # deliberately propagates to the mission driver.
            cost_status='numerical_trial_rejected'
    return best,{'evaluations':calls,'newton_iterations':newton_iterations,'cost_evaluations':cost_evaluations,'cost_stop_reason':cost_status,
        'segment_cache_hits':segment_cache.hits,'segment_cache_misses':segment_cache.misses,
        'initial_feasible_cost':initial_cost,'best_feasible_cost':best_cost if np.isfinite(best_cost) else None,
        'elapsed_s':time.monotonic()-start,'status':best_result['status'] if best_result else 'No valid correction','score':best_score if np.isfinite(best_score) else None}
