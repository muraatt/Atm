from __future__ import annotations

import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy.optimize import least_squares, minimize

from .dynamics import Cancelled, InvalidTrialState, NumericalFailure, Simulation, SolverDeadline, compare_target
from .earth import MU, R_E, V_SCALE, targeted_elements
from .models import Burn, FlightPlan, Scenario
from .consistency import analyze_consistency
from .screening import azimuth_bounds,azimuth_margin,departure_options,seed_shape


@dataclass
class PhaseSpec:
    stage: int
    restart: bool
    transfer: bool


def launch_azimuth(scenario):
    return departure_options(scenario.launch.latitude_deg,scenario.target,scenario.launch.azimuth_sectors_deg)[1]


def terminal_errors(orbit,target):
    errors=[]
    if target.perigee_altitude_m is not None and target.apogee_altitude_m is not None:
        rp,ra=R_E+target.perigee_altitude_m,R_E+target.apogee_altitude_m
        p=(orbit['semi_major_axis_m'] or R_E)*(1-orbit['eccentricity']**2)
        errors=[(p-2*rp*ra/(rp+ra))/50000,(orbit['eccentricity']-(ra-rp)/(ra+rp))*R_E/50000]
    for row in compare_target(orbit,target):
        if row['parameter'] in ('perigee_altitude_m','apogee_altitude_m') and target.perigee_altitude_m is not None and target.apogee_altitude_m is not None:continue
        errors.append(1000 if row['error'] is None else row['error']/(50000 if row['parameter'].endswith('_m') else 5))
    return errors


def templates(scenario):
    direct=[PhaseSpec(i,False,False) for i in range(len(scenario.vehicle.stages))]
    options=[direct]
    for stage_idx in reversed(range(len(scenario.vehicle.stages))):
        last=scenario.vehicle.stages[stage_idx]
        if last.engine_type!='liquid' or last.ignition_limit<2:
            continue
        # Enumerate legal burn/coast structures; discrete ignition count is never
        # hidden inside a continuous optimizer.
        for count in range(2,last.ignition_limit+1):
            options.append(direct[:stage_idx+1]+[PhaseSpec(stage_idx,True,True) for _ in range(count-1)]+direct[stage_idx+1:])
    if (scenario.target.apogee_altitude_m or scenario.target.perigee_altitude_m or 200000)>2e6 and len(options)>1:
        options=options[1:]+options[:1]
    return options


class MultipleShooting:
    """Normalized controls plus independent phase-end states and continuity defects."""
    def __init__(self,sim:Simulation,specs:list[PhaseSpec],seed_angle:float):
        self.sim,self.specs=sim,specs
        self.scenario=sim.scenario
        self.parameter_slices=[]
        self.bounds=[]
        self.seed=[]
        self.cache=OrderedDict()
        self.invalid_candidates=0
        self.numerical_failures=0
        self.failure_reasons={}
        self._propagation_failed=False
        self.scale=np.r_[[R_E]*3,[V_SCALE]*3,1]
        count_last=sum(s.stage==len(self.scenario.vehicle.stages)-1 for s in specs)
        for i,spec in enumerate(specs):
            stage=self.scenario.vehicle.stages[spec.stage]
            begin=len(self.seed)
            # [duration/full_burn, coast/time_scale, angle x3, clock x3, throttle x3]
            if stage.engine_type=="solid":
                duration_bounds=(1,1)
                duration=1
            else:
                duration_bounds=(0.005,1/stage.throttle_min)
                duration=0.97 if spec.stage<len(self.scenario.vehicle.stages)-1 else (0.9 if count_last==1 else 0.7)
                if spec.restart:
                    duration=0.20
            coast_bounds=(0,0) if i==0 else (0,self.scenario.constraints.max_mission_duration_s/1000 if spec.transfer else min(180,self.scenario.constraints.max_mission_duration_s)/1000)
            coast=0.0
            if spec.transfer:
                rp=R_E+max(200000,seed_shape(self.scenario.target)[0])
                ra=R_E+seed_shape(self.scenario.target)[1]
                coast=min(self.scenario.constraints.max_mission_duration_s*0.8,np.pi*np.sqrt(((rp+ra)/2)**3/MU))/1000
                # A low-orbit restart is not a half-orbit transfer before
                # insertion. Leave its coast free, but seed it at zero.
                if seed_shape(self.scenario.target)[1] < 2e6:
                    coast=0.0
            angles=[seed_angle,seed_angle*0.4,seed_angle*0.1] if spec.stage==0 else [0.05,0.02,0.0]
            if spec.restart:
                angles=[0.05,0.02,0.01]
            self.seed.extend([duration,coast,*angles,0,0,0,1,1,1])
            self.bounds.extend([duration_bounds,coast_bounds,*[(0,1)]*3,*[(-np.pi,np.pi)]*3,
                                *(([(1,1)]*3) if stage.engine_type=="solid" else [(stage.throttle_min,stage.throttle_max)]*3)])
            self.parameter_slices.append(slice(begin,len(self.seed)))
        self.azimuth_index=len(self.seed)
        az=launch_azimuth(self.scenario)
        az_bounds,az=azimuth_bounds(self.scenario.launch,az)
        self.seed.append(az/180)
        self.bounds.append(tuple(x/180 for x in az_bounds))
        self.control_size=len(self.seed)
        self.state_slices=[]
        # Initialize shooting states with an actual continuous trajectory.
        y,t=sim.initial.copy(),0.0
        for idx,spec in enumerate(specs):
            burn=self.decode_burn(np.asarray(self.seed),idx)
            if idx and spec.stage != specs[idx-1].stage:
                y[6]=self.scenario.vehicle.stages[spec.stage].propellant_mass_kg
            delay=self.scenario.vehicle.stages[specs[idx-1].stage].separation_delay_s if idx and spec.stage!=specs[idx-1].stage else 0
            segment=self.propagate(y,t,burn,az,delay)
            y,t=segment[0].copy(),segment[1]
            norm=y/self.state_scale(spec.stage)
            begin=len(self.seed)
            self.seed.extend(norm)
            # Bounds allow high elliptical transfers, but reject unphysical fuel.
            rmax=max(2,(R_E+seed_shape(self.scenario.target)[1])*1.5/R_E)
            self.bounds.extend([(-rmax,rmax)]*3+[(-2,2)]*3+[(0,1)])
            self.state_slices.append(slice(begin,len(self.seed)))
        self.seed=np.array(self.seed,dtype=float)
        self.seed=np.clip(self.seed,[a for a,b in self.bounds],[b for a,b in self.bounds])
        self.last_x=None
        self.last_eval=None
        self.best_x=self.seed.copy()
        self.best_merit=np.inf
        self.iterations=0

    def state_scale(self,stage):
        return np.r_[[R_E]*3,[V_SCALE]*3,self.scenario.vehicle.stages[stage].propellant_mass_kg]

    def decode_burn(self,x,idx):
        values=x[self.parameter_slices[idx]]
        stage=self.scenario.vehicle.stages[self.specs[idx].stage]
        return Burn(stage_index=self.specs[idx].stage,duration_s=float(max(0.001,values[0]*stage.full_burn_s)),
                    coast_before_s=float(max(0,values[1]*1000)),angle_fractions=tuple(np.clip(values[2:5],0,1)),
                    clock_angles_rad=tuple(values[5:8]),throttles=tuple(np.clip(values[8:11],stage.throttle_min,stage.throttle_max)))

    def plan(self,x):
        return FlightPlan(launch_azimuth_deg=float(x[self.azimuth_index]*180),burns=[self.decode_burn(x,i) for i in range(len(self.specs))])

    def propagate(self,y,t,burn,azimuth,delay=0):
        key=(burn.stage_index,*y,t,burn.duration_s,burn.coast_before_s,delay,*burn.angle_fractions,*burn.clock_angles_rad,*burn.throttles,azimuth)
        if key in self.cache:
            self.cache.move_to_end(key)
            out,failed=self.cache[key]
            self._propagation_failed |= failed
            return out
        failed=False
        try:
            self.sim.validate_state(t,y)
            if self.sim.surface_clearance(t,y[:3])[0] < -.1:
                raise InvalidTrialState("Phase starts below the collision surface.")
            before=self.sim.integrate(y,t,burn.coast_before_s+delay,burn.stage_index,azimuth=azimuth)
            if before.status=="Impact":
                out=(before.state,before.time,before.max_q,before.max_load,1.0,before.min_departure_margin_deg)
            else:
                after=self.sim.integrate(before.state,before.time,burn.duration_s,burn.stage_index,burn,azimuth)
                shortfall=max(0,(burn.duration_s-(after.time-before.time))/max(1,burn.duration_s))
                out=(after.state,after.time,max(before.max_q,after.max_q),max(before.max_load,after.max_load),
                     max(shortfall,1.0 if after.status=="Impact" else 0.0),min(before.min_departure_margin_deg,after.min_departure_margin_deg))
        except NumericalFailure as exc:
            failed=True
            self._propagation_failed=True
            if not isinstance(exc,InvalidTrialState): self.numerical_failures+=1
            reason=str(exc)
            self.failure_reasons[reason]=self.failure_reasons.get(reason,0)+1
            # Finite failed-phase residuals let SLSQP backtrack. Surface and
            # continuity constraints still expose the invalid trial; no state
            # is projected onto a fictitious successful flight.
            out=(y.copy(),t,0.0,0.0,1.0,-180.0)
        self.cache[key]=(out,failed)
        if len(self.cache)>3000:
            self.cache.popitem(last=False)
        return out

    def evaluate(self,x):
        self.sim.check()
        if self.last_x is not None and np.array_equal(x,self.last_x):
            return self.last_eval
        if not np.all(np.isfinite(x)):
            return self.invalid_evaluation(x,"Nonfinite optimizer variables.")
        try:
            with np.errstate(over='raise',divide='raise',invalid='raise'):
                return self._evaluate(x)
        except (InvalidTrialState,FloatingPointError) as exc:
            return self.invalid_evaluation(x,str(exc))

    def invalid_evaluation(self,x,reason):
        self.invalid_candidates+=1
        self.failure_reasons[reason]=self.failure_reasons.get(reason,0)+1
        orbit=targeted_elements(self.sim.initial[:3],self.sim.initial[3:6],self.scenario.target)
        value={"eq":np.full(7*len(self.specs),10.0),
               "ineq":np.full(4*len(self.specs)+4+int(self.scenario.constraints.max_dynamic_pressure_pa is not None)+int(self.scenario.constraints.max_load_factor_g is not None),-10.0),
               "terminal":np.full(2*len(compare_target(orbit,self.scenario.target)),-1000.0),
               "errors":np.full(len(terminal_errors(orbit,self.scenario.target)),1000.0),
               "objective":1e6,"orbit":orbit,"time":0.0,"valid":False}
        self.last_x=x.copy();self.last_eval=value
        return value

    def _evaluate(self,x):
        self._propagation_failed=False
        equalities,inequalities=[],[]
        y,t=self.sim.initial.copy(),0.0
        az=x[self.azimuth_index]*180
        maxq,maxg=0.0,0.0
        burned=0.0
        for i,spec in enumerate(self.specs):
            stage=self.scenario.vehicle.stages[spec.stage]
            if i and spec.stage!=self.specs[i-1].stage:
                y[6]=stage.propellant_mass_kg
            burn=self.decode_burn(x,i)
            consumption=stage.mass_flow_kg_s*burn.duration_s*(stage.curve_mean if stage.engine_type=="solid" else np.dot([.25,.5,.25],burn.throttles))
            inequalities.append((y[6]-consumption)/stage.propellant_mass_kg)
            delay=self.scenario.vehicle.stages[self.specs[i-1].stage].separation_delay_s if i and spec.stage!=self.specs[i-1].stage else 0
            end,endtime,q,g,shortfall,corridor_margin=self.propagate(y,t,burn,az,delay)
            boundary=x[self.state_slices[i]]*self.state_scale(spec.stage)
            equalities.extend((end-boundary)/self.state_scale(spec.stage))
            inequalities.extend([1e-6-shortfall,corridor_margin/180])
            clearance,_=self.sim.surface_clearance(endtime,boundary[:3])
            inequalities.append((clearance+.1)/R_E)
            if clearance < -.1: self._propagation_failed=True
            maxq,maxg=max(q,maxq),max(g,maxg)
            burned+=consumption
            y=boundary.copy()
            t=endtime
        inequalities.append((self.scenario.constraints.max_mission_duration_s-t)/self.scenario.constraints.max_mission_duration_s)
        if self.scenario.constraints.max_dynamic_pressure_pa:
            inequalities.append(1-maxq/self.scenario.constraints.max_dynamic_pressure_pa)
        if self.scenario.constraints.max_load_factor_g:
            inequalities.append(1-maxg/self.scenario.constraints.max_load_factor_g)
        self.sim.validate_state(t,y)
        orbit=targeted_elements(y[:3],y[3:6],self.scenario.target)
        errors=terminal_errors(orbit,self.scenario.target)
        inequalities.extend([(orbit['perigee_altitude_m']-100000)/R_E,1-orbit['eccentricity'],azimuth_margin(az,self.scenario.launch.azimuth_sectors_deg)/180])
        # Propellant objective includes discarded reserves: initial fuel minus
        # final carried reserve. Early-stage unburned fuel is expendable loss.
        expended=sum(s.propellant_mass_kg for s in self.scenario.vehicle.stages)-y[6]
        terminal=[]
        for row in compare_target(orbit,self.scenario.target):
            e=10000 if row["error"] is None else row["error"]/row["tolerance"]
            terminal.extend([1-e,1+e])
        value={"eq":np.asarray(equalities),"ineq":np.asarray(inequalities),"terminal":np.asarray(terminal),
               "errors":np.asarray(errors),"objective":expended/self.scenario.vehicle.initial_mass_kg if self.scenario.objective=="propellant" else t/1000,
               "orbit":orbit,"time":t,"valid":not self._propagation_failed}
        merit=float(np.dot(value["errors"],value["errors"])+1e5*np.dot(value["eq"],value["eq"])+1e4*np.sum(np.minimum(0,value["ineq"])**2))
        if not value['valid']: self.invalid_candidates+=1
        if value['valid'] and merit < self.best_merit:
            self.best_merit=merit
            self.best_x=x.copy()
        self.last_x=x.copy()
        self.last_eval=value
        return value


def result_score(result,objective='propellant'):
    if result["status"]=="Target Achieved":
        cost=(result['summary']['propellant_burned_kg']+result['summary']['propellant_discarded_kg'])/1e6 if objective=='propellant' else result['summary']['arrival_time_s']/1e6
        return -1e6+cost
    score=sum((1000 if row["error"] is None else abs(row["error"])/(50000 if row["parameter"].endswith("_m") else 5))**2 for row in result["target_comparison"])
    if result["status"] in ("Impact","No Liftoff"):
        score+=1e5
    score+=100*sum(not b for b in result["constraint_checks"].values())
    return score


def warm_start(sim:Simulation,progress,guidance_frame='launch_heading',stop_on_target=False):
    """Continuous forward shooting gives the constrained solve a physical seed."""
    scenario=sim.scenario
    stages=scenario.vehicle.stages
    count=len(stages)
    # One angle-amplitude per stage, last liquid burn fraction, launch azimuth.
    upper_liquid=stages[-1].engine_type=='liquid'
    x0=[.25 if i==0 else .20 for i in range(count)]
    if count>1 and (scenario.target.perigee_altitude_m is None or scenario.target.apogee_altitude_m is None):x0[-1]=.8
    bounds_low=[0.0]*count
    bounds_high=[.9]*count
    if upper_liquid:
        x0.append(.85);bounds_low.append(.01);bounds_high.append(1.0)
    az_bounds,az=azimuth_bounds(scenario.launch,launch_azimuth(scenario))
    x0.append(az/180);bounds_low.append(az_bounds[0]/180);bounds_high.append(az_bounds[1]/180)
    best_plan=None
    best_result=None
    best_score=float('inf')
    calls=0
    class SeedReady(Exception):pass
    sim.warm_evaluations=0
    sim.warm_stopped_on_target=False
    def decode(x):
        burns=[]
        for i,stage in enumerate(stages):
            fraction=x[count] if i==count-1 and upper_liquid else (.995 if stage.engine_type=='liquid' else 1)
            duration=stage.full_burn_s if stage.engine_type=='solid' else fraction*stage.full_burn_s/stage.throttle_max
            burns.append(Burn(stage_index=i,duration_s=float(duration),
                              angle_fractions=(float(x[i]),float(x[i]*.4),float(x[i]*.1) if i==0 else 0),
                              throttles=(1,1,1) if stage.engine_type=='solid' else (stage.throttle_max,)*3))
        return FlightPlan(launch_azimuth_deg=float(x[-1]*180),burns=burns,guidance_frame=guidance_frame)
    def physical_residual(x):
        nonlocal best_plan,best_result,best_score,calls
        sim.check()
        plan=decode(x)
        configured_limits=scenario.constraints.max_dynamic_pressure_pa is not None or scenario.constraints.max_load_factor_g is not None or bool(scenario.launch.azimuth_sectors_deg)
        result=sim.run(plan,strict=False,record=configured_limits)
        row=result['orbit']
        last=sim._last_state
        r,v=last[:3],last[3:6]
        h=np.cross(r,v);p=np.dot(h,h)/MU
        evec=np.cross(v,h)/MU-r/np.linalg.norm(r)
        target=scenario.target
        if target.perigee_altitude_m is not None and target.apogee_altitude_m is not None and abs(target.perigee_altitude_m-target.apogee_altitude_m)<1e-3:
            rp=R_E+target.perigee_altitude_m
            radial=r/np.linalg.norm(r);transverse=np.cross(h/np.linalg.norm(h),radial)
            values=[(p-rp)/50000,np.dot(evec,radial)*R_E/50000,np.dot(evec,transverse)*R_E/50000]
            for comparison in result['target_comparison']:
                if comparison['parameter'] not in ('perigee_altitude_m','apogee_altitude_m'):values.append(100 if comparison['error'] is None else comparison['error']/5)
        else:values=terminal_errors(row,target)
        values.extend([max(0,100000-row['perigee_altitude_m'])/50000,max(0,row['eccentricity']-.999)*100,max(0,-sim.departure_margin(result['series']))/10])
        if scenario.constraints.max_dynamic_pressure_pa:
            values.append(10*max(0,result['summary']['max_dynamic_pressure_pa']/scenario.constraints.max_dynamic_pressure_pa-1))
        if scenario.constraints.max_load_factor_g:
            values.append(10*max(0,result['summary']['max_load_factor_g']/scenario.constraints.max_load_factor_g-1))
        if result['status'] in ('Impact','No Liftoff'):
            values[0]+=100
        score=float(np.dot(values,values))
        if score<best_score:
            best_plan=plan;best_result=result;best_score=score
        calls+=1
        sim.warm_evaluations=calls
        if calls%5==0:
            progress({'phase':'Building a physically continuous initial trajectory','iteration':calls,
                      'target_error_score':float(np.sqrt(best_score))})
        if stop_on_target and result['status']=='Target Achieved':
            # This is only seed generation. Preserve a physically continuous
            # target-satisfying seed and spend the remaining budget on the NLP.
            best_plan=plan;best_result=result
            sim.warm_stopped_on_target=True
            raise SeedReady()
        return np.asarray(values)
    initial_orbit=targeted_elements(sim.initial[:3],sim.initial[3:6],scenario.target)
    circular=scenario.target.perigee_altitude_m is not None and scenario.target.apogee_altitude_m is not None and abs(scenario.target.perigee_altitude_m-scenario.target.apogee_altitude_m)<1e-3
    residual_size=len(terminal_errors(initial_orbit,scenario.target))+int(circular)+3+int(scenario.constraints.max_dynamic_pressure_pa is not None)+int(scenario.constraints.max_load_factor_g is not None)
    sim.warm_invalid_candidates=0
    def residual(x):
        try:
            with np.errstate(over='raise',divide='raise',invalid='raise'):
                return physical_residual(x)
        except (NumericalFailure,FloatingPointError):
            sim.warm_invalid_candidates+=1
            # Same residual dimension for every candidate, including failures.
            return np.full(residual_size,1000.0)
    try:
        fit=least_squares(residual,x0,bounds=(bounds_low,bounds_high),diff_step=1e-3,
                          ftol=1e-8,xtol=1e-7,gtol=1e-7,max_nfev=35)
        plan=decode(fit.x)
        result=sim.run(plan,strict=False,record=True)
        return plan,result
    except SeedReady:
        return best_plan,best_result
    except (SolverDeadline,NumericalFailure):
        if best_plan is not None:
            return best_plan,None
        raise


def optimize(scenario:Scenario,check_cancel:Callable[[],bool] | None=None,
             progress:Callable[[dict],None] | None=None):
    if scenario.constraints.solver_method == 'mission':
        from .mission_solver import optimize_mission
        return optimize_mission(scenario,check_cancel,progress)
    start=time.monotonic()
    deadline=start+scenario.constraints.solver_budget_s
    check_cancel=check_cancel or (lambda:False)
    progress=progress or (lambda value:None)
    enforcing_deadline=True
    local_end=None
    local_scope='search_time_limit'
    last_deadline_scope=None
    def check():
        nonlocal last_deadline_scope
        if check_cancel(): raise Cancelled()
        if enforcing_deadline and time.monotonic()>deadline:
            last_deadline_scope='total_budget'
            raise SolverDeadline('total_budget')
        if enforcing_deadline and local_end is not None and time.monotonic()>local_end:
            last_deadline_scope=local_scope
            raise SolverDeadline(local_scope)
    progress({'phase':'Preparing environment and Earth frames','iteration':0})
    sim=Simulation(scenario,check=check)
    candidates=[]
    solver_messages=[]
    search_reports=[]
    iterations=0
    options=templates(scenario)
    warm_plan=None
    stage_burns=[Burn(stage_index=i,duration_s=s.full_burn_s/(1 if s.engine_type=='solid' else s.throttle_max),
                     angle_fractions=(.3,.12,.03),throttles=(s.throttle_max,)*3 if s.engine_type=='liquid' else (1,1,1))
                 for i,s in enumerate(scenario.vehicle.stages)]
    baseline=FlightPlan(launch_azimuth_deg=launch_azimuth(scenario),burns=stage_burns)
    candidates.append((baseline,1e12,None))
    no_liftoff=False
    try:
        baseline_result=sim.run(baseline,strict=False,record=True)
        candidates[0]=(baseline,result_score(baseline_result,scenario.objective),None)
        no_liftoff=baseline_result['status']=='No Liftoff'
    except SolverDeadline: pass
    except NumericalFailure as exc:
        solver_messages.append(f'Baseline could not be integrated: {exc}')
    if not no_liftoff and (scenario.target.apogee_altitude_m or scenario.target.perigee_altitude_m or 200000)<2e6 and time.monotonic()<deadline:
        local_end=min(deadline,start+scenario.constraints.solver_budget_s*.4)
        local_scope='initial_guess_time_limit'
        try:
            warm_plan,warm_result=warm_start(sim,progress)
            if warm_result is not None:
                candidates.append((warm_plan,result_score(warm_result,scenario.objective),None))
            else:
                candidates.append((warm_plan,1e11,None))
        except SolverDeadline: pass
        except NumericalFailure as exc:
            solver_messages.append(f'Initial guess failed: {exc}')
        finally: local_end=None
        if last_deadline_scope=='initial_guess_time_limit':
            solver_messages.append('Initial-guess time allowance reached; remaining time is reserved for constrained searches.')
    schedule=[(ti,specs,angle) for ti,specs in enumerate(options)
              for angle in ((.25,) if warm_plan and len(warm_plan.burns)==len(specs) else (.18,.25))]
    if no_liftoff: schedule=[]
    # Each search receives a share of remaining time, including setup and RHS
    # work. Global and local limits have separate, machine-readable reasons.
    for si,(ti,specs,seed_angle) in enumerate(schedule):
        if time.monotonic()>=deadline:
            last_deadline_scope='total_budget'
            break
        problem=None
        search_start=time.monotonic()
        start_iterations=iterations
        local_end=search_start+(deadline-search_start)/(len(schedule)-si)
        local_scope='search_time_limit'
        report={'template':ti+1,'burn_count':len(specs),'seed_angle':seed_angle,
                'stop_reason':'solver_stopped','solver_messages':[]}
        search_phase='feasibility'
        try:
            progress({'phase':f'Optimizing {len(specs)}-burn trajectory','iteration':iterations,'template':ti+1})
            problem=MultipleShooting(sim,specs,seed_angle)
            if warm_plan and len(warm_plan.burns)==len(specs):
                problem.seed[problem.azimuth_index]=warm_plan.launch_azimuth_deg/180
                state,clock=sim.initial.copy(),0.0
                for i,burn in enumerate(warm_plan.burns):
                    stage=scenario.vehicle.stages[burn.stage_index]
                    problem.seed[problem.parameter_slices[i]]=[burn.duration_s/stage.full_burn_s,burn.coast_before_s/1000,
                        *burn.angle_fractions,*burn.clock_angles_rad,*burn.throttles]
                    delay=0
                    if i and burn.stage_index!=warm_plan.burns[i-1].stage_index:
                        state[6]=stage.propellant_mass_kg
                        delay=scenario.vehicle.stages[warm_plan.burns[i-1].stage_index].separation_delay_s
                    propagated=problem.propagate(state,clock,burn,warm_plan.launch_azimuth_deg,delay)
                    state,clock=propagated[0].copy(),propagated[1]
                    problem.seed[problem.state_slices[i]]=state/problem.state_scale(burn.stage_index)
            problem.seed=np.clip(problem.seed,[a for a,b in problem.bounds],[b for a,b in problem.bounds])
            candidates.append((problem.plan(problem.seed),1e10,ti+1))
            def callback(x):
                nonlocal iterations
                iterations+=1
                value=problem.evaluate(x)
                progress({'phase':'Finding a feasible trajectory' if search_phase=='feasibility' else 'Improving selected objective',
                          'iteration':iterations,'target_error_score':float(np.linalg.norm(value['errors'])),
                          'template':ti+1,'rejected_candidates':problem.invalid_candidates})
                check()
            def feasible(value):
                return value['valid'] and np.max(np.abs(value['eq']))<1e-5 and np.min(value['terminal'])>=0 and np.min(value['ineq'])>=-1e-6
            seed_value=problem.evaluate(problem.seed)
            if not feasible(seed_value):
                fit=minimize(lambda x:float(np.dot(problem.evaluate(x)['errors'],problem.evaluate(x)['errors'])),problem.seed,
                    method='SLSQP',bounds=problem.bounds,
                    constraints=[{'type':'eq','fun':lambda x:problem.evaluate(x)['eq']},
                                 {'type':'ineq','fun':lambda x:problem.evaluate(x)['ineq']}],
                    callback=callback,options={'maxiter':scenario.constraints.max_iterations,'ftol':1e-8,'eps':2e-5})
                report['solver_messages'].append(str(fit.message))
                report['stop_reason']='converged' if fit.success else ('iteration_limit' if fit.status==9 else 'solver_stopped')
            x=problem.best_x
            value=problem.evaluate(x)
            if feasible(value):
                search_phase='objective'
                progress({'phase':'Improving selected objective','iteration':iterations})
                polished=minimize(lambda z:problem.evaluate(z)['objective'],x,method='SLSQP',bounds=problem.bounds,
                    constraints=[{'type':'eq','fun':lambda z:problem.evaluate(z)['eq']},
                                 {'type':'ineq','fun':lambda z:np.r_[problem.evaluate(z)['ineq'],problem.evaluate(z)['terminal']]}],
                    callback=callback,options={'maxiter':min(30,scenario.constraints.max_iterations),'ftol':1e-7,'eps':2e-5})
                x=polished.x if polished.success else x
                report['solver_messages'].append(str(polished.message))
                report['stop_reason']='converged' if polished.success else ('iteration_limit' if polished.status==9 else 'solver_stopped')
            plan=problem.plan(x)
            local_end=None
            trial=sim.run(plan,strict=False,record=True)
            candidates.append((plan,result_score(trial,scenario.objective),ti+1))
        except SolverDeadline as exc:
            report['stop_reason']=exc.scope
        except NumericalFailure as exc:
            report['stop_reason']='numerical_failure'
            report['solver_messages'].append(str(exc))
        finally:
            local_end=None
            if problem is not None:
                if np.isfinite(problem.best_merit):
                    candidates.append((problem.plan(problem.best_x),problem.best_merit,ti+1))
                report['invalid_candidates']=problem.invalid_candidates
                report['numerical_failures']=problem.numerical_failures
                report['rejection_reasons']=problem.failure_reasons
            report['iterations']=iterations-start_iterations
            report['elapsed_s']=time.monotonic()-search_start
            report['phase']=search_phase
            search_reports.append(report)
            labels={'total_budget':'Total optimization budget reached','search_time_limit':'Time allowance for this search reached',
                    'iteration_limit':'Iteration limit reached','numerical_failure':'Search stopped after a numerical failure',
                    'converged':'Solver converged','solver_stopped':'Solver stopped before convergence'}
            solver_messages.append(f"Search {si+1} ({len(specs)} burns): {labels[report['stop_reason']]}. "+' '.join(report['solver_messages']))
            if report.get('invalid_candidates',0):
                solver_messages.append(f"Search {si+1}: rejected {report['invalid_candidates']} invalid candidates; retained valid candidates.")
    enforcing_deadline=False
    search_elapsed=time.monotonic()-start
    total_budget_exhausted=last_deadline_scope=='total_budget' or search_elapsed>=scenario.constraints.solver_budget_s
    progress({'phase':'Verifying trajectory with strict integration','iteration':iterations})
    verified=[]
    verification_failures=[]
    shortlist=sorted(candidates,key=lambda c:c[1])[:3]
    # Continuously simulated fallbacks cannot be displaced by a shooting merit.
    if warm_plan: shortlist.append((warm_plan,0,None))
    shortlist.append((baseline,0,None))
    seen=set()
    for plan,_,template in shortlist:
        key=plan.model_dump_json()
        if key in seen: continue
        seen.add(key)
        try:
            result=sim.run(plan,strict=True,record=True)
            verified.append((result,template))
        except NumericalFailure as exc:
            verification_failures.append(str(exc))
    if not verified:
        raise NumericalFailure('No candidate passed numerical replay: '+'; '.join(verification_failures))
    result,selected_template=min(verified,key=lambda item:result_score(item[0],scenario.objective))
    reason='no_liftoff' if no_liftoff else ('total_budget' if total_budget_exhausted else 'searches_completed')
    result['optimizer']={'method':'Scaled multiple shooting + constrained SLSQP',
                        'iterations':iterations,'elapsed_s':time.monotonic()-start,
                        'search_elapsed_s':search_elapsed,'total_budget_exhausted':total_budget_exhausted,
                        'termination_reason':reason,'searches':search_reports,
                        'invalid_candidates':sum(r.get('invalid_candidates',0) for r in search_reports)+getattr(sim,'warm_invalid_candidates',0),
                        'numerical_failures':sum(r.get('numerical_failures',0) for r in search_reports),
                        'verification_failures':verification_failures,
                        'messages':solver_messages,'selected_template':selected_template,'objective':scenario.objective,
                        'note':'A nonconverged local search is not proof that the target is physically impossible.'}
    result['consistency']=analyze_consistency(scenario,result)
    return result
