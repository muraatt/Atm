"""Bounded mission-structure search + sparse direct collocation + exact replay."""
import time
import numpy as np

from .collocation import CollocationProblem
from .consistency import analyze_consistency
from .dynamics import Cancelled, NumericalFailure, Simulation, SolverDeadline
from .mission import mission_topologies, seed_plan, seed_from_ascent, seed_from_transfer, seed_from_parking, needs_node_transfer, resample_burn, phase_manifest
from .models import FlightPlan, TargetOrbit, Scenario


def target_norm(result):
    return float(np.linalg.norm([10000 if r['error'] is None else r['error']/r['tolerance'] for r in result['target_comparison']]))


def mission_score(result,objective):
    if result['status']=='Target Achieved':
        cost=result['summary']['arrival_time_s'] if objective=='time' else result['summary']['propellant_burned_kg']+result['summary']['propellant_discarded_kg']
        return -1e12+cost
    return target_norm(result)**2+1e8*int(result['status'] in ('Impact','No Liftoff'))+1e6*sum(not passed for passed in result['constraint_checks'].values())


def optimize_mission(scenario,check_cancel=None,progress=None):
    from .optimization import warm_start
    result_score=mission_score
    progress=progress or (lambda value:None);check_cancel=check_cancel or (lambda:False)
    input_snapshot=scenario.model_dump(mode='json')
    scenario=scenario.model_copy(deep=True)
    sizing=scenario.vehicle.optimize_stage_masses
    if sizing:
        from .sizing import mass_bounds,seed_masses
        for stage,bounds in zip(scenario.vehicle.stages,mass_bounds(scenario)):stage.mass_bounds=bounds
    start=time.monotonic();deadline=start+scenario.constraints.solver_budget_s
    local_end=None;enforcing=True
    def check():
        if check_cancel():raise Cancelled()
        if enforcing and time.monotonic()>=deadline:raise SolverDeadline('total_budget')
        if enforcing and local_end is not None and time.monotonic()>=local_end:raise SolverDeadline('search_time_limit')
    progress({'phase':'Preparing mission dynamics and environment','iteration':0})
    sim=Simulation(scenario,check=check)
    topologies,total_structures=mission_topologies(scenario)
    candidates=[];feasible_plans=[];reports=[];iterations=0;messages=[];best_continuous=None
    baseline=seed_plan(scenario,topologies[0])
    if sizing:baseline=baseline.model_copy(update={'stage_masses':seed_masses(scenario)})
    candidates.append((baseline,float('inf'),1))
    no_liftoff=False
    try:
        result=sim.run(baseline,strict=False,record=True)
        candidates[0]=(baseline,result_score(result,scenario.objective),1)
        no_liftoff=result['status']=='No Liftoff' and not sizing
        if result['status']=='Target Achieved' or all(result['constraint_checks'].values()):best_continuous=target_norm(result)
    except SolverDeadline:pass
    except NumericalFailure as exc:messages.append(f'Baseline replay: {exc}')
    # A direct warm fit is useful for ascent, but does not prescribe a parking
    # orbit or replace the general mission structures for high targets.
    warm=None;warm_terminal=None;parking_warm=None;parking_sim=None
    warm_start_info={'elapsed_s':0.,'evaluations':0,'stopped_on_target':False};parking_info={}
    warm_end=start+scenario.constraints.solver_budget_s*.22
    parking=(scenario.target.perigee_altitude_m or 0)>2e6
    if not no_liftoff and time.monotonic()<deadline:
        local_end=min(deadline,time.monotonic()+scenario.constraints.solver_budget_s*.22)
        warm_sim=sim.for_design(baseline.stage_masses)
        original_target=warm_sim.scenario.target
        warm_started=time.monotonic()
        try:
            if parking:
                from .screening import departure_options
                inclination=departure_options(scenario.launch.latitude_deg,original_target,scenario.launch.azimuth_sectors_deg)[2]
                from .screening import seed_shape
                warm_sim.scenario.target=TargetOrbit(perigee_altitude_m=200000,apogee_altitude_m=seed_shape(original_target)[1],inclination_deg=inclination,
                                                altitude_tolerance_m=10000,angle_tolerance_deg=.5)
            warm,fit=warm_start(warm_sim,lambda v:progress({**v,'phase':'Preparing continuous ascent seed','score_units':'seed residual'}),'local_orbital',stop_on_target=True)
            if warm:
                warm=warm.model_copy(update={'stage_masses':baseline.stage_masses,'burns':[resample_burn(b,scenario.constraints.control_nodes) for b in warm.burns]})
        except (SolverDeadline,NumericalFailure):pass
        finally:
            local_end=None;warm_sim.scenario.target=original_target
            warm_start_info={'elapsed_s':time.monotonic()-warm_started,'evaluations':getattr(warm_sim,'warm_evaluations',0),
                             'stopped_on_target':getattr(warm_sim,'warm_stopped_on_target',False)}
        if warm:
            # A deadline can return a useful seed without a fit result. Score
            # that actual flight before choosing which structure to refine.
            candidates.append((warm,1e10,1))
            try:
                seed_result=fit if fit is not None and not parking and not sizing else sim.run(warm,strict=False,record=True)
                candidates[-1]=(warm,result_score(seed_result,scenario.objective),1)
                if seed_result['orbit']['bound'] and seed_result['orbit']['perigee_altitude_m']>=0:
                    if parking:
                        warm_sim.run(warm.model_copy(update={'stage_masses':None}),strict=False,record=False)
                        warm_terminal=(warm_sim._last_state.copy(),seed_result['summary']['arrival_time_s'])
                    norm=target_norm(seed_result)
                    best_continuous=norm if best_continuous is None else min(best_continuous,norm)
            except SolverDeadline:pass
            except NumericalFailure as exc:messages.append(f'Ascent seed replay: {exc}')
    def search(plan,template,mesh,allowance,level):
        nonlocal local_end,iterations,best_continuous
        check();search_start=time.monotonic();local_end=min(deadline,search_start+allowance)
        report={'template':template,'burn_count':len(plan.burns),'mesh_segments':mesh,'control_nodes':len(plan.burns[0].throttles),
                'refinement_level':level,'stop_reason':'solver_stopped','solver_messages':[],'iterations':0,'invalid_candidates':0,'numerical_failures':0,
                'construction_elapsed_s':None,'solve_elapsed_s':0.}
        problem=None;outplan=plan;polished_success=False
        try:
            progress({'phase':f'Building mission {template}: {len(plan.burns)} burns, mesh {mesh}',
                      'iteration':iterations,'best_continuous_error':best_continuous})
            problem=CollocationProblem(sim,plan,mesh)
            report['construction_elapsed_s']=time.monotonic()-search_start
            offset=iterations
            def update(value):
                nonlocal iterations
                iterations=offset+value['iteration']
                progress({**value,'iteration':iterations,'phase':'Optimizing complete mission','template':template,
                          'score_units':'target tolerances','best_continuous_error':best_continuous})
            solve_start=time.monotonic()
            try:
                outplan,stats=problem.solve(update,scenario.constraints.max_iterations,max(.01,local_end-time.monotonic()))
            finally:report['solve_elapsed_s']=time.monotonic()-solve_start
            report['solver_messages'].append(stats['return_status'])
            status=stats['return_status']
            report['stop_reason']='feasible_candidate' if problem.feasibility_ready else 'converged' if stats['success'] else 'iteration_limit' if status=='Maximum_Iterations_Exceeded' else 'search_time_limit' if status=='Maximum_WallTime_Exceeded' else 'solver_stopped'
            # Objective polishing is only entered after continuous constraints
            # and final tolerances have been satisfied by the NLP candidate.
            nlp_feasible=problem.feasible(problem.last_x)
            if nlp_feasible:feasible_plans.append((outplan,0,template))
            continuous_feasible=False
            if nlp_feasible:
                # An under-resolved mesh can satisfy its algebraic equations
                # while missing the actual orbit. Do not spend the remaining
                # search allowance polishing cost on that disconnected outcome.
                pre_polish=sim.run(outplan,strict=False,record=True)
                continuous_feasible=pre_polish['status']=='Target Achieved'
                report['pre_polish_replay_status']=pre_polish['status']
                report['pre_polish_replay_target_error']=target_norm(pre_polish)
                candidates.append((outplan,result_score(pre_polish,scenario.objective),template))
                if pre_polish['orbit']['bound'] and pre_polish['orbit']['perigee_altitude_m']>=0:
                    norm=target_norm(pre_polish)
                    best_continuous=norm if best_continuous is None else min(best_continuous,norm)
            if continuous_feasible and time.monotonic()<local_end and problem.iterations<scenario.constraints.max_iterations:
                progress({'phase':'Minimizing complete mission cost','iteration':iterations})
                outplan,polished=problem.solve(update,max(1,scenario.constraints.max_iterations-problem.iterations),local_end-time.monotonic(),polish=True)
                report['solver_messages'].append(polished['return_status'])
                polished_success=bool(polished['success'])
        except SolverDeadline as exc:
            report['stop_reason']=exc.scope
        except (NumericalFailure,RuntimeError,ValueError) as exc:
            # IPOPT failure is a search outcome; environment errors and user
            # cancellation propagate and are never presented as impossibility.
            if problem is not None and problem.interrupted is not None:raise problem.interrupted
            report['stop_reason']='numerical_failure';report['numerical_failures']=1
            report['solver_messages'].append(str(exc)[:600])
        finally:
            local_end=None
            if problem is not None:
                report['iterations']=problem.iterations;report['invalid_candidates']=problem.invalid_candidates
                if problem.best_polish_x is not None:outplan=problem.decode(problem.best_polish_x)
                elif problem.best_x is not None and not polished_success:outplan=problem.decode(problem.best_x)
                reported_x=problem.best_polish_x if problem.best_polish_x is not None else problem.best_x if problem.best_x is not None else problem.last_x
                if reported_x is not None:
                    advance=np.asarray(problem.coast_resolution_function(reported_x)).ravel()
                    report['maximum_coast_mesh_advance_rad']=float(np.max(advance,initial=0))
                    report['coast_mesh_advance_limit_rad']=.5
                    report['steering_conditioning_penalty']=float(problem.conditioning_function(reported_x))
            report['elapsed_s']=time.monotonic()-search_start;reports.append(report)
        # Keep the structure candidate even when its search allocation expired.
        candidates.append((outplan,1e11,template))
        try:
            trial=sim.run(outplan,strict=False,record=True)
            score=result_score(trial,scenario.objective)
            candidates[-1]=(outplan,score,template)
            if trial['status'] in ('Target Achieved','Target Not Reached') and trial['orbit']['bound'] and trial['orbit']['perigee_altitude_m']>=0:
                norm=target_norm(trial)
                best_continuous=norm if best_continuous is None else min(best_continuous,norm)
                progress({'phase':'Continuous mission candidate evaluated','iteration':iterations,
                          'best_continuous_error':best_continuous,'score_units':'target tolerances'})
            report['replay_target_error']=target_norm(trial);report['replay_status']=trial['status']
        except SolverDeadline:pass
        except NumericalFailure as exc:
            report['solver_messages'].append(f'Continuous replay failed: {exc}')
        return outplan
    if (not no_liftoff and needs_node_transfer(scenario) and any(t.counts[-1]==3 for t in topologies)
            and time.monotonic()<warm_end):
        local_end=min(deadline,warm_end);parking_started=time.monotonic()
        parking_masses=baseline.stage_masses
        if sizing:
            # A plane-changing arrival needs a useful fuel reserve. Seed the
            # freely sized upper stage at its allowed structural dry-mass
            # floor instead of clipping an underfunded arrival maneuver.
            parking_masses=[m.model_copy() for m in baseline.stage_masses]
            bound=scenario.vehicle.stages[-1].mass_bounds
            mass=parking_masses[-1]
            mass.dry_mass_kg=max(bound.dry_mass_min_kg,bound.hardware_mass_kg+bound.tank_structure_ratio*mass.propellant_mass_kg)
        parking_sim=sim.for_design(parking_masses)
        # Use a separate design simulation so this temporary seed target cannot
        # mutate the full-mission target when fixed-mass mode returns sim itself.
        if parking_sim is sim:
            parking_sim=Simulation(scenario.model_copy(deep=True),check=check,atmosphere=sim.atmosphere,frames=sim.frames)
            parking_sim.initial=sim.initial.copy()
        original_target=parking_sim.scenario.target
        try:
            from .screening import departure_options
            inclination=departure_options(scenario.launch.latitude_deg,original_target,scenario.launch.azimuth_sectors_deg)[2]
            parking_sim.scenario.target=TargetOrbit(perigee_altitude_m=250000,apogee_altitude_m=250000,inclination_deg=inclination,
                                                   altitude_tolerance_m=50000,angle_tolerance_deg=.5)
            parking_warm,_=warm_start(parking_sim,lambda v:progress({**v,'phase':'Preparing node-aligned parking seed','score_units':'seed residual'}),
                                      'local_orbital',stop_on_target=True)
            if parking_warm:
                parking_warm=parking_warm.model_copy(update={'stage_masses':parking_masses,
                    'burns':[resample_burn(b,scenario.constraints.control_nodes) for b in parking_warm.burns]})
        except (SolverDeadline,NumericalFailure):pass
        finally:
            local_end=None;parking_sim.scenario.target=original_target
            parking_info={'elapsed_s':time.monotonic()-parking_started,'evaluations':getattr(parking_sim,'warm_evaluations',0),
                          'stopped_on_seed_target':getattr(parking_sim,'warm_stopped_on_target',False)}
    if not no_liftoff:
        coarse_end=start+scenario.constraints.solver_budget_s*((.55 if parking else .65) if scenario.constraints.refinement_levels>1 else (.7 if parking else 1.))
        for i,topology in enumerate(topologies):
            if time.monotonic()>=coarse_end:break
            plan=seed_from_ascent(scenario,topology,warm,False) if warm else seed_plan(scenario,topology)
            if warm_terminal is not None:
                try:
                    plan=seed_from_transfer(warm_sim,topology,warm,*warm_terminal)
                    seed_replay=sim.run(plan,strict=False,record=True)
                    candidates.append((plan,result_score(seed_replay,scenario.objective),i+1))
                    if all(seed_replay['constraint_checks'].values()):
                        norm=target_norm(seed_replay)
                        best_continuous=norm if best_continuous is None else min(best_continuous,norm)
                except (ValueError,NumericalFailure) as exc:messages.append(f'Transfer seed: {exc}')
                except SolverDeadline:break
            if parking_warm is not None and topology.counts[-1]==3:
                try:
                    node_plan=seed_from_parking(parking_sim,topology,parking_warm)
                    seed_replay=sim.run(node_plan,strict=False,record=True)
                    candidates.append((node_plan,result_score(seed_replay,scenario.objective),i+1))
                    # Preserve other initialization candidates for final replay;
                    # use the physically positioned node seed for this structure.
                    plan=node_plan
                    if all(seed_replay['constraint_checks'].values()):
                        norm=target_norm(seed_replay)
                        best_continuous=norm if best_continuous is None else min(best_continuous,norm)
                    parking_info['node_aligned_template']=i+1
                    parking_info['seed_target_error']=target_norm(seed_replay)
                except (ValueError,NumericalFailure) as exc:messages.append(f'Node-aligned seed: {exc}')
                except SolverDeadline:break
            if sizing and plan.stage_masses is None:plan=plan.model_copy(update={'stage_masses':baseline.stage_masses})
            allowance=max(.05,(coarse_end-time.monotonic())/(len(topologies)-i))
            if parking and topology.counts[-1]==1 and len(topologies)>1:
                # Keep this legal alternative, but give separated arrival
                # structures most of the high-perigee search allocation.
                allowance=min(allowance,scenario.constraints.solver_budget_s*.06)
            try:search(plan,i+1,scenario.constraints.mesh_segments,allowance,1)
            except SolverDeadline:break
        # Refine the best continuously replayed mission, not a disconnected
        # shooting-state score. Increased control and state resolution are free
        # to change the whole mission, including every coast and burn duration.
        refinement_end=deadline-scenario.constraints.solver_budget_s*(.30 if parking else .15)
        for level in range(2,scenario.constraints.refinement_levels+1):
            if time.monotonic()>=refinement_end:break
            plan,_,template=min(candidates,key=lambda c:c[1])
            nodes=min(129,(scenario.constraints.control_nodes-1)*2**(level-1)+1)
            refined=plan.model_copy(update={'burns':[resample_burn(b,nodes) for b in plan.burns]})
            mesh=min(256,scenario.constraints.mesh_segments*2**(level-1))
            allowance=(refinement_end-time.monotonic())/(scenario.constraints.refinement_levels-level+1)
            previous=next((r for r in reversed(reports) if r['template']==(template or 1) and r.get('construction_elapsed_s') is not None),None)
            # Mesh construction is part of the search budget. Do not spend
            # the allocation on a larger graph with effectively no solve
            # time; preserve it for continuous correction of the actual flight.
            if previous and previous['construction_elapsed_s']*mesh/previous['mesh_segments']>=allowance*.5:
                messages.append(f'Refinement {level} deferred: estimated graph construction would consume most of its time allocation; retained time for continuous restoration.')
                break
            try:search(refined,template or 1,mesh,allowance,level)
            except SolverDeadline:break
    restoration={};restored_candidate=None
    if not no_liftoff and time.monotonic()<deadline:
        from .restoration import restore_continuous
        plan,_,template=min(candidates,key=lambda c:c[1])
        try:
            remaining=deadline-time.monotonic();passes=[]
            for attempt in range(2 if parking and remaining>=90 else 1):
                allowance=(deadline-time.monotonic())*(.5 if attempt==0 and parking and remaining>=90 else 1.)
                restored,info=restore_continuous(sim,plan,allowance,progress);passes.append(info)
                restored_candidate=(restored,0,template)
                candidates.append((restored,1e11,template))
                trial=sim.run(restored,strict=False,record=True)
                candidates[-1]=(restored,result_score(trial,scenario.objective),template)
                # Recenter on the retained physical flight for a second
                # bounded correction, refreshing control weights/ranges.
                # Every pass still obeys the original design/flight bounds.
                plan=restored
                if info.get('status')=='Target Achieved' or deadline-time.monotonic()<20:break
            restoration={**passes[-1],'passes':passes,
                **{key:sum(p.get(key,0) for p in passes) for key in ('evaluations','newton_iterations','cost_evaluations','segment_cache_hits','segment_cache_misses','elapsed_s')}}
        except (SolverDeadline,NumericalFailure,ValueError) as exc:messages.append(f'Continuous restoration: {exc}')
    search_elapsed=time.monotonic()-start;enforcing=False;local_end=None
    progress({'phase':'Independently verifying complete missions','iteration':iterations})
    failures=[];verified=[];seen=set()
    shortlist=sorted(candidates,key=lambda c:c[1])[:5]+feasible_plans+[(baseline,0,1)]
    if restored_candidate is not None:shortlist.append(restored_candidate)
    if warm:shortlist.append((warm,0,1))
    for plan,_,template in shortlist:
        check();key=plan.model_dump_json()
        if key in seen:continue
        seen.add(key)
        try:verified.append((sim.run(plan,strict=True,record=True),template))
        except NumericalFailure as exc:failures.append(str(exc))
    if not verified:raise NumericalFailure('No mission candidate survived strict forward replay: '+'; '.join(failures))
    result,template=min(verified,key=lambda item:result_score(item[0],scenario.objective))
    verified_error=target_norm(result)
    if result['status'] in ('Target Achieved','Target Not Reached') and result['orbit']['bound'] and result['orbit']['perigee_altitude_m']>=0:
        best_continuous=verified_error if best_continuous is None else min(best_continuous,verified_error)
    exhausted=search_elapsed>=scenario.constraints.solver_budget_s or any(r['stop_reason']=='total_budget' for r in reports)
    result['optimizer']={'method':'Multi-phase Hermite–Simpson collocation + CasADi/IPOPT; continuous shooting and SLSQP cost polishing',
        'iterations':iterations,'elapsed_s':time.monotonic()-start,'search_elapsed_s':search_elapsed,
        'termination_reason':'no_liftoff' if no_liftoff else 'total_budget' if exhausted else 'searches_completed',
        'total_budget_exhausted':exhausted,'searches':reports,'selected_template':template,'objective':scenario.objective,
        'invalid_candidates':sum(r['invalid_candidates'] for r in reports),'numerical_failures':sum(r['numerical_failures'] for r in reports),
        'verification_failures':failures,'messages':messages,
        'note':'A bounded topology portfolio and local NLP searches are not a global optimality or infeasibility proof.',
        'mission':{'legal_structure_count':total_structures,'selected_structure_count':len(topologies),
                   'structures':[t.name for t in topologies],'phases':phase_manifest(scenario,FlightPlan.model_validate(result['flight_plan'])),
                   'derivatives':('Automatic differentiation of production equations; exact atmosphere-cell and Earth-frame interpolation slopes; terminal-angle finite differences'
                                  if hasattr(sim.atmosphere,'sample_with_derivatives') else
                                  'Sparse CasADi assembly with guarded finite differences for custom atmosphere dynamics'),
                   'refinement_levels':scenario.constraints.refinement_levels,
                   'warm_start':warm_start_info,
                   'parking_initialization':parking_info,
                   'continuous_restoration':restoration,
                   'best_continuous_error':best_continuous,'verified_target_error':verified_error}}
    result['metadata']['optimizer_backend']='CasADi/IPOPT'
    import casadi
    result['metadata']['casadi_version']=casadi.__version__
    result['input_scenario']=input_snapshot
    result['consistency']=analyze_consistency(Scenario.model_validate(result['scenario']),result)
    result['optimizer']['stage_mass_optimization']=sizing
    return result
