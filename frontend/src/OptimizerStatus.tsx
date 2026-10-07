import type {Result} from './types';
import {fmt,labels} from './utils';
import {searchGuidance} from './searchGuidance';

const reasons:Record<string,string>={
  total_budget:'Total optimization time reached',search_time_limit:'Search time allowance reached',
  initial_guess_time_limit:'Initial guess time allowance reached',iteration_limit:'Iteration limit reached',
  converged:'Solver converged',solver_stopped:'Stopped before convergence',
  feasible_candidate:'Candidate ready for flight verification',
  numerical_failure:'Numerical failure',searches_completed:'Scheduled searches finished',no_liftoff:'Insufficient liftoff thrust',
};

export function OptimizerStatus({result}:{result:Result}){
  const optimizer=result.optimizer;
  const missed=result.target_comparison.filter(row=>!row.within_tolerance);
  const failedLimits=Object.entries(result.constraint_checks).filter(([,passed])=>!passed);
  return <section className="card">
    <div className="card-heading"><div><span className="eyebrow">Search and flight verification</span><h3>Optimization & acceptance</h3></div></div>
    <p className="muted">{optimizer.termination_reason?reasons[optimizer.termination_reason]??optimizer.termination_reason:'This saved analysis predates detailed search termination reporting.'}</p>
    {result.termination_reason==='Time Limit'&&<div className="note warning">Flight stopped at the maximum mission duration. The remaining commanded phases were not flown. Increase the mission duration only if your mission permits a longer flight; the optimization time budget is a separate setting.</div>}
    <p className="field-hint">{optimizer.method}</p>{optimizer.mission?.verified_target_error!=null&&<p className="field-hint">Combined target error after strict replay: {fmt(optimizer.mission.verified_target_error,3)} tolerance units. Each target parameter is accepted separately.</p>}
    {searchGuidance(result).map(message=><div className="note warning" key={message}>{message}</div>)}
    {optimizer.mission&&<details className="solver-settings"><summary>Complete mission plan</summary><p className="field-hint">{optimizer.mission.selected_structure_count} structures selected from {optimizer.mission.legal_structure_count} legal ignition-count combinations. This is a bounded search, not exhaustive global optimization. Times below describe commanded phases; actual cutoff and separation times appear in Mission events.</p><div className="table-scroll"><table><thead><tr><th>Phase</th><th>Stage</th><th>Start</th><th>Duration</th><th>Control nodes</th></tr></thead><tbody>{optimizer.mission.phases.map((phase,i)=><tr key={i}><td>{phase.kind.replaceAll('_',' ')}</td><td>{phase.stage_index+1}</td><td>{fmt(phase.time_s,1)} s</td><td>{fmt(phase.duration_s,1)} s</td><td>{phase.control_nodes??'—'}</td></tr>)}</tbody></table></div><p className="field-hint">{optimizer.mission.derivatives}</p></details>}
    {optimizer.termination_reason&&<p className="field-hint">Search time: {fmt(optimizer.search_elapsed_s??optimizer.elapsed_s,1)} s / {fmt(result.scenario.constraints.solver_budget_s,0)} s budget. Final flight verification adds time. {optimizer.invalid_candidates??0} invalid candidates rejected.</p>}
    {missed.length>0&&<div className="note warning"><div><strong>Target tolerances not met</strong>{missed.map(row=>{
      const scale=row.parameter.endsWith('_m')?.001:1, unit=scale===1?'°':' km';
      return <p key={row.parameter}>{labels[row.parameter]??row.parameter}: error {row.error===null?'undefined':fmt(Math.abs(row.error)*scale,3)+unit}; allowed ±{fmt(row.tolerance*scale,3)}{unit}.</p>;
    })}</div></div>}
    {failedLimits.length>0&&<p className="field-hint">Flight checks not met: {failedLimits.map(([name])=>name.replaceAll('_',' ')).join(', ')}.</p>}
    {optimizer.searches&&optimizer.searches.length>0&&<details className="solver-settings"><summary>Search outcomes</summary><div className="table-scroll"><table>
      <thead><tr><th>Search</th><th>Resolution</th><th>Stop reason</th><th>Iterations</th><th>Time</th><th>Rejected candidates</th></tr></thead>
      <tbody>{optimizer.searches.map((search,index)=><tr key={index}><td>{index+1} · {search.burn_count} burns</td><td>{search.mesh_segments?`${search.mesh_segments} segments / ${search.control_nodes} controls`:'Legacy'}</td><td>{reasons[search.stop_reason]??search.stop_reason}</td><td>{search.iterations}</td><td>{fmt(search.elapsed_s,1)} s{search.construction_elapsed_s!=null&&<p className="field-hint">Setup: {fmt(search.construction_elapsed_s,1)} s · Solve: {fmt(search.solve_elapsed_s,1)} s</p>}</td><td>{search.invalid_candidates??0}</td></tr>)}</tbody>
    </table></div><p className="field-hint">Rejected trials are excluded from candidate selection. The returned trajectory is independently replayed; search convergence alone does not establish success.</p></details>}
    {!!optimizer.verification_failures?.length&&<p className="field-hint">{optimizer.verification_failures.length} candidates failed numerical replay and were excluded.</p>}
  </section>;
}
