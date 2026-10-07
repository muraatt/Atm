import type {Result} from './types';

export function searchGuidance(result:Pick<Result,'status'|'target_comparison'|'optimizer'>):string[] {
  if(result.status!=='Target Not Reached'||!result.target_comparison.some(row=>!row.within_tolerance))return [];
  const searches=result.optimizer.searches??[],messages:string[]=[];
  const limited=searches.filter(search=>search.stop_reason==='iteration_limit').length;
  if(limited)messages.push(`${limited} searches reached the iteration limit. Increase the maximum iterations per search and allow enough optimization time. This is separate from target tolerances and does not establish physical infeasibility.`);
  if(searches.some(search=>search.pre_polish_replay_status==='Target Not Reached'))messages.push('A candidate passed the discretized model checks but missed the target during continuous flight replay. More mesh refinement and time for continuous correction may reduce this gap; the final target tolerances remain unchanged.');
  return messages;
}
