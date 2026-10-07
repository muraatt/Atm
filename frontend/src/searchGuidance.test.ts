import {describe,it,expect} from 'vitest';
import type {Result} from './types';
import {searchGuidance} from './searchGuidance';

describe('search termination and physical acceptance',()=>{
  const missed={status:'Target Not Reached',target_comparison:[{within_tolerance:false}],optimizer:{total_budget_exhausted:false,searches:[{stop_reason:'iteration_limit'},{stop_reason:'iteration_limit'},{stop_reason:'feasible_candidate',pre_polish_replay_status:'Target Not Reached'}]}} as unknown as Result;
  it('explains iteration exhaustion even when the total time was not exhausted',()=>{
    const messages=searchGuidance(missed);
    expect(messages).toHaveLength(2);
    expect(messages[0]).toContain('2 searches reached the iteration limit');
    expect(messages[0]).toContain('does not establish physical infeasibility');
    expect(messages[1]).toContain('continuous flight replay');
  });
  it('does not describe earlier unsuccessful searches as a failed final flight',()=>{
    expect(searchGuidance({...missed,status:'Target Achieved'})).toEqual([]);
  });
  it('does not recommend target-error correction for a constraint-only failure',()=>{
    expect(searchGuidance({...missed,target_comparison:[]})).toEqual([]);
  });
  it('keeps old results without detailed search reports readable',()=>{
    expect(searchGuidance({...missed,optimizer:{...missed.optimizer,searches:undefined}})).toEqual([]);
  });
});
