import {describe,it,expect} from 'vitest';
import {branches,nextWorkspaceSection} from './workspace';
describe('mission workspace navigation',()=>{
  it('visits sub-sections before moving to the next stage',()=>{
    let current={step:0,section:'shape'};
    const visited=[];
    for(let i=0;i<18;i++){visited.push(current);current=nextWorkspaceSection(current.step,current.section);}
    expect(visited[1]).toEqual({step:0,section:'orientation'});
    expect(visited[3]).toEqual({step:1,section:'location'});
    expect(visited).toContainEqual({step:3,section:'stage2'});
    expect(visited).toContainEqual({step:5,section:'solver'});
  });
  it('provides unique accessible section ids within each stage',()=>{
    for(const list of branches)expect(new Set(list.map(x=>x.id)).size).toBe(list.length);
  });
});
