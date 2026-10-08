import {describe,it,expect} from 'vitest';
import {branches,nextWorkspaceSection,previousWorkspaceSection} from './workspace';
describe('mission workspace navigation',()=>{
  it('visits sub-sections before moving to the next stage',()=>{
    let current={step:0,section:'shape'};
    const visited=[];
    for(let i=0;i<branches.flat().length;i++){visited.push(current);current=nextWorkspaceSection(current.step,current.section);}
    expect(visited[1]).toEqual({step:0,section:'orientation'});
    expect(visited[3]).toEqual({step:1,section:'location'});
    expect(visited).toContainEqual({step:3,section:'stage2'});
    expect(visited).toContainEqual({step:5,section:'solver'});
    expect(visited.at(-1)).toEqual({step:5,section:'models'});
  });
  it('returns through sub-sections and crosses stage boundaries',()=>{
    expect(previousWorkspaceSection(0,'orientation')).toEqual({step:0,section:'shape'});
    expect(previousWorkspaceSection(0,'shape')).toEqual({step:0,section:'shape'});
    expect(previousWorkspaceSection(1,'location')).toEqual({step:0,section:'tolerances'});
    for(let step=0;step<branches.length;step++){
      for(const section of branches[step].slice(1)){
        const previous=previousWorkspaceSection(step,section.id);
        expect(nextWorkspaceSection(previous.step,previous.section)).toEqual({step,section:section.id});
      }
    }
  });
  it('provides unique accessible section ids within each stage',()=>{
    for(const list of branches)expect(new Set(list.map(x=>x.id)).size).toBe(list.length);
  });
});
