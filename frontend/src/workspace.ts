export const branches = [
  [{id:'shape',label:'Orbit definition'},{id:'orientation',label:'Orientation & timing'},{id:'tolerances',label:'Acceptance'}],
  [{id:'location',label:'Location'},{id:'time',label:'Launch time'},{id:'range',label:'Departure sectors'},{id:'recommendations',label:'Launch options'},{id:'terrain',label:'Elevation'},{id:'files',label:'Terrain files'}],
  [{id:'model',label:'Atmosphere model'},{id:'profile',label:'Density & temperature'}],
  [{id:'payload',label:'Payload & mass'},{id:'stage1',label:'Stage 01 · Booster'},{id:'stage2',label:'Stage 02 · Upper'},{id:'sizing',label:'Sizing bounds'},{id:'capacity',label:'Capacity check'}],
  [{id:'steering',label:'Steering envelope'},{id:'limits',label:'Flight limits'}],
  [{id:'mission',label:'Run'},{id:'solver',label:'Optimizer'},{id:'capacity',label:'Readiness'},{id:'overview',label:'Results'},{id:'verification',label:'Acceptance'},{id:'diagnostics',label:'Diagnosis'},{id:'trajectory',label:'Trajectory'},{id:'orbit',label:'Orbit'},{id:'fuel',label:'Propellant'},{id:'charts',label:'Charts'},{id:'events',label:'Events'},{id:'models',label:'Models'}],
] as const;

export function nextWorkspaceSection(step:number,section:string){
  const list=branches[step];
  const index=list.findIndex(item=>item.id===section);
  if(index>=0&&index<list.length-1)return {step,section:list[index+1].id};
  const next=Math.min(step+1,branches.length-1);
  return {step:next,section:branches[next][0].id};
}

export function previousWorkspaceSection(step:number,section:string){
  const index=branches[step].findIndex(item=>item.id===section);
  if(index>0)return {step,section:branches[step][index-1].id};
  const previous=Math.max(0,step-1);
  return {step:previous,section:step===0?branches[0][0].id:branches[previous].at(-1)!.id};
}
