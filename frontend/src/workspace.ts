export const branches = [
  [{id:'shape',label:'Orbit definition'},{id:'orientation',label:'Orientation & timing'},{id:'tolerances',label:'Acceptance'}],
  [{id:'location',label:'Location & time'},{id:'recommendations',label:'Launch options'},{id:'terrain',label:'Terrain & elevation'}],
  [{id:'model',label:'Atmosphere model'},{id:'profile',label:'Density & temperature'}],
  [{id:'payload',label:'Payload & mass'},{id:'stage1',label:'Stage 01 · Booster'},{id:'stage2',label:'Stage 02 · Upper'},{id:'sizing',label:'Sizing bounds'},{id:'capacity',label:'Capacity check'}],
  [{id:'steering',label:'Steering envelope'},{id:'limits',label:'Flight limits'}],
  [{id:'mission',label:'Mission & run'},{id:'solver',label:'Optimizer'},{id:'capacity',label:'Readiness check'},{id:'overview',label:'Results overview'},{id:'diagnostics',label:'Flight diagnosis'},{id:'trajectory',label:'Trajectory'},{id:'orbit',label:'Orbit comparison'},{id:'fuel',label:'Propellant'},{id:'charts',label:'Flight charts'},{id:'events',label:'Events & models'}],
] as const;

export function nextWorkspaceSection(step:number,section:string){
  const list=branches[step];
  const index=list.findIndex(item=>item.id===section);
  if(index>=0&&index<list.length-1)return {step,section:list[index+1].id};
  const next=Math.min(step+1,branches.length-1);
  return {step:next,section:branches[next][0].id};
}
