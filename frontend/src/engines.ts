import catalog from '../../shared/engines.json';
import type {Stage} from './types';
export const engineCatalog = catalog;
export type Engine = (typeof catalog.engines)[number];
export const findEngine = (id?:string|null)=>catalog.engines.find(e=>e.id===id);
export function applyEngine(stage:Stage, engine:Engine, count:number, vacuumThrust:number, vacuumIsp:number, seaIsp:number):Stage {
  if(!Number.isInteger(count)||count<1||count>40||![vacuumThrust,vacuumIsp,seaIsp].every(n=>Number.isFinite(n)&&n>0)||seaIsp>vacuumIsp)throw new Error('Enter a valid engine count, thrust and Isp values. Sea-level Isp cannot exceed vacuum Isp.');
  if(engine.vacuum_thrust_n!==null&&vacuumThrust!==engine.vacuum_thrust_n)throw new Error('Use the published per-engine vacuum thrust.');
  if(engine.isp_vacuum_s!==null&&vacuumIsp!==engine.isp_vacuum_s)throw new Error('Use the published vacuum Isp reference.');
  if(engine.sea_level_thrust_n!==null&&Math.abs(seaIsp-vacuumIsp*engine.sea_level_thrust_n/vacuumThrust)>1e-6)throw new Error('Sea-level Isp must reproduce the published sea-level thrust with the model mass flow.');
  // Mass, drag and guidance constraints stay with the user's stage design.
  return {...stage,engine_id:engine.id,engine_count:count,engine_catalog_version:catalog.version,engine_type:'liquid',vacuum_thrust_n:vacuumThrust*count,isp_vacuum_s:vacuumIsp,isp_sea_level_s:seaIsp,throttle_min:engine.throttle_min??1,throttle_max:1,ignition_limit:engine.ignition_limit,solid_burn_curve:[]};
}
export function changeEngineCount(stage:Stage,count:number):Stage {
  if(!Number.isInteger(count)||count<1||count>40)throw new Error('Engine count must be an integer from 1 to 40.');
  return {...stage,engine_count:count,vacuum_thrust_n:stage.vacuum_thrust_n/(stage.engine_count??1)*count};
}
export function removeDesignStage(stages:Stage[],index:number):Stage[] {
  return stages.filter((_,i)=>i!==index).map((stage,i)=>{
    const engine=findEngine(stage.engine_id);
    return engine&&engine.role!==(i===0?'booster':'upper')?{...stage,engine_id:null,engine_catalog_version:null}:stage;
  });
}
