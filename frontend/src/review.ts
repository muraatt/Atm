import type {Result} from './types';
import type {Scenario} from './types';

export function parseScenario(text:string):Scenario {
  const s=JSON.parse(text);
  if(!s?.launch||!Number.isFinite(s.launch.latitude_deg)||!Number.isFinite(s.launch.longitude_deg)||!Number.isFinite(Date.parse(s.launch.epoch))||
     !s.environment||!['msis','standard'].includes(s.environment.model)||!s.target||!s.constraints||!s.vehicle||
     !Number.isFinite(s.vehicle.payload_mass_kg)||!Array.isArray(s.vehicle.stages)||!s.vehicle.stages.length||
     s.vehicle.stages.some((x:{name:unknown;dry_mass_kg:unknown;propellant_mass_kg:unknown})=>typeof x?.name!=='string'||!Number.isFinite(x.dry_mass_kg)||!Number.isFinite(x.propellant_mass_kg)))
    throw new Error('Open a complete Atmosphere scenario JSON file. Flight inputs are validated by the local engine before analysis.');
  return s as Scenario;
}

export function parseAnalysis(text:string):Result {
  const value=JSON.parse(text),object=(x:unknown)=>!!x&&typeof x==='object'&&!Array.isArray(x);
  const finite=(x:unknown)=>typeof x==='number'&&Number.isFinite(x);
  if(!object(value)||typeof value.status!=='string'||!object(value.scenario)||!object(value.orbit)||!object(value.summary)||
     !object(value.constraint_checks)||!object(value.optimizer)||!Array.isArray(value.optimizer.messages)||!object(value.metadata)||
     !Array.isArray(value.warnings)||!Array.isArray(value.events)||!Array.isArray(value.propellant_ledger)||!Array.isArray(value.target_comparison)||
     !Array.isArray(value.actual_orbit_curve)||!Array.isArray(value.target_orbit_curve)||!Array.isArray(value.series)||!value.series.length)
    throw new Error('Open a complete Atmosphere Analysis JSON file, not a scenario file.');
  const s=value.scenario;
  if(!object(s.launch)||!Number.isFinite(Date.parse(s.launch.epoch))||!object(s.environment)||!object(s.target)||!object(s.constraints)||
     !object(s.vehicle)||!finite(s.vehicle.payload_mass_kg)||!Array.isArray(s.vehicle.stages)||!s.vehicle.stages.length||
     s.vehicle.stages.some((stage:{name:unknown;dry_mass_kg:unknown;propellant_mass_kg:unknown})=>typeof stage.name!=='string'||!finite(stage.dry_mass_kg)||!finite(stage.propellant_mass_kg))||
     !finite(value.summary.arrival_time_s)||value.series.some((row:{time_s:unknown;position_gcrs_m:unknown;velocity_gcrs_m_s:unknown})=>
       !finite(row.time_s)||![row.position_gcrs_m,row.velocity_gcrs_m_s].every(p=>Array.isArray(p)&&p.length===3&&p.every(finite))))
    throw new Error('The analysis has incomplete scenario or trajectory data.');
  return value as Result;
}

export function analysisCsv(result:Result):string {
  const vectors=['position_gcrs_m','velocity_gcrs_m_s'];
  const columns=Object.keys(result.series[0]??{}).filter(k=>!vectors.includes(k));
  const all=[...columns,'position_x_m','position_y_m','position_z_m','velocity_x_m_s','velocity_y_m_s','velocity_z_m_s'];
  const escape=(value:unknown)=>value==null?'':String(value).match(/[",\r\n]/)?'"'+String(value).replaceAll('"','""')+'"':String(value);
  return [all.join(','),...result.series.map(row=>[...columns.map(key=>row[key as keyof typeof row]),...row.position_gcrs_m,...row.velocity_gcrs_m_s].map(escape).join(','))].join('\r\n')+'\r\n';
}

async function snapshots():Promise<IDBDatabase> {
  return new Promise((resolve,reject)=>{
    const request=indexedDB.open('atmosphere-review',1);
    request.onupgradeneeded=()=>request.result.createObjectStore('snapshots');
    request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);
  });
}
export async function saveReview(result:Result):Promise<void> {
  const db=await snapshots();
  try{await new Promise<void>((resolve,reject)=>{
    const tx=db.transaction('snapshots','readwrite');tx.objectStore('snapshots').put(result,'latest');
    tx.oncomplete=()=>resolve();tx.onerror=()=>reject(tx.error);tx.onabort=()=>reject(tx.error);
  });}finally{db.close();}
}
export async function loadReview():Promise<Result|null> {
  const db=await snapshots();
  try{return await new Promise<Result|null>((resolve,reject)=>{
    const tx=db.transaction('snapshots'),request=tx.objectStore('snapshots').get('latest');
    request.onsuccess=()=>resolve(request.result??null);request.onerror=()=>reject(request.error);
  });}finally{db.close();}
}
