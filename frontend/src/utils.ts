import {sizingValid} from './sizing';
import type {Scenario, Stage, Target} from './types';
import {findEngine} from './engines';
import defaults from '../../shared/default-scenario.json';
import {engineApiUrl,hasEngine,markEngineOffline} from './connection';
export const labels:Record<string,string>={perigee_altitude_m:'Perigee altitude',apogee_altitude_m:'Apogee altitude',inclination_deg:'Inclination',raan_deg:'RAAN',argument_of_periapsis_deg:'Argument of periapsis',arrival_phase_deg:'Arrival phase'};
export const fmt=(n:number|null|undefined,digits=1)=>n==null||!Number.isFinite(n)?'—':n.toLocaleString('en-US',{maximumFractionDigits:digits,minimumFractionDigits:digits});
export const initialMass=(s:Scenario)=>s.vehicle.payload_mass_kg+s.vehicle.stages.reduce((sum,x)=>sum+x.dry_mass_kg+x.propellant_mass_kg,0);
export const stageDuration=(s:Stage)=>s.propellant_mass_kg/(s.vacuum_thrust_n/(9.80665*s.isp_vacuum_s));
export function targetCurve(t:Target):number[][] {
  const perigee=t.perigee_altitude_m??Math.min(200000,t.apogee_altitude_m??200000),apogee=t.apogee_altitude_m??perigee;
  const rp=6378137+perigee,ra=6378137+Math.max(perigee,apogee),a=(rp+ra)/2,e=(ra-rp)/(ra+rp);
  const inc=(t.inclination_deg??28.5)*Math.PI/180,raan=(t.raan_deg||0)*Math.PI/180,arg=(t.argument_of_periapsis_deg||0)*Math.PI/180;
  return Array.from({length:241},(_,i)=>{
    const nu=i/240*2*Math.PI,r=a*(1-e*e)/(1+e*Math.cos(nu)),x=r*Math.cos(nu+arg),y=r*Math.sin(nu+arg);
    return [x*Math.cos(raan)-y*Math.cos(inc)*Math.sin(raan),x*Math.sin(raan)+y*Math.cos(inc)*Math.cos(raan),y*Math.sin(inc)];
  });
}
export function groundTrackParts(rows:{latitude_deg:number;longitude_deg:number}[]):number[][][] {
  const parts:number[][][]=[[]];
  rows.forEach((row,i)=>{if(i&&Math.abs(row.longitude_deg-rows[i-1].longitude_deg)>180)parts.push([]);parts.at(-1)!.push([row.longitude_deg,row.latitude_deg]);});
  return parts.filter(p=>p.length>1);
}
export function stepValidity(s:Scenario):boolean[] {
  const v=s.vehicle,t=s.target;
  const targetValid=targetInputsValid(t);
  return [targetValid,s.launch.elevation_resolved&&Math.abs(s.launch.latitude_deg)<=90&&Math.abs(s.launch.longitude_deg)<=180,
    !!s.environment.model,
    sizingValid(s)&&v.payload_mass_kg>0&&v.stages.length>0&&v.stages.length<=2&&v.stages.every((x,i)=>!!findEngine(x.engine_id)&&findEngine(x.engine_id)?.role===(i===0?'booster':'upper')&&x.dry_mass_kg>0&&x.propellant_mass_kg>0&&x.vacuum_thrust_n>0&&x.isp_sea_level_s>0&&x.isp_vacuum_s>=x.isp_sea_level_s&&x.throttle_min<=x.throttle_max&&x.throttle_min>0&&x.throttle_max<=1&&x.throttle_min>=(findEngine(x.engine_id)?.throttle_min??0)-1e-8&&Number.isInteger(x.engine_count??1)&&(x.engine_count??1)>=1&&(x.engine_count??1)<=40),
    s.constraints.max_mission_duration_s>1&&v.stages.every(x=>x.max_thrust_velocity_angle_deg>=0&&x.max_thrust_velocity_angle_deg<=90&&(x.space_thrust_velocity_angle_deg==null||(x.space_thrust_velocity_angle_deg>=0&&x.space_thrust_velocity_angle_deg<=180))),
    true];
}
export function targetInputsValid(t:Scenario['target']):boolean {
  const range=(x:number|null|undefined,min:number,max:number,inclusive=true)=>x==null||(Number.isFinite(x)&&x>=min&&(inclusive?x<=max:x<max));
  const angles=[t.raan_deg,t.argument_of_periapsis_deg,t.arrival_phase_deg];
  const supplied=[t.perigee_altitude_m,t.apogee_altitude_m,t.inclination_deg,...angles].some(x=>x!=null);
  const circular=t.perigee_altitude_m!=null&&t.apogee_altitude_m!=null&&Math.abs(t.perigee_altitude_m-t.apogee_altitude_m)<.001;
  const equatorial=t.inclination_deg!=null&&Math.min(t.inclination_deg,180-t.inclination_deg)<1e-6;
  return supplied&&range(t.perigee_altitude_m,100000,1e9)&&range(t.apogee_altitude_m,100000,1e9)&&range(t.inclination_deg,0,180)&&angles.every(x=>range(x,0,360,false))&&
    (t.perigee_altitude_m==null||t.apogee_altitude_m==null||t.apogee_altitude_m>=t.perigee_altitude_m)&&
    (!circular||t.argument_of_periapsis_deg==null)&&(!equatorial||t.raan_deg==null)&&
    Number.isFinite(t.altitude_tolerance_m)&&t.altitude_tolerance_m>0&&t.altitude_tolerance_m<=100000&&
    Number.isFinite(t.angle_tolerance_deg)&&t.angle_tolerance_deg>0&&t.angle_tolerance_deg<=10&&
    range(t.arrival_time_min_s,0,Infinity)&&range(t.arrival_time_max_s,Number.MIN_VALUE,Infinity)&&
    (t.arrival_time_min_s==null||t.arrival_time_max_s==null||t.arrival_time_min_s<=t.arrival_time_max_s);
}
export async function api<T>(url:string,options?:RequestInit):Promise<T>{
  if(!hasEngine()&&url==='/api/defaults')return structuredClone(defaults) as T;
  if(!hasEngine())throw new Error('Connect your local engine to calculate this. Saved results remain available in review mode.');
  let r:Response;
  try{
    r=await fetch(engineApiUrl(url),url==='/api/defaults'?{...options,signal:AbortSignal.timeout(8000)}:options);
    if(!r.headers.get('Content-Type')?.includes('application/json'))throw new Error('The calculation service did not return an API response. Check the local engine address.');
    if(url==='/api/defaults'&&!r.ok)throw new Error('Calculation engine unavailable.');
  }catch(error){
    if(url==='/api/defaults'){markEngineOffline();return structuredClone(defaults) as T;}
    throw error;
  }
  const data=await r.json();
  if(!r.ok){const detail=data.detail;throw new Error(typeof detail==='string'?detail:Array.isArray(detail)?detail.map((x:{loc:string[];msg:string})=>`${x.loc.slice(1).join(' → ')}: ${x.msg}`).join('; '):'The request could not be completed.');}
  return data;
}
export const post=(data:unknown):RequestInit=>({method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
export function flightInputSignature(s:Scenario):string {
  const {budget:advisory,...flight}=s;void advisory;
  return JSON.stringify(flight,(_key,value)=>value&&typeof value==='object'&&!Array.isArray(value)?Object.fromEntries(Object.keys(value).sort().map(key=>[key,value[key]])):value);
}
export function download(name:string,content:string,type='application/json') {const url=URL.createObjectURL(new Blob([content],{type}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
export function localDateInput(iso:string){const d=new Date(iso);const pad=(n:number)=>`${n}`.padStart(2,'0');return `${d.getFullYear()}-${pad(d.getMonth()+1)}-${pad(d.getDate())}T${pad(d.getHours())}:${pad(d.getMinutes())}`;}

export function targetSummary(target:Target):string {
  return Object.entries(labels).filter(([key])=>target[key as keyof Target]!==null).map(([key,label])=>`${label}: ${fmt((target[key as keyof Target] as number)*(key.endsWith('_m')?.001:1),1)}${key.endsWith('_m')?' km':'°'}`).join(' · ')||'No target parameters specified';
}
