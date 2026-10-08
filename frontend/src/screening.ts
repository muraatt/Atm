import {launchSites} from './launchSites';
import type {LaunchSite,SiteScreening,Target} from './types';

const rad=Math.PI/180,mu=3.986004418e14,radius=6378137,omega=7.292115e-5;
const bearing=(v:number)=>(v%360+360)%360;
export function azimuthMargin(az:number,sectors:readonly (readonly number[])[]):number {
  if(!sectors.length)return 180;
  return Math.max(...sectors.map(([start,end])=>{
    if(end-start===360)return 180;
    const width=bearing(end-start),offset=bearing(az-start);
    return offset<=width?Math.min(offset,width-offset):-Math.min(offset-width,360-offset);
  }));
}

// Same reference geometry as the local screening engine; no flight simulation.
export function screenLaunchSites(target:Target,launch:LaunchSite):SiteScreening {
  const rp=target.perigee_altitude_m??Math.min(200000,target.apogee_altitude_m??200000);
  const ra=Math.max(rp,target.apogee_altitude_m??rp);
  const speed=Math.sqrt(mu*(2/(radius+rp)-2/(2*radius+rp+ra)));
  const sites=launchSites.map(entry=>{
    const override=launch.launch_site_id===entry.id&&launch.range_source?.startsWith('User-defined');
    const sectors=override?(launch.azimuth_sectors_deg??[]):entry.azimuth_sectors_deg;
    const rotation=omega*radius*Math.cos(entry.latitude_deg*rad),desired=target.inclination_deg;
    let candidates:{error:number;heading:number;inclination:number}[]=[];
    if(desired!==null){
      const ratio=Math.cos(desired*rad)/Math.max(1e-12,Math.cos(entry.latitude_deg*rad));
      if(Math.abs(ratio)<=1+1e-10){
        const angle=Math.asin(Math.max(-1,Math.min(1,ratio)))/rad;
        for(const a of [angle,180-angle]){
          const heading=bearing(Math.atan2(speed*Math.sin(a*rad)-rotation,speed*Math.cos(a*rad))/rad);
          if(azimuthMargin(heading,sectors)>=-1e-6)candidates.push({error:0,heading,inclination:desired});
        }
      }
    }
    if(!candidates.length){
      const headings=sectors.length?sectors.flatMap(([a,b])=>Array.from({length:361},(_,i)=>bearing(a+((b>a?b:b+360)-a)*i/360))):Array.from({length:721},(_,i)=>i/2);
      candidates=headings.map(heading=>{
        const a=heading*rad,v=-rotation*Math.sin(a)+Math.sqrt(Math.max(0,speed**2-rotation**2*Math.cos(a)**2));
        const inclination=Math.acos(Math.max(-1,Math.min(1,Math.cos(entry.latitude_deg*rad)*(v*Math.sin(a)+rotation)/speed)))/rad;
        return {heading,inclination,error:desired===null?0:Math.abs(inclination-desired)};
      });
    }
    candidates.sort((a,b)=>Math.round(a.error*1e8)-Math.round(b.error*1e8)||rotation*(Math.sin(b.heading*rad)-Math.sin(a.heading*rad)));
    const best=candidates[0],benefit=rotation*Math.sin(best.heading*rad),penalty=2*speed*Math.sin(best.error*rad/2),verified=!!sectors.length;
    return {...entry,azimuth_sectors_deg:sectors as [number,number][],range_reference:override?launch.range_source!:entry.range_reference,range_source_url:override?null:entry.range_source_url,
      status:verified&&best.error<=target.angle_tolerance_deg?'Direct reference match':best.error>target.angle_tolerance_deg?'Maneuver required':'Range data unavailable',
      range_verified:verified,recommended_azimuth_deg:best.heading,reference_inclination_deg:best.inclination,minimum_plane_change_deg:best.error,
      plane_change_estimate_m_s:penalty,rotation_benefit_m_s:benefit,ranking_cost_m_s:penalty-benefit+(verified?0:250),launch_windows_utc:[]};
  }).sort((a,b)=>a.ranking_cost_m_s-b.ranking_cost_m_s);
  return {sites,note:'Preliminary departure geometry, Earth rotation and impulsive plane-change estimates are evaluated in this browser using the local engine’s reference equations. This ranking is not a feasible trajectory, fuel estimate or range clearance. Unpublished departure sectors remain unverified.',window_note:target.raan_deg!==null?'Connect the local engine for date-specific RAAN crossing windows and precise Earth orientation. RAAN feasibility is not established by this offline ranking.':'No RAAN crossing window is requested. Precise departure timing is evaluated by the local engine.'};
}
