import {PanelTabs} from './PanelTabs';
import type {SiteScreening} from './types';
import {fmt} from './utils';

export function LaunchRecommendations({screening,busy,error,onSelect,onWindow}:{screening:SiteScreening|null;busy:boolean;error:string;onSelect:(id:string)=>void;onWindow:(epoch:string)=>void}){
  return <section className="card recommendation-card"><div className="card-heading"><div><span className="eyebrow">Orbit-driven selection</span><h3>Recommended launch options</h3></div></div>
    {busy?<p className="muted">Screening launch geometry and reference departure sectors…</p>:error?<p className="error">{error}</p>:screening&&<>
      <PanelTabs label="Launch screening pages" pages={[
{id:'options',label:'Launch options',content:<>      <p className="field-hint">Ranking estimates Earth-rotation benefit and plane-change cost. Vehicle capability is evaluated after the vehicle is defined.</p>
      <div className="recommendation-list">{screening.sites.slice(0,5).map((site,i)=><div className="recommendation-option" key={site.id}><div className="recommendation-title"><b>{i+1}</b><div><strong>{site.name}</strong><span>{site.pad} · {site.country}</span></div><button className="button secondary small" onClick={()=>onSelect(site.id)}>Select</button></div><span className={`badge ${site.status==='Direct reference match'?'success':'warning'}`}>{site.status}</span>{!site.range_verified&&site.status!=='Range data unavailable'&&<span className="badge warning">Range data unavailable</span>}<div className="recommendation-metrics"><span>Departure heading <b>{fmt(site.recommended_azimuth_deg)}°</b></span><span>Plane change <b>{fmt(site.minimum_plane_change_deg,2)}°</b></span><span>Rotation benefit <b>{fmt(site.rotation_benefit_m_s,0)} m/s</b></span></div><p className="field-hint">{site.range_reference}{site.range_source_url&&<> · <a href={site.range_source_url} target="_blank" rel="noreferrer">Source</a></>}</p>{site.launch_windows_utc.length>0&&<div className="launch-window-list"><span>Estimated plane-crossing windows (UTC)</span>{site.launch_windows_utc.map(epoch=><button className="text-button" key={epoch} onClick={()=>{onSelect(site.id);onWindow(epoch);}}>{new Date(epoch).toISOString().slice(0,19).replace('T',' ')} · Use time</button>)}</div>}</div>)}</div>
      {screening.sites.length===0&&<p className="muted">No sites match this filter. Include maneuver-required sites or use a manual launch site.</p>}
</>},
{id:'method',label:'Ranking method',content:<><p className="field-hint">{screening.note}</p><p className="field-hint">{screening.window_note}</p><p className="field-hint">Altitude, periapsis orientation and arrival phase are enforced by the trajectory optimizer. Missing range data never counts as verified access.</p></>},
      ]}/>
    </>}
  </section>;
}
