import {memo,useEffect,useMemo,useRef,useState} from 'react';
import type {Data,Layout,PlotlyHTMLElement} from 'plotly.js';
import type {Result,Target} from './types';
import {api,targetCurve} from './utils';
import {globeEpoch,globeGeometry,type EarthRotation,type Point3} from './earthGlobe';

let plotlyPromise:Promise<typeof import('plotly.js-dist-min')>|undefined;
export function Plot({data,layout,className=''}:{data:Data[];layout:Partial<Layout>;className?:string}){
  const ref=useRef<HTMLDivElement>(null);const [error,setError]=useState('');
  useEffect(()=>{let mounted=true;const element=ref.current;let instance:typeof import('plotly.js-dist-min')['default']|undefined;
    plotlyPromise??=import('plotly.js-dist-min');
    plotlyPromise.then(async({default:p})=>{if(!mounted||!element)return;instance=p;await p.react(element,data,{paper_bgcolor:'transparent',plot_bgcolor:'transparent',font:{family:'Inter, Segoe UI, sans-serif',color:'#a9b5c7',size:11},margin:{l:55,r:20,b:45,t:24},...layout},{responsive:true,displaylogo:false,modeBarButtonsToRemove:['sendDataToCloud']});}).catch(e=>mounted&&setError(e.message));
const observer=new ResizeObserver(()=>{if(instance&&element)instance.Plots.resize(element as unknown as PlotlyHTMLElement);});if(element)observer.observe(element);
    return()=>{mounted=false;observer.disconnect();if(instance&&element)instance.purge(element);};
  },[data,layout]);
  return <div className={`plot ${className}`} ref={ref}>{error&&<p className="error">Chart unavailable: {error}</p>}</div>;
}

const orientations=new Map<string,Promise<EarthRotation>>();
function orientation(epoch:string):Promise<EarthRotation>{
  let request=orientations.get(epoch);
  if(!request){
    request=api<{itrs_to_gcrs:EarthRotation}>(`/api/earth/orientation?epoch=${encodeURIComponent(epoch)}`).then(value=>value.itrs_to_gcrs).catch(error=>{orientations.delete(epoch);throw error;});
    orientations.set(epoch,request);
    if(orientations.size>32)orientations.delete(orientations.keys().next().value!);
  }
  return request;
}
export const OrbitView=memo(function OrbitView({target,result,epoch}:{target:Target;result?:Result|null;epoch?:string}){
  const time=result?globeEpoch(result.scenario.launch.epoch,result.summary.arrival_time_s):epoch?globeEpoch(epoch):null;
  const [frame,setFrame]=useState<{time:string;matrix:EarthRotation}|null>(null),[error,setError]=useState(''),[retry,setRetry]=useState(0);
  const savedFrame=result?.metadata.earth_orientation as {epoch:string;itrs_to_gcrs:EarthRotation}|undefined;
  useEffect(()=>{let active=true;setError('');if(time&&savedFrame&&Date.parse(savedFrame.epoch)===Date.parse(time)){setFrame({time,matrix:savedFrame.itrs_to_gcrs});return;}if(time)orientation(time).then(matrix=>{if(active)setFrame({time,matrix});}).catch(()=>{if(active)setError('Geographic orientation unavailable. Orbit scale is preserved.');});return()=>{active=false;};},[time,retry,savedFrame]);
  const matrix=frame?.time===time?frame.matrix:undefined;
  const earth=useMemo(()=>globeGeometry(matrix),[matrix]);
  const path=result?.target_orbit_curve||targetCurve(target);
  const line=(points:number[][],name:string,color:string,width=3):Data=>({type:'scatter3d',mode:'lines',x:points.map(p=>p[0]/1000),y:points.map(p=>p[1]/1000),z:points.map(p=>p[2]/1000),line:{color,width},name,hovertemplate:`${name}<extra></extra>`});
  const mapped=!!matrix||!time;
  const globe:Data & {surfacecolor:number[][];cmin:number;cmax:number;lighting:{ambient:number;diffuse:number;specular:number;roughness:number}}={type:'surface',x:earth.x,y:earth.y,z:earth.z,surfacecolor:mapped?earth.surfacecolor:earth.surfacecolor.map(row=>row.map(()=>0)),cmin:0,cmax:1,colorscale:[[0,'#123347'],[.49,'#123347'],[.5,'#427864'],[1,'#427864']],showscale:false,hoverinfo:'skip',opacity:1,name:'Earth',lighting:{ambient:.85,diffuse:.55,specular:.15,roughness:.9}};
  const data:Data[]=[globe,line(path,'Target orbit','#e3bd80',3)];
  const geographyLine=(points:(Point3|null)[],name:string,color:string,width:number):Data=>({type:'scatter3d',mode:'lines',x:points.map(p=>p?.[0]??null),y:points.map(p=>p?.[1]??null),z:points.map(p=>p?.[2]??null),line:{color,width},hoverinfo:'skip',name,connectgaps:false});
  if(mapped)data.push(geographyLine(earth.grid,'Geographic grid','#285463',1),geographyLine(earth.coastlines,'Coastlines','#9fc7b0',1.5),geographyLine(earth.borders,'Country boundaries','#719b83',1));
  if(result){if(result.actual_orbit_curve.length)data.push(line(result.actual_orbit_curve,'Achieved orbit','#899bfd',3));data.push(line(result.series.map(r=>r.position_gcrs_m),'Flight trajectory','#59e5c1',5));const end=result.series.at(-1)?.position_gcrs_m;if(end)data.push({type:'scatter3d',mode:'markers',x:[end[0]/1000],y:[end[1]/1000],z:[end[2]/1000],marker:{color:'#59e5c1',size:5},name:'Final state'});}
  const layout:Partial<Layout>={margin:{l:0,r:0,b:0,t:0},showlegend:false,scene:{xaxis:{visible:false},yaxis:{visible:false},zaxis:{visible:false},aspectmode:'data',bgcolor:'rgba(0,0,0,0)',camera:{eye:{x:1.4,y:1.5,z:.8}}},uirevision:'orbit-camera'};
  return <div className="orbit-view"><Plot data={data} layout={layout} className="orbit-plot"/><div className="globe-caption"><span>WGS84 · True scale · {result?'Earth at arrival':time?'Earth at launch':'Earth-fixed reference'}</span><span>{time&&!matrix?(error||'Loading geographic orientation…'):time?`${time.slice(0,19).replace('T',' ')} UTC`:''}</span><a href="https://www.naturalearthdata.com/" target="_blank" rel="noreferrer">Map: Natural Earth</a>{error&&<button className="text-button" onClick={()=>setRetry(value=>value+1)}>Retry map</button>}</div></div>;
});
