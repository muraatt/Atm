export const isLoopback = (host:string) => ['localhost','127.0.0.1','[::1]','::1'].includes(host.toLowerCase());
export function normalizeEngineUrl(value:string):string {
  const url=new URL(value);
  if(!isLoopback(url.hostname)||!['http:','https:'].includes(url.protocol)||url.username||url.password||url.search||url.hash||!['','/'].includes(url.pathname))
    throw new Error('Use a local engine address, such as http://127.0.0.1:8001.');
  return url.origin;
}
export function engineBase():string|null {
  const saved=localStorage.getItem('atmosphere-engine-url');
  if(saved){try{return normalizeEngineUrl(saved);}catch{/* Ignore stale configuration. */}}
  return isLoopback(location.hostname)&&localStorage.getItem('atmosphere-review-only')!=='true'?'':null;
}
let engineOffline=false;
export const markEngineOffline=()=>{engineOffline=true;};
export const hasEngine=()=>engineBase()!==null&&!engineOffline;
export function engineApiUrl(path:string):string {
  const base=engineBase();
  if(base===null)throw new Error('Connect your local engine to calculate this. Saved results remain available in review mode.');
  return base+path;
}
