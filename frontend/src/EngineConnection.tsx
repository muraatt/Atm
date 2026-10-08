import {useState} from 'react';
import {engineBase,normalizeEngineUrl} from './connection';

export function EngineConnection(){
  const [open,setOpen]=useState(false),[address,setAddress]=useState(engineBase()||'http://127.0.0.1:8000'),[busy,setBusy]=useState(false),[error,setError]=useState('');
  const connect=async()=>{
    setBusy(true);setError('');
    try{
      const base=normalizeEngineUrl(address),response=await fetch(base+'/api/health',{signal:AbortSignal.timeout(8000)});
      const health=await response.json();
      if(!response.ok||health.status!=='ok'||health.application!=='atmosphere')throw new Error('This address is not an Atmosphere calculation engine.');
      localStorage.setItem('atmosphere-engine-url',base);localStorage.removeItem('atmosphere-review-only');location.reload();
    }catch{setError('Could not connect. Start Atmosphere on this computer, allow this website in start-web.cmd, and permit local network access if your browser asks. If an embedded browser blocks the connection, open this website in Chrome or Edge.');}
    finally{setBusy(false);}
  };
  return <div className="engine-connection"><button className="button secondary small" aria-expanded={open} onClick={()=>setOpen(!open)}>Local engine</button>{open&&<section className="card connection-popover" aria-label="Local engine connection"><h3>Calculate on this computer</h3><p className="muted">Edit missions online and review saved flights. Full trajectory analysis runs on your computer.</p><label className="field"><span className="field-label">Local engine address</span><input value={address} onChange={e=>setAddress(e.target.value)} placeholder="http://127.0.0.1:8000"/></label><p className="field-hint">Start Atmosphere using start-web.cmd with this website's address. If your browser blocks local connections, open the local workspace and load your saved mission.</p>{error&&<p role="alert" className="error">{error}</p>}<div className="inline-actions"><button className="button primary small" disabled={busy} onClick={()=>void connect()}>{busy?'Connecting…':'Connect'}</button><button className="button secondary small" onClick={()=>{localStorage.removeItem('atmosphere-engine-url');localStorage.setItem('atmosphere-review-only','true');location.reload();}}>Work offline</button></div><a className="text-button" href="http://127.0.0.1:8000/" target="_blank" rel="noreferrer">Open local workspace →</a></section>}</div>;
}
