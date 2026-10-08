import {useEffect,useState} from 'react';
import {CircleHelp} from 'lucide-react';
import type {BudgetAssumptions,ConsistencyReport,Scenario} from './types';
import {MissionBudget} from './MissionBudget';
import {api,fmt,post} from './utils';
import {hasEngine} from './connection';

export function ConsistencyPanel({scenario,resultId,savedReport,onBudgetChange}:{scenario:Scenario;resultId?:string;savedReport?:ConsistencyReport;onBudgetChange?:(value:BudgetAssumptions)=>void}){
  const [report,setReport]=useState<ConsistencyReport|null>(null),[error,setError]=useState(''),[busy,setBusy]=useState(true);
  useEffect(()=>{
    if(savedReport){setReport(savedReport);setBusy(false);setError('');return;}
    if(!hasEngine()){setReport(null);setBusy(false);setError('Connect your local engine to calculate planning estimates.');return;}
    const controller=new AbortController();setBusy(true);setError('');
    const timer=setTimeout(()=>api<ConsistencyReport>(resultId?`/api/analyses/${resultId}/diagnostics`:'/api/consistency',resultId?{signal:controller.signal}:{...post(scenario),signal:controller.signal})
      .then(value=>{if(!controller.signal.aborted)setReport(value);})
      .catch(e=>{if(!controller.signal.aborted)setError(e.message);})
      .finally(()=>{if(!controller.signal.aborted)setBusy(false);}),350);
    return()=>{clearTimeout(timer);controller.abort();};
  },[JSON.stringify(scenario),resultId,savedReport]);
  return <section className="card consistency-card"><div className="card-heading"><div><span className="eyebrow">{resultId||savedReport?'Saved flight snapshot':'Before optimization'}</span><h3>{resultId||savedReport?'Flight diagnosis':'Consistency check'}</h3></div>{!busy&&!error&&report&&<span className={`badge ${report.status==='Review required'?'warning':'success'}`}>{report.status}</span>}</div>
    {busy&&report&&<p className="field-hint" role="status">Updating planning estimates…</p>}
    {busy&&!report?<p className="muted">Checking vehicle capacity and mission consistency…</p>:error?<p className={hasEngine()?'error':'muted'}>{error}</p>:report&&<>
      <p className="field-hint">An optimistic performance screen. Passing these checks does not establish that the target is reachable. Warnings do not prevent a diagnostic flight analysis.</p>
      <div className="consistency-stats">{[
        ['Ideal vehicle delta-v',fmt(report.ideal_delta_v_m_s/1000,2),'km/s'],
        ['Energy screening requirement',fmt(report.energy_screen_delta_v_m_s/1000,2),'km/s'],
        ['Optimistic margin',fmt(report.optimistic_margin_m_s/1000,2),'km/s'],
        ['Initial pad T/W',fmt(report.initial_twr,2),''],
      ].map(([label,value,unit])=><div key={label}><span>{label}</span><strong>{value}<small>{unit}</small></strong></div>)}</div>
      {report.diagnostic&&<div className="consistency-flight-stats"><span>Maximum altitude <b>{fmt(report.diagnostic.maximum_altitude_m===null?null:report.diagnostic.maximum_altitude_m/1000,2)} km</b></span><span>Final radial velocity <b>{fmt(report.diagnostic.final_radial_speed_m_s,0)} m/s</b></span><span>Fuel accounting error <b>{fmt(report.diagnostic.propellant_balance_error_kg,4)} kg</b></span></div>}
      <div className="consistency-findings">{report.findings.map(f=><div className={`note ${f.severity==='warning'?'warning':'info'}`} key={f.code}><CircleHelp size={16}/><div><strong>{f.title}</strong><p>{f.message}</p></div></div>)}</div>
      <details><summary>Stage performance & screening assumptions</summary><div className="table-scroll"><table><thead><tr><th>Stage</th><th>Ignition mass</th><th>Burnout mass</th><th>Ideal delta-v</th><th>Full-throttle burn</th></tr></thead><tbody>{report.stages.map((s,i)=><tr key={i}><td>{s.stage}</td><td>{fmt(s.ignition_mass_kg/1000,2)} t</td><td>{fmt(s.burnout_mass_kg/1000,2)} t</td><td>{fmt(s.ideal_delta_v_m_s/1000,2)} km/s</td><td>{fmt(s.full_throttle_burn_s,1)} s</td></tr>)}</tbody></table></div>
        {report.optimistic_payload_ceiling_kg!==null&&<p className="field-hint">Optimistic payload ceiling for this energy screen: {fmt(report.optimistic_payload_ceiling_kg/1000,2)} t. This is not a deliverable payload rating; trajectory losses and plane constraints lower actual capability.</p>}
        <p className="field-hint">{report.assumptions}</p><p className="field-hint"><a href={report.source_url} target="_blank" rel="noreferrer">NASA ideal rocket equation</a></p>
      </details>
      {report.mission_budget&&<MissionBudget budget={report.mission_budget} scenario={scenario} onChange={resultId?undefined:onBudgetChange}/>}
    </>}
  </section>;
}
