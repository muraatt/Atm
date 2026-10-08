import {PanelTabs} from './PanelTabs';
import {useEffect,useState} from 'react';
import type {BudgetAssumptions,MissionBudget as Budget,Scenario} from './types';
import {fmt} from './utils';

function PlanningInput({label,value,unit,max,onApply}:{label:string;value:number;unit:string;max:number;onApply?:(value:number)=>void}){
  const [draft,setDraft]=useState(String(value));const [error,setError]=useState('');
  useEffect(()=>{setDraft(String(value));setError('');},[value]);
  return <label className="field"><span className="field-label">{label}</span><span className="input-wrap"><input type="number" min={0} max={max} step="any" disabled={!onApply} value={draft} onChange={e=>setDraft(e.target.value)} onBlur={()=>{const number=Number(draft);if(draft.trim()===''||!Number.isFinite(number)||number<0||number>max){setError(`Enter a value from 0 to ${max}.`);return;}setError('');onApply?.(number);}}/><span className="input-unit">{unit}</span></span>{error&&<small className="error">{error}</small>}</label>;
}

export function MissionBudget({budget,scenario,onChange}:{budget:Budget;scenario:Scenario;onChange?:(value:BudgetAssumptions)=>void}){
  const assumptions=onChange?(scenario.budget??budget.assumptions):budget.assumptions;
  const margin=budget.tiers.at(-1)!;
  const onward=budget.onward_geo;
  return <div className="mission-budget"><div className="card-heading"><div><span className="eyebrow">Payload & propellant planning</span><h3>Mission Budget</h3></div><span className="badge">Reference estimate</span></div>
    <PanelTabs label="Mission budget pages" pages={[
{id:'loads',label:'Fuel loads',content:<>    <p className="field-hint">Fuel estimates for {fmt(scenario.vehicle.payload_mass_kg/1000,2)} t payload. Fixed dry masses; required fuel loads are sized backward through the stages. Existing fuel load: {fmt(margin.total_available_fuel_kg===undefined?null:margin.total_available_fuel_kg/1000,2)} t. Estimates never change your vehicle or solver settings.</p>
    <div className="budget-tiers">{budget.tiers.map(tier=><div key={tier.name}><span>{tier.name}</span><strong>{fmt(tier.delta_v_m_s/1000,2)} <small>km/s</small></strong><p>{tier.available?`${fmt(tier.total_required_fuel_kg!/1000,2)} t required fuel`:tier.reason}</p>{tier.available&&<small className={tier.within_stage_capacities?'':'error'}>{tier.within_stage_capacities?'Within reference stage loads':`${fmt(tier.stage_shortfall_kg!/1000,2)} t shortfall across stages`}</small>}</div>)}</div>

</>},
{id:'ledger',label:'Stage ledger',content:<>    <div className="table-scroll"><table><thead><tr><th>Stage · with margin</th><th>Required load</th><th>Available load</th><th>Ascent burn</th><th>Arrival burn</th><th>Protected reserve</th><th>Shortfall</th></tr></thead><tbody>{margin.stages.map(row=><tr key={row.stage_index}><td>{row.stage}{!row.resizable&&' · fixed solid'}</td>{[row.required_fuel_kg,row.available_fuel_kg,row.ascent_fuel_kg,row.arrival_fuel_kg,row.protected_reserve_kg,row.shortfall_kg].map((value,i)=><td key={i} className={i===5&&value>.01?'error':''}>{fmt(value/1000,2)} t</td>)}</tr>)}</tbody></table></div>
    <p className="field-hint">Arrival fuel: {fmt(margin.arrival_fuel_kg===undefined?null:margin.arrival_fuel_kg/1000,2)} t. Protected fuel after arrival: {fmt(margin.protected_reserve_kg===undefined?null:margin.protected_reserve_kg/1000,2)} t. The ideal reference excludes ascent losses and contingency; it is not a global minimum.</p></>},
{id:'losses',label:'Loss allowances',content:<><p className="field-hint">Loss allowances are user estimates, not measured losses or statistical confidence bounds. Defaults: gravity 1,800 m/s, drag 150 m/s, steering 150 m/s, contingency 10% of nominal delta-v. Edit and leave a field to update the budget.</p><div className="fields two">{([
      ['gravity_loss_m_s','Gravity loss allowance','m/s',15000],['drag_loss_m_s','Drag loss allowance','m/s',15000],['steering_loss_m_s','Steering loss allowance','m/s',15000],['contingency_percent','Contingency delta-v','%',100]
    ] as const).map(([key,label,unit,max])=><PlanningInput key={key} label={label} value={assumptions[key]} unit={unit} max={max} onApply={onChange?value=>onChange({...assumptions,[key]:value}):undefined}/>)}</div>
</>},
{id:'maneuvers',label:'Maneuvers',content:<>      <p className="field-hint">Reference orbit: {fmt(budget.reference.perigee_altitude_m/1000,1)} × {fmt(budget.reference.apogee_altitude_m/1000,1)} km. Reference departure inclination: {fmt(budget.reference.departure_inclination_deg,2)}°. Plane correction: {fmt(budget.reference.plane_change_deg,2)}°.</p>
      <div className="table-scroll"><table><thead><tr><th>Component</th><th>Delta-v</th></tr></thead><tbody>{budget.components.map(row=><tr key={row.name}><td>{row.name}</td><td>{fmt(row.delta_v_m_s,0)} m/s</td></tr>)}</tbody></table></div><p className="field-hint">Arrival combines speed and plane correction at an aligned apogee/node; separate maneuvers are not added again. Coplanar arrival alone: {fmt(budget.coplanar_arrival_delta_v_m_s,0)} m/s.</p></>},
{id:'payload',label:'Payload sensitivity',content:<><p className="field-hint">Same dry masses, reference mission and margin assumptions. Allocation is recalculated for each payload; these are sizing estimates, not demonstrated payload ratings.</p><div className="table-scroll"><table><thead><tr><th>Payload</th><th>Fuel with margin</th><th>Stage shortfall</th></tr></thead><tbody>{budget.payload_sweep.map(row=><tr key={row.payload_kg}><td>{fmt(row.payload_kg/1000,2)} t</td><td>{fmt(row.total_required_fuel_kg===undefined?null:row.total_required_fuel_kg/1000,2)} t</td><td>{fmt(row.stage_shortfall_kg===undefined?null:row.stage_shortfall_kg/1000,2)} t</td></tr>)}</tbody></table></div></>},
{id:'notes',label:'Model notes',content:<><p className="field-hint">{budget.method}</p><p className="field-hint">{budget.reserve_policy}</p><div className="data-scroll">    <div className="consistency-findings">{budget.warnings.map(warning=><div className="note warning" key={warning}>{warning}</div>)}</div></div></>},
...(onward?[{id:'onward',label:'Onward GEO',content:<>    {onward&&<div className="budget-onward"><h3>Onward GEO estimate</h3><p className="field-hint">From this saved flight's remaining fuel, separate from its accepted target.</p><div className="consistency-flight-stats"><span>Remaining ideal delta-v <b>{fmt(onward.remaining_delta_v_m_s,0)} m/s</b></span><span>Coplanar circularization <b>{fmt(onward.coplanar_delta_v_m_s,0)} m/s</b></span><span>Aligned equatorial GEO maneuver <b>{fmt(onward.aligned_node_geo_delta_v_m_s,0)} m/s</b></span></div><p className="field-hint">Remaining fuel: {fmt(onward.remaining_fuel_kg/1000,3)} t. Required for coplanar circularization: {fmt(onward.coplanar_fuel_kg===null?null:onward.coplanar_fuel_kg/1000,2)} t; aligned equatorial GEO reference: {fmt(onward.aligned_node_geo_fuel_kg===null?null:onward.aligned_node_geo_fuel_kg/1000,2)} t.</p><p className="field-hint">{onward.note}</p></div>}
</>}]:[])
    ]}/>
  </div>;
}

