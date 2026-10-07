import {describe,expect,it} from 'vitest';
import {analysisCsv,parseAnalysis,parseScenario} from './review';
import scenario from '../../shared/default-scenario.json';

const analysis={status:'Target Not Reached',scenario,orbit:{},summary:{arrival_time_s:12},constraint_checks:{},optimizer:{messages:[]},metadata:{},warnings:[],events:[],propellant_ledger:[],target_comparison:[],actual_orbit_curve:[],target_orbit_curve:[],series:[{time_s:12,phase:'Burn, "final"',temperature_k:null,position_gcrs_m:[1,2,3],velocity_gcrs_m_s:[4,5,6]}]};
describe('saved flight review',()=>{
  it('loads a complete snapshot without a calculation service and preserves inputs',()=>{
    expect(parseAnalysis(JSON.stringify(analysis)).scenario.target).toEqual(scenario.target);
    expect(parseScenario(JSON.stringify(scenario)).vehicle.payload_mass_kg).toBe(scenario.vehicle.payload_mass_kg);
  });
  it('rejects scenarios mistaken for results and incomplete or invalid trajectories',()=>{
    expect(()=>parseAnalysis(JSON.stringify(scenario))).toThrow('Analysis JSON');
    expect(()=>parseAnalysis(JSON.stringify({...analysis,series:[{time_s:12,position_gcrs_m:[1,2],velocity_gcrs_m_s:[4,5,6]}]}))).toThrow('trajectory');
    expect(()=>parseScenario(JSON.stringify({target:scenario.target}))).toThrow('scenario JSON');
  });
  it('exports vector columns and escapes CSV text without converting missing temperature to zero',()=>{
    const csv=analysisCsv(parseAnalysis(JSON.stringify(analysis)));
    expect(csv).toContain('position_x_m,position_y_m,position_z_m,velocity_x_m_s,velocity_y_m_s,velocity_z_m_s');
    expect(csv).toContain('12,"Burn, ""final""",,1,2,3,4,5,6\r\n');
  });
});
