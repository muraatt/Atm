import {describe,it,expect} from 'vitest';
import {sizingValid,suggestedMassBounds} from './sizing';
import type {Scenario,Stage} from './types';
describe('joint stage sizing inputs',()=>{
  const stage={dry_mass_kg:14000,propellant_mass_kg:220000} as Stage;
  const scenario={vehicle:{optimize_stage_masses:true,minimum_initial_twr:1.1,stages:[stage]},constraints:{solver_method:'mission'}} as Scenario;
  it('anchors the suggested hardware and tank law to the entered design',()=>{
    const bounds=suggestedMassBounds(stage);
    expect(bounds.hardware_mass_kg+bounds.tank_structure_ratio*stage.propellant_mass_kg).toBeCloseTo(stage.dry_mass_kg);
    expect(bounds.propellant_min_kg).toBe(22000);
    expect(sizingValid({...scenario,vehicle:{...scenario.vehicle,stages:[{...stage,mass_bounds:bounds}]}})).toBe(true);
  });
  it('rejects a structural allowance that cannot fit the minimum fuel load',()=>{
    const bounds={...suggestedMassBounds(stage),dry_mass_max_kg:6000};
    expect(sizingValid({...scenario,vehicle:{...scenario.vehicle,stages:[{...stage,mass_bounds:bounds}]}})).toBe(false);
  });
  it('requires the joint mission solver and a pad thrust margin',()=>{
    expect(sizingValid({...scenario,constraints:{...scenario.constraints,solver_method:'legacy'}})).toBe(false);
    expect(sizingValid({...scenario,vehicle:{...scenario.vehicle,minimum_initial_twr:1}})).toBe(false);
  });
});
