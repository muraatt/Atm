import type {Scenario,Stage,StageMassBounds} from './types';
export function suggestedMassBounds(stage:Stage):StageMassBounds {
  return {dry_mass_min_kg:.5*stage.dry_mass_kg,dry_mass_max_kg:5*stage.dry_mass_kg,propellant_min_kg:.1*stage.propellant_mass_kg,propellant_max_kg:5*stage.propellant_mass_kg,hardware_mass_kg:.35*stage.dry_mass_kg,tank_structure_ratio:Math.min(1,.65*stage.dry_mass_kg/stage.propellant_mass_kg)};
}
export function sizingValid(s:Scenario):boolean {
  if(s.vehicle.optimize_stage_masses&&((s.constraints.solver_method??'mission')!=='mission'||(s.vehicle.minimum_initial_twr??1.05)<=1||(s.vehicle.minimum_initial_twr??1.05)>5))return false;
  return s.vehicle.stages.every(stage=>{
    const b=stage.mass_bounds;if(!b)return true;
    return Object.values(b).every(Number.isFinite)&&b.dry_mass_min_kg>0&&b.propellant_min_kg>0&&b.hardware_mass_kg>0&&b.dry_mass_max_kg>=b.dry_mass_min_kg&&b.propellant_max_kg>=b.propellant_min_kg&&b.tank_structure_ratio>=0&&b.tank_structure_ratio<=1&&b.hardware_mass_kg+b.tank_structure_ratio*b.propellant_min_kg<=b.dry_mass_max_kg;
  });
}
