import {describe,it,expect} from 'vitest';
import {applyEngine,changeEngineCount,engineCatalog,removeDesignStage} from './engines';
import type {Stage} from './types';
const stage={name:'Upper',dry_mass_kg:3500,propellant_mass_kg:65000,vacuum_thrust_n:800000,isp_sea_level_s:310,isp_vacuum_s:360,reference_area_m2:7,cd:.3,cd_table:[],throttle_min:.4,throttle_max:1,ignition_limit:2,separation_delay_s:2,max_thrust_velocity_angle_deg:15,solid_burn_curve:[],engine_type:'liquid'} as Stage;
const engine=engineCatalog.engines.find(e=>e.id==='rutherford-vac')!;
describe('catalog engine application',()=>{
  it('uses aggregate cluster thrust without scaling Isp, fuel or dry mass',()=>{
    const applied=applyEngine(stage,engine,2,25800,343,200);
    expect(applied.vacuum_thrust_n).toBe(51600);expect(applied.isp_vacuum_s).toBe(343);
    expect(applied.dry_mass_kg).toBe(stage.dry_mass_kg);expect(applied.propellant_mass_kg).toBe(stage.propellant_mass_kg);
    expect(applied.engine_catalog_version).toBe(engineCatalog.version);expect(applied.ignition_limit).toBe(1);
    expect(stage.engine_id).toBeUndefined();
    const increased=changeEngineCount(applied,3);expect(increased.vacuum_thrust_n).toBe(77400);
    expect(increased.vacuum_thrust_n/(9.80665*increased.isp_vacuum_s)).toBeCloseTo(3*25800/(9.80665*343));
  });
  it('rejects missing model values, fractional counts and changed reference ratings',()=>{
    expect(()=>applyEngine(stage,engine,1,25800,343,0)).toThrow();
    expect(()=>applyEngine(stage,engine,1.5,25800,343,200)).toThrow();
    expect(()=>applyEngine(stage,engine,1,30000,343,200)).toThrow();
    expect(()=>applyEngine(stage,engine,1,25800,360,200)).toThrow();
    expect(()=>changeEngineCount(stage,0)).toThrow();
  });
  it('does not relabel an upper-stage motor as a booster after stage deletion',()=>{
    const upper=applyEngine(stage,engine,1,25800,343,200);
    const remaining=removeDesignStage([stage,upper],0);
    expect(remaining[0].engine_id).toBeNull();expect(remaining[0].vacuum_thrust_n).toBe(25800);
    expect(upper.engine_id).toBe('rutherford-vac');
  });
  it('preserves retained stages when deleting a legacy third stage',()=>{
    const upper=applyEngine(stage,engine,1,25800,343,200);
    expect(removeDesignStage([stage,upper,stage],2)).toEqual([stage,upper]);
  });
  it('uses the published Merlin Vacuum rating and throttle range',()=>{
    const merlin=engineCatalog.engines.find(e=>e.id==='merlin-vac')!;
    const applied=applyEngine(stage,merlin,1,981000,348,200);
    expect(applied.vacuum_thrust_n).toBe(981000);
    expect(applied.throttle_min).toBeCloseTo(140679/220500);
    expect(()=>applyEngine(stage,merlin,1,800000,348,200)).toThrow();
  });
  it('does not fix a pressure-unspecified Isp as a vacuum rating',()=>{
    const rutherford=engineCatalog.engines.find(e=>e.id==='rutherford-sl')!;
    expect(rutherford.isp_unspecified_s).toBe(311);
    expect(rutherford.isp_vacuum_s).toBeNull();
    const applied=applyEngine(stage,rutherford,9,25000,330,330*rutherford.sea_level_thrust_n!/25000);
    expect(applied.isp_vacuum_s).toBe(330);
    expect(applied.vacuum_thrust_n*applied.isp_sea_level_s/applied.isp_vacuum_s).toBeCloseTo(190000);
  });
});
