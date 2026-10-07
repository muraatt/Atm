import {describe,it,expect} from 'vitest';
import {groundTrackParts,targetCurve,flightInputSignature,targetInputsValid} from './utils';
import type {Scenario} from './types';
describe('orbit geometry',()=>{
  it('preserves a circular target radius and retrograde inclination',()=>{
    const points=targetCurve({perigee_altitude_m:200000,apogee_altitude_m:200000,inclination_deg:98,raan_deg:0,argument_of_periapsis_deg:null,arrival_phase_deg:null,altitude_tolerance_m:1000,angle_tolerance_deg:.1});
    expect(Math.hypot(...points[60])).toBeCloseTo(6578137,5);
    expect(points[60][1]).toBeLessThan(0);
    expect(points[60][2]).toBeGreaterThan(0);
  });
  it('splits the ground track at the antimeridian',()=>{
    const lines=groundTrackParts([{longitude_deg:170,latitude_deg:0},{longitude_deg:179,latitude_deg:1},{longitude_deg:-179,latitude_deg:2},{longitude_deg:-170,latitude_deg:3}]);
    expect(lines).toEqual([[[170,0],[179,1]],[[-179,2],[-170,3]]]);
  });
});

describe('saved flight comparison',()=>{
  it('ignores object key order and advisory budget edits',()=>{
    const saved={name:'Study',vehicle:{payload_mass_kg:5000},target:{inclination_deg:38}} as Scenario;
    const current={target:{inclination_deg:38},vehicle:{payload_mass_kg:5000},name:'Study',budget:{gravity_loss_m_s:1800,drag_loss_m_s:150,steering_loss_m_s:150,contingency_percent:20}} as Scenario;
    expect(flightInputSignature(saved)).toBe(flightInputSignature(current));
  });
  it('marks changed payload or target as a different flight',()=>{
    const saved={vehicle:{payload_mass_kg:5000},target:{inclination_deg:38}} as Scenario;
    expect(flightInputSignature(saved)).not.toBe(flightInputSignature({...saved,vehicle:{...saved.vehicle,payload_mass_kg:6000}}));
    expect(flightInputSignature(saved)).not.toBe(flightInputSignature({...saved,target:{...saved.target,inclination_deg:0}}));
  });
});

describe('target inputs and API agreement',()=>{
  const target={perigee_altitude_m:200000,apogee_altitude_m:null,inclination_deg:null,raan_deg:null,argument_of_periapsis_deg:null,arrival_phase_deg:null,altitude_tolerance_m:1000,angle_tolerance_deg:.1};
  it('keeps partial targets free and accepts LEO, MEO, GTO, GEO and retrograde shapes',()=>{
    expect(targetInputsValid(target)).toBe(true);
    for(const [perigee,apogee,inc] of [[200000,200000,28.5],[20200000,20200000,55],[200000,35786000,98],[35786000,35786000,0],[200000,500000,180]]){
      expect(targetInputsValid({...target,perigee_altitude_m:perigee,apogee_altitude_m:apogee,inclination_deg:inc})).toBe(true);
    }
    expect(targetInputsValid({...target,perigee_altitude_m:null,apogee_altitude_m:35786000})).toBe(true);
  });
  it('blocks invalid tolerances and undefined or out-of-range orbital angles',()=>{
    for(const patch of [{altitude_tolerance_m:0},{angle_tolerance_deg:0},{angle_tolerance_deg:11},{altitude_tolerance_m:Infinity},{perigee_altitude_m:NaN},{raan_deg:360},{inclination_deg:0,raan_deg:30},{apogee_altitude_m:200000,argument_of_periapsis_deg:20}])expect(targetInputsValid({...target,...patch})).toBe(false);
  });
  it('requires an orbit parameter and a valid optional arrival window',()=>{
    expect(targetInputsValid({...target,perigee_altitude_m:null})).toBe(false);
    expect(targetInputsValid({...target,arrival_time_min_s:100,arrival_time_max_s:50})).toBe(false);
    expect(targetInputsValid({...target,arrival_time_max_s:0})).toBe(false);
    expect(targetInputsValid({...target,arrival_time_min_s:Infinity})).toBe(false);
    expect(targetInputsValid({...target,arrival_time_min_s:0,arrival_time_max_s:36000})).toBe(true);
  });
});
