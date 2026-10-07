import {describe,it,expect} from 'vitest';
import map from './data/globe-map.json';
import {globePoint,rotateGlobePoint,globeGeometry,globeEpoch} from './earthGlobe';

describe('geographic globe scale and frames',()=>{
  it('uses WGS84 kilometres, consistent with metre-to-kilometre orbit rendering',()=>{
    expect(globePoint(0,0)[0]).toBeCloseTo(6378137/1000,9);
    expect(globePoint(90,0)[2]).toBeCloseTo(6356.752314245,8);
    expect(globePoint(0,90)[1]).toBeCloseTo(6378.137,9);
    expect(globePoint(0,0,200)[0]-globePoint(0,0)[0]).toBeCloseTo(200,9);
  });
  it('places real land and ocean at their geographic coordinates',()=>{
    const land=(lat:number,lon:number)=>map.landMask[lat+90][lon+180]==='1';
    expect(land(39,33)).toBe(true);
    expect(land(-25,135)).toBe(true);
    expect(land(64,-19)).toBe(true);
    expect(land(0,0)).toBe(false);
    expect(land(20,-40)).toBe(false);
    expect(map.landMask).toHaveLength(181);
    expect(map.landMask.every(row=>row.length===361)).toBe(true);
  });
  it('rotates the surface and geographic outlines together without changing scale',()=>{
    const rotation=[[0,-1,0],[1,0,0],[0,0,1]];
    expect(rotateGlobePoint(globePoint(0,0),rotation)[1]).toBeCloseTo(6378.137,9);
    const base=globeGeometry(),rotated=globeGeometry(rotation);
    expect(rotated.x[90][180]).toBeCloseTo(0,9);
    expect(rotated.y[90][180]).toBeCloseTo(base.x[90][180],9);
    const point=base.coastlines.find(p=>p!==null)!,turned=rotated.coastlines.find(p=>p!==null)!;
    expect(Math.hypot(...turned)).toBeCloseTo(Math.hypot(...point),8);
    expect(turned[0]).toBeCloseTo(-point[1],8);
    expect(rotated.coastlines.filter(p=>p===null).length).toBe(base.coastlines.filter(p=>p===null).length);
  });
  it('shows arrival geography at the saved flight time rather than launch time',()=>{
    expect(globeEpoch('2026-10-06T23:00:00+03:00',7200)).toBe('2026-10-06T22:00:00.000Z');
  });
});
