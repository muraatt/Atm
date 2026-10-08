import {describe,it,expect} from 'vitest';
import fixtures from './data/screening-reference.json';
import {azimuthMargin,screenLaunchSites} from './screening';
import type {Target,LaunchSite} from './types';

describe('browser launch screening',()=>{
  it('agrees with the local engine for free, equatorial, prograde and retrograde planes',()=>{
    for(const fixture of fixtures){
      const result=screenLaunchSites(fixture.target as Target,fixture.launch as LaunchSite);
      expect(result.sites.map(s=>s.id)).toEqual(fixture.sites.map(s=>s.id));
      for(const site of result.sites){
        const reference=fixture.sites.find(s=>s.id===site.id)!;
        expect(site.minimum_plane_change_deg).toBeCloseTo(reference.minimum_plane_change_deg,7);
        // North/south mirror headings can tie; both provide the same plane and rotation benefit.
        expect(Math.sin(site.recommended_azimuth_deg*Math.PI/180)).toBeCloseTo(Math.sin(reference.recommended_azimuth_deg*Math.PI/180),7);
        expect(site.rotation_benefit_m_s).toBeCloseTo(reference.rotation_benefit_m_s,7);
        expect(site.status).toBe(reference.status);
      }
    }
  });
  it('handles wrapped sectors and user overrides without fabricating timing windows',()=>{
    expect(azimuthMargin(355,[[350,10]])).toBe(5);
    expect(azimuthMargin(180,[[350,10]])).toBeLessThan(0);
    expect(azimuthMargin(180,[[0,360]])).toBe(180);
    const f=fixtures[2],id=f.sites[0].id;
    const r=screenLaunchSites({...f.target,raan_deg:45} as Target,{...f.launch,launch_site_id:id,azimuth_sectors_deg:[[0,1]],range_source:'User-defined sectors'} as LaunchSite);
    const site=r.sites.find(s=>s.id===id)!;
    expect(site.recommended_azimuth_deg).toBeGreaterThanOrEqual(0);
    expect(site.recommended_azimuth_deg).toBeLessThanOrEqual(1);
    expect(site.launch_windows_utc).toEqual([]);
    expect(r.window_note).toContain('RAAN feasibility is not established');
  });
});
