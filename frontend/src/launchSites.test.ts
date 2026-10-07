import {describe,it,expect} from 'vitest';
import {catalogLaunch,defaultLaunchSite,findLaunchSite,identifyLaunchSite,launchSites} from './launchSites';
import type {LaunchSite} from './types';

describe('launch-site selection',()=>{
  const manual:LaunchSite={latitude_deg:0,longitude_deg:0,elevation_m:123,geoid_height_m:0,elevation_source:'Manual elevation',elevation_resolution_arcsec:null,elevation_resolved:true,surface:'auto',epoch:'2026-10-04T00:00:00Z',launch_site_id:null};
  it('keeps unique worldwide pad coordinates',()=>{
    expect(new Set(launchSites.map(site=>site.id)).size).toBe(launchSites.length);
    expect(launchSites.every(site=>Math.abs(site.latitude_deg)<=90&&Math.abs(site.longitude_deg)<=180&&site.source_url.startsWith('https://'))).toBe(true);
    expect(launchSites.some(site=>site.latitude_deg<0)).toBe(true);
  });
  it('invalidates stale elevation while retaining the launch time',()=>{
    const selected=catalogLaunch(manual,defaultLaunchSite);
    expect(selected.launch_site_id).toBe(defaultLaunchSite.id);
    expect(selected.latitude_deg).toBe(defaultLaunchSite.latitude_deg);
    expect(selected.elevation_resolved).toBe(false);
    expect(selected.surface).toBe('land');
    expect(selected.epoch).toBe(manual.epoch);
    expect(identifyLaunchSite(selected)).toBe(defaultLaunchSite);
    expect(findLaunchSite(null)).toBeUndefined();
    expect(identifyLaunchSite(manual)).toBeUndefined();
  });
});
