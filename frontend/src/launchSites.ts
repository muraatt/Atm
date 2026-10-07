import catalog from '../../backend/atmosphere/data/launch_sites.json';
import type {LaunchSite} from './types';

export const launchSites=catalog;
export type KnownLaunchSite=typeof launchSites[number];
export const defaultLaunchSite=launchSites[0];
export const findLaunchSite=(id:string|null|undefined)=>launchSites.find(site=>site.id===id);
export function catalogLaunch(launch:LaunchSite,site:KnownLaunchSite):LaunchSite {
  return {...launch,launch_site_id:site.id,latitude_deg:site.latitude_deg,longitude_deg:site.longitude_deg,
    surface:'land',azimuth_sectors_deg:site.azimuth_sectors_deg as [number,number][],range_source:site.range_reference,elevation_resolved:false,elevation_source:'Pending ETOPO lookup',elevation_resolution_arcsec:null};
}
export function identifyLaunchSite(launch:LaunchSite):KnownLaunchSite|undefined {
  return launchSites.find(site=>Math.abs(site.latitude_deg-launch.latitude_deg)<1e-5&&Math.abs(site.longitude_deg-launch.longitude_deg)<1e-5);
}
