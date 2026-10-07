import {useEffect,useRef,useState} from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import type {LaunchSite,SeriesRow} from './types';
import {groundTrackParts} from './utils';
import {launchSites} from './launchSites';

export function LaunchMap({launch,onPick,series,showSites=false,selectedSiteId,onSelectSite,allowedSiteIds}:{launch:LaunchSite;onPick?:(lat:number,lon:number)=>void;series?:SeriesRow[];showSites?:boolean;selectedSiteId?:string;onSelectSite?:(id:string)=>void;allowedSiteIds?:string[]}){
  const ref=useRef<HTMLDivElement>(null),mapRef=useRef<maplibregl.Map|null>(null),markerRef=useRef<maplibregl.Marker|null>(null),pickRef=useRef(onPick);const [error,setError]=useState('');pickRef.current=onPick;
  const siteMarkers=useRef<Map<string,HTMLButtonElement>>(new Map()),selectRef=useRef(onSelectSite),previousLocation=useRef([launch.latitude_deg,launch.longitude_deg]);selectRef.current=onSelectSite;
  const worldView=()=>mapRef.current?.fitBounds([[-175,-50],[179,73]],{padding:32,maxZoom:1.4,duration:600});
  useEffect(()=>{if(!ref.current)return;let map:maplibregl.Map;
    try{map=new maplibregl.Map({container:ref.current,center:showSites?[10,15]:[launch.longitude_deg,launch.latitude_deg],zoom:showSites?0.8:series?1.2:3.3,renderWorldCopies:!showSites,style:'https://tiles.openfreemap.org/styles/dark'});
      mapRef.current=map;map.addControl(new maplibregl.NavigationControl({showCompass:false}),'top-right');
      const el=document.createElement('div');el.className='launch-marker';el.style.pointerEvents='none';markerRef.current=new maplibregl.Marker({element:el}).setLngLat([launch.longitude_deg,launch.latitude_deg]).addTo(map);
      if(showSites)for(const site of launchSites){
        const button=document.createElement('button');button.type='button';button.className='known-site-marker';
        button.setAttribute('aria-label',`Select ${site.name} — ${site.pad}`);button.title=`${site.name} — ${site.pad} (${site.country})`;
        const label=document.createElement('span');label.className='known-site-label';label.textContent=site.name;button.append(label);
        button.addEventListener('click',event=>{event.stopPropagation();selectRef.current?.(site.id);});
        button.classList.toggle('selected',site.id===selectedSiteId);button.setAttribute('aria-pressed',String(site.id===selectedSiteId));
        siteMarkers.current.set(site.id,button);new maplibregl.Marker({element:button}).setLngLat([site.longitude_deg,site.latitude_deg]).addTo(map);
      }
      map.on('click',e=>pickRef.current?.(e.lngLat.lat,((e.lngLat.lng+180)%360+360)%360-180));
      map.on('error',()=>setError('Map tiles could not load. Use the launch-site list or manual coordinates.'));
      map.on('load',()=>{if(showSites)worldView();map.addSource('track',{type:'geojson',data:{type:'Feature',properties:{},geometry:{type:'MultiLineString',coordinates:[]}}});map.addLayer({id:'track',type:'line',source:'track',paint:{'line-color':'#59e5c1','line-width':3}});if(series)(map.getSource('track') as maplibregl.GeoJSONSource).setData({type:'Feature',properties:{},geometry:{type:'MultiLineString',coordinates:groundTrackParts(series)}});});
    }catch{setError('Interactive map requires WebGL. Use the launch-site list or manual coordinates.');}
    return()=>{mapRef.current=null;siteMarkers.current.clear();map?.remove();};
  },[]);
  useEffect(()=>{const map=mapRef.current;if(!map)return;markerRef.current?.setLngLat([launch.longitude_deg,launch.latitude_deg]);const changed=previousLocation.current[0]!==launch.latitude_deg||previousLocation.current[1]!==launch.longitude_deg;if(!series&&changed)map.easeTo({center:[launch.longitude_deg,launch.latitude_deg],zoom:showSites?4.2:map.getZoom(),duration:600});previousLocation.current=[launch.latitude_deg,launch.longitude_deg];},[launch.latitude_deg,launch.longitude_deg]);
  useEffect(()=>{for(const [id,button] of siteMarkers.current){button.classList.toggle('selected',id===selectedSiteId);button.setAttribute('aria-pressed',String(id===selectedSiteId));}},[selectedSiteId]);
  useEffect(()=>{for(const [id,button] of siteMarkers.current){const excluded=allowedSiteIds!==undefined&&!allowedSiteIds.includes(id);button.disabled=false;button.classList.toggle('filtered-out',excluded);const site=launchSites.find(site=>site.id===id);if(site)button.title=`${site.name} — ${site.pad} (${site.country})${excluded?' · Outside recommendations; select to evaluate':''}`;}},[allowedSiteIds?.join(',')]);
  useEffect(()=>{const map=mapRef.current;if(!map||!map.getSource('track')||!series)return;(map.getSource('track') as maplibregl.GeoJSONSource).setData({type:'Feature',properties:{},geometry:{type:'MultiLineString',coordinates:groundTrackParts(series)}});},[series]);
  return <div className="map-shell"><div className="map-canvas" ref={ref}/>{error&&<div className="map-message">{error}</div>}{showSites&&<button className="map-world-button" onClick={worldView}>World view · {launchSites.length} sites</button>}<div className="map-caption"><span className="live-dot"/>{series?'Earth-fixed ground track':onPick?'Click the map or a marked launch site':'Select a marked launch site'}</div></div>;
}
