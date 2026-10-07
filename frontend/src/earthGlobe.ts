import map from './data/globe-map.json';

export type Point3 = [number,number,number];
export type EarthRotation = number[][];
export const identityRotation:EarthRotation=[[1,0,0],[0,1,0],[0,0,1]];
const a=6378.137,f=1/298.257223563,e2=f*(2-f);

export function globePoint(latitude:number,longitude:number,heightKm=0):Point3 {
  const lat=latitude*Math.PI/180,lon=longitude*Math.PI/180,n=a/Math.sqrt(1-e2*Math.sin(lat)**2);
  return [(n+heightKm)*Math.cos(lat)*Math.cos(lon),(n+heightKm)*Math.cos(lat)*Math.sin(lon),(n*(1-e2)+heightKm)*Math.sin(lat)];
}
export function rotateGlobePoint(point:Point3,matrix:EarthRotation):Point3 {
  return matrix.map(row=>row.reduce((sum,value,i)=>sum+value*point[i],0)) as Point3;
}
export function globeEpoch(launchEpoch:string,arrivalSeconds=0):string {
  return new Date(new Date(launchEpoch).getTime()+arrivalSeconds*1000).toISOString();
}

const surfacePoints=map.landMask.map((row,j)=>Array.from(row,(_,k)=>globePoint(-90+j*map.latitudeStepDeg,-180+k*map.longitudeStepDeg)));
const surfacecolor=map.landMask.map(row=>Array.from(row,Number));
function geographyPaths(lines:number[][][]):(Point3|null)[] {
  const points:(Point3|null)[]=[];
  for(const line of lines){
    for(let i=0;i<line.length;i++){
      const [lon,lat]=line[i];
      if(i===0){points.push(globePoint(lat,lon,.025));continue;}
      const [previousLon,previousLat]=line[i-1];
      const deltaLon=((lon-previousLon+540)%360)-180,deltaLat=lat-previousLat;
      const steps=Math.max(1,Math.ceil(Math.max(Math.abs(deltaLon),Math.abs(deltaLat))/.5));
      for(let k=1;k<=steps;k++)points.push(globePoint(previousLat+deltaLat*k/steps,previousLon+deltaLon*k/steps,.025));
    }
    points.push(null);
  }
  return points;
}
const coastlines=geographyPaths(map.coastlines),borders=geographyPaths(map.borders);
const grid=geographyPaths([
  ...[-60,-30,0,30,60].map(lat=>Array.from({length:361},(_,i)=>[-180+i,lat])),
  ...[-180,-120,-60,0,60,120].map(lon=>Array.from({length:181},(_,i)=>[lon,-90+i])),
]);
export function globeGeometry(rotation:EarthRotation=identityRotation){
  const points=surfacePoints.map(row=>row.map(point=>rotateGlobePoint(point,rotation)));
  const transform=(path:(Point3|null)[])=>path.map(point=>point===null?null:rotateGlobePoint(point,rotation));
  return {x:points.map(row=>row.map(p=>p[0])),y:points.map(row=>row.map(p=>p[1])),z:points.map(row=>row.map(p=>p[2])),surfacecolor,
          coastlines:transform(coastlines),borders:transform(borders),grid:transform(grid)};
}
