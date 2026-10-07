"""Varied full surface-to-orbit regressions, isolated from application jobs.

All positive cases retain 1 km / 0.1 degree target tolerances. A solver miss is
reported separately from bookkeeping correctness; neither counts as success.
Engine parameters not published in the catalog are explicit study assumptions.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from verify_stage_sizing import benchmark, ROOT
from atmosphere.dynamics import Simulation
from atmosphere.engines import validate_design
from atmosphere.models import Scenario, TargetOrbit, FlightPlan
from atmosphere.optimization import optimize

CASES={
    'leo-cape':dict(family='leo',perigee=300e3,apogee=300e3,inc=28.6,site='cape'),
    'leo-polar':dict(family='leo',perigee=550e3,apogee=550e3,inc=97.4,site='vandenberg'),
    'leo-elliptic':dict(family='leo',perigee=200e3,apogee=800e3,inc=None,lat=10,lon=80,elevation=-400,geoid=80,payload=1500),
    'meo-fast':dict(family='meo',perigee=2000e3,apogee=2000e3,inc=0,objective='time',atmosphere='msis',payload=6000),
    'meo-gnss':dict(family='meo',perigee=20200e3,apogee=20200e3,inc=55,site='baikonur',payload=2000),
    'meo-be4':dict(family='meo',perigee=10000e3,apogee=10000e3,inc=30,site='cape',booster='be4',payload=1500),
    'gto-cape':dict(family='gto',perigee=200e3,apogee=35786e3,inc=28.6,site='cape',sizing=False),
    'gto-retrograde':dict(family='gto',perigee=200e3,apogee=40000e3,inc=120,payload=1500),
    'gto-partial':dict(family='gto',perigee=None,apogee=35786e3,inc=None,lat=10,lon=80,elevation=1500,geoid=80,upper='merlin'),
    'geo-msis':dict(family='geo',perigee=35786e3,apogee=35786e3,inc=0,lon=100,atmosphere='msis',payload=1500,epoch='2026-04-20T12:00:00+00:00'),
    'geo-inclined':dict(family='geo',perigee=35786e3,apogee=35786e3,inc=10,lon=140,payload=1500),
    'geo-plane-change':dict(family='geo',perigee=35786e3,apogee=35786e3,inc=0,site='cape',payload=1500),
}


def scenario_for(name,budget=900):
    case=CASES[name]
    s=benchmark(case['family'],budget=budget,iterations=500,mesh=24 if case['family']=='geo' else 12,refinements=3)
    s.name=f'System validation: {name}'
    s.target=TargetOrbit(perigee_altitude_m=case['perigee'],apogee_altitude_m=case['apogee'],inclination_deg=case['inc'])
    s.objective=case.get('objective','propellant')
    s.vehicle.payload_mass_kg=case.get('payload',3000)
    s.vehicle.optimize_stage_masses=case.get('sizing',True)
    s.launch.latitude_deg=case.get('lat',0);s.launch.longitude_deg=case.get('lon',0)
    s.launch.elevation_m=case.get('elevation',0);s.launch.geoid_height_m=case.get('geoid',0)
    s.launch.surface='land' if s.launch.elevation_m else 'ocean'
    s.launch.elevation_source='Explicit synthetic test pad elevation and geoid; not a measured terrain claim'
    if case.get('site'):
        sites=json.loads((ROOT/'backend/atmosphere/data/launch_sites.json').read_text(encoding='utf-8'))
        if isinstance(sites,dict):sites=sites['sites']
        identity={'cape':'ll2-87','vandenberg':'ll2-16','baikonur':'ll2-20'}[case['site']]
        site=next(site for site in sites if site['id']==identity)
        s.launch.launch_site_id=identity
        s.launch.latitude_deg=site['latitude_deg'];s.launch.longitude_deg=site['longitude_deg']
        s.launch.azimuth_sectors_deg=[tuple(pair) for pair in site.get('azimuth_sectors_deg',[])]
        s.launch.range_source=site.get('range_reference','No numeric departure sector supplied')
        s.launch.surface='land'
    s.launch.epoch=datetime.fromisoformat(case.get('epoch','2026-10-05T00:00:00+00:00'))
    s.environment.model=case.get('atmosphere','standard');s.environment.solar_mode='manual'
    s.environment.f107=150;s.environment.f107_average=150;s.environment.ap=4
    s.constraints.max_topologies=3 if case['family'] in ('meo','geo') else 1
    if case.get('booster')=='be4':
        stage=s.vehicle.stages[0]
        stage.engine_id='be-4';stage.engine_count=2
        stage.vacuum_thrust_n=2*3100000;stage.isp_vacuum_s=340
        stage.isp_sea_level_s=340*2846000/3100000;stage.throttle_min=.3436401968
        stage.dry_mass_kg=14000;stage.propellant_mass_kg=220000
    if case.get('upper')=='merlin':
        stage=s.vehicle.stages[1]
        stage.engine_id='merlin-vac';stage.engine_count=1;stage.vacuum_thrust_n=981000
        stage.isp_vacuum_s=348;stage.isp_sea_level_s=200;stage.throttle_min=.638
    validate_design(s,require_engines=True)
    return Scenario.model_validate(s.model_dump(mode='json'))


def sources():
    paths=sorted((ROOT/'backend/atmosphere').glob('*.py'))+[ROOT/'shared/engines.json']
    return {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


LOADED_SOURCE_HASHES=sources()


def non_json_paths(value,path='result'):
    if isinstance(value,dict):return sum((non_json_paths(v,path+'.'+k) for k,v in value.items()),[])
    if isinstance(value,(tuple,list)):return sum((non_json_paths(v,f'{path}[{i}]') for i,v in enumerate(value)),[])
    try:json.dumps(value,allow_nan=False)
    except (TypeError,ValueError):return [f'{path}: {type(value).__name__}']
    return []


def audit(s,result,replay,before,after):
    rows=replay['series'];ledger=replay['propellant_ledger'];designed=Scenario.model_validate(replay['scenario'])
    actual_fields={r['parameter'] for r in replay['target_comparison']}
    fields=('perigee_altitude_m','apogee_altitude_m','inclination_deg','raan_deg','argument_of_periapsis_deg','arrival_phase_deg')
    expected_fields={f for f in fields if getattr(s.target,f) is not None}
    first=rows[0]
    fixed_fields=set(type(s.vehicle.stages[0]).model_fields)-{'dry_mass_kg','propellant_mass_kg','mass_bounds'}
    hardware_fixed=all(all(getattr(a,f)==getattr(b,f) for f in fixed_fields) for a,b in zip(s.vehicle.stages,designed.vehicle.stages))
    separated_dry=sum(e.get('discarded_dry_mass_kg',0) for e in replay['events'])
    closure=designed.vehicle.initial_mass_kg-replay['summary']['final_mass_kg']-separated_dry-sum(l['burned_kg']+l['discarded_kg'] for l in ledger)
    events=replay['events']
    ignitions={stage.name:sum(e['event']=='Ignition' and e['stage']==stage.name for e in events) for stage in designed.vehicle.stages}
    checks={
        'source_unchanged':before==after,
        'target_achieved':result['status']=='Target Achieved' and replay['status']=='Target Achieved',
        'strict_independent_replay':replay['metadata']['verified_with_strict_integration'],
        'all_requested_tolerances':all(r['within_tolerance'] for r in replay['target_comparison']),
        'only_requested_orbit_parameters':actual_fields==expected_fields,
        'all_flight_constraints':all(replay['constraint_checks'].values()),
        'surface_height_with_geoid':abs(first['altitude_m']-s.launch.elevation_m-s.launch.geoid_height_m)<.1,
        'surface_latitude':abs(first['latitude_deg']-s.launch.latitude_deg)<1e-5,
        'surface_longitude':abs((first['longitude_deg']-s.launch.longitude_deg+180)%360-180)<1e-5,
        'initial_mass':abs(first['mass_kg']-designed.vehicle.initial_mass_kg)<.01,
        'total_mass_closure':abs(closure)<.1,
        'per_stage_fuel_closure':all(abs(l['initial_kg']-l['burned_kg']-l['remaining_kg']-l['discarded_kg'])<.01 and min(l['burned_kg'],l['remaining_kg'],l['discarded_kg'])>=-.01 for l in ledger),
        'fuel_never_negative':all(r['active_propellant_kg']>=-.01 for r in rows),
        'event_times_ordered':all(a['time_s']<=b['time_s']+1e-6 for a,b in zip(events,events[1:])),
        'ignition_limits':all(ignitions[stage.name]<=stage.ignition_limit for stage in designed.vehicle.stages),
        'payload_and_engine_configuration_fixed':hardware_fixed and s.vehicle.payload_mass_kg==designed.vehicle.payload_mass_kg,
        'input_preserved':result.get('input_scenario',result['scenario'])==s.model_dump(mode='json'),
        'sizing_bounds_or_fixed_masses':bool(replay.get('stage_sizing',{}).get('bounds_verified')) if s.vehicle.optimize_stage_masses else all(a.dry_mass_kg==b.dry_mass_kg and a.propellant_mass_kg==b.propellant_mass_kg for a,b in zip(s.vehicle.stages,designed.vehicle.stages)),
        'max_q_matches_series':abs(replay['summary']['max_dynamic_pressure_pa']-max(r['dynamic_pressure_pa'] for r in rows))<1e-5,
        'vacuum_temperature_not_invented':all(r['temperature_k'] is None for r in rows if r['altitude_m']>1e6),
    }
    return {'passed':all(checks.values()),'checks':checks,'status':replay['status'],
            'target_comparison':replay['target_comparison'],'constraint_checks':replay['constraint_checks'],
            'mass_closure_error_kg':closure,'summary':replay['summary'],
            'source_files_changed_during_run':[k for k,v in before.items() if after.get(k)!=v]}


def run(name,label,budget):
    dest=ROOT/'data/system-validation'/label/name;dest.mkdir(parents=True,exist_ok=True)
    started=time.monotonic();before=sources();s=scenario_for(name,budget);result=None
    (dest/'input.json').write_text(s.model_dump_json(indent=2),encoding='utf-8')
    (dest/'implementation.json').write_text(json.dumps(before,indent=2),encoding='utf-8')
    def progress(value):
        (dest/'progress.json').write_text(json.dumps({'wall_s':time.monotonic()-started,**value},allow_nan=False),encoding='utf-8')
    try:
        if before!=LOADED_SOURCE_HASHES:raise RuntimeError('Source changed after this worker loaded. Start a fresh validation process.')
        result=optimize(s,progress=progress,check_cancel=lambda:(dest.parent/'STOP').exists())
        (dest/'flight-plan.json').write_text(FlightPlan.model_validate(result['flight_plan']).model_dump_json(indent=2),encoding='utf-8')
        (dest/'result.json').write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
        replay=Simulation(s).run(FlightPlan.model_validate(result['flight_plan']),strict=True)
        (dest/'independent-replay.json').write_text(json.dumps(replay,allow_nan=False),encoding='utf-8')
        outcome=audit(s,result,replay,before,sources())
    except Exception as exc:
        (dest/'error-trace.txt').write_text(traceback.format_exc(),encoding='utf-8')
        outcome={'passed':False,'status':'Harness or analysis failure','error':f'{type(exc).__name__}: {exc}',
                 'non_json_paths':non_json_paths(result) if result is not None else []}
    outcome.update(case=name,wall_s=time.monotonic()-started)
    (dest/'audit.json').write_text(json.dumps(outcome,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'case':name,'passed':outcome['passed'],'status':outcome['status'],'wall_s':round(outcome['wall_s'],1),'error':outcome.get('error')}),flush=True)
    return outcome


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--case',nargs='+',choices=list(CASES)+['all'],default=['all'])
    parser.add_argument('--budget',type=float,default=900);parser.add_argument('--label',default='varied-initial')
    parser.add_argument('--workers',type=int,choices=(1,2),default=1)
    args=parser.parse_args();names=list(CASES) if args.case==['all'] else args.case
    if args.workers==1:results=[run(name,args.label,args.budget) for name in names]
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            pending=[pool.submit(run,name,args.label,args.budget) for name in names]
            results=[f.result() for f in as_completed(pending)]
    directory=ROOT/'data/system-validation'/args.label;directory.mkdir(parents=True,exist_ok=True)
    (directory/'matrix.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    sys.exit(0 if all(r['passed'] for r in results) else 1)
