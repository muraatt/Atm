"""Reproducible SURFACE mission benchmarks; no browser or saved user job changes."""
import argparse
from datetime import datetime, timezone
import json
import hashlib
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from atmosphere.models import default_scenario, StageMassBounds, TargetOrbit, FlightPlan, Scenario
from atmosphere.engines import CATALOG, validate_design
from atmosphere.optimization import optimize
from atmosphere.dynamics import Simulation
from atmosphere.sizing import apply_stage_masses


def benchmark(case,budget=900,iterations=400,mesh=12,refinements=3):
    s=default_scenario();s.name=f'Joint stage sizing surface-to-{case.upper()}'
    s.launch.latitude_deg=0;s.launch.longitude_deg=0;s.launch.launch_site_id=None
    s.launch.elevation_m=0;s.launch.geoid_height_m=0;s.launch.surface='ocean';s.launch.elevation_resolved=True
    s.launch.elevation_source='Benchmark sea surface';s.launch.azimuth_sectors_deg=[]
    s.launch.epoch=datetime(2026,10,5,tzinfo=timezone.utc)
    s.environment.model='standard';s.vehicle.optimize_stage_masses=True
    s.vehicle.payload_mass_kg=3000;s.vehicle.minimum_initial_twr=1.1
    booster,upper=s.vehicle.stages
    booster.engine_id='merlin-sl';booster.engine_count=9;booster.vacuum_thrust_n=9*950000
    booster.isp_vacuum_s=311;booster.isp_sea_level_s=311*845000/950000;booster.throttle_min=.57
    booster.dry_mass_kg=22000;booster.propellant_mass_kg=350000
    booster.mass_bounds=StageMassBounds(dry_mass_min_kg=10000,dry_mass_max_kg=60000,
        propellant_min_kg=80000,propellant_max_kg=650000,hardware_mass_kg=9000,tank_structure_ratio=.035)
    upper.engine_id='be-3u';upper.engine_count=2;upper.vacuum_thrust_n=1780000
    upper.isp_vacuum_s=450;upper.isp_sea_level_s=200;upper.throttle_min=.5
    upper.dry_mass_kg=8000;upper.propellant_mass_kg=90000;upper.ignition_limit=3
    upper.mass_bounds=StageMassBounds(dry_mass_min_kg=2500,dry_mass_max_kg=30000,
        propellant_min_kg=5000,propellant_max_kg=200000,hardware_mass_kg=2000,tank_structure_ratio=.035)
    for stage in s.vehicle.stages:
        stage.engine_catalog_version=CATALOG['version'];stage.space_thrust_velocity_angle_deg=180
    alt={'leo':200000,'meo':20200000,'geo':35786000,'gto':35786000}[case]
    s.target=TargetOrbit(perigee_altitude_m=200000 if case=='gto' else alt,apogee_altitude_m=alt,
        inclination_deg=0,altitude_tolerance_m=1000,angle_tolerance_deg=.1)
    s.constraints.solver_budget_s=budget;s.constraints.max_iterations=iterations
    s.constraints.control_nodes=9;s.constraints.mesh_segments=mesh;s.constraints.refinement_levels=refinements
    s.constraints.max_topologies=1 if case in ('leo','gto') else 3
    validate_design(s,require_engines=True)
    return Scenario.model_validate(s.model_dump(mode='json'))


def run(case,args):
    destination=ROOT/'data'/'stage-sizing'/args.label/case;destination.mkdir(parents=True,exist_ok=True)
    s=benchmark(case,args.budget,args.iterations,args.mesh,args.refinements)
    sources=sorted((ROOT/'backend'/'atmosphere').glob('*.py'))+[ROOT/'shared'/'engines.json']
    implementation={str(path.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (destination/'implementation.json').write_text(json.dumps(implementation,indent=2),encoding='utf-8')
    (destination/'input.json').write_text(s.model_dump_json(indent=2),encoding='utf-8')
    last=0;started=time.monotonic()
    def progress(value):
        nonlocal last
        if time.monotonic()-last>20:
            print(json.dumps({'case':case,'wall_s':round(time.monotonic()-started,1),**value}),flush=True);last=time.monotonic()
        (destination/'progress.json').write_text(json.dumps(value,allow_nan=False),encoding='utf-8')
    result=optimize(s,progress=progress)
    (destination/'result.json').write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
    plan=FlightPlan.model_validate(result['flight_plan'])
    replay=Simulation(s).run(plan,strict=True)
    (destination/'independent-replay.json').write_text(json.dumps(replay,allow_nan=False),encoding='utf-8')
    final_implementation={str(path.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (destination/'implementation-final.json').write_text(json.dumps(final_implementation,indent=2),encoding='utf-8')
    checks={'optimizer_changed_stage_masses':plan.stage_masses is not None and any(abs(a.dry_mass_kg-b.dry_mass_kg)>1 or abs(a.propellant_mass_kg-b.propellant_mass_kg)>1 for a,b in zip(plan.stage_masses,s.vehicle.stages)),
        'actual_surface_start':abs(replay['series'][0]['altitude_m'])<.1,
        'joint_design_saved':bool(result.get('stage_sizing')),
        'target_achieved':result['status']=='Target Achieved' and replay['status']=='Target Achieved',
        'all_orbit_tolerances':all(r['within_tolerance'] for r in replay['target_comparison']),
        'all_flight_constraints':all(replay['constraint_checks'].values()),
        'strict_replay':replay['metadata']['verified_with_strict_integration'],
        'fuel_closure':all(abs(r['initial_kg']-r['burned_kg']-r['remaining_kg']-r['discarded_kg'])<.01 for r in replay['propellant_ledger']),
        'input_preserved':result.get('input_scenario')==s.model_dump(mode='json')}
    if plan.stage_masses is not None:
        designed=apply_stage_masses(s,plan.stage_masses)
        checks['bounds_verified']=bool(replay['stage_sizing']['bounds_verified'])
        checks['payload_and_engines_fixed']=designed.vehicle.payload_mass_kg==s.vehicle.payload_mass_kg and all(a.engine_id==b.engine_id and a.engine_count==b.engine_count and a.vacuum_thrust_n==b.vacuum_thrust_n and a.isp_vacuum_s==b.isp_vacuum_s for a,b in zip(designed.vehicle.stages,s.vehicle.stages))
    summary={'case':case,'passed':all(checks.values()),'checks':checks,'status':replay['status'],
        'wall_s':time.monotonic()-started,'target_comparison':replay['target_comparison'],
        'continuous_restoration':result['optimizer'].get('mission',{}).get('continuous_restoration',{}),
        'source_files_changed_during_run':[name for name,digest in implementation.items() if final_implementation.get(name)!=digest],
        'stage_sizing':replay.get('stage_sizing'),
        'assumptions':'Published engine thrust references; explicitly assumed missing Isp/vacuum booster thrust and tank/hardware mass bounds. Preliminary two-stage design, not a manufacturer vehicle performance claim.'}
    (destination/'audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(summary,allow_nan=False),flush=True)
    return summary['passed']


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['leo','meo','geo','gto','all'],default='all')
    parser.add_argument('--budget',type=float,default=900);parser.add_argument('--iterations',type=int,default=400)
    parser.add_argument('--mesh',type=int,default=12);parser.add_argument('--refinements',type=int,default=3)
    parser.add_argument('--label',default='initial')
    args=parser.parse_args()
    outcomes=[run(case,args) for case in (['leo','meo','gto','geo'] if args.case=='all' else [args.case])]
    sys.exit(0 if all(outcomes) else 1)
