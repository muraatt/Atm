"""Cheap performance screening and explanations; never a feasibility certificate."""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import brentq

from .earth import FLATTENING, G0, MU, OMEGA, R_E, geodetic_to_ecef, gravity
from .environment import StandardAtmosphere
from .models import Scenario
from .screening import departure_options


def ideal_capacity(scenario: Scenario, payload: float | None = None):
    stages = scenario.vehicle.stages
    mass = (scenario.vehicle.payload_mass_kg if payload is None else payload) + sum(s.dry_mass_kg+s.propellant_mass_kg for s in stages)
    rows = []
    for stage in stages:
        burnout = mass-stage.propellant_mass_kg
        rows.append({"stage": stage.name, "ignition_mass_kg": mass,
                     "burnout_mass_kg": burnout,
                     "ideal_delta_v_m_s": G0*stage.isp_vacuum_s*math.log1p(stage.propellant_mass_kg/burnout),
                     "vacuum_initial_twr": stage.vacuum_thrust_n*(1 if stage.engine_type=='solid' else stage.throttle_max)/(mass*G0),
                     "full_throttle_burn_s": stage.full_burn_s/(1 if stage.engine_type=='solid' else stage.throttle_max)})
        mass = burnout-stage.dry_mass_kg
    return rows


def analyze_consistency(scenario: Scenario, result: dict | None = None):
    from .budget import mission_budget
    rows = ideal_capacity(scenario)
    capacity = sum(r['ideal_delta_v_m_s'] for r in rows)
    launch = scenario.launch
    position = geodetic_to_ecef(launch.latitude_deg,launch.longitude_deg,launch.elevation_m+launch.geoid_height_m)
    velocity = np.cross([0,0,OMEGA],position)
    radius = float(np.linalg.norm(position))
    speed = float(np.linalg.norm(velocity))
    initial_energy = speed**2/2-MU/radius
    target = scenario.target
    # Choose the least energetic orbit permitted by the supplied apses and the
    # simulator's 100 km perigee floor. Free apses are not filled as constraints.
    rp = R_E+max(100000,(target.perigee_altitude_m-target.altitude_tolerance_m) if target.perigee_altitude_m is not None else 100000)
    ra = max(rp,R_E+target.apogee_altitude_m-target.altitude_tolerance_m) if target.apogee_altitude_m is not None else rp
    target_energy = -MU/(rp+ra)
    # Central-field screening bound: w=sqrt(2*(E+mu/r_min)), |v|<=w and
    # dE/dt=v.a_thrust imply dw/dt<=|a_thrust|. Use a deliberately low global
    # surface radius (polar WGS84 minus the allowed 500 m terrain / 150 m geoid).
    # This ignores J2, atmospheric energy exchange, drag and steering losses.
    minimum_radius = min(radius,R_E*(1-FLATTENING)-650)
    equivalent = lambda energy: math.sqrt(max(0,2*(energy+MU/minimum_radius)))
    required = max(0,equivalent(target_energy)-equivalent(initial_energy))
    margin = capacity-required
    stage = scenario.vehicle.stages[0]
    pressure = StandardAtmosphere().sample(launch.latitude_deg,launch.longitude_deg,launch.elevation_m+launch.geoid_height_m,0).pressure_pa
    isp = stage.isp_vacuum_s-(stage.isp_vacuum_s-stage.isp_sea_level_s)*np.clip(pressure/101325,0,1)
    shape = max((y for x,y in stage.solid_burn_curve),default=1) if stage.engine_type=='solid' else stage.throttle_max
    initial_shape = stage.solid_burn_curve[0][1] if stage.engine_type=='solid' and stage.solid_burn_curve else (1 if stage.engine_type=='solid' else stage.throttle_max)
    surface_acc = np.cross([0,0,OMEGA],velocity)
    weight = float(np.dot(surface_acc-gravity(position),position/radius))*scenario.vehicle.initial_mass_kg
    maximum_twr = float(stage.vacuum_thrust_n*isp/stage.isp_vacuum_s*shape/weight)
    initial_twr = float(stage.vacuum_thrust_n*isp/stage.isp_vacuum_s*initial_shape/weight)
    findings = []
    def finding(code,severity,title,message):
        findings.append({"code":code,"severity":severity,"title":title,"message":message})
    if margin < 0:
        finding('delta_v_shortfall','warning','Insufficient ideal delta-v',
                f'Even full fuel use with vacuum Isp provides {capacity/1000:.2f} km/s, below the optimistic central-field screening requirement of {required/1000:.2f} km/s. Reduce payload or revise stage performance. A longer solver budget does not increase vehicle capacity.')
    else:
        finding('delta_v_margin','info','Ideal delta-v screen passed',
                f'The optimistic margin is {margin/1000:.2f} km/s before drag, steering and finite-burn effects. This does not establish target reachability.')
    if maximum_twr <= 1:
        finding('liftoff','warning','Liftoff concern','Maximum available thrust does not exceed the estimated local weight at the initial mass. Actual atmosphere and motor profile must be checked by the flight simulator.')
    elif initial_twr < 1.2:
        finding('low_twr','warning','Low initial thrust-to-weight margin','Initial T/W is below the screening reference of 1.2. This is not a hard flight limit; slow ascent can produce substantial gravity losses. Solid motors may burn on the pad before liftoff.')
    plane_error,_,_ = departure_options(launch.latitude_deg,target,launch.azimuth_sectors_deg)
    if plane_error > target.angle_tolerance_deg:
        finding('plane_change','warning','Target plane needs a maneuver',f'Direct departure geometry misses the requested inclination by about {plane_error:.2f} degrees. The energy screen excludes the maneuver cost.')
    if not launch.azimuth_sectors_deg:
        finding('range_unknown','info','Departure sector not constrained','No numeric azimuth envelope is supplied. This screen does not verify range access.')
    if sum(r['full_throttle_burn_s'] for r in rows)>scenario.constraints.max_mission_duration_s:
        finding('duration','warning','Full fuel use exceeds the mission duration','The sum of full-throttle burn times exceeds the mission duration, even before coast and separation delays.')
    ceiling = None
    if required>0 and sum(r['ideal_delta_v_m_s'] for r in ideal_capacity(scenario,1))>=required:
        upper=max(1,scenario.vehicle.initial_mass_kg)
        while sum(r['ideal_delta_v_m_s'] for r in ideal_capacity(scenario,upper))>required: upper*=2
        ceiling=float(brentq(lambda p:sum(r['ideal_delta_v_m_s'] for r in ideal_capacity(scenario,p))-required,1,upper))
    diagnostic = None
    if result is not None:
        series=result.get('series',[])
        last=series[-1] if series else None
        radial=None if last is None else float(np.dot(last['position_gcrs_m'],last['velocity_gcrs_m_s'])/np.linalg.norm(last['position_gcrs_m']))
        ledger=result.get('propellant_ledger',[])
        ledger_error=sum(r['initial_kg']-r['burned_kg']-r['remaining_kg']-r['discarded_kg'] for r in ledger)
        optimizer=result.get('optimizer',{})
        # An old generic message does not establish global time exhaustion.
        # Null means scope unavailable, not zero budget usage.
        legacy_limit='total_budget_exhausted' not in optimizer and any('budget' in m.lower() for m in optimizer.get('messages',[]))
        budget_hit=optimizer.get('total_budget_exhausted',None if legacy_limit else False)
        search_limits=sum(r.get('stop_reason')=='search_time_limit' for r in optimizer.get('searches',[]))
        diagnostic={"final_radial_speed_m_s":radial,"maximum_altitude_m":max((r['altitude_m'] for r in series),default=None),
                    "propellant_balance_error_kg":ledger_error,"optimizer_budget_exhausted":budget_hit,
                    "search_time_limits":search_limits,"invalid_candidates":optimizer.get('invalid_candidates',0)}
        if result['status']!='Target Achieved':
            if budget_hit:
                finding('solver_budget','warning','Optimization budget exhausted',
                        'The local search ran out of time. No completed SLSQP iteration was reported.' if optimizer.get('iterations')==0 else 'The local search ran out of time. The returned candidate does not prove physical infeasibility.')
            elif legacy_limit:
                finding('legacy_search_limit','info','Search time limit reported; scope unavailable',
                        'This older result uses a generic budget message. It does not distinguish an individual search allowance from the total optimization budget. A new analysis records each stop reason separately.')
            elif search_limits:
                finding('search_time_limit','info','Individual search time allowances reached',
                        f'{search_limits} searches reached their allotted time. The total optimization budget was not exhausted; inspect each search termination reason.')
            if optimizer.get('invalid_candidates',0):
                finding('invalid_trials','info','Invalid optimization candidates rejected',
                        f"{optimizer['invalid_candidates']} trial candidates were rejected. They were not accepted as trajectories; the returned flight was independently replayed.")
            for row in result.get('target_comparison',[]):
                if not row['within_tolerance']:
                    height=row['parameter'].endswith('_m')
                    error='undefined' if row['error'] is None else f"{abs(row['error'])/(1000 if height else 1):.4f} {'km' if height else 'degrees'}"
                    tolerance=f"{row['tolerance']/(1000 if height else 1):.4f} {'km' if height else 'degrees'}"
                    finding('target_'+row['parameter'],'warning',row['parameter'].replace('_altitude_m',' altitude').replace('_deg','').replace('_',' ').capitalize()+' outside tolerance',
                            f'The independently replayed target error is {error}; the allowed tolerance is {tolerance}. Search termination and target acceptance are separate checks.')
            perigee=result.get('orbit',{}).get('perigee_altitude_m',100000)
            if perigee<minimum_radius-R_E:
                finding('suborbital','warning','Final orbit intersects Earth','The final osculating perigee lies inside the Earth reference radius. This is a suborbital state; future impact has not been propagated unless the outcome explicitly says Impact.')
            elif perigee<100000:
                finding('low_perigee','warning','Final perigee is below the orbital floor','The final perigee is below the required 100 km minimum. A bound osculating orbit alone does not satisfy the mission.')
            if radial is not None and radial<0:
                finding('descending','warning','Vehicle is descending',f'Final radial velocity is {radial:.0f} m/s. Inspect the altitude history and guidance as well as the capacity screen.')
            reserve=result.get('summary',{}).get('propellant_remaining_kg',0)
            if reserve>1:
                finding('fuel_remaining','info','Failure is not fuel exhaustion alone',f'{reserve/1000:.2f} t of propellant remains aboard at the final state. Cutoff, guidance or search termination also matters.')
        if abs(ledger_error)>max(.01,sum(r['initial_kg'] for r in ledger)*1e-6):
            finding('fuel_balance','warning','Propellant accounting mismatch','Initial fuel does not balance burned, carried and discarded fuel. Inspect the numerical result.')
    return {"mission_budget":mission_budget(scenario,result),"status":"Review required" if any(f['severity']=='warning' for f in findings) else 'Screening passed',
            "ideal_delta_v_m_s":capacity,"energy_screen_delta_v_m_s":required,"optimistic_margin_m_s":margin,
            "initial_rotation_speed_m_s":speed,"initial_twr":initial_twr,"maximum_pad_twr":maximum_twr,
            "optimistic_payload_ceiling_kg":ceiling,"stages":rows,"findings":findings,"diagnostic":diagnostic,
            "assumptions":"Full propellant use, immediate empty-stage disposal and vacuum Isp give ideal capacity. The central-gravity energy screen permits any direction and a deliberately low Earth surface radius; it omits J2, atmospheric energy exchange and trajectory losses. It is a necessary-condition screen within that simplified model, not a flight feasibility certificate. Initial T/W uses standard-atmosphere pad pressure; the full simulator uses the selected atmosphere.",
            "source_url":"https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-rocket-equation/"}
