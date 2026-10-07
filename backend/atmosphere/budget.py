"""Fast, explicit reference-mission sizing; never a trajectory certificate.

Dry masses are fixed. Ascent delta-v is allocated in proportion to the existing
vehicle's vacuum stage contributions. Inverse sizing works backward, carrying
the estimated upper-stage fuel. Arrival and contingency fuel stay in the final
stage. No estimate changes the scenario or creates fuel in a flight simulation.
"""
import math

import numpy as np
from .earth import G0, MU, OMEGA, R_E, geodetic_to_ecef, gravity
from .environment import StandardAtmosphere
from .screening import departure_options


def inverse_fuel(final_mass, delta_v, isp):
    exponent = delta_v / (G0 * isp)
    if exponent > 600:
        return None  # JSON-safe unavailable estimate instead of infinity
    return final_mass * math.expm1(exponent)


def stage_capacities(scenario, payload):
    mass = payload + sum(s.dry_mass_kg+s.propellant_mass_kg for s in scenario.vehicle.stages)
    contributions = []
    for stage in scenario.vehicle.stages:
        contributions.append(G0*stage.isp_vacuum_s*math.log1p(stage.propellant_mass_kg/(mass-stage.propellant_mass_kg)))
        mass -= stage.propellant_mass_kg+stage.dry_mass_kg
    return contributions


def size_reference(scenario, payload, departure_dv, arrival_dv, contingency_dv=0):
    stages = scenario.vehicle.stages
    capacity = stage_capacities(scenario, payload)
    allocations = [departure_dv*c/sum(capacity) for c in capacity]
    rows = []
    carried = payload
    protected = inverse_fuel(payload+stages[-1].dry_mass_kg, contingency_dv, stages[-1].isp_vacuum_s)
    if protected is None:
        return {'available': False, 'reason': 'Reference mass ratio exceeds the supported sizing range.', 'stages': []}
    for i in reversed(range(len(stages))):
        stage = stages[i]
        dry_stack = carried + stage.dry_mass_kg
        reserve = protected if i == len(stages)-1 else 0.
        arrival = inverse_fuel(dry_stack+reserve, arrival_dv if i == len(stages)-1 else 0., stage.isp_vacuum_s)
        if arrival is None:
            return {'available': False, 'reason': 'Reference mass ratio exceeds the supported sizing range.', 'stages': []}
        ascent = inverse_fuel(dry_stack+reserve+arrival, allocations[i], stage.isp_vacuum_s)
        if ascent is None or not math.isfinite(ascent+arrival+reserve):
            return {'available': False, 'reason': 'Reference mass ratio exceeds the supported sizing range.', 'stages': []}
        required = ascent+arrival+reserve
        rows.append({'stage': stage.name, 'stage_index': i, 'ascent_delta_v_m_s': allocations[i],
                     'ascent_fuel_kg': ascent, 'arrival_fuel_kg': arrival, 'protected_reserve_kg': reserve,
                     'required_fuel_kg': required, 'available_fuel_kg': stage.propellant_mass_kg,
                     'shortfall_kg': max(0., required-stage.propellant_mass_kg),
                     'fuel_after_ascent_kg': arrival+reserve, 'resizable': stage.engine_type == 'liquid'})
        carried = dry_stack+required
    rows.reverse()
    return {'available': True, 'total_required_fuel_kg': sum(r['required_fuel_kg'] for r in rows),
            'total_available_fuel_kg': sum(s.propellant_mass_kg for s in stages),
            'stage_shortfall_kg': sum(r['shortfall_kg'] for r in rows),
            'arrival_fuel_kg': sum(r['arrival_fuel_kg'] for r in rows),
            'protected_reserve_kg': protected, 'initial_mass_kg': carried,
            'within_stage_capacities': all(r['shortfall_kg'] < .01 for r in rows), 'stages': rows}


def onward_geo(result, scenario):
    orbit = result.get('orbit', {})
    if not orbit.get('bound') or abs((orbit.get('apogee_altitude_m') or 0)-35786000) > 1000000:
        return None
    rows = result.get('series', [])
    if not rows:
        return None
    index = rows[-1]['stage_index']
    stage = scenario.vehicle.stages[index]
    fuel = rows[-1]['active_propellant_kg']
    mass = result['summary']['final_mass_kg']
    dry = mass-fuel
    if dry <= 0:
        return None
    radius = R_E+orbit['apogee_altitude_m']
    vt = math.sqrt(MU*(2/radius-1/orbit['semi_major_axis_m']))
    vc = math.sqrt(MU/radius)
    coplanar = abs(vc-vt)
    combined = math.sqrt(max(0, vc*vc+vt*vt-2*vc*vt*math.cos(math.radians(orbit['inclination_deg']))))
    return {'remaining_delta_v_m_s': G0*stage.isp_vacuum_s*math.log(mass/dry), 'remaining_fuel_kg': fuel,
            'coplanar_delta_v_m_s': coplanar, 'coplanar_fuel_kg': inverse_fuel(dry,coplanar,stage.isp_vacuum_s),
            'aligned_node_geo_delta_v_m_s': combined, 'aligned_node_geo_fuel_kg': inverse_fuel(dry,combined,stage.isp_vacuum_s),
            'note': 'Diagnostic reference at the existing apogee radius. Combined circularization and equatorial plane change assumes apogee lies on the plane intersection; this geometry and steering capability are not verified. Reserves would have to be carried throughout ascent.'}


def mission_budget(scenario, result=None):
    target = scenario.target
    rp = R_E+(target.perigee_altitude_m if target.perigee_altitude_m is not None else 100000)
    ra = max(rp,R_E+(target.apogee_altitude_m if target.apogee_altitude_m is not None else rp-R_E))
    parking = min(rp,R_E+200000)
    transfer_a = (parking+ra)/2
    target_a = (rp+ra)/2
    plane, azimuth, departure_inc = departure_options(scenario.launch.latitude_deg,target,scenario.launch.azimuth_sectors_deg)
    position = geodetic_to_ecef(scenario.launch.latitude_deg,scenario.launch.longitude_deg,
                                scenario.launch.elevation_m+scenario.launch.geoid_height_m)
    rotation = OMEGA*math.hypot(position[0],position[1])
    injection_speed = math.sqrt(MU*(2/parking-1/transfer_a))
    departure = math.sqrt(max(0,injection_speed**2+rotation**2-2*injection_speed*rotation*math.sin(math.radians(azimuth))))
    transfer_speed = math.sqrt(MU*(2/ra-1/transfer_a))
    final_speed = math.sqrt(MU*(2/ra-1/target_a))
    circularization = abs(final_speed-transfer_speed)
    arrival = math.sqrt(max(0,final_speed**2+transfer_speed**2-2*final_speed*transfer_speed*math.cos(math.radians(plane))))
    assumptions = scenario.budget
    losses = assumptions.gravity_loss_m_s+assumptions.drag_loss_m_s+assumptions.steering_loss_m_s
    nominal = departure+arrival+losses
    contingency = nominal*assumptions.contingency_percent/100
    payload = scenario.vehicle.payload_mass_kg
    tiers = []
    first = scenario.vehicle.stages[0]
    pressure = StandardAtmosphere().sample(scenario.launch.latitude_deg,scenario.launch.longitude_deg,
        scenario.launch.elevation_m+scenario.launch.geoid_height_m,0).pressure_pa
    pad_isp = first.isp_vacuum_s-(first.isp_vacuum_s-first.isp_sea_level_s)*min(1.,max(0.,pressure/101325))
    shape = first.solid_burn_curve[0][1] if first.engine_type == 'solid' and first.solid_burn_curve else (1. if first.engine_type == 'solid' else first.throttle_max)
    pad_thrust = first.vacuum_thrust_n*pad_isp/first.isp_vacuum_s*shape
    omega = np.array([0.,0.,OMEGA])
    local_weight = float(np.dot(np.cross(omega,np.cross(omega,position))-gravity(position),position/np.linalg.norm(position)))
    for name, extra, reserve in [('Ideal reference',0,0),('Nominal',losses,0),('With margin',losses,contingency)]:
        sizing = size_reference(scenario,payload,departure+extra,arrival,reserve)
        tiers.append({'name': name, 'delta_v_m_s': departure+arrival+extra+reserve,
                      'initial_pad_twr': pad_thrust/(sizing['initial_mass_kg']*local_weight) if sizing['available'] else None,
                      **sizing})
    warnings = []
    advanced = [key for key in ('raan_deg','argument_of_periapsis_deg','arrival_phase_deg','arrival_time_min_s','arrival_time_max_s') if getattr(target,key) is not None]
    if advanced:
        warnings.append('Advanced angles and arrival windows are not priced by this reference: '+', '.join(advanced)+'.')
    if target.perigee_altitude_m is None or target.apogee_altitude_m is None:
        warnings.append('Free apses use a low-energy illustrative orbit with the 100 km floor. These values do not become flight constraints.')
    if any(s.engine_type == 'solid' for s in scenario.vehicle.stages):
        warnings.append('Solid propellant loads and profiles are fixed. Inverse fuel amounts are mathematical allocations, not valid resized solid motors.')
    if arrival > 1 and scenario.vehicle.stages[-1].ignition_limit < 2:
        warnings.append('This reference uses the final stage during ascent and arrival and needs two ignitions. A dedicated arrival stage would need a different allocation.')
    if not tiers[-1].get('within_stage_capacities',False):
        warnings.append('The margin estimate exceeds one or more stage propellant loads. A larger total tank elsewhere does not cover a local shortage.')
    if tiers[-1]['initial_pad_twr'] is not None and tiers[-1]['initial_pad_twr'] <= 1:
        warnings.append('The sized margin load has estimated initial pad T/W at or below one. Extra fuel cannot be loaded with the current first-stage thrust without a liftoff review.')
    warnings.append('Steering, thrust-to-weight, launch corridors and flight limits require continuous trajectory verification. These reference estimates do not certify feasibility.')
    sweep = [{'payload_kg': payload*factor, **size_reference(scenario,payload*factor,departure+losses,arrival,contingency)} for factor in (.5,1,1.5,2)]
    return {'version': 1, 'assumptions': assumptions.model_dump(),
            'reference': {'perigee_altitude_m': rp-R_E, 'apogee_altitude_m': ra-R_E,
                          'parking_altitude_m': parking-R_E, 'departure_inclination_deg': departure_inc,
                          'plane_change_deg': plane},
            'components': [{'name':'Departure / transfer injection','delta_v_m_s':departure},
                           {'name':'Arrival maneuver (combined)','delta_v_m_s':arrival},
                           {'name':'Gravity loss allowance','delta_v_m_s':assumptions.gravity_loss_m_s},
                           {'name':'Drag loss allowance','delta_v_m_s':assumptions.drag_loss_m_s},
                           {'name':'Steering loss allowance','delta_v_m_s':assumptions.steering_loss_m_s},
                           {'name':'Contingency reserve','delta_v_m_s':contingency}],
            'coplanar_arrival_delta_v_m_s':circularization, 'tiers':tiers, 'payload_sweep':sweep,
            'warnings':warnings, 'onward_geo':onward_geo(result,scenario) if result else None,
            'method': 'Impulsive reference plus user loss allowances; inverse vacuum rocket equation with fixed dry masses and proportional ascent delta-v allocation. Upper-stage fuel is carried in all lower-stage sizing. Not a global minimum or confidence interval.',
            'reserve_policy':'Contingency delta-v is stored as fuel in the final stage after the arrival maneuver. Loss allowances are applied to ascent. Required loads are advisory and never applied to the vehicle automatically.'}
