"""Bounded joint stage sizing. Structural laws are explicit planning assumptions."""
import numpy as np
from .models import Scenario, StageMass, StageMassBounds


def suggested_bounds(stage):
    return StageMassBounds(dry_mass_min_kg=.5*stage.dry_mass_kg,
        dry_mass_max_kg=5*stage.dry_mass_kg, propellant_min_kg=.1*stage.propellant_mass_kg,
        propellant_max_kg=5*stage.propellant_mass_kg, hardware_mass_kg=.35*stage.dry_mass_kg,
        tank_structure_ratio=min(1.,.65*stage.dry_mass_kg/stage.propellant_mass_kg))


def pad_reference(sim):
    from .earth import gravity, unit
    stage=sim.scenario.vehicle.stages[0]
    _,row=sim.forces(0,sim.initial,0)
    isp=stage.isp_vacuum_s-(stage.isp_vacuum_s-stage.isp_sea_level_s)*np.clip(row['pressure_pa']/101325,0,1)
    peak=max((value for _,value in stage.solid_burn_curve),default=1.) if stage.engine_type=='solid' else 1.
    thrust=stage.vacuum_thrust_n*isp/stage.isp_vacuum_s*peak
    surface=np.cross(sim.frames._angular[0],sim.frames.earth_velocity(0,sim.initial[:3]))
    weight=float(np.dot(surface-gravity(sim.initial[:3],sim.frames.matrix(0)),unit(sim.initial[:3])))
    return float(thrust),weight


def mass_bounds(scenario):
    return [stage.mass_bounds or suggested_bounds(stage) for stage in scenario.vehicle.stages]


def masses(scenario):
    return [StageMass(dry_mass_kg=s.dry_mass_kg,propellant_mass_kg=s.propellant_mass_kg) for s in scenario.vehicle.stages]


def seed_masses(scenario):
    """A bounded initial design; the NLP still varies every liquid stage mass."""
    values=[]
    for stage,bound in zip(scenario.vehicle.stages,mass_bounds(scenario)):
        if stage.engine_type=='solid':
            values.append(StageMass(dry_mass_kg=stage.dry_mass_kg,propellant_mass_kg=stage.propellant_mass_kg))
            continue
        ceiling=bound.propellant_max_kg
        if bound.tank_structure_ratio:
            ceiling=min(ceiling,(bound.dry_mass_max_kg-bound.hardware_mass_kg)/bound.tank_structure_ratio)
        fuel=float(np.clip(stage.propellant_mass_kg,bound.propellant_min_kg,ceiling))
        dry=float(np.clip(stage.dry_mass_kg,max(bound.dry_mass_min_kg,bound.hardware_mass_kg+bound.tank_structure_ratio*fuel),bound.dry_mass_max_kg))
        values.append(StageMass(dry_mass_kg=dry,propellant_mass_kg=fuel))
    return values


def apply_stage_masses(scenario, values, check_bounds=True):
    if not scenario.vehicle.optimize_stage_masses:
        raise ValueError('This input snapshot does not enable stage mass optimization.')
    if len(values) != len(scenario.vehicle.stages):
        raise ValueError('A sized mission must supply masses for every stage.')
    result=scenario.model_copy(deep=True)
    result.vehicle.expected_initial_mass_kg=None
    for old,stage,value,bounds in zip(scenario.vehicle.stages,result.vehicle.stages,values,mass_bounds(scenario)):
        value=StageMass.model_validate(value)
        dry,fuel=value.dry_mass_kg,value.propellant_mass_kg
        tolerance=1e-6*max(1.,bounds.dry_mass_max_kg,bounds.propellant_max_kg)
        if old.engine_type=='solid':
            if abs(dry-old.dry_mass_kg)>1e-6 or abs(fuel-old.propellant_mass_kg)>1e-6:
                raise ValueError('Solid motor masses and burn profiles are fixed.')
        elif check_bounds and (dry < bounds.dry_mass_min_kg-tolerance or dry > bounds.dry_mass_max_kg+tolerance or
                fuel < bounds.propellant_min_kg-tolerance or fuel > bounds.propellant_max_kg+tolerance or
                dry < bounds.hardware_mass_kg+bounds.tank_structure_ratio*fuel-tolerance):
            raise ValueError('Optimized masses violate stage bounds or the hardware/tank structural law.')
        stage.mass_bounds=bounds
        stage.dry_mass_kg=dry;stage.propellant_mass_kg=fuel
    # Revalidate all numerical inputs; no change to engines, payload or guidance.
    return Scenario.model_validate(result.model_dump(mode='json'))


def design_report(original, sized):
    bounds=mass_bounds(original)
    rows=[]
    for i,(before,after,bound) in enumerate(zip(original.vehicle.stages,sized.vehicle.stages,bounds)):
        rows.append({'stage_index':i,'stage':before.name,'original_dry_mass_kg':before.dry_mass_kg,
            'original_propellant_mass_kg':before.propellant_mass_kg,'dry_mass_kg':after.dry_mass_kg,
            'propellant_mass_kg':after.propellant_mass_kg,'bounds':bound.model_dump(),
            'minimum_structural_dry_mass_kg':bound.hardware_mass_kg+bound.tank_structure_ratio*after.propellant_mass_kg,
            'fixed_solid_motor':before.engine_type=='solid'})
    return {'enabled':True,'bounds_verified':True,'original_initial_mass_kg':original.vehicle.initial_mass_kg,
        'optimized_initial_mass_kg':sized.vehicle.initial_mass_kg,'payload_mass_kg':sized.vehicle.payload_mass_kg,
        'minimum_initial_twr':original.vehicle.minimum_initial_twr,'stages':rows,
        'structural_model':'Dry mass >= fixed hardware mass + tank/structure ratio * loaded propellant. User-defined sizing assumptions, not structural certification.'}
