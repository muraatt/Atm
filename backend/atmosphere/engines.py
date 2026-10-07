"""Source-backed engine identities; Stage thrust remains aggregate thrust."""
import json
import math
from pathlib import Path

CATALOG = json.loads((Path(__file__).resolve().parents[2] / 'shared' / 'engines.json').read_text(encoding='utf-8'))
BY_ID = {engine['id']: engine for engine in CATALOG['engines']}


def validate_design(scenario, require_engines=False):
    # Historical simulations remain readable with more stages.
    if len(scenario.vehicle.stages) > 2:
        raise ValueError('New vehicle designs support at most two stages and one payload. Remove an extra stage in Vehicle & Stages; historical results are preserved.')
    for index, stage in enumerate(scenario.vehicle.stages):
        if stage.engine_id is None:
            if require_engines:
                raise ValueError(f'Select a catalog engine for stage {index+1} in Vehicle & Stages before starting a new analysis.')
            continue
        engine = BY_ID.get(stage.engine_id)
        if engine is None:
            raise ValueError(f'Unknown engine catalog identity: {stage.engine_id}.')
        if engine['role'] != ('booster' if index == 0 else 'upper'):
            raise ValueError(f'{engine["name"]} is not offered for stage {index+1}. Select an engine for this stage role.')
        if stage.engine_type != 'liquid':
            raise ValueError('Catalog engines use liquid propulsion.')
        if engine['throttle_min'] is not None and stage.throttle_min < engine['throttle_min']-1e-8:
            raise ValueError(f'{engine["name"]} minimum throttle cannot be below its published {engine["throttle_min"]*100:.1f}% reference. Review Stage & flight-model settings.')
        if engine['vacuum_thrust_n'] is not None and not math.isclose(stage.vacuum_thrust_n, engine['vacuum_thrust_n']*stage.engine_count, rel_tol=1e-8):
            raise ValueError('Stage thrust must equal catalog per-engine vacuum thrust times engine count.')
        if engine['isp_vacuum_s'] is not None and not math.isclose(stage.isp_vacuum_s, engine['isp_vacuum_s'], rel_tol=1e-8):
            raise ValueError('Vacuum Isp must match the selected catalog reference.')
        if engine['sea_level_thrust_n'] is not None and not math.isclose(stage.vacuum_thrust_n*stage.isp_sea_level_s/stage.isp_vacuum_s,engine['sea_level_thrust_n']*stage.engine_count,rel_tol=1e-8):
            raise ValueError('Stage sea-level thrust must reproduce the catalog reference using the model mass flow.')
