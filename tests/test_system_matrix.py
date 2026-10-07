"""Validate study inputs before spending time on the full mission matrix."""
import importlib.util
from pathlib import Path
import sys

import pytest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from verify_system_matrix import CASES,scenario_for
from atmosphere.engines import validate_design
from atmosphere.dynamics import Simulation


@pytest.mark.parametrize('name',list(CASES))
def test_varied_matrix_inputs_resolve_real_surface_state(name):
    s=scenario_for(name,5)
    validate_design(s,require_engines=True)
    sim=Simulation(s)
    lat,lon,height=sim.frames.geographic(0,sim.initial[:3])
    assert lat==pytest.approx(s.launch.latitude_deg,abs=1e-6)
    assert lon==pytest.approx(s.launch.longitude_deg,abs=1e-6)
    assert height==pytest.approx(s.launch.elevation_m+s.launch.geoid_height_m,abs=1e-5)
    assert sim.surface_clearance(0,sim.initial[:3])[0]==pytest.approx(0,abs=1e-5)
    assert s.target.altitude_tolerance_m==1000 and s.target.angle_tolerance_deg==.1
    assert s.launch.elevation_resolved
    assert len(s.vehicle.stages)==2


def test_matrix_covers_distinct_mission_behaviors():
    scenarios=[scenario_for(name,5) for name in CASES]
    assert {c['family'] for c in CASES.values()}=={'leo','meo','gto','geo'}
    assert any(s.target.inclination_deg and s.target.inclination_deg>90 for s in scenarios)
    assert any(s.target.perigee_altitude_m is None for s in scenarios)
    assert any(s.target.inclination_deg is None for s in scenarios)
    assert any(s.launch.latitude_deg>40 for s in scenarios)
    assert any(s.launch.elevation_m<0 for s in scenarios)
    assert any(s.launch.elevation_m>1000 for s in scenarios)
    assert {s.objective for s in scenarios}=={'propellant','time'}
    assert {s.environment.model for s in scenarios}=={'standard','msis'}
    assert {s.vehicle.optimize_stage_masses for s in scenarios}=={True,False}
