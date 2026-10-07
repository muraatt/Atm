"""GEO guidance keeps the launch lock and honors the actual target tolerances."""
import casadi as ca
import numpy as np
import pytest

from test_mission import orbital_case
from atmosphere.collocation import CollocationProblem
from atmosphere.dynamics import compare_target
from atmosphere.earth import MU,R_E,unit,targeted_elements
from atmosphere.models import TargetOrbit
from atmosphere.restoration import restore_continuous


def synchronous_case(inclination=.05,deficit=.3):
    sim,plan=orbital_case(0)
    sim.scenario.target=TargetOrbit(perigee_altitude_m=35786000,apogee_altitude_m=35786000,
                                  inclination_deg=0,altitude_tolerance_m=1000,angle_tolerance_deg=.1)
    r=unit(np.array([-1.,.0514,0]))*(R_E+35786000)
    tangent=unit(np.cross([0,0,1],r));angle=np.radians(inclination)
    direction=tangent*np.cos(angle)+np.array([0,0,np.sin(angle)])
    sim.initial=np.r_[r,direction*(np.sqrt(MU/np.linalg.norm(r))-deficit),10000.]
    burn=plan.burns[0];burn.duration_s=.016
    burn.steering_reference='inertial_velocity';burn.angle_fractions=(0.,)*3
    return sim,plan


def test_circular_feasibility_does_not_force_an_already_acceptable_plane_to_zero():
    sim,plan=synchronous_case(deficit=0)
    rows=compare_target(targeted_elements(sim.initial[:3],sim.initial[3:6],sim.scenario.target),sim.scenario.target)
    assert all(row['within_tolerance'] for row in rows)
    problem=CollocationProblem(sim,plan,4)
    x=np.asarray(problem.w0);x[problem.final_slice]=sim.initial/problem.scale(0)
    cost=ca.Function('acceptable_plane_cost',[problem.x],[problem.feasibility_objective])
    # The separate velocity-conditioning merit can prefer another allowed
    # plane, but the orbital residual itself must accept this plane.
    assert float(cost(x))-float(problem.conditioning_function(x))<1e-10


def test_geo_restoration_uses_plane_tolerance_without_relaxing_orbit_acceptance():
    sim,plan=synchronous_case()
    target=sim.scenario.target.model_dump()
    before=sim.run(plan,strict=True)
    assert before['status']=='Target Not Reached'
    restored,report=restore_continuous(sim,plan,30,lambda _:None)
    after=sim.run(restored,strict=True)
    assert after['status']=='Target Achieved',after['target_comparison']
    assert all(after['constraint_checks'].values())
    assert all(row['within_tolerance'] for row in after['target_comparison'])
    assert .02<after['orbit']['inclination_deg']<.08
    assert sim.scenario.target.model_dump()==target


def test_synchronous_relative_speed_still_uses_the_explicit_vertical_lock():
    sim,plan=synchronous_case(inclination=0,deficit=0)
    _,row=sim.forces(0,sim.initial,0,plan.burns[0],0,90)
    assert row['relative_speed_m_s']<1
    assert row['thrust_velocity_angle_deg']==0


def test_plane_margin_uses_the_actual_angle_at_large_tolerances():
    sim,plan=synchronous_case(inclination=8.01,deficit=0)
    sim.scenario.target.angle_tolerance_deg=10
    problem=CollocationProblem(sim,plan,4)
    x=np.asarray(problem.w0);x[problem.final_slice]=sim.initial/problem.scale(0)
    cost=ca.Function('outside_angular_margin',[problem.x],[problem.feasibility_objective])
    # sin(i)/tolerance can lie inside .8 even though i/tolerance exceeds .8.
    assert float(cost(x))>1e-10


def test_inertial_arrival_merit_prefers_well_conditioned_allowed_planes():
    values=[]
    for inclination in (0,.05):
        sim,plan=synchronous_case(inclination=inclination,deficit=0)
        assert all(row['within_tolerance'] for row in compare_target(
            targeted_elements(sim.initial[:3],sim.initial[3:6],sim.scenario.target),sim.scenario.target))
        problem=CollocationProblem(sim,plan,4)
        x=np.asarray(problem.w0);x[problem.final_slice]=sim.initial/problem.scale(0)
        cost=ca.Function(f'conditioned_arrival_{len(values)}',[problem.x],[problem.feasibility_objective])
        values.append(float(cost(x)))
    assert values[0]>values[1]+1
