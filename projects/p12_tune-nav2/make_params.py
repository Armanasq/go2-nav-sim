#!/usr/bin/env python3
"""Write one Nav2 params file per setting into params/, each = the stack's config/nav2_params.yaml + a few changes.
usage: python3 projects/p12_tune-nav2/make_params.py          (prints the changes of every setting)
The stack's own file is only read, never written."""
import copy, pathlib, yaml

HERE = pathlib.Path(__file__).resolve().parent
BASE = HERE.parents[1] / 'src/go2_nav_sim/config/nav2_params.yaml'
# Go2 body 0.65 m x 0.30 m (chapter 10), centred on base_footprint; corners lie on the 0.358 m circumscribed circle
POLYGON = '[[0.325, 0.15], [0.325, -0.15], [-0.325, -0.15], [-0.325, 0.15]]'

SMAC_HYBRID = dict(
    plugin='nav2_smac_planner/SmacPlannerHybrid', tolerance=0.3, allow_unknown=False, downsample_costmap=False,
    max_iterations=1000000, max_planning_time=5.0, motion_model_for_search='DUBIN', angle_quantization_bins=72,
    minimum_turning_radius=0.3,          # a Go2 can turn on the spot; 0.3 m keeps the search space small
    analytic_expansion_ratio=3.5, analytic_expansion_max_length=3.0, reverse_penalty=2.0, change_penalty=0.0,
    non_straight_penalty=1.2, cost_penalty=2.0, retrospective_penalty=0.015, lookup_table_size=20.0,
    cache_obstacle_heuristic=False, smooth_path=True,
    smoother=dict(max_iterations=1000, w_smooth=0.3, w_data=0.2, tolerance=1.0e-10, do_refinement=True, refinement_num=2))

MPPI = dict(
    plugin='nav2_mppi_controller::MPPIController',
    time_steps=42, model_dt=0.067,       # model_dt = controller period (15 Hz); 42 steps = 2.8 s horizon
    batch_size=1000,                     # half the example value: the machine runs other simulators
    iteration_count=1, temperature=0.3, gamma=0.015, motion_model='DiffDrive', visualize=False, reset_period=1.0,
    vx_std=0.2, vy_std=0.0, wz_std=0.4, vx_max=0.25, vx_min=-0.1, vy_max=0.0, wz_max=0.5,   # = RPP and velocity smoother
    prune_distance=1.7, transform_tolerance=0.2, regenerate_noises=False,
    critics=['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic', 'PathAlignCritic', 'PathFollowCritic',
             'PathAngleCritic', 'PreferForwardCritic'],
    ConstraintCritic=dict(enabled=True, cost_power=1, cost_weight=4.0),
    CostCritic=dict(enabled=True, cost_power=1, cost_weight=3.81, critical_cost=300.0, consider_footprint=True,
                    collision_cost=1000000.0, near_goal_distance=1.0, trajectory_point_step=2),
    GoalCritic=dict(enabled=True, cost_power=1, cost_weight=5.0, threshold_to_consider=1.4),
    GoalAngleCritic=dict(enabled=True, cost_power=1, cost_weight=3.0, threshold_to_consider=0.5),
    PathAlignCritic=dict(enabled=True, cost_power=1, cost_weight=14.0, max_path_occupancy_ratio=0.05, trajectory_point_step=4,
                         threshold_to_consider=0.5, offset_from_furthest=20, use_path_orientations=False),
    PathFollowCritic=dict(enabled=True, cost_power=1, cost_weight=5.0, offset_from_furthest=5, threshold_to_consider=1.4),
    PathAngleCritic=dict(enabled=True, cost_power=1, cost_weight=2.0, offset_from_furthest=4, threshold_to_consider=0.5,
                         max_angle_to_furthest=1.0, mode=0),
    PreferForwardCritic=dict(enabled=True, cost_power=1, cost_weight=5.0, threshold_to_consider=0.5))

# name: (inflation_radius, cost_scaling_factor, footprint, planner, controller); None = the stack's value
SETTINGS = {
    'base':       (None, None, None, None, None),          # the stack as shipped: r 0.75, scale 4.0, circle 0.33, NavFn + RPP
    'infl_r045':  (0.45, None, None, None, None),
    'infl_r150':  (1.50, None, None, None, None),
    'scale_1':    (None, 1.0, None, None, None),
    'scale_10':   (None, 10.0, None, None, None),
    'polygon':    (None, None, POLYGON, None, None),
    'smac_mppi':  (None, None, POLYGON, SMAC_HYBRID, MPPI),
    # Task 4: NavFn + RPP with the polygon, but RPP never turns on the spot (keys merged into the stack's RPP block)
    'polygon_norot': (None, None, POLYGON, None, dict(use_rotate_to_heading=False)),
}


def make(base, radius, scale, footprint, planner, controller):
    P = copy.deepcopy(base)
    for cm in ('local_costmap', 'global_costmap'):
        q = P[cm][cm]['ros__parameters']
        if radius is not None:
            q['inflation_layer']['inflation_radius'] = radius
        if scale is not None:
            q['inflation_layer']['cost_scaling_factor'] = scale
        if footprint is not None:
            q.pop('robot_radius'); q['footprint'] = footprint
    if planner is not None:
        P['planner_server']['ros__parameters']['GridBased'] = planner
    if controller is not None:
        fp = P['controller_server']['ros__parameters']['FollowPath']
        P['controller_server']['ros__parameters']['FollowPath'] = controller if 'plugin' in controller else {**fp, **controller}
    return P


if __name__ == '__main__':
    base = yaml.safe_load(BASE.read_text())
    for name, s in SETTINGS.items():
        (HERE / 'params' / f'{name}.yaml').write_text(yaml.safe_dump(make(base, *s), sort_keys=False))  # type: ignore[reportArgumentType]  # PyYAML has no stubs: pyright infers bytes | str | None from source; safe_dump without a stream returns str
        changes = [f'{k}={v}' for k, v in zip(('inflation_radius', 'cost_scaling_factor', 'footprint'), s[:3]) if v is not None]
        changes += [d.get('plugin', str(d)) for d in s[3:] if d is not None]
        print(f'params/{name}.yaml: {", ".join(changes) or "unchanged"}')
