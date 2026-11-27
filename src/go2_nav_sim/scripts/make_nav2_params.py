#!/usr/bin/env python3
"""config/nav2_params.yaml = Humble's stock nav2_params.yaml + the Go2 deltas below (regenerate: python3 scripts/make_nav2_params.py)."""
import pathlib, yaml
ROOT = pathlib.Path(__file__).resolve().parents[1]
P = yaml.safe_load(open('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'))
p = lambda n: P[n]['ros__parameters']
A = (-4.5, -3.5)                                                   # start marker, map frame (== make_arena.A)
R = 0.33                                                           # circumscribed radius of a Go2 (~0.65 m long)

p('amcl').update(base_frame_id='base_footprint', scan_topic='scan', set_initial_pose=True, initial_pose=dict(x=A[0], y=A[1], z=0.0, yaw=0.0), laser_max_range=10.0, max_beams=120,
                 update_min_d=0.05, update_min_a=0.1, robot_model_type='nav2_amcl::DifferentialMotionModel')
p('bt_navigator').update(robot_base_frame='base_footprint', odom_topic='/odom')

c = p('controller_server')
c.update(controller_frequency=15.0, min_x_velocity_threshold=0.001, min_theta_velocity_threshold=0.001)
c['progress_checker'].update(required_movement_radius=0.2, movement_time_allowance=20.0)
c['general_goal_checker'].update(xy_goal_tolerance=0.25, yaw_goal_tolerance=0.3)
c['FollowPath'] = dict(                                              # smooth, forward-only; rotates in place at start/goal
    plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    desired_linear_vel=0.25, lookahead_dist=0.6, min_lookahead_dist=0.3, max_lookahead_dist=0.9,
    lookahead_time=1.5, transform_tolerance=0.2, use_velocity_scaled_lookahead_dist=True,
    use_rotate_to_heading=True, rotate_to_heading_min_angle=0.6, rotate_to_heading_angular_vel=0.4, max_angular_accel=1.0,
    allow_reversing=False, use_regulated_linear_velocity_scaling=True, regulated_linear_scaling_min_radius=0.9,
    regulated_linear_scaling_min_speed=0.1, use_cost_regulated_linear_velocity_scaling=False,
    use_collision_detection=True, max_allowed_time_to_collision_up_to_carrot=1.0, use_interpolation=True)

for cm, rolling in (('local_costmap', True), ('global_costmap', False)):
    q = P[cm][cm]['ros__parameters']
    q.update(robot_base_frame='base_footprint', robot_radius=R, resolution=0.05, update_frequency=5.0, publish_frequency=2.0)
    q.pop('footprint', None)
    q['inflation_layer'].update(inflation_radius=0.75, cost_scaling_factor=4.0)
    q['obstacle_layer'] = dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True, observation_sources='scan',
        scan=dict(topic='/scan', max_obstacle_height=2.0, clearing=True, marking=True, data_type='LaserScan',
                  raytrace_max_range=3.8, raytrace_min_range=0.0, obstacle_max_range=3.5, obstacle_min_range=0.0))
    q.pop('voxel_layer', None)
    if rolling:
        q.update(rolling_window=True, width=5, height=5, plugins=['obstacle_layer', 'inflation_layer'])
    else:
        q.update(plugins=['static_layer', 'obstacle_layer', 'inflation_layer'])

pl = p('planner_server')['GridBased']; pl.update(tolerance=0.3, use_astar=True, allow_unknown=False)
p('smoother_server')  # stock simple smoother kept
vs = p('velocity_smoother')
vs.update(max_velocity=[0.3, 0.0, 0.5], min_velocity=[-0.1, 0.0, -0.5], max_accel=[0.5, 0.0, 1.0], max_decel=[-0.8, 0.0, -1.5],
          odom_topic='odom', velocity_timeout=0.5)
p('behavior_server').update(local_costmap_topic='local_costmap/costmap_raw', robot_base_frame='base_footprint')
for s in ('amcl', 'bt_navigator', 'controller_server', 'planner_server', 'behavior_server', 'velocity_smoother'):
    p(s)['use_sim_time'] = True

# sensor ranges: local window is 5x5 m, global costmap spans the whole map -> use the 3D lidar's reach there
for cm, (om, rm) in (('local_costmap', (4.0, 4.5)), ('global_costmap', (8.0, 9.0))):
    s = P[cm][cm]['ros__parameters']['obstacle_layer']['scan']; s.update(obstacle_max_range=om, raytrace_max_range=rm)

(ROOT / 'config/nav2_params.yaml').write_text(yaml.safe_dump(P, sort_keys=False))

# ---- slam_toolbox: passive mapper (publishes /slam/map, never the map->odom TF, so it cannot fight AMCL) ----
M = yaml.safe_load(open('/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml'))
m = M['slam_toolbox']['ros__parameters']
m.update(use_sim_time=True, max_laser_range=12.0, scan_buffer_maximum_scan_distance=12.0, minimum_travel_distance=0.15,
         minimum_travel_heading=0.15, minimum_time_interval=0.3, map_update_interval=1.0, enable_interactive_mode=False,
         transform_timeout=0.3, throttle_scans=1, map_frame='slam_map', transform_publish_period=0.0)
(ROOT / 'config/slam.yaml').write_text(yaml.safe_dump(M, sort_keys=False))
print('wrote nav2_params.yaml, slam.yaml')
