"""Mapping stack: the simulator, the estimation stack (scan, lidar odometry, EKF) and slam_toolbox in mapping mode, which
owns the map frame (it publishes /map and TF map -> odom). No map server, no AMCL, no Nav2.
usage: ros2 launch projects/p07_build-a-map/map.launch.py
The estimation stack and slam_toolbox start once the robot's IMU publishes (the same readiness check as scripts/up.sh), so
the odom frame begins where the robot stands after spawning, not while it is still falling onto the floor."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    me = share('go2_nav_sim')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'sim.launch.py')))
    est = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(me, 'launch', 'estimation.launch.py')))
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
                # config/slam.yaml is written for the passive mapper of nav.launch.py (frame slam_map, no TF).
                # These two overrides make it the mapper that owns 'map'.
                parameters=[os.path.join(me, 'config', 'slam.yaml'),
                            {'map_frame': 'map', 'transform_publish_period': 0.05, 'use_sim_time': True}])
    wait_imu = ExecuteProcess(cmd=['bash', '-c', 'until timeout 10 ros2 topic echo --once /imu/data --no-daemon '
                                   '> /dev/null 2>&1; do sleep 2; done; echo robot is up'], output='screen')
    return LaunchDescription([
        sim, wait_imu,
        RegisterEventHandler(OnProcessExit(target_action=wait_imu, on_exit=[est, slam])),
    ])
