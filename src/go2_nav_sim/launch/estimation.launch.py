"""Perception + state estimation: 3D cloud -> /scan, planar lidar(-inertial) odometry -> EKF -> odom TF. Robot/sim agnostic."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(share('go2_nav_sim'), 'config')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}     # listed after a YAML file, so it overrides the file's value
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='take time from /clock (Gazebo or a bag)'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', name='pointcloud_to_laserscan',
             parameters=[os.path.join(cfg, 'scan.yaml'), sim], remappings=[('cloud_in', '/lidar/points'), ('scan', '/scan')]),
        Node(package='go2_nav_sim', executable='lidar_odom.py', name='lidar_odom', parameters=[sim], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='odom_ekf',
             parameters=[os.path.join(cfg, 'ekf.yaml'), sim], remappings=[('odometry/filtered', '/odom')]),
    ])
