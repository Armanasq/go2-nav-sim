"""Gazebo-classic arena + Go2 (CHAMP gait controller) + sensors + ground-truth odom. Start pose = marker A, heading +x."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

START_X, START_Y = -4.5, -3.5          # == make_arena.A


def generate_launch_description():
    me, cfg = share('go2_nav_sim'), share('go2_config')
    xacro = f"{os.path.join(me, 'urdf/go2_nav.xacro')} start_x:={START_X} start_y:={START_Y}"
    sim_time = {'use_sim_time': True}
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('champ_bringup'), 'launch/bringup.launch.py')),
        launch_arguments={
            'description_path': xacro,
            'joints_map_path': os.path.join(cfg, 'config/joints/joints.yaml'),
            'links_map_path': os.path.join(cfg, 'config/links/links.yaml'),
            'gait_config_path': os.path.join(cfg, 'config/gait/gait.yaml'),
            'use_sim_time': 'true', 'robot_name': 'go2', 'gazebo': 'true', 'lite': 'false', 'rviz': 'false',
            'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
            'hardware_connected': 'false', 'publish_foot_contacts': 'false', 'close_loop_odom': 'true',
            'gt_odom': 'true',                       # patched arg: skip leg-odometry EKF, gt_odom node owns odom->base_footprint
        }.items())
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('champ_gazebo'), 'launch/gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'robot_name': 'go2', 'lite': 'false', 'description_path': xacro,
            'world': LaunchConfiguration('world'), 'headless': LaunchConfiguration('headless'),
            'world_init_x': str(START_X), 'world_init_y': str(START_Y), 'world_init_z': '0.275', 'world_init_heading': '0.0',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(me, 'worlds/arena.world')),
        DeclareLaunchArgument('headless', default_value='True', description='True = no gzclient window'),
        bringup, gazebo,
        DeclareLaunchArgument('odom', default_value='lidar', description="odom->base_footprint source: 'lidar' (estimation.launch.py) or 'gt' (simulator ground truth)"),
        Node(package='go2_nav_sim', executable='gt_odom.py', parameters=[sim_time], output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('odom'), "' == 'gt'"]))),
    ])
