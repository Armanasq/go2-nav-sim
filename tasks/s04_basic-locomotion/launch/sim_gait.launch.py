"""sim.launch.py with one more argument, gait:=<gait.yaml>, so a stage-4 task can change CHAMP's gait without editing upstream.
Same robot, sensors and start pose (marker A, heading +x). Default world: this stage's flat.world (no obstacles, link states).
  ros2 launch tasks/s04_basic-locomotion/launch/sim_gait.launch.py gait:=$PWD/tasks/s04_basic-locomotion/params/gait_swing008.yaml"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

STAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
START_X, START_Y = -4.5, -3.5          # == sim.launch.py


def generate_launch_description():
    me, cfg = share('go2_nav_sim'), share('go2_config')
    xacro = f"{os.path.join(me, 'urdf/go2_nav.xacro')} start_x:={START_X} start_y:={START_Y}"
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('champ_bringup'), 'launch/bringup.launch.py')),
        launch_arguments={
            'description_path': xacro,
            'joints_map_path': os.path.join(cfg, 'config/joints/joints.yaml'),
            'links_map_path': os.path.join(cfg, 'config/links/links.yaml'),
            'gait_config_path': LaunchConfiguration('gait'),           # the only change from sim.launch.py
            'use_sim_time': 'true', 'robot_name': 'go2', 'gazebo': 'true', 'lite': 'false', 'rviz': 'false',
            'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
            'hardware_connected': 'false', 'publish_foot_contacts': 'false', 'close_loop_odom': 'true', 'gt_odom': 'true',
        }.items())
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('champ_gazebo'), 'launch/gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'robot_name': 'go2', 'lite': 'false', 'description_path': xacro,
            'world': LaunchConfiguration('world'), 'headless': LaunchConfiguration('headless'),
            'world_init_x': str(START_X), 'world_init_y': str(START_Y), 'world_init_z': '0.275', 'world_init_heading': '0.0',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(STAGE, 'worlds/flat.world')),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('gait', default_value=os.path.join(STAGE, 'params/gait.yaml')),
        bringup, gazebo,
        Node(package='go2_nav_sim', executable='gt_odom.py', parameters=[{'use_sim_time': True}], output='screen'),
    ])
