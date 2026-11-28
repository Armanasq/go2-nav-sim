"""Nav2 + AMCL on the arena map (the map is a launch argument), plus a passive slam_toolbox that builds its own map on /slam/map.
Needs sim.launch.py and estimation.launch.py (odom->base_footprint, /scan)."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    me = share('go2_nav_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(me, 'maps/arena.yaml'), description='map for AMCL / Nav2'),
        DeclareLaunchArgument('mapper', default_value='true', description='run the passive mapper on /slam/map'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(me, 'config/nav2_params.yaml'),
                              description='Nav2 + AMCL parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='take time from /clock (Gazebo or a bag)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('nav2_bringup'), 'launch/bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': use_sim_time, 'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(me, 'config/slam.yaml'), {'use_sim_time': use_sim_time}],
             remappings=[('/map', '/slam/map'), ('/map_metadata', '/slam/map_metadata')],
             condition=IfCondition(LaunchConfiguration('mapper'))),
    ])
