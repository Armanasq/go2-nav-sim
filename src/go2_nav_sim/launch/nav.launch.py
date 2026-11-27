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
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(me, 'maps/arena.yaml'), description='map for AMCL / Nav2'),
        DeclareLaunchArgument('mapper', default_value='true', description='run the passive mapper on /slam/map'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('nav2_bringup'), 'launch/bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'params_file': os.path.join(me, 'config/nav2_params.yaml'),
                              'use_sim_time': 'true', 'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(me, 'config/slam.yaml')], remappings=[('/map', '/slam/map'), ('/map_metadata', '/slam/map_metadata')],
             condition=IfCondition(LaunchConfiguration('mapper'))),
    ])
