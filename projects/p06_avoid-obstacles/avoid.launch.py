"""One trial: a fresh estimation stack (scan, lidar odometry, EKF) and avoid.py. Ends when avoid.py exits.
usage: ros2 launch projects/p06_avoid-obstacles/avoid.launch.py [gx:=4.0] [gy:=3.0] [method:=vfh] [name:=run]
Run it with only sim.launch.py up. The estimator restarts on every trial because its yaw drifts while the robot stands
(If it goes wrong, in the README); avoid.py starts 10 s later, once the odometry is publishing."""
import os, sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    est = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('go2_nav_sim'), 'launch', 'estimation.launch.py')))
    avoid = ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, 'avoid.py'), '--goal', C('gx'), C('gy'),
                                '--method', C('method'), '--name', C('name')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('gx', default_value='4.0'), DeclareLaunchArgument('gy', default_value='3.0'),
        DeclareLaunchArgument('method', default_value='vfh'), DeclareLaunchArgument('name', default_value='run'),
        est, TimerAction(period=10.0, actions=[avoid]),
        RegisterEventHandler(OnProcessExit(target_action=avoid, on_exit=[EmitEvent(event=Shutdown())])),
    ])
