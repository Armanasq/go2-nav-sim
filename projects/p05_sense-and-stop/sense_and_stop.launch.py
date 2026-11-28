"""sense.py and stop_at_obstacle.py together. The launch ends when the robot has stopped.
usage: ros2 launch projects/p05_sense-and-stop/sense_and_stop.launch.py [speed:=0.3] [gap:=0.5]"""
import os, sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    sense = ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, 'sense.py')], output='screen')
    stop = ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, 'stop_at_obstacle.py'), '--ros-args',
                               '-p', ['speed:=', LaunchConfiguration('speed')], '-p', ['gap:=', LaunchConfiguration('gap')]],
                          output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.3'), DeclareLaunchArgument('gap', default_value='0.5'),
        sense, stop,
        RegisterEventHandler(OnProcessExit(target_action=stop, on_exit=[EmitEvent(event=Shutdown())])),   # robot stopped: end
    ])
