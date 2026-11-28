"""Nav2 behind a twist_mux and a Collision Monitor. Start the stack with scripts/up.sh, stop its Nav2 with scripts/stop.sh nav, then
  ros2 launch projects/p11_command-arbitration/arbitration.launch.py [monitor:=false] [deadman:=false]
Chain:  teleop/cmd_vel (priority 100) --+
        nav/cmd_vel    (priority 10)  --+--> twist_mux --> cmd_vel_mux --> collision_monitor --> cmd_vel_cm --> dead-man --> cmd_vel
                    e_stop (Bool lock) --+                                                                                  (CHAMP)
monitor:=false leaves out the Collision Monitor, deadman:=false the dead-man (p02's safety_node.py); the links close up.
nav.launch.py itself is included unchanged."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap

HERE = os.path.dirname(os.path.abspath(__file__))
monitor, deadman = LaunchConfiguration('monitor'), LaunchConfiguration('deadman')
SAFETY = os.path.join(os.path.dirname(HERE), 'p02_teleop-and-safety/safety_node.py')


def topic(cond, yes, no):
    """yes if the launch argument cond is 'true', else no: closes the chain when a link is left out."""
    return PythonExpression(["'", yes, "' if '", cond, "' == 'true' else '", no, "'"])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('monitor', default_value='true', description='put the Collision Monitor between twist_mux and CHAMP'),
        DeclareLaunchArgument('deadman', default_value='true', description='p02 safety_node.py as the last link: zero after 0.6 s of silence'),
        DeclareLaunchArgument('nav', default_value='true', description='also start Nav2 + AMCL (nav.launch.py) with its output remapped'),
        # Remaps set here come before each node's own remaps, and ROS uses the first rule that matches. So velocity_smoother's
        # output (cmd_vel_smoothed, which nav2_bringup remaps to cmd_vel) and behavior_server's cmd_vel (spin, back up) both go
        # to the mux. controller_server already publishes cmd_vel_nav, the smoother's input, and is left alone.
        GroupAction([
            SetRemap('cmd_vel_smoothed', 'nav/cmd_vel'),
            SetRemap('behavior_server:cmd_vel', 'nav/cmd_vel'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share('go2_nav_sim'), 'launch/nav.launch.py'))),
        ], condition=IfCondition(LaunchConfiguration('nav'))),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[os.path.join(HERE, 'config/twist_mux.yaml')],
             remappings=[('cmd_vel_out', topic(monitor, 'cmd_vel_mux', topic(deadman, 'cmd_vel_cm', 'cmd_vel')))]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen',
             parameters=[os.path.join(HERE, 'config/collision_monitor.yaml')], condition=IfCondition(monitor),
             remappings=[('cmd_vel', topic(deadman, 'cmd_vel_cm', 'cmd_vel'))]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_monitor', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['collision_monitor']}], condition=IfCondition(monitor)),
        # The lock silences twist_mux, and CHAMP walks on its last command when nothing arrives (p02). A dead-man in the last link
        # turns silence into a stop. a_max is set high so it only times out and does not ramp; v_max is above CHAMP's limits.
        ExecuteProcess(cmd=['python3', SAFETY, '--ros-args', '-r', 'teleop/cmd_vel:=cmd_vel_cm', '-p', 'timeout:=0.6',
                            '-p', 'v_max:=[0.5, 0.5, 1.0]', '-p', 'a_max:=[10.0, 10.0, 20.0]'],
                       output='screen', condition=IfCondition(deadman)),
    ])
