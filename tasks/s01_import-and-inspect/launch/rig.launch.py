"""Fixed-base rig: the Go2 hanging from the world in an empty Gazebo world, both controllers active, no CHAMP.
    ros2 launch tasks/s01_import-and-inspect/launch/rig.launch.py urdf:=tasks/s01_import-and-inspect/urdf/go2_rig.urdf
The URDF must have a link named world fixed to base_link (run.py 1.06 writes one). Send joint targets to
/joint_group_effort_controller/joint_trajectory (all 12 joints in every message)."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def nodes(context):
    urdf = open(LaunchConfiguration('urdf').perform(context)).read()
    sim = {'use_sim_time': True}
    spawner = lambda c: Node(package='controller_manager', executable='spawner', output='screen',
                             arguments=[c, '--controller-manager', '/controller_manager'])
    return [
        ExecuteProcess(cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                            LaunchConfiguration('world').perform(context), '--ros-args', '--params-file',
                            os.path.join(share('champ_gazebo'), 'config/gazebo.yaml')], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, sim]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'go2', '-topic', '/robot_description', '-z', '0.6']),
        spawner('joint_states_controller'), spawner('joint_group_effort_controller'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', description='expanded URDF with a world link'),
        DeclareLaunchArgument('world', default_value=os.path.join(HERE, 'worlds/empty.world')),
        OpaqueFunction(function=nodes),
    ])
