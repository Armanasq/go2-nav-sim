"""Chapter 5 exercise: a small Go2 sensor tree with two faults. Find them with tf2_echo, tf2_monitor and view_frames.
Run it with the stack stopped (the frame names are the same as the robot's):
    ros2 launch docs/exercises/broken_tree.launch.py
Expected tree, from the robot's URDF:
    base_footprint -> base_link -> lidar_link, camera_link -> camera_optical, trunk -> imu_link
Do not read further than this docstring before you have found both faults."""
from launch import LaunchDescription
from launch_ros.actions import Node


def static(name, parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name, arguments=[
        '--x', str(x), '--y', str(y), '--z', str(z), '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
        '--frame-id', parent, '--child-frame-id', child])


def generate_launch_description():
    return LaunchDescription([
        static('footprint_to_base', 'base_footprint', 'base_link', z=0.217),
        static('base_to_lidar', 'base_link', 'lidar_link', z=0.1),
        static('base_to_camera', 'base_link', 'camera_link', x=0.28, z=0.03),
        static('camera_to_optical', 'camera_link', 'camera_optical', roll=-1.5707963, yaw=1.5707963),   # fault 1: yaw sign
        static('trunk_to_imu', 'trunk', 'imu_link'),                     # fault 2: nothing publishes base_link -> trunk
    ])
