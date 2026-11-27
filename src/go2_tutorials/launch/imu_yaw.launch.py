from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='go2_tutorials', executable='imu_yaw', name='imu_yaw', output='screen',
             parameters=[{'use_sim_time': True, 'publish_rate': 10.0}]),
    ])
