"""Smoke test of the stack: headless Gazebo + estimation + map_server/AMCL start, then one test checks that /scan, /imu/data
and /odom arrive within DEADLINE seconds at their configured rates, and that map -> base_footprint can be looked up.
usage:  launch_test projects/p14_test-the-stack/smoke.launch.py                     (about 1 to 2 minutes)
        launch_test projects/p14_test-the-stack/smoke.launch.py estimation:=false   (a broken stack: the test must fail)
The rules are in smoke_checks.py. Each run writes its measurements to results/last_run.json."""
import json, os, pathlib, sys, time, unittest
import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import smoke_checks as sc  # noqa: E402

TOPICS = {'/scan': LaserScan, '/imu/data': Imu, '/odom': Odometry}


def include(pkg, path, args=None, **kw):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(pkg), path)),
                                    launch_arguments=(args or {}).items(), **kw)


@pytest.mark.launch_test
def generate_test_description():
    me = share('go2_nav_sim')
    return LaunchDescription([
        DeclareLaunchArgument('estimation', default_value='true', description='false = leave out scan, lidar odometry and EKF'),
        include('go2_nav_sim', 'launch/sim.launch.py'),                                   # headless by default
        include('go2_nav_sim', 'launch/estimation.launch.py', condition=IfCondition(LaunchConfiguration('estimation'))),
        include('nav2_bringup', 'launch/localization_launch.py',                         # map_server + AMCL only, not all of Nav2
                {'map': os.path.join(me, 'maps/arena.yaml'), 'params_file': os.path.join(me, 'config/nav2_params.yaml'),
                 'use_sim_time': 'true', 'autostart': 'true', 'use_composition': 'False'}),
        launch_testing.actions.ReadyToTest(),
    ])


class TestStackComesUp(unittest.TestCase):
    def test_sensors_odometry_and_tf(self):
        rclpy.init(args=[])                                   # not sys.argv: launch_test passes the launch arguments there
        node = rclpy.create_node('smoke_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        t0, first, stamps, tf_seen = time.monotonic(), {}, {t: [] for t in TOPICS}, None

        def on_msg(topic):
            def cb(m):
                first.setdefault(topic, time.monotonic() - t0)
                stamps[topic].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            return cb
        subs = [node.create_subscription(T, t, on_msg(t), qos_profile_sensor_data) for t, T in TOPICS.items()]  # noqa: F841
        tf = Buffer(); TransformListener(tf, node)

        def enough(t):                                       # WINDOW seconds of simulated time collected for topic t
            s = stamps[t]
            return len(s) > 1 and s[-1] - s[0] >= sc.WINDOW
        while time.monotonic() - t0 < sc.DEADLINE + 30:     # +30 s: time to measure a rate for a topic that started late
            rclpy.spin_once(node, timeout_sec=0.1)
            if tf_seen is None and tf.can_transform('map', 'base_footprint', Time()):
                tf_seen = time.monotonic() - t0
            if tf_seen is not None and all(enough(t) for t in TOPICS):
                break
            if time.monotonic() - t0 > sc.DEADLINE and (tf_seen is None or any(t not in first for t in TOPICS)):
                break                                        # something never came: no point waiting longer

        rows = [(t, first.get(t), sc.rate(stamps[t]), sc.EXPECTED[t]) for t in TOPICS]
        print('\nsmoke: topic          first msg (s)   rate (Hz)   expected (Hz)')
        for t, f, r, e in rows:
            print(f'smoke: {t:14s} {"never" if f is None else f"{f:13.1f}"}   {r:9.1f}   {e:13.0f}')
        print(f'smoke: map -> base_footprint  {"never" if tf_seen is None else f"{tf_seen:.1f} s"}')
        (HERE / 'results').mkdir(exist_ok=True)
        (HERE / 'results' / 'last_run.json').write_text(json.dumps(
            {'first_msg_s': first, 'rate_hz': {t: r for t, _, r, _ in rows}, 'tf_s': tf_seen}, indent=1))
        node.destroy_node(); rclpy.shutdown()
        fails = sc.check(first, stamps, tf_seen)
        self.assertEqual(fails, [], 'smoke test failed:\n  ' + '\n  '.join(fails))
