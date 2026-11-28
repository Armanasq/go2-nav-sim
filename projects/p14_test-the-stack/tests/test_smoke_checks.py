"""ROS-free checks of the smoke test's rules.   /usr/bin/python3 -m pytest -q projects/p14_test-the-stack/tests"""
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from smoke_checks import check, rate  # noqa: E402


def ticks(hz, seconds, t0=100.0):
    return [t0 + i / hz for i in range(int(hz * seconds) + 1)]


def test_rate_counts_intervals_not_messages():
    assert rate(ticks(10, 5)) == 10.0                  # 51 messages span 5 s: 50 intervals
    assert rate([]) == rate([3.0]) == rate([3.0, 3.0]) == 0.0


def test_healthy_stack_passes():
    stamps = {'/scan': ticks(10, 5), '/imu/data': ticks(100, 5), '/odom': ticks(50, 5)}
    first = {'/scan': 30.0, '/imu/data': 25.0, '/odom': 40.0}
    assert check(first, stamps, tf_seen=45.0) == []


def test_each_kind_of_failure_is_reported():
    stamps = {'/scan': ticks(10, 5), '/imu/data': ticks(60, 5)}
    first = {'/scan': 130.0, '/imu/data': 25.0}                 # scan too late, IMU too slow, odom never
    assert check(first, stamps, tf_seen=None) == [
        '/scan: no message within 120 s',
        '/imu/data: 60.0 Hz, expected 100 Hz +- 10%',
        '/odom: no message within 120 s',
        'map -> base_footprint: no transform within 120 s']
