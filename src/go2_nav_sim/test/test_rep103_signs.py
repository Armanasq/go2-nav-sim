"""REP-103 sign test against a running stack: a forward walk raises x, a counter-clockwise turn raises yaw, and only the EKF
publishes odom -> base_footprint.

It drives the robot (1 m forward, then about 2 rad to the left), so it runs only when asked for:

    python3 src/go2_nav_sim/test/test_rep103_signs.py        # stack up (scripts/up.sh), robot with free space ahead

Collected by plain `pytest src/go2_nav_sim/test` it is skipped, so the unit tests never move a robot.
"""
import math
import os
import sys

import pytest

LIVE = os.environ.get('GO2_LIVE_TEST') == '1'
pytestmark = pytest.mark.skipif(not LIVE, reason='drives the robot; run this file directly to enable')

FORWARD, TURN = 0.25, 0.5                          # m/s and rad/s, inside CHAMP's limits (0.3, 0.5)
PLAN = [('settle', 0.0, 0.0, 1.0), ('forward', FORWARD, 0.0, 4.0), ('stop1', 0.0, 0.0, 1.5),
        ('ccw', 0.0, TURN, 4.0), ('stop2', 0.0, 0.0, 1.5)]   # (phase, vx, wz, sim seconds)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@pytest.fixture(scope='module')
def run():
    """Drive PLAN once and record /odom, /odom_gt, /imu/data, every odom -> base_footprint transform on /tf, and every parent
    of base_footprint on /tf and /tf_static."""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('rep103_sign_test')
    d = dict(now=None, phase=None, odom=[], gt=[], imu=[], tf=[], parents=set(), spans={})
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_clock(m):
        d['now'] = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(key):
        def cb(m):
            p, t = m.pose.pose, m.twist.twist
            d[key].append((stamp(m.header), d['phase'], p.position.x, p.position.y, yaw_of(p.orientation), t.linear.x, t.angular.z))
        return cb

    def on_tf(m):
        for tf in m.transforms:
            if tf.child_frame_id == 'base_footprint':
                d['parents'].add(tf.header.frame_id)
                if tf.header.frame_id == 'odom':
                    d['tf'].append((stamp(tf.header), tf.transform.translation.x, tf.transform.translation.y))

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', on_odom('odom'), 50)
    node.create_subscription(Odometry, '/odom_gt', on_odom('gt'), 50)
    node.create_subscription(Imu, '/imu/data', lambda m: d['imu'].append((d['phase'], m.angular_velocity.z)), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', on_tf, 100)
    node.create_subscription(TFMessage, '/tf_static', on_tf, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    t_wait = node.get_clock().now()
    while (d['now'] is None or not d['odom']) and (node.get_clock().now() - t_wait).nanoseconds < 15e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    if d['now'] is None or not d['odom']:
        node.destroy_node(); rclpy.shutdown()
        pytest.skip('no /clock or /odom within 15 s: is the stack up?')
    d['odom_publishers'] = sorted(i.node_name for i in node.get_publishers_info_by_topic('/odom'))
    try:
        for phase, vx, wz, seconds in PLAN:
            t0, d['phase'] = d['now'], phase
            while d['now'] - t0 < seconds:
                cmd = Twist(); cmd.linear.x, cmd.angular.z = vx, wz
                pub.publish(cmd); rclpy.spin_once(node, timeout_sec=0.05)
            d['spans'][phase] = (t0, d['now'])
    finally:                                       # CHAMP keeps walking on the last command: always end with zeros
        for _ in range(10):
            pub.publish(Twist()); rclpy.spin_once(node, timeout_sec=0.05)
    yield d
    node.destroy_node(); rclpy.shutdown()


def phase_rows(rows, phase):
    return [r for r in rows if r[1] == phase]


def test_forward_walk_raises_x(run):
    for key in ('odom', 'gt'):                     # estimator and simulator must agree in sign
        rows = phase_rows(run[key], 'forward')
        (_, _, x0, y0, yaw0, *_), (_, _, x1, y1, *_) = rows[0], rows[-1]
        ahead = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)   # displacement along the start heading
        vx = sum(r[5] for r in rows) / len(rows)
        print(f'\n{key:5s} forward: moved {ahead:+.2f} m along the start heading, mean twist.linear.x {vx:+.2f} m/s')
        assert ahead > 0.3
        if key == 'odom':
            assert vx > 0.1                        # REP-103: odometry twist is in the child frame, x forward


def test_ccw_turn_raises_yaw(run):
    for key in ('odom', 'gt'):
        rows = phase_rows(run[key], 'ccw')
        dyaw = sum(math.remainder(b[4] - a[4], 2 * math.pi) for a, b in zip(rows, rows[1:]))   # unwrapped
        wz = sum(r[6] for r in rows) / len(rows)
        print(f'\n{key:5s} ccw turn: yaw changed {dyaw:+.2f} rad, mean twist.angular.z {wz:+.2f} rad/s')
        assert dyaw > 0.5
        if key == 'odom':
            assert wz > 0.2
    gz = [w for p, w in run['imu'] if p == 'ccw']
    print(f'imu   ccw turn: mean angular_velocity.z {sum(gz) / len(gz):+.2f} rad/s')
    assert sum(gz) / len(gz) > 0.2


def test_only_the_ekf_publishes_odom_to_base_footprint(run):
    odom = {round(r[0], 6): r for r in run['odom']}
    unmatched = [t for t in run['tf'] if round(t[0], 6) not in odom
                 or math.hypot(t[1] - odom[round(t[0], 6)][2], t[2] - odom[round(t[0], 6)][3]) > 1e-3]
    print(f"\nparents of base_footprint: {sorted(run['parents'])}; /odom publishers: {run['odom_publishers']}; "
          f"odom->base_footprint transforms {len(run['tf'])}, not matching an /odom message {len(unmatched)}")
    assert run['parents'] == {'odom'}
    assert run['odom_publishers'] == ['odom_ekf']
    assert len(run['tf']) > 20 and not unmatched


if __name__ == '__main__':
    os.environ['GO2_LIVE_TEST'] = '1'
    sys.exit(pytest.main([__file__, '-q', '-s', '-p', 'no:cacheprovider']))
