"""REP-103 sign test: a forward walk raises x, a counter-clockwise turn raises yaw, and only the EKF publishes
odom -> base_footprint.

Two sources, the same three tests:

    python3 src/go2_nav_sim/test/test_rep103_signs.py              # live: drives the running stack (scripts/up.sh),
                                                                    # 1 m forward then about 2 rad left; needs free space
    python3 src/go2_nav_sim/test/test_rep103_signs.py bags/signs   # offline: any bag with /cmd_vel /odom /odom_gt
                                                                    # /imu/data /tf /tf_static

In a bag, each message gets the phase of the last /cmd_vel recorded before it (receive time): forward when only x is
commanded, turn when only z is. Collected by plain `pytest src/go2_nav_sim/test` it is skipped, so the unit tests never
move a robot.
"""
import math
import os
import sys

import pytest

LIVE = os.environ.get('GO2_LIVE_TEST') == '1'
BAG = os.environ.get('GO2_SIGN_BAG')
pytestmark = pytest.mark.skipif(not (LIVE or BAG), reason='drives the robot or reads a bag; run this file directly')

FORWARD, TURN = 0.25, 0.5                          # m/s and rad/s, inside CHAMP's limits (0.3, 0.5)
PLAN = [(0.0, 0.0, 1.0), (FORWARD, 0.0, 4.0), (0.0, 0.0, 1.5), (0.0, TURN, 4.0), (0.0, 0.0, 1.5)]   # (vx, wz, sim s)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def phase_of(vx, wz):
    return 'forward' if vx > 0.05 and abs(wz) < 0.05 else 'turn' if abs(wz) > 0.05 and abs(vx) < 0.05 else None


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder:
    """Rows per topic. Odometry rows are (stamp, phase, x, y, yaw, forward velocity in the body, yaw rate)."""

    def __init__(self):
        self.phase, self.odom, self.gt, self.imu, self.tf, self.parents = None, [], [], [], [], set()
        self.odom_publishers = None                # known only live

    def odometry(self, key, m):
        p, t = m.pose.pose, m.twist.twist
        yaw = yaw_of(p.orientation)
        if key == 'gt':                            # gazebo_ros_p3d publishes the twist in world: rotate it into the body
            vx = t.linear.x * math.cos(yaw) + t.linear.y * math.sin(yaw)
        else:                                      # REP-105: odometry twist is in the child frame
            vx = t.linear.x
        getattr(self, key).append((stamp(m.header), self.phase, p.position.x, p.position.y, yaw, vx, t.angular.z))

    def transforms(self, m):
        for tf in m.transforms:
            if tf.child_frame_id == 'base_footprint':
                self.parents.add(tf.header.frame_id)
                if tf.header.frame_id == 'odom':
                    r = tf.transform
                    self.tf.append((stamp(tf.header), r.translation.x, r.translation.y, yaw_of(r.rotation)))

    def handle(self, topic, m):
        if topic == '/cmd_vel':
            self.phase = phase_of(m.linear.x, m.angular.z)
        elif topic in ('/odom', '/odom_gt'):
            self.odometry('odom' if topic == '/odom' else 'gt', m)
        elif topic == '/imu/data':
            self.imu.append((self.phase, m.angular_velocity.z))
        elif topic in ('/tf', '/tf_static'):
            self.transforms(m)


TOPICS = ('/cmd_vel', '/odom', '/odom_gt', '/imu/data', '/tf', '/tf_static')


def from_bag(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in types]
    assert not missing, f'{uri} lacks {missing}'
    r.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    rec = Recorder()
    while r.has_next():                            # messages come in receive order, so /cmd_vel sets the phase of what follows
        topic, data, _ = r.read_next()
        rec.handle(topic, deserialize_message(data, types[topic]))
    return rec


def from_live():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('rep103_sign_test')
    rec, now = Recorder(), [None]
    node.create_subscription(Clock, '/clock', lambda m: now.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', lambda m: rec.handle('/odom', m), 50)
    node.create_subscription(Odometry, '/odom_gt', lambda m: rec.handle('/odom_gt', m), 50)
    node.create_subscription(Imu, '/imu/data', lambda m: rec.handle('/imu/data', m), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', lambda m: rec.handle('/tf', m), 100)
    node.create_subscription(TFMessage, '/tf_static', lambda m: rec.handle('/tf_static', m),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    t_wait = node.get_clock().now()
    while (now[0] is None or not rec.odom) and (node.get_clock().now() - t_wait).nanoseconds < 15e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    if now[0] is None or not rec.odom:
        node.destroy_node(); rclpy.shutdown()
        pytest.skip('no /clock or /odom within 15 s: is the stack up?')
    rec.odom_publishers = sorted(i.node_name for i in node.get_publishers_info_by_topic('/odom'))
    try:
        for vx, wz, seconds in PLAN:
            t0 = now[0]
            cmd = Twist(); cmd.linear.x, cmd.angular.z = vx, wz
            rec.handle('/cmd_vel', cmd)
            while now[0] - t0 < seconds:
                pub.publish(cmd); rclpy.spin_once(node, timeout_sec=0.05)
    finally:                                       # CHAMP keeps walking on the last command: always end with zeros
        for _ in range(10):
            pub.publish(Twist()); rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node(); rclpy.shutdown()
    return rec


@pytest.fixture(scope='module')
def run():
    return from_bag(BAG) if BAG else from_live()


def steps(rows, phase):
    """Consecutive pairs that both lie in the phase, so motion between two separate segments is never counted."""
    return [(a, b) for a, b in zip(rows, rows[1:]) if a[1] == b[1] == phase]


def test_forward_walk_raises_x(run):
    for key in ('odom', 'gt'):                     # estimator and simulator must agree in sign
        pairs = steps(getattr(run, key), 'forward')
        assert pairs, f'no forward phase on {key}'
        ahead = sum((b[2] - a[2]) * math.cos(a[4]) + (b[3] - a[3]) * math.sin(a[4]) for a, b in pairs)   # along the heading
        vx = sum(a[5] for a, _ in pairs) / len(pairs)
        print(f'\n{key:5s} forward: moved {ahead:+.2f} m along its heading, mean body forward velocity {vx:+.2f} m/s')
        assert ahead > 0.3
        assert vx > 0.1


def test_ccw_turn_raises_yaw(run):
    for key in ('odom', 'gt'):
        pairs = steps(getattr(run, key), 'turn')
        assert pairs, f'no turn phase on {key}'
        dyaw = sum(math.remainder(b[4] - a[4], 2 * math.pi) for a, b in pairs)    # unwrapped
        wz = sum(a[6] for a, _ in pairs) / len(pairs)
        print(f'\n{key:5s} ccw turn: yaw changed {dyaw:+.2f} rad, mean twist.angular.z {wz:+.2f} rad/s')
        assert dyaw > 0.5
        assert wz > 0.2
    gz = [w for p, w in run.imu if p == 'turn']
    print(f'imu   ccw turn: mean angular_velocity.z {sum(gz) / len(gz):+.2f} rad/s')
    assert sum(gz) / len(gz) > 0.2


def test_only_the_ekf_publishes_odom_to_base_footprint(run):
    odom = {round(r[0], 6): r for r in run.odom}
    last = max(odom)                               # a transform can arrive before the /odom message with its stamp
    tf = [t for t in run.tf if t[0] <= last]
    unmatched = [t for t in tf if round(t[0], 6) not in odom
                 or math.hypot(t[1] - odom[round(t[0], 6)][2], t[2] - odom[round(t[0], 6)][3]) > 1e-3
                 or abs(math.remainder(t[3] - odom[round(t[0], 6)][4], 2 * math.pi)) > 1e-3]
    print(f"\nparents of base_footprint: {sorted(run.parents)}; /odom publishers: {run.odom_publishers or 'not in a bag'}; "
          f"odom->base_footprint transforms {len(tf)}, not matching an /odom pose {len(unmatched)}")
    assert run.parents == {'odom'}
    if run.odom_publishers is not None:
        assert run.odom_publishers == ['odom_ekf']
    assert len(tf) > 20 and not unmatched


if __name__ == '__main__':
    if len(sys.argv) > 1:
        os.environ['GO2_SIGN_BAG'] = os.path.abspath(sys.argv[1])
    else:
        os.environ['GO2_LIVE_TEST'] = '1'
    sys.exit(pytest.main([__file__, '-q', '-s', '-p', 'no:cacheprovider']))
