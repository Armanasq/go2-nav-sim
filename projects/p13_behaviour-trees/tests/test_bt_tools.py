"""ROS-free checks for bt_tools.py and the three trees.   /usr/bin/python3 -m pytest -q projects/p13_behaviour-trees"""
import math, pathlib, sys
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import bt_tools  # noqa: E402

T = HERE / 'trees'


def test_swap_changes_only_the_recovery_order():
    assert bt_tools.recovery_order(T / 'default.xml') == ['ClearingActions', 'Spin', 'Wait', 'BackUp']
    assert bt_tools.recovery_order(T / 'recovery_swapped.xml') == ['ClearingActions', 'BackUp', 'Wait', 'Spin']
    dot_a, n_a = bt_tools.to_dot(T / 'default.xml')
    dot_b, n_b = bt_tools.to_dot(T / 'recovery_swapped.xml')
    assert n_a == n_b == 18 and dot_a.count('->') == 17                 # a tree: one edge per node but the root
    labels = lambda dot: sorted(l.split(' ', 3)[-1] for l in dot.splitlines() if 'label=' in l)
    assert labels(dot_a) == labels(dot_b)                               # same nodes, only their order differs


def test_patrol_waypoints_face_along_the_leg_that_reaches_them():
    wp = bt_tools.waypoints(T / 'patrol.xml')
    assert len(wp) == 3
    prev = wp[-1]                                                       # the patrol is a loop: W1 is reached from home
    for x, y, yaw in wp:
        leg = math.atan2(y - prev[1], x - prev[0])
        assert abs(math.remainder(yaw - leg, 2 * math.pi)) < math.radians(1.0), (x, y)
        prev = (x, y, yaw)


def test_started_lists_each_recovery_once_per_start():
    log = ['    0.30  Spin                         IDLE     -> RUNNING',
           '    4.00  Spin                         RUNNING  -> SUCCESS',
           '    4.00  Wait                         IDLE     -> RUNNING',
           '    9.10  ComputePathToPose            IDLE     -> RUNNING',
           '    9.20  ClearLocalCostmap-Subtree    IDLE     -> SUCCESS',
           'bt_log: stopped']
    events = bt_tools.parse_log(log)
    assert bt_tools.started(events) == [(0.3, 'Spin'), (4.0, 'Wait'), (9.2, 'ClearLocalCostmap-Subtree')]
    assert bt_tools.intervals(events) == [('Spin', 0.3, 4.0)]            # Wait never finished: no interval
