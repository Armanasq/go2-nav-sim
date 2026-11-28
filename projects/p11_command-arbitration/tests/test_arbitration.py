import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import arbitration_check as ac


def test_stop_zone_covers_front_half_and_reaches_past_scan_blind_spot():
    x0, x1, y0, y1 = ac.box_extent(ac.polygon(ac.load('collision_monitor.yaml'), 'PolygonStop'))
    assert x1 > ac.BODY_FRONT and y1 > ac.BODY_HALF_WIDTH and y0 < -ac.BODY_HALF_WIDTH
    assert x0 >= 0.0        # nothing behind the base: an obstacle there blocked Nav2's recoveries (README, Try this 3)
    # /scan drops returns nearer than range_min: a stop edge inside it could never fire
    assert x1 - ac.SCAN_RANGE_MIN >= 0.15


def test_slow_zone_contains_stop_zone():
    cm = ac.load('collision_monitor.yaml')
    s = ac.box_extent(ac.polygon(cm, 'PolygonStop'))
    w = ac.box_extent(ac.polygon(cm, 'PolygonSlow'))
    assert w[0] <= s[0] and w[1] > s[1] and w[2] < s[2] and w[3] > s[3]
    assert 0 < cm['PolygonSlow']['slowdown_ratio'] < 1


def test_mux_order_teleop_over_nav_and_lock_over_all():
    mux = ac.load('twist_mux.yaml')
    t, lock = mux['topics'], mux['locks']['e_stop']
    assert t['teleop']['priority'] == 100 and t['navigation']['priority'] == 10
    assert lock['priority'] > max(v['priority'] for v in t.values())
    assert lock['timeout'] == 0.0


def test_first_after():
    log = [(0.0, 0.1), (0.5, 0.2), (1.0, 1.0), (1.5, 0.2)]
    assert ac.first_after(log, 0.6, lambda v: v > 0.9) == (1.0, 1.0)
    assert ac.first_after(log, 1.1, lambda v: v > 0.9) is None


def test_stop_gap():
    assert abs(ac.stop_gap(1.0, 1.6) - 0.275) < 1e-9
    assert ac.stop_gap(1.5, 1.6) < 0        # body front past the face: contact


def test_is_key_accepts_scaled_key_and_rejects_nav2():
    assert ac.is_key(0.0, 1.0) and ac.is_key(0.0, 0.3)
    assert not ac.is_key(0.075, -0.0449)     # a Nav2 command: walking and steering
    assert not ac.is_key(0.0, 0.5)           # Nav2 turning in place at its limit
