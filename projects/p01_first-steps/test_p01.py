import math, sys, pathlib
from types import SimpleNamespace as Q

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from measure import yaw_of, wrap, summarize
from drive import parse, command_at, DEFAULT


def quat(yaw):
    return Q(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def test_yaw_of_round_trips():
    for deg in (-179, -90, 0, 45, 179):
        assert math.isclose(math.degrees(yaw_of(quat(math.radians(deg)))), deg, abs_tol=1e-9)


def test_wrap():
    assert math.isclose(wrap(math.radians(190)), math.radians(-170))
    assert math.isclose(wrap(math.radians(-190)), math.radians(170))


def test_straight_line():
    s = summarize([(t * 0.1, 0.03 * t, 0.0, 0.0) for t in range(101)])   # 0.3 m/s for 10 s
    assert math.isclose(s['duration'], 10.0) and math.isclose(s['path'], 3.0) and math.isclose(s['speed'], 0.3)
    assert math.isclose(s['displacement'], 3.0) and s['turn_deg'] == 0.0


def test_turn_counts_past_180():
    # 0.5 rad/s for 10 s in place = 286.5 deg; the stored yaw wraps through +-180 on the way
    s = summarize([(t * 0.02, 0.0, 0.0, wrap(0.5 * t * 0.02)) for t in range(501)])
    assert math.isclose(s['turn_deg'], math.degrees(5.0), rel_tol=1e-9)
    assert s['path'] == 0.0 and math.isclose(s['yaw_deg'], math.degrees(wrap(5.0)))


def test_summarize_needs_two_samples():
    assert summarize([]) is None and summarize([(0, 0, 0, 0)]) is None


def test_parse_and_sequence():
    segs = parse(['0.3,0,0,10', '0,0,0.5,3.2'])
    assert segs == [(0.3, 0.0, 0.0, 10.0), (0.0, 0.0, 0.5, 3.2)]
    assert command_at(segs, 0.0) == (0.3, 0.0, 0.0)
    assert command_at(segs, 10.1) == (0.0, 0.0, 0.5)
    assert command_at(segs, 13.3) is None
    assert command_at(DEFAULT, 16.3) is None and command_at(DEFAULT, 16.1) == (0.3, 0.0, 0.0)


def test_parse_rejects_bad_segments():
    for bad in ('0.3,0,10', '0.3,0,0,-1', 'a,b,c,d'):
        try:
            parse([bad])
        except ValueError:
            continue
        raise AssertionError(bad)
