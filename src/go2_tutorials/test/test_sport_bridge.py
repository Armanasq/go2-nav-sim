import math

from go2_tutorials.sport_bridge import decide


def test_clamps_each_axis():
    assert decide((2.0, -1.0, 3.0), 0.0, False) == ('move', (0.3, -0.25, 0.5))
    assert decide((0.2, 0.1, -0.4), 0.1, True) == ('move', (0.2, 0.1, -0.4))


def test_timeout_sends_one_stop():
    assert decide((0.3, 0.0, 0.0), 0.6, False) == ('stop', None)
    assert decide((0.3, 0.0, 0.0), 0.6, True) == (None, None)     # already stopped: send nothing


def test_no_command_yet_sends_nothing():
    assert decide(None, 0.0, True) == (None, None)


def test_non_finite_stops():
    assert decide((math.nan, 0.0, 0.0), 0.0, False) == ('stop', None)
    assert decide((0.0, 0.0, math.inf), 0.0, False) == ('stop', None)
