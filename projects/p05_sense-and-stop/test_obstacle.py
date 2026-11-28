import math

import numpy as np

from obstacle import centre_depth, front_min, should_stop

INC = math.radians(1.0)


def scan(n=360, fill=10.0):
    """A scan from -180 deg in 1 deg steps; index 180 is straight ahead."""
    return np.full(n, fill), -math.pi


def test_nearest_in_sector():
    r, a0 = scan()
    r[180 + 5] = 2.0            # +5 deg, inside +-20
    r[180 - 10] = 1.5           # -10 deg, inside, nearer
    r[180 + 60] = 0.5           # +60 deg, outside the sector
    d, b = front_min(r, a0, INC, math.radians(20), 0.4, 15.0)
    assert d == 1.5 and math.isclose(math.degrees(b), -10, abs_tol=1e-6)


def test_invalid_returns_are_skipped():
    r, a0 = scan()
    r[180] = math.inf; r[181] = math.nan; r[182] = 0.2; r[183] = 30.0   # no return, nan, below range_min, above range_max
    d, _ = front_min(r, a0, INC, math.radians(20), 0.4, 15.0)
    assert d == 10.0


def test_empty_sector():
    r, a0 = scan(fill=math.inf)
    d, b = front_min(r, a0, INC, math.radians(20), 0.4, 15.0)
    assert d == math.inf and math.isnan(b)


def test_centre_depth_median_ignores_nan():
    img = np.full((360, 640), 3.0, np.float32)
    img[180, 320] = 0.1          # one bad pixel in the 5x5 patch
    img[178:181, 318:320] = np.nan
    assert centre_depth(img) == 3.0


def test_centre_depth_no_return():
    assert centre_depth(np.full((360, 640), np.nan, np.float32)) == math.inf
    img = np.full((360, 640), np.inf, np.float32)   # nothing within the far clip ...
    img[178:183, 318] = 7.2                          # ... except 5 of the 25 patch pixels, the edge of an object
    assert centre_depth(img) == math.inf


def test_should_stop_uses_front_offset():
    assert not should_stop(0.79, 0.5, 0.28)
    assert should_stop(0.78, 0.5, 0.28)
    assert not should_stop(math.inf, 0.5, 0.28)
