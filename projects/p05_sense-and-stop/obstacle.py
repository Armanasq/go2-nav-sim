"""Pure logic for project 5, no ROS: nearest obstacle in a scan sector, depth at the image centre, stop decision."""
import math

import numpy as np

FRONT = 0.28      # m from base_link to the camera at the front of the head (urdf/go2_nav.xacro: rgbd_camera xyz="0.28 0 0.03")


def front_min(ranges, angle_min, angle_inc, half_angle, range_min, range_max):
    """Nearest valid return with |bearing| <= half_angle. Returns (range, bearing) or (inf, nan) if the sector is empty.
    Invalid returns (inf, nan, below range_min, above range_max) are skipped: inf means 'nothing seen', not 'far away'."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = (np.abs(a) <= half_angle) & np.isfinite(r) & (r >= range_min) & (r <= range_max)
    if not ok.any():
        return math.inf, math.nan
    i = np.flatnonzero(ok)[np.argmin(r[ok])]
    return float(r[i]), float(a[i])


def centre_depth(depth, half=2):
    """Median depth (m) of the (2*half+1)^2 pixel patch at the image centre. The median of a patch, not one pixel, so one
    noisy or missing pixel does not decide. A pixel with nothing within the far clip is +inf ('far'), and it stays in the
    median: dropping it would let the few pixels that do hit something decide. Only nan pixels are ignored."""
    h, w = depth.shape
    p = depth[h // 2 - half:h // 2 + half + 1, w // 2 - half:w // 2 + half + 1]
    p = p[~np.isnan(p)]
    return float(np.median(p)) if p.size else math.inf


def should_stop(front_range, stop_gap, front_offset):
    """True when the gap between the robot's front and the obstacle is at or below stop_gap.
    front_range is measured from base_footprint (the /scan frame); the front of the head is front_offset ahead of it."""
    return front_range - front_offset <= stop_gap

