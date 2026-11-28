"""Scoring helpers for project 12, no ROS. Grids are indexed [iy, ix] with row 0 at the map origin (bottom), like a Nav2
costmap; a ROS map image (row 0 at the top) must be flipped first (load_map does this)."""
import math, pathlib, re
import numpy as np
from scipy.ndimage import distance_transform_edt

LETHAL, INSCRIBED = 254, 253


def inflation_cost(d, inscribed, radius, scale):
    """Nav2 InflationLayer::computeCost for a cell d metres from the nearest obstacle cell (Humble, inflation_layer.hpp)."""
    if d <= 0.0:
        return LETHAL
    if d <= inscribed:
        return INSCRIBED
    if d > radius:
        return 0
    return int((INSCRIBED - 1) * math.exp(-scale * (d - inscribed)))


def load_map(yaml_path):
    """Occupancy (True = obstacle) flipped so row 0 is y = origin_y, plus origin (x, y) and resolution."""
    import yaml
    from PIL import Image
    y = yaml.safe_load(open(yaml_path))
    img = np.array(Image.open(pathlib.Path(yaml_path).parent / y['image']))
    occ = (255 - img) / 255.0 > y['occupied_thresh']
    return occ[::-1], tuple(y['origin'][:2]), y['resolution']


def distance_grid(occ, res):
    """Metres from each cell centre to the nearest obstacle cell centre (0 on obstacles)."""
    return distance_transform_edt(~occ) * res


def lookup(grid, origin, res, xy):
    """grid value at the cell under each (x, y); points outside the grid get 0 (as if on an obstacle)."""
    xy = np.atleast_2d(xy)
    ix = np.floor((xy[:, 0] - origin[0]) / res).astype(int); iy = np.floor((xy[:, 1] - origin[1]) / res).astype(int)
    inside = (ix >= 0) & (iy >= 0) & (ix < grid.shape[1]) & (iy < grid.shape[0])
    out = np.zeros(len(xy), grid.dtype)
    out[inside] = grid[iy[inside], ix[inside]]
    return out


def perimeter(polygon, step=0.025):
    """Points every `step` metres along a closed polygon, in the robot frame."""
    P = np.asarray(polygon, float); pts = []
    for a, b in zip(P, np.roll(P, -1, 0)):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        pts += [a + (b - a) * k / n for k in range(n)]
    return np.array(pts)


def body_clearance(poses, polygon, dist, origin, res):
    """Smallest distance from the body outline to an obstacle over a trajectory of (x, y, yaw) poses."""
    edge = perimeter(polygon); best = math.inf
    for x, y, th in poses:
        c, s = math.cos(th), math.sin(th)
        w = edge @ np.array([[c, s], [-s, c]]) + (x, y)
        best = min(best, float(lookup(dist, origin, res, w).min()))
    return best


def path_length(xy):
    xy = np.asarray(xy, float)
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0


def parse_mission(text):
    """Outcome, navigation time (sim s) and recoveries from a go_to_goal.py log."""
    m = re.search(r'RESULT (\w+)\s+sim_time ([\d.]+) s', text)
    rec = re.findall(r'recoveries (\d+)', text)
    if m:
        return dict(result=m.group(1), nav_time_s=float(m.group(2)), recoveries=int(rec[-1]) if rec else 0)
    return dict(result='TIMEOUT' if 'RESULT TIMEOUT' in text else 'NO_RESULT', nav_time_s=None,
                recoveries=int(rec[-1]) if rec else 0)
