"""Compare a SLAM occupancy grid with the true arena grid. Numpy/scipy only, no ROS, so it can be unit-tested.

A grid here is (cells, res, origin): cells is an int8 array indexed [row, col] with row 0 at the *bottom* (min y), values
-1 unknown, 0 free, 1 occupied; origin is the world (x, y) of the lower-left corner of cell [0, 0].

The SLAM map only sees the faces of obstacles that face the robot: box interiors and the outside of the walls stay unknown.
So its occupied cells are compared with the *visible outline* of the true map: true occupied cells that touch the free space
the robot can reach from marker A."""
import math, pathlib, sys

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'src/go2_nav_sim/scripts'))
import registration  # noqa: E402   the point-to-line Gauss-Newton fit that lidar_odom.py uses

UNKNOWN, FREE, OCC = -1, 0, 1


def load(yaml_path):
    """map_server .yaml + .pgm -> grid. Reads the trinary values map_saver writes (0 occupied, 254 free, 205 unknown)."""
    yaml_path = pathlib.Path(yaml_path)
    meta = dict(line.split(':', 1) for line in yaml_path.read_text().splitlines() if ':' in line)
    raw = (yaml_path.parent / meta['image'].strip()).read_bytes()
    magic, w, h, maxval, data = raw.split(maxsplit=4)
    assert magic == b'P5' and int(maxval) == 255, 'expected an 8-bit binary PGM'
    img = np.frombuffer(data, np.uint8)[:int(w) * int(h)].reshape(int(h), int(w))[::-1]     # PGM row 0 is the top
    cells = np.full(img.shape, UNKNOWN, np.int8)
    cells[img < 50], cells[img > 250] = OCC, FREE
    origin = [float(v) for v in meta['origin'].strip(' []\n').split(',')[:2]]
    return cells, float(meta['resolution']), np.array(origin)


def centres(grid, mask):
    """World (x, y) of the centres of the cells where mask is true, Nx2."""
    _, res, origin = grid
    r, c = np.nonzero(mask)
    return origin + (np.column_stack([c, r]) + 0.5) * res


def visible_outline(grid, seed):
    """True occupied cells that touch (8-neighbourhood) the free region connected to the world point seed."""
    cells, res, origin = grid
    lab, _ = ndimage.label(cells == FREE)
    c, r = ((np.asarray(seed) - origin) / res).astype(int)
    reach = lab == lab[r, c]
    return (cells == OCC) & ndimage.binary_dilation(reach, np.ones((3, 3), bool)), reach


def transform(pts, pose):
    x, y, th = pose
    return pts @ registration.rot(th).T + [x, y]


def align(slam, truth_outline_pts, guess):
    """Rigid SE(2) fit of the SLAM map's occupied cells onto the true outline: pose (x, y, yaw) of the SLAM map frame in
    the world, the number of matched cells and the rms point-to-line residual (m)."""
    lmap = registration.LineMap(truth_outline_pts)
    pose, matches, rms, ok, _ = registration.register(centres(slam, slam[0] == OCC), lmap, guess, sigma=0.05,
                                                      trans_prior=0.0, fix_yaw=False, iters=40)
    assert ok, 'alignment failed: too few occupied cells near the true outline'
    return pose, matches, rms


def resample(slam, pose, like):
    """The SLAM grid placed in the world at pose, sampled (nearest cell) on the cells of grid like. Outside -> unknown."""
    cells, res, origin = slam
    H, W = like[0].shape
    r, c = np.mgrid[0:H, 0:W]
    world = like[2] + (np.stack([c, r], -1).reshape(-1, 2) + 0.5) * like[1]
    local = (world - pose[:2]) @ registration.rot(pose[2])           # world -> SLAM map frame (inverse rotation)
    cc, rr = np.floor((local - origin) / res).astype(int).T
    inside = (rr >= 0) & (rr < cells.shape[0]) & (cc >= 0) & (cc < cells.shape[1])
    out = np.full(H * W, UNKNOWN, np.int8)
    out[inside] = cells[rr[inside], cc[inside]]
    return out.reshape(H, W)


def scores(slam_on_truth, truth, seed):
    """Metrics of a SLAM grid already resampled on the true grid's cells (resample()). Distances in metres."""
    outline, reach = visible_outline(truth, seed)
    s_occ, s_free, t_occ = slam_on_truth == OCC, slam_on_truth == FREE, truth[0] == OCC
    near = lambda m: ndimage.binary_dilation(m, np.ones((3, 3), bool))           # within one cell (5 cm), diagonals too
    known = s_occ | s_free
    d, _ = cKDTree(centres(truth, outline)).query(centres(truth, s_occ))
    return {
        'iou_occupied': (s_occ & outline).sum() / (s_occ | outline).sum(),     # exact cell overlap
        'precision_1cell': (s_occ & near(outline)).sum() / s_occ.sum(),       # SLAM walls that are on a true wall
        'recall_1cell': (outline & near(s_occ)).sum() / outline.sum(),        # true outline that the SLAM map has
        'cell_accuracy': ((s_occ & t_occ) | (s_free & ~t_occ)).sum() / known.sum(),   # over cells the SLAM map knows
        'free_coverage': (s_free & reach).sum() / reach.sum(),                # reachable free space mapped as free
        'dist_median_m': float(np.median(d)), 'dist_p95_m': float(np.percentile(d, 95)),
    }
