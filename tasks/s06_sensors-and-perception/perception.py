"""Stage 6 logic without ROS: IMU statistics, depth back-projection, plane fits, floor removal, clustering, platform estimate.
Every function takes and returns numpy arrays, so tests/ can check it without a simulator."""
import math

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


def rpy(qx, qy, qz, qw):
    """Roll, pitch, yaw (rad) of a unit quaternion (ZYX convention, as tf2)."""
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def rot(roll, pitch, yaw):
    """Rotation matrix R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def tilt_from_accel(a):
    """Roll and pitch (rad) of a body at rest from its mean specific force a = R^T (0, 0, g)."""
    ax, ay, az = a
    return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))


def allan_deviation(x, dt, taus):
    """Overlapping Allan deviation of a rate signal x sampled every dt, at averaging times taus (s).
    theta = integral of x; sigma^2(tau) = sum (theta[k+2m] - 2 theta[k+m] + theta[k])^2 / (2 tau^2 (N - 2m))."""
    theta = np.concatenate([[0.0], np.cumsum(x) * dt])
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        out.append(math.sqrt(np.sum(d * d) / (2 * (m * dt) ** 2 * len(d))))
    return np.array(out)


def integrate(t, w):
    """Cumulative trapezoidal integral of w(t), starting at 0."""
    t, w = np.asarray(t, float), np.asarray(w, float)
    return np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])


def floor_depth(v, cy, fy, h, pitch=0.0):
    """Optical-axis depth of the floor seen on image row v by a camera h above it, pitched by `pitch` (rad, REP-103: positive
    = nose down); nan above the horizon. The ray (x_n, y_n, 1) drops y_n cos(pitch) + sin(pitch) per metre of depth."""
    drop = (np.asarray(v, float) - cy) / fy * math.cos(pitch) + math.sin(pitch)
    return np.where(drop > 0, h / np.where(drop > 0, drop, 1), np.nan)


def backproject(depth, fx, fy, cx, cy, zmax=np.inf):
    """32FC1 depth image -> (N, 3) points in the optical frame (x right, y down, z forward); nan and z > zmax dropped."""
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(depth) & (depth > 0) & (depth <= zmax)
    z = depth[ok]
    return np.column_stack([(u[ok] - cx) * z / fx, (v[ok] - cy) * z / fy, z])


def fit_plane(P):
    """Least-squares plane through P: unit normal n and offset d with n . p + d = 0."""
    c = P.mean(0)
    n = np.linalg.svd(P - c, full_matrices=False)[2][-1]
    return n, -n @ c


def ransac_plane(P, thresh, iters=300, axis=None, max_angle_deg=10.0, seed=0):
    """Plane with the most points within `thresh`, refined on its inliers. With `axis`, only planes whose normal lies
    within max_angle_deg of it are considered, and the normal is returned pointing along `axis`. Returns (n, d, inlier mask)."""
    rng = np.random.default_rng(seed)
    best, cos_min = None, math.cos(math.radians(max_angle_deg))
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if axis is not None and abs(n @ axis) < cos_min:
            continue
        inl = np.abs(P @ n - n @ a) < thresh
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None:
        return None, None, np.zeros(len(P), bool)
    n, d = fit_plane(P[best])
    if axis is not None and n @ axis < 0:
        n, d = -n, -d
    return n, d, np.abs(P @ n + d) < thresh


def remove_floor_and_body(P, floor_n, floor_d, floor_tol=0.05, body=(0.40, 0.20)):
    """Keep mask for points P (base_link) that are more than floor_tol above the floor plane (normal up) and outside the
    body footprint |x| <= body[0], |y| <= body[1]."""
    above = P @ floor_n + floor_d > floor_tol
    in_body = (np.abs(P[:, 0]) <= body[0]) & (np.abs(P[:, 1]) <= body[1])
    return above & ~in_body


def cluster(xy, radius=0.15, min_pts=5):
    """Euclidean clustering: points closer than `radius` share a cluster. Labels (-1 = cluster smaller than min_pts)."""
    if len(xy) == 0:
        return np.zeros(0, int)
    pairs = cKDTree(xy).query_pairs(radius, output_type='ndarray')
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(xy), len(xy)))
    _, lab = connected_components(g, directed=False)
    out = np.full(len(xy), -1)
    for i, c in enumerate(np.flatnonzero(np.bincount(lab) >= min_pts)):
        out[lab == c] = i
    return out


def footprint_dist(xy, shape):
    """Distance (m) from points to an obstacle footprint, 0 inside. shape = ('box', cx, cy, sx, sy) or ('cyl', cx, cy, r)."""
    x, y = xy[:, 0] - shape[1], xy[:, 1] - shape[2]
    if shape[0] == 'cyl':
        return np.maximum(np.hypot(x, y) - shape[3], 0.0)
    dx, dy = np.maximum(np.abs(x) - shape[3] / 2, 0.0), np.maximum(np.abs(y) - shape[4] / 2, 0.0)
    return np.hypot(dx, dy)


def ray_hit(o, d, shape):
    """Distance along the unit ray o + t d to the first hit on a footprint (inf if missed)."""
    ox, oy = o[0] - shape[1], o[1] - shape[2]
    if shape[0] == 'cyl':
        b, c = ox * d[0] + oy * d[1], ox * ox + oy * oy - shape[3] ** 2
        disc = b * b - c
        t = -b - math.sqrt(disc) if disc >= 0 else math.inf
        return t if t > 0 else math.inf
    t0, t1 = -math.inf, math.inf
    for p, v, h in ((ox, d[0], shape[3] / 2), (oy, d[1], shape[4] / 2)):     # slab test
        if abs(v) < 1e-12:
            if abs(p) > h:
                return math.inf
            continue
        a, b = (-h - p) / v, (h - p) / v
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    return t0 if t1 >= t0 > 0 else math.inf


def visible(origin, shapes, n_rays=720, max_range=15.0):
    """Number of rays (of n_rays around origin) whose first hit is each shape."""
    hits = np.zeros(len(shapes), int)
    for a in np.linspace(-math.pi, math.pi, n_rays, endpoint=False):
        d = (math.cos(a), math.sin(a))
        t = [ray_hit(origin, d, s) for s in shapes]
        k = int(np.argmin(t))
        if t[k] <= max_range:
            hits[k] += 1
    return hits


def platform_from_points(P, thresh=0.015, min_height=0.03, max_height=0.5, up=(0.0, -1.0, 0.0), bin_w=0.01, min_pts=200):
    """Find the floor and one raised horizontal surface in optical-frame points P.
    1. floor: RANSAC plane with the normal within 10 deg of `up` (it holds most of the points looking down a room);
    2. height of every point above it, h = n . p + d;
    3. the top: the most common height between min_height and max_height (a horizontal surface puts all its points in one
       height bin, a vertical face spreads them over many), refined by a plane fit;
    4. keep only the largest connected patch of those points, so slices of walls at the same height are dropped.
    Returns dict with the planes, masks, height and the angle between the two normals."""
    up = np.asarray(up, float)
    nf, df, floor = ransac_plane(P, thresh, axis=up)
    if nf is None:
        return None
    h = P @ nf + df
    cand = (h > min_height) & (h < max_height)
    out = dict(floor=(nf, df), floor_mask=floor, top=None, top_mask=np.zeros(len(P), bool), height=np.nan, angle_deg=np.nan,
               detected=False)
    if cand.sum() < min_pts:
        return out
    counts, edges = np.histogram(h[cand], bins=np.arange(min_height, max_height + bin_w, bin_w))
    k = int(np.argmax(counts))
    top = cand & (np.abs(h - (edges[k] + edges[k + 1]) / 2) < thresh)
    nt, dt = fit_plane(P[top])
    if nt @ up < 0:
        nt, dt = -nt, -dt
    top = cand & (np.abs(P @ nt + dt) < thresh)
    g = ground_frame(P[top], nf, df)
    lab = cluster(g[:, :2], 0.15, min_pts)              # 0.15 m: rows on a surface seen at a grazing angle are up to 0.1 m apart
    if lab.max() < 0:
        return out
    idx = np.flatnonzero(top)
    top = np.zeros(len(P), bool)
    top[idx[lab == np.argmax(np.bincount(lab[lab >= 0]))]] = True
    nt, dt = fit_plane(P[top])
    if nt @ up < 0:
        nt, dt = -nt, -dt
    # height measured at the platform: the two fitted normals differ by a fraction of a degree, so comparing the plane
    # offsets d (which are taken at the camera, metres away) would add that tilt times the distance
    out.update(top=(nt, dt), top_mask=top, height=float(np.median(h[top])), angle_deg=math.degrees(math.acos(min(1.0, abs(nf @ nt)))),
               detected=bool(top.sum() >= min_pts))
    return out


def ground_frame(P, n, d, forward=(0.0, 0.0, 1.0)):
    """Coordinates of optical-frame points in a frame on the floor below the camera: x along the camera's forward axis
    projected on the floor, y to the left, z up along the floor normal n (n . p + d = 0, n up)."""
    f = np.asarray(forward, float) - (np.asarray(forward, float) @ n) * n
    f /= np.linalg.norm(f)
    left = np.cross(n, f)
    return np.column_stack([P @ f, P @ left, P @ n + d])
