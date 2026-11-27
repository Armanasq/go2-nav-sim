#!/usr/bin/env python3
"""Render a 1080p dashboard video from a recorded bag (no simulator needed; the bag holds everything).

usage: render_dashboard.py BAG_DIR OUT.mp4 [--still OUT.png] [--fps 15] [--cards]

Panels: external camera | onboard RGB | depth (turbo) / 3D lidar sweep | map + plan + trajectories | metrics | IMU traces.
Timeline = bag receive time (the sim runs at ~1.0x real time). Pose on the map comes from /amcl_pose, ground truth from /odom_gt.
Stats are evaluation-only: ground truth is never used by the robot."""
import argparse, collections, math, pathlib, subprocess, sys
import cv2, numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2  # noqa: F401  (kept for parity; points are decoded with numpy below)

W, H, M, G, HEAD = 1920, 1080, 24, 16, 76
CW = (W - 2 * M - 2 * G) // 3                  # column width 613
IH = CW * 9 // 16                              # 16:9 image height 345
TB = 32                                        # panel title bar
PH = IH + TB
ROW1, ROW2 = HEAD + 8, HEAD + 8 + PH + G
ROW3 = ROW2 + PH + G
STRIP_H = H - ROW3 - M
A, B = (-4.5, -3.5), (4.0, 3.0)                # start / goal (world == AMCL map frame), == make_arena.py
BG, CARD, EDGE = (14, 17, 22), (21, 26, 34), (40, 49, 62)
TXT, MUTE = (232, 237, 244), (139, 150, 164)
CYAN, AMBER, GREEN, RED, VIOLET = (86, 204, 242), (245, 176, 65), (74, 222, 128), (248, 113, 113), (167, 139, 250)
AX = {'x': (255, 123, 114), 'y': (126, 231, 135), 'z': (121, 192, 255)}
FONTS = ['/usr/share/fonts/truetype/roboto/unhinted/RobotoTTF/Roboto-%s.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf']
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'


def font(size, weight='Regular', mono=False):
    for p in ([MONO] if mono else [FONTS[0] % weight, FONTS[1] % ('-Bold' if weight == 'Bold' else '')]):
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            pass
    return ImageFont.load_default()


F = {k: font(*v) for k, v in dict(h1=(30, 'Bold'), h2=(15, 'Medium'), small=(13,), label=(12, 'Medium'), big=(34, 'Bold'), mid=(20, 'Medium')).items()}
F['mono'], F['monob'] = font(15, mono=True), font(26, mono=True)
TURBO = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(256, 3)[:, ::-1]   # RGB LUT


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rpy_of(q):
    r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    p = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
    return r, p, yaw_of(q)


def img_of(m, ch=3):
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, ch)


class State:
    def __init__(self):
        self.t0 = None; self.clock = 0.0
        self.img = {}; self.depth = None; self.cloud = None; self.scan = None; self.map = None; self.plan = None
        self.amcl = None; self.gt = None; self.odom = None; self.cmd = (0, 0)
        self.imu = collections.deque(maxlen=1000)
        self.imu_t = collections.deque(maxlen=900)
        self.traj_gt, self.traj_est, self.err = [], [], []
        self.dist = 0.0; self.slam_off = None


def decode(topic, msg, S, t):
    if topic == '/clock':
        S.clock = msg.clock.sec + msg.clock.nanosec * 1e-9
    elif topic in ('/spectator/cam/image_raw', '/camera/image_raw', '/chase/cam/image_raw'):
        S.img[topic] = img_of(msg)
    elif topic == '/camera/depth/image_raw':
        S.depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
    elif topic == '/lidar/points':
        a = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        S.cloud = a[np.isfinite(a).all(1)]
    elif topic == '/scan':
        r = np.array(msg.ranges); ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        S.scan = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], 1)
    elif topic == '/slam/map':
        S.map = (np.array(msg.data, np.int8).reshape(msg.info.height, msg.info.width), msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
    elif topic == '/plan':
        S.plan = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]) if msg.poses else None
    elif topic == '/amcl_pose':
        p = msg.pose.pose; S.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))
    elif topic == '/odom_gt':
        p = msg.pose.pose; new = (p.position.x + A[0], p.position.y + A[1], yaw_of(p.orientation))
        if S.gt is not None:
            S.dist += math.hypot(new[0] - S.gt[0], new[1] - S.gt[1])
        S.gt = new
        if S.slam_off is None and S.odom is not None:
            S.slam_off = (new[0] - S.odom[0], new[1] - S.odom[1])            # world position of the estimator's odom origin
        S.traj_gt.append(new[:2])
        if S.amcl:
            S.traj_est.append(S.amcl[:2]); S.err.append((t, math.hypot(S.amcl[0] - new[0], S.amcl[1] - new[1])))
    elif topic == '/odom':
        p = msg.pose.pose; S.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        S.odom_v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    elif topic == '/imu/data':
        a, g = msg.linear_acceleration, msg.angular_velocity; r, p, y = rpy_of(msg.orientation)
        S.imu.append((t, a.x, a.y, a.z, g.x, g.y, g.z, math.degrees(r), math.degrees(p)))


# ----------------------------------------------------------------------------------------------------------------------------
def panel(canvas, x, y, w, h, title, tag=None):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), CARD, -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), EDGE, 1)
    return (x, y + TB, w, h - TB), (title, tag)


def fit(img, w, h, interp=cv2.INTER_AREA):
    return cv2.resize(img, (w, h), interpolation=interp)


def render_lidar(S, w, h):
    out = np.full((h, w, 3), (10, 13, 18), np.uint8)
    pos = np.array([-3.4, 0.0, 2.7]); tgt = np.array([2.5, 0, -0.3]); f = tgt - pos; f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r); u = np.cross(r, f); fx = 0.95 * w

    def proj(P):
        d = P - pos; z = d @ f; ok = z > 0.3
        return fx * (d @ r) / np.maximum(z, 1e-3) + w / 2, -fx * (d @ u) / np.maximum(z, 1e-3) + h * 0.52, z, ok
    for rad in (2, 4, 6, 8, 10):                                          # range rings on the ground plane
        th = np.linspace(0, 2 * np.pi, 120); ring = np.stack([rad * np.cos(th), rad * np.sin(th), np.full_like(th, -0.36)], 1)
        px, py, _, ok = proj(ring); pts = np.stack([px, py], 1)[ok].astype(np.int32)
        if len(pts) > 2:
            cv2.polylines(out, [pts], False, (34, 42, 54), 1, cv2.LINE_AA)
    if S.cloud is not None and len(S.cloud):
        P = S.cloud; px, py, z, ok = proj(P); px, py, z, Z = px[ok].astype(int), py[ok].astype(int), z[ok], P[ok, 2]
        inb = (px > 0) & (px < w - 2) & (py > 0) & (py < h - 2); px, py, z, Z = px[inb], py[inb], z[inb], Z[inb]
        o = np.argsort(-z); px, py, z, Z = px[o], py[o], z[o], Z[o]         # far to near
        col = TURBO[np.clip(((Z + 0.45) / 1.9 * 255), 0, 255).astype(np.uint8)].astype(np.float32)
        fade = np.clip(1.25 - z / 14.0, 0.35, 1.0)[:, None]
        col = (col * fade).astype(np.uint8)
        for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
            out[py + dy, px + dx] = col
    box = np.array([[sx * .33, sy * .15, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-0.33, -0.12)])   # robot body outline
    px, py, _, ok = proj(box)
    for a, b in ((0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)):
        if ok[a] and ok[b]:
            cv2.line(out, (int(px[a]), int(py[a])), (int(px[b]), int(py[b])), CYAN, 1, cv2.LINE_AA)
    return out


def render_depth(S, w, h):
    out = np.full((h, w, 3), (10, 13, 18), np.uint8)
    if S.depth is not None:
        d = S.depth; ok = np.isfinite(d) & (d > 0)
        n = np.zeros(d.shape, np.uint8); n[ok] = np.clip((d[ok] - 0.3) / 7.7 * 255, 0, 255).astype(np.uint8)
        col = TURBO[255 - n]; col[~ok] = (18, 22, 29)
        out[:, :w - 26] = fit(col, w - 26, h)
    bar = TURBO[np.linspace(255, 0, h).astype(int)]                       # near = warm, far = cool
    out[:, w - 16:w - 6] = bar[:, None, :]
    return out


def render_map(S, w, h, gt_map):
    out = np.full((h, w, 3), (10, 13, 18), np.uint8)
    xmin, xmax, ymin, ymax = -6.7, 6.7, -5.7, 5.7
    s = min(w / (xmax - xmin), h / (ymax - ymin)) * 0.96; ox = (w - s * (xmax - xmin)) / 2; oy = (h - s * (ymax - ymin)) / 2
    P = lambda x, y: (int(ox + (x - xmin) * s), int(h - oy - (y - ymin) * s))
    out[:] = (12, 15, 20)
    for gx in np.arange(-6, 7, 1.0):                                       # 1 m grid
        cv2.line(out, P(gx, ymin), P(gx, ymax), (22, 28, 36), 1)
    for gy in np.arange(-5, 6, 1.0):
        cv2.line(out, P(xmin, gy), P(xmax, gy), (22, 28, 36), 1)
    for c in gt_map:                                                       # ground-truth layout, hairline
        cv2.polylines(out, [np.array([P(x, y) for x, y in c], np.int32)], True, (70, 82, 100), 1, cv2.LINE_AA)
    if S.map is not None and S.slam_off is not None:                       # robot-built (slam_toolbox) map, drawn as a raster
        g, res, mx, my = S.map; hh, ww = g.shape
        rgb = np.zeros((hh, ww, 3), np.uint8); rgb[g == 0] = (30, 62, 74); rgb[g >= 50] = CYAN
        mask = (g >= 0).astype(np.uint8) * 255
        sc = res * s                                                       # px per cell
        x0 = ox + (S.slam_off[0] + mx - xmin) * s; y0 = h - oy - (S.slam_off[1] + my + hh * res - ymin) * s
        Mx = np.float32([[sc, 0, x0], [0, -sc, y0 + hh * sc]])
        layer = cv2.warpAffine(rgb[::-1].copy(), np.float32([[sc, 0, x0], [0, sc, y0]]), (w, h), flags=cv2.INTER_NEAREST)
        lm = cv2.warpAffine(mask[::-1].copy(), np.float32([[sc, 0, x0], [0, sc, y0]]), (w, h), flags=cv2.INTER_NEAREST) > 0
        out[lm] = layer[lm]
    if len(S.traj_gt) > 1:
        cv2.polylines(out, [np.array([P(*p) for p in S.traj_gt], np.int32)], False, (96, 108, 126), 1, cv2.LINE_AA)
    if S.plan is not None and len(S.plan) > 1:
        cv2.polylines(out, [np.array([P(*p) for p in S.plan[::3]], np.int32)], False, AMBER, 2, cv2.LINE_AA)
    pose = S.amcl or S.gt
    if S.scan is not None and pose:
        c, sn = math.cos(pose[2]), math.sin(pose[2]); X = pose[0] + c * S.scan[:, 0] - sn * S.scan[:, 1]; Y = pose[1] + sn * S.scan[:, 0] + c * S.scan[:, 1]
        for x, y in zip(X[::2], Y[::2]):
            cv2.circle(out, P(x, y), 1, RED, -1)
    for q, col, lab in ((A, GREEN, 'A'), (B, RED, 'B')):
        cv2.circle(out, P(*q), 9, col, 2, cv2.LINE_AA); cv2.circle(out, P(*q), 3, col, -1, cv2.LINE_AA)
    if pose:
        x, y = P(pose[0], pose[1]); d = 17; a = pose[2]
        tri = np.array([[x + d * math.cos(-a), y + d * math.sin(-a)], [x + 9 * math.cos(-a + 2.5), y + 9 * math.sin(-a + 2.5)], [x + 9 * math.cos(-a - 2.5), y + 9 * math.sin(-a - 2.5)]], np.int32)
        cv2.fillConvexPoly(out, tri, (255, 255, 255), cv2.LINE_AA)
    return out, P


def strip_chart(canvas, x, y, w, h, series, ylim, names, t_now, win=8.0, unit=''):
    """series: list of (T array, V array, color)."""
    px0, py0, pw, ph = x + 44, y + 10, w - 60, h - 26
    cv2.rectangle(canvas, (px0, py0), (px0 + pw, py0 + ph), (17, 21, 28), -1)
    lo, hi = ylim
    for g in np.linspace(lo, hi, 5):
        yy = int(py0 + ph - (g - lo) / (hi - lo) * ph); cv2.line(canvas, (px0, yy), (px0 + pw, yy), (30, 37, 48), 1)
    for T, V, col in series:
        if len(T) < 2:
            continue
        xs = px0 + (T - (t_now - win)) / win * pw; ys = py0 + ph - (np.clip(V, lo, hi) - lo) / (hi - lo) * ph
        cv2.polylines(canvas, [np.stack([xs, ys], 1).astype(np.int32)], False, col, 2, cv2.LINE_AA)
    return (px0, py0, pw, ph), lo, hi


def text_layer(img, items):
    pil = PILImage.fromarray(img); d = ImageDraw.Draw(pil)
    for (x, y), s, f, col, *anc in items:
        d.text((x, y), s, font=F[f] if isinstance(f, str) else f, fill=col, anchor=(anc[0] if anc else 'la'))
    return np.asarray(pil)


def draw_frame(S, t, gt_map, T0, goal_hit, tcards=None):
    cv = np.full((H, W, 3), BG, np.uint8); txt = []
    cols = [M + i * (CW + G) for i in range(3)]
    # header
    txt += [((M, 14), 'UNITREE GO2  ·  SIMULATION', 'label', MUTE), ((M, 32), 'LiDAR-inertial navigation, mapping and perception', 'h1', TXT)]
    tt = S.clock if S.clock else t - T0
    state = 'ARRIVED' if goal_hit else 'NAVIGATING'
    chip = GREEN if goal_hit else CYAN
    cv2.rectangle(cv, (W - M - 150, 24), (W - M, 56), (24, 38, 36) if goal_hit else (20, 34, 44), -1)
    cv2.rectangle(cv, (W - M - 150, 24), (W - M, 56), chip, 1)
    txt += [((W - M - 75, 40), state, 'h2', chip, 'mm'), ((W - M - 168, 40), f'{tt:6.1f} s', 'monob', TXT, 'rm'), ((W - M - 168 - 150, 33), 'SIM TIME', 'label', MUTE, 'rm')]
    # row 1 / row 2 panels
    specs = [(0, ROW1, 'EXTERNAL VIEW', 'spectator camera'), (1, ROW1, 'ONBOARD RGB', '/camera/image_raw'), (2, ROW1, 'DEPTH', '0.3 – 8 m, 32FC1'),
             (0, ROW2, 'LIDAR  ·  3D SWEEP', '16 beams, 10 Hz'), (1, ROW2, 'MAP  ·  PLAN  ·  TRAJECTORY', 'slam_toolbox / AMCL'), (2, ROW2, 'RUN METRICS', 'evaluation only')]
    for (ci, ry, title, tag) in specs:
        (ix, iy, iw, ih), _ = panel(cv, cols[ci], ry, CW, PH, title)
        txt += [((cols[ci] + 12, ry + 8), title, 'h2', TXT), ((cols[ci] + CW - 12, ry + 10), tag, 'small', MUTE, 'ra')]
    def blit(ci, ry, im):
        cv[ry + TB:ry + TB + IH, cols[ci]:cols[ci] + CW] = im[:IH, :CW]
    ext = S.img.get('/spectator/cam/image_raw'); rgb = S.img.get('/camera/image_raw')
    if ext is not None: blit(0, ROW1, fit(ext, CW, IH))
    if rgb is not None: blit(1, ROW1, fit(rgb, CW, IH))
    blit(2, ROW1, render_depth(S, CW, IH))
    txt += [((cols[2] + CW - 40, ROW1 + TB + 4), '0.3 m', 'label', TXT, 'ra'), ((cols[2] + CW - 40, ROW1 + TB + IH - 18), '8 m', 'label', TXT, 'ra')]
    blit(0, ROW2, render_lidar(S, CW, IH))
    mp, P = render_map(S, CW, IH, gt_map); blit(1, ROW2, mp)
    for q, lab in ((A, 'A'), (B, 'B')):
        x, y = P(*q); txt.append(((cols[1] + x + 12, ROW2 + TB + y - 9), lab, 'h2', GREEN if lab == 'A' else RED))
    # legend on map
    lx, ly = cols[1] + 12, ROW2 + TB + 10
    for i, (lab, col) in enumerate((('SLAM map', CYAN), ('plan', AMBER), ('scan', RED), ('GT path', (120, 132, 150)))):
        cv2.rectangle(cv, (lx, ly + 4 + i * 17), (lx + 12, ly + 8 + i * 17), col, -1); txt.append(((lx + 18, ly + i * 17), lab, 'small', MUTE))
    # metrics panel
    mx, my = cols[2] + 16, ROW2 + TB + 12
    pose = S.amcl or S.gt; dgoal = math.hypot(B[0] - pose[0], B[1] - pose[1]) if pose else float('nan')
    err = S.err[-1][1] if S.err else float('nan'); spd = getattr(S, 'odom_v', 0.0)
    items = [('SPEED', f'{spd:0.2f}', 'm/s'), ('DISTANCE TO GOAL', f'{dgoal:0.2f}', 'm'), ('PATH TRAVELLED', f'{S.dist:0.2f}', 'm'), ('LOCALISATION ERROR', f'{err*100:0.1f}', 'cm')]
    for i, (lab, val, unit) in enumerate(items):
        cx, cy = mx + (i % 2) * (CW // 2 - 8), my + (i // 2) * 84
        txt += [((cx, cy), lab, 'label', MUTE), ((cx, cy + 18), val, 'big', TXT), ((cx + 8 + F['big'].getlength(val), cy + 34), unit, 'small', MUTE)]
    # error sparkline
    sy0 = my + 176; txt.append(((mx, sy0 - 4), 'LOCALISATION ERROR OVER TIME', 'label', MUTE))
    if len(S.err) > 2:
        e = np.array(S.err); ex = (mx + (e[:, 0] - e[0, 0]) / max(e[-1, 0] - e[0, 0], 1) * (CW - 40)); ey = sy0 + 100 - np.clip(e[:, 1], 0, 0.5) / 0.5 * 88
        cv2.line(cv, (mx, sy0 + 100), (mx + CW - 40, sy0 + 100), EDGE, 1)
        cv2.polylines(cv, [np.stack([ex, ey], 1).astype(np.int32)], False, CYAN, 2, cv2.LINE_AA)
        txt += [((mx + CW - 40, sy0 + 12), '0.5 m', 'small', MUTE, 'ra')]
    # IMU strip
    imu = np.array(S.imu) if len(S.imu) else np.zeros((0, 9)); tn = t - T0
    if len(imu):
        imu[:, 0] -= T0; imu = imu[imu[:, 0] > tn - 8.0]
    sx = [M + i * (CW + G) for i in range(3)]
    for i, (title, tag) in enumerate((('IMU  ·  ACCELERATION', 'm/s²'), ('IMU  ·  ANGULAR VELOCITY', 'rad/s'), ('IMU  ·  ATTITUDE', 'deg'))):
        cv2.rectangle(cv, (sx[i], ROW3), (sx[i] + CW, ROW3 + STRIP_H), CARD, -1); cv2.rectangle(cv, (sx[i], ROW3), (sx[i] + CW, ROW3 + STRIP_H), EDGE, 1)
        txt += [((sx[i] + 12, ROW3 + 8), title + '  ·  ' + tag, 'h2', TXT)]
    ch_h = STRIP_H - TB
    cfg = [((-6, 14), [(1, 'x'), (2, 'y'), (3, 'z')]), ((-1.5, 1.5), [(4, 'x'), (5, 'y'), (6, 'z')]), ((-12, 12), [(7, 'roll'), (8, 'pitch')])]
    for i, (lim, chans) in enumerate(cfg):
        sm = lambda v: np.convolve(np.pad(v, 3, mode='edge'), np.ones(7) / 7, mode='valid')
        ser = [(imu[:, 0], sm(imu[:, c]), AX.get(n, AX['x'] if n == 'roll' else AX['z'])) for c, n in chans] if len(imu) else []
        box, lo, hi = strip_chart(cv, sx[i], ROW3 + TB, CW, ch_h, ser, lim, [n for _, n in chans], tn)
        for g in (lo, (lo + hi) / 2, hi):
            yy = box[1] + box[3] - (g - lo) / (hi - lo) * box[3]; txt.append(((box[0] - 6, yy), f'{g:g}', 'small', MUTE, 'rm'))
        for k, (c, n) in enumerate(chans):
            col = AX.get(n, AX['x'] if n == 'roll' else AX['z']); xx = sx[i] + CW - 14 - (len(chans) - k) * 64
            cv2.line(cv, (xx, ROW3 + 19), (xx + 12, ROW3 + 19), col, 3); txt.append(((xx + 17, ROW3 + 11), n, 'small', MUTE))
    return text_layer(cv, txt)


def card(lines, sub, alpha=1.0, stats=None):
    cv = np.full((H, W, 3), BG, np.uint8)
    cv2.rectangle(cv, (W // 2 - 360, H // 2 - 4), (W // 2 + 360, H // 2 - 2), CYAN, -1)
    txt = [((W // 2, H // 2 - 130), 'UNITREE GO2  ·  ROS 2 HUMBLE  ·  GAZEBO', 'label', MUTE, 'mm')]
    txt += [((W // 2, H // 2 - 70 + 0), lines, 'h1', TXT, 'mm'), ((W // 2, H // 2 + 36), sub, 'mid', MUTE, 'mm')]
    if stats:
        for i, (k, v) in enumerate(stats):
            x = W // 2 - 540 + i * 270; txt += [((x + 135, H // 2 + 110), v, 'big', TXT, 'mm'), ((x + 135, H // 2 + 150), k, 'label', MUTE, 'mm')]
    return (text_layer(cv, txt).astype(np.float32) * alpha).astype(np.uint8)


def gt_contours():
    pgm = pathlib.Path(__file__).resolve().parents[1] / 'maps/arena.pgm'
    if not pgm.exists():
        return []
    raw = pgm.read_bytes(); _, dims, _, body = raw.split(b'\n', 3); w, h = map(int, dims.split())
    img = np.frombuffer(body, np.uint8).reshape(h, w); occ = (img < 128).astype(np.uint8)[::-1]
    cs, _ = cv2.findContours(occ, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return [[(-6.5 + (p[0][0] + .5) * 0.05, -5.5 + (p[0][1] + .5) * 0.05) for p in c] for c in cs if len(c) > 3]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('out'); ap.add_argument('--still'); ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--cards', action='store_true'); ap.add_argument('--still-at', type=float, default=0.62)
    a = ap.parse_args()
    for out in (a.out, a.still):
        if out:
            pathlib.Path(out).parent.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    info = rosbag2_py.Info().read_metadata(a.bag, 'mcap'); total = info.duration.nanoseconds * 1e-9; start = info.starting_time.nanoseconds * 1e-9
    S = State(); gt_map = gt_contours(); dt = 1.0 / a.fps; nxt = start; goal_hit = False; n = 0; still_done = False
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(a.fps), '-i', '-',
                           '-c:v', 'libx264', '-preset', 'medium', '-crf', '17', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', a.out], stdin=subprocess.PIPE)
    put = lambda fr: ff.stdin.write(np.ascontiguousarray(fr).tobytes())
    if a.cards:
        for i in range(int(2.5 * a.fps)):
            k = i / (2.5 * a.fps); put(card('Navigation without a prior map or GPS', 'LiDAR-inertial odometry · AMCL · Nav2 · slam_toolbox · RGB-D', alpha=min(1, k * 3) * min(1, (1 - k) * 4)))
    while reader.has_next():
        topic, data, tns = reader.read_next(); t = tns * 1e-9
        decode(topic, deserialize_message(data, types[topic]), S, t)
        while t >= nxt:
            if S.gt is not None and S.img.get('/camera/image_raw') is not None and S.cloud is not None:
                pose = S.amcl or S.gt; goal_hit = goal_hit or math.hypot(B[0] - pose[0], B[1] - pose[1]) < 0.4
                fr = draw_frame(S, nxt, gt_map, start, goal_hit); put(fr); n += 1
                if a.still and not still_done and (nxt - start) / total >= a.still_at:
                    PILImage.fromarray(fr).save(a.still); still_done = True
            nxt += dt
    if a.cards:
        e = np.array(S.err)[:, 1] if S.err else np.array([0.0]); sec = S.clock if S.clock else total
        stats = [('PATH', f'{S.dist:0.1f} m'), ('DURATION', f'{total:0.0f} s'), ('MEAN SPEED', f'{S.dist / max(total, 1):0.2f} m/s'), ('LOCALISATION RMSE', f'{np.sqrt((e ** 2).mean()) * 100:0.1f} cm')]
        for i in range(int(4 * a.fps)):
            k = i / (4 * a.fps); put(card('Goal reached' if goal_hit else 'Run finished', 'Ground truth was used for evaluation only', alpha=min(1, k * 4) * min(1, (1 - k) * 3), stats=stats))
    ff.stdin.close(); ff.wait(); print(f'\nwrote {a.out}: {n} frames', file=sys.stderr)


if __name__ == '__main__':
    main()
