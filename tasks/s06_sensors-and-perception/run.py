#!/usr/bin/env python3
"""Stage 6 acceptance tests against the running simulator.
usage: python3 run.py <task-id>|all [--video]        e.g. python3 run.py 6.03
Starts the simulator and estimation launch itself when they are not running (logs/), restarts them when a task needs the robot
fresh on marker A or the platform world. A task with 'runs': N in acceptance.json runs N times, each on a fresh simulator,
and is verified only when every run passes. Writes results/<id>.json (metrics, thresholds from acceptance.json, verdict) and
figures/. --video also records media/6.14_new_obstacle.mp4: wrap that run in `flock /tmp/go2_render.lock`."""
import csv
import json
import math
import re
import pathlib
import signal
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
P05 = ROOT / 'projects/p05_sense-and-stop'
sys.path[:0] = [str(HERE), str(HERE / 'worlds'), str(ROOT / 'src/go2_nav_sim/scripts'), str(P05), str(ROOT / 'modules')]

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, Twist  # noqa: E402
from gazebo_msgs.srv import DeleteEntity, SpawnEntity  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2, Range  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
import tf2_ros  # noqa: E402
from tf2_sensor_msgs import do_transform_cloud  # noqa: E402

import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import make_arena as arena  # noqa: E402
import make_worlds as worlds  # noqa: E402
import obstacle  # noqa: E402  (p05: front_min, centre_depth, FRONT)
import perception as pc  # noqa: E402

PY = '/usr/bin/python3'
ACC = json.loads((HERE / 'acceptance.json').read_text())
FIG, RES, LOGS, MEDIA = HERE / 'figures', HERE / 'results', HERE / 'logs', HERE / 'media'
WORLDS = {'arena': ROOT / 'src/go2_nav_sim/worlds/arena.world', 'platform': HERE / 'worlds/platform.world'}
A = arena.A
LIDAR_Z, CAM = 0.10, (0.28, 0.03)                    # urdf/go2_nav.xacro: lidar 0.10 above base_link, camera 0.28 ahead, 0.03 up
RELIABLE = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
TASKS = {}


def task(tid, world='arena', at_a=False):
    def wrap(f):
        TASKS[tid] = (f, world, at_a)
        return f
    return wrap


# ---- ROS side ---------------------------------------------------------------------------------------------------------------
class Probe(Node):
    """Records IMU, ground truth, /front_range and /cmd_vel with their simulated stamps; grabs single messages on demand."""

    def __init__(self):
        super().__init__('s06_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.imu, self.gt, self.rng, self.cmd = [], [], [], []
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, qos_profile_sensor_data)
        self.create_subscription(Range, 'front_range', lambda m: self.rng.append((self.now(), stamp(m), m.range)), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.cmd.append((self.now(), m.linear.x, m.angular.z)), 10)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        t0 = time.time()                                 # wait up to 10 s for /clock; with no simulator, ensure() starts one
        while self.now() == 0 and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, m):
        w, a = m.angular_velocity, m.linear_acceleration
        self.imu.append((stamp(m), w.x, w.y, w.z, a.x, a.y, a.z))

    def on_gt(self, m):
        p, q, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.angular
        self.gt.append((stamp(m), p.x + A[0], p.y + A[1], p.z, *pc.rpy(q.x, q.y, q.z, q.w), w.z))   # world frame

    def clear(self):
        self.sync()                                      # drop what queued up before this point, then start recording
        self.imu.clear(); self.gt.clear(); self.rng.clear(); self.cmd.clear()

    def send(self, vx=0.0, wz=0.0):
        t = Twist(); t.linear.x, t.angular.z = float(vx), float(wz); self.pub.publish(t)

    def sync(self):
        """Process what queued up while we were not spinning (e.g. during a subprocess), so now() is current."""
        t_end = time.time() + 0.3
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def spin_for(self, dt, sync=True):
        if sync:
            self.sync()
        t0 = self.now()
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            t = self.now()
            if t < t0:                                   # simulator restarted: the clock jumped back
                t0 = t
            if t - t0 >= dt:
                return

    def drive(self, vx, wz, dur):
        """Send a twist at 20 Hz for `dur` s of simulated time (CHAMP holds the last command, so it is resent)."""
        self.sync()
        t0 = self.now()
        while self.now() - t0 < dur:
            self.send(vx, wz); self.spin_for(0.05, sync=False)

    def stand(self, dur):
        self.drive(0.0, 0.0, dur)

    def grab(self, msg_type, topic, qos=qos_profile_sensor_data, n=1, timeout=60.0):
        got = []
        sub = self.create_subscription(msg_type, topic, got.append, qos)
        t0 = time.time()
        while len(got) < n and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if len(got) < n:
            raise RuntimeError(f'no message on {topic} in {timeout} s')
        return got if n > 1 else got[0]

    def pose(self):
        """Latest ground-truth row: t, x, y, z, roll, pitch, yaw, wz (world frame)."""
        self.gt.clear()
        while not self.gt:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.gt[-1]

    def call(self, client, req):
        client.wait_for_service(timeout_sec=30)
        fut = client.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result()


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def up(n, timeout=20.0):
    """True when /imu/data and /scan both publish."""
    try:
        n.grab(Imu, 'imu/data', timeout=timeout); n.grab(LaserScan, 'scan', timeout=timeout)
        return True
    except RuntimeError:
        return False


def start_stack(n, world):
    print(f'starting the simulator ({world} world) and estimation, logs in {LOGS}', flush=True)
    subprocess.run(['bash', str(ROOT / 'scripts/stop.sh')], check=False)
    n.tf.clear()                                          # the new simulator clock starts again at 0
    time.sleep(3)
    LOGS.mkdir(exist_ok=True)
    for name, cmd in (('sim', ['ros2', 'launch', 'go2_nav_sim', 'sim.launch.py', f'world:={WORLDS[world]}', 'headless:=True']),
                      ('est', ['ros2', 'launch', 'go2_nav_sim', 'estimation.launch.py'])):
        subprocess.Popen(cmd, stdout=open(LOGS / f'{name}.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        if name == 'sim':
            n.grab(Imu, 'imu/data', timeout=300)
    if not up(n, 120):
        sys.exit('stack did not come up: see logs/')
    (LOGS / 'world.txt').write_text(world)
    n.tf.clear()
    n.stand(5.0)                                          # let the robot settle on its feet


def ensure(n, world, at_a):
    cur = (LOGS / 'world.txt').read_text().strip() if (LOGS / 'world.txt').exists() else None
    if cur != world or not up(n, 10):
        start_stack(n, world)
    if at_a:
        _, x, y, *_, yaw, _ = n.pose()
        if math.hypot(x - A[0], y - A[1]) > 0.3 or abs(yaw) > math.radians(3):
            print(f'robot at ({x:.2f}, {y:.2f}, {math.degrees(yaw):+.1f} deg), not on A: restarting', flush=True)
            start_stack(n, world)


def proc(args, **kw):
    return subprocess.Popen([PY, *map(str, args)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kw)


def goto_lane_start():
    """p05: walk to a true gap of 2.0 +- 0.1 m before the pallet's west face, on y = -2.5, facing +x."""
    out = subprocess.run([PY, str(P05 / 'measure_stops.py'), '--goto-start'], capture_output=True, text=True, timeout=900).stdout
    print(out.strip(), flush=True)
    return out.strip()


def stop_proc(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.kill()
    return p.stdout.read() if p.stdout else ''


def save(fig, name):
    p = FIG / f'{name}.png'
    fig.savefig(p, dpi=130, bbox_inches='tight', pad_inches=0.05)
    plt.close(fig)
    return f'figures/{name}.png'


def cloud_xyz(m):
    return point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)


def to_world(P, g, full=True):
    """base_link points -> world with a ground-truth row (full 3D pose, or horizontal pose with the camera height kept)."""
    _, x, y, z, r, p, yaw, _ = g
    R = pc.rot(r, p, yaw) if full else pc.rot(0, 0, yaw)
    return P @ R.T + np.array([x, y, z if full else 0.0])


def camera_K(n, topic='camera/camera_info'):
    k = n.grab(CameraInfo, topic, RELIABLE).k
    return k[0], k[4], k[2], k[5]


def depth_array(m):
    return np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width].astype(float)


def filtered_cloud(n):
    """One /lidar/points cloud in base_link (tf2), the floor plane fitted in it, and the keep mask of 6.10."""
    m = n.grab(PointCloud2, 'lidar/points')
    tf = n.tf.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=5))
    P = cloud_xyz(do_transform_cloud(m, tf))
    cand = P[:, 2] < -0.1                                        # floor candidates: well below base_link
    fn, fd, _ = pc.ransac_plane(P[cand], 0.03, axis=np.array([0, 0, 1.0]))
    return m, P, (fn, fd), pc.remove_floor_and_body(P, fn, fd)


# ---- 6.01 to 6.04: IMU ------------------------------------------------------------------------------------------------------
def imu_window(n, dur):
    n.clear(); n.stand(dur)
    return np.array(n.imu), np.array(n.gt)


@task('6.01', at_a=True)
def t601(n):
    I, G = imu_window(n, 20.0)
    a = I[:, 4:7].mean(0)
    roll, pitch = pc.tilt_from_accel(a)
    tilt_err = max(abs(roll - G[:, 4].mean()), abs(pitch - G[:, 5].mean()))
    return dict(n_samples=len(I), imu_rate_hz_sim=(len(I) - 1) / (I[-1, 0] - I[0, 0]),
                accel_mean_ms2=a.round(4).tolist(), accel_std_ms2=I[:, 4:7].std(0).round(4).tolist(),
                accel_norm_ms2=float(np.linalg.norm(a)), accel_norm_error_ms2=abs(float(np.linalg.norm(a)) - 9.8),
                gyro_mean_rads=I[:, 1:4].mean(0).tolist(), gyro_mean_abs_max_rads=float(np.abs(I[:, 1:4].mean(0)).max()),
                roll_pitch_from_accel_deg=[math.degrees(roll), math.degrees(pitch)],
                roll_pitch_gt_deg=[math.degrees(G[:, 4].mean()), math.degrees(G[:, 5].mean())],
                tilt_vs_gt_deg=math.degrees(tilt_err)), []


@task('6.03', at_a=True)
def t603(n):
    I, G = imu_window(n, 60.0)
    t = I[:, 0]
    dt = float(np.median(np.diff(t)))
    taus = np.logspace(np.log10(dt), np.log10((t[-1] - t[0]) / 9), 30)
    N_cfg_g, N_cfg_a = 2e-4 * math.sqrt(dt), 1.7e-2 * math.sqrt(dt)       # gazebo.xacro noise sigma * sqrt(dt)
    ad = {k: pc.allan_deviation(I[:, i], dt, taus) for i, k in enumerate(['gx', 'gy', 'gz', 'ax', 'ay', 'az'], 1)}
    N = {k: float(pc.allan_deviation(I[:, i], dt, [0.1])[0] * math.sqrt(0.1)) for i, k in enumerate(['gx', 'gy', 'gz', 'ax', 'ay', 'az'], 1)}
    ratios = [N[k] / N_cfg_g for k in ('gx', 'gy', 'gz')]
    resid = I[:, 3] - np.interp(t, G[:, 0], G[:, 7])                     # gyro z minus true yaw rate
    fig, ax = plt.subplots(1, 2, figsize=(7.6, 3.2))
    for k, c in zip(('gx', 'gy', 'gz'), (fs.C['est'], fs.C['est2'], fs.C['plan'])):
        ax[0].loglog(taus, ad[k], color=c, label=k[1])
    ax[0].loglog(taus, N_cfg_g / np.sqrt(taus), color=fs.C['gt'], ls='--', label='configured white noise')
    ax[0].set_xlabel(r'averaging time $\tau$ [s]'); ax[0].set_ylabel(r'Allan deviation [rad/s]'); ax[0].legend()
    ax[0].set_title('gyro, robot standing 60 s')
    for k, c in zip(('ax', 'ay', 'az'), (fs.C['est'], fs.C['est2'], fs.C['plan'])):
        ax[1].loglog(taus, ad[k], color=c, label=k[1])
    ax[1].loglog(taus, N_cfg_a / np.sqrt(taus), color=fs.C['gt'], ls='--', label='configured white noise')
    ax[1].set_xlabel(r'averaging time $\tau$ [s]'); ax[1].set_ylabel(r'Allan deviation [m/s$^2$]'); ax[1].legend()
    ax[1].set_title('accelerometer')
    out = save(fig, '6.03_allan')
    gaps = np.diff(t)
    return dict(duration_s=float(t[-1] - t[0]), n_samples=len(I), dt_s=dt, dropped_samples=int(np.sum(np.round(gaps / dt) - 1)),
                gyro_bias_rads=I[:, 1:4].mean(0).tolist(), gyro_bias_abs_max_rads=float(np.abs(I[:, 1:4].mean(0)).max()),
                gyro_std_rads=I[:, 1:4].std(0).tolist(), gyro_N_est=[N[k] for k in ('gx', 'gy', 'gz')],
                gyro_N_configured=N_cfg_g, gyro_white_noise_ratio=ratios,
                gyro_white_noise_ratio_min=min(ratios), gyro_white_noise_ratio_max=max(ratios),
                gyro_z_minus_gt_std_rads=float(resid.std()),
                accel_bias_ms2=(I[:, 4:7].mean(0) - [0, 0, 9.8]).tolist(), accel_std_ms2=I[:, 4:7].std(0).tolist(),
                accel_N_est=[N[k] for k in ('ax', 'ay', 'az')], accel_N_configured=N_cfg_a,
                allan_min_gyro_z=float(ad['gz'].min()), allan_min_tau_gyro_z_s=float(taus[np.argmin(ad['gz'])])), [out]


def turn_record(n, segments):
    """Drive (vx, wz, dur) segments, recording IMU and ground truth; returns arrays and the segment start/end times."""
    n.clear(); marks = []
    for vx, wz, dur in segments:
        t0 = n.now(); n.drive(vx, wz, dur); marks.append((t0, n.now(), wz))
    return np.array(n.imu), np.array(n.gt), marks


@task('6.02')
def t602(n):
    I, G, marks = turn_record(n, [(0, 0, 2), (0, 0.5, 8), (0, 0, 2), (0, -0.5, 8), (0, 0, 3)])
    gz_at_gt = np.interp(G[:, 0], I[:, 0], I[:, 3])
    win = []
    for t0, t1, wz in marks:
        if wz == 0:
            continue
        s = (G[:, 0] >= t1 - 5) & (G[:, 0] <= t1)                         # last 5 s of the turn: steady
        gt_mean, imu_mean = G[s, 7].mean(), gz_at_gt[s].mean()
        win.append(dict(commanded=wz, gt_mean=float(gt_mean), gyro_mean=float(imu_mean),
                        rel_error=float(abs(imu_mean - gt_mean) / abs(gt_mean)),
                        rms_diff=float(np.sqrt(np.mean((gz_at_gt[s] - G[s, 7]) ** 2)))))
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    t0 = G[0, 0]
    ax.plot(I[:, 0] - t0, I[:, 3], color=fs.C['est'], lw=0.8, label='gyro z (/imu/data)')
    ax.plot(G[:, 0] - t0, G[:, 7], color=fs.C['gt'], lw=1.0, ls='--', label='true yaw rate (/odom_gt)')
    for a, b, wz in marks:
        ax.hlines(wz, a - t0, b - t0, color=fs.C['plan'], lw=2, label='commanded' if wz == 0.5 else None)
    ax.set_xlabel('simulated time [s]'); ax.set_ylabel('yaw rate [rad/s]'); ax.legend(loc='lower left', ncol=3)
    ax.set_title('6.02 turn in place at +-0.5 rad/s')
    out = save(fig, '6.02_turn_rate')
    return dict(windows=win, mean_rate_rel_error=max(w['rel_error'] for w in win), rms_rate_diff_rads=max(w['rms_diff'] for w in win),
                achieved_over_commanded=[w['gt_mean'] / w['commanded'] for w in win]), [out]


@task('6.04')
def t604(n):
    """Turn until the integrated gyro says +90 deg, then back to 0, then stand 30 s."""
    n.clear(); n.stand(2.0)
    t_turn = []
    for target, wz in ((math.pi / 2, 0.5), (0.0, -0.5)):
        t0 = n.now()
        while True:
            I = np.array(n.imu)
            h = pc.integrate(I[:, 0], I[:, 3])[-1]
            if (wz > 0 and h >= target) or (wz < 0 and h <= target):
                break
            n.send(0, wz); n.spin_for(0.02, sync=False)
        n.stand(3.0); t_turn.append((t0, n.now()))
    t_stand0 = n.now(); n.stand(30.0)
    I, G = np.array(n.imu), np.array(n.gt)
    h_imu = pc.integrate(I[:, 0], I[:, 3])
    h_gt = np.unwrap(G[:, 6]) - G[0, 6]
    h_gt_i = np.interp(I[:, 0], G[:, 0], h_gt)
    err = np.degrees(h_imu - h_gt_i - (h_imu[0] - h_gt_i[0]))
    at = lambda t: float(np.interp(t, I[:, 0], err))
    turn_err = [at(b) for _, b in t_turn]
    drift = at(I[-1, 0]) - at(t_stand0)
    # is the standing drift a constant bias? estimate it on the first half of the stand, remove it from the second half
    tm = t_stand0 + 15.0
    s1, s2 = (I[:, 0] >= t_stand0) & (I[:, 0] < tm), I[:, 0] >= tm
    b = I[s1, 3].mean() - (np.interp(tm, G[:, 0], h_gt) - np.interp(t_stand0, G[:, 0], h_gt)) / 15.0
    corrected = at(I[-1, 0]) - at(tm) - math.degrees(b * (I[-1, 0] - tm))
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 3.8), sharex=True)
    t0 = I[0, 0]
    ax[0].plot(I[:, 0] - t0, np.degrees(h_imu), color=fs.C['est'], label='integrated gyro z')
    ax[0].plot(G[:, 0] - t0, np.degrees(h_gt), color=fs.C['gt'], ls='--', label='true yaw (/odom_gt)')
    ax[0].set_ylabel('heading [deg]'); ax[0].legend(loc='upper right'); ax[0].set_title('6.04 heading from the gyro')
    ax[1].plot(I[:, 0] - t0, err, color=fs.C['bad'])
    ax[1].set_ylabel('gyro - true [deg]'); ax[1].set_xlabel('simulated time [s]')
    out = save(fig, '6.04_heading')
    return dict(true_turns_deg=[float(np.degrees(np.interp(b, G[:, 0], h_gt))) for _, b in t_turn],
                turn_errors_deg=turn_err, turn_heading_error_deg=max(abs(e) for e in turn_err),
                standing_drift_deg=abs(drift), standing_s=30.0, final_error_deg=float(err[-1]), standing_gyro_z_bias_rads=float(b),
                second_half_drift_deg=at(I[-1, 0]) - at(tm), second_half_drift_bias_removed_deg=corrected), [out]


# ---- 6.05 to 6.07: camera ---------------------------------------------------------------------------------------------------
@task('6.05', at_a=True)
def t605(n):
    m = n.grab(Image, 'camera/image_raw', RELIABLE)
    fx, fy, cx, cy = camera_K(n)
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    plt.imsave(FIG / '6.05_rgb.png', img)
    msgs = n.grab(Image, 'camera/image_raw', RELIABLE, n=11)
    st = [stamp(x) for x in msgs]
    return dict(width=m.width, height=m.height, encoding=m.encoding, encoding_ok=m.encoding == 'rgb8',
                frame_id=m.header.frame_id, frame_ok=m.header.frame_id == 'camera_optical', fx=fx, fy=fy, cx=cx, cy=cy,
                fx_expected=320 / math.tan(0.6), fx_error_px=abs(fx - 320 / math.tan(0.6)), pixel_std=float(img.std()),
                rate_hz_sim=10 / (st[-1] - st[0])), ['figures/6.05_rgb.png']


@task('6.06', at_a=True)
def t606(n):
    """Five frames; the camera's height and pitch for each come from /odom_gt interpolated at that frame's stamp (the trunk
    pitch oscillates by about 0.3 deg while standing, so a ground-truth sample from another instant is not a reference)."""
    fx, fy, cx, cy = camera_K(n, 'camera/depth/camera_info')
    n.clear()
    ms = n.grab(Image, 'camera/depth/image_raw', RELIABLE, n=5)
    n.spin_for(0.3)                                             # ground truth past the last frame's stamp
    G = np.array(n.gt)
    v = np.arange(ms[0].height)
    frames = []
    for m in ms:
        t, D = stamp(m), depth_array(m)
        roll, pitch, z = (np.interp(t, G[:, 0], G[:, i]) for i in (4, 5, 3))
        h = z + (pc.rot(roll, pitch, 0.0) @ [CAM[0], 0, CAM[1]])[2]     # camera height above the floor
        pred, lvl = pc.floor_depth(v, cy, fy, h, pitch), pc.floor_depth(v, cy, fy, h)
        meas = np.nanmedian(D[:, int(cx) - 20:int(cx) + 20], axis=1)
        rows = np.isfinite(pred) & (pred >= 0.7) & (pred <= 6.0) & np.isfinite(meas)
        rel = np.abs(meas[rows] - pred[rows]) / pred[rows]
        frames.append(dict(D=D, h=h, pitch=pitch, pred=pred, meas=meas, rows=rows, err=float(np.median(rel)), max=float(rel.max()),
                           level=float(np.median(np.abs(meas[rows] - lvl[rows]) / lvl[rows]))))
    w = max(frames, key=lambda f: f['err'])                     # the figure shows the worst frame
    D, h, pred, meas, rows = w['D'], w['h'], w['pred'], w['meas'], w['rows']
    fig, ax = plt.subplots(1, 2, figsize=(7.8, 2.9), gridspec_kw=dict(width_ratios=[1.5, 1]))
    im = ax[0].imshow(D, cmap='viridis', vmin=0.3, vmax=8.0); ax[0].grid(False); ax[0].set_xticks([]); ax[0].set_yticks([])
    fig.colorbar(im, ax=ax[0], label='depth [m] (white: no return)', shrink=0.8)
    ax[0].set_title('6.06 /camera/depth/image_raw at A')
    ax[1].plot(pred[rows], v[rows], color=fs.C['gt'], ls='--', label=f'floor, h = {h:.3f} m')
    ax[1].plot(meas[rows], v[rows], color=fs.C['est'], lw=1, label='measured (median of 40 columns)')
    ax[1].invert_yaxis(); ax[1].set_xlabel('depth [m]'); ax[1].set_ylabel('image row v'); ax[1].legend(loc='lower right')
    out = save(fig, '6.06_depth')
    return dict(encoding=ms[0].encoding, encoding_ok=all(m.encoding == '32FC1' for m in ms), n_frames=len(ms),
                camera_height_m=[f['h'] for f in frames], pitch_at_stamp_deg=[math.degrees(f['pitch']) for f in frames],
                n_rows=int(rows.sum()), frame_median_rel_errors=[f['err'] for f in frames],
                level_model_median_rel_errors=[f['level'] for f in frames],
                floor_depth_median_rel_error=w['err'], floor_depth_max_rel_error=w['max'],
                nan_fraction=float(np.isnan(D).mean())), [out]


@task('6.07')
def t607(n):
    goto_lane_start()
    snap = RES / '6.07_snapshot.npz'
    subprocess.run([PY, str(P05 / 'snapshot.py'), str(snap)], check=True, timeout=300)
    z = np.load(snap)
    r, _ = obstacle.front_min(z['ranges'], float(z['angle_min']), float(z['angle_inc']), math.radians(20),
                              float(z['range_min']), float(z['range_max']))
    d = obstacle.centre_depth(z['depth'])
    x, yaw = z['pose'][0] + A[0], z['pose'][2]
    face = arena.BOXES[2][0] - arena.BOXES[2][2] / 2
    true = face - x - obstacle.FRONT * math.cos(yaw)
    return dict(true_gap_m=true, scan_range_m=r, scan_gap_m=r - obstacle.FRONT, depth_gap_m=d,
                scan_gap_error_m=abs(r - obstacle.FRONT - true), depth_gap_error_m=abs(d - true),
                yaw_deg=math.degrees(yaw)), ['results/6.07_snapshot.npz']


# ---- 6.08 to 6.12: lidar ----------------------------------------------------------------------------------------------------
@task('6.08', at_a=True)
def t608(n):
    m = n.grab(PointCloud2, 'lidar/points')
    g = n.pose()
    P = cloud_xyz(m)
    el = np.degrees(np.arctan2(P[:, 2], np.hypot(P[:, 0], P[:, 1])))
    low = np.abs(el + 15) < 0.5
    h = g[3] + LIDAR_Z
    r = np.hypot(P[low, 0], P[low, 1])
    pred = h / math.tan(math.radians(15))
    fig, ax = plt.subplots(figsize=(5.2, 4.4))
    s = ax.scatter(P[:, 0], P[:, 1], c=P[:, 2], s=0.6, cmap='viridis', vmin=-0.4, vmax=0.8, rasterized=True)
    a = np.linspace(0, 2 * math.pi, 200)
    ax.plot(pred * np.cos(a), pred * np.sin(a), color=fs.C['bad'], lw=0.8, ls='--', label=f'-15 deg beam on the floor, {pred:.2f} m')
    ax.set_aspect('equal'); ax.set_xlim(-6, 11); ax.set_ylim(-3, 9)
    ax.set_xlabel('x in lidar_link [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='upper right')
    fig.colorbar(s, ax=ax, label='z in lidar_link [m]', shrink=0.8); ax.set_title('6.08 one /lidar/points cloud at A, top view')
    out = save(fig, '6.08_cloud')
    names = [f.name for f in m.fields]
    return dict(frame_id=m.header.frame_id, frame_ok=m.header.frame_id == 'lidar_link', fields=names,
                xyz_fields_ok=all(k in names for k in 'xyz'), n_points=len(P), n_low_beam=int(low.sum()),
                lidar_height_m=h, ring_radius_pred_m=pred, ring_radius_median_m=float(np.median(r)),
                ring_radius_error_m=abs(float(np.median(r)) - pred)), [out]


@task('6.09', at_a=True)
def t609(n):
    m = n.grab(PointCloud2, 'lidar/points')
    g = n.pose()
    tf = n.tf.lookup_transform('base_link', 'lidar_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=5))
    tfp = n.tf.lookup_transform('base_footprint', 'lidar_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=5))
    tr = tf.transform.translation
    P = cloud_xyz(do_transform_cloud(m, tf))
    Pf = cloud_xyz(do_transform_cloud(m, tfp))
    rr = np.hypot(P[:, 0], P[:, 1])
    fl = (P[:, 2] < -0.1) & (rr > 1.0) & (rr < 4.0)
    zf = float(np.median(P[fl, 2]))
    fig, ax = plt.subplots(figsize=(6.4, 2.8))
    ax.scatter(P[:, 0], P[:, 2], s=0.5, color=fs.C['est'], rasterized=True, label='cloud in base_link')
    ax.axhline(-g[3], color=fs.C['gt'], ls='--', label=f'true floor, z = -{g[3]:.3f} m')
    ax.axhline(zf, color=fs.C['plan'], lw=0.8, label=f'median floor return, z = {zf:.3f} m')
    ax.set_xlim(-6, 11); ax.set_ylim(-0.5, 1.6); ax.set_xlabel('x in base_link [m]'); ax.set_ylabel('z [m]')
    ax.legend(loc='upper left', ncol=3, fontsize=7.5); ax.set_title('6.09 side view after tf2 lidar_link -> base_link')
    out = save(fig, '6.09_base_link')
    return dict(tf_translation=[tr.x, tr.y, tr.z], tf_translation_error_m=float(np.linalg.norm([tr.x, tr.y, tr.z - LIDAR_Z])),
                true_base_height_m=g[3], floor_z_base_link_m=zf, floor_z_error_m=abs(zf + g[3]),
                floor_z_base_footprint_m=float(np.median(Pf[fl, 2])), n_floor_points=int(fl.sum())), [out]


@task('6.10', at_a=True)
def t610(n):
    m, P, (fn, fd), keep = filtered_cloud(n)
    g = n.pose()
    W = to_world(P, g)
    floor, obst = W[:, 2] < 0.02, W[:, 2] > 0.10
    body = (np.abs(P[:, 0]) <= 0.40) & (np.abs(P[:, 1]) <= 0.20)
    fig, ax = plt.subplots(1, 2, figsize=(8.4, 3.6))
    for a, sel, title in ((ax[0], np.ones(len(P), bool), 'all returns'), (ax[1], keep, 'after floor and body removal')):
        a.scatter(P[sel & ~keep, 0], P[sel & ~keep, 1], s=0.4, color=fs.C['gt'], rasterized=True, label='removed')
        a.scatter(P[sel & keep, 0], P[sel & keep, 1], s=0.6, color=fs.C['est'], rasterized=True, label='kept')
        a.set_aspect('equal'); a.set_xlim(-2, 11); a.set_ylim(-2, 9); a.set_title(title); a.set_xlabel('x in base_link [m]')
    ax[0].set_ylabel('y [m]'); ax[0].legend(loc='upper right', markerscale=8)
    out = save(fig, '6.10_floor_removed')
    return dict(n_points=len(P), floor_normal=fn.tolist(), floor_offset_m=float(fd), n_floor=int(floor.sum()), n_obstacle=int(obst.sum()),
                floor_removed_frac=float((~keep[floor]).mean()), obstacle_kept_frac=float(keep[obst].mean()),
                body_points_raw=int(body.sum()), body_points_left=int((body & keep).sum()), n_kept=int(keep.sum())), [out]


def clusters_world(n):
    m, P, _, keep = filtered_cloud(n)
    g = n.pose()
    K = P[keep]
    lab = pc.cluster(K[:, :2], 0.15, 5)
    return K, lab, to_world(K, g, full=False), g


def draw_arena(ax):
    for x, y, sx, sy, *_ in arena.BOXES + [w + (0,) for w in arena.WALLS]:
        ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, fill=False, ec=fs.C['gt'], lw=0.8))
    for x, y, r, *_ in arena.CYLS:
        ax.add_patch(plt.Circle((x, y), r, fill=False, ec=fs.C['gt'], lw=0.8))


@task('6.11')
def t611(n):
    goto_lane_start()
    K, lab, W, g = clusters_world(n)
    roi = []
    for c in range(lab.max() + 1):
        cx, cy = K[lab == c, :2].mean(0)
        if 0.3 <= cx <= 4.0 and abs(cy) <= 1.0:
            roi.append(c)
    face = arena.BOXES[2][0] - arena.BOXES[2][2] / 2
    best = max(roi, key=lambda c: (lab == c).sum()) if roi else None
    Q = W[lab == best] if best is not None else np.zeros((0, 3))
    fig, ax = plt.subplots(figsize=(5.4, 3.6))
    draw_arena(ax)
    ax.scatter(W[:, 0], W[:, 1], s=0.4, color=fs.C['gt'], rasterized=True)
    ax.scatter(Q[:, 0], Q[:, 1], s=1.5, color=fs.C['est'], label='detected cluster', rasterized=True)
    ax.plot(g[1], g[2], 'o', color=fs.C['plan'], label='robot')
    ax.set_aspect('equal'); ax.set_xlim(-1.5, 4); ax.set_ylim(-5.2, -0.5); ax.legend(loc='upper right')
    ax.set_xlabel('world x [m]'); ax.set_ylabel('world y [m]'); ax.set_title('6.11 one obstacle: the pallet face')
    out = save(fig, '6.11_one_obstacle')
    width = float(Q[:, 1].max() - Q[:, 1].min()) if len(Q) else math.nan
    fx = float(np.median(Q[:, 0])) if len(Q) else math.nan
    return dict(n_clusters_total=int(lab.max() + 1), n_clusters_in_roi=len(roi), cluster_points=len(Q),
                face_x_m=fx, face_x_error_m=abs(fx - face), width_m=width, width_error_m=abs(width - arena.BOXES[2][3]),
                robot_xy=[g[1], g[2]]), [out]


@task('6.12', at_a=True)
def t612(n):
    K, lab, W, g = clusters_world(n)
    obst = [('box', *b[:4]) for b in arena.BOXES] + [('cyl', *c[:3]) for c in arena.CYLS]
    walls = [('box', *w) for w in arena.WALLS]
    names = [f'box{i}' for i in range(len(arena.BOXES))] + [f'pillar{i}' for i in range(len(arena.CYLS))]
    lidar_xy = (g[1], g[2])
    hits = pc.visible(lidar_xy, obst + walls)[:len(obst)]
    vis = [i for i, h in enumerate(hits) if h >= 3]
    matched, false, dists = set(), 0, {}
    for c in range(lab.max() + 1):
        q = W[lab == c, :2]
        md = [float(np.median(pc.footprint_dist(q, s))) for s in obst + walls]
        k = int(np.argmin(md))
        if md[k] >= 0.15:
            false += 1
        elif k < len(obst):
            matched.add(k); dists[names[k]] = min(md[k], dists.get(names[k], 9))
    fig, ax = plt.subplots(figsize=(6.0, 4.8))
    draw_arena(ax)
    cols = [fs.C['est'], fs.C['est2'], fs.C['plan'], fs.C['good'], fs.C['bad']]
    for c in range(lab.max() + 1):
        q = W[lab == c]
        ax.scatter(q[:, 0], q[:, 1], s=0.6, color=cols[c % len(cols)], rasterized=True)
    for i in vis:
        s = obst[i]
        ax.annotate(names[i] + (' found' if i in matched else ' MISSED'), (s[1], s[2]), fontsize=7.5, ha='center',
                    color=fs.C['ink'] if i in matched else fs.C['bad'])
    ax.plot(*lidar_xy, 'o', color=fs.C['plan'])
    ax.set_aspect('equal'); ax.set_xlim(-6.4, 6.4); ax.set_ylim(-5.4, 5.4); ax.set_xlabel('world x [m]'); ax.set_ylabel('world y [m]')
    ax.set_title(f'6.12 {lab.max() + 1} clusters from A; colour = cluster')
    out = save(fig, '6.12_several')
    return dict(visible=[names[i] for i in vis], rays_per_obstacle=dict(zip(names, hits.tolist())),
                detected=sorted(names[i] for i in matched), recall_visible=len(matched & set(vis)) / max(1, len(vis)),
                false_clusters=false, n_clusters=int(lab.max() + 1), median_boundary_dist_m=dists,
                max_median_boundary_dist_m=max(dists.values()) if dists else math.nan), [out]


# ---- 6.13 to 6.15: stop and missing data (p05 nodes) ------------------------------------------------------------------------
@task('6.13')
def t613(n):
    out = RES / '6.13_p05_runs'
    for f in out.glob('**/*.csv'):
        f.unlink()
    sense = proc([P05 / 'sense.py'])
    try:
        r = subprocess.run([PY, str(P05 / 'measure_stops.py'), '--speeds', '0.2', '--runs', '3', '--out', str(out)],
                           capture_output=True, text=True, timeout=3600)
    finally:
        stop_proc(sense)
    print(r.stdout, flush=True)
    rows = list(csv.DictReader(open(out / 'stop.csv')))
    gaps = [float(x['gap_final']) for x in rows]
    return dict(n_runs=len(rows), gap_final_m=gaps, coast_m=[float(x['coast']) for x in rows],
                max_abs_gap_error_m=max(abs(x - 0.5) for x in gaps) if gaps else math.nan,
                min_final_gap_m=min(gaps) if gaps else math.nan, stdout=r.stdout.strip().splitlines()), ['results/6.13_p05_runs/stop.csv']


def front_gap(g, face):
    return face - g[1] - obstacle.FRONT * math.cos(g[6])


@task('6.14')
def t614(n, video=False):
    goto_lane_start()
    sense = proc([P05 / 'sense.py'])
    n.grab(Range, 'front_range', timeout=60)
    rec = proc([ROOT / 'projects/common/record_cam.py', MEDIA / '6.14_new_obstacle.mp4', '--topic', '/chase/cam/image_raw',
                '--width', '640']) if video else None
    n.spin_for(1.0); n.clear()
    stop = proc([P05 / 'stop_at_obstacle.py', '--ros-args', '-p', 'speed:=0.2'])
    while not any(c[1] > 0 for c in n.cmd):
        rclpy.spin_once(n, timeout_sec=0.05)
    n.spin_for(1.0)
    bx, by, sx, sy, h = worlds.NEW_BOX
    req = SpawnEntity.Request(name='new_box', xml=(HERE / 'worlds/new_box.sdf').read_text())
    req.initial_pose = Pose(); req.initial_pose.position.x, req.initial_pose.position.y = bx, by
    res = n.call(n.spawn, req)
    t_spawn, g_spawn = n.now(), n.gt[-1]
    while stop.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.stand(2.0)
    if rec:
        n.spin_for(1.0); stop_proc(rec)
    out_stop = stop.stdout.read()
    stop_proc(sense)
    face_box, face_pallet = bx - sx / 2, arena.BOXES[2][0] - arena.BOXES[2][2] / 2
    g_end = n.gt[-1]
    n.call(n.delete, DeleteEntity.Request(name='new_box'))
    G = np.array(n.gt)
    fig, ax = plt.subplots(figsize=(6.4, 2.8))
    gap = face_box - G[:, 1] - obstacle.FRONT * np.cos(G[:, 6])
    ax.plot(G[:, 0] - t_spawn, gap, color=fs.C['gt'], label='true gap to the new box')
    R = np.array(n.rng)
    if len(R):
        ax.plot(R[:, 0] - t_spawn, R[:, 2] - obstacle.FRONT, '.', ms=3, color=fs.C['est'], label='/front_range - 0.28 m')
    ax.axvline(0, color=fs.C['plan'], lw=1, label='box spawned'); ax.axhline(0.5, color=fs.C['muted'], ls='--', lw=0.8)
    ax.set_ylim(0, 3); ax.set_xlabel('simulated time from spawn [s]'); ax.set_ylabel('gap [m]'); ax.legend(loc='upper right')
    ax.set_title('6.14 stop before an obstacle that was not there at the start')
    outs = [save(fig, '6.14_new_obstacle')]
    if rec:
        outs.append('media/6.14_new_obstacle.mp4')
    gf = front_gap(g_end, face_box)
    return dict(spawn_ok=bool(res.success), gap_at_spawn_m=front_gap(g_spawn, face_box), final_gap_m=gf,
                abs_gap_error_m=abs(gf - 0.5), gap_to_pallet_at_end_m=front_gap(g_end, face_pallet),
                stop_line=[l for l in out_stop.splitlines() if 'STOP' in l or 'TIMEOUT' in l]), outs


@task('6.15')
def t615(n):
    goto_lane_start()
    # (a) no readings at all: the node must wait, not walk
    n.clear()
    p = proc([P05 / 'stop_at_obstacle.py', '--ros-args', '-p', 'speed:=0.2'])
    n.spin_for(3.0)
    G = np.array(n.gt); stop_proc(p)
    blind = float(np.hypot(G[-1, 1] - G[0, 1], G[-1, 2] - G[0, 2]))
    nonzero_blind = sum(1 for c in n.cmd if abs(c[1]) > 0)
    # (b) readings stop while walking
    sense = proc([P05 / 'sense.py'])
    n.grab(Range, 'front_range', timeout=60)
    n.clear()
    p = proc([P05 / 'stop_at_obstacle.py', '--ros-args', '-p', 'speed:=0.2'])
    while not any(c[1] > 0 for c in n.cmd):
        rclpy.spin_once(n, timeout_sec=0.05)
    n.spin_for(2.0)
    t_kill = n.now(); stop_proc(sense)
    while p.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.stand(2.0)
    out = p.stdout.read()
    G, C, R = np.array(n.gt), np.array(n.cmd), np.array(n.rng)
    m = re.search(r'TIMEOUT t=(\S+): no reading for (\S+) s', out)
    t_last = R[:, 1].max()                                   # stamp of the last scan that reached us
    t_zero = float(m.group(1)) if m else math.nan            # the node's own clock when it sent the zero
    i_last = np.argmin(np.abs(G[:, 0] - t_last))
    walked = float(np.hypot(G[-1, 1] - G[i_last, 1], G[-1, 2] - G[i_last, 2]))
    fig, ax = plt.subplots(figsize=(6.4, 2.8))
    v = np.hypot(np.gradient(G[:, 1], G[:, 0]), np.gradient(G[:, 2], G[:, 0]))
    t0 = t_last
    ax.plot(G[:, 0] - t0, v, color=fs.C['gt'], lw=1, label='true speed')
    ax.step(C[:, 0] - t0, C[:, 1], where='post', color=fs.C['plan'], lw=1, label='/cmd_vel x')
    ax.plot(R[:, 1] - t0, np.full(len(R), 0.3), '|', color=fs.C['est'], ms=6, label='/front_range (scan stamps)')
    ax.axvline(0, color=fs.C['bad'], lw=0.8, label='last scan')
    ax.set_xlabel('simulated time from the last scan [s]'); ax.set_ylabel('m/s'); ax.set_ylim(-0.02, 0.33)
    ax.legend(loc='center right', fontsize=7.5); ax.set_title('6.15 sensor stops while walking: the node stops the robot')
    fig_out = save(fig, '6.15_missing')
    return dict(walked_without_readings_m=blind, nonzero_cmds_without_readings=nonzero_blind,
                timeout_line=any('TIMEOUT' in l for l in out.splitlines()), node_output=out.strip().splitlines(),
                zero_after_last_reading_s=float(m.group(2)) if m else math.nan, zero_after_last_scan_stamp_s=t_zero - t_last,
                walked_after_last_reading_m=walked), [fig_out]


# ---- 6.16 and 6.17: raised platform from depth -------------------------------------------------------------------------------
def platform_points(n):
    ms = n.grab(Image, 'camera/depth/image_raw', RELIABLE, n=3)
    g = n.pose()
    fx, fy, cx, cy = camera_K(n, 'camera/depth/camera_info')
    D = np.nanmedian(np.stack([depth_array(m) for m in ms]), axis=0)
    P = pc.backproject(D, fx, fy, cx, cy, zmax=4.0)
    return D, P, pc.platform_from_points(P), g, (fx, fy, cx, cy)


@task('6.16', world='platform', at_a=True)
def t616(n):
    D, P, r, g, K = platform_points(n)
    fx, fy, cx, cy = K
    mask = np.zeros(D.shape, int)
    h, w = D.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    ok = np.isfinite(D) & (D > 0) & (D <= 4.0)
    lab = np.zeros(ok.sum(), int); lab[r['floor_mask']] = 1; lab[r['top_mask']] = 2
    mask[ok] = lab
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.imshow(D, cmap='Greys_r', vmin=0.3, vmax=8.0)
    ov = np.zeros((h, w, 4)); ov[mask == 1] = (0.04, 0.45, 0.66, 0.35); ov[mask == 2] = (0.79, 0.52, 0.06, 0.6)
    ax.imshow(ov); ax.grid(False); ax.set_xticks([]); ax.set_yticks([])
    ax.set_title(f'6.16 depth at A: floor plane (blue), raised plane {r["height"] * 100:.1f} cm higher (amber)')
    out = save(fig, '6.16_platform_depth')
    return dict(detected=r['detected'], top_inliers=int(r['top_mask'].sum()), floor_inliers=int(r['floor_mask'].sum()),
                height_m=r['height'], top_normal_angle_deg=r['angle_deg'], n_points=len(P)), [out]


@task('6.17', world='platform', at_a=True)
def t617(n):
    D, P, r, g, K = platform_points(n)
    nf, df = r['floor']
    T = pc.ground_frame(P[r['top_mask']], nf, df)
    _, x, y, z, roll, pitch, yaw, _ = g
    cxw, cyw = x + CAM[0] * math.cos(yaw), y + CAM[0] * math.sin(yaw)
    X = cxw + math.cos(yaw) * T[:, 0] - math.sin(yaw) * T[:, 1]
    Y = cyw + math.sin(yaw) * T[:, 0] + math.cos(yaw) * T[:, 1]
    px, py, sx, sy, ph = worlds.PLATFORM
    est = dict(near_x=np.percentile(X, 1), far_x=np.percentile(X, 99), right_y=np.percentile(Y, 1), left_y=np.percentile(Y, 99))
    true = dict(near_x=px - sx / 2, far_x=px + sx / 2, right_y=py - sy / 2, left_y=py + sy / 2)
    err = {k: float(est[k] - true[k]) for k in est}
    ok = np.isfinite(D) & (D > 0) & (D <= 4.0)                  # backproject's mask, row-major like P
    rows = np.nonzero(ok)[0][r['top_mask']]                     # image row of every top point
    far = sorted((np.median(X[rows == v]), int((rows == v).sum())) for v in np.unique(rows))[-2:]
    fig, ax = plt.subplots(figsize=(5.0, 4.0))
    ax.scatter(X[::5], Y[::5], s=0.5, color=fs.C['plan'], rasterized=True, label='top-plane points (every 5th)')
    ax.add_patch(plt.Rectangle((true['near_x'], true['right_y']), sx, sy, fill=False, ec=fs.C['gt'], lw=1.5, ls='--', label='true platform'))
    ax.add_patch(plt.Rectangle((est['near_x'], est['right_y']), est['far_x'] - est['near_x'], est['left_y'] - est['right_y'],
                               fill=False, ec=fs.C['est'], lw=1.2, label='estimated edges'))
    ax.plot(cxw, cyw, 'o', color=fs.C['ink'], label='camera')
    ax.set_aspect('equal'); ax.set_xlim(-4.5, -1.3); ax.set_ylim(-4.6, -2.4); ax.legend(loc='upper left', fontsize=7.5)
    ax.set_xlabel('world x [m]'); ax.set_ylabel('world y [m]')
    ax.set_title(f'6.17 height {r["height"] * 100:.1f} cm (true {ph * 100:.0f} cm)')
    out = save(fig, '6.17_platform_edges')
    return dict(height_m=r['height'], true_height_m=ph, height_error_m=abs(r['height'] - ph), edges_est_m={k: float(v) for k, v in est.items()},
                edges_true_m=true, edge_errors_m=err, max_edge_error_m=max(abs(e) for e in err.values()),
                far_x_error_m=err['far_x'], far_row_spacing_m=float(far[1][0] - far[0][0]), far_row_points=far[1][1],
                floor_camera_height_m=float(df), camera_height_gt_m=z + CAM[1]), [out]


# ---- results ----------------------------------------------------------------------------------------------------------------
def rule_text(rule):
    k, v = next(iter(rule.items()))
    return {'max': '<=', 'min': '>=', 'equals': '=='}[k] + f' {v}'


def passes(value, rule):
    k, v = next(iter(rule.items()))
    if isinstance(value, float) and math.isnan(value):
        return False
    return value <= v if k == 'max' else value >= v if k == 'min' else value == v


def jsonable(x):
    if isinstance(x, dict):
        return {k: jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [jsonable(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return None if math.isnan(x) else float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def run(n, tid, video):
    """Runs the task acc['runs'] times (default 1), each repeat on a fresh simulator; verified only when every run passes."""
    f, world, at_a = TASKS[tid]
    acc = ACC[tid]
    n_runs = acc.get('runs', 1)
    runs, t0 = [], time.time()
    for k in range(n_runs):
        if k:
            start_stack(n, world)
        else:
            ensure(n, world, at_a)
        metrics, outputs = f(n, video) if tid == '6.14' else f(n)
        n.send(0.0)
        checks = {c: dict(value=metrics.get(c), rule=rule_text(r), passed=passes(metrics.get(c), r)) for c, r in acc['checks'].items()}
        runs.append((metrics, outputs, checks))
        fmt = lambda v: f'{v:.4g}' if isinstance(v, float) else repr(v)
        ok = all(c['passed'] for c in checks.values())
        print(f'{tid} run {k + 1}/{n_runs} {"passed" if ok else "failed"}: '
              + ', '.join(f'{c}={fmt(x["value"])}' for c, x in checks.items()), flush=True)
    n_pass = sum(all(c['passed'] for c in ch.values()) for _, _, ch in runs)
    metrics, outputs, checks = runs[-1]                        # metrics and figures of the last run; every run's checks below
    rule = '; '.join(f'{c} {x["rule"]}' for c, x in checks.items())
    if n_runs > 1:
        rule += f'; in each of {n_runs} runs, each on a fresh simulator'
    res = dict(id=tid, title=acc['title'], status='verified' if n_pass == n_runs else 'failed',
               metrics=metrics, acceptance=f'{rule}. Basis: {acc["basis"]}', checks=checks,
               runs_passed=n_pass, runs_total=n_runs,
               per_run=[{k: v for k, v in m.items() if isinstance(v, (int, float))} for m, _, _ in runs],   # every scalar metric
               outputs=outputs, command=f'python3 tasks/s06_sensors-and-perception/run.py {tid}',
               world=world, wall_s=round(time.time() - t0, 1))
    (RES / f'{tid}.json').write_text(json.dumps(jsonable(res), indent=2) + '\n')
    print(f'{tid} {res["status"]}: {n_pass} of {n_runs} runs passed', flush=True)


ORDER = ['6.01', '6.03', '6.05', '6.06', '6.08', '6.09', '6.10', '6.12', '6.02', '6.04', '6.07', '6.11', '6.13', '6.14', '6.15',
         '6.16', '6.17']

if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    video = '--video' in sys.argv
    ids = ORDER if args == ['all'] else args
    if not ids or any(i not in TASKS for i in ids):
        sys.exit(__doc__)
    RES.mkdir(exist_ok=True); FIG.mkdir(exist_ok=True); MEDIA.mkdir(exist_ok=True)
    rclpy.init()
    node = Probe()
    try:
        for i in ids:
            run(node, i, video)
    finally:
        node.send(0.0)
