#!/usr/bin/env python3
"""Stage 1 acceptance tests: import and inspect the Go2.
usage (after `source env.sh`, from anywhere):  /usr/bin/python3 run.py <task-id>|all      e.g. /usr/bin/python3 run.py 1.04
Each task starts what it needs (robot_state_publisher, RViz, Gazebo) and stops it again, then writes results/<id>.json with the
measured metrics and the verdict against acceptance.json. Tasks 1.02, 1.05 and 1.09 render (RViz window, chase camera video) and
take /tmp/go2_render.lock while they do."""
import contextlib
import fcntl
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from typing import Any, cast

import numpy as np
import yaml

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path[:0] = [str(HERE), str(REPO / 'modules')]
from urdf_fk import Urdf, quat_matrix, rotation_angle  # noqa: E402
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

import rclpy  # noqa: E402
import rclpy.executors  # noqa: E402
import rclpy.time  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

ACC = json.loads((HERE / 'acceptance.json').read_text())
FIG, MEDIA, RES, UDIR = HERE / 'figures', HERE / 'media', HERE / 'results', HERE / 'urdf'
LOG = pathlib.Path(f"/tmp/s01_logs_{os.environ.get('ROS_DOMAIN_ID', '42')}")
XACRO = REPO / 'src/go2_nav_sim/urdf/go2_nav.xacro'
RVIZ = REPO / 'src/go2_nav_sim/rviz/go2.rviz'
CFG = REPO / 'src/third_party/unitree-go2-ros2/robots/configs/go2_config/config'
SPAWN = (-4.5, -3.5)                                   # sim.launch.py START_X, START_Y
JOINTS = [f'{l}_{j}_joint' for l in ('lf', 'rf', 'lh', 'rh') for j in ('hip', 'upper_leg', 'lower_leg')]   # ros_control.yaml order
SLIDER_DEFAULT = {n: (-1.78023 if 'lower' in n else 0.0) for n in JOINTS}   # joint_state_publisher start pose (docs/06 Task 2.3)
RIG_HOLD = {n: {'hip': 0.0, 'upper': 0.8, 'lower': -1.5}[n.split('_')[1]] for n in JOINTS}
TASKS, PROCS = {}, []


def task(tid):
    def wrap(f):
        TASKS[tid] = f
        return f
    return wrap


# ---- processes ----------------------------------------------------------------------------------------------------------------
def start(cmd, name):
    LOG.mkdir(exist_ok=True)
    f = open(LOG / f'{name}.log', 'w')
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    PROCS.append(p)
    return p


def stop_all(gazebo=False):
    for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        live = [p for p in PROCS if p.poll() is None]
        for p in live:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(p.pid, sig)
        t = time.time() + wait
        while time.time() < t and any(p.poll() is None for p in live):
            time.sleep(0.2)
    PROCS.clear()
    if gazebo:                                         # backstop for gzserver children that left the process group
        subprocess.run(['bash', str(REPO / 'scripts/stop.sh')], check=False)


@contextlib.contextmanager
def render_lock():
    with open('/tmp/go2_render.lock', 'w') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        yield


def expand(path, *args):
    return subprocess.run(['xacro', str(path), *args], capture_output=True, text=True, check=True).stdout


def check_urdf(text):
    p = LOG / 'check.urdf'
    LOG.mkdir(exist_ok=True)
    p.write_text(text)
    r = subprocess.run(['check_urdf', str(p)], capture_output=True, text=True)
    return r.returncode == 0 and 'Successfully Parsed XML' in r.stdout, r.stdout


def rsp(urdf_text, name='rsp'):
    # A params file, not -p robot_description:=...: that value is parsed as YAML and a ': ' in an XML comment aborts the node.
    LOG.mkdir(exist_ok=True)
    f = LOG / f'{name}_params.yaml'
    f.write_text(cast(str, yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': urdf_text}}})))  # PyYAML has no type hints
    return start(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(f)], name)


# ---- ROS probe ------------------------------------------------------------------------------------------------------------------
class Probe:
    """One rclpy node spun in a thread: tf2 buffer, /joint_states and /odom_gt recorders, publishers."""

    def __init__(self, sim_time=False):
        if not rclpy.ok():
            rclpy.init()
        self.n = Node('s01_probe', parameter_overrides=[Parameter('use_sim_time', value=sim_time)])
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self.n)
        self.js, self.odom, self.image, self.rec = [], [], None, False
        self.n.create_subscription(JointState, '/joint_states', self._js, 200)
        self.n.create_subscription(Odometry, '/odom_gt', self._odom, 50)
        self.n.create_subscription(Image, '/chase/cam/image_raw', self._img, qos_profile_sensor_data)
        self.pub_js = self.n.create_publisher(JointState, '/joint_states', 10)
        self.pub_traj = self.n.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)
        self.pub_cmd = self.n.create_publisher(Twist, '/cmd_vel', 10)
        self.cli = self.n.create_client(ListControllers, '/controller_manager/list_controllers')
        self.ex = rclpy.executors.MultiThreadedExecutor()
        self.ex.add_node(self.n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def _js(self, m):
        if self.rec:
            self.js.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, time.time(), list(m.name),
                            list(m.position), list(m.velocity), list(m.effort)))

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom.append((self.now(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def _img(self, m):
        self.image = m

    def now(self):
        return self.n.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, dt):
        t0 = self.now()
        while self.now() - t0 < dt:
            time.sleep(0.02)

    def close(self):
        self.ex.shutdown()
        self.n.destroy_node()

    def lookup(self, base, link, stamp=None, timeout=3.0):
        t = rclpy.time.Time() if stamp is None else rclpy.time.Time.from_msg(stamp)
        end = time.time() + timeout
        while True:                                    # retry: an unknown frame raises at once instead of waiting
            try:
                tf = self.buf.lookup_transform(base, link, t)
                break
            except Exception:
                if time.time() > end:
                    raise
                time.sleep(0.05)
        p, r = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = quat_matrix(r.x, r.y, r.z, r.w), (p.x, p.y, p.z)
        return T

    def publish_q(self, q):
        m = JointState()
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.name, m.position = list(q), [float(v) for v in q.values()]
        self.pub_js.publish(m)
        return m.header.stamp

    def pose_and_lookup(self, q, links, base='base_link'):
        """Publish q until robot_state_publisher has broadcast it, then return {link: 4x4 in base} at that stamp."""
        for _ in range(20):
            stamp = self.publish_q(q)
            try:
                return {l: self.lookup(base, l, stamp, timeout=0.5) for l in links}
            except Exception:                          # tf2 LookupException / ExtrapolationException until the frame arrives
                continue
        raise RuntimeError(f'no transform for {links} after 20 tries')

    def send_traj(self, q, T=1.0):
        m = JointTrajectory()
        m.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(q[n]) for n in JOINTS])
        pt.time_from_start.sec, pt.time_from_start.nanosec = int(T), int((T % 1) * 1e9)
        m.points = [pt]
        self.pub_traj.publish(m)

    def active_controllers(self, timeout=240):
        t = time.time() + timeout
        while time.time() < t:
            if self.cli.wait_for_service(timeout_sec=1.0):
                fut = self.cli.call_async(ListControllers.Request())
                t1 = time.time() + 5
                while not fut.done() and time.time() < t1:
                    time.sleep(0.05)
                if fut.done() and (res := fut.result()) is not None:
                    act = sorted(c.name for c in res.controller if c.state == 'active')
                    if len(act) >= 2:
                        return act
            time.sleep(1.0)
        return []


def wait_for(cond, timeout, what):
    t = time.time() + timeout
    while time.time() < t:
        if cond():
            return
        time.sleep(0.2)
    raise TimeoutError(what)


# ---- RViz -----------------------------------------------------------------------------------------------------------------------
def rviz_window(timeout=60):
    t = time.time() + timeout
    while time.time() < t:
        out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
        m = re.search(r'(0x[0-9a-f]+) "[^"]*RViz[^"]*": \([^)]*\) +(\d+)x(\d+)', out)
        if m and int(m.group(2)) > 400:
            return m.group(1)
        time.sleep(1)
    raise TimeoutError('no RViz window')


def screenshot(win, out, width=1100):
    raw = LOG / 'shot.png'
    subprocess.run(['import', '-window', win, str(raw)], check=True)
    subprocess.run(['convert', str(raw), '-resize', f'{width}x', '-strip', '-colors', '256', str(out)], check=True)
    from PIL import Image as PImage
    return float(np.asarray(PImage.open(raw).convert('L'), float).std())


# ---- results --------------------------------------------------------------------------------------------------------------------
def rule_text(r):
    return ' and '.join(f'{k} {v}' for k, v in r.items())


def passes(v, r):
    if v is None:
        return False
    return all({'max': lambda: v <= x, 'min': lambda: v >= x, 'equals': lambda: v == x}[k]() for k, x in r.items())


def write_result(tid, metrics, outputs=()):
    a = ACC[tid]
    checks = {k: dict(value=metrics.get(k), rule=rule_text(r), passed=passes(metrics.get(k), r)) for k, r in a['checks'].items()}
    out = dict(id=tid, title=a['title'], status='verified' if all(c['passed'] for c in checks.values()) else 'failed',
               metrics=metrics, acceptance='; '.join(f'{k} {rule_text(r)}' for k, r in a['checks'].items()),
               checks=checks, command=f'python run.py {tid}', outputs=list(outputs), date=time.strftime('%Y-%m-%d %H:%M'))
    RES.mkdir(exist_ok=True)
    (RES / f'{tid}.json').write_text(json.dumps(out, indent=2, allow_nan=False) + '\n')
    print(f"{tid} {out['status']}: " + ', '.join(f"{k}={c['value']}" for k, c in checks.items()))
    return out


def finite_fraction(rows):
    a = np.array([r if len(r) else [np.nan] for r in rows], float)
    return float(np.isfinite(a).mean())


# ---- 1.01 -----------------------------------------------------------------------------------------------------------------------
@task('1.01')
def t101():
    text = expand(XACRO)
    ok, out = check_urdf(text)
    u = Urdf(text)
    return dict(check_urdf_parsed=ok, check_urdf_head=out.splitlines()[:3], root_link=u.root, n_links=len(u.links),
                n_revolute=sum(j['type'] == 'revolute' for j in u.joints.values()),
                n_fixed=sum(j['type'] == 'fixed' for j in u.joints.values())), []


# ---- 1.02 -----------------------------------------------------------------------------------------------------------------------
@task('1.02')
def t102():
    text = expand(XACRO)
    u = Urdf(text)
    rsp(text)
    start(['ros2', 'run', 'joint_state_publisher', 'joint_state_publisher'], 'jsp')
    pr = Probe()
    try:
        resolved = []
        for l in u.links:
            with contextlib.suppress(Exception):
                pr.lookup('base_link', l, timeout=10.0)
                resolved.append(l)
        with render_lock():
            t0 = time.time()
            rv = start(['rviz2', '-d', str(RVIZ), '-f', 'base_link'], 'rviz')
            win = rviz_window()
            time.sleep(8)                               # let the meshes load and the first frames render
            std = screenshot(win, FIG / '1-02_rviz.png')
            while rv.poll() is None and time.time() - t0 < 12.0:  # observe at least the 10 s the check asks for
                time.sleep(0.2)
            alive = time.time() - t0
    finally:
        pr.close()
        stop_all()
    return dict(links_in_urdf=len(u.links), links_resolved_in_tf=len(resolved),
                missing=sorted(set(u.links) - set(resolved)), rviz_alive_s=round(alive, 1),
                screenshot_pixel_std=round(std, 1)), ['figures/1-02_rviz.png']


# ---- 1.03 -----------------------------------------------------------------------------------------------------------------------
@task('1.03')
def t103():
    u = Urdf(expand(XACRO))
    act = u.actuated()
    jmap = cast(dict[str, Any], yaml.safe_load(open(CFG / 'joints/joints.yaml')))['/**']['ros__parameters']['joints_map']  # PyYAML has no type hints
    champ = {n for leg in jmap.values() for n in leg[:3]}
    rows = [dict(name=n, type=u.joints[n]['type'], parent=u.joints[n]['parent'], child=u.joints[n]['child'],
                 axis=u.joints[n]['axis'].tolist(), **u.joints[n]['limit']) for n in act]
    ax = lambda n: tuple(u.joints[n]['axis'])
    lim = lambda n: u.joints[n]['limit']
    return dict(n_actuated=len(act), joints=rows, names_match_champ_map=set(act) == champ,
                hip_axes_x=sum(ax(n) == (1, 0, 0) for n in act if '_hip_' in n),
                thigh_knee_axes_y=sum(ax(n) == (0, 1, 0) for n in act if '_hip_' not in n),
                joints_with_valid_limits=sum(l['lower'] < l['upper'] and l['effort'] > 0 and l['velocity'] > 0
                                             for l in map(lim, act)),
                left_right_limits_equal=all(lim(n) == lim(n.replace('l', 'r', 1)) for n in act if n[0] == 'l')), []


# ---- 1.04 -----------------------------------------------------------------------------------------------------------------------
@task('1.04')
def t104():
    text = expand(XACRO)
    u = Urdf(text)
    J, foot = 'lf_upper_leg_joint', 'lf_foot_link'
    angles = np.round(np.linspace(-0.6, 1.4, 11), 3)
    rsp(text)
    pr = Probe()
    rows = []
    try:
        for a in angles:
            q = dict(SLIDER_DEFAULT, **{J: float(a)})
            T = pr.pose_and_lookup(q, [foot, 'lf_upper_leg_link'], base='base_link')
            rel = pr.lookup('lf_hip_link', 'lf_upper_leg_link')          # latest = this pose
            Tf = u.fk(q, foot)
            R_joint = rel[:3, :3] @ np.linalg.inv(u.fk({J: a}, 'lf_upper_leg_link', base='lf_hip_link')[:3, :3])
            rows.append(dict(angle=float(a), foot_tf=T[foot][:3, 3].tolist(), foot_fk=Tf[:3, 3].tolist(),
                             foot_err=float(np.linalg.norm(T[foot][:3, 3] - Tf[:3, 3])), rot_err=rotation_angle(R_joint)))
    finally:
        pr.close()
        stop_all()
    x = np.array([r['foot_tf'] for r in rows])
    fk = np.array([r['foot_fk'] for r in rows])
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.0))
    th = np.linspace(-0.6, 1.4, 200)
    line = np.array([u.fk(dict(SLIDER_DEFAULT, **{J: t}), foot)[:3, 3] for t in th])
    for i, (k, lab) in enumerate(((0, 'foot x [m]'), (2, 'foot z [m]'))):
        ax[i].plot(th, line[:, k], color=fs.C['plan'], label='numpy FK from the URDF')
        ax[i].plot(angles, x[:, k], 'o', ms=4, mfc='none', color=fs.C['est'], label='tf2, robot_state_publisher')
        ax[i].set_xlabel(f'{J} [rad]'); ax[i].set_ylabel(lab)
    ax[0].legend(loc='lower left')
    ax[0].set_title('1.04 lf foot in base_link while one joint moves')
    fs.save(fig, FIG / '1-04_one_joint')
    i0 = int(np.argmin(np.abs(angles)))
    return dict(joint=J, angles_rad=angles.tolist(), foot_max_error_m=float(max(r['foot_err'] for r in rows)),
                joint_rotation_max_error_rad=float(max(r['rot_err'] for r in rows)),
                foot_moves_backward_for_positive_angle=bool(x[i0 + 1, 0] < x[i0, 0]),   # first step up from 0 rad
                foot_x_range_m=[float(x[:, 0].min()), float(x[:, 0].max())], fk_vs_tf_rms_m=float(np.sqrt(((x - fk) ** 2).mean()))
                ), ['figures/1-04_one_joint.png']


# ---- 1.05 -----------------------------------------------------------------------------------------------------------------------
@task('1.05')
def t105():
    text = expand(XACRO)
    u = Urdf(text)
    leg = ['lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint']
    links = ['lf_hip_link', 'lf_upper_leg_link', 'lf_lower_leg_link', 'lf_foot_link']
    rng = np.random.default_rng(1)
    confs = [{n: float(rng.uniform(u.joints[n]['limit']['lower'], u.joints[n]['limit']['upper'])) for n in leg} for _ in range(25)]
    rsp(text)
    pr = Probe()
    errs, feet = [], []
    try:
        for c in confs:
            q = dict(SLIDER_DEFAULT, **c)
            T = pr.pose_and_lookup(q, links)
            e = {l: float(np.linalg.norm(T[l][:3, 3] - u.fk(q, l)[:3, 3])) for l in links}
            errs.append(e)
            feet.append((T['lf_foot_link'][:3, 3], u.fk(q, 'lf_foot_link')[:3, 3]))
        shot = dict(SLIDER_DEFAULT, lf_hip_joint=0.6, lf_upper_leg_joint=-0.4, lf_lower_leg_joint=-0.9)
        with render_lock():
            start(['rviz2', '-d', str(HERE / 'params/leg_view.rviz'), '-f', 'base_link'], 'rviz')
            win = rviz_window()
            t = time.time() + 8
            while time.time() < t:                     # keep publishing: RViz needs the leg frames at the time it renders
                pr.publish_q(shot)
                time.sleep(0.1)
            screenshot(win, FIG / '1-05_rviz_leg.png')
    finally:
        pr.close()
        stop_all()
    f = np.array([a for a, _ in feet]), np.array([b for _, b in feet])
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.2))
    for i, (a, b, la, lb) in enumerate(((0, 2, 'x', 'z'), (1, 2, 'y', 'z'))):
        ax[i].plot(f[1][:, a], f[1][:, b], 'o', ms=7, mfc='none', color=fs.C['plan'], label='numpy FK')
        ax[i].plot(f[0][:, a], f[0][:, b], '.', ms=4, color=fs.C['est'], label='tf2')
        ax[i].set_xlabel(f'foot {la} in base_link [m]'); ax[i].set_ylabel(f'foot {lb} [m]'); ax[i].set_aspect('equal')
    fig.legend(*ax[0].get_legend_handles_labels(), loc='lower center', ncol=2, bbox_to_anchor=(0.5, -0.08))
    ax[0].set_title('1.05 lf foot, 25 random in-limit leg poses')
    fs.save(fig, FIG / '1-05_leg_poses')
    return dict(n_configurations=len(confs), foot_max_error_m=max(e['lf_foot_link'] for e in errs),
                link_max_error_m=max(max(e.values()) for e in errs), screenshot_pose=shot), \
        ['figures/1-05_leg_poses.png', 'figures/1-05_rviz_leg.png']


# ---- rig (1.06, 1.10) -----------------------------------------------------------------------------------------------------------
def make_rig(limit_upper=0.3):
    """Copy of the upstream Go2 URDF (no sensors) with three edits: lf_hip_joint upper limit, a world link fixed to base_link, and an
    effort state interface on every joint. Written to urdf/go2_rig.urdf."""
    text = expand(pathlib.Path(get_package_share_directory('go2_description')) / 'xacro/robot.xacro')
    root = ET.fromstring(text)
    lim = root.find("joint[@name='lf_hip_joint']/limit")
    assert lim is not None
    lim.set('upper', str(limit_upper))
    root.insert(0, ET.Element('link', name='world'))
    j = ET.Element('joint', name='world_fixed', type='fixed')
    ET.SubElement(j, 'parent', link='world'), ET.SubElement(j, 'child', link='base_link'), ET.SubElement(j, 'origin', xyz='0 0 0')
    root.insert(1, j)
    for jt in root.iter('ros2_control'):
        for je in jt.findall('joint'):
            ET.SubElement(je, 'state_interface', name='effort')
    ET.indent(root)
    out = UDIR / 'go2_rig.urdf'
    UDIR.mkdir(exist_ok=True)
    # No ': ' anywhere in the file: gazebo_ros2_control hands the URDF to rcl as a YAML parameter and a colon-space aborts it.
    out.write_text('<?xml version="1.0"?>\n<!-- Generated by run.py make_rig() from upstream robot.xacro, with three edits. lf_hip_joint '
                   f'upper limit 1.0472 to {limit_upper}, a world link fixed to base_link, an effort state interface per joint. -->\n'
                   + ET.tostring(root, encoding='unicode') + '\n')
    return out


@contextlib.contextmanager
def rig_session():
    path = make_rig()
    start(['ros2', 'launch', str(HERE / 'launch/rig.launch.py'), f'urdf:={path}'], 'rig')
    pr = Probe(sim_time=True)
    try:
        act = pr.active_controllers()
        if len(act) < 2:
            raise RuntimeError(f'rig controllers not active: {act}; see {LOG}/rig.log')
        yield pr, path
    finally:
        pr.close()
        stop_all(gazebo=True)


def js_array(js, names, field):
    idx = [js[0][2].index(n) for n in names]
    return np.array([[r[field][i] for i in idx] for r in js])


@task('1.06')
def t106():
    with rig_session() as (pr, path):
        text = path.read_text()
        ok, _ = check_urdf(text)
        lim = Urdf(text).joints['lf_hip_joint']['limit']['upper']
        pr.send_traj(RIG_HOLD)
        pr.sleep_sim(3.0)
        pr.rec = True
        t_cmd = pr.now()
        pr.send_traj(dict(RIG_HOLD, lf_hip_joint=0.6, rf_hip_joint=-0.6))
        pr.sleep_sim(4.0)
        pr.rec = False
    js = pr.js
    t = np.array([r[0] for r in js]) - t_cmd
    q = js_array(js, ['lf_hip_joint', 'rf_hip_joint'], 3)
    last = t > t[-1] - 1.0
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    ax.axhline(0.6, color=fs.C['plan'], lw=1, ls='--', label='command 0.6 rad (outward)')
    ax.axhline(lim, color=fs.C['bad'], lw=1, ls=':', label=f'edited lf limit {lim} rad')
    ax.plot(t, q[:, 0], color=fs.C['est'], label='lf_hip_joint (limit edited)')
    ax.plot(t, -q[:, 1], color=fs.C['est2'], label='-rf_hip_joint (limit unchanged)')
    ax.set_xlabel('sim time after the command [s]'); ax.set_ylabel('outward hip angle [rad]'); ax.legend(loc='center right', fontsize=8)
    ax.set_title('1.06 the edited limit stops the joint in Gazebo')
    fs.save(fig, FIG / '1-06_limit')
    return dict(copy=str(path.relative_to(HERE)), copy_parsed=ok, copy_limit_upper_rad=lim,
                limited_joint_max_rad=float(q[:, 0].max()), limited_joint_final_rad=float(q[last, 0].mean()),
                unlimited_joint_reached_rad=float(-q[last, 1].mean()), n_samples=len(js)), ['figures/1-06_limit.png']


# ---- 1.07 -----------------------------------------------------------------------------------------------------------------------
@task('1.07')
def t107():
    src = UDIR / 'go2_extra.xacro'
    u = Urdf(expand(src))
    rsp(expand(src))
    pr = Probe()
    try:
        T = pr.lookup('base_link', 'rear_cam_link', timeout=10.0)
    finally:
        pr.close()
        stop_all()
    want = u.fk({}, 'rear_cam_link', base='base_link')
    return dict(copy=str(src.relative_to(HERE)), copy_n_links=len(u.links),
                parent_is_base_link=u.joints['rear_cam_joint']['parent'] == 'base_link',
                translation_tf=T[:3, 3].round(6).tolist(), translation_error_m=float(np.linalg.norm(T[:3, 3] - (-0.25, 0, 0.06))),
                rotation_error_rad=rotation_angle(T[:3, :3].T @ want[:3, :3])), []


# ---- Gazebo with CHAMP (1.08, 1.09, 1.10) --------------------------------------------------------------------------------------
@contextlib.contextmanager
def sim_session(world, log='sim'):
    p = start(['ros2', 'launch', 'go2_nav_sim', 'sim.launch.py', f'world:={world}', 'odom:=gt', 'headless:=True'], log)
    pr = Probe(sim_time=True)
    try:
        act = pr.active_controllers()
        if len(act) < 2:
            raise RuntimeError(f'controllers not active: {act}; see {LOG}/{log}.log')
        wait_for(lambda: len(pr.odom) > 10, 60, '/odom_gt')
        yield pr, act, (LOG / f'{log}.log')
    finally:
        pr.close()
        stop_all(gazebo=True)
        p.wait(timeout=5)


def tilt(o):
    x, y, z, w = o[:, 4], o[:, 5], o[:, 6], o[:, 7]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return np.maximum(np.abs(roll), np.abs(pitch))


def save_image(m, out):
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
    plt.imsave(out, img)


@task('1.08')
def t108():
    with sim_session(HERE / 'worlds/empty.world') as (pr, act, log):
        pr.sleep_sim(8.0)                              # drop onto the feet, CHAMP stands up
        n0 = len(pr.odom)
        pr.sleep_sim(10.0)
        o = np.array(pr.odom[n0:])
        if pr.image is not None:
            save_image(pr.image, FIG / '1-08_spawned.png')
        spawned = 'Successfully spawned entity' in log.read_text()
    z = o[:, 3]
    return dict(spawned=spawned, controllers=act, controllers_active=len(act), base_z_mean_m=float(z.mean()),
                base_height_error_m=float(abs(z.mean() - 0.245)), base_z_std_m=float(z.std()), max_tilt_rad=float(tilt(o).max()),
                hold_sim_s=float(o[-1, 0] - o[0, 0]), drift_xy_m=float(np.hypot(*(o[-1, 1:3] - o[0, 1:3])))), \
        ['figures/1-08_spawned.png']


def gz_model(name, flag):
    return subprocess.run(['gz', 'model', '-m', name, flag], capture_output=True, text=True, timeout=30).stdout


def sdf_geometry(world, model):
    m = ET.parse(world).getroot().find(f".//model[@name='{model}']")
    assert m is not None
    geom = m.find('link/collision/geometry')
    assert geom is not None
    g = geom[0]
    return g.tag, {c.tag: [float(v) for v in cast(str, c.text).split()] for c in g}


def gz_geometry(info, kind):
    """Collision geometry from `gz model -i` text: the box size or the cylinder radius and length."""
    col = info[info.index('collision {'):]
    if kind == 'box':
        m = re.search(r'box \{\s*size \{\s*x: ([\d.e-]+)\s*y: ([\d.e-]+)\s*z: ([\d.e-]+)', col)
        assert m is not None
        return {'size': [float(v) for v in m.groups()]}
    m = re.search(r'cylinder \{\s*radius: ([\d.e-]+)\s*length: ([\d.e-]+)', col)
    assert m is not None
    return {'radius': [float(m.group(1))], 'length': [float(m.group(2))]}


def gz_pose(name):
    m = re.search(r'([-\d.e]+) ([-\d.e]+) ([-\d.e]+)', gz_model(name, '-p'))
    assert m is not None
    return np.array([float(v) for v in m.groups()])


@task('1.09')
def t109():
    world = HERE / 'worlds/obstacles.world'
    face_x = -2.8 - 0.2                                # box centre x minus half its depth
    pred_gap = 0.1934 + 0.046                          # hip joint x + hip collision cylinder radius
    with render_lock(), sim_session(world) as (pr, act, log):
        pr.sleep_sim(5.0)
        geo = {}
        for name in ('box', 'cylinder'):
            kind, want = sdf_geometry(world, name)
            info = gz_model(name, '-i')
            (LOG / f'gz_{name}.txt').write_text(info)
            got = gz_geometry(info, kind)
            geo[name] = dict(kind=kind, world_file=want, gazebo=got,
                             match=all(np.allclose(got[k], want[k], atol=1e-6) for k in want))
        box0 = gz_pose('box')
        vid = MEDIA / '1-09_walk_into_box.mp4'
        MEDIA.mkdir(exist_ok=True)
        vid.unlink(missing_ok=True)
        rec = start(['/usr/bin/python3', str(REPO / 'projects/common/record_cam.py'), str(vid), '--topic', '/chase/cam/image_raw',
                     '--fps', '10', '--width', '640'], 'record')
        wait_for(vid.exists, 90, 'recorder output')    # ffmpeg creates the file on the first frame
        n0 = len(pr.odom)
        pr.sleep_sim(1.5)
        n1 = len(pr.odom)
        t0, cmd = pr.now(), Twist()
        cmd.linear.x = 0.3
        while pr.now() - t0 < 12.0:
            pr.pub_cmd.publish(cmd)
            time.sleep(0.05)
        t_end = pr.now()
        pr.pub_cmd.publish(Twist())
        pr.sleep_sim(1.5)
        os.killpg(rec.pid, signal.SIGINT)
        rec.wait(timeout=20)
        box1 = gz_pose('box')
        o = np.array(pr.odom)
    stand = o[n0:n1]
    walk = o[(o[:, 0] >= t0) & (o[:, 0] <= t_end)]
    xw = walk[:, 1] + SPAWN[0]                          # /odom_gt is offset by the spawn point
    free = (xw > SPAWN[0] + 0.3) & (xw < SPAWN[0] + 0.9)
    v = np.polyfit(walk[free, 0], xw[free], 1)[0] if free.sum() > 5 else None
    pressed = walk[:, 0] > t_end - 2.0
    gap = float(np.median(face_x - xw[pressed]))
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    ax.plot(walk[:, 0] - t0, face_x - xw, color=fs.C['est'], label='gap base_link to box face (ground truth)')
    ax.axhline(pred_gap, color=fs.C['plan'], ls='--', lw=1, label=f'predicted contact gap {pred_gap:.4f} m')
    ax.set_xlabel('sim time after the walk command [s]'); ax.set_ylabel('gap [m]'); ax.legend(fontsize=8)
    ax.set_title('1.09 walking at 0.3 m/s into a static box')
    fs.save(fig, FIG / '1-09_gap')
    return dict(geometry=geo, collision_geometry_match=all(g['match'] for g in geo.values()),
                speed_before_contact_m_s=None if v is None else float(v), stop_gap_m=gap, predicted_gap_m=pred_gap,
                stop_gap_error_m=abs(gap - pred_gap), box_moved_m=float(np.linalg.norm(box1 - box0)),
                base_height_error_m=float(abs(stand[:, 3].mean() - 0.245)), max_tilt_walk_rad=float(tilt(walk).max()),
                final_y_m=float(walk[-1, 2])), ['figures/1-09_gap.png', 'media/1-09_walk_into_box.mp4']


@task('1.10')
def t110():
    with sim_session(HERE / 'worlds/empty.world') as (pr, act, log):
        pr.sleep_sim(5.0)
        pr.rec = True
        pr.sleep_sim(5.0)
        pr.rec = False
        js = pr.js
    stamps, wall = np.array([r[0] for r in js]), np.array([r[1] for r in js])
    rate = (len(js) - 1) / (stamps[-1] - stamps[0])
    m: dict[str, Any] = dict(n_messages=len(js), names=js[0][2], n_names=len(js[0][2]), rate_sim_hz=float(rate),
             rate_sim_hz_error=float(abs(rate - 250.0)), rate_wall_hz=float((len(js) - 1) / (wall[-1] - wall[0])),
             position_len=len(js[0][3]), velocity_len=len(js[0][4]), effort_len=len(js[0][5]),
             position_finite_fraction=finite_fraction([r[3] for r in js]), velocity_finite_fraction=finite_fraction([r[4] for r in js]),
             effort_finite_fraction=finite_fraction([r[5] for r in js]), effort_sample=[v if math.isfinite(v) else str(v) for v in js[-1][5][:3]])
    # The fix measured on the rig: the same URDF with an effort state interface per joint.
    with rig_session() as (rp, path):
        rp.send_traj(RIG_HOLD)
        rp.sleep_sim(4.0)
        rp.rec = True
        rp.sleep_sim(2.0)
        rp.rec = False
        rjs = rp.js
        u = Urdf(path.read_text())
    eff, q = js_array(rjs, JOINTS, 5), js_array(rjs, JOINTS, 3)
    qm = dict(zip(JOINTS, q.mean(0)))
    pred = np.array([u.holding_effort(qm, n) for n in JOINTS])
    m['rig'] = dict(effort_finite_fraction=finite_fraction([r[5] for r in rjs]), joints=JOINTS,
                    effort_mean_nm=eff.mean(0).round(4).tolist(), effort_gravity_predicted_nm=pred.round(4).tolist(),
                    effort_vs_gravity_max_error_nm=float(np.abs(eff.mean(0) - pred).max()))
    fig, ax = plt.subplots(figsize=(7.0, 3.0))
    xs = np.arange(12)
    ax.bar(xs - 0.2, pred, 0.4, color=fs.C['gt'], label='gravity torque from URDF masses')
    ax.bar(xs + 0.2, eff.mean(0), 0.4, color=fs.C['est'], label='/joint_states effort (rig)')
    ax.set_xticks(xs, [n.replace('_joint', '').replace('upper_leg', 'thigh').replace('lower_leg', 'knee') for n in JOINTS],
                  rotation=60, fontsize=7.5)
    ax.set_ylabel('holding effort [N m]'); ax.legend(fontsize=8)
    ax.set_title('1.10 effort reported once the effort state interface exists')
    fs.save(fig, FIG / '1-10_effort')
    return m, ['figures/1-10_effort.png']


def main():
    ids = list(TASKS) if sys.argv[1:] == ['all'] else sys.argv[1:]
    if not ids or any(i not in TASKS for i in ids):
        sys.exit(__doc__)
    for tid in ids:
        try:
            metrics, outputs = TASKS[tid]()
        except Exception as e:                         # a crash is a bug in the test or the stack, not a measured failure
            print(f'{tid} error: {e!r}', file=sys.stderr)
            stop_all(gazebo=True)
            continue
        write_result(tid, metrics, outputs)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
