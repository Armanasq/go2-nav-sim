"""Drawn figures for chapters 6, 7 and 8 (docs/img/c06-*.png). Every number on a figure comes from a file in this repository
or from the chapter text; the source is named next to the code that draws it.
Run from the repository root:
    xacro src/go2_nav_sim/urdf/go2_nav.xacro > /tmp/go2.urdf      # needs `source env.sh`; only the URDF tree figure reads it
    .venv-modules/bin/python docs/img/c06-figures.py [/tmp/go2.urdf] [figure names...]"""
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules'))
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

OUT = ROOT / 'docs/img'
C = fs.C
URDF_PATH = '/tmp/go2.urdf'


def save(fig, name):
    p = OUT / f'{name}.png'
    fig.savefig(p, dpi=130, bbox_inches='tight', pad_inches=0.04)
    plt.close(fig)
    print(p, p.stat().st_size // 1024, 'KB')


def node(ax, xy, w, h, text, color=C['ink'], fill=None, fontsize=8.5, mono=False, style='round'):
    """Box centred at xy."""
    x, y = xy[0] - w / 2, xy[1] - h / 2
    bs = 'round,pad=0,rounding_size=0.12' if style == 'round' else 'square,pad=0'
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=bs, fc=fill or C['fill'], ec=color, lw=1.0))
    ax.text(xy[0], xy[1], text, ha='center', va='center', fontsize=fontsize, color=C['ink'],
            family='monospace' if mono else None)


def link(ax, a, b, text=None, color=C['ink'], lw=1.1, style='-|>', ls='-', off=(0, 4), fontsize=7.5, rad=0.0, ha='center',
         mono=True):
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle=style, color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0,
                                                    mutation_scale=10, connectionstyle=f'arc3,rad={rad}'))
    if text:
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        ax.annotate(text, mid, xytext=off, textcoords='offset points', ha=ha, va='bottom', fontsize=fontsize, color=color,
                    family='monospace' if mono else None)


# ---------------------------------------------------------------------------------------------------------------- chapter 6

def description_flow():
    """Which file is read by which process (chapter 6 tables and sim.launch.py)."""
    fig, ax = fs.diagram(10, 5.4, (0, 20), (0, 10.8))
    # files
    node(ax, (3.2, 9.6), 5.6, 0.8, 'go2_nav.xacro  (this repository)', C['est'], '#e8f2f8', mono=True)
    node(ax, (1.7, 8.1), 3.0, 0.8, 'robot.xacro', C['gt'], mono=True)
    node(ax, (4.9, 8.1), 3.0, 0.8, 'sensors.xacro', C['est'], '#e8f2f8', mono=True)
    for i, f in enumerate(('const.xacro', 'materials.xacro', 'leg.xacro  (macro, × 4)', 'gazebo.xacro')):
        node(ax, (1.7, 6.9 - 0.75 * i), 2.6, 0.6, f, C['gt'], fontsize=7.5, mono=True)
        link(ax, (0.25, 7.7), (0.4, 6.9 - 0.75 * i), color=C['gt'], style='-', lw=0.7)
    ax.plot([0.25, 0.25], [7.7, 4.65], color=C['gt'], lw=0.7)
    link(ax, (3.2, 9.2), (1.7, 8.5), color=C['ink'], lw=0.9)
    link(ax, (3.2, 9.2), (4.9, 8.5), color=C['ink'], lw=0.9)
    ax.text(3.2, 10.35, 'robot description', ha='center', fontsize=9, fontweight='semibold')
    node(ax, (3.2, 3.1), 5.6, 0.8, 'ros_control.yaml', C['gt'], mono=True)
    ax.text(3.2, 3.85, 'controller parameters', ha='center', fontsize=9, fontweight='semibold')
    for i, f in enumerate(('joints.yaml', 'links.yaml', 'gait.yaml')):
        node(ax, (1.25 + 1.95 * i, 1.2), 1.8, 0.7, f, C['plan'], '#fbf1df', fontsize=8, mono=True)
    ax.text(3.2, 1.95, 'CHAMP maps (go2_config)', ha='center', fontsize=9, fontweight='semibold')
    # consumers
    cons = [((14.6, 9.3), 'robot_state_publisher\n/robot_description, TF of the links'),
            ((14.6, 7.3), 'spawn_entity.py  →  gzserver\ninserts the model at marker A'),
            ((14.6, 5.0), 'gazebo_ros2_control plugin\ncontroller manager, 250 Hz'),
            ((14.6, 2.5), 'quadruped_controller_node (CHAMP)\nleg model from URDF link offsets')]
    for xy, t in cons:
        node(ax, xy, 6.6, 1.25, t, C['ink'], fontsize=8.5)
    node(ax, (9.0, 8.2), 2.2, 0.9, 'xacro', C['muted'], 'white', mono=True)
    link(ax, (6.0, 9.6), (7.9, 8.4), color=C['est'])
    ax.text(9.0, 7.35, 'run by each node\nthat needs the URDF', ha='center', va='top', fontsize=7.5, color=C['muted'])
    link(ax, (10.1, 8.4), (11.3, 9.3), color=C['ink'])
    link(ax, (10.1, 8.1), (11.3, 7.3), color=C['ink'])
    link(ax, (10.1, 7.9), (11.3, 2.7), color=C['ink'], rad=0.05)
    link(ax, (14.6, 8.67), (14.6, 7.92), '/robot_description', C['est'], off=(4, -6), ha='left')
    link(ax, (14.6, 6.67), (14.6, 5.62), 'plugin inside the model;\nURDF from robot_state_publisher', C['muted'], off=(4, -10),
         ha='left', mono=False)
    link(ax, (6.0, 3.1), (11.3, 4.7), 'path named in gazebo.xacro', C['gt'], off=(0, 2), mono=False)
    link(ax, (6.0, 1.2), (11.3, 2.2), 'joints_map_path, links_map_path,\ngait_config_path', C['plan'], off=(0, 0), mono=True)
    save(fig, 'c06-description-flow')


def urdf_tree(path=URDF_PATH):
    """Link tree of the expanded URDF: revolute joints solid, fixed joints dashed."""
    root = ET.parse(path).getroot()
    kids, joint = {}, {}
    for j in root.findall('joint'):
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        kids.setdefault(p, []).append(c)
        joint[c] = (j.get('name'), j.get('type'), (j.find('origin').get('xyz') if j.find('origin') is not None else '0 0 0'))
    order = {'trunk': 0, 'lidar_link': 1, 'camera_link': 2, 'chase_cam': 3}
    kids['base_link'].sort(key=lambda k: order.get(k, 9))
    legorder = ['imu_link', 'lf_hip_link', 'rf_hip_link', 'lh_hip_link', 'rh_hip_link']
    kids['trunk'].sort(key=lambda k: legorder.index(k))
    rows, pos = [0], {}

    def place(n, depth):
        ch = kids.get(n, [])
        if not ch:
            pos[n] = (depth, rows[0]); rows[0] += 1
            return pos[n][1]
        ys = [place(c, depth + 1) for c in ch]
        pos[n] = (depth, (ys[0] + ys[-1]) / 2)
        return pos[n][1]
    place('base_link', 0)
    n = rows[0]
    dx = 1.55
    fig, ax = fs.diagram(11.5, 0.36 * n + 0.7, (-0.5, 5 * dx + 0.6), (n, -1.3), equal=False)
    for c, (name, typ, xyz) in joint.items():
        p = [k for k, v in kids.items() if c in v][0]
        (d0, y0), (d1, y1) = pos[p], pos[c]
        x0, x1 = d0 * dx, d1 * dx
        xm = (x0 + x1) / 2
        col = C['est'] if typ == 'revolute' else C['muted']
        ax.plot([x0, xm, xm, x1], [y0, y0, y1, y1], color=col, lw=1.5 if typ == 'revolute' else 0.9,
                ls='-' if typ == 'revolute' else (0, (3, 2)), zorder=1)
        if c.startswith('lf') or c in ('trunk', 'lidar_link', 'camera_link', 'camera_optical', 'chase_cam', 'imu_link'):
            left = p == 'base_link'
            ax.text(xm + (-0.05 if left else 0.04), y1 - 0.1, xyz.replace(' ', ', '), fontsize=6.3, color=col,
                    ha='right' if left else 'left', va='bottom',
                    family='monospace', zorder=3)
    for k, (d, y) in pos.items():
        sensor = k in ('lidar_link', 'lidar_visual', 'camera_link', 'camera_optical', 'chase_cam')
        col = C['est'] if sensor else C['ink']
        ax.text(d * dx, y, k, ha='center', va='center', fontsize=7.4, family='monospace', color=col, zorder=4,
                bbox=dict(boxstyle='round,pad=0.25', fc='#e8f2f8' if sensor else C['fill'], ec=col, lw=0.7))
    ax.plot([], [], color=C['est'], lw=1.4, label='revolute joint (12)')
    ax.plot([], [], color=C['muted'], lw=0.9, ls=(0, (3, 2)), label='fixed joint')
    ax.legend(loc='upper right', bbox_to_anchor=(1.0, 1.02), fontsize=8)
    ax.text(-0.5, -1.0, 'xacro src/go2_nav_sim/urdf/go2_nav.xacro, joint origins in metres (lf leg labelled; '
            'the other legs mirror it)', fontsize=7.5, color=C['muted'])
    save(fig, 'c06-urdf-tree')


def ros2_control():
    """Data path from CHAMP's joint targets to torques and back (ros_control.yaml, chapter 6)."""
    fig, ax = fs.diagram(10, 4.2, (0, 20), (0, 8.4))
    node(ax, (2.4, 6.4), 4.2, 1.2, 'CHAMP\nquadruped_controller_node\n200 Hz', C['plan'], '#fbf1df')
    ax.add_patch(FancyBboxPatch((6.0, 0.4), 13.6, 7.6, boxstyle='round,pad=0,rounding_size=0.2', fc='none', ec=C['muted'],
                                lw=0.8, ls='--'))
    ax.text(6.25, 7.65, 'controller manager  /controller_manager  (update_rate: 250)', fontsize=8.5, color=C['muted'])
    node(ax, (9.6, 6.0), 6.2, 1.5, 'joint_group_effort_controller\nJointTrajectoryController\nPID per joint: p 100, i 0.2, d 1.0, i_clamp 2.5',
         C['est'], '#e8f2f8', fontsize=8)
    node(ax, (9.6, 2.0), 6.2, 1.2, 'joint_states_controller\nJointStateBroadcaster',
         C['est'], '#e8f2f8', fontsize=8)
    for i, leg in enumerate(('rf', 'lf', 'rh', 'lh')):
        node(ax, (16.6, 6.5 - 1.45 * i), 4.6, 1.15, f'<ros2_control name="{leg}">\nGazeboSystem, 3 joints', C['gt'], fontsize=7.5)
    link(ax, (4.5, 6.4), (6.5, 6.4), None, C['plan'])
    ax.text(2.4, 4.95, '/joint_group_effort_controller/\njoint_trajectory\n(12 position targets)', ha='center', va='top',
            fontsize=7.5, family='monospace', color=C['plan'])
    ax.plot([13.9, 13.9], [1.6, 7.0], color=C['gt'], lw=1.2)
    for i in range(4):
        ax.plot([13.9, 14.3], [6.5 - 1.45 * i] * 2, color=C['gt'], lw=1.2)
    link(ax, (12.7, 6.4), (13.9, 6.4), None, C['bad'])
    ax.text(13.3, 6.55, 'effort', fontsize=7.5, color=C['bad'], ha='center', va='bottom')
    link(ax, (13.9, 5.2), (12.7, 5.6), None, C['good'])
    link(ax, (13.9, 2.0), (12.7, 2.0), None, C['good'])
    ax.text(13.3, 4.0, 'position,\nvelocity\n(state\ninterfaces)', fontsize=7.3, color=C['good'], ha='center', va='center')
    link(ax, (6.5, 2.0), (4.5, 2.0), None, C['est'])
    node(ax, (2.4, 2.0), 4.2, 1.0, '/joint_states  249.6 Hz\nrobot_state_publisher', C['est'], 'white', fontsize=8)
    ax.text(9.6, 0.65, r'$\tau = k_p (q^\ast - q) + k_i \int (q^\ast - q)\,dt + k_d (\dot q^\ast - \dot q)$', ha='center', fontsize=9)
    save(fig, 'c06-ros2-control')


def startup_sequence():
    """The five start-up steps of chapter 6 as a sequence diagram."""
    lanes = ['ros2 launch', 'robot_state_publisher', 'gzserver', 'spawn_entity.py', 'gazebo_ros2_control', 'spawner ×2']
    fig, ax = fs.diagram(10, 4.6, (-0.5, len(lanes) * 2 - 0.5), (-7.4, 1.0), equal=False)
    X = {n: 2 * i + 0.5 for i, n in enumerate(lanes)}
    for n, x in X.items():
        ax.text(x, 0.55, n, ha='center', fontsize=8.3, fontweight='semibold', family='monospace')
        ax.plot([x, x], [0.2, -7.2], color=C['grid'], lw=1.2)
    steps = [('ros2 launch', 'robot_state_publisher', '1  start; publishes /robot_description', -0.8, C['ink']),
             ('ros2 launch', 'gzserver', '2  arena world + gazebo_ros init, factory', -1.8, C['ink']),
             ('spawn_entity.py', 'gzserver', '3  insert go2 at (-4.5, -3.5, 0.275)', -2.9, C['ink']),
             ('gzserver', 'gazebo_ros2_control', '4  plugin loads inside the model', -3.9, C['ink']),
             ('robot_state_publisher', 'gazebo_ros2_control', 'URDF', -4.7, C['est']),
             ('spawner ×2', 'gazebo_ros2_control', '5  load + activate both controllers', -5.8, C['ink'])]
    for a, b, t, y, c in steps:
        link(ax, (X[a], y), (X[b], y), None, c)
        ax.text((X[a] + X[b]) / 2, y + 0.12, t, ha='center', va='bottom', fontsize=7.8, color=c)
    ax.text(X['gazebo_ros2_control'] - 0.12, -4.95, "'hold_joints' error × 3,\ncontrollers start after it", ha='right', va='top',
            fontsize=7.2, color=C['bad'], bbox=dict(boxstyle='square,pad=0.15', fc='white', ec='none', alpha=0.9))
    ax.text(X['spawn_entity.py'], -3.25, 'drops 0.275 m onto its feet', ha='center', va='top', fontsize=7.2, color=C['muted'])
    ax.text(X['spawner ×2'] - 0.9, -6.25, 'replaces upstream\'s "ros2 control load_controller"\n(ros2controlcli is not installed)',
            ha='center', va='top', fontsize=7.2, color=C['plan'])
    save(fig, 'c06-startup-sequence')


# ---------------------------------------------------------------------------------------------------------------- chapter 7

def ros_graph_walk():
    """Nodes and topics running after `sim.launch.py odom:=gt`; rates measured in chapter 7."""
    fig, ax = fs.diagram(10.5, 5.2, (0, 21), (0, 10.4))
    N = dict(teleop=((2.2, 8.6), 'teleop_twist_keyboard\nor ros2 topic pub'),
             champ=((8.6, 8.6), 'quadruped_controller_node\n(CHAMP gait, 200 Hz loop)'),
             cm=((16.6, 8.6), 'controller_manager\n(inside gzserver)'),
             gz=((16.6, 4.6), 'gzserver\nGo2 + sensors + p3d'),
             rsp=((8.6, 4.6), 'robot_state_publisher'),
             gt=((8.6, 1.2), 'gt_odom.py  (odom:=gt)'),
             est=((2.2, 4.6), 'state_estimation_node\n+ base_to_footprint_ekf\nin: /joint_states, /imu/data'))
    for k, (xy, t) in N.items():
        node(ax, xy, 5.2 if k != 'teleop' else 3.9, 1.3 if k != 'est' else 1.6, t, C['ink'], fontsize=8.2)

    def topic(xy, t, col=C['est']):
        ax.text(*xy, t, ha='center', va='center', fontsize=7.6, family='monospace', color=col,
                bbox=dict(boxstyle='square,pad=0.25', fc='white', ec=col, lw=0.8))
    link(ax, (4.15, 8.6), (5.98, 8.6), None, C['plan'])
    topic((5.1, 9.65), '/cmd_vel', C['plan'])
    link(ax, (11.2, 8.6), (14.0, 8.6), None, C['plan'])
    topic((12.6, 9.65), '.../joint_trajectory\n200.0 Hz', C['plan'])
    link(ax, (16.6, 7.95), (16.6, 5.25), None, C['bad'])
    ax.text(16.8, 6.6, 'joint torques', fontsize=7.6, color=C['bad'])
    link(ax, (14.0, 8.2), (11.2, 5.0), None, C['est'])
    topic((12.2, 6.85), '/joint_states\n249.6 Hz')
    link(ax, (14.0, 4.3), (11.2, 1.5), None, C['gt'])
    topic((12.9, 2.2), '/odom_gt  50 Hz', C['gt'])
    link(ax, (14.0, 4.0), (4.8, 4.0), None, C['est'], rad=-0.12)
    topic((9.4, 3.2), '/imu/data  100 Hz')
    topic((2.9, 1.2), '/tf  odom → base_footprint', C['gt'])
    link(ax, (6.0, 1.2), (5.25, 1.2), None, C['gt'])
    topic((2.2, 2.6), '/tf  base_footprint → base_link\n(constant 0.426 m)', C['muted'])
    link(ax, (2.2, 3.8), (2.2, 3.15), None, C['muted'])
    topic((8.6, 6.3), '/tf  leg links', C['est'])
    link(ax, (8.6, 5.25), (8.6, 5.95), None, C['est'])
    ax.text(0.2, 0.1, 'Rates from ros2 topic hz in chapter 7 (joint_states, joint_trajectory) and the sensor table in chapter 8.',
            fontsize=7.3, color=C['muted'])
    save(fig, 'c06-ros-graph-walk')


def champ_loop():
    """One 5 ms tick of QuadrupedController::controlLoop_ (quadruped_controller.cpp lines 101-113)."""
    fig, ax = fs.diagram(10.5, 3.9, (0, 21), (0, 7.8))
    ax.add_patch(FancyBboxPatch((3.3, 2.8), 12.6, 4.6, boxstyle='round,pad=0,rounding_size=0.2', fc='none', ec=C['plan'], lw=0.9,
                                ls='--'))
    ax.text(3.5, 7.05, 'quadruped_controller_node: controlLoop_() every 1/200 s', fontsize=8.5, color=C['plan'])
    node(ax, (1.5, 5.0), 2.6, 1.2, '/cmd_vel\nvx, vy, wz', C['plan'], '#fbf1df', fontsize=8)
    node(ax, (5.6, 5.0), 3.6, 1.6, 'clip to\n±0.3, ±0.25 m/s,\n±0.5 rad/s', C['ink'], fontsize=8)
    node(ax, (9.6, 5.6), 3.6, 1.3, 'phase generator\nstance 0.25 s, swing 0.25 s', C['ink'], fontsize=7.8)
    node(ax, (9.6, 3.8), 3.6, 1.3, 'Raibert step length\n0.25 × v per leg', C['ink'], fontsize=7.8)
    node(ax, (13.7, 4.7), 3.4, 1.6, 'foot trajectory\nBézier swing,\ncosine stance', C['ink'], fontsize=8)
    node(ax, (13.7, 0.9), 3.4, 1.2, 'inverse kinematics\nknee_orientation ">>"', C['ink'], fontsize=7.8)
    node(ax, (18.6, 4.7), 3.6, 1.6, 'PID, 250 Hz\nGazebo physics', C['est'], '#e8f2f8', fontsize=8)
    link(ax, (2.8, 5.0), (3.8, 5.0), None, C['plan'])
    link(ax, (7.4, 5.3), (7.8, 5.6), None)
    link(ax, (7.4, 4.7), (7.8, 3.8), None)
    link(ax, (11.4, 5.6), (12.0, 5.0), None)
    link(ax, (11.4, 3.8), (12.0, 4.4), None)
    link(ax, (13.7, 3.9), (13.7, 1.5), 'foot positions\nin base_link', off=(4, -8), ha='left', mono=False)
    link(ax, (15.4, 0.9), (18.6, 3.9), '12 joint targets', C['plan'], off=(14, -8), mono=False)
    ax.text(6.2, 2.15, 'no path back: nothing measures the body velocity\n(open loop; slip and tracking error go uncorrected)',
            fontsize=7.8, color=C['bad'], ha='center', va='top')
    save(fig, 'c06-champ-loop')


def champ_phase(t, stance=0.25, swing=0.25):
    """phase_generator.h, PhaseGenerator::run for a constant non-zero command started at t = 0 (the has_swung_ start-up rule
    included). Returns stance and swing signals, shape (4, len(t)), legs in CHAMP order lf, rf, lh, rh."""
    T = stance + swing
    st, sw = np.zeros((4, len(t))), np.zeros((4, len(t)))
    has_swung = False
    for k, tk in enumerate(t):
        e = tk % T
        clocks = [e, e - 0.5 * T, e - 0.5 * T, e]
        for i, c in enumerate(clocks):
            st[i, k] = c / stance if 0 < c < stance else 0
            if -swing < c < 0:
                sw[i, k] = (c + swing) / swing
            elif stance < c < T:
                sw[i, k] = (c - stance) / swing
        if not has_swung and st[0, k] < 0.5:
            st[0, k] = st[3, k] = 0; sw[1, k] = sw[2, k] = 0
        else:
            has_swung = True
    return st, sw


def gait_timing():
    t = np.arange(0, 1.5, 0.0005)
    st, sw = champ_phase(t)
    fig, ax = plt.subplots(figsize=(9, 2.9))
    names = ['lf', 'rf', 'lh', 'rh']
    col = [fs.LEGS['FL'], fs.LEGS['FR'], fs.LEGS['RL'], fs.LEGS['RR']]
    for i in range(4):
        y = 3 - i
        stance = st[i] > 0
        swing = sw[i] > 0
        for mask, alpha, hatch in ((stance, 0.85, None), (swing, 0.18, None)):
            edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
            for a, b in zip(edges[::2], edges[1::2]):
                ax.add_patch(plt.Rectangle((t[a], y - 0.35), t[b - 1] - t[a], 0.7, fc=col[i], alpha=alpha, ec='none'))
    for x in np.arange(0, 1.51, 0.25):
        ax.axvline(x, color=C['grid'], lw=0.6, zorder=0)
    ax.set_yticks(range(4)); ax.set_yticklabels(names[::-1]); ax.set_ylim(-0.6, 3.9); ax.set_xlim(0, 1.5)
    ax.set_xlabel('time since the first non-zero /cmd_vel [s]'); ax.grid(False)
    ax.annotate('', xy=(0.75, 3.62), xytext=(1.25, 3.62), arrowprops=dict(arrowstyle='<|-|>', lw=0.9, color=C['ink'], mutation_scale=8))
    ax.text(1.0, 3.68, 'stride 0.5 s', ha='center', va='bottom', fontsize=8)
    ax.text(0.625, 3, 'stance 0.25 s', ha='center', va='center', fontsize=7.5, color='white')
    ax.text(0.875, 3, 'swing 0.25 s', ha='center', va='center', fontsize=7.5, color=C['ink'])
    ax.add_patch(plt.Rectangle((0, -0.35), 0.125, 3.7, fc='none', ec='#b8bec7', hatch='///', lw=0))
    ax.text(0.0625, 1.5, 'all feet hold\n(has_swung_ rule)', ha='center', va='center', fontsize=7, rotation=90, color=C['ink'],
            bbox=dict(boxstyle='square,pad=0.2', fc='white', ec='none'))
    ax.set_title('CHAMP trot: diagonal pairs lf + rh and rf + lh alternate (solid = stance, pale = swing)')
    save(fig, 'c06-gait-timing')
    return st, sw


def bezier_swing(s, L, h=0.04):
    """trajectory_planner.h: 12-point Bézier swing, control points scaled by step length L / 0.4 and swing height h / 0.15."""
    rx = np.array([-0.15, -0.2805, -0.3, -0.3, -0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15])
    ry = np.array([-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5])
    cx = rx * L / 0.4; cx[0], cx[-1] = -L / 2, L / 2
    cy = -((ry * h / 0.15) + 0.5 * h / 0.15)
    n = 11
    B = np.array([[math.comb(n, i) * si ** i * (1 - si) ** (n - i) for i in range(12)] for si in np.atleast_1d(s)])
    return B @ cx, -(B @ cy)


def stance_path(s, L, depth=0.01):
    x = L / 2 * (1 - 2 * s)
    return x, -depth * np.cos(np.pi * x / L)


def foot_trajectory():
    fig, ax = plt.subplots(figsize=(10, 3.4))
    s = np.linspace(0, 1, 200)
    for v, c in ((0.1, C['gt']), (0.2, C['est']), (0.3, C['est2'])):
        L = 0.25 * v                                            # gait.yaml stance_duration 0.25 -> stroke 0.25 x v (chapter 7)
        xs, ys = stance_path(s, L); xw, yw = bezier_swing(s, L)
        ax.plot(xs * 100, ys * 100, color=c, lw=1.6)
        ax.plot(xw * 100, yw * 100, color=c, lw=1.6, ls='--', label=f'{v} m/s: stroke {L * 100:.1f} cm, peak lift {yw.max() * 100:.1f} cm')
    ax.axhline(4.0, color=C['plan'], lw=0.8, ls=':'); ax.text(-7.3, 4.12, 'swing_height 4 cm', fontsize=7.5, color=C['plan'])
    ax.axhline(-1.0, color=C['plan'], lw=0.8, ls=':'); ax.text(-7.3, -1.45, 'stance_depth 1 cm', fontsize=7.3, color=C['plan'])
    ax.axhline(3.2 - 1.0, color=C['bad'], lw=0.8, ls='-.')
    ax.text(-7.3, 2.3, 'measured lf foot travel 3.2 cm (0.2 m/s, chapter 7),\ndrawn from the -1 cm stance bottom',
            fontsize=7.2, color=C['bad'], bbox=dict(boxstyle='square,pad=0.15', fc='white', ec='none', alpha=0.9))
    ax.annotate('swing (dashed): foot moves forward', (0.0, 4.1), xytext=(1.2, 4.75), fontsize=7.5, arrowprops=dict(arrowstyle='-', lw=0.6))
    ax.annotate('stance (solid): foot pushed back', (0.0, -1.0), xytext=(1.6, -1.75), fontsize=7.5, arrowprops=dict(arrowstyle='-', lw=0.6))
    ax.set_xlim(-7.5, 7.5); ax.set_ylim(-2.2, 5.2); ax.set_aspect('equal')
    ax.set_xlabel('x relative to the nominal foot point [cm]'); ax.set_ylabel('z [cm]')
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=7.5)
    ax.set_title('Commanded foot path for one stride (trajectory_planner.h, gait.yaml values)')
    save(fig, 'c06-foot-trajectory')


def cmd_vs_actual():
    """The table 'Commanded and actual speed' in chapter 7."""
    rows = [('vx 0.1', 0.1, 0.063), ('vx 0.2', 0.2, 0.181), ('vx 0.3', 0.3, 0.300), ('vx 0.5', 0.5, 0.302), ('vx -0.2', -0.2, -0.260),
            ('vy 0.2', 0.2, 0.214), ('wz 0.5', 0.5, 0.484), ('wz -0.5', -0.5, -0.477), ('wz 0.25', 0.25, 0.234)]
    fig, ax = plt.subplots(figsize=(8, 3.2))
    y = np.arange(len(rows))[::-1]
    for yi, (n, c, a) in zip(y, rows):
        ax.plot([c, a], [yi, yi], color=C['grid'], lw=3, solid_capstyle='round', zorder=1)
        ax.plot(c, yi, 'o', mfc='white', mec=C['plan'], mew=1.5, ms=7, zorder=3)
        ax.plot(a, yi, 'o', color=C['est'], ms=6, zorder=4)
        r = a / c
        ax.text(0.62, yi, f'{r * 100:.0f} %', va='center', fontsize=8, color=C['bad'] if abs(r - 1) > 0.1 else C['muted'])
    for x in (0.3, -0.3):
        ax.plot([x, x], [y[4] - 0.4, y[0] + 0.4], color=C['bad'], lw=0.8, ls=':')
    ax.text(-0.29, y[4] - 0.3, 'max_linear_velocity_x ±0.3 m/s', fontsize=7.2, color=C['bad'], va='top')
    ax.set_yticks(y); ax.set_yticklabels([r[0] for r in rows], family='monospace', fontsize=8)
    ax.set_xlim(-0.6, 0.7); ax.set_xlabel('speed: m/s for vx, vy; rad/s for wz')
    ax.plot([], [], 'o', mfc='white', mec=C['plan'], mew=1.5, label='commanded'); ax.plot([], [], 'o', color=C['est'], label='actual (/odom_gt, last 7 s)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.45, -0.2), ncol=2, fontsize=7.5)
    ax.text(0.62, len(rows) - 0.4, 'actual /\ncommand', fontsize=7.3, color=C['muted'], va='bottom')
    ax.set_title('CHAMP follows /cmd_vel open loop: small and backward commands miss most')
    save(fig, 'c06-cmd-vs-actual')


# ---------------------------------------------------------------------------------------------------------------- chapter 8

def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx                                          # URDF convention: fixed-axis roll, then pitch, then yaw


def camera_optical():
    R = rpy_R(-math.pi / 2, 0, -math.pi / 2)                    # camera_optical_joint in sensors.xacro
    assert np.allclose(R, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-6)
    fig = plt.figure(figsize=(9.5, 3.9))
    for k, (title, M) in enumerate((('camera_link (REP-103): x forward, y left, z up', np.eye(3)),
                                    ('camera_optical: z forward, x right, y down', R))):
        ax = fig.add_subplot(1, 3, k + 1, projection='3d')
        ax.view_init(elev=18, azim=-140)
        for i, (n, c) in enumerate((('x', C['bad']), ('y', C['good']), ('z', C['est']))):
            v = M[:, i]
            ax.quiver(0, 0, 0, *v, color=c, lw=2, arrow_length_ratio=0.15)
            ax.text(*(1.18 * v), n, color=c, fontsize=11, fontweight='semibold', ha='center')
        ax.quiver(0, 0, 0, 1.5, 0, 0, color=C['muted'], lw=0.8, ls=':', arrow_length_ratio=0.05)
        ax.text(1.6, 0, -0.15, 'robot\nforward', fontsize=7, color=C['muted'])
        ax.set_xlim(-1, 1.4); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1); ax.set_box_aspect((1.2, 1, 1))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.set_xlabel('robot x', fontsize=7, labelpad=-12); ax.set_ylabel('robot y', fontsize=7, labelpad=-12)
        ax.set_zlabel('robot z', fontsize=7, labelpad=-12)
        ax.set_title(title, fontsize=8.5, loc='center')
    ax = fig.add_subplot(1, 3, 3)
    W, H = 640, 360
    ax.add_patch(plt.Rectangle((0, 0), W, H, fc=C['fill'], ec=C['ink'], lw=0.8))
    ax.set_xlim(-40, W + 60); ax.set_ylim(H + 60, -60); ax.set_aspect('equal'); ax.axis('off')
    cx, cy = W / 2, H / 2
    ax.annotate('', xy=(110, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', color=C['ink'], lw=1.2))
    ax.annotate('', xy=(0, 110), xytext=(0, 0), arrowprops=dict(arrowstyle='-|>', color=C['ink'], lw=1.2))
    ax.text(115, -10, 'u (column)', fontsize=7.5); ax.text(6, 125, 'v (row)', fontsize=7.5)
    ax.text(-8, -12, '(0, 0)', fontsize=7, ha='right')
    ax.annotate('', xy=(cx + 120, cy), xytext=(cx, cy), arrowprops=dict(arrowstyle='-|>', color=C['bad'], lw=2))
    ax.annotate('', xy=(cx, cy + 120), xytext=(cx, cy), arrowprops=dict(arrowstyle='-|>', color=C['good'], lw=2))
    ax.plot(cx, cy, 'o', mfc='white', mec=C['est'], ms=11, mew=2); ax.plot(cx, cy, '.', color=C['est'], ms=5)
    ax.text(cx + 125, cy, 'x', color=C['bad'], fontsize=11, fontweight='semibold', va='center')
    ax.text(cx, cy + 140, 'y', color=C['good'], fontsize=11, fontweight='semibold', ha='center')
    ax.text(cx + 14, cy - 18, 'z into the image', color=C['est'], fontsize=8)
    ax.text(W / 2, H + 50, '640 × 360 image, 1.2 rad horizontal FOV', ha='center', fontsize=7.5, color=C['muted'])
    ax.set_title('the image seen from behind the camera', fontsize=8.5, loc='center')
    save(fig, 'c06-camera-optical')


def sensor_frames():
    """Frames each sensor message is stamped in (chapter 8 table), hung off the TF tree."""
    fig, ax = fs.diagram(10.5, 4.4, (0, 21), (0, 8.8))
    F = {'odom': (2.0, 7.6), 'base_footprint': (2.0, 5.6), 'base_link': (2.0, 3.6),
         'lidar_link': (7.6, 7.2), 'camera_link': (7.6, 5.0), 'camera_optical': (12.0, 5.0), 'trunk': (7.6, 2.8),
         'chase_cam': (7.6, 0.9)}
    for k, xy in F.items():
        node(ax, xy, 3.4 if k != 'camera_optical' else 3.6, 0.8, k, C['est'], 'white', mono=True, fontsize=8)
    link(ax, (2.0, 7.2), (2.0, 6.0), None, C['gt']); ax.text(2.2, 6.6, 'gt_odom.py or lidar odometry', fontsize=7, color=C['gt'])
    link(ax, (2.0, 5.2), (2.0, 4.0), None, C['muted']); ax.text(2.2, 4.6, '(0, 0, 0.426), constant', fontsize=7, color=C['muted'])
    for k, t in (('lidar_link', '(0, 0, 0.10)'), ('camera_link', '(0.28, 0, 0.03)'), ('trunk', '(0, 0, 0)'), ('chase_cam', '(-0.9, 0, 0.6)')):
        link(ax, (3.7, 3.6), (F[k][0] - 1.7, F[k][1]), None, C['muted'], ls='--')
        ax.text(F[k][0] - 1.75, F[k][1] + 0.5, t, fontsize=6.8, color=C['muted'], family='monospace', ha='right')
    link(ax, (9.3, 5.0), (10.2, 5.0), None, C['muted'], ls='--')
    ax.text(9.75, 5.55, 'rpy (-π/2, 0, -π/2)', fontsize=6.8, color=C['muted'], family='monospace', ha='center')

    def msg(xy, t, frm, c=C['plan']):
        ax.text(*xy, t, ha='left', va='center', fontsize=7.6, family='monospace', color=c,
                bbox=dict(boxstyle='square,pad=0.3', fc='#fbf1df', ec=c, lw=0.7))
        ax.annotate('', xy=(F[frm][0] + 1.7 + (0.1 if frm != 'camera_optical' else 0.2), F[frm][1]), xytext=(xy[0], xy[1]),
                    arrowprops=dict(arrowstyle='-', color=c, lw=0.8, ls=':'))
    msg((14.6, 7.5), '/lidar/points  10 Hz', 'lidar_link')
    msg((14.6, 5.6), '/camera/image_raw  10 Hz', 'camera_optical')
    msg((14.6, 4.6), '/camera/depth/image_raw  10 Hz', 'camera_optical')
    msg((14.6, 3.6), '/imu/data  100 Hz', 'base_link')
    msg((14.6, 0.9), '/chase/cam/*  10 Hz (video only)', 'chase_cam')
    ax.text(0.3, 0.4, '/odom_gt  50 Hz: frame world,\nthe simulator frame (not drawn)', fontsize=7.6, color=C['gt'],
            va='center', family='monospace')
    ax.text(0.1, 8.55, 'TF frames (boxes) and the frame_id in each sensor message (orange); the leg links under trunk are omitted',
            fontsize=7.5, color=C['muted'])
    save(fig, 'c06-sensor-frames')


def lidar_rings():
    """Where the downward lidar beams meet the floor: 16 beams spaced 2 degrees over ±15 degrees (sensors.xacro), lidar 0.10 m
    above base_link, base_link 0.216 m above the floor standing (chapter 7)."""
    h = 0.216 + 0.10
    el = np.radians(np.arange(-15, 16, 2))
    down = el[el < 0]
    r = h / np.tan(-down)
    fig, axs = plt.subplots(1, 2, figsize=(9.5, 3.6), gridspec_kw=dict(width_ratios=[1.35, 1]))
    ax = axs[0]
    for e in el:
        L = 4.0
        x, z = L * np.cos(e), h + L * np.sin(e)
        if z < 0:
            x, z = h / np.tan(-e), 0
        ax.plot([0, x], [h, z], color=C['est'] if e < 0 else C['grid'], lw=0.9)
    ax.plot(r, np.zeros_like(r), 'o', color=C['bad'], ms=4)
    for ri, e in zip(r, down):
        if ri < 4:
            ax.text(ri, -0.05, f'{ri:.2f}', fontsize=6.8, ha='center', va='top', rotation=90, color=C['bad'])
    ax.axhline(0, color=C['ink'], lw=0.8)
    ax.plot(0, h, 's', color=C['ink'], ms=6)
    ax.text(0.05, h + 0.06, f'lidar_link, {h:.3f} m above the floor', fontsize=7.5)
    ax.set_xlim(-0.1, 4.0); ax.set_ylim(-0.32, 1.1); ax.set_xlabel('distance ahead [m]'); ax.set_ylabel('height [m]')
    ax.set_title('side view: 8 of 16 beams point down')
    ax.text(3.95, -0.27, f'-3° and -1° beams: {r[-2]:.1f} m and {r[-1]:.1f} m', ha='right', fontsize=7.3, color=C['bad'])
    ax = axs[1]
    th = np.linspace(0, 2 * np.pi, 400)
    for ri, e in zip(r, down):
        if ri <= 20:
            ax.plot(ri * np.cos(th), ri * np.sin(th), color=C['est'], lw=0.9)
    ax.plot(0.3 * np.cos(th), 0.3 * np.sin(th), color=C['bad'], lw=0.9, ls='--')
    ax.text(0.0, -0.55, 'min range 0.3 m', ha='center', fontsize=6.8, color=C['bad'])
    ax.add_patch(plt.Rectangle((-0.19, -0.05), 0.38, 0.1, fc=C['ink']))
    ax.set_xlim(-4, 4); ax.set_ylim(-4, 4); ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('top view: floor rings inside 4 m')
    print('ring radii [m]:', np.round(r, 2))
    save(fig, 'c06-lidar-rings')


def sensor_rates():
    """Message arrival times over 0.3 s at the nominal rates of the chapter 8 table (phases arbitrary)."""
    rows = [('/joint_states', 250, C['gt']), ('/imu/data', 100, C['est']), ('/odom_gt', 50, C['gt']), ('/lidar/points', 10, C['est']),
            ('/camera/image_raw', 10, C['plan']), ('/camera/depth/image_raw', 10, C['plan'])]
    fig, ax = plt.subplots(figsize=(8.5, 2.6))
    for i, (n, hz, c) in enumerate(rows):
        t = np.arange(0, 0.3 + 1e-9, 1 / hz)
        ax.vlines(t, len(rows) - i - 0.35, len(rows) - i + 0.35, color=c, lw=1.0 if hz > 60 else 2.0)
        ax.text(0.305, len(rows) - i, f'{hz} Hz', va='center', fontsize=8, color=c)
    ax.set_yticks(np.arange(len(rows), 0, -1)); ax.set_yticklabels([r[0] for r in rows], family='monospace', fontsize=8)
    ax.set_xlim(0, 0.33); ax.set_xlabel('time [s]'); ax.grid(axis='y', visible=False)
    ax.set_title('Ten IMU messages and 25 joint states arrive between two lidar sweeps')
    save(fig, 'c06-sensor-rates')


ALL = ['description_flow', 'urdf_tree', 'ros2_control', 'startup_sequence', 'ros_graph_walk', 'champ_loop', 'gait_timing',
       'foot_trajectory', 'cmd_vs_actual', 'camera_optical', 'sensor_frames', 'lidar_rings', 'sensor_rates']

if __name__ == '__main__':
    args = sys.argv[1:]
    if args and args[0].endswith('.urdf'):
        URDF_PATH = args.pop(0)
    for name in args or ALL:
        f = globals()[name]
        f(URDF_PATH) if name == 'urdf_tree' else f()
