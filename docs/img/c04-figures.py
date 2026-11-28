"""Diagrams and plots for chapters 4 (ROS 2 from scratch) and 5 (frames and TF2). Every number drawn here is quoted in the
chapter text or read from site/data/runs/w2/run.json; nothing is measured anew. Run from the repository root:
    .venv-modules/bin/python docs/img/c04-figures.py [name ...]
Writes docs/img/c04-<name>.png (150 dpi)."""
import io
import json
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules'))
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, Polygon  # noqa: E402
from PIL import Image  # noqa: E402

C = fs.C
OUT = ROOT / 'docs/img'
MONO = 'DejaVu Sans Mono'
LAUNCH = dict(sim='#5b636e', est=C['est'], nav=C['est2'])       # one colour per launch file (terminal 1, 2, 3)


def save(fig, name):
    buf = io.BytesIO(); fig.savefig(buf, format='png', dpi=150, bbox_inches='tight', pad_inches=0.05); plt.close(fig)
    im = Image.open(buf).convert('RGB').quantize(colors=96, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    path = OUT / f'c04-{name}.png'; im.save(path, optimize=True)
    print(path.name, path.stat().st_size // 1024, 'KB', im.size)


def node(ax, x, y, w, h, text, color, fs_=8.0, fill=None, mono=True, lw=1.0):
    """ROS node: rounded box tinted with its launch-file colour."""
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0,rounding_size=0.06', fc=fill or 'white', ec=color, lw=lw))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs_, color=C['ink'], family=MONO if mono else None)


def topic(ax, x, y, w, h, text, fs_=7.6):
    """ROS topic: square-cornered grey box."""
    ax.add_patch(Rectangle((x, y), w, h, fc=C['fill'], ec=C['muted'], lw=0.7))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs_, color=C['ink'], family=MONO)


def arr(ax, a, b, color=None, lw=0.9, style='-|>', ls='-', rad=0.0, text=None, toff=(0, 4), tfs=7.5, tcolor=None):
    color = color or C['ink']
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle=style, color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0,
                                                    mutation_scale=9, connectionstyle=f'arc3,rad={rad}'))
    if text:
        ax.annotate(text, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), xytext=toff, textcoords='offset points', ha='center',
                    va='bottom', fontsize=tfs, color=tcolor or color)


# ----------------------------------------------------------------------------------------------------------- chapter 4
def stack_launch():
    """Which launch file starts what, as layers from simulator to navigation, with the topics that cross the layers."""
    fig, ax = fs.diagram(8.8, 5.0, (0, 17.6), (0, 10), equal=False)
    rows = [  # (launch key, y, title, [node labels])
        ('nav', 7.0, 'nav.launch.py  (terminal 3)', ['map_server', 'amcl', 'planner_server', 'controller_server',
                                                      'bt_navigator', 'behavior_server', 'velocity_smoother', 'slam_toolbox']),
        ('est', 4.1, 'estimation.launch.py  (terminal 2)', ['pointcloud_to_laserscan', 'lidar_odom', 'odom_ekf']),
        ('sim', 0.6, 'sim.launch.py  (terminal 1)', ['Gazebo + arena.world', 'sensor plugins', 'ros2_control',
                                                     'quadruped_controller_node', 'state_estimation_node',
                                                     'base_to_footprint_ekf', 'robot_state_publisher']),
    ]
    for key, y, title, names in rows:
        col = LAUNCH[key]
        ax.add_patch(FancyBboxPatch((0.2, y), 16.4, 2.3, boxstyle='round,pad=0,rounding_size=0.12', fc='white', ec=col, lw=1.2))
        ax.text(0.45, y + 2.0, title, fontsize=9, color=col, weight='semibold', va='center', family=MONO)
        n = len(names); per = 4 if n > 4 else n
        w = (16.0 - 0.25 * (per - 1)) / per
        for i, nm in enumerate(names):
            r, c = divmod(i, per)
            node(ax, 0.4 + c * (w + 0.25), y + 1.0 - r * 0.85, w, 0.65, nm, col, fs_=7.6)
    # what crosses the layers
    arr(ax, (3.2, 2.9), (3.2, 4.1), C['ink'], text='/imu/data 100 Hz\n/lidar/points 10 Hz', toff=(-58, -12))
    arr(ax, (8.4, 6.4), (8.4, 7.0), C['ink'], text='/odom   /scan   /tf', toff=(42, -6))
    arr(ax, (16.6, 8.15), (17.1, 8.15), C['plan'], lw=1.2, style='-'); arr(ax, (17.1, 8.15), (17.1, 1.75), C['plan'], lw=1.2, style='-')
    arr(ax, (17.1, 1.75), (16.6, 1.75), C['plan'], lw=1.2)
    ax.text(17.25, 5.0, '/cmd_vel', rotation=-90, color=C['plan'], fontsize=8, va='center', family=MONO)
    ax.text(16.6, 9.75, 'ROS_DOMAIN_ID=42, every node with use_sim_time', ha='right', fontsize=7.8, color=C['muted'])
    save(fig, 'stack-launch')


def ros_graph():
    """The data path of this stack: nodes (rounded, coloured by launch file) and topics (square, grey)."""
    fig, ax = fs.diagram(9.0, 6.0, (0, 18), (0, 12.0), equal=False)
    S, E, N = LAUNCH['sim'], LAUNCH['est'], LAUNCH['nav']
    node(ax, 0.2, 4.6, 2.6, 2.4, 'Gazebo\n+ sensor\nplugins\n+ ros2_control', S, fs_=7.8)
    for name, y in (('/lidar/points', 9.0), ('/imu/data', 7.2), ('/joint_states', 3.0), ('/odom_gt', 1.3)):
        topic(ax, 3.6, y, 2.6, 0.7, name)
        arr(ax, (2.8, 5.8), (3.6, y + 0.35), S, lw=0.7)
    ax.text(4.9, 1.2, 'evaluation only', ha='center', va='top', fontsize=7, color=C['muted'])
    # middle nodes
    node(ax, 7.0, 9.6, 3.3, 0.8, 'pointcloud_to_laserscan', E, fs_=7.2)
    node(ax, 7.0, 8.0, 3.3, 0.8, 'lidar_odom', E)
    node(ax, 7.0, 6.0, 3.3, 0.8, 'odom_ekf', E)
    ax.text(8.65, 5.9, 'also writes /tf odom→base_footprint', fontsize=6.6, color=E, ha='center', va='top')
    node(ax, 7.0, 3.0, 3.3, 0.8, 'robot_state_publisher', S, fs_=7.4)
    node(ax, 7.0, 1.6, 3.3, 0.8, 'state_estimation_node', S, fs_=7.4)
    arr(ax, (6.2, 9.35), (7.0, 10.0), E, lw=0.7); arr(ax, (6.2, 9.35), (7.0, 8.4), E, lw=0.7)
    arr(ax, (6.2, 7.55), (7.0, 8.3), E, lw=0.7); arr(ax, (6.2, 7.55), (7.0, 6.4), E, lw=0.7)
    arr(ax, (6.2, 3.35), (7.0, 3.4), S, lw=0.7); arr(ax, (6.2, 3.35), (7.0, 2.0), S, lw=0.7)
    # second topic column
    topic(ax, 11.0, 9.65, 1.9, 0.7, '/scan'); topic(ax, 11.0, 8.05, 1.9, 0.7, '/lidar_odom'); topic(ax, 11.0, 6.05, 1.9, 0.7, '/odom')
    arr(ax, (10.3, 10.0), (11.0, 10.0), E, lw=0.7); arr(ax, (10.3, 8.4), (11.0, 8.4), E, lw=0.7)
    arr(ax, (11.95, 8.05), (10.3, 6.6), E, lw=0.7, rad=-0.25); arr(ax, (10.3, 6.4), (11.0, 6.4), E, lw=0.7)
    # tf bar
    ax.add_patch(Rectangle((7.0, 0.1), 10.8, 0.7, fc=C['fill'], ec=C['muted'], lw=0.7))
    ax.text(12.4, 0.45, '/tf   /tf_static   (written by 4 nodes, read by most)', ha='center', va='center', fontsize=7.6,
            family=MONO)
    arr(ax, (10.3, 3.2), (11.0, 3.2), S, lw=0.7); ax.text(10.35, 3.3, '/tf', fontsize=6.6, color=S, family=MONO)
    arr(ax, (11.0, 3.2), (11.4, 0.8), S, lw=0.7)
    arr(ax, (10.3, 2.0), (11.0, 2.0), S, lw=0.7); topic(ax, 11.0, 1.65, 3.2, 0.7, '/base_to_footprint_pose', fs_=7.0)
    node(ax, 14.6, 1.65, 3.2, 0.7, 'base_to_footprint_ekf', S, fs_=7.2); arr(ax, (14.2, 2.0), (14.6, 2.0), S, lw=0.7)
    arr(ax, (16.2, 1.65), (16.2, 0.8), S, lw=0.7)
    # nav column
    node(ax, 13.7, 9.5, 2.0, 0.8, 'amcl', N); node(ax, 16.0, 9.5, 1.9, 0.8, 'map_server', N, fs_=7.2)
    ax.text(13.7, 10.45, 'also writes /tf map→odom', fontsize=6.6, color=N, ha='left', va='bottom')
    node(ax, 13.7, 7.9, 4.2, 0.8, 'slam_toolbox (passive)', N, fs_=7.6)
    node(ax, 13.7, 5.2, 4.2, 1.6, 'planner_server\ncontroller_server\nbt_navigator, ...', N, fs_=7.4)
    topic(ax, 16.6, 10.95, 1.3, 0.55, '/map', fs_=7.2); arr(ax, (17.25, 10.3), (17.25, 10.95), N, lw=0.7)
    arr(ax, (16.6, 11.2), (15.7, 10.0), N, lw=0.7)
    arr(ax, (12.9, 10.0), (13.7, 9.9), E, lw=0.7); arr(ax, (12.9, 10.0), (13.7, 8.3), E, lw=0.7)
    arr(ax, (12.9, 9.9), (13.7, 6.4), E, lw=0.7)
    arr(ax, (12.9, 6.4), (13.7, 5.8), E, lw=0.7)
    topic(ax, 13.7, 7.0, 1.9, 0.6, '/amcl_pose', fs_=7.0); arr(ax, (14.4, 9.5), (14.4, 7.6), N, lw=0.7)
    topic(ax, 16.2, 7.0, 1.7, 0.6, '/slam/map', fs_=7.0); arr(ax, (17.0, 7.9), (17.0, 7.6), N, lw=0.7)
    # command path back to the robot
    topic(ax, 15.9, 4.2, 1.9, 0.7, '/cmd_vel'); arr(ax, (16.85, 5.2), (16.85, 4.9), C['plan'], lw=1.1)
    node(ax, 11.0, 4.15, 4.2, 0.8, 'quadruped_controller_node', S, fs_=7.2)
    arr(ax, (15.9, 4.55), (15.2, 4.55), C['plan'], lw=1.1)
    arr(ax, (11.0, 4.55), (2.8, 5.0), C['plan'], lw=1.1)
    ax.text(3.3, 5.08, 'joint_group_effort_controller/joint_trajectory', fontsize=6.6, color=C['plan'], rotation=-2.0,
            family=MONO, ha='left', va='bottom')
    # legend
    for i, (k, lab) in enumerate((('sim', 'sim.launch.py'), ('est', 'estimation.launch.py'), ('nav', 'nav.launch.py'))):
        node(ax, 0.2 + i * 2.55, 11.2, 2.4, 0.6, lab, LAUNCH[k], fs_=6.8)
    topic(ax, 7.95, 11.2, 0.9, 0.6, 'topic', fs_=6.8)
    ax.text(9.0, 11.5, 'command path in amber; /clock and /rosout left out', fontsize=6.8, color=C['muted'], va='center')
    save(fig, 'ros-graph')


def mechanisms():
    """Topic, service, action, parameter, each with the example from this stack and the number the chapter measured."""
    fig, axs = plt.subplots(2, 2, figsize=(8.6, 5.2))
    for ax in axs.flat:
        ax.set_xlim(0, 10); ax.set_ylim(0, 6); ax.axis('off')
    a, b, c, d = axs.flat
    a.set_title('Topic: one-to-many stream')
    node(a, 0.2, 2.5, 2.6, 1.0, 'imu_plugin', LAUNCH['sim'])
    topic(a, 3.8, 2.6, 2.4, 0.8, '/imu/data')
    node(a, 7.2, 3.9, 2.6, 0.9, 'lidar_odom', LAUNCH['est']); node(a, 7.2, 1.2, 2.6, 0.9, 'odom_ekf', LAUNCH['est'])
    arr(a, (2.8, 3.0), (3.8, 3.0)); arr(a, (6.2, 3.0), (7.2, 4.3)); arr(a, (6.2, 3.0), (7.2, 1.65))
    a.text(5.0, 4.3, '100 Hz\n(99.751 Hz, -s)', ha='center', fontsize=7.5, color=C['muted'])
    a.text(5.0, 0.4, 'publisher count 1, subscription count 2', ha='center', fontsize=7.5, color=C['muted'])
    b.set_title('Service: request and reply')
    node(b, 0.2, 2.3, 3.0, 1.2, 'ros2 param get\n(client)', C['muted'], mono=False)
    node(b, 6.6, 2.3, 3.2, 1.2, '/lidar_odom\n(server)', LAUNCH['est'])
    arr(b, (3.2, 3.2), (6.6, 3.2), text='get_parameters [voxel]', tfs=7.3)
    arr(b, (6.6, 2.6), (3.2, 2.6), text='', tfs=7.3)
    b.text(4.9, 1.9, 'Double value is: 0.15', ha='center', fontsize=7.5, family=MONO, color=C['ink'])
    b.text(5.0, 0.6, 'each node serves 6 parameter services; 316 services in all', ha='center', fontsize=7.5, color=C['muted'])
    c.set_title('Action: goal, feedback, result (/spin)')
    c.plot([1.6, 1.6], [0.3, 5.0], color=C['muted'], lw=0.8); c.plot([8.4, 8.4], [0.3, 5.0], color=C['muted'], lw=0.8)
    c.text(1.6, 5.3, 'ros2 action send_goal', ha='center', fontsize=7.5); c.text(8.4, 5.3, 'behavior_server', ha='center', fontsize=7.5,
                                                                                    family=MONO)
    arr(c, (1.6, 4.6), (8.4, 4.4), text='goal: target_yaw 1.57', tfs=7.3)
    arr(c, (8.4, 4.1), (1.6, 3.9), color=C['muted'], text='accepted', tfs=7.3)
    for i, y in enumerate(np.linspace(3.4, 1.6, 7)):
        arr(c, (8.4, y), (1.6, y - 0.12), color=C['est'], lw=0.6)
    c.text(5.0, 2.55, '41 feedback messages\nangular_distance_traveled', ha='center', fontsize=7.3, color=C['est'],
           bbox=dict(fc='white', ec='none', pad=1))
    arr(c, (8.4, 1.0), (1.6, 0.8), color=C['good'], lw=1.1, text='result: SUCCEEDED, 3.38 s', tfs=7.3)
    c.text(8.6, 2.3, '1.62 rad\nreached\n(overshoot)', fontsize=7, color=C['muted'], va='center')
    d.set_title('Parameter: typed setting inside a node')
    d.add_patch(FancyBboxPatch((0.5, 0.6), 9.0, 4.4, boxstyle='round,pad=0,rounding_size=0.15', fc='white', ec=LAUNCH['est']))
    d.text(0.8, 4.6, '/lidar_odom', fontsize=8.5, family=MONO, va='center')
    rows = [('name', 'type', 'value'), ('voxel', 'double', '0.15'), ('fix_yaw', 'boolean', 'true'),
            ('use_sim_time', 'boolean', 'true'), ('base_frame, ...', '', '')]
    for i, (n, t, v) in enumerate(rows):
        y = 3.85 - i * 0.68
        kw = dict(fontsize=7.8, va='center', family=MONO, color=C['muted'] if i == 0 else C['ink'])
        d.text(1.0, y, n, **kw); d.text(4.8, y, t, **kw); d.text(7.4, y, v, **kw)
    d.plot([0.9, 9.1], [3.5, 3.5], color=C['grid'], lw=0.8)
    d.text(5.0, 0.25, 'the declared default fixes the type: 25 is rejected, 25.0 accepted', ha='center', fontsize=7.5,
           color=C['muted'])
    save(fig, 'mechanisms')


def qos_matrix():
    """Request-offered compatibility for reliability and durability, with this stack's examples in the cells."""
    fig, axs = plt.subplots(1, 2, figsize=(8.8, 3.9))
    good, bad, part = '#e3f1e8', '#f6e0de', '#f7ecd6'
    panels = [
        ('Reliability', ('RELIABLE', 'BEST_EFFORT'),
         [[(good, 'works', '/amcl_pose → go_to_goal.py'), (good, 'works', '/imu/data → lidar_odom\n(sensor data QoS)')],
          [(bad, 'no messages', '/scan echoed with\n--qos-reliability reliable'), (good, 'works', '/scan → its subscribers')]]),
        ('Durability', ('TRANSIENT_LOCAL', 'VOLATILE'),
         [[(good, 'works, gets the\nlast sample at once', '/amcl_pose → go_to_goal.py\n/map → ros2 topic echo'),
           (part, 'compatible, no history', '/map echoed volatile:\nnothing, exit 124 after 10 s')],
          [(bad, 'no messages', ''), (good, 'works', 'most topics')]]),
    ]
    for ax, (title, vals, cells) in zip(axs, panels):
        ax.set_xlim(0, 2); ax.set_ylim(0, 2); ax.axis('off'); ax.set_aspect('equal'); ax.set_title(title, pad=34)
        for i in range(2):          # publisher offers, rows top->bottom
            for j in range(2):      # subscription requests, columns
                col, head, ex = cells[i][j]
                x, y = j, 1 - i
                ax.add_patch(Rectangle((x, y), 1, 1, fc=col, ec='white', lw=2))
                ax.text(x + 0.5, y + 0.68, head, ha='center', va='center', fontsize=8.2, weight='semibold')
                ax.text(x + 0.5, y + 0.3, ex, ha='center', va='center', fontsize=6.7, color=C['muted'])
        for k, v in enumerate(vals):
            ax.text(k + 0.5, 2.04, v, ha='center', va='bottom', fontsize=7.4, family=MONO)
            ax.text(-0.04, 1.5 - k, v, ha='right', va='center', fontsize=7.4, family=MONO)
        ax.text(1.0, 2.22, 'subscription requests', ha='center', va='bottom', fontsize=7.8, color=C['muted'])
        ax.text(-0.75, 1.0, 'publisher offers', ha='center', va='center', rotation=90, fontsize=7.8, color=C['muted'])
    fig.text(0.5, -0.02, 'Rule: a subscription may request less than the publisher offers, never more.', ha='center', fontsize=8.5)
    save(fig, 'qos-matrix')


def sim_time():
    """Same IMU messages counted per simulated second and per wall-clock second."""
    rtf = 81.158 / 99.751                      # ratio of the two `ros2 topic hz` readings in the chapter
    fig, ax = plt.subplots(figsize=(8.0, 2.6))
    k = np.arange(0, 130)
    t_sim = k / 100.0                          # IMU stamps, 100 per simulated second
    t_wall = t_sim / rtf                       # when they arrive on the wall clock if the sim runs at rtf
    ax.vlines(t_sim, 1.15, 1.75, color=C['est'], lw=0.8)
    ax.vlines(t_wall, 0.15, 0.75, color=C['gt'], lw=0.8)
    ax.axvline(1.0, color=C['ink'], lw=1.0, ls='--')
    ax.text(1.01, 1.95, '1 s', fontsize=8)
    ax.text(0.0, 1.95, 'per simulated second (ros2 topic hz -s): 100 stamps → 99.751 Hz', fontsize=8.3, color=C['est'])
    ax.text(0.0, 0.95, 'per wall-clock second (ros2 topic hz): about 81 arrivals → 81.158 Hz', fontsize=8.3,
            color=C['gt'])
    ax.set_xlim(-0.01, 1.25); ax.set_ylim(0, 2.2); ax.set_yticks([]); ax.grid(False)
    ax.spines['left'].set_visible(False)
    ax.set_xlabel('time on that clock [s]')
    ax.set_title(f'One IMU stream, two clocks (real-time factor 81.158 / 99.751 = {rtf:.2f} at that moment)')
    save(fig, 'sim-time')


def overlay():
    """Workspace build flow and the underlay/overlay lookup order."""
    fig, ax = fs.diagram(8.6, 3.8, (0, 17.2), (0, 7.6), equal=False)
    # left: build flow
    node(ax, 0.2, 5.4, 3.0, 1.0, 'src/\n13 packages', C['muted'], fs_=7.8)
    node(ax, 0.2, 3.0, 3.0, 1.0, 'build/', C['muted'], fs_=7.8)
    node(ax, 0.2, 0.6, 3.0, 1.0, 'install/', C['est'], fs_=7.8)
    arr(ax, (1.7, 5.4), (1.7, 4.0), text='colcon build', toff=(38, -4))
    arr(ax, (1.7, 3.0), (1.7, 1.6), text='install step', toff=(36, -4))
    arr(ax, (3.2, 1.1), (3.2, 5.9), C['est'], rad=0.45, ls='--')
    ax.text(4.6, 3.5, '--symlink-install:\nlinks back to src,\n.py and launch edits\nneed no rebuild', fontsize=7.3, color=C['est'],
            va='center')
    # right: lookup order
    ax.add_patch(FancyBboxPatch((8.0, 4.0), 9.0, 2.9, boxstyle='round,pad=0,rounding_size=0.12', fc='white', ec=C['est'], lw=1.1))
    ax.text(8.25, 6.55, 'overlay: ./install (this workspace)', fontsize=8.3, color=C['est'], weight='semibold')
    for i, p in enumerate(['go2_tutorials', 'go2_nav_sim', 'go2_description', '...', 'champ']):
        topic(ax, 8.25 + i * 1.73, 5.2, 1.63, 0.7, p, fs_=6.6)
    ax.add_patch(FancyBboxPatch((8.0, 0.5), 9.0, 2.6, boxstyle='round,pad=0,rounding_size=0.12', fc='white', ec=C['muted'], lw=1.1))
    ax.text(8.25, 2.75, 'underlay: /opt/ros/humble', fontsize=8.3, color=C['muted'], weight='semibold')
    for i, p in enumerate(['nav2_amcl', 'rclpy', 'tf2_ros', 'robot_localization', '...']):
        topic(ax, 8.25 + i * 1.73, 1.3, 1.63, 0.7, p, fs_=6.6)
    arr(ax, (9.05, 7.5), (9.05, 5.9), C['good'], lw=1.1)
    ax.text(9.25, 7.25, 'ros2 pkg prefix go2_tutorials → ./install/go2_tutorials', fontsize=7.2, color=C['good'], family=MONO)
    arr(ax, (16.9, 4.6), (9.9, 2.0), C['plan'], lw=1.1, rad=-0.25)
    ax.text(8.1, 3.55, 'ros2 pkg prefix nav2_amcl → /opt/ros/humble\n(not in the overlay, found below)', fontsize=7.0,
            color=C['plan'], ha='left', va='center', family=MONO)
    ax.text(12.5, 0.05, 'AMENT_PREFIX_PATH is searched top to bottom; the first match wins', fontsize=7.6, color=C['muted'],
            ha='center')
    save(fig, 'overlay')


def lifecycle():
    """Managed-node states with the IDs ros2 lifecycle prints, and the two managers of this stack."""
    fig, ax = fs.diagram(8.6, 3.9, (0, 17.2), (0, 7.8), equal=False)
    st = {'unconfigured [1]': (0.4, 4.6), 'inactive [2]': (4.9, 4.6), 'active [3]': (9.4, 4.6), 'finalized [4]': (4.9, 1.2)}
    for name, (x, y) in st.items():
        node(ax, x, y, 3.2, 1.0, name, C['good'] if 'active [3]' == name else C['ink'], fs_=8, lw=1.4 if 'active [3]' == name else 1)
    arr(ax, (3.6, 5.35), (4.9, 5.35), text='configure [1]', toff=(0, 3))
    arr(ax, (4.9, 4.85), (3.6, 4.85), color=C['muted'], text='cleanup [2]', toff=(0, -13), tcolor=C['muted'])
    arr(ax, (8.1, 5.35), (9.4, 5.35), text='activate [3]', toff=(0, 3))
    arr(ax, (9.4, 4.85), (8.1, 4.85), color=C['muted'], text='deactivate [4]', toff=(0, -13), tcolor=C['muted'])
    for x0 in (2.0, 6.5, 11.0):
        arr(ax, (x0, 4.6), (6.5 if x0 != 6.5 else 6.5, 2.2), color=C['bad'], lw=0.7, rad=0.0 if x0 == 6.5 else (0.15 if x0 < 6.5 else -0.15))
    ax.text(8.9, 2.9, 'shutdown [5] [6] [7]', fontsize=7.5, color=C['bad'])
    ax.text(0.4, 7.2, 'ros2 lifecycle get /amcl  →  active [3]', fontsize=8, family=MONO)
    # managers
    ax.add_patch(FancyBboxPatch((13.2, 4.1), 3.9, 3.2, boxstyle='round,pad=0,rounding_size=0.1', fc='white', ec=LAUNCH['nav']))
    ax.text(13.35, 6.95, 'lifecycle_manager_\nlocalization', fontsize=7.3, family=MONO, va='top')
    for i, n in enumerate(['1 map_server', '2 amcl']):
        ax.text(13.45, 5.55 - i * 0.55, n, fontsize=7.3, family=MONO)
    ax.add_patch(FancyBboxPatch((13.2, 0.2), 3.9, 3.7, boxstyle='round,pad=0,rounding_size=0.1', fc='white', ec=LAUNCH['nav']))
    ax.text(13.35, 3.6, 'lifecycle_manager_\nnavigation', fontsize=7.3, family=MONO, va='top')
    for i, n in enumerate(['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator',
                           'waypoint_follower', 'velocity_smoother']):
        ax.text(13.45, 2.3 - i * 0.3, f'{i + 1} {n}', fontsize=6.4, family=MONO)
    ax.text(15.15, 7.45, 'configure + activate in list order;\nbond_timeout 4.0 s', fontsize=7, ha='center', color=C['muted'])
    save(fig, 'lifecycle')


def executor():
    """Schematic callback timelines for lidar_odom before and after the callback-group fix (durations not to scale)."""
    fig, axs = plt.subplots(2, 1, figsize=(8.2, 3.6), sharex=True)
    imu_t = np.arange(0.0, 10.0, 0.5)            # schematic arrival times of IMU messages
    reg = (2.2, 6.4)                             # one registration (schematic)
    for ax in axs:
        ax.set_ylim(-0.2, 2.2); ax.set_yticks([]); ax.grid(False); ax.spines['left'].set_visible(False); ax.set_xlim(0, 10)
        ax.vlines(imu_t, 1.85, 2.15, color=C['muted'], lw=0.8)
        ax.text(10.05, 2.0, 'IMU msgs\narrive', fontsize=7, va='center', color=C['muted'])
    a, b = axs
    a.set_title('Before: rclpy.spin(node), one thread, one callback group')
    a.add_patch(Rectangle((reg[0], 0.9), reg[1] - reg[0], 0.6, fc=C['est'], alpha=0.85))
    a.text(sum(reg) / 2, 1.2, 'on_cloud: registration', ha='center', va='center', color='white', fontsize=8)
    for t in imu_t:
        if reg[0] <= t < reg[1] + 0.3:
            continue
        a.add_patch(Rectangle((t, 0.9), 0.18, 0.6, fc=C['plan']))
    q = [t for t in imu_t if reg[0] <= t < reg[1]]
    for i, t in enumerate(q):
        a.plot([t], [0.4], 'x' if i < 4 else 'o', color=C['bad'] if i < 4 else C['plan'], ms=5)
        if i >= 4:                                   # the survivors run in a burst once the registration returns
            a.add_patch(Rectangle((reg[1] + 0.02 + (i - 4) * 0.07, 0.9), 0.05, 0.6, fc=C['plan']))
    a.text(reg[0], -0.05, 'IMU callbacks wait in the queue (o, run late); some miss the next scan\'s integration window (x)', fontsize=7.5,
           color=C['bad'], va='bottom')
    a.text(0.0, 1.62, 'thread 1', fontsize=7.5, color=C['muted'])
    b.set_title('After: MultiThreadedExecutor(3), IMU and cloud in separate mutually exclusive groups')
    b.add_patch(Rectangle((reg[0], 0.15), reg[1] - reg[0], 0.6, fc=C['est'], alpha=0.85))
    b.text(sum(reg) / 2, 0.45, 'on_cloud: registration', ha='center', va='center', color='white', fontsize=8)
    for t in imu_t:
        b.add_patch(Rectangle((t, 1.05), 0.18, 0.6, fc=C['plan']))
    b.text(0.0, 1.72, 'thread A: on_imu', fontsize=7.5, color=C['muted']); b.text(0.0, 0.82, 'thread B: on_cloud', fontsize=7.5,
                                                                                    color=C['muted'])
    b.set_xlabel('time (schematic, not to scale)'); b.set_xticks([])
    save(fig, 'executor')


def bag_clock():
    """Two ways to give a replay a clock, and which one matches the message stamps of these bags."""
    fig, ax = fs.diagram(8.6, 3.4, (0, 17.2), (0, 6.8), equal=False)
    ax.add_patch(FancyBboxPatch((0.2, 1.0), 4.4, 4.8, boxstyle='round,pad=0,rounding_size=0.12', fc='white', ec=C['muted']))
    ax.text(0.45, 5.4, 'bags/v3  (mcap, 69.8 s)', fontsize=8, family=MONO, weight='semibold')
    for i, (t, n) in enumerate((('/clock', '13900'), ('/tf', '25029'), ('/imu/data', '6953'), ('/odom_gt', '3477'),
                                ('/lidar/points', '696'))):
        ax.text(0.45, 4.7 - i * 0.62, t, fontsize=7.6, family=MONO); ax.text(4.35, 4.7 - i * 0.62, n, fontsize=7.6, ha='right',
                                                                           family=MONO, color=C['muted'])
    ax.text(0.45, 1.3, 'stamps: simulated s (/odom_gt ≈ 52 s)', fontsize=7.0, color=C['muted'])
    # option 1
    node(ax, 6.0, 4.0, 5.2, 1.4, 'ros2 bag play bags/v3 --clock\n--topics /imu/data /odom_gt', C['bad'], fs_=7.2)
    node(ax, 6.0, 1.0, 5.2, 1.4, 'ros2 bag play bags/v3 --topics\n/clock /odom_gt /tf /tf_static', C['good'], fs_=7.2)
    arr(ax, (4.6, 4.2), (6.0, 4.7)); arr(ax, (4.6, 2.6), (6.0, 1.7))
    ax.text(11.6, 5.05, '/clock = time the recorder received\neach message: sec 1795810128', fontsize=7.6, color=C['bad'],
            va='center')
    ax.text(11.6, 4.1, 'stamps say ≈ 52 s: use_sim_time nodes\nand the data disagree by decades', fontsize=7.3, color=C['muted'],
            va='center')
    ax.text(11.6, 2.05, "/clock = the simulator's own recorded\nclock: sec 50", fontsize=7.6, color=C['good'], va='center')
    ax.text(11.6, 1.1, 'clock and stamps agree', fontsize=7.3, color=C['muted'], va='center')
    save(fig, 'bag-clock')


def yaw_drift():
    """yaw_check output quoted in the chapter: integrated gyro minus ground truth."""
    fig, ax = plt.subplots(figsize=(7.2, 3.3))
    ts, es = [0, 10, 20, 30, 40, 50, 60], [0.00, 0.57, 1.51, 2.36, 3.92, 5.07, 6.37]
    tt, et = [10, 30, 60, 90], [-1.61, 1.15, 2.76, 5.16]
    ax.plot(ts, es, 'o-', color=C['est'], label='standing still, 60 s')
    ax.plot(tt, et, 's-', color=C['est2'], label='turning at about 0.47 rad/s, 90 s')
    ax.plot([60], [-4.02], 'D', color=C['plan'], label='bags/v3 replay, first 60 s')
    ax.axhline(0, color=C['gt'], lw=1.0)
    ax.fill_between([0, 90], -0.01, 0.01, color=C['gt'], alpha=0.5)
    ax.text(91, 0.25, 'configured gyro noise alone:\nabout 0.01° after 60 s', fontsize=7.5, color=C['muted'], va='bottom', ha='right')
    ax.set_xlabel('simulated time since start [s]'); ax.set_ylabel('imu_yaw − ground truth [deg]')
    ax.set_title('yaw_check: integrated gyro yaw minus ground truth')
    ax.set_xlim(-2, 92); ax.legend(loc='upper left')
    save(fig, 'yaw-drift')


# ----------------------------------------------------------------------------------------------------------- chapter 5
def tf_tree():
    """The TF tree of the running stack, from the view_frames output in chapter 5."""
    fig, ax = fs.diagram(9.9, 6.6, (0, 19.8), (0, 13.2), equal=False)
    pub = dict(amcl=LAUNCH['nav'], ekf=LAUNCH['est'], champ=C['plan'], rsp=C['good'], static=C['muted'])
    W, H = 2.5, 0.72
    pos = {'map': (9.0, 12.2), 'odom': (9.0, 10.8), 'base_footprint': (9.0, 9.4), 'base_link': (9.0, 8.0),
           'trunk': (7.2, 6.6), 'chase_cam': (10.6, 6.6), 'lidar_link': (14.4, 6.6), 'camera_link': (17.1, 6.6),
           'imu_link': (0.2, 5.2), 'lidar_visual': (14.4, 5.2), 'camera_optical': (17.1, 5.2)}
    legs = ('lf', 'rf', 'lh', 'rh')                 # URDF leg prefixes (go2_config links.yaml)
    for i, leg in enumerate(legs):
        x = 3.0 + i * 2.8
        for j, part in enumerate(('hip_link', 'upper_leg_link', 'lower_leg_link', 'foot_link')):
            pos[f'{leg}_{part}'] = (x, 5.2 - j * 1.35)
    edges = [('map', 'odom', 'amcl', '10.3 Hz'), ('odom', 'base_footprint', 'ekf', '50.3 Hz'),
             ('base_footprint', 'base_link', 'champ', '49.8 Hz'),
             ('base_link', 'trunk', 'static', ''), ('base_link', 'lidar_link', 'static', ''), ('base_link', 'camera_link', 'static', ''),
             ('base_link', 'chase_cam', 'static', ''), ('trunk', 'imu_link', 'static', ''),
             ('lidar_link', 'lidar_visual', 'static', ''), ('camera_link', 'camera_optical', 'static', '')]
    for leg in legs:
        edges += [('trunk', f'{leg}_hip_link', 'rsp', ''), (f'{leg}_hip_link', f'{leg}_upper_leg_link', 'rsp', ''),
                  (f'{leg}_upper_leg_link', f'{leg}_lower_leg_link', 'rsp', ''), (f'{leg}_lower_leg_link', f'{leg}_foot_link', 'static', '')]
    for p, c, who, rate in edges:
        (x0, y0), (x1, y1) = pos[p], pos[c]
        ax.annotate('', xy=(x1 + W / 2, y1 + H), xytext=(x0 + W / 2, y0),
                    arrowprops=dict(arrowstyle='-|>', color=pub[who], lw=1.4 if who != 'static' else 0.8,
                                    ls='--' if who == 'static' else '-', shrinkA=0, shrinkB=0, mutation_scale=8))
        if rate:
            ax.text(x0 + W / 2 + 0.15, (y0 + y1 + H) / 2, rate, fontsize=7.4, color=pub[who], va='center')
    for name, (x, y) in pos.items():
        bold = name in ('map', 'odom', 'base_footprint', 'base_link')
        node(ax, x, y, W, H, name, C['ink'] if bold else C['muted'], fs_=6.9 if len(name) > 14 else 7.4, lw=1.2 if bold else 0.8)
    ax.text(11.9, 9.75, 'constant (0, 0, 0.426):\nwrong, see below', fontsize=7.2, color=C['bad'], va='center')
    lx, ly = 13.6, 12.6
    for i, (k, lab) in enumerate((('amcl', 'amcl'), ('ekf', 'odom_ekf'), ('champ', 'base_to_footprint_ekf (CHAMP)'),
                                  ('rsp', 'robot_state_publisher, 250 Hz from /joint_states'), ('static', 'static (/tf_static)'))):
        y = ly - i * 0.5
        ax.plot([lx, lx + 0.7], [y, y], color=pub[k], lw=1.4 if k != 'static' else 0.8, ls='--' if k == 'static' else '-')
        ax.text(lx + 0.85, y, lab, fontsize=7.0, va='center')
    save(fig, 'tf-tree')


def amcl_vs_odom():
    """Recorded run w2: odometry is continuous and drifts, AMCL corrects in steps. map->odom is the difference."""
    run = json.loads((ROOT / 'site/data/runs/w2/run.json').read_text())
    S = [r for r in run['samples'] if 'odom' in r and 'amcl' in r]
    t = np.array([r['t'] for r in S]); gt = np.array([r['gt'] for r in S]); od = np.array([r['odom'] for r in S])
    am = np.array([r['amcl'] for r in S])

    def se2_err(a, b):        # position difference a - b in metres
        return np.hypot(a[:, 0] - b[:, 0], a[:, 1] - b[:, 1])
    fig, (a, b) = plt.subplots(1, 2, figsize=(9.0, 3.7), gridspec_kw=dict(width_ratios=[1, 1.35]))
    for w in run['arena']['walls']:
        a.add_patch(Rectangle((w[0] - w[2] / 2, w[1] - w[3] / 2), w[2], w[3], fc=C['grid'], ec='none'))
    for bx in run['arena']['boxes']:
        a.add_patch(Rectangle((bx[0] - bx[2] / 2, bx[1] - bx[3] / 2), bx[2], bx[3], fc=C['grid'], ec='none'))
    for cy in run['arena']['cylinders']:
        a.add_patch(Circle((cy[0], cy[1]), cy[2], fc=C['grid'], ec='none'))
    a.plot(gt[:, 0], gt[:, 1], color=C['gt'], lw=2.2, label='ground truth')
    a.plot(od[:, 0], od[:, 1], color=C['est'], lw=1.0, label='odometry (odom)')
    a.plot(am[:, 0], am[:, 1], '.', color=C['est2'], ms=2.2, label='AMCL (map)')
    a.set_aspect('equal'); a.set_xlim(-6.2, 6.2); a.set_ylim(-5.2, 5.2); a.set_title('Run w2, A to B, in map')
    a.set_xlabel('x [m]'); a.set_ylabel('y [m]'); a.legend(loc='upper left', fontsize=7)
    b.plot(t, 100 * se2_err(od, gt), color=C['est'], label='odometry − ground truth')
    b.plot(t, 100 * se2_err(am, gt), color=C['est2'], drawstyle='steps-post', label='AMCL − ground truth')
    b.set_xlabel('time since start of recording [s]'); b.set_ylabel('position error [cm]')
    m = run['metrics']
    b.set_title(f"AMCL at its {m['amcl_updates']} updates: RMSE {m['amcl_rmse_cm']} cm, max {m['amcl_max_cm']} cm")
    b.text(0.6, 0.99, 'held pose between updates drawn as steps,\nso this trace peaks higher', transform=b.transAxes,
           ha='center', va='top', fontsize=7, color=C['muted'])
    b.legend(loc='upper left', fontsize=7)
    save(fig, 'amcl-vs-odom')


def one_parent():
    fig, (a, b) = plt.subplots(1, 2, figsize=(8.0, 2.8))
    for ax in (a, b):
        ax.set_xlim(0, 10); ax.set_ylim(0, 5.6); ax.axis('off')
    a.set_title('One parent: a tree')
    node(a, 3.5, 4.2, 3.0, 0.8, 'map', C['ink']); node(a, 3.5, 2.3, 3.0, 0.8, 'odom', C['ink'])
    node(a, 3.5, 0.4, 3.0, 0.8, 'base_footprint', C['ink'], fs_=7.4)
    arr(a, (5.0, 4.2), (5.0, 3.1), LAUNCH['nav'], lw=1.3, text='amcl', toff=(18, -6))
    arr(a, (5.0, 2.3), (5.0, 1.2), LAUNCH['est'], lw=1.3, text='odom_ekf', toff=(26, -6))
    b.set_title('Two publishers of one child: last message wins')
    node(b, 0.5, 4.2, 3.0, 0.8, 'map', C['ink']); node(b, 6.5, 4.2, 3.0, 0.8, 'slam_map', C['muted'])
    node(b, 3.5, 2.3, 3.0, 0.8, 'odom', C['bad']); node(b, 3.5, 0.4, 3.0, 0.8, 'base_footprint', C['ink'], fs_=7.4)
    arr(b, (2.0, 4.2), (4.3, 3.1), LAUNCH['nav'], lw=1.3, text='amcl', toff=(-22, -6))
    arr(b, (8.0, 4.2), (5.7, 3.1), C['bad'], lw=1.0, ls='--', text='slam_toolbox', toff=(34, -6))
    arr(b, (5.0, 2.3), (5.0, 1.2), LAUNCH['est'], lw=1.3)
    b.text(5.0, -0.1, 'config/slam.yaml: transform_publish_period 0.0, so this edge is never sent', ha='center', fontsize=7.2,
           color=C['muted'])
    save(fig, 'one-parent')


def optical_frame():
    """camera_link and camera_optical axes at the same origin, with the optical axis ray to a 2 m depth pixel."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    fig = plt.figure(figsize=(8.4, 3.9))
    for k, (title, show_link, show_opt) in enumerate((('camera_link (REP-103 body axes)', True, False),
                                                      ('camera_optical (z forward, x right, y down)', False, True),
                                                      ('both, same origin', True, True))):
        ax = fig.add_subplot(1, 3, k + 1, projection='3d')
        ax.set_proj_type('ortho'); ax.view_init(elev=22, azim=-128)
        cols = ('#c0392b', '#1e8449', '#1f5fbf')
        if show_link:
            for i, n in enumerate('xyz'):
                v = np.eye(3)[i]
                ax.quiver(0, 0, 0, *v, color=cols[i], lw=2.0 if not show_opt else 1.0, arrow_length_ratio=0.15,
                          alpha=1 if not show_opt else 0.45)
                if not (show_opt and n == 'x'):          # x_link lies under z_opt in the combined panel
                    ax.text(*(1.12 * v), n + ('' if not show_opt else '$_{link}$'), color=cols[i],
                            fontsize=9 if not show_opt else 7.5, alpha=1 if not show_opt else 0.6)
        if show_opt:
            for i, n in enumerate('xyz'):
                v = R[:, i]
                ax.quiver(0, 0, 0, *v, color=cols[i], lw=2.0, arrow_length_ratio=0.15)
                lab = n if not show_link else n + '$_{opt}$'
                ax.text(*(1.2 * v + np.array([0, 0, 0.08])), lab, color=cols[i], fontsize=9)
        ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([]); ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()
        ax.set_title(title, fontsize=8.5, loc='center')
    fig.text(0.5, 0.01, r'$x_{opt} = -y_{link}$,   $y_{opt} = -z_{link}$,   $z_{opt} = x_{link}$   (columns of $R$ from rpy = (−π/2, 0, −π/2))',
             ha='center', fontsize=9)
    save(fig, 'optical-frame')


def tilt_error():
    fig, (a, b) = plt.subplots(1, 2, figsize=(8.6, 3.0), gridspec_kw=dict(width_ratios=[1.0, 1.25]))
    th = math.radians(12)                      # exaggerated for the sketch
    a.set_xlim(-0.3, 3.6); a.set_ylim(-0.25, 1.0); a.axis('off'); a.set_aspect('equal')
    a.plot([0, 3.2], [0, 0], color=C['gt'], lw=1.2); a.plot([0, 3.2 * math.cos(th)], [0, 3.2 * math.sin(th)], color=C['est'], lw=1.4)
    a.plot([3.2 * math.cos(th)] * 2, [0, 3.2 * math.sin(th)], ':', color=C['bad'])
    a.text(3.22 * math.cos(th) + 0.06, 1.6 * math.sin(th), r'$\Delta z = d\sin\theta$', color=C['bad'], fontsize=9, va='center')
    a.text(1.5, -0.12, 'where TF places the point (trunk assumed level)', fontsize=7, color=C['gt'], ha='center', va='top')
    a.text(1.3, 0.42, 'actual ray, trunk tilted by θ', fontsize=7, color=C['est'], rotation=math.degrees(th), ha='center')
    a.text(0.55, 0.03, 'θ', fontsize=9); a.text(1.5, 0.58, '', fontsize=8)
    a.set_title('Sketch (θ exaggerated)')
    d = np.linspace(0, 8, 100)
    for deg, col, lab in ((3.6, C['bad'], 'roll 3.6°, chapter 7 walk'), (2.95, C['plan'], 'pitch −2.95°, bags/v3'),
                          (1.63, C['est'], 'pitch +1.63°, bags/v3')):
        b.plot(d, d * math.sin(math.radians(deg)), color=col, label=lab)
    b.plot([3, 3], [0, 3 * math.sin(math.radians(3.6))], ':', color=C['muted'])
    b.plot([3], [3 * math.sin(math.radians(3.6))], 'o', color=C['bad']); b.annotate('0.188 m at 3 m', (3, 0.188), xytext=(8, 12),
                                                                                   textcoords='offset points', fontsize=7.5)
    b.plot([3], [3 * math.sin(math.radians(2.95))], 'o', color=C['plan']); b.annotate('0.154 m', (3, 0.154), xytext=(8, -12),
                                                                                       textcoords='offset points', fontsize=7.5)
    b.set_xlabel('distance to the point d [m]'); b.set_ylabel('height error Δz [m]'); b.legend(loc='upper left', fontsize=7.2)
    b.set_title('Height error from a tilt TF does not know about')
    save(fig, 'tilt-error')


def lookup_time():
    """The three ExtrapolationException messages of chapter 5 on their own time axes."""
    cases = [('Into the past by 4 ms: the listener had just started', 880.096, 880.100, None, (880.06, 880.14)),
             ('Into the future by 0.511 s: asked for now + 0.5 s', 935.490, None, 934.979, (934.3, 935.7)),
             ('Into the past by 29.4 s: asked for now − 30 s', 904.990, 934.400, None, (902, 937))]
    fig, axs = plt.subplots(3, 1, figsize=(8.0, 3.9))
    for ax, (title, req, earliest, latest, xl) in zip(axs, cases):
        ax.set_xlim(*xl); ax.set_ylim(0, 1); ax.set_yticks([]); ax.grid(False); ax.spines['left'].set_visible(False)
        lo = earliest if earliest is not None else xl[0]
        hi = latest if latest is not None else xl[1]
        ax.add_patch(Rectangle((lo, 0.3), hi - lo, 0.4, fc=C['est'], alpha=0.25, ec=C['est']))
        wide = (hi - lo) > 0.25 * (xl[1] - xl[0])
        ax.text(lo + (hi - lo) * 0.5 if wide else lo - 0.01 * (xl[1] - xl[0]), 0.5, 'transforms in the buffer',
                ha='center' if wide else 'right', va='center', fontsize=7.5, color=C['est'])
        ax.axvline(req, color=C['bad'], lw=1.5)
        ax.text(req, 0.95, f' requested {req:.3f}', color=C['bad'], fontsize=7.5, va='top',
                ha='left' if req < (xl[0] + xl[1]) / 2 else 'right')
        edge = earliest if earliest is not None else latest
        ax.text(edge, 0.05, f'{"earliest" if earliest is not None else "latest"} {edge:.3f}', fontsize=7.2, color=C['est'],
                ha='left' if earliest is not None else 'right')
        ax.set_title(title, fontsize=8.8)
        ax.tick_params(axis='x', labelsize=7.5)
    axs[-1].set_xlabel('simulated time [s]')
    save(fig, 'lookup-time')


def compose_chain():
    """One lidar point carried through lidar_link -> base_link -> base_footprint -> odom, with the numbers of chapter 5."""
    fig = plt.figure(figsize=(9.0, 3.7))
    ax = fig.add_axes([0, 0.0, 0.66, 1]); ax.set_xlim(0, 13.2); ax.set_ylim(0, 7.4); ax.axis('off')
    frames = [('lidar_link', '(1.2545, −0.0044, 0.0219)'), ('base_link', '(1.2545, −0.0044, 0.1219)'),
              ('base_footprint', '(1.2545, −0.0044, 0.5479)'), ('odom', '(0.7036, 0.4181, 0.5479)')]
    ys = [6.2, 4.4, 2.6, 0.8]
    for (name, p), y in zip(frames, ys):
        node(ax, 0.3, y - 0.4, 3.6, 0.9, name, C['ink'], fs_=8.2)
        ax.text(4.3, y + 0.05, p, fontsize=8.2, family=MONO, va='center')
    labels = ['+ (0, 0, 0.1)   static, URDF', '+ (0, 0, 0.426)   CHAMP, constant', 'yaw 7.83°, t = (−0.540, 0.252)   odom_ekf']
    for (y0, y1), lab in zip(zip(ys[:-1], ys[1:]), labels):
        arr(ax, (2.1, y0 - 0.4), (2.1, y1 + 0.5), C['est'])
        ax.text(2.35, (y0 + y1) / 2 + 0.05, lab, fontsize=7.6, color=C['est'], va='center')
    ax.text(0.3, 7.15, 'p_odom = T_odom←base_footprint · T_base_footprint←base_link · T_base_link←lidar · p_lidar', fontsize=7.4,
            family=MONO)
    ax.text(4.3, 0.05, 'tf2 lookup odom←lidar_link: (0.7035, 0.4181, 0.5479)', fontsize=7.6, color=C['good'], family=MONO)
    # top view in odom
    b = fig.add_axes([0.70, 0.13, 0.29, 0.78])
    yaw = math.atan2(0.1362, 0.9907); o = np.array([-0.5398, 0.2516])
    fs.frame_axes(b, (0, 0), length=0.35, names=('x', 'y'), color=C['ink'])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    fs.frame_axes(b, tuple(o), R2=Rz.tolist(), length=0.35, names=('x', 'y'), color=C['est'])
    b.plot([o[0], 0.7036], [o[1], 0.4181], ':', color=C['est'])
    b.plot([0.7036], [0.4181], 'o', color=C['plan'], ms=6)
    b.text(0.72, 0.50, 'point', fontsize=7.5, color=C['plan'])
    b.text(o[0], o[1] - 0.12, 'base_footprint', fontsize=7.2, color=C['est'], va='top', ha='center')
    b.text(0.02, -0.08, 'odom', fontsize=7.2, va='top')
    b.set_xlim(-0.8, 1.0); b.set_ylim(-0.3, 0.9); b.set_aspect('equal'); b.set_xlabel('x in odom [m]'); b.set_ylabel('y [m]')
    b.set_title('top view in odom', fontsize=8.5)
    save(fig, 'compose-chain')


def map_odom_jump():
    """The relocalisation test of chapter 5, drawn in map coordinates over the arena."""
    run = json.loads((ROOT / 'site/data/runs/w2/run.json').read_text())
    states = [('before', (-6.599, -2.287, 167.68), (-2.354, 1.091, 22.53), C['est2']),
              ('+0.5 m', (-6.255, -2.022, 163.76), (-2.422, 1.073, 23.15), C['bad']),
              ('restored', (-6.826, -2.040, 164.09), (-2.442, 1.097, 21.96), C['good'])]

    def compose(a, b):
        c, s = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
        return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], (a[2] + b[2] + 180) % 360 - 180)
    fig, (a, b) = plt.subplots(1, 2, figsize=(9.0, 4.0), gridspec_kw=dict(width_ratios=[1.25, 1]))
    for w in run['arena']['walls']:
        a.add_patch(Rectangle((w[0] - w[2] / 2, w[1] - w[3] / 2), w[2], w[3], fc=C['grid'], ec='none'))
    for bx in run['arena']['boxes']:
        a.add_patch(Rectangle((bx[0] - bx[2] / 2, bx[1] - bx[3] / 2), bx[2], bx[3], fc=C['grid'], ec='none'))
    for cy in run['arena']['cylinders']:
        a.add_patch(Circle((cy[0], cy[1]), cy[2], fc=C['grid'], ec='none'))

    def glyph(ax, x, y, deg, col, L=0.6, lw=1.4):
        ax.annotate('', (x + L * math.cos(math.radians(deg)), y + L * math.sin(math.radians(deg))), (x, y),
                    arrowprops=dict(arrowstyle='-|>', color=col, lw=lw, mutation_scale=9, shrinkA=0, shrinkB=0))
        ax.plot([x], [y], 'o', color=col, ms=3.5)
    glyph(a, 0, 0, 0, C['ink'], L=1.0); a.text(0.1, -0.45, 'map', fontsize=8)
    for name, mo, ob, col in states:
        glyph(a, mo[0], mo[1], mo[2], col)
        r = compose(mo, ob)
        glyph(a, r[0], r[1], r[2], col, L=0.5)
        if name == 'before':
            a.text(mo[0] + 0.25, mo[1] + 0.35, 'odom origin', fontsize=7.2, color=col)
            a.annotate('', (r[0], r[1]), (mo[0], mo[1]), arrowprops=dict(arrowstyle='-|>', color=col, lw=0.7, ls=':',
                                                                         mutation_scale=7))
            a.text(r[0] + 0.25, r[1] - 0.35, f'robot: ({r[0]:.2f}, {r[1]:.2f}), {r[2]:.1f}°', fontsize=7.0, color=col)
    glyph(a, -4.60, -3.70, -173.4, C['gt'], L=0.5, lw=2.2)
    from matplotlib.lines import Line2D
    a.legend([Line2D([], [], color=st[3], lw=2) for st in states] + [Line2D([], [], color=C['gt'], lw=2.2)],
             [f'{st[0]}' for st in states] + ['ground truth'], loc='lower right', fontsize=7.2)
    a.set_aspect('equal'); a.set_xlim(-7.6, 1.6); a.set_ylim(-5.4, 1.2); a.set_xlabel('x in map [m]'); a.set_ylabel('y [m]')
    a.set_title('map→odom (large glyphs) and the robot (small) in map')
    # displacement bars
    def dist(p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1])
    mo = [s[1] for s in states]; ob = [s[2] for s in states]
    vals = [dist(mo[0], mo[1]), dist(mo[1], mo[2]), dist(ob[0], ob[1]), dist(ob[1], ob[2])]
    labs = ['map→odom\n1st step', 'map→odom\n2nd step', 'odom→base_fp\n1st step', 'odom→base_fp\n2nd step']
    cols = [C['est2'], C['est2'], C['est'], C['est']]
    bars = b.bar(range(4), vals, color=cols, width=0.6)
    for rect, v in zip(bars, vals):
        b.text(rect.get_x() + rect.get_width() / 2, v + 0.01, f'{v:.2f} m', ha='center', fontsize=8)
    b.set_xticks(range(4)); b.set_xticklabels(labs, fontsize=7.2); b.set_ylabel('moved in one step [m]')
    b.set_title('Relocalisation moves map→odom, not odom')
    save(fig, 'map-odom-jump')


def ladder(steps, name, size, title, qw=0.36):
    """Troubleshooting ladder: questions down the left, each one's answer (the cause and fix) to its right."""
    fig, ax = plt.subplots(figsize=size); ax.set_axis_off(); n = len(steps)
    ax.set_xlim(-0.005, 1.005); ax.set_ylim(-0.4, n - 0.6); ax.set_title(title, fontsize=9.5)
    for i, (q, branch, ans, col) in enumerate(steps):
        y = n - 1 - i
        ax.add_patch(FancyBboxPatch((0.0, y - 0.32), qw, 0.64, boxstyle='round,pad=0,rounding_size=0.02', fc='white',
                                    ec=C['ink'], lw=0.9))
        ax.text(qw / 2, y, q, ha='center', va='center', fontsize=7.8, color=C['ink'])
        ax.add_patch(FancyBboxPatch((qw + 0.07, y - 0.32), 1.0 - qw - 0.07, 0.64, boxstyle='round,pad=0,rounding_size=0.02',
                                    fc=C['fill'], ec=col, lw=0.9))
        ax.text(qw + 0.085, y, ans, ha='left', va='center', fontsize=7.4, color=C['ink'])
        arr(ax, (qw, y), (qw + 0.07, y), color=col, text=branch, tfs=7.5)
        if i < n - 1:
            arr(ax, (qw / 2, y - 0.32), (qw / 2, y - 0.68), text=None)
            ax.text(qw / 2 + 0.01, y - 0.5, 'no' if branch == 'yes' else 'yes', fontsize=7.5, va='center', color=C['muted'])
    save(fig, name)


def echo_tree():
    """Chapter 4, task 2: why ros2 topic echo prints nothing, from the cases measured in tasks 1 and 2."""
    ladder([
        ('ros2 topic list does not\nshow the topic?', 'yes',
         'Other domain or unsourced terminal: source env.sh sets\nROS_DOMAIN_ID=42; nodes on different domains never meet', C['bad']),
        ('ros2 topic info -v:\nPublisher count 0?', 'yes',
         'Nothing publishes it yet: the launch file that starts\nthe publisher is not up', C['bad']),
        ('Warning "offering incompatible\nQoS ... RELIABILITY"?', 'yes',
         'Request no more than the offer: /scan is BEST_EFFORT,\nso drop --qos-reliability reliable (2.2)', C['bad']),
        ('Published once, before you\nsubscribed (/map, /amcl_pose)?', 'yes',
         'Subscribe TRANSIENT_LOCAL as go_to_goal.py does;\n/amcl_pose also waits for 5 cm or 0.1 rad of motion (2.3)',
         C['plan']),
        ('Your node, use_sim_time:=true,\nand nothing on /clock?', 'yes',
         'Its clock does not advance and its timers never fire;\nstart the simulator or a bag with /clock (2.4)', C['plan']),
    ], 'echo-tree', (8.0, 4.6), 'ros2 topic echo prints nothing: ask in this order')


def tf_lookup_tree():
    """Chapter 5: read an ExtrapolationException; the numbers are the three probe messages in the chapter."""
    ladder([
        ('Requested time a Unix time\n(about 1.8e9 s), data in\nsimulated seconds?', 'yes',
         'The node runs on the wall clock: pass use_sim_time\n(chapter 4, 2.4)', C['bad']),
        ('"into the future"?', 'yes',
         'Tens of ms: the publisher is late, pass a timeout Duration.\nMore: that time has not happened (probe: now + 0.5 s, 0.511 s)',
         C['plan']),
        ('"into the past" by a\nfew ms, at start-up?', 'yes',
         'The listener had just started and lacks the first stamps\n(probe: 4 ms short of the earliest data)', C['plan']),
        ('"into the past" by more\nthan the buffer holds?', 'yes',
         'Older than the history kept (10 s by default)\n(probe: now − 30 s, 29.4 s before the earliest data)', C['bad']),
    ], 'tf-lookup-tree', (8.0, 3.8),
           'ExtrapolationException: read the message (static transforms have no time and never raise it)')


ALL = dict(**{f.__name__: f for f in (stack_launch, ros_graph, mechanisms, qos_matrix, sim_time, overlay, lifecycle, executor,
                                       bag_clock, yaw_drift, tf_tree, amcl_vs_odom, one_parent, optical_frame, tilt_error,
                                       lookup_time, compose_chain, map_odom_jump, echo_tree, tf_lookup_tree)})

if __name__ == '__main__':
    for n in sys.argv[1:] or ALL:
        ALL[n]()
