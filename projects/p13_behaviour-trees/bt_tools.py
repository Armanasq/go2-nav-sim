#!/usr/bin/env python3
"""Read a Nav2 behaviour-tree XML without ROS: draw it, list its recoveries and waypoints, and summarise a bt_log.py log.
usage:  python3 bt_tools.py draw trees/default.xml figures/default_tree.png
        python3 bt_tools.py recoveries trees/default.xml
        python3 bt_tools.py waypoints trees/patrol.xml
        python3 bt_tools.py summary results/default_run.log"""
import math, re, subprocess, sys
import xml.etree.ElementTree as ET

CONTROL = {'Sequence', 'Fallback', 'ReactiveFallback', 'ReactiveSequence', 'PipelineSequence', 'RoundRobin', 'RecoveryNode',
           'RateController', 'Repeat', 'Inverter', 'KeepRunningUntilFailure', 'DistanceController', 'SpeedController'}
CONDITION = {'GoalUpdated', 'GoalReached', 'IsStuck', 'IsPathValid', 'InitialPoseReceived', 'GlobalUpdatedGoal'}
SHOW = ('number_of_retries', 'hz', 'num_cycles', 'spin_dist', 'wait_duration', 'backup_dist', 'goal')
RECOVERIES = ('Spin', 'Wait', 'BackUp', 'ClearingActions')


def main_tree(path):
    root = ET.parse(path).getroot()
    main = root.get('main_tree_to_execute')
    return next(bt for bt in root.iter('BehaviorTree') if bt.get('ID') == main)[0]


def label(el):
    name = el.get('name')
    text = el.tag if not name or name == el.tag else f'{el.tag}\n"{name}"'
    for k in SHOW:
        if k in el.attrib:
            v = el.get(k)
            if k == 'goal' and v.count(';') == 8:                        # literal PoseStamped: show x, y only
                v = '({}, {})'.format(*v.split(';')[2:4])
            text += f'\n{k}={v}'
    return text


def to_dot(path):
    lines, n = ['digraph bt {', '  rankdir=TB; nodesep=0.15; ranksep=0.3;',
                '  node [fontname="Helvetica", fontsize=10, style=filled, fillcolor=white, color="#555555"];'], [0]

    def walk(el, parent):
        i = n[0]; n[0] += 1
        shape = 'box' if el.tag in CONTROL else 'hexagon' if el.tag in CONDITION else 'ellipse'
        fill = '#fde9c9' if el.get('name') in RECOVERIES or el.tag in RECOVERIES else 'white'   # the recovery actions
        lbl = label(el).replace('"', '\\"').replace('\n', '\\n')
        lines.append(f'  n{i} [label="{lbl}", shape={shape}, fillcolor="{fill}"];')
        if parent is not None:
            lines.append(f'  n{parent} -> n{i};')
        for child in el:
            walk(child, i)
    walk(main_tree(path), None)
    return '\n'.join(lines + ['}']), n[0]


def recovery_order(path):
    """Children of the RoundRobin, in the order the tree tries them."""
    rr = next(main_tree(path).iter('RoundRobin'))
    return [c.get('name') or c.tag for c in rr]


def waypoints(path):
    """(x, y, yaw) of every literal goal="stamp;frame;x;y;z;qx;qy;qz;qw" in the tree."""
    out = []
    for el in main_tree(path).iter('ComputePathToPose'):
        f = el.get('goal').split(';')
        if len(f) == 9:
            x, y, qz, qw = float(f[2]), float(f[3]), float(f[7]), float(f[8])
            out.append((x, y, 2 * math.atan2(qz, qw)))
    return out


LINE = re.compile(r'^\s*([\d.]+)\s+(\S+)\s+(\w+)\s*->\s*(\w+)')


def parse_log(lines):
    """bt_log.py lines '  t  node  PREV -> CURR' -> list of (t, node, prev, curr)."""
    return [(float(m[1]), m[2], m[3], m[4]) for m in map(LINE.match, lines) if m]


def started(events, names=('Spin', 'Wait', 'BackUp', 'ClearLocalCostmap-Subtree')):
    """(t, node) each time one of the recovery leaves starts RUNNING (or finishes at once, as a service call can)."""
    return [(t, n) for t, n, p, c in events if n in names and p == 'IDLE' and c != 'IDLE']


def intervals(events, names=('Spin', 'Wait', 'BackUp')):
    """(node, t_start, t_end) for each run of a recovery action: from leaving IDLE to SUCCESS or FAILURE."""
    out, open_ = [], {}
    for t, n, p, c in events:
        if n not in names:
            continue
        if p == 'IDLE' and c == 'RUNNING':
            open_[n] = t
        elif c in ('SUCCESS', 'FAILURE') and n in open_:
            out.append((n, open_.pop(n), t))
    return out


def main(argv):
    cmd, path = argv[1], argv[2]
    if cmd == 'draw':
        dot, count = to_dot(path)
        subprocess.run(['dot', '-Tpng', '-Gdpi=110', '-o', argv[3]], input=dot.encode(), check=True)
        print(f'{argv[3]}: {count} nodes')
    elif cmd == 'recoveries':
        print(' -> '.join(recovery_order(path)))
    elif cmd == 'waypoints':
        for x, y, yaw in waypoints(path):
            print(f'({x:5.2f}, {y:5.2f})  yaw {math.degrees(yaw):7.1f} deg')
    elif cmd == 'summary':
        for t, node in started(parse_log(open(path))):
            print(f'{t:7.1f} s  {node}')


if __name__ == '__main__':
    main(sys.argv)
