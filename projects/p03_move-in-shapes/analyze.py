#!/usr/bin/env python3
"""Score the logged runs against ground truth and draw them.
usage: python3 projects/p03_move-in-shapes/analyze.py      (reads results/*.csv, writes figures/*.png, prints the tables)"""
import csv, math, pathlib, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import shapes

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src/go2_nav_sim/scripts'))
import make_arena as arena  # noqa: E402  (obstacle list and marker A)


def load(name):
    with open(HERE / 'results' / f'{name}.csv') as f:
        rows = list(csv.DictReader(f))
    num = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != 'label'}
    num['label'] = [r['label'] for r in rows]
    for k in ('x', 'gt_x'):                    # odom and odom_gt start at marker A: shift both into the world frame
        num[k] = num[k] + arena.A[0]
    for k in ('y', 'gt_y'):
        num[k] = num[k] + arena.A[1]
    return num


def to_world(pts, x0, y0, a0):
    """Start-frame points -> world, given the start pose."""
    return [(x0 + px * math.cos(a0) - py * math.sin(a0), y0 + px * math.sin(a0) + py * math.cos(a0)) for px, py in pts]


def last_of(run, label):
    """Index of the last row with this label (the end of a settle, a lobe or a leg)."""
    return max(i for i, l in enumerate(run['label']) if l == label)


def score(name, shape):
    r = load(name)
    x0, y0, a0 = r['gt_x'][0], r['gt_y'][0], r['gt_yaw'][0]
    if shape == 'square':
        ideal = to_world(shapes.square_corners(), x0, y0, a0)
        idx = [last_of(r, f'corner{k}') for k in range(4)]
    else:                                      # the crossing after the first lobe, then the end
        mid = 'lobe0' if 'lobe0' in r['label'] else 'walk11'
        ideal, idx = [(x0, y0), (x0, y0)], [last_of(r, mid), len(r['t']) - 1]
    err = [math.hypot(r['gt_x'][i] - cx, r['gt_y'][i] - cy) for i, (cx, cy) in zip(idx, ideal)]
    # the same corners as /odom saw them, against the shape placed at /odom's start pose (what closed loop steers on)
    ox, oy, oa = r['x'][0], r['y'][0], r['yaw'][0]
    pts = shapes.square_corners() if shape == 'square' else [(0.0, 0.0), (0.0, 0.0)]
    err_odom = [math.hypot(r['x'][i] - cx, r['y'][i] - cy) for i, (cx, cy) in zip(idx, to_world(pts, ox, oy, oa))]
    e = len(r['t']) - 1
    final = math.hypot(r['gt_x'][e] - x0, r['gt_y'][e] - y0)
    head = math.degrees(shapes.wrap(r['gt_yaw'][e] - a0))
    # how far /odom's displacement since the start is from the true one (the robot lands a few cm off, so compare changes)
    odom_err = math.hypot((r['x'][e] - r['x'][0]) - (r['gt_x'][e] - x0), (r['y'][e] - r['y'][0]) - (r['gt_y'][e] - y0))
    # the odom frame and the world frame can start rotated by the drift accumulated before the run: compare changes
    turn_odom = math.degrees(shapes.wrap(r['yaw'][e] - r['yaw'][0]))     # the end heading as the controller saw it
    dyaw_odom = math.degrees(shapes.wrap((r['yaw'][e] - r['yaw'][0]) - (r['gt_yaw'][e] - r['gt_yaw'][0])))
    if shape != 'square':                      # the end point is already the final position error
        err, err_odom = err[:1], err_odom[:1]
    return r, ideal, idx, err, err_odom, final, head, turn_odom, odom_err, dyaw_odom


HEADER = ('| Run | Corner (square) or crossing (figure-eight) errors, m | The same on /odom, m | Final position error, m | '
          'Final heading error, deg | /odom end heading, deg | /odom displacement error at end, m | /odom yaw drift in run, deg | '
          'Time, s |\n|---|---|---|---|---|---|---|---|---|')


def print_row(name, shape):
    """One table row; returns the score as well for the plots."""
    sc = score(name, shape)
    r, ideal, idx, err, err_odom, final, head, turn_odom, odom_err, dyaw = sc
    print(f'| {name} | {" ".join(f"{e:.2f}" for e in err)} | {" ".join(f"{e:.2f}" for e in err_odom)} | {final:.2f} | '
          f'{head:.1f} | {turn_odom:.1f} | {odom_err:.2f} | {dyaw:.1f} | {r["t"][-1]:.1f} |')
    return sc


def achieved(r):
    """Open loop: commanded vs achieved, per segment of the schedule (ground truth)."""
    out, labels = [], r['label']
    starts = [i for i in range(len(labels)) if i == 0 or labels[i] != labels[i - 1]]
    for a, b in zip(starts, starts[1:] + [len(labels)]):
        b -= 1
        if b <= a:
            continue
        dt = r['t'][b] - r['t'][a]
        k = list(range(a, b, 10)) + [b]                           # path length on 0.5 s steps: a lobe ends where it began
        xs, ys = r['gt_x'][k], r['gt_y'][k]
        dist = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
        dyaw = float(np.sum(np.diff(np.unwrap(r['gt_yaw'][a:b + 1]))))
        # steady state: leave out the first 1.5 s (the gait speeding up) and the last 0.5 s
        c, d = (a + 30, b - 10) if dt > 3 else (a, b)
        k = list(range(c, d, 10)) + [d]
        v_ss = float(np.sum(np.hypot(np.diff(r['gt_x'][k]), np.diff(r['gt_y'][k])))) / (r['t'][d] - r['t'][c])
        w_ss = float(np.sum(np.diff(np.unwrap(r['gt_yaw'][c:d + 1])))) / (r['t'][d] - r['t'][c])
        out.append((labels[a], dt, r['v_cmd'][a], dist, r['w_cmd'][a], dyaw, v_ss, w_ss))
    return out


def draw_arena(ax):
    for x, y, sx, sy, *_ in arena.BOXES + arena.WALLS:
        ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='0.8', zorder=0))
    for x, y, rad, *_ in arena.CYLS:
        ax.add_patch(plt.Circle((x, y), rad, color='0.8', zorder=0))
    ax.add_patch(plt.Circle(arena.A, 0.4, color='#cde8cd', zorder=0))
    ax.set_aspect('equal'); ax.set_xlim(-6.1, -1.3); ax.set_ylim(-5.1, -0.9); ax.grid(alpha=0.3)


def main():
    (HERE / 'figures').mkdir(exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(10, 9.4))
    print(HEADER)
    for row, shape in enumerate(('square', 'figure8')):
        for col, mode in enumerate(('open', 'closed')):
            name = f'{shape}_{mode}'
            r, ideal, idx, err, err_odom, final, *_ = print_row(name, shape)
            ax = axes[row][col]
            draw_arena(ax)
            x0, y0, a0 = r['gt_x'][0], r['gt_y'][0], r['gt_yaw'][0]
            dense = (shapes.square_corners() if shape == 'square' else shapes.figure8_points(step_deg=5))
            path = to_world([(0.0, 0.0)] + dense, x0, y0, a0)
            ax.plot(*zip(*path), 'k--', lw=1, label='intended')
            ax.plot(r['gt_x'], r['gt_y'], color='0.35', lw=2, label='ground truth')
            ax.plot(r['x'], r['y'], color='tab:blue', lw=1, label='/odom')
            ax.plot([r['gt_x'][i] for i in idx], [r['gt_y'][i] for i in idx], 'o', color='0.35', ms=5)
            ax.plot(*zip(*ideal), 'kx', ms=7)
            ax.set_title(f'{shape}, {mode} loop: final error {final:.2f} m')
            ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout(); fig.savefig(HERE / 'figures/shapes.png', dpi=110)
    for name in ('square_open_calibrated', 'square_closed_on_odom_gt'):       # the "Try this" runs, if you made them
        if (HERE / 'results' / f'{name}.csv').exists():
            print_row(name, 'square')
    repeats = sorted((HERE / 'results').glob('repeats/*.csv'))
    if repeats:                                # more runs of the same shapes, to see how much single runs vary
        print('\nRepeat runs (results/repeats/)')
        print(HEADER)
        for f in repeats:
            print_row(f'repeats/{f.stem}', f.stem.split('_')[0])

    print('\nOpen loop, commanded vs achieved (ground truth)')
    print('| Run | Segment | Duration, s | v cmd, m/s | v mean, m/s | v steady, m/s | distance, m | w cmd, rad/s | w mean, rad/s | w steady, rad/s | turned, deg |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for shape in ('square', 'figure8'):
        r = load(f'{shape}_open')
        for label, dt, vc, dist, wc, dyaw, v_ss, w_ss in achieved(r):
            if label == 'stop':
                continue
            print(f'| {shape} | {label} | {dt:.2f} | {vc:.2f} | {dist / dt:.3f} | {v_ss:.3f} | {dist:.3f} | {wc:.2f} | '
                  f'{dyaw / dt:.3f} | {w_ss:.3f} | {math.degrees(dyaw):.1f} |')

    # commanded vs achieved speed over time, open-loop square (0.5 s windows of ground truth)
    r = load('square_open')
    t, n = r['t'], 10
    v = np.hypot(r['gt_x'][n:] - r['gt_x'][:-n], r['gt_y'][n:] - r['gt_y'][:-n]) / (t[n:] - t[:-n])
    w = np.diff(np.unwrap(r['gt_yaw']))
    w = np.convolve(w, np.ones(n), 'valid') / (t[n:] - t[:-n])
    tc = (t[n:] + t[:-n]) / 2
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    a1.plot(t, r['v_cmd'], 'k--', lw=1, label='commanded'); a1.plot(tc, v, color='0.35', label='achieved (ground truth, 0.5 s window)')
    a1.set_ylabel('trunk speed, m/s'); a1.legend(fontsize=8); a1.grid(alpha=0.3)
    a2.plot(t, r['w_cmd'], 'k--', lw=1); a2.plot(tc, w, color='0.35')
    a2.set_ylabel('yaw rate, rad/s'); a2.set_xlabel('time since start, s (simulated)'); a2.grid(alpha=0.3)
    a1.set_title('Open-loop square: commanded vs achieved')
    fig.tight_layout(); fig.savefig(HERE / 'figures/commanded_vs_achieved.png', dpi=110)
    print('\nwrote figures/shapes.png figures/commanded_vs_achieved.png')


if __name__ == '__main__':
    main()
