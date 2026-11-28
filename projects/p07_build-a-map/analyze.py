#!/usr/bin/env python3
"""Score a mapping run: the saved map against the true arena map, and the robot's path on /odom and on slam_toolbox's pose
against ground truth. Draws figures/NAME.png. No ROS needed.
usage: python3 projects/p07_build-a-map/analyze.py [--name NAME]      (reads results/NAME.csv and maps/NAME.yaml)
The SLAM map frame is the odom frame at the moment slam_toolbox started. The first row of the log gives where that frame
sits in the world (true pose and /odom at the same tick): 'placed' scores the map there, 'fitted' after a rigid point-to-line
fit of its occupied cells onto the true outline. The difference between the two is how far the map drifted from odom."""
import argparse, csv, math, pathlib, sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / 'p06_avoid-obstacles')]
import arena2d  # noqa: E402
import mapcmp  # noqa: E402
import reactive  # noqa: E402
import route  # noqa: E402

TRUTH = HERE.parents[1] / 'src/go2_nav_sim/maps/arena.yaml'


def track(rows, keys, anchor=None):
    p = [tuple(float(r[k]) for k in keys) for r in rows]
    return np.array([reactive.compose(anchor, q) if anchor else q for q in p])


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('--name', default='my_map')
    a = ap.parse_args()
    rows = list(csv.DictReader(open(HERE / 'results' / f'{a.name}.csv')))
    gt = track(rows, ('gt_x', 'gt_y', 'gt_yaw'))
    odom0 = track(rows[:1], ('x', 'y', 'yaw'))[0]
    anchor = reactive.compose(tuple(gt[0]), reactive.inverse(tuple(odom0)))      # odom (= SLAM map) frame in the world
    odom = track(rows, ('x', 'y', 'yaw'), anchor)
    ok = [i for i, r in enumerate(rows) if r['slam_x'] != 'nan']
    slam_t = track([rows[i] for i in ok], ('slam_x', 'slam_y', 'slam_yaw'), anchor)

    truth, slam = mapcmp.load(TRUTH), mapcmp.load(HERE / 'maps' / f'{a.name}.yaml')
    outline, _ = mapcmp.visible_outline(truth, route.A)
    fit, matches, rms = mapcmp.align(slam, mapcmp.centres(truth, outline), guess=anchor)
    on_truth = {'placed': mapcmp.resample(slam, anchor, truth), 'fitted': mapcmp.resample(slam, fit, truth)}

    walked = np.sum(np.hypot(*np.diff(gt[:, :2], axis=0).T))
    print(f'run {a.name}: {float(rows[-1]["t"]):.0f} s simulated, {walked:.1f} m walked (true path), '
          f'legs ended in timeout: {len({r["leg"] for r in rows if r["phase"] == "timeout"})}')
    for name, est, idx in (('odom', odom, range(len(rows))), ('slam', slam_t, ok)):
        e = np.hypot(*(est[:, :2] - gt[list(idx), :2]).T)
        print(f'  {name:4s} position error vs ground truth: rms {np.sqrt(np.mean(e**2)):.3f} m, '
              f'max {e.max():.3f} m, at the end {e[-1]:.3f} m')
    d = np.subtract(fit[:2], anchor[:2])
    print(f'  fit moves the placed map by {np.hypot(*d):.3f} m and {math.degrees(reactive.wrap(fit[2] - anchor[2])):+.2f} deg '
          f'({matches} cells matched, rms {rms * 100:.1f} cm)')
    print(f'  {"":7s}' + ''.join(f'{k:>16s}' for k in mapcmp.scores(on_truth['fitted'], truth, route.A)))
    for k, g in on_truth.items():
        print(f'  {k:7s}' + ''.join(f'{v:16.3f}' for v in mapcmp.scores(g, truth, route.A).values()))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.4))
    for ax in (ax1, ax2):
        for cx, cy, sx, sy in arena2d.RECTS:
            ax.add_patch(plt.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, fc='none' if ax is ax2 else '0.85', ec='0.45', lw=0.8))
        for cx, cy, r in arena2d.CIRCLES:
            ax.add_patch(plt.Circle((cx, cy), r, fc='none' if ax is ax2 else '0.85', ec='0.45', lw=0.8))
        ax.set_aspect('equal'); ax.set_xlim(-6.3, 6.3); ax.set_ylim(-5.3, 5.3); ax.set_xlabel('x (m)')
    ax1.plot(*gt[:, :2].T, color='0.3', lw=2.2, label='ground truth')
    ax1.plot(*odom[:, :2].T, color='tab:blue', lw=1, label='/odom')
    ax1.plot(*slam_t[:, :2].T, color='tab:purple', lw=1, ls='--', label='slam_toolbox (map -> base_footprint)')
    ax1.plot(*np.array(route.WAYPOINTS).T, 'k+', ms=8, label='waypoints')
    ax1.plot(*route.A, 'o', color='tab:green', ms=7, label='A (start and end)')
    ax1.set_ylabel('y (m)'); ax1.legend(loc='lower right', fontsize=8); ax1.set_title(f'Route, {walked:.1f} m')
    g = on_truth['fitted']
    occ, near = g == mapcmp.OCC, mapcmp.ndimage.binary_dilation(outline, np.ones((3, 3), bool))
    img = np.full(g.shape + (3,), 0.8)
    img[g == mapcmp.FREE] = 1.0
    img[occ & near] = (0.0, 0.0, 0.0)
    img[occ & ~near] = (0.85, 0.1, 0.1)
    ext = [truth[2][0], truth[2][0] + g.shape[1] * truth[1], truth[2][1], truth[2][1] + g.shape[0] * truth[1]]
    ax2.imshow(img, origin='lower', extent=ext, interpolation='nearest')
    ax2.set_title('Saved map after the fit: black within 5 cm of a true outline (grey), red further')
    ax2.title.set_fontsize(9)
    fig.tight_layout()
    (HERE / 'figures').mkdir(exist_ok=True)
    fig.savefig(HERE / 'figures' / f'{a.name}.png', dpi=110)
    print(f'figure: {HERE / "figures" / (a.name + ".png")}')


if __name__ == '__main__':
    main()
