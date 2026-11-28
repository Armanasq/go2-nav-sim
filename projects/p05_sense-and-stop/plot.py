#!/usr/bin/env python3
"""Figures and summary for project 5.
usage: python3 projects/p05_sense-and-stop/plot.py [--dir DIR] [--tag stop]
       python3 projects/p05_sense-and-stop/plot.py --snap SNAP.npz
Reads DIR/<tag>.csv and DIR/runs/<tag>_*.csv from measure_stops.py; prints one summary line per speed; writes DIR/stops.png
(this project's figures/stops.png when DIR is its results/, the default).
With --snap it only draws the snapshot, to scan_and_depth.png next to SNAP.npz (figures/ again for this project's results/)."""
import argparse, collections, csv, math, pathlib, statistics as st, sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parents[1] / 'src/go2_nav_sim/scripts'))
import make_arena as arena
from obstacle import FRONT, front_min

GAP = 0.5
SPEED_COLOUR = {0.1: '#9ecae1', 0.2: '#4292c6', 0.3: '#08306b'}    # light to dark = slow to fast


def draw_arena(ax):
    for x, y, sx, sy in arena.WALLS:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='0.35'))
    for x, y, sx, sy, *_ in arena.BOXES:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='0.75'))
    for x, y, r, *_ in arena.CYLS:
        ax.add_patch(Circle((x, y), r, color='0.75'))


def load_run(p):
    out = collections.defaultdict(list)
    for r in csv.DictReader(open(p)):
        out[r['source']].append(tuple(float(v) if v else math.nan for v in (r['t'], r['a'], r['b'], r['c'])))
    return out


def static_bias(run):
    """Sensed gap minus true gap over the last 1 s of the run, when the robot stands still."""
    t_end, gap_true = run['gt'][-1][0], run['gt'][-1][3]
    s = [r - FRONT - gap_true for t, _, r, _ in run['range'] if t > t_end - 1.0 and math.isfinite(r)]
    return st.mean(s) if s else math.nan


def stops_figure(rows, runs, out):
    fig, (a1, a2, a3) = plt.subplots(1, 3, figsize=(15, 4.6), gridspec_kw=dict(width_ratios=[1.2, 1.3, 1]))
    draw_arena(a1)
    for (row, run) in zip(rows, runs):
        c = SPEED_COLOUR.get(float(row['speed_cmd']), 'k')
        xs, ys = [g[1] for g in run['gt']], [g[2] for g in run['gt']]
        a1.plot(xs, ys, color='0.6', lw=1)
        a1.plot(xs[-1] + FRONT, ys[-1], 'o', color=c, ms=4)            # where the front of the head ended
    a1.axvline(arena.BOXES[2][0] - arena.BOXES[2][2] / 2 - GAP, color='#d97706', lw=1, ls='--')
    a1.set_xlim(-0.8, 3.3); a1.set_ylim(-3.4, -1.6); a1.set_aspect('equal')
    a1.set_title('true paths (grey), front of head at the end (dots)\ndashed: 0.5 m before the pallet face')
    a1.set_xlabel('x, world (m)'); a1.set_ylabel('y (m)')

    for v in sorted({float(r['speed_cmd']) for r in rows}):
        row, run = next((r, u) for r, u in zip(rows, runs) if float(r['speed_cmd']) == v)
        t0 = next(t for t, vx, _, _ in run['cmd'] if vx == 0.0 and t > run['cmd'][0][0] + 0.5)   # the stop command
        c = SPEED_COLOUR[v]
        a2.plot([g[0] - t0 for g in run['gt']], [g[3] for g in run['gt']], color=c, lw=2, label=f'{v} m/s true gap')
        a2.plot([t - t0 for t, _, r, _ in run['range']], [r - FRONT for t, _, r, _ in run['range']], '.', color=c, ms=3)
    a2.axhline(GAP, color='#d97706', lw=1, ls='--'); a2.axvline(0, color='k', lw=0.6)
    a2.set_xlim(-2.5, 2.0); a2.set_ylim(0.35, 1.2)
    a2.set_xlabel('time from the stop command (s, simulated)'); a2.set_ylabel('gap, front of head to pallet (m)')
    a2.set_title('one run per speed: true gap (line), /front_range - 0.28 m (dots)'); a2.legend(fontsize=8)

    for i, row in enumerate(rows):
        v = float(row['speed_cmd'])
        a3.plot(v + 0.004 * (i % 5 - 2), float(row['gap_final']), 'o', color=SPEED_COLOUR.get(v, 'k'))   # spread runs sideways
    a3.axhline(GAP, color='#d97706', lw=1, ls='--')
    a3.set_xlabel('commanded speed (m/s)'); a3.set_ylabel('true final gap (m)'); a3.set_xticks([0.1, 0.2, 0.3])
    a3.set_title('where each run stopped')
    fig.tight_layout(); fig.savefig(out, dpi=110); print(f'wrote {out}')


def snapshot_figure(snap, out):
    z = np.load(snap)
    r, a0, inc = z['ranges'], float(z['angle_min']), float(z['angle_inc'])
    a = a0 + inc * np.arange(len(r))
    half = math.radians(20)
    d, b = front_min(r, a0, inc, half, float(z['range_min']), float(z['range_max']))
    fig = plt.figure(figsize=(13, 4.8))
    ax = fig.add_subplot(1, 2, 1, projection='polar')
    ax.set_theta_zero_location('N')                                    # straight ahead up, left to the left
    ok = np.isfinite(r)
    ax.plot(a[ok], r[ok], '.', ms=1.5, color='#2563eb')
    ax.fill_between(np.linspace(-half, half, 30), 0, 15, color='#d97706', alpha=0.15)
    ax.plot([b], [d], 'o', color='#d97706')
    ax.set_rmax(8); ax.set_title(f'/scan (base_footprint, bearing + = left), nearest in +-20 deg: {d:.2f} m at {math.degrees(b):+.1f} deg', fontsize=9)
    dep = z['depth']
    ax2 = fig.add_subplot(1, 2, 2)
    im = ax2.imshow(dep, cmap='viridis', vmin=0.3, vmax=8.0)
    h, w = dep.shape
    ax2.add_patch(Rectangle((w // 2 - 2.5, h // 2 - 2.5), 5, 5, fill=False, ec='r'))
    c = np.nanmedian(dep[h // 2 - 2:h // 2 + 3, w // 2 - 2:w // 2 + 3])
    ax2.set_title(f'/camera/depth/image_raw (m, white = no return); centre 5x5 median {c:.2f} m', fontsize=9)
    fig.colorbar(im, ax=ax2, fraction=0.03)
    fig.tight_layout(); fig.savefig(out, dpi=110); print(f'wrote {out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default=str(HERE / 'results')); ap.add_argument('--tag', default='stop'); ap.add_argument('--snap')
    a = ap.parse_args()
    figdir = lambda d: HERE / 'figures' if d.resolve() == (HERE / 'results').resolve() else d   # mine go to figures/, yours stay put
    if a.snap:
        snapshot_figure(a.snap, figdir(pathlib.Path(a.snap).parent) / 'scan_and_depth.png')
        return
    d = pathlib.Path(a.dir)
    rows = [r for r in csv.DictReader(open(d / f'{a.tag}.csv')) if r['tag'] == a.tag]
    runs = [load_run(d / 'runs' / f"{a.tag}_{r['run']}.csv") for r in rows]
    print('speed  true v  n  final gap (m)  error mean  sensed-0.5  trigger sensed-true  coast   to still (s)  standing bias  scan age (s)')
    for v in sorted({float(r['speed_cmd']) for r in rows}):
        sel = [(r, u) for r, u in zip(rows, runs) if float(r['speed_cmd']) == v]
        f = lambda k: [float(r[k]) for r, _ in sel]
        g = f('gap_final')
        trig = [s - t for s, t in zip(f('gap_sensed'), f('gap_true_at_trigger'))]
        print(f'{v:.1f}    {st.mean(f("v_true")):.3f}  {len(sel)}  {min(g):.3f}..{max(g):.3f}  {st.mean(g) - GAP:+.3f}      '
              f'{st.mean(f("gap_sensed")) - GAP:+.3f}      {st.mean(trig):+.3f}               {st.mean(f("coast")):.3f}   '
              f'{st.mean(f("t_to_still")):.2f}          {st.mean(static_bias(u) for _, u in sel):+.3f}         {st.mean(f("scan_age")):.3f}')
    stops_figure(rows, runs, figdir(d) / (f'{a.tag}s.png' if a.tag != 'stop' else 'stops.png'))


if __name__ == '__main__':
    main()
