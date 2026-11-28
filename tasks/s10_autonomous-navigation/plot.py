#!/usr/bin/env python3
"""Stage-10 figures from results/runs/:  python3 plot.py
figures/a_to_b.png       map and true tracks of every run of 10.01-10.07 and 10.15
figures/behaviours.png   10.08 replan, 10.09 reject, 10.10 cancel, 10.12 waypoints, 10.14 go and return"""
import json, math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'worlds')); sys.path.insert(0, str(HERE.parents[1] / 'modules'))
import make_worlds as mw
import score
import figstyle as fs
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

RUNS = HERE / 'results/runs'
TITLES = {'10.01': 'empty room', '10.02': 'one obstacle', '10.03': 'several obstacles', '10.04': 'narrow passage',
          '10.05': 'doorway', '10.06': 'corner', '10.07': 'connected rooms', '10.15': 'modified room, old map'}


def runs(tid):
    out = []
    for j in sorted(RUNS.glob(f'{tid}_[0-9].json')):
        out.append((json.loads(j.read_text()), np.loadtxt(j.with_suffix('.csv'), delimiter=',', skiprows=1, ndmin=2)))
    return out


def draw_map(ax, name, outline=None):
    w = mw.WORLDS[name]
    occ, (ox, oy) = mw.occupancy(w)
    ax.imshow(np.where(occ, 0.55, 1.0), cmap='gray', vmin=0, vmax=1, origin='lower', interpolation='nearest',
              extent=(ox, ox + occ.shape[1] * mw.RES, oy, oy + occ.shape[0] * mw.RES))
    if outline:                                   # the real world, drawn over a stale map
        rects, circles = mw.obstacles(mw.WORLDS[outline])
        for x, y, sx, sy in rects[4:]:
            ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, fill=False, ec=fs.C['bad'], lw=1.2))
        for x, y, r in circles:
            ax.add_patch(plt.Circle((x, y), r, fill=False, ec=fs.C['bad'], lw=1.2))
    ax.plot(*mw.A, 'o', color=fs.C['good'], ms=6, zorder=5)
    ax.plot(*w['goal'][:2], 'o', color=fs.C['bad'], ms=6, zorder=5)
    ax.set_aspect('equal'); ax.grid(False); ax.set_xticks([]); ax.set_yticks([])


def fig_a_to_b():
    fig, axs = plt.subplots(2, 4, figsize=(12, 6.4))
    for ax, tid in zip(axs.flat, TITLES):
        acc = json.loads((HERE / 'acceptance.json').read_text())[tid]
        world = acc.get('map', acc['world'])
        draw_map(ax, world, outline=acc['world'] if 'map' in acc else None)
        rs = runs(tid)
        for r, gt in rs:
            ax.plot(gt[:, 1], gt[:, 2], color=fs.C['est'], lw=1.1, alpha=0.8)
        res = HERE / f'results/{tid}.json'
        m = json.loads(res.read_text())['metrics'] if res.exists() else {}
        n_ok = round(m.get('success_rate', 0) * m.get('n_runs', 0))
        ax.set_title(f'{tid} {TITLES[tid]}', fontsize=9.5)
        if m:
            ax.text(0.02, 0.02, f"{n_ok}/{m['n_runs']} reached, {m['mean_sim_time_s']} s mean", transform=ax.transAxes,
                    fontsize=8, color=fs.C['ink'], bbox=dict(fc='white', ec='none', alpha=0.8, pad=1.5))
    fig.suptitle('True tracks of every run (blue); start A green, goal red. 10.15: map of 10.03, real obstacles outlined red',
                 x=0.01, ha='left', fontsize=10)
    fs.save(fig, HERE / 'figures/a_to_b')


def fig_behaviours():
    fig, axs = plt.subplots(1, 5, figsize=(16, 3.9))
    # 10.08 replan: plan at the spawn, first plan clear of the box, track
    ax = axs[0]; draw_map(ax, 'room')
    for k, (r, gt) in enumerate(runs('10.08')):
        b = r.get('box')
        if not b:
            continue
        c, s = math.cos(b['yaw']), math.sin(b['yaw'])
        corners = [(b['x'] + c * u - s * v, b['y'] + s * u + c * v) for u, v in [(-.15, -.8), (.15, -.8), (.15, .8), (-.15, .8)]]
        ax.add_patch(Polygon(corners, fc=fs.C['plan'], ec='none', alpha=0.5))
        ax.plot(gt[:, 1], gt[:, 2], color=fs.C['est'], lw=1.1)
        pf = RUNS / f'10.08_{k}_plans.csv'
        if k == 0 and pf.exists():
            P = np.loadtxt(pf, delimiter=',', skiprows=1)
            ts = np.unique(P[:, 0])
            t_before = ts[ts <= b['t']].max() if (ts <= b['t']).any() else ts[0]
            ax.plot(*P[P[:, 0] == t_before, 1:].T, '--', color=fs.C['plan'], lw=1.2, label='plan at spawn')
            if b.get('replan_s') is not None:
                t_after = ts[np.argmin(abs(ts - (b['t'] + b['replan_s'])))]
                ax.plot(*P[P[:, 0] == t_after, 1:].T, color=fs.C['plan'], lw=1.4, label='first plan clear of box')
            ax.legend(loc='upper left', fontsize=7, frameon=False)
    ax.set_title('10.08 replan around a spawned box', fontsize=9.5)
    # 10.09 reject
    ax = axs[1]; draw_map(ax, 'one_obstacle')
    for r, gt in runs('10.09'):
        for l in r['legs']:
            g = gt[(gt[:, 0] >= l['t_start']) & (gt[:, 0] <= l['t_end'])]
            ax.plot(g[:, 1], g[:, 2], color=fs.C['est'], lw=1.5)
            gx, gy = l['goal'][:2]
            if abs(gx) < 6:
                ax.plot(gx, gy, 'x', color=fs.C['bad'], ms=9, mew=2)
            ax.annotate(f"{l['status_name']} ({l['status']}) after {l['sim_time_s']:.0f} s", (0.03, 0.9 - 0.08 * r['legs'].index(l)),
                        xycoords='axes fraction', fontsize=7.5)
    ax.set_title('10.09 goal in the box, goal at (8, 8)', fontsize=9.5)
    # 10.10 cancel: true speed against time from each cancel
    ax = axs[2]
    for r, gt in runs('10.10'):
        t, xy = gt[:, 0], gt[:, 1:3]
        v = np.r_[0, np.hypot(*np.diff(xy, axis=0).T) / np.maximum(np.diff(t), 1e-3)]
        v = np.convolve(v, np.ones(25) / 25, mode='same')
        for k, l in enumerate(r['legs']):
            if not l.get('t_cancel'):
                continue
            sel = (t >= l['t_cancel'] - 6) & (t <= l['t_cancel'] + 6)
            ax.plot(t[sel] - l['t_cancel'], v[sel], lw=1.2, label=f"cancel {k + 1}: still after {l.get('stop_s')} s")
    ax.axvline(0, color=fs.C['muted'], lw=0.8)
    ax.set_xlabel('time from cancelTask() (s)'); ax.set_ylabel('true speed (m/s)'); ax.legend(fontsize=7, frameon=False)
    ax.set_title('10.10 cancel a goal', fontsize=9.5)
    # 10.12 waypoints
    ax = axs[3]; draw_map(ax, 'obstacles')
    for r, gt in runs('10.12'):
        ax.plot(gt[:, 1], gt[:, 2], color=fs.C['est'], lw=1.0, alpha=0.8)
        for k, p in enumerate(r['legs'][0]['waypoints']):
            ax.plot(p[0], p[1], 's', color=fs.C['plan'], ms=6, zorder=6)
            ax.annotate(f'W{k + 1}', (p[0] + 0.15, p[1] + 0.15), fontsize=8)
    ax.set_title('10.12 three waypoints', fontsize=9.5)
    # 10.14 go and return
    ax = axs[4]; draw_map(ax, 'obstacles')
    for r, gt in runs('10.14'):
        for l, col in zip(r['legs'], [fs.C['est'], fs.C['est2']]):
            g = gt[(gt[:, 0] >= l['t_start']) & (gt[:, 0] <= l['t_end'])]
            ax.plot(g[:, 1], g[:, 2], color=col, lw=1.0, alpha=0.8)
    ax.set_title('10.14 A to B (blue), back to A (violet)', fontsize=9.5)
    fs.save(fig, HERE / 'figures/behaviours')


if __name__ == '__main__':
    fig_a_to_b()
    fig_behaviours()
    for p in sorted((HERE / 'figures').glob('*.png')):
        print(p.name, p.stat().st_size // 1024, 'KB')
