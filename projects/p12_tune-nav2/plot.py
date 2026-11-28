#!/usr/bin/env python3
"""Results table and figures from scored runs:  python3 projects/p12_tune-nav2/plot.py [--dir ~/p12_runs]
Reads <dir>/<setting>_<k>.json and .npz (written by score.py). Prints one markdown row per setting and writes figures/ (or <dir>
when --dir is given and is not results/)."""
import argparse, collections, json, pathlib
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from tuning import inflation_cost

HERE = pathlib.Path(__file__).resolve().parent
ORDER = ['infl_r045', 'base', 'infl_r150', 'scale_1', 'scale_10', 'polygon', 'smac_mppi']
LABEL = dict(base='r 0.75, scale 4 (stack)', infl_r045='r 0.45, scale 4', infl_r150='r 1.50, scale 4', scale_1='r 0.75, scale 1',
             scale_10='r 0.75, scale 10', polygon='polygon footprint', smac_mppi='Smac Hybrid-A* + MPPI, polygon')
PLAN, TRUE = '#d97706', '#333333'
BODY = np.array([[0.325, 0.15], [0.325, -0.15], [-0.325, -0.15], [-0.325, 0.15]])


def span(v, f='{:.1f}'):
    v = [x for x in v if x is not None]
    return '-' if not v else f.format(v[0]) if len(v) == 1 else f'{f.format(np.mean(v))} ({f.format(min(v))}-{f.format(max(v))})'


def table(runs):
    print('| Setting | Reached B | Nav time, s | True path, m | First plan, m | Min centre clearance, m | Min body clearance, m '
          '| Recoveries | AMCL RMSE, cm |')
    print('|---|---|---|---|---|---|---|---|---|')
    for s in ORDER:
        R = runs.get(s)
        if not R:
            continue
        ok = sum(r['result'] == 'SUCCEEDED' for r in R)
        print(f"| {LABEL[s]} | {ok}/{len(R)} | {span([r['nav_time_s'] for r in R])} | {span([r.get('path_m') for r in R], '{:.2f}')} "
              f"| {span([r['first_plan_m'] for r in R], '{:.2f}')} | {span([r['min_centre_clearance_m'] for r in R], '{:.2f}')} "
              f"| {span([r['min_body_clearance_m'] for r in R], '{:.2f}')} | {sum(r['recoveries'] for r in R)} "
              f"| {span([r.get('amcl_rmse_cm') for r in R])} |")


def panel(ax, s, npz, title=None, box=None):
    d = np.load(npz[s][0]); c, (ox, oy), res = d['costmap'], d['origin'], float(d['res'])
    ext = (ox, ox + c.shape[1] * res, oy, oy + c.shape[0] * res)
    ax.imshow(np.where(c == 255, 0, c), origin='lower', extent=ext, cmap='Greys', vmin=0, vmax=300, interpolation='nearest')
    ax.contour(np.arange(c.shape[1]) * res + ox + res / 2, np.arange(c.shape[0]) * res + oy + res / 2, c >= 253, [0.5],
               colors='#b91c1c', linewidths=0.6)
    for k, f in enumerate(npz[s]):
        g = np.load(f)['gt']; ax.plot(g[:, 0], g[:, 1], color=TRUE, lw=1.0, alpha=0.8, label='walked (3 runs)' if k == 0 else None)
    ax.plot(d['plan'][:, 0], d['plan'][:, 1], color=PLAN, lw=1.6, label='first plan, run 1')
    ax.set_title(title or LABEL[s], fontsize=9); ax.set_aspect('equal'); ax.tick_params(labelsize=7)
    if box:
        ax.set_xlim(box[0], box[1]); ax.set_ylim(box[2], box[3])


def figures(npz, out):
    # 1. sweep: radius in the top row, scale in the bottom row (the stack's setting is in both)
    fig, axs = plt.subplots(2, 3, figsize=(11, 7.2), constrained_layout=True)
    for ax, s in zip(axs.flat, ['infl_r045', 'base', 'infl_r150', 'scale_1', 'base', 'scale_10']):
        if s in npz:
            panel(ax, s, npz, box=(-5.5, 5.5, -4.8, 4.8))
    axs[0, 0].legend(fontsize=7, loc='upper left')
    fig.suptitle('Global costmap (grey = cost, red = 253 contour the centre may not cross) and paths', fontsize=10)
    fig.savefig(out / 'inflation_sweep.png', dpi=110); plt.close(fig)

    # 2. footprint and planners, zoomed on the two narrowest passages: cylinder-pallet and brick box-blue cylinder
    fig, axs = plt.subplots(1, 3, figsize=(11, 4.0), constrained_layout=True)
    for ax, s in zip(axs, ['base', 'polygon', 'smac_mppi']):
        if s not in npz:
            continue
        panel(ax, s, npz, box=(-4.2, 1.8, -2.4, 2.6))
        g = np.load(npz[s][0])['gt']
        for x, y, th in g[::150]:                                         # the body every 3 s of run 1
            R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
            ax.add_patch(Polygon(BODY @ R.T + (x, y), closed=True, fill=False, ec='#2563eb', lw=0.7))
    axs[0].legend(fontsize=7, loc='upper left')
    fig.suptitle('Footprint and planner/controller: body outline (blue) every 3 s of run 1', fontsize=10)
    fig.savefig(out / 'footprint_planners.png', dpi=110); plt.close(fig)


def profile(out):
    d = np.linspace(0, 1.6, 641)
    fig, ax = plt.subplots(figsize=(6.4, 3.4), constrained_layout=True)
    for (r, k, ins), lab, st in [((0.45, 4, 0.33), 'r 0.45, scale 4', ':'), ((0.75, 4, 0.33), 'r 0.75, scale 4 (stack)', '-'),
                                 ((1.5, 4, 0.33), 'r 1.50, scale 4', '--'), ((0.75, 1, 0.33), 'r 0.75, scale 1', '-.'),
                                 ((0.75, 10, 0.33), 'r 0.75, scale 10', (0, (5, 1, 1, 1))),
                                 ((0.75, 4, 0.15), 'polygon (inscribed 0.15), r 0.75, scale 4', (0, (1, 2)))]:
        ax.plot(d, [inflation_cost(x, ins, r, k) for x in d], ls=st, lw=1.4, label=lab)
    ax.axhline(253, color='#b91c1c', lw=0.6); ax.text(1.58, 240, '253: centre in collision', ha='right', fontsize=7, color='#b91c1c')
    ax.set_xlabel('distance from the nearest obstacle cell, m'); ax.set_ylabel('cost'); ax.legend(fontsize=7)
    fig.savefig(out / 'inflation_profile.png', dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--dir', default=str(HERE / 'results')); a = ap.parse_args()
    src = pathlib.Path(a.dir).expanduser()
    runs, npz = collections.defaultdict(list), collections.defaultdict(list)
    for f in sorted(src.glob('*.json')):
        r = json.loads(f.read_text()); runs[r['setting']].append(r)
        if f.with_suffix('.npz').exists():
            npz[r['setting']].append(f.with_suffix('.npz'))
    table(runs)
    out = HERE / 'figures' if src == HERE / 'results' else src
    figures(npz, out); profile(out)
    print(f'figures in {out}')


if __name__ == '__main__':
    main()
