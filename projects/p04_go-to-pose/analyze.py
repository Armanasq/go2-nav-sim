#!/usr/bin/env python3
"""Score the logged runs per goal and draw them.
usage: python3 projects/p04_go-to-pose/analyze.py [--mine] [RUN ...]
  default: my logs, every results/*.csv except my_*.csv; writes figures/trajectories.png and figures/gains.png.
  --mine: your logs, results/my_*.csv; writes figures/my_trajectories.png and figures/my_gains.png from my_run,
  my_high_gains and my_k_alpha_20, skipping a figure whose logs you have not recorded yet.
Errors are taken at the last row of each goal, which is when go_to_pose.py printed them (1.5 s after 'done')."""
import csv, math, pathlib, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src/go2_nav_sim/scripts'))
import make_arena as arena  # noqa: E402  (obstacle list and marker A)

GREY, BLUE, RED = '0.35', 'tab:blue', 'tab:red'


def load(name):
    with open(HERE / 'results' / f'{name}.csv') as f:
        rows = list(csv.DictReader(f))
    r = {k: np.array([float(row[k]) for row in rows]) for k in rows[0] if k != 'phase'}
    r['phase'] = np.array([row['phase'] for row in rows])
    return r


def wrap_deg(a):
    return np.degrees(np.arctan2(np.sin(a), np.cos(a)))


def goals(r):
    """Per goal: (goal, time to done or timeout s, outcome, /odom error when the drive phase ended m,
    /odom error m, deg, true error m, deg, true path m, turn reversals)."""
    out = []
    for g in np.unique(r['goal']).astype(int):
        i = np.flatnonzero(r['goal'] == g)
        end = i[np.isin(r['phase'][i], ['done', 'timeout', 'abort'])]
        j = i[r['phase'][i] != 'drive'][0]                      # first row after the drive phase
        k = i[-1]                                               # the row the node printed its errors from
        e = lambda n, p='': math.hypot(r[p + 'x'][n] - r[p + 'goal_x'][n], r[p + 'y'][n] - r[p + 'goal_y'][n])
        path = float(np.sum(np.hypot(np.diff(r['gt_x'][i]), np.diff(r['gt_y'][i]))))
        w = r['w_cmd'][i][np.abs(r['w_cmd'][i]) > 0.25]       # ignore small steering noise
        out.append((g, r['t'][end[0]] - r['t'][i[0]], r['phase'][k], e(j), e(k), wrap_deg(r['yaw'][k] - r['goal_yaw'][k]),
                    e(k, 'gt_'), wrap_deg(r['gt_yaw'][k] - r['gt_goal_yaw'][k]), path, int(np.sum(np.diff(np.sign(w)) != 0))))
    return out


def table(name):
    rows = goals(load(name))
    print(f'\n{name}\n| Goal | Time, s | Outcome | /odom error at end of drive, m | /odom error, m | deg '
          '| True error, m | deg | True path, m | Turn reversals |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for g, t, ph, ed, eo, ho, eg, hg, p, n in rows:
        print(f'| {g} | {t:.1f} | {ph} | {ed:.3f} | {eo:.3f} | {ho:+.1f} | {eg:.3f} | {hg:+.1f} | {p:.2f} | {n} |')
    t, ed, eo, ho, eg, hg, p, n = (np.array(c) for i, c in enumerate(zip(*rows)) if i not in (0, 2))
    print(f'| all | {t.sum():.1f} | | mean {ed.mean():.3f} | mean {eo.mean():.3f} | mean abs {np.abs(ho).mean():.1f} '
          f'| mean {eg.mean():.3f} | mean abs {np.abs(hg).mean():.1f} | {p.sum():.2f} | {n.sum()} |')


def sway(name, t0=1.0, t1=5.0):
    """Heading sway of the trunk while walking straight to goal 1: what k_alpha multiplies even when the robot is on course."""
    r = load(name)
    m = (r['goal'] == 1) & (r['t'] > t0) & (r['t'] < t1)
    for k, label in (('yaw', '/odom'), ('gt_yaw', 'true')):
        y = np.degrees(np.unwrap(r[k][m]))
        y -= np.polyval(np.polyfit(r['t'][m], y, 1), r['t'][m])          # remove the slow heading change
        print(f'{name}: {label} heading sway walking straight ({t0:.0f}-{t1:.0f} s): '
              f'peak to peak {np.ptp(y):.1f} deg, std {np.std(y):.1f} deg')


def near(name, goal=2, radius=0.3):
    """How the drive phase spent its time once /odom put the robot within radius of the goal."""
    r = load(name)
    i = np.flatnonzero((r['goal'] == goal) & (r['phase'] == 'drive'))
    m = np.hypot(r['goal_x'][i] - r['x'][i], r['goal_y'][i] - r['y'][i]) < radius
    t = r['t'][i] - r['t'][i[0]]
    print(f'{name}, goal {goal}: within {radius} m after {t[np.argmax(m)]:.1f} s, then {np.sum(np.diff(t)[m[1:]]):.1f} s of drive '
          f'with median v {np.median(r["v_cmd"][i][m]):.2f} m/s and |w| at the 0.5 rad/s limit '
          f'{100 * np.mean(np.abs(r["w_cmd"][i][m]) >= 0.49):.0f}% of the time')


def to_world(r):
    """Ground truth + A is the world frame. /odom is turned and shifted so its start pose sits on the true start pose."""
    gx, gy = r['gt_x'] + arena.A[0], r['gt_y'] + arena.A[1]
    da = r['gt_yaw'][0] - r['yaw'][0]
    c, s = math.cos(da), math.sin(da)
    dx, dy = r['x'] - r['x'][0], r['y'] - r['y'][0]
    return gx, gy, gx[0] + c * dx - s * dy, gy[0] + s * dx + c * dy


def draw_arena(ax, xlim=(-6.1, 0.6), ylim=(-5.1, -0.9)):
    for x, y, sx, sy, *_ in arena.BOXES + arena.WALLS:
        ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='0.8', zorder=0))
    for x, y, rad, *_ in arena.CYLS:
        ax.add_patch(plt.Circle((x, y), rad, color='0.8', zorder=0))
    ax.add_patch(plt.Circle(arena.A, 0.4, color='#cde8cd', zorder=0))
    ax.set_aspect('equal'); ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.grid(alpha=0.3)
    ax.set_xlabel('x, m'); ax.set_ylabel('y, m')


def draw_run(ax, name, title, odom=True):
    r = load(name)
    gx, gy, ox, oy = to_world(r)
    draw_arena(ax)
    ax.plot(gx, gy, color=GREY, lw=2, label='ground truth')
    if odom:
        ax.plot(ox, oy, color=BLUE, lw=1, label='/odom')
    for g in np.unique(r['goal']).astype(int):
        k = np.flatnonzero(r['goal'] == g)[-1]
        x, y, a = r['gt_goal_x'][k] + arena.A[0], r['gt_goal_y'][k] + arena.A[1], r['gt_goal_yaw'][k]
        ax.arrow(x, y, 0.35 * math.cos(a), 0.35 * math.sin(a), width=0.02, color='k', zorder=3)
        ax.annotate(str(g), (x, y), (6, -12), textcoords='offset points', fontsize=9)
        ax.plot(gx[k], gy[k], 'o', color=GREY, ms=4)
    ax.set_title(title, fontsize=10)
    ax.legend(loc='upper left', fontsize=8)


def main():
    args = sys.argv[1:]
    pre = 'my_' if '--mine' in args else ''
    names = [a for a in args if a != '--mine'] or sorted(
        p.stem for p in (HERE / 'results').glob(pre + '*.csv') if pre or not p.stem.startswith('my_'))
    have = lambda *ns: all((HERE / 'results' / f'{pre}{n}.csv').exists() for n in ns)
    for n in names:
        table(n)
    print()
    if have('run'):
        sway(pre + 'run')
    for n in ('run', 'k_alpha_20'):
        if have(n):
            near(pre + n)
    (HERE / 'figures').mkdir(exist_ok=True)
    if have('run'):
        fig, ax = plt.subplots(figsize=(8, 5.4))
        draw_run(ax, pre + 'run', 'Five goals, gains (0.5, 1.5, -0.5), steering on /odom')
        fig.tight_layout(); fig.savefig(HERE / f'figures/{pre}trajectories.png', dpi=110)
        print(f'\nwrote figures/{pre}trajectories.png')
    if not have('run', 'high_gains', 'k_alpha_20'):
        print(f'skipped figures/{pre}gains.png: it needs results/{pre}run.csv, {pre}high_gains.csv and {pre}k_alpha_20.csv')
        return
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), gridspec_kw={'width_ratios': [1.4, 0.9, 1.1]})
    total = sum(g[1] for g in goals(load(pre + 'high_gains')))
    draw_run(axes[0], pre + 'high_gains', f'k = (3, 8, -1.5), the textbook example: {total:.0f} s for all five', odom=False)
    ax = axes[1]                                                        # zoom on goal 2, true paths
    for name, col, lab in ((pre + 'k_alpha_20', RED, 'k_alpha = 20'), (pre + 'run', GREY, 'k_alpha = 1.5')):
        r = load(name)
        i = np.flatnonzero(r['goal'] == 2)
        gx, gy, _, _ = to_world(r)
        ax.plot(gx[i], gy[i], color=col, lw=1.5 if col == GREY else 0.8, label=lab)
        cx, cy = r['gt_goal_x'][i[-1]] + arena.A[0], r['gt_goal_y'][i[-1]] + arena.A[1]
        ax.plot(cx, cy, '+', color=col, ms=14, mew=2)                  # each run places its goals from its own start
    ax.set_aspect('equal'); ax.set_xlim(cx - 0.45, cx + 0.25); ax.set_ylim(cy - 0.55, cy + 0.25); ax.grid(alpha=0.3)
    ax.set_xlabel('x, m'); ax.set_ylabel('y, m'); ax.legend(fontsize=8, loc='lower right')
    ax.set_title('Last 0.5 m to goal 2: true path, + true goal', fontsize=10)
    ax = axes[2]
    for name, col, lab, lw in ((pre + 'k_alpha_20', RED, 'k_alpha = 20', 0.7), (pre + 'run', 'k', 'k_alpha = 1.5', 1.8)):
        r = load(name)
        i = np.flatnonzero(r['goal'] == 2)
        ax.plot(r['t'][i] - r['t'][i[0]], r['w_cmd'][i], color=col, lw=lw, label=lab)
    ax.set_xlim(0, 30); ax.set_xlabel('time since goal 2 was set, s'); ax.set_ylabel('commanded w, rad/s')
    ax.set_title('Turn command on the way to goal 2 (first 30 s)', fontsize=10); ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='lower right')
    fig.tight_layout(); fig.savefig(HERE / f'figures/{pre}gains.png', dpi=100)
    print(f'wrote figures/{pre}gains.png')


if __name__ == '__main__':
    main()
