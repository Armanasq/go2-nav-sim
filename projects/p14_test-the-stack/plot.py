#!/usr/bin/env python3
"""Time to first message for the healthy and the broken smoke run.
usage:  python3 projects/p14_test-the-stack/plot.py     (reads results/healthy.json, broken.json; writes figures/first_message.png)"""
import json, pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import smoke_checks as sc

HERE = pathlib.Path(__file__).resolve().parent
ITEMS = ['/imu/data', '/odom', '/scan', 'map -> base_footprint']


def seconds(run):
    return [run['tf_s'] if i == ITEMS[-1] else run['first_msg_s'].get(i) for i in ITEMS]


fig, ax = plt.subplots(figsize=(8, 3.0))
for k, (name, colour) in enumerate([('healthy', '#4a7fb5'), ('broken', '#d9902f')]):
    run = json.load(open(HERE / 'results' / f'{name}.json'))
    for row, s in enumerate(seconds(run)):
        y = row + (k - 0.5) * 0.36
        if s is None:                                            # never arrived: draw a hatched bar to the deadline
            ax.barh(y, sc.DEADLINE, height=0.34, color='none', edgecolor=colour, hatch='///')
            ax.text(sc.DEADLINE + 1, y, 'never', va='center', fontsize=8)
        else:
            ax.barh(y, s, height=0.34, color=colour)
            ax.text(s + 1, y, f'{s:.1f} s', va='center', fontsize=8)
ax.axvline(sc.DEADLINE, color='black', lw=1)
ax.text(sc.DEADLINE - 1, -0.55, f'deadline {sc.DEADLINE:.0f} s', ha='right', fontsize=8)
ax.set_yticks(range(len(ITEMS)), ITEMS); ax.invert_yaxis()
ax.set_xlim(0, sc.DEADLINE + 14); ax.set_xlabel('wall-clock time from the start of the test (s)')
ax.legend(loc='upper center', bbox_to_anchor=(0.45, 1.22), ncol=2, frameon=False, fontsize=8,
          labels=['estimation included (healthy)', 'estimation:=false (broken)'], handles=[plt.Rectangle((0, 0), 1, 1, color=c) for c in ('#4a7fb5', '#d9902f')])
for s in ('top', 'right'):
    ax.spines[s].set_visible(False)
fig.tight_layout(); fig.savefig(HERE / 'figures' / 'first_message.png', dpi=110)
