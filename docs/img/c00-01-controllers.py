"""Chapter 1 figure: four locomotion controllers that all sit behind /cmd_vel (docs/img/c00-01-controllers.png).
CHAMP facts come from the CHAMP source in this repository (chapter 1); the other three columns from research/papers.md
(Di Carlo et al. 2018, Kim et al. 2019, Rudin et al. 2021). Run from the repository root:

    python3 docs/img/c00-01-controllers.py
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules'))
import figstyle as fs  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

C = fs.C
COLS = [  # title, source, reads, computes, sends to the joints, rate
    ('CHAMP (this stack)', 'chvmp/champ', '/cmd_vel, /body_pose\nnothing about the trunk',
     'phase clock, Raibert\nfootstep, Bezier swing,\nleg IK', '12 joint positions\n-> joint PID', '200 Hz'),
    ('Convex MPC', 'Di Carlo et al. 2018', '/cmd_vel, trunk pose\nand velocity, contacts',
     'ground forces over a\n0.5 s horizon, single\nrigid body, convex QP', 'stance torques from\nforces, swing PD', '20-30 Hz'),
    ('MPC + WBIC', 'Kim et al. 2019', 'as MPC, plus the\nfull joint state',
     'MPC forces, then a\nwhole-body QP on the\nfull model', 'joint torques\n(+ PD targets)', 'MPC slow,\nWBIC faster'),
    ('RL policy', 'Rudin et al. 2021', '/cmd_vel, IMU, joint\npositions, velocities',
     'neural network trained\nwith PPO in simulation', 'joint position targets\n-> joint PD', 'one forward\npass per step'),
]


def main():
    fig, ax = fs.diagram(7.6, 4.0, (0, 1), (0, 1), equal=False)
    w, gap, x0 = 0.225, 0.017, 0.035
    ax.text(0.5, 0.965, '/cmd_vel  (vx, vy, yaw rate)  from Nav2 or teleop', ha='center', va='center', fontsize=8.6,
            family='DejaVu Sans Mono', bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=C['plan'], lw=0.9))
    rows = [('reads', 0.70), ('computes', 0.50), ('to the joints', 0.30), ('rate', 0.155)]
    for name, y in rows:
        ax.text(0.0, y, name, rotation=90, ha='center', va='center', fontsize=7.6, color=C['muted'])
    for i, (title, src, reads, comp, out, rate) in enumerate(COLS):
        x = x0 + i * (w + gap)
        champ = i == 0
        ax.add_patch(FancyBboxPatch((x, 0.09), w, 0.75, boxstyle='round,pad=0,rounding_size=0.012',
                                    fc='#e3f0f7' if champ else C['fill'], ec=C['est'] if champ else C['muted'], lw=0.9))
        fs.arrow(ax, (x + w / 2, 0.935), (x + w / 2, 0.84), color=C['plan'], lw=1.0)
        ax.text(x + w / 2, 0.81, title, ha='center', va='center', fontsize=8.8, fontweight='semibold')
        ax.text(x + w / 2, 0.775, src, ha='center', va='center', fontsize=7.2, color=C['muted'])
        col = C['bad'] if champ else C['ink']
        ax.text(x + w / 2, 0.70, reads, ha='center', va='center', fontsize=7.6, color=col, linespacing=1.2)
        for txt, y in ((comp, 0.50), (out, 0.30), (rate, 0.155)):
            ax.text(x + w / 2, y, txt, ha='center', va='center', fontsize=7.6, linespacing=1.2)
        fs.arrow(ax, (x + w / 2, 0.09), (x + w / 2, 0.045), color=C['ink'], lw=1.0)
    ax.text(0.5, 0.02, 'ros2_control hardware interface: 12 joints (Gazebo here, motor drivers on a robot)', ha='center',
            va='center', fontsize=8, bbox=dict(boxstyle='round,pad=0.3', fc='#eceff3', ec=C['gt'], lw=0.8))
    for y in (0.605, 0.405, 0.215):
        ax.plot([x0, x0 + 4 * w + 3 * gap], [y, y], color=C['grid'], lw=0.6)
    p = ROOT / 'docs/img/c00-01-controllers.png'
    fig.savefig(p, dpi=150, bbox_inches='tight', pad_inches=0.06)
    kb = p.stat().st_size / 1024
    assert kb < 400, f'{p.name} is {kb:.0f} KB'
    print(f'{p.relative_to(ROOT)}  {kb:.0f} KB')


if __name__ == '__main__':
    main()
