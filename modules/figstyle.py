"""One matplotlib style for every module figure, colour semantics shared with the website (docs/WRITING.md), and small helpers
for annotated diagrams. Importing this module applies the style.
    import figstyle as fs
    fig, ax = plt.subplots(); ax.plot(t, z_gt, color=fs.C['gt'], label='ground truth'); fs.save(fig, 'figures/height')"""
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

C = dict(gt='#7a8392',      # ground truth
         est='#0b74a8',     # estimate / odometry
         est2='#7a45d6',    # second estimate
         plan='#c98410',    # plan / reference
         good='#1b7f47', bad='#b8352d',
         ink='#1f2328', muted='#5b636e', grid='#d9dde3', paper='#ffffff', fill='#f3f5f8')
LEGS = dict(FL='#0b74a8', FR='#7a45d6', RL='#c98410', RR='#1b7f47')   # fixed per-leg colours when legs are plotted together


def _font():
    for name in ('IBM Plex Sans', 'DejaVu Sans'):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            return name
        except ValueError:
            continue
    return 'sans-serif'


FONT = _font()
plt.rcParams.update({
    'font.family': FONT, 'font.size': 9.5, 'axes.titlesize': 10.5, 'axes.titleweight': 'semibold', 'axes.titlelocation': 'left',
    'axes.labelsize': 9.5, 'axes.labelcolor': C['ink'], 'axes.edgecolor': C['muted'], 'axes.linewidth': 0.6,
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'axes.axisbelow': True,
    'axes.prop_cycle': matplotlib.cycler(color=[C['est'], C['est2'], C['plan'], C['good'], C['bad'], C['gt']]),
    'grid.color': C['grid'], 'grid.linewidth': 0.5, 'grid.linestyle': '-',
    'xtick.color': C['muted'], 'ytick.color': C['muted'], 'xtick.labelsize': 8.5, 'ytick.labelsize': 8.5,
    'xtick.major.width': 0.6, 'ytick.major.width': 0.6, 'text.color': C['ink'],
    'figure.facecolor': C['paper'], 'axes.facecolor': C['paper'], 'savefig.facecolor': C['paper'],
    'figure.figsize': (6.4, 3.6), 'figure.dpi': 110, 'figure.constrained_layout.use': True,
    'lines.linewidth': 1.4, 'lines.markersize': 4, 'legend.frameon': False, 'legend.fontsize': 8.5,
    'svg.fonttype': 'none', 'mathtext.fontset': 'dejavusans',
})


def save(fig, path):
    """Write <path>.png (150 dpi) and <path>.svg (extension in `path` ignored); creates the folder. Returns both paths."""
    p = pathlib.Path(path).with_suffix('')
    p.parent.mkdir(parents=True, exist_ok=True)
    out = p.with_suffix('.png'), p.with_suffix('.svg')
    fig.savefig(out[0], dpi=150, bbox_inches='tight', pad_inches=0.05)
    fig.savefig(out[1], bbox_inches='tight', pad_inches=0.05)
    plt.close(fig)
    return out


def diagram(w=6.4, h=3.6, xlim=(0, 1), ylim=(0, 1), equal=True):
    """Blank axes for a drawn diagram: no grid, no spines, data coordinates set by xlim/ylim."""
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.axis('off')
    if equal:
        ax.set_aspect('equal')
    return fig, ax


def arrow(ax, start, end, text=None, color=None, lw=1.2, offset=(0, 6), ha='center', style='-|>', **kw):
    """Arrow from start to end in data coordinates, with an optional label at its midpoint shifted by `offset` points."""
    color = color or C['ink']
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle=style, color=color, lw=lw, shrinkA=0, shrinkB=0, mutation_scale=10))
    if text:
        mid = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        ax.annotate(text, mid, xytext=offset, textcoords='offset points', ha=ha, va='bottom', color=color, **kw)


def label(ax, xy, text, xytext=None, color=None, ha='left', **kw):
    """Text at xy; with xytext (points offset) a thin leader line points from the text to xy."""
    color = color or C['ink']
    if xytext is None:
        return ax.text(*xy, text, color=color, ha=ha, va='center', **kw)
    return ax.annotate(text, xy, xytext=xytext, textcoords='offset points', color=color, ha=ha, va='center',
                       arrowprops=dict(arrowstyle='-', color=C['muted'], lw=0.6, shrinkA=2, shrinkB=2), **kw)


def box(ax, xy, w, h, text=None, color=None, fill=None, **kw):
    """Rounded box with lower-left corner xy and centred text: blocks in a pipeline or block diagram."""
    color = color or C['ink']
    ax.add_patch(FancyBboxPatch(xy, w, h, boxstyle='round,pad=0,rounding_size=0.02', fc=fill or C['fill'], ec=color, lw=0.9))
    if text:
        ax.text(xy[0] + w / 2, xy[1] + h / 2, text, ha='center', va='center', color=C['ink'], **kw)


def frame_axes(ax, origin, R2=((1, 0), (0, 1)), length=0.1, names=('x', 'z'), color=None):
    """2-D coordinate frame glyph: two arrows from origin along the columns of R2, labelled at their tips."""
    color = color or C['ink']
    for i, n in enumerate(names):
        tip = (origin[0] + length * R2[0][i], origin[1] + length * R2[1][i])
        arrow(ax, origin, tip, color=color, lw=1.0)
        ax.text(tip[0], tip[1], f' ${n}$', color=color, va='center', fontsize=8.5)


if __name__ == '__main__':
    import numpy as np
    out = pathlib.Path(__file__).resolve().parent / 'assets/demo'
    t = np.linspace(0, 3, 300)
    fig, ax = plt.subplots()
    ax.plot(t, 0.25 + 0.01 * np.sin(3 * t), color=C['gt'], label='ground truth')
    ax.plot(t, 0.25 + 0.01 * np.sin(3 * t + 0.2), color=C['est'], label='estimate')
    ax.plot(t, np.full_like(t, 0.25), '--', color=C['plan'], label='reference')
    ax.set(xlabel='time [s]', ylabel='base height [m]', title='Style check'); ax.legend()
    print(save(fig, out / 'figstyle_plot'))
    fig, ax = diagram(5, 3, (0, 1), (0, 0.6))
    box(ax, (0.05, 0.25), 0.25, 0.12, 'controller'); box(ax, (0.6, 0.25), 0.3, 0.12, 'MuJoCo step')
    arrow(ax, (0.3, 0.31), (0.6, 0.31), r'$\tau$'); frame_axes(ax, (0.1, 0.05))
    label(ax, (0.75, 0.37), 'ground truth', xytext=(10, 25), color=C['gt'])
    print(save(fig, out / 'figstyle_diagram'), FONT)
