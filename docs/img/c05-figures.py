"""Chapter 5 figure: camera_link and camera_optical axes from the two tf2_echo matrices of the broken-tree exercise.
Run from the repository root:  .venv-modules/bin/python docs/img/c05-figures.py"""
import io
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules'))
import figstyle  # noqa: E402,F401  (house style)
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402

# Rotation part of `tf2_echo camera_link camera_optical`, as printed (columns = optical axes in camera_link).
AS_LAUNCHED = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])   # yaw +1.5708
FIXED = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])         # yaw -1.5708, same as the robot's URDF
COLS = ('#c0392b', '#1e8449', '#1f5fbf')


def broken_optical():
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(7.6, 3.9))
    for k, (title, R) in enumerate((('as launched: yaw +1.5708\noptical z points backwards, x left', AS_LAUNCHED),
                                    ('fixed: yaw -1.5708\noptical z forward, x right, y down', FIXED))):
        ax = fig.add_subplot(1, 2, k + 1, projection='3d')
        ax.set_proj_type('ortho'); ax.view_init(elev=22, azim=-128)
        for i, n in enumerate('xyz'):                        # camera_link: thin, faded
            v = np.eye(3)[i]
            ax.quiver(0, 0, 0, *v, color=COLS[i], lw=1.0, alpha=0.4, arrow_length_ratio=0.12)
            ax.text(*(1.1 * v + np.array([0, 0, -0.14])), n + '$_{link}$', color=COLS[i], alpha=0.6, fontsize=7.5)
        for i, n in enumerate('xyz'):                        # camera_optical: thick
            v = R[:, i]
            ax.quiver(0, 0, 0, *v, color=COLS[i], lw=2.4, arrow_length_ratio=0.15)
            ax.text(*(1.25 * v + np.array([0, 0, 0.08])), n + '$_{opt}$', color=COLS[i], fontsize=9.5)
        ax.text(1.5, 0, -0.35, 'forward', fontsize=7, color='#555')
        ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1); ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
        ax.set_title(title, fontsize=8.5)
    buf = io.BytesIO(); fig.savefig(buf, format='png', dpi=150, bbox_inches='tight', pad_inches=0.05); plt.close(fig)
    im = Image.open(buf).convert('RGB').quantize(colors=96, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    path = ROOT / 'docs/img/c05-broken-optical.png'; im.save(path, optimize=True)
    print(path.name, path.stat().st_size // 1024, 'KB', im.size)


if __name__ == '__main__':
    assert np.allclose(AS_LAUNCHED @ AS_LAUNCHED.T, np.eye(3)) and np.allclose(FIXED @ FIXED.T, np.eye(3))
    assert np.allclose(FIXED @ [0, 0, 1], [1, 0, 0])            # fixed: optical z = link x (forward)
    assert np.allclose(AS_LAUNCHED @ [0, 0, 1], [-1, 0, 0])     # as launched: optical z = -link x (backwards)
    broken_optical()
