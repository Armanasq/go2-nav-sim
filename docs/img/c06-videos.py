"""Short videos for chapters 7 and 8 (media/c06-*.mp4).
Run from the repository root:  DISPLAY=:1 MUJOCO_GL=glfw .venv-modules/bin/python docs/img/c06-videos.py [trot] [clips]
trot:  kinematic replay of CHAMP's trot at 0.2 m/s on the MuJoCo Menagerie Go2. The phase generator and foot paths are
       re-implemented from phase_generator.h and trajectory_planner.h (in c06-figures.py), the feet are placed by closed-form IK
       (modules/m04_kinematics). The trunk is moved at the commanded speed at a fixed height; no physics is simulated, so slip,
       tracking error and trunk bounce are absent. Played at half speed.
clips: two cuts from media/go2_a2b_dashboard.mp4 (the recorded A-to-B run)."""
import importlib.util
import os
import pathlib
import subprocess
import sys

import numpy as np
os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'modules')); sys.path.insert(0, str(ROOT / 'modules/m04_kinematics'))
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from kinematics import Leg  # noqa: E402

MEDIA = ROOT / 'media'
DASH = MEDIA / 'go2_a2b_dashboard.mp4'


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f'docs/img/{name}.py')
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


def encode(frames_iter, w, h, fps, out, crf=26):
    cmd = ['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
           '-c:v', 'libx264', '-preset', 'slow', '-crf', str(crf), '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert p.stdin is not None
    for f in frames_iter:
        p.stdin.write(np.ascontiguousarray(f).tobytes())
    p.stdin.close(); p.wait()
    print(out, out.stat().st_size // 1024, 'KB')


def trot(v=0.2, T=4.0, fps=25, slow=2.0, W=1280, H=720, strip_h=210):
    F = _load('c06-figures'); R = _load('c06-render')
    m, d, r = R.setup(H - strip_h, W, wash=(0.06, 0.55))
    L = 0.25 * v                                                # stroke per step, chapter 7
    nominal, depth_, lift = 0.225, 0.01, 0.04                    # gait.yaml
    legs = [Leg(m, n) for n in ('FL', 'FR', 'RL', 'RR')]
    champ_idx = [0, 1, 2, 3]                                     # CHAMP order lf, rf, lh, rh == Menagerie FL, FR, RL, RR
    n = int(T * fps * slow)
    t = np.arange(n) / (fps * slow)
    st, sw = F.champ_phase(t)
    # gait strip: stance solid, swing pale, cursor drawn per frame
    fig = plt.figure(figsize=(W / 100, strip_h / 100), dpi=100)
    ax = fig.add_axes((0.05, 0.25, 0.93, 0.56))
    col = [fs.LEGS['FL'], fs.LEGS['FR'], fs.LEGS['RL'], fs.LEGS['RR']]
    for i, name in enumerate(('lf', 'rf', 'lh', 'rh')):
        for sig, a in ((st[i] > 0, 0.9), (sw[i] > 0, 0.2)):
            e = np.flatnonzero(np.diff(np.r_[0, sig.astype(int), 0]))
            for a0, b0 in zip(e[::2], e[1::2]):
                ax.add_patch(Rectangle((float(t[a0]), 3 - i - 0.38), float(t[b0 - 1] - t[a0]), 0.76, fc=col[i], alpha=a, ec='none'))
    ax.set_xlim(0, T); ax.set_ylim(-0.5, 3.5); ax.set_yticks(range(4)); ax.set_yticklabels(['rh', 'lh', 'rf', 'lf'], fontsize=11)
    ax.tick_params(axis='x', labelsize=10)
    ax.grid(False); ax.set_xlabel('gait time [s]   (solid = stance, pale = swing)', fontsize=11)
    fig.text(0.05, 0.87, f'CHAMP trot at {v} m/s, kinematic replay (no physics), half speed', fontsize=12, fontweight='semibold')
    fig.canvas.draw()
    strip = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()  # type: ignore[attr-defined]  # stubs type canvas as FigureCanvasBase; the Agg canvas has buffer_rgba
    x0, x1 = (ax.transData.transform([(0, 0), (T, 0)])[:, 0]).astype(int)
    yb, yt = sorted(strip_h - ax.transData.transform([(0, -0.5), (0, 3.5)])[:, 1].astype(int))
    plt.close(fig)
    prev = [np.array([0, 0.9, -1.8])] * 4
    held: list[tuple[float, float] | None] = [None] * 4

    def frames():
        for k in range(n):
            q = np.zeros(m.nq); q[3] = 1
            moving = t[k]                                           # trunk moves from the first command on
            q[0], q[2] = v * moving, nominal + 0.022 - depth_
            for i, leg in enumerate(legs):
                j = champ_idx[i]
                if st[j, k] > sw[j, k]:
                    dx, dz = F.stance_path(st[j, k], L, depth_)
                elif sw[j, k] > st[j, k]:
                    xs, zs = F.bezier_swing(sw[j, k], L, lift); dx, dz = float(xs[0]), float(zs[0])
                else:
                    dx, dz = held[i] or (L / 2, 0.0)
                held[i] = (dx, dz)
                target = [leg.hip[0] + dx, leg.hip[1] + leg.d, -nominal + dz]
                sols = leg.ik_closed_form(target)
                qi = sols[np.argmin(np.linalg.norm(sols - prev[i], axis=1))]
                prev[i] = qi; q[7 + 3 * i:10 + 3 * i] = qi
            m.light_pos[:, 0] = q[0]                                  # keep the shadow map centred on the robot
            d.qpos[:] = q; mujoco.mj_forward(m, d)
            img, _ = R.render(m, d, r, [q[0] + 0.03, 0, 0.17], 1.05, -118, -18, fovy=40, far=8.0)
            s = strip.copy()
            cx = int(x0 + (x1 - x0) * t[k] / T)
            s[yb:yt, max(cx - 1, 0):cx + 1] = (31, 35, 40)
            yield np.vstack([img, s])
    encode(frames(), W, H, fps, MEDIA / 'c06-trot-kinematic.mp4')


def clips():
    # chapter 7: the robot starts walking; lower half of the dashboard (lidar, map, run metrics, IMU traces)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', '11', '-t', '12', '-i', str(DASH), '-vf',
                    'crop=1920:600:0:470,scale=1280:-2', '-an', '-c:v', 'libx264', '-preset', 'slow', '-crf', '27',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(MEDIA / 'c06-walk-start.mp4')], check=True)
    # chapter 8: the four sensor panels while the robot passes the obstacles
    fc = ('[0:v]split=4[a][b][c][d];[a]crop=613:376:24:84[e];[b]crop=613:376:653:84[r];[c]crop=613:376:1282:84[dp];'
          '[d]crop=613:376:24:476[l];[e][r]hstack[top];[dp][l]hstack[bot];[top][bot]vstack,scale=-2:720[out]')
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', '40', '-t', '12', '-i', str(DASH), '-filter_complex', fc, '-map', '[out]',
                    '-an', '-c:v', 'libx264', '-preset', 'slow', '-crf', '28', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    str(MEDIA / 'c06-sensors.mp4')], check=True)
    for f in ('c06-walk-start.mp4', 'c06-sensors.mp4'):
        print(MEDIA / f, (MEDIA / f).stat().st_size // 1024, 'KB')


if __name__ == '__main__':
    for w in sys.argv[1:] or ['trot', 'clips']:
        globals()[w]()
