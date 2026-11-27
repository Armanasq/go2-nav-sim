# Module contract

Every module in `modules/` follows this file. Modules 00 and 04 are the reference implementations. Text follows
[docs/WRITING.md](../docs/WRITING.md): plain engineering prose, sentence-case headings, every number with its source.

## Folder layout

```
modules/mNN_<name>/
  README.md          the chapter (template below)
  <impl>.py          the method, first principles, importable, no plotting
  validate.py        the experiment: runs the method, checks it, writes results.json, figures/ and media/
  results.json       written by validate.py, never by hand
  test_<impl>.py     pytest acceptance tests, < 60 s on one core
  figures/           PNG + SVG pairs from figstyle.save()
  media/             MP4 videos and PNG stills from sim.Recorder
```

Import the shared code the way module 04 does:
`sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))`, then `import sim, figstyle` and `from common import ...`.
Run everything with `.venv-modules/bin/python`. Install nothing new: write small solvers yourself (a QP by active set, or
`scipy.optimize`), not osqp, cvxpy or torch.

## Shared code

| File | Use it for |
|---|---|
| `common.py` | `GO2_XML`, `SCENE_XML`, `LEGS`, `JOINTS`, `G`, `require_model(scene)`, `quat_to_R`, `axis_angle_R` |
| `sim.py` | `load()`, `index(m)`, `place_on_ground()`, `stand()`, `joint_pd()`, `min_jerk()`, `run()`, ground truth (`base_pose`, `base_velocity`, `foot_positions`, `foot_forces`, `contacts`), `Recorder` |
| `figstyle.py` | colour dict `C` (+ per-leg `LEGS`), `save(fig, path)`, diagram helpers `diagram`, `box`, `arrow`, `label`, `frame_axes` |
| `m00_math`, `m04_kinematics` | rotations, SO(3)/SE(3); leg FK, Jacobian, closed-form IK, statics. Reuse, do not rewrite |

Conventions fixed by `sim.py`: qpos is `[p (world), quat (w x y z), q (12)]`, qvel is `[v (world), ω (body), dq (12)]`, joint
and actuator order is FL, FR, RL, RR × hip, thigh, calf. Controllers return 12 torques; `run()` clips them to the actuator
range (±23.7 N m hip and thigh, ±45.43 N m knee). Physics step 2 ms (MuJoCo default); pass `ctrl_dt` for a slower controller.
The Menagerie foot contact is soft: standing, the foot sphere centre sits about 9 mm above the floor, 13 mm less than its
22 mm radius (`python modules/sim.py`; `foot_positions` after `stand()`). Account for this before calling a foot height error.

## README template

Sections in this order, all present:

1. Objective: the problem in two or three sentences and what the reader can do afterwards.
2. Prerequisites: earlier modules by link, and the maths assumed.
3. Theory: the derivation, in `$$ ... $$` blocks, each symbol defined once. A worked numeric example on the Go2.
4. Implementation: which function implements which equation, and the design choices with their reason.
5. Experiment: the exact command, inputs, seeds, duration, what is measured.
6. Validation: the acceptance table (below), the figures, the video.
7. Failure analysis: what did not work or degrades, with the measured cause. At least one failure case shown, not described.
8. References: papers and books actually used, with section or equation numbers.

## Figures and video

- At least 2 figures per module, made with `figstyle` and saved with `figstyle.save(fig, 'figures/<name>')` (PNG at 150 dpi +
  SVG). One of them is a diagram that explains the method (frames, forces, block diagram), not only a result plot.
- Colours carry meaning and nothing else: ground truth `C['gt']` grey, estimate or odometry `C['est']` blue, second estimate
  `C['est2']` violet, plan or reference `C['plan']` amber, pass `C['good']`, fail `C['bad']`. Axis labels with units.
- Anything that moves gets at least 1 video: `sim.Recorder(m, 'media/<name>.mp4')` passed as `on_step` to `sim.run`, plus a
  still with `rec.still(d, 'media/<name>.png')` for the README. Limits: H.264, yuv420p, ≤ 1280×720, ≤ 25 s, < 6 MB. The
  Recorder enforces all but the size; check it with `ls -l`.
- Rendering needs `MUJOCO_GL=glfw` and an X display (`DISPLAY=:1` here; egl and osmesa do not work). Without one, `rec.ok`
  is False and recording is skipped with a warning; the experiment must still run and write `results.json`.
- Embed in the README with relative paths: `![caption](figures/x.png)` and `[video](media/x.mp4)` next to the still.

## Acceptance criteria

Each README has a table that `results.json` backs line by line:

| Check | Measured | Acceptance | Source |
|---|---|---|---|
| what is compared, against what, which statistic | number with unit | threshold set before the run | `results.json` key |

- Thresholds are written before running and not moved afterwards. A missed criterion stays in the table, marked as missed,
  with its cause in Failure analysis. Do not delete a row to make the table pass.
- `test_<impl>.py` asserts the same thresholds, so `pytest -q modules` fails when a module regresses.
- Experiments are CPU-light: one process, a few minutes at most. The machine is shared.

## Citing numbers

- A number in prose names where it came from: the `results.json` key, the command, or the table and paper.
- Quote measured values with 2 to 3 significant figures and their unit. State the statistic (max, RMS, median, p95) and the
  sample (how many trials, which seed, which legs).
- Timings depend on machine load; report the load or a ratio between methods run in the same process.
- Literature values carry the paper, table and dataset. Do not mix them with this repository's measurements in one column.

## First principles vs checked

| Written here from first principles | Used only to check it |
|---|---|
| the method itself: kinematics, dynamics terms, estimators, controllers, planners, solvers | MuJoCo (`mj_kinematics`, `mj_jac`, `mj_rne`, `mj_fullM`, contact forces, the simulation itself as ground truth) |
| small numerical tools the method needs (QP, least squares, integration) | Pinocchio (`forwardKinematics`, `computeFrameJacobian`, `rnea`, `crba`, `computeCentroidalMap`) |
| | SciPy (`spatial.transform`, `optimize`, `linalg`) and finite differences |

- `<impl>.py` may call MuJoCo only to read model parameters (masses, offsets, axes) and to step the simulation. It never
  calls a MuJoCo or Pinocchio function that computes the quantity the module teaches.
- Say in the README which check compares against which library, and state the tolerance that counts as agreement.
- MuJoCo state is ground truth only for things an estimator would not have. Estimators read sensors, not `d.qpos`.

## Before marking a module validated

```bash
.venv-modules/bin/python modules/mNN_<name>/validate.py      # writes results.json, figures/, media/
.venv-modules/bin/python -m pytest -q modules                # all modules, not only yours
```

Then add the module row to the table in `modules/README.md` with status `validated`, `partial` (some criteria missed, named in
the README) or `planned`.
