# Project 6. Reactive obstacle avoidance without a map

## Goal

Walk the Go2 from marker A to marker B around the arena obstacles with no map and no Nav2. The node reads only the 2D scan
(`/scan`) and its own pose estimate (`/odom`), and on every scan picks a direction to walk in. You write two ways of picking it:
a potential field, which adds a pull towards the goal to a push away from every nearby return, and a simplified vector field
histogram, which takes the free corridor nearest to the goal. You tune both in a 2D model of the arena first, then run at least
five trials each way in Gazebo, score them on ground truth (success, time, path length, minimum clearance), compare them with
the Nav2 runs of [chapter 11](../../docs/11-record-and-visualise.md), and make each method fail in a way you can explain.

## What you will learn

- Turning a `LaserScan` into points the controller can reason about, and dropping the returns that mean "nothing seen".
- A potential field: the pull, Khatib's push, and why their sum can be zero in front of an obstacle (a local minimum).
- A free-corridor test: which directions are clear for a robot of a given width, and how to pick one of them.
- Why a reactive choice needs memory (hysteresis), and the traps that it still walks into.
- Placing a goal given in the world frame into the odometry frame once, at the start, and living with odometry drift after that.
- Tuning against a cheap 2D model before spending simulator time, and checking where the model and Gazebo disagree.

## Before you start

You need [project 1](../p01_first-steps/README.md) (starting the stack, `/odom_gt`, why only a zero twist stops CHAMP),
[project 3](../p03_move-in-shapes/README.md) (`/odom` drifts while the robot stands), [project 4](../p04_go-to-pose/README.md)
(steering on `/odom`, `wrap`, reading a pose from `nav_msgs/Odometry`) and [project 5](../p05_sense-and-stop/README.md)
(bearings of a `LaserScan`, `inf` returns, `range_min` = 0.4 m).

Every terminal starts with:

```bash
cd go2-nav-sim
source env.sh
```

This project needs the simulator only. Nav2 must not run (it would also publish on `/cmd_vel`), and the estimation stack must
not run either, because the project's launch file starts its own copy for every trial (step 4 says why). If `scripts/up.sh` is
running, stop it with `scripts/stop.sh` first.

The arena is defined in `src/go2_nav_sim/scripts/make_arena.py`: walls at x = ±6 and y = ±5 (inner faces at ±5.9 and ±4.9),
four boxes, two cylinders, A = (-4.5, -3.5) and B = (4.0, 3.0). The two boxes that matter most here are the long brick box
centred at (1.0, 0.0), 0.8 × 2.4 m, and the pallet at (2.5, -2.5), 1.2 m square. The gap between their nearest corners is
0.86 m. `/odom_gt` is relative to A, so world = `/odom_gt` + A.

| File | What it is |
|---|---|
| `reactive.py` | the pure logic, no ROS: scan to points, potential field, free corridors, the command, the stuck detector |
| `arena2d.py` | the arena in 2D: a ray-cast `/scan`, distance to the nearest obstacle, a kinematic robot that runs `reactive.py` |
| `test_reactive.py` | 11 unit tests, including two whole runs in the 2D model |
| `avoid.py` | the node: steers on `/scan` and `/odom`, logs every scan to `results/NAME.csv`, prints a `RESULT` line |
| `avoid.launch.py` | one trial: a fresh estimation stack plus `avoid.py`; ends when `avoid.py` exits |
| `analyze.py` | scores runs on ground truth, adds the Nav2 runs, draws `figures/trials.png` |
| `results/` | my logs: `ab*` (A to B), `ba*` (B to A), `apf_*`, `vfh_wall`, `stood_*`, the Nav2 tracks `nav2_w*.csv`, `trials.txt`, `extra.txt`, `analyze_output.txt` |

Name your own runs `my_...` so they do not overwrite mine.

## Steps

### 1. Run the unit tests

```bash
python3 -m pytest -q projects/p06_avoid-obstacles
```

Expected:

```
11 passed in 5.54s
```

Check: they need no ROS graph, so they pass with the simulator down. The two slowest are whole runs of the 2D model.

### 2. Try both methods in the 2D model

`arena2d.py` drives a kinematic robot (it walks exactly as commanded, no gait, no sensor noise, no delay) with the same
`reactive.step` the node uses, on a scan ray-cast from the arena geometry with the beam layout of `config/scan.yaml`.

```bash
python3 projects/p06_avoid-obstacles/arena2d.py                                   # vfh, A to B, facing +x
python3 projects/p06_avoid-obstacles/arena2d.py --method apf                      # potential field, same start
python3 projects/p06_avoid-obstacles/arena2d.py --method apf --start -1 0 0 --goal 3 0
```

Expected:

```
reached after 46.7 s, path 12.62 m, min clearance 0.55 m, end (3.72, 2.93)
stuck after 61.3 s, path 6.71 m, min clearance 0.56 m, end (1.14, -1.92)
stuck after 33.2 s, path 0.93 m, min clearance 0.78 m, end (-0.19, 0.04)
```

The output is deterministic, so yours should match to the last digit. The second run ends between the brick box and the pallet,
the third in front of the brick box with the goal straight behind it. Both are local minima of the potential field, explained
in [How the code works](#the-potential-field-and-its-local-minimum). Check: `stuck` in lines 2 and 3, `reached` in line 1.

Every value of `reactive.P` can be overridden with `--set KEY=VALUE`, which is how I tuned it: a run of the model takes a few
seconds, a Gazebo trial one to two minutes.

### 3. Start the simulator

Terminal 1:

```bash
ros2 launch go2_nav_sim sim.launch.py
```

Gazebo took one to two minutes to come up on my machine. Check, in terminal 2:

```bash
ros2 topic echo --once /odom_gt --field pose.pose.position
```

prints a position near x = 0, y = 0 (the robot on A). Mine printed x = 0.225, y = 0.022 after the robot had stood for about
six minutes: it creeps forward by about half a millimetre per second while standing.

### 4. Run one trial from A to B

```bash
ros2 launch projects/p06_avoid-obstacles/avoid.launch.py name:=my_ab1
```

The launch file starts `estimation.launch.py` (scan, lidar odometry, EKF), waits 10 s, then starts `avoid.py` with the goal B.
Expected, in between the estimation logs:

```
[python3-4] [INFO] [...] [avoid]: vfh: goal (4.00, 3.00) in the world frame
[python3-4] [INFO] [...] [avoid]: start at (-4.33, -3.39, -169 deg)
[python3-4] RESULT REACHED method=vfh time=41.1s path=10.94m min_clearance=0.57m true_dist_to_goal=0.31m odom_err_end=0.03m
[INFO] [python3-4]: process has finished cleanly [pid ...]
```

then a `KeyboardInterrupt` traceback from `lidar_odom.py` as the launch shuts the estimator down, which is harmless. The RESULT
line above is my run `ab7`. `time` here runs from the first scan to the end; [Results](#results) uses a stricter definition.
Everything after `time` is measured on `/odom_gt`: the path walked, the smallest distance from the robot centre to an obstacle
surface, how far from the goal it really stopped, and how far `/odom` was from the truth at the end.

Check: `REACHED`, `true_dist_to_goal` below 0.4 m (the node stops when its own estimate is within 0.3 m, plus the odometry
error) and `min_clearance` above 0.5 m.

The estimator is restarted on every trial on purpose: with an estimator that had run for 7 and 11 minutes, mostly with the robot
standing, my runs ended 0.25 m and 1.87 m away from where `/odom` placed them ([If it goes wrong](#if-it-goes-wrong)).

### 5. Walk back to A

```bash
ros2 launch projects/p06_avoid-obstacles/avoid.launch.py gx:=-4.5 gy:=-3.5 name:=my_ba1
```

My `ba7`:

```
[python3-4] RESULT REACHED method=vfh time=65.3s path=16.65m min_clearance=0.57m true_dist_to_goal=0.31m odom_err_end=0.02m
```

Check: the path back is about 5 m longer than the path out. [Results](#results) shows the two routes and why they differ.

### 6. Run five round trips

```bash
for i in 2 3 4 5 6; do
  ros2 launch projects/p06_avoid-obstacles/avoid.launch.py name:=my_ab$i 2>&1 | grep RESULT
  ros2 launch projects/p06_avoid-obstacles/avoid.launch.py gx:=-4.5 gy:=-3.5 name:=my_ba$i 2>&1 | grep RESULT
done
```

Each leg took one to two minutes of wall time. Each trial starts where the last one ended, so after a `STUCK` the next leg
does not start on A or B. In my runs `ba4` got stuck, and `ab5` then started 7.8 m from A and walked only 3.1 m; I deleted it
and ran `ab6` and `ab7` after walking the robot back to A. Do the same: after a `STUCK`, send the robot to the start with one
more launch, and do not count that leg.

Check: one `RESULT` line per leg, each starting with `REACHED`, `STUCK` or `TIMEOUT`.

### 7. Score the trials and draw them

```bash
python3 projects/p06_avoid-obstacles/analyze.py --runs my_ab1 my_ab2 my_ab3 my_ab4 my_ab5 my_ab6 \
    my_ba1 my_ba2 my_ba3 my_ba4 my_ba5 my_ba6 --out figures/my_trials.png
```

prints one line per run plus the three Nav2 runs, and writes `figures/my_trials.png`. Mine are in [Results](#results).
`analyze.py` uses only `/odom_gt`: a run starts when the robot has moved 5 cm from where it stood and ends when it last moved
5 cm from where it finished, so standing time at either end is not counted, for our runs and for Nav2 alike.

The Nav2 tracks are already in `results/nav2_w1.csv` to `nav2_w3.csv`. I extracted them from the mission bags of
[chapter 9](../../docs/09-state-estimation.md) with `--nav2-bags bags/w1 bags/w2 bags/w3` (which needs the 0.7 GB bags);
you do not need to repeat that.

Check: in each line `min clear m` is above 0.5 and `true miss m` below 0.4 for every run that reached its goal.

### 8. Make the potential field fail in Gazebo

Walk to a point 1.6 m in front of the brick box, then ask the potential field to reach the point straight behind it:

```bash
ros2 launch projects/p06_avoid-obstacles/avoid.launch.py gx:=-1.0 gy:=0.0 name:=my_to_wall
ros2 launch projects/p06_avoid-obstacles/avoid.launch.py gx:=3.0 gy:=0.0 method:=apf name:=my_apf_wall
```

Mine (from `results/extra.txt`, where each line starts with the run name):

```
to_wall [python3-4] RESULT REACHED method=vfh time=24.1s path=6.01m min_clearance=0.60m true_dist_to_goal=0.22m odom_err_end=0.07m
apf_wall [python3-4] RESULT STUCK method=apf time=35.0s path=2.97m min_clearance=0.76m true_dist_to_goal=3.21m odom_err_end=0.00m
```

The robot walked up to the box, stopped 0.76 to 0.80 m from its face and turned back and forth on the spot until the stuck rule
ended the run, 30 s after it last got 0.25 m closer. Check: `STUCK`, and the robot stands near x = -0.2, y = 0 (the 2D model
of step 2 stopped at -0.19, 0.04). Then try the histogram from the same spot:

```bash
ros2 launch projects/p06_avoid-obstacles/avoid.launch.py gx:=3.0 gy:=0.0 name:=my_vfh_wall
```

Mine also failed, in a different place (`vfh_wall` in `results/extra.txt`, and the right panel of the figure). Section
[Where the histogram gets trapped](#where-the-histogram-gets-trapped) explains both.

### 9. Record a video

In another terminal, before a trial:

```bash
python3 projects/p05_sense-and-stop/record.py ~/p06_ab.mp4 --topic /spectator/cam/image_raw
```

and Ctrl-C it after the trial. It prints the frame count (mine: `627 frames`, 62.7 s at 10 frames per second of simulated time).
Use project 5's recorder, not `projects/common/record_cam.py`: on my loaded machine the common recorder kept 5 frames of the
whole run. Speed it up to fit in 25 s:

```bash
ffmpeg -i ~/p06_ab.mp4 -vf "setpts=PTS/2.6,scale=960:-2" -r 15 -c:v libx264 -crf 28 -pix_fmt yuv420p -an ~/p06_ab_fast.mp4
```

Check: `ffprobe ~/p06_ab_fast.mp4` gives a duration near 24 s (mine 24.3 s, 105 kB).

### 10. Stop

Every trial ends with zeros on `/cmd_vel`, so the robot is standing. Ctrl-C the simulator in terminal 1, then
`scripts/stop.sh`.

## How the code works

### From a scan to points

```python
def valid_returns(ranges, angle_min, angle_inc, range_min, range_max):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return r[ok], a[ok]
```

As in project 5: beam i points at `angle_min + i * angle_increment` in `base_footprint` (x forward, y left), `inf` means no
return, and values outside `[range_min, range_max]` are unusable. Here the whole 360 degrees is used: 901 beams, 0.4 degrees
apart. `range_min` is 0.4 m, so anything closer than 0.4 m to the scan origin is invisible. Both methods below keep the robot
centre at least 0.5 m from everything for that reason.

### The potential field and its local minimum

```python
def apf_direction(r, a, g, angle_inc, p=P):
    att = p['k_att'] * np.asarray(g) / max(math.hypot(*g), 1e-9)
    near = r < p['d0']
    d, b = r[near], a[near]
    mag = p['k_rep'] * (1 / d - 1 / p['d0']) / d ** 2 * angle_inc
    f = att - np.array([np.sum(mag * np.cos(b)), np.sum(mag * np.sin(b))])
    return math.atan2(f[1], f[0])
```

The pull has constant size `k_att` and points at the goal `g` (in the robot frame). Each return at distance d < d0 pushes
straight away from itself with

    |push| = k_rep (1/d - 1/d0) / d²

which is minus the gradient of Khatib's potential ½ k_rep (1/d - 1/d0)². Each beam's push is multiplied by its angular
width, so a wall pushes as hard at 0.4 degree spacing as at 1 degree. With d0 = 1.5 m and k_rep = 1.0 a
wall starts to matter 1.5 m away and the push grows like 1/d³ close to it.

The robot walks along the sum. A local minimum is a place where the sum is zero. With the goal straight behind the brick box,
the pull is +x and, by symmetry, so is the whole push, reversed: the y parts of the push from the upper and lower halves of the
face cancel. At some distance the two are equal. In my Gazebo run `apf_wall` the robot reached that point 0.76 m from the face
and stopped making progress. The direction of a near-zero sum is set by noise, so the log shows it flipping from side to side:

```
t       x      y      v     w      th
20.06  -0.18   0.07   0.00  -0.50  -2.65
27.64  -0.21  -0.10   0.00   0.50   2.60
```

(columns `t, gt_x, gt_y, v, w, th` of `results/apf_wall.csv`). The second failure in step 2, between the brick box and the
pallet, is the same balance with two obstacles: their pushes cancel the pull in a gap of 0.86 m between corners. Nothing in the
field says which way round is shorter, so the method has no way out by itself. The unit test
`test_apf_symmetric_wall_points_straight_at_it` checks the symmetric case: the direction comes out as exactly 0.

### Free corridors (the histogram)

```python
def free_directions(r, a, look, r_safe, n_dir):
    t = np.linspace(-math.pi, math.pi, n_dir, endpoint=False)
    rel = a[None, :] - t[:, None]
    along, side = r * np.cos(rel), np.abs(r * np.sin(rel))
    blocked = ((along > 0) & (along < look) & (side < r_safe)).any(axis=1)
    return t, ~blocked
```

For each of 180 candidate bearings t (2 degrees apart), each return is split into a distance along t and a distance across it.
The bearing is blocked if any return lies ahead within `look` = 2.0 m and closer than `r_safe` = 0.55 m to the centre line:
the robot, walking that way, would pass within 0.55 m of it. That is the polar histogram of VFH reduced to a yes or no per
bearing, with the robot's width built in (Nav2's footprint radius for this robot is 0.33 m, `config/nav2_params.yaml`, which
leaves 0.22 m of margin). It is a matrix of 180 × up to 901 numbers per scan: 7.6 ms per call on my loaded machine, with a scan every 100 ms.

```python
def vfh_direction(r, a, g, prev=None, p=P):
    dist = math.hypot(*g)
    t, free = free_directions(r, a, min(p['look'], dist), p['r_safe'], int(p['n_dir']))
    if not free.any():
        return None
    ang = lambda x: np.abs(np.arctan2(np.sin(t - x), np.cos(t - x)))
    cost = ang(math.atan2(g[1], g[0])) + p['k_turn'] * np.abs(t) + (0 if prev is None else p['k_prev'] * ang(prev))
    cost[~free] = np.inf
    return float(t[np.argmin(cost)])
```

Of the free bearings it takes the cheapest:

    cost(t) = |t - goal bearing| + k_turn |t| + k_prev |t - last choice|

The second term prefers what is already ahead (k_turn = 0.3), the third keeps the last choice (k_prev = 0.5). The look-ahead
is cut to the distance to the goal, so a wall behind the goal does not block it. Without the third term my first Gazebo version
flipped between two gaps of nearly the same cost (chosen bearing +1.75 rad, then -2.83 rad, then back) and turned on the spot
until it was declared stuck after 25 s. `step()` takes the last choice in world coordinates and turns it into the robot frame,
so the choice survives the robot turning. Choosing a corridor 0.55 m wide also closes the 0.86 m gap between the brick box and
the pallet, which is where the potential field got caught.

The look-ahead mattered most in tuning. On six starts around A (±0.3 m, ±20 degrees) the 2D model reached B six times with
`look` 1.5, 2.0 and 2.5 m, but with 1.5 m it went south round the pallet on three of them (15.5 to 15.8 m), and with 2.0 m it
took the 11 m route on five.

### From a direction to a command

```python
def command(th, front, p=P):
    if th is None:
        return 0.0, p['w_max']
    w = max(-p['w_max'], min(p['w_max'], p['k_w'] * th))
    brake = min(1.0, max(0.0, (front - p['stop']) / (p['slow'] - p['stop'])))
    return p['v_max'] * max(0.0, math.cos(th)) * brake, w
```

Both methods give a bearing th in the robot frame. The robot turns towards it at w = 1.2 th, limited to CHAMP's 0.5 rad/s, and
walks at 0.3 m/s × cos th: full speed when th = 0, nothing when the direction is 90 degrees or more to the side. `front` is the
nearest return within ±35 degrees of straight ahead; the speed falls linearly from full at 1.2 m to zero at 0.5 m. CHAMP keeps
walking on the last command, so the node sends a command on every scan and five zeros at the end.

### Placing the goal, and the stuck rule

```python
if self.anchor is None:                                  # once: where the odom frame sits in the world
    self.anchor, self.t0 = reactive.compose(self.gt, reactive.inverse(self.odom)), t
pose = reactive.compose(self.anchor, self.odom)
```

The goal is given in the world frame, `/odom` starts wherever the estimator was started. On the first scan, the node takes the
true pose from `/odom_gt` once and computes the transform T = T_world_base · T_odom_base⁻¹ that places the odom frame in the
world. From then on the pose is T · `/odom`. This is the same shortcut `go_to_goal.py` takes to start AMCL (top-level README). After that `/odom_gt` is
only logged. Without a map nothing corrects the odometry, so its error goes straight into where the robot stops
(`odom_err_end` in the RESULT line).

```python
def stuck(self, t, dist):
    if self.t_best is None or dist < self.best - self.eps:
        self.best, self.t_best = dist, t
    return t - self.t_best > self.window
```

The run is declared stuck when the distance to the goal has not dropped 0.25 m below its best value for 30 s. This cannot
tell a local minimum from a long detour that first leads away from the goal. With a 20 s window the 2D model declared a run
stuck that would have arrived (from (-1, 0) to (3, 0) with `look` = 1.5 m it gave up after 26.6 s; with 30 s it arrives after
35.7 s, `--start -1 0 0 --goal 3 0 --set look=1.5`); 30 s was enough for the detours in this arena.

## Results

Measured on a shared machine running other simulators (load average 44 to 75 on 32 cores from `uptime` at the start and the
end, Gazebo real-time factor 0.98 from `gz stats`). All times are simulated seconds. Simulator only, a fresh estimator for every leg, method `vfh`
unless the run name starts with `apf`. Table from

```bash
python3 projects/p06_avoid-obstacles/analyze.py --runs ab1 ab2 ab3 ab4 ab6 ab7 apf_ab1 ba1 ba2 ba3 ba4 ba5 ba7 --fail ba4 apf_wall vfh_wall
```

(full output in `results/analyze_output.txt`; the RESULT lines in `results/trials.txt` and `results/extra.txt`):

| Run | Goal | Result | Time (s) | Path (m) | Min clearance (m) | True miss (m) | /odom error at end (m) |
|---|---|---|---|---|---|---|---|
| ab1 | B | reached | 38.6 | 10.97 | 0.57 | 0.27 | 0.06 |
| ab2 | B | reached | 40.0 | 10.94 | 0.57 | 0.27 | 0.11 |
| ab3 | B | reached | 39.8 | 10.85 | 0.57 | 0.31 | 0.06 |
| ab4 | B | reached | 40.2 | 10.87 | 0.57 | 0.32 | 0.05 |
| ab6 (video) | B | reached | 49.2 | 12.57 | 0.57 | 0.29 | 0.13 |
| ab7 | B | reached | 39.5 | 10.83 | 0.57 | 0.31 | 0.03 |
| ba1 | A | reached | 58.6 | 15.73 | 0.56 | 0.36 | 0.14 |
| ba2 | A | reached | 59.8 | 15.94 | 0.57 | 0.33 | 0.18 |
| ba3 | A | reached | 58.9 | 15.94 | 0.57 | 0.23 | 0.18 |
| ba4 | A | stuck | 43.6 | 10.94 | 0.56 | 7.79 | 0.03 |
| ba5 | A | reached | 59.2 | 15.94 | 0.57 | 0.32 | 0.12 |
| ba7 | A | reached | 64.2 | 16.57 | 0.57 | 0.31 | 0.02 |
| apf_ab1 (potential field) | B | reached | 57.7 | 12.29 | 0.58 | 0.26 | 0.13 |
| Nav2 w1 (map, AMCL) | B | succeeded | 50.9 | 11.31 | 0.62 | 0.25 | |
| Nav2 w2 | B | succeeded | 51.5 | 11.37 | 0.62 | 0.21 | |
| Nav2 w3 | B | succeeded | 79.4 | 13.54 | 0.51 | 0.34 | |

Summary:

| | Success | Time (s) | Path (m) | Min clearance (m) | True miss (m) |
|---|---|---|---|---|---|
| Reactive (vfh), A to B | 6 of 6 | 38.6 to 49.2, mean 41.2 | 10.83 to 12.57, mean 11.17 | 0.57 in every run | 0.27 to 0.32 |
| Reactive (vfh), B to A | 5 of 6 | 58.6 to 64.2 | 15.73 to 16.57 | 0.56 to 0.57 | 0.23 to 0.36 |
| Nav2 with the map, A to B | 3 of 3 | 50.9 to 79.4 | 11.31 to 13.54 | 0.51 to 0.62 | 0.21 to 0.34 |

![Three arena plots. Left: six reactive runs from A to B, true paths in black with the odometry estimate dashed in blue, and three Nav2 runs in amber; all but one reactive run follow nearly the Nav2 route north of the low pallet and above the brick box. Middle: five reactive runs from B to A, all going south past the east side of the brick box and the pallet and then west along the south wall. Right: three failures, each ending in a cross: ba4 turning back north beside the pallet and stopping east of the brick box, apf_wall stopping in front of the brick box, vfh_wall stopping between the brick box and the pallet](figures/trials.png)

<video controls muted playsinline src="media/a_to_b_vfh.mp4"></video>

The video is run `ab6` from the spectator camera, 62.7 s of simulated time sped up 2.6 times to 24.3 s (step 9). It is the one
A-to-B run that went south of the first pallet and then north between it and the brick box.

What the numbers say:

- From A to B the reactive navigator arrived in all six runs, and five of them took almost the route Nav2 plans on the map
  (10.83 to 10.97 m against Nav2's 11.31 and 11.37 m in w1 and w2). It was faster: 38.6 to 40.2 s against 50.9 and 51.5 s.
  Nav2's controller is set to `desired_linear_vel: 0.25` m/s (`config/nav2_params.yaml`) and slows down in turns tighter than
  0.9 m radius (`use_regulated_linear_velocity_scaling`); this node walks at 0.3 m/s whenever the way ahead is clear.
- The minimum clearance was 0.56 to 0.57 m in every reactive run: the robot always passed one obstacle as close as the corridor
  half-width (0.55 m) allows. Nav2 kept 0.62 m in w1 and w2 and 0.51 m in w3. Measured from the robot centre; with the 0.33 m
  footprint radius the closest the body came was about 0.24 m.
- B to A is 5 m longer than A to B. The robot arrives on B facing about +25 degrees, and with that start heading the
  histogram goes south down the east side of the arena and west along the south wall. In the 2D model the same happens:
  `arena2d.py --start 3.75 2.9 25 --goal -4.5 -3.5` gives 15.63 m, while the same start facing west (`--start 4 3 180`) gives
  10.89 m. Without a plan the route is decided by where the robot happens to be facing.
- The robot stopped 0.23 to 0.36 m from the goal: the 0.3 m tolerance on its own estimate plus an odometry error of 0.02 to
  0.18 m. Nav2 has the same kind of tolerance (0.25 m) and corrects its odometry with AMCL; this node cannot, and over longer
  routes the error would grow ([project 3](../p03_move-in-shapes/README.md) measures the drift).
- The one potential-field run from A to B (`apf_ab1`) arrived, although the 2D model of step 2 predicted a stall between the
  brick box and the pallet. The Gazebo run started facing 180 degrees (it had just walked back to A); the model from that pose
  (`--method apf --start -4.26 -3.62 180`) also arrives, after 11.49 m. The local minimum is there, but the robot only falls
  into it from some starting headings.
- Six and three runs are small samples. One histogram failure in twelve legs is not a failure rate.

### Where the histogram gets trapped

All three failures in the right panel of the figure are traps: places where every free bearing points away from the goal and
the cheapest one changes as the robot turns.

- `ba4`, 29 s into the run, beside the pallet at (3.7, -3.1). In `ba3` the robot stood in the same place (3.71, -3.04) and
  chose a bearing of -40 degrees, round the pallet's south-east corner; 4.5 s later it was walking west below the pallet. In
  `ba4` it chose -49 degrees, then, half a second later, +91 degrees: the corridor round the corner and the corridor back north
  cost almost the same, a few degrees of heading tipped it, and `k_prev` then held it on the way back. It walked north into
  the space between the brick box's east face and the pallet, where the brick box blocks the way to A, and turned there until
  the stuck rule ended the run at (2.52, -0.12).
- `vfh_wall` started where `apf_wall` gave up, facing south (-76 degrees). It walked south past the end of the brick box into
  the corner between the box and the pallet (closed to it, since 0.86 m < 2 × 0.55 m) and turned there; the chosen bearing went
  -2.83, -1.57, -0.49, -2.79 rad over 7.5 s. The 2D model from (-1, 0) facing +x went north round the box instead and arrived
  ([Try this](#try-this), exercise 2).
- `apf_wall` is the potential-field minimum, explained above.

A reactive method sees only the current scan. It cannot know that a corner is closed until it is in it, and it has no memory
of where it has been, so it can walk back into the same place. Nav2 plans on the whole map and does not have this problem in
this arena; that is the price of having no map.

## Try this

1. Halve the push and predict where the potential field stalls in front of the brick box.

   ```bash
   python3 projects/p06_avoid-obstacles/arena2d.py --method apf --start -1 0 0 --goal 3 0 --set k_rep=0.5
   ```

   Answer: `stuck after 33.2 s, path 1.07 m, min clearance 0.66 m, end (-0.07, 0.02)`, against 0.78 m with k_rep = 1.0. Halving
   the gain moves the balance only 12 cm closer, because near the wall the push grows like 1/d³: a small step closer makes up
   for a factor of two.

2. Move the goal 1 m to the side and show that the minimum needs symmetry.

   ```bash
   python3 projects/p06_avoid-obstacles/arena2d.py --method apf --start -1 0 0 --goal 3 1
   ```

   Answer: `reached after 21.4 s, path 5.45 m, min clearance 0.63 m, end (2.82, 1.23)`. The pull now points at the upper end of
   the box, the push no longer cancels its sideways part, and the robot slides off the end. For comparison, the histogram from
   the original start, `--start -1 0 0 --goal 3 0` without `--method`, gives
   `reached after 19.5 s, path 5.54 m, min clearance 0.56 m, end (2.85, 0.24)`.

3. Show that the start heading picks the route. Run the histogram from B to A facing west and facing +25 degrees:

   ```bash
   python3 projects/p06_avoid-obstacles/arena2d.py --start 4 3 180 --goal -4.5 -3.5
   python3 projects/p06_avoid-obstacles/arena2d.py --start 3.75 2.9 25 --goal -4.5 -3.5
   ```

   Answer: `reached after 37.5 s, path 10.89 m` and `reached after 58.8 s, path 15.63 m`. The second matches the Gazebo runs
   `ba1` to `ba7` (15.73 to 16.57 m). Then do it in Gazebo: turn the robot on B to face west before a B-to-A leg (project 4's
   controller, or `ros2 topic pub` on `/cmd_vel` followed by a zero), and the route should be the 11 m one north of the brick box.
   I did not run this last part.

## If it goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `odom_err_end` of several tenths of a metre or more; the robot stops far from the goal or wanders near it and ends `STUCK` | The estimator had been running for minutes while the robot stood. In my first two runs it had run 7 and 11 minutes: `/odom` ended 0.25 m and 1.87 m from the truth (`results/stood_7min.csv`, `stood_11min.csv`). After about six minutes standing, `/odom` gave a heading of 18.4 degrees while the true heading was 2.3 degrees, and in the 7-minute run the estimated heading jumped 0.41 rad in 0.09 s as the gait started. Anchoring removes a constant offset, but the position estimate kept drifting during the run: in the 11-minute run the estimated direction of travel was 10 to 27 degrees off the estimated heading, while the true direction of travel stayed within 8 degrees of the true heading. My guess is that the lidar odometry's local map was built while the heading drifted, so later scans match a smeared map; I did not confirm this | Restart the estimator just before each trial. `avoid.launch.py` does this; with it, `odom_err_end` was 0.02 to 0.18 m in all 13 scored legs |
| The next trial starts far from A or B (`start at` line) and its numbers make no sense | The previous leg ended `STUCK` or `TIMEOUT` and left the robot where it gave up (my `ab5` after `ba4`) | Send the robot to the start with one more launch (`gx:=-4.5 gy:=-3.5`) and do not count that leg |
| The robot stops facing an obstacle and never turns, in the 2D model or in Gazebo | Potential field with a push too weak for the brake: the brake stops forward motion at `stop` before the push is strong enough to turn the robot. My first parameters (k_rep = 0.06, d0 = 1.2, stop = 0.55) stopped the model 0.55 m from the first pallet: `--method apf --set k_rep=0.06 d0=1.2 stop=0.55` gives `stuck after 39.8 s, path 2.24 m, ... end (-2.66, -2.26)` | Raise `k_rep` (1.0 is the default) or lower `stop`, and check in `arena2d.py` first |
| The robot turns left and right on the spot in open space | The histogram flips between two corridors of nearly equal cost; with `k_prev` = 0 my first Gazebo version did this and was declared stuck after 25 s | Keep `k_prev` above 0 (0.5 default) |
| The recorder writes only a few frames (`... .mp4: 5 frames`) | `projects/common/record_cam.py` subscribes best effort; on a loaded machine most 960 × 540 images were dropped | Use `projects/p05_sense-and-stop/record.py`, which subscribes reliable (627 frames for the same kind of run) |
| A `KeyboardInterrupt` traceback from `lidar_odom.py` and `process has died ... exit code -2` at the end of every trial | The launch shuts the estimator down with SIGINT when `avoid.py` exits, and `lidar_odom.py` does not catch it | Harmless: the trial's RESULT line is already printed and the log written |
| `UserWarning: Unable to import Axes3D` when running `analyze.py` | Two matplotlib installs on my machine (system and user); this script draws only 2D plots | Harmless |
| `avoid.py` logs its goal line and nothing more | No `/scan`, `/odom` or `/odom_gt` yet: the simulator is not up, or you ran `avoid.py` alone without the estimation stack | Use the launch file, or start `estimation.launch.py` yourself and wait for its `pose=` lines |
