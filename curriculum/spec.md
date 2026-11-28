# Practical task specification (owner's brief, 2026-11-28)

Third level of granularity: small, independently executable engineering tasks. Every task produces a visible, measurable result and prepares the robot for a later task.

```
Course > Part > Chapter > Practical task (required knowledge, implementation, experiment, acceptance test)
```

## Standard for every task

Objective (one capability) | Prerequisites (earlier task ids) | Theory (only the maths this task needs) | Implementation (code, nodes, config) | Execution (exact commands) | Demonstration (image, video or recorded output) | Measurement (numbers) | Acceptance (explicit pass/fail) | Failure analysis | Next task (how it is reused).

Status rules: a task is `verified` only when its acceptance test was run and passed; `failed` when run and not passed (keep the measured number and the reason); `spec` when not yet built. Reuse existing verified experiments where they fit. Never publish a `spec` task as a finished tutorial.

## Stage 1: import and inspect the robot
1.01 import the Go2 model (loads) | 1.02 display in RViz (all links) | 1.03 identify the 12 actuated joints (names, axes) | 1.04 move one joint in RViz (correct rotation) | 1.05 move the three joints of one leg (link motion) | 1.06 modify a joint limit (constraint appears) | 1.07 add a sensor link (TF connection) | 1.08 spawn in an empty Gazebo world | 1.09 create a floor and obstacles (collision geometry) | 1.10 read joint states (position, velocity, effort)

## Stage 2: joint and leg control
2.01 rotate a single hip joint (tracking error) | 2.02 move a joint between two angles (limits respected) | 2.03 control joint velocity | 2.04 apply controlled joint torque (dynamic response) | 2.05 PD controller (overshoot, settling) | 2.06 change gains (compare tracking) | 2.07 foot position from joint angles (FK accuracy) | 2.08 foot to a Cartesian position (IK accuracy) | 2.09 raise and lower one foot | 2.10 foot forward and backward | 2.11 foot along a triangle | 2.12 foot along an arc | 2.13 unreachable foot target (correct rejection) | 2.14 coordinate two legs on a supported robot (synchronised)

## Stage 3: posture and balance
3.01 resting configuration | 3.02 stand up (four-foot support) | 3.03 hold standing (no fall in test interval) | 3.04 sit or crouch (controlled transition) | 3.05 raise the body (height accuracy) | 3.06 lower the body (ground clearance) | 3.07 pitch the body (pitch tracking) | 3.08 roll the body (roll tracking) | 3.09 shift the centre of mass (support stability) | 3.10 lift one foot while balancing (three-leg stability) | 3.11 place the lifted foot at a new position | 3.12 return to the original posture (repeatability)

## Stage 4: basic locomotion
4.01 one footstep (stance-to-swing) | 4.02 one gait cycle (leg coordination) | 4.03 walk in place | 4.04 walk 0.5 m (distance, lateral deviation) | 4.05 walk 1 m | 4.06 walk backward | 4.07 walk sideways | 4.08 walk diagonally | 4.09 turn 45 deg | 4.10 turn 90 deg | 4.11 turn 180 deg | 4.12 walk a curve (path deviation) | 4.13 accelerate and decelerate | 4.14 different speeds (tracking error) | 4.15 walk-trot transition | 4.16 change step length (stride) | 4.17 change foot clearance (foot height) | 4.18 stop during locomotion (time, distance) | 4.19 keyboard control | 4.20 disconnect the publisher (automatic safe stop)

## Stage 5: terrain interaction and advanced locomotion (explicit branch, not hidden in advanced control)
5.01 slight incline | 5.02 uphill (pitch, tracking) | 5.03 downhill | 5.04 lateral slope (roll) | 5.05 one foot onto a low platform | 5.06 both front feet on the platform | 5.07 transfer weight forward (balance, contact forces) | 5.08 rear feet onto the platform (single-step ascent) | 5.09 descend one step | 5.10 step over a low obstacle | 5.11 cross a narrow gap | 5.12 uneven terrain (completion rate) | 5.13 reduced friction (slip, recovery) | 5.14 two consecutive steps | 5.15 staircase | 5.16 recover from a small push
Known terrain geometry first; perception-based terrain later (stage 11). Stair climbing cannot be assumed to work with CHAMP. Single step broken down: approach and stop at a controlled distance, raise a front foot on a reachable swing, place the front feet, transfer weight, bring the rear legs up. Measure: maximum successful step height, foot clearance, foot placement error, body roll and pitch, joint torque vs limits, slips, collisions, falls, ascent success rate. Start from a low step chosen from the measured reachable workspace; raise height only after the previous one passes.

## Stage 6: sensors and perception
6.01 stationary IMU | 6.02 rotate and measure angular rate | 6.03 IMU noise and bias | 6.04 integrate gyro for heading | 6.05 RGB camera | 6.06 depth camera | 6.07 distance to a wall | 6.08 lidar point cloud | 6.09 cloud into the robot frame | 6.10 remove floor and body returns | 6.11 detect one obstacle | 6.12 detect several | 6.13 stop before a wall | 6.14 stop before a new obstacle | 6.15 handle missing measurements | 6.16 detect a raised platform from depth | 6.17 estimate platform height and edges

## Stage 7: state estimation
7.01 rotation from the gyro | 7.02 displacement from commanded velocity | 7.03 base velocity from stance-leg kinematics | 7.04 effect of foot slip | 7.05 register two stationary scans | 7.06 translation between scans | 7.07 rotation between scans | 7.08 motion while walking | 7.09 continuous odometry | 7.10 simple Kalman filter | 7.11 fuse IMU and lidar | 7.12 publish odometry and TF | 7.13 compare with ground truth | 7.14 drift after returning to start | 7.15 diagnose failure in a feature-poor corridor

## Stage 8: feedback movement
8.01 reach a point with timed commands | 8.02 same point with odometry feedback | 8.03 target orientation | 8.04 target pose | 8.05 straight line | 8.06 square | 8.07 circle | 8.08 figure-eight | 8.09 waypoints | 8.10 correct after a disturbance | 8.11 return to start

## Stage 9: mapping and localization
9.01 map one wall | 9.02 map a room | 9.03 map a corridor | 9.04 map connected rooms | 9.05 loop closure on return | 9.06 save the map | 9.07 reload it | 9.08 localise from an approximate pose | 9.09 keep localisation while moving | 9.10 kidnapped robot | 9.11 compare map with the world

## Stage 10: autonomous navigation
10.01 A to B in an empty room | 10.02 around one obstacle | 10.03 several obstacles | 10.04 narrow passage | 10.05 doorway | 10.06 corner | 10.07 connected rooms | 10.08 replan when blocked | 10.09 reject an unreachable goal | 10.10 cancel a goal | 10.11 manual override | 10.12 waypoints | 10.13 patrol | 10.14 go and return | 10.15 repeat in a modified environment

## Stage 11: perceptive locomotion and terrain-aware navigation (extension beyond planar Nav2)
11.01 detect a step (RGB-D or lidar) | 11.02 traversable or not | 11.03 foothold estimate | 11.04 approach and align | 11.05 climb one step autonomously | 11.06 descend one step autonomously | 11.07 step over vs go around | 11.08 simple uneven surface | 11.09 traversability-aware route | 11.10 mixed-terrain environment
