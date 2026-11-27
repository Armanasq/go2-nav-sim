# Modules

The roadmap's first-principles track: each module solves one problem on the Go2 model in MuJoCo, with its own code, an
experiment, measured validation against an established library, and an account of what failed. The ROS 2 / Gazebo stack in the rest of the repository covers the system-level modules (sensors,
estimation, mapping, navigation, validation).

```bash
modules/setup.sh                                   # venv with MuJoCo + Pinocchio, Go2 MJCF from MuJoCo Menagerie (pinned)
.venv-modules/bin/python -m pytest -q modules      # all module tests
```

| Module | Status | What is validated |
|---|---|---|
| [00 · Mathematical foundations](m00_math/README.md) | partial | quaternion, roll-pitch-yaw, SO(3) exp/log and SE(3) inverse equal scipy and Pinocchio to ≤ 8e-15 (log vs Pinocchio 5.7e-11 near π); yaw rate from body gyro matches Pinocchio's RPY Jacobian to 3e-15. SO(3) right/left Jacobians match central differences to 5e-10. Gauss-Newton and Levenberg-Marquardt reach the closed-form SE(2) registration to 9e-15 / 6e-12 from every start on a 5° heading grid; Cauchy IRLS with 20% outliers leaves 0.046° and 1.1 mm. FL leg calibration (3 encoder offsets, 2 link lengths) equals scipy to 4e-12, mean NEES 4.64 for 5 parameters. 29 of 32 checks pass; misses: `jr_vs_pinocchio_Jexp3` 5.2e-14 (> 1e-14), `jr_inv_vs_pinocchio_Jlog3` 3.8e-10 (> 1e-10), `registration_gn_max_iters` 17 (> 10, from −145°) |
| [04 · Kinematics](m04_kinematics/README.md) | validated | FK and Jacobian equal MuJoCo and Pinocchio to ~1e-16; multi-start IK converges on 99.3–99.8% of reachable targets; closed-form IK recovers 100% to 7e-14 rad; τ = Jᵀf equals a MuJoCo standing equilibrium to 2.4e-6 N m |

Every module README follows the same order: objective, prerequisites, theory, implementation, experiment, validation, failure
analysis, references. The full roadmap with all 18 modules and their status is on the website.
