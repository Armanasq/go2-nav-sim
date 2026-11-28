# Module 01 · Robot anatomy: bodies, joints, actuators and why the base is underactuated

## Objective

Read the Unitree Go2 model (`modules/assets/unitree_go2/go2.xml`, MuJoCo Menagerie) with our own parser and turn it into the
quantities every later module uses: the kinematic tree, the joint and actuator tables, the generalised coordinates, and the
actuation matrix. Then show with numbers why a legged robot is underactuated: twelve motors act on eighteen velocity directions,
and the six directions of the floating base can only be pushed by contact forces. The module has three levels:

| Level | Content | Applied on the Go2 |
|---|---|---|
| Fundamental | MJCF parsing with default classes; degrees of freedom; body, joint and actuator tables; kinematic tree | annotated front, side and top renders with every joint axis; turntable video |
| Intermediate | forward kinematics and velocity propagation down the tree; centre of mass and centroidal momentum; transmission and armature | torque needed to stand vs the motor limits; joint-space inertia with and without armature; joint-sweep video |
| Advanced | underactuation: the zero rows of $B$, momentum balance in flight and in stance, integrator drift, zero-momentum reorientation | flight with flailing legs; momentum balance during a squat; the body pitching 12.4° in zero gravity with zero angular momentum |

After this module a reader can count the degrees of freedom of any MJCF robot, write $\tau = Bu$ for it, compute its centre of
mass and centroidal momentum from the tree, and predict which motions the motors can produce without contact.

![The Go2 from the front, the right side and the top, orthographic, with every hinge axis](figures/views_annotated.png)

[Turntable video](media/turntable.mp4) (12 s): one turn with the visual meshes and MuJoCo's joint-axis arrows, one with the
collision geometry (red), the equivalent inertia boxes and the body centres of mass.

![Turntable still: visual meshes and joint axes](media/turntable.png)
![Turntable still: collision geometry and inertia boxes](media/turntable_collision_inertia.png)

## Prerequisites

[Module 00](../m00_math/README.md): rotation matrices, quaternions (MuJoCo order w x y z), Rodrigues' formula, the cross product
as a skew matrix. [Module 04](../m04_kinematics/README.md) works out one leg in detail; this module covers the whole tree and
reuses module 04's leg Jacobian only for the torque budget. Mechanics: Newton's second law for a particle and Euler's equation for a
rigid body. Python with NumPy; MuJoCo and Pinocchio are used only to check this code.

## Theory

Notation: world frame $W$ with $z$ up; base frame $B$ with $x$ forward, $y$ left. $R_i, p_i$ are the orientation and origin of
body $i$ in $W$; $m_i$, $c_i$ and $I_i$ its mass, centre of mass (world) and rotational inertia about $c_i$ (body frame).
$M = \sum_i m_i$ is the total mass and $c = \sum_i m_i c_i / M$ the centre of mass of the robot.

### Fundamental level

#### Reading an MJCF file

An MJCF body lists its offset in the parent (`pos`, `quat`), an `<inertial>` (mass, centre of mass, principal moments and their
orientation), its joints and geoms. Values that are not written on an element come from a default class. The resolution rule,
from the MuJoCo XML reference ("default" and "childclass"), is:

$$\text{attr}(e) = \text{element}(e) \;\triangleright\; \text{class}(e) \;\triangleright\; \text{parent classes of class}(e) \;\triangleright\; \text{main} \;\triangleright\; \text{built-in},$$

where $\triangleright$ means "overrides" and $\text{class}(e)$ is the element's `class` attribute, or else the `childclass` of the
nearest enclosing body. In go2.xml the base has `childclass="go2"`; the knee joint has `class="knee"`, which inherits `axis="0 1 0"`,
`damping="2"`, `armature="0.01"` and `frictionloss="0.2"` from `go2` and sets its own `range="-2.7227 -0.83776"`. The knee motor
gets `ctrlrange="-45.43 45.43"` from the same class; the hip and thigh motors get $\pm 23.7$ N m from `go2`. A `<freejoint>`
takes no defaults. With `autolimits="true"` a joint is limited if and only if it has a `range`.

The inertia tensor about the body's centre of mass, in the body frame, is rebuilt from the principal moments $d$ and the
inertial quaternion $q_I$:

$$I_i = R(q_I)\,\mathrm{diag}(d)\,R(q_I)^T .$$

MuJoCo normalises every quaternion on compile. The inertial quaternions in go2.xml are written to six digits; their norms differ
from 1 by up to $4 \times 10^{-7}$ (FL calf). Over the same 1000 random states, the centroidal momentum differs from Pinocchio's by
$6.5 \times 10^{-8}$ without the normalisation (`centroidal_momentum_vs_pinocchio_unnormalised_quat`, from
`validate.parse_unnormalised()`) and by $4.3 \times 10^{-14}$ with it (`centroidal_momentum_vs_pinocchio`).

#### Degrees of freedom

The configuration of a floating-base robot is the base pose plus the joint angles:

$$q = (p_B,\; \mathbf{q}_B,\; q_1, \dots, q_{12}) \in \mathbb{R}^3 \times S^3 \times \mathbb{R}^{12}, \qquad n_q = 3 + 4 + 12 = 19,$$

$$v = (v_B,\; \omega_B,\; \dot q_1, \dots, \dot q_{12}) \in \mathbb{R}^{18}, \qquad n_v = 3 + 3 + 12 = 18 .$$

The base orientation is stored as a unit quaternion (4 numbers, 1 constraint) but moves with 3 angular velocities, so
$n_q - n_v = 1$. Each MJCF joint type contributes $(n_q, n_v)$: free $(7, 6)$, ball $(4, 3)$, hinge and slide $(1, 1)$. MuJoCo's
velocity convention for the free joint is $v_B$ in the world frame and $\omega_B$ in the base frame (`sim.py` docstring).

For the Go2: 13 bodies, 1 free joint, 12 hinges and 12 motors.

| Body | Parent | Mass [kg] | Joint | Axis | Range [rad] | Range [deg] |
|---|---|---|---|---|---|---|
| base | world | 6.921 | free | (6 DoF) | none | none |
| FL/FR/RL/RR_hip | base | 0.678 | hip abduction | $x$ | $[-1.0472, 1.0472]$ | $[-60, 60]$ |
| FL/FR_thigh | hip | 1.152 | hip flexion | $y$ | $[-1.5708, 3.4907]$ | $[-90, 200]$ |
| RL/RR_thigh | hip | 1.152 | hip flexion | $y$ | $[-0.5236, 4.5379]$ | $[-30, 260]$ |
| FL/FR/RL/RR_calf | thigh | 0.241352 | knee | $y$ | $[-2.7227, -0.83776]$ | $[-156, -48]$ |

Total mass $M = 6.921 + 4\,(0.678 + 1.152 + 0.241352) = 15.2064$ kg (`total_mass_kg`); the legs carry 54.5% of it. Every hinge
has damping 2 N m s/rad, armature 0.01 kg m² and friction loss 0.2 N m (`tables.joints` in results.json).

| Actuator | Type | Transmission | Gear | Torque range [N m] |
|---|---|---|---|---|
| `<leg>_hip`, `<leg>_thigh` (8) | motor | joint `<leg>_hip_joint`, `<leg>_thigh_joint` | 1 | $\pm 23.7$ |
| `<leg>_calf` (4) | motor | joint `<leg>_calf_joint` | 1 | $\pm 45.43$ |

The figure below is drawn from `anatomy.parse()` output, not by hand.

![Kinematic tree generated from go2.xml](figures/kinematic_tree.png)

Offsets read from the file (body `pos`): hip abduction axes at $(\pm 0.1934, \pm 0.0465, 0)$ m in the base frame, so the front and
rear hips are 0.3868 m apart and the left and right abduction axes 0.093 m apart; the thigh joint is 0.0955 m further out along
$y$; thigh and calf are each 0.213 m long (calf: knee to foot-sphere centre, `pos="-0.002 0 -0.213"`); the foot is a sphere of
radius 0.022 m. All joints sit at their body origin ($\text{pos} = 0$).

#### Camera model for the annotated views

To draw joint axes onto a render, a world point $P$ has to be mapped to a pixel. MuJoCo's camera has origin $o$, unit vectors
$f$ (forward) and $u$ (up); image right is $r = f \times u$. With camera coordinates $X = r \cdot (P - o)$, $Y = u \cdot (P - o)$,
$Z = f \cdot (P - o)$, the pixel is

$$(x_{px}, y_{px}) = \left(\frac{X' - (c - w)}{2w}\,W,\;\; \frac{t - Y'}{t - b}\,H\right),\qquad (X', Y') = \begin{cases}(X, Y) & \text{orthographic}\\ (X, Y)\,n/Z & \text{perspective}\end{cases}$$

with frustum centre $c$, half-width $w$ (from the aspect ratio when MuJoCo stores 0), top $t$, bottom $b$ and near plane $n$.
MuJoCo renders a mono image from the average of its two eye cameras, which sit $\pm \text{ipd}/2 = \pm 0.034$ m apart. Taking
only `scene.camera[0]` instead of the average shifts every projected point by 31.4 px in each of the three views, against a
0.45 px bounding-box error with the average (`camera_eye0_only_shift_px`, `camera_projection_bbox_px`). The views here are
orthographic, so the shift is the same for every point and one scale bar holds for the whole image.

### Intermediate level

#### Forward kinematics of the tree

Bodies are stored parents first. For body $i$ with parent $\pi(i)$, fixed offset $(\bar p_i, \bar R_i)$ and hinge axis $s_i$
(body frame) through the anchor $a_i$ (body frame):

$$R_i = R_{\pi(i)} \bar R_i\, \mathrm{Rot}(s_i, q_i), \qquad p_i = p_{\pi(i)} + R_{\pi(i)} \bar p_i + R'_i a_i - R_i a_i,\quad R'_i = R_{\pi(i)}\bar R_i .$$

The joint's world anchor is $p_{\pi(i)} + R_{\pi(i)}\bar p_i + R'_i a_i$ and its world axis $\hat s_i = R'_i s_i$. For the base,
$p_B$ and $R(\mathbf q_B)$ come straight from $q$. This is the product of exponentials of Lynch and Park, ch. 4, written body by body.

#### Velocities down the tree

Differentiate $p_i$ and $R_i$. A hinge adds angular velocity $\hat s_i \dot q_i$, and a point $x$ on body $i$ gains velocity
$\hat s_i \dot q_i \times (x - \hat a_i)$ from it, with $\hat a_i$ the world anchor:

$$\omega_i = \omega_{\pi(i)} + \hat s_i \dot q_i, \qquad v_i = v_{\pi(i)} + \omega_{\pi(i)} \times (p_i - p_{\pi(i)}) + \hat s_i \dot q_i \times (p_i - \hat a_i),$$

starting from $v_B$ and $\omega_B^W = R_B\,\omega_B$. This is the outward velocity pass of Featherstone's recursive algorithms,
in 3-vectors instead of spatial vectors.

#### Centre of mass and centroidal momentum

With $c_i = p_i + R_i\,\bar c_i$ and $\dot c_i = v_i + \omega_i \times (c_i - p_i)$:

$$l = \sum_i m_i \dot c_i = M \dot c, \qquad k = \sum_i \left[ m_i (c_i - c) \times \dot c_i + R_i I_i R_i^T \omega_i \right].$$

$h_G = (l, k)$ is the centroidal momentum of Orin, Goswami and Lee (2013). It is linear in $v$: $h_G = A_G(q)\,v$.

#### Transmission and armature

A joint-transmission motor with gear $g_k$ on joint $j(k)$ produces the generalised force

$$\tau = B u, \qquad B_{n,k} = \begin{cases} g_k & n = \text{dof}(j(k)) \\ 0 & \text{otherwise}\end{cases},\qquad u_k \leftarrow \mathrm{clip}(u_k, u_k^{\min}, u_k^{\max}) .$$

For the Go2, $g_k = 1$ and actuator $k$ drives joint $k$, so $B = \begin{bmatrix} 0_{6 \times 12} \\ I_{12} \end{bmatrix}$: the
model's torques are joint-side torques and the gearbox is not modelled. Its only trace is the armature. A rotor of inertia $I_r$
turning at $N \dot q$ behind an $N{:}1$ reduction stores $\tfrac12 I_r (N \dot q)^2 = \tfrac12 (N^2 I_r)\,\dot q^2$, so it adds
$N^2 I_r$ to the joint's diagonal of the mass matrix:

$$M(q) = M_{\text{links}}(q) + \mathrm{diag}(0_6,\; I_a \mathbf 1_{12}), \qquad I_a = N^2 I_r = 0.01 \text{ kg m}^2 .$$

go2.xml gives $I_a$ but neither $N$ nor $I_r$. The knee limit is $45.43 / 23.7 = 1.92$ times the hip limit (my inference: an
extra reduction on the knee, not checked against a Unitree document).

Worked example at the stand pose ($q = (0, 0.9, -1.8)$ per leg, `stand_M_diag_*`): the link-only diagonal is 0.0236, 0.0204
and 0.00633 kg m² for hip, thigh and knee; armature raises it to 0.0336, 0.0304 and 0.0163. At the knee the rotor is 61% of the
inertia the motor sees.

#### Torque budget at the stand pose

Standing still under joint PD (`sim.stand`, 1.5 s settle, base height 0.256 m), the feet carry 33.4 N (front) and 41.2 N (rear),
which sum to $Mg = 149.2$ N. The motors hold this with (results.json `stand_torque_Nm`):

| Joint | Front legs [N m] | Rear legs [N m] | Limit [N m] | Largest share |
|---|---|---|---|---|
| hip abduction | 0.25 | 0.28 | 23.7 | 1.2% |
| hip flexion | 1.33 | 0.82 | 23.7 | 5.6% |
| knee | 5.94 | 7.06 | 45.43 | 15.5% |

From $\tau = J^T f$ with $f = (0, 0, f_z)$ (module 04's foot Jacobian $J$, base frame), the largest vertical force one foot can
hold at this pose before some joint saturates is

$$f_{z,\max} = \min_i \frac{\tau_{i,\max}}{|J_{zi}|} = 245.5 \text{ N (front)},\; 245.3 \text{ N (rear)} .$$

That is a static upper bound of 100 kg on all four feet, or 84.9 kg beyond the robot's own mass (`static_payload_kg`). It
ignores acceleration, motor heating, velocity limits (the model has none) and every other pose; it is a property of this model
at this pose, not a payload rating.

![Standing torque as a share of the limit, and joint-space inertia with and without armature](figures/torque_budget_and_inertia.png)

![Joint ranges in their planes of motion](figures/joint_ranges.png)

[Joint-sweep video](media/joint_sweep.mp4) (10.8 s, kinematic playback, base held 0.45 m up): all four legs sweep hip abduction
over its full range (left and right legs mirrored), then hip flexion over 80% of its range (the full range, -90° to 200° on the front legs, would swing the
thigh through the body), then the knee over its full range.

![Joint sweep still: thigh at 80% of its upper limit](media/joint_sweep.png)

### Advanced level: underactuation

#### Equations of motion, split into base and joint rows

With contact forces $f_c$ at points with Jacobian $J_c$ (stacked, $3n_c \times 18$):

$$M(q)\,\dot v + h(q, v) = B u + J_c^T f_c .$$

Split $v = (v_b, \dot{\mathbf q})$ with $v_b \in \mathbb R^6$:

$$\begin{bmatrix} M_{bb} & M_{bj} \\ M_{jb} & M_{jj} \end{bmatrix}\begin{bmatrix}\dot v_b \\ \ddot{\mathbf q}\end{bmatrix} + \begin{bmatrix} h_b \\ h_j\end{bmatrix} = \begin{bmatrix} 0 \\ u \end{bmatrix} + \begin{bmatrix} J_{c,b}^T \\ J_{c,j}^T\end{bmatrix} f_c .$$

$B$ has rank 12 and 18 rows, so six directions of $\dot v$ receive no actuator force. Those six rows are the base rows, and $u$
does not appear in them.

![B and the mass matrix: the base rows are empty in B and full in M](figures/selection_and_mass_matrix.png)

#### What the base rows say

Write Newton's law for every body and add them up. Joint torques, joint damping, friction loss and joint-limit forces each act
on a parent and a child with equal and opposite force and moment (Newton's third law), so they cancel in the sum. Gravity acts at
each $c_i$ and contacts with the world act at the contact points $p_k$ with force $f_k$ and, for MuJoCo's condim-6 feet,
torsional and rolling moments $\tau_k$:

$$\dot l = M g + \sum_k f_k, \qquad \dot k = \sum_k \left[(p_k - c) \times f_k + \tau_k\right] .$$

Gravity contributes no moment about $c$ because $\sum_i m_i (c_i - c) \times g = 0$ by the definition of $c$. The base rows of the
equations of motion carry exactly this information, expressed at the base instead of at $c$ (Orin, Goswami and Lee 2013).
Two consequences:

1. In flight ($f_c = 0$), $\ddot c = g$ and $k$ is constant whatever the motors do. The motors can still rotate and shift the
   base, because $M_{bj} \ddot{\mathbf q} \neq 0$: the base moves so that the total momentum does not.
2. To accelerate the centre of mass or change $k$, the robot must push on the ground. Locomotion is the problem of choosing joint
   torques so that the resulting contact forces produce the wanted $\dot h_G$, inside the friction cones.

![Block diagram: the motors reach only the joint rows](figures/underactuation_diagram.png)

#### Discrete time does not conserve momentum exactly

Semi-implicit Euler updates $v_{k+1} = v_k + \Delta t\,\dot v_k$, then $q_{k+1} = q_k \oplus \Delta t\, v_{k+1}$. MuJoCo's
`mjINT_EULER` treats joint damping implicitly: it solves $(M + \Delta t\,D)\,\dot v_k = f(q_k, v_k)$, so that
$M\dot v_k = f_{\text{other}} - D\,v_{k+1}$, with $D$ = diag(0, ..., 0, 2, ..., 2) N m s/rad (zero on the six base rows, 2 on every
hinge). The base rows of the equations of motion, which alone fix $\dot h_G$, are therefore the same as without the implicit
term, and the argument below is unchanged. Differentiating
$h_G = A_G(q)\,v$ gives the continuous balance $A_G(q)\,\dot v + \dot A_G(q, v)\,v = w$, with $w$ the external wrench
($(Mg, 0)$ in flight). Suppose the integrator's $\dot v_k$ satisfies it exactly at step $k$:
$A_G(q_k)\dot v_k + \dot A_G(q_k, v_k)\,v_k = w_k$. With $h_k = A_G(q_k)\,v_k$ the momentum changes per step by

$$h_{k+1} - h_k = \Delta t\, A_G(q_k)\dot v_k + \big(A_G(q_{k+1}) - A_G(q_k)\big)\, v_{k+1} = \Delta t\, w_k + \underbrace{\big(A_G(q_{k+1}) - A_G(q_k)\big)\, v_{k+1} - \Delta t\,\dot A_G(q_k, v_k)\, v_k}_{\text{residual}} .$$

$\dot A_G(q, v) = \sum_n \frac{\partial A_G}{\partial q_n} v_n$ is linear in $v$, and a Taylor expansion of $A_G$ about $q_k$ gives
$A_G(q_{k+1}) - A_G(q_k) = \Delta t\,\dot A_G(q_k, v_{k+1}) + O(\Delta t^2)$. The residual is therefore

$$\Delta t\,\big[\dot A_G(q_k, v_{k+1})\,v_{k+1} - \dot A_G(q_k, v_k)\,v_k\big] + O(\Delta t^2) = O(\Delta t^2),$$

because $v_{k+1} - v_k = \Delta t\,\dot v_k$. Over a fixed time $T$ the $T/\Delta t$ residuals add up to $O(\Delta t)$, so halving
$\Delta t$ should halve Euler's drift.

#### Reorientation with zero angular momentum

With $k = 0$ the base angular velocity is fixed by the joint velocities: $k = I_c(q)\,\omega_B + A_{kq}(q)\,\dot{\mathbf q} = 0$
gives $\omega_B = -I_c^{-1} A_{kq}\,\dot{\mathbf q}$. For a motion confined to the sagittal plane (all legs alike), only pitch
$\theta$ changes and this reduces to $\dot\theta = a(\mathbf q)^T \dot{\mathbf q}$ with $\mathbf q = (q_{\text{thigh}}, q_{\text{knee}})$.
Over a closed loop $\gamma$ in joint space, Stokes' theorem gives

$$\Delta\theta = \oint_\gamma a(\mathbf q)\cdot d\mathbf q = \iint_{\text{inside }\gamma} \left(\frac{\partial a_2}{\partial q_1} - \frac{\partial a_1}{\partial q_2}\right) dq_1\,dq_2 \neq 0 \text{ in general.}$$

Three predictions follow: the net rotation does not depend on how fast the loop is run; running the loop backwards reverses it;
a loop around a different region gives a different rotation. This is the falling-cat mechanism (Kane and Scher 1969).

## Implementation

| Equation | Function | Notes |
|---|---|---|
| default-class resolution, inertia tensor, quaternion normalisation | `anatomy.parse()` | ElementTree only; asserts joint transmissions and hinge gears with zero tail |
| $n_q$, $n_v$, rank of $B$ | `anatomy.dof()` | |
| $\tau = Bu$ with clipping | `anatomy.selection()`, `anatomy.actuator_force()` | |
| forward kinematics, anchors, axes | `anatomy.fk()` | general anchor $a_i$, not assumed zero |
| velocity propagation | `anatomy.velocities()` | |
| $M$, $c$, $l$, $k$ | `anatomy.centroidal()` | |
| $\dot h_G$ along the motion | `anatomy.momentum_rate()` | central difference in time, $\epsilon = 10^{-6}$ s: moves $q$ with `anatomy.integrate()` and $v$ by $\epsilon\dot v$ |
| external wrench | `anatomy.external_rate()` | gravity plus contact forces and moments |
| camera projection | `validate.project()`, `validate.mono_camera()` | orthographic and perspective |

Design choices:

- `anatomy.py` imports neither MuJoCo nor Pinocchio. The tests compare it with `mj_forward` (`xpos`, `xmat`, `xanchor`, `xaxis`),
  `mj_objectVelocity`, `qfrc_actuator`, `mj_fullM` and with Pinocchio's `computeCentroidalMomentum`,
  `computeCentroidalMomentumTimeVariation` and model limits.
- $\dot h_G$ is a finite difference of $h_G$ rather than an analytic derivative. The analytic one needs the bias accelerations of
  every body; the difference reuses `centroidal()` and is accurate to about $10^{-8}$, enough to separate "equals $Mg$" from
  "differs by newtons". Pinocchio's analytic $\dot h_G$ is the tighter check ($2.8 \times 10^{-11}$).
- The momentum checks set MuJoCo's constraint solver to tolerance 0 and 1000 iterations. At the default tolerance
  ($10^{-8}$, 100 iterations) the solver stops before $M\dot v = \tau + J^T f$ holds to machine precision (Failure analysis).
- Flight and reorientation use RK4. The flight experiment launches the robot with $v = (1.2, 0, 4.5)$ m/s from 0.6 m and stops
  after 0.9 s, before it lands: no floor contact in any run (`*_floor_contacts = 0`).
- `validate.Rec` extends `sim.Recorder` with slow motion and marker spheres, using the same encoder settings and limits.

## Experiment

```bash
DISPLAY=:1 .venv-modules/bin/python modules/m01_anatomy/validate.py      # wall time depends on machine load: 174 s at load 127 (wall_time_s)
.venv-modules/bin/python -m pytest -q modules/m01_anatomy
```

| Experiment | Inputs | Measured |
|---|---|---|
| parse vs compiled models | go2.xml | counts; every body, joint and actuator field |
| camera model | 3 orthographic views (960x720), robot lifted 0.4 m so the floor hides nothing | bounding box of 419,023 projected mesh vertices vs the segmentation silhouette |
| kinematics, momentum, transmission | 1000 random states, seed 0: base pose $\mathcal N(0,1)$, joints $U(-\pi, \pi)$, $v \sim \mathcal N(0, 1)$, $u \sim U(-1.5, 1.5)\,u_{\max}$ | max abs error vs MuJoCo and Pinocchio |
| armature | 50 random configurations, model with armature set to 0 | $M - M_0 - \mathrm{diag}(0_6, 0.01\cdot\mathbf 1_{12})$ |
| stand | `sim.stand`, 1.5 s | actuator torques, foot forces |
| flight identity | 1000 random states (seed 1) and torques $u \sim U(-1, 1)\,u_{\max}$, no floor | $\dot h_G - (Mg, 0)$ |
| stance identity | squat: min-jerk PD (kp 80, kd 2) from (0, 0.9, -1.8) to (0, 1.25, -2.5) and back, 1.8 s, sampled every 10 ms (180 samples) | $\dot h_G -$ (gravity + contact wrenches) |
| drift | flight 0.9 s, torques $15 \sin(2\pi f_k t + k)$ N m, $f_k \in [0.5, 1.7]$ Hz; Euler and RK4 at 2 and 1 ms | $\max_t |k(t) - k(0)|$, $\max_t |l - l_0 - Mgt|$ |
| reorientation | zero gravity, RK4, PD kp 400, kd 6 on the loop below, 5 cycles at 1 Hz, reversed, at 0.5 Hz, and over a second region of the same area | base pitch, $|k|$, $|l|$ |

The reorientation loop for every leg is $q_{\text{thigh}} = 0.9 + s\,0.45 \sin \omega t$, $q_{\text{knee}} = -1.8 + 0.45(\cos\omega t - 1)$:
a circle of radius 0.45 rad through the stand pose, with $s = -1$ for the reversed run.

## Validation

Thresholds were fixed in `validate.ACCEPT` before the first full run; `test_anatomy.py` asserts the same ones.

| Check | Measured | Acceptance | Source (`results.json`) |
|---|---|---|---|
| counts $n_q, n_v, n_u$, bodies, joints: parsed vs MuJoCo vs Pinocchio | 0 mismatches (19, 18, 12, 13, 13; Pinocchio has no actuators, so $n_u$ is compared with MuJoCo only) | 0 | `count_mismatches_vs_mujoco_pinocchio` |
| body offsets, masses, COM, inertia tensors vs MuJoCo, max abs | 0 | < 1e-12 | `parse_bodies_vs_mujoco` |
| joint type, axis, range, damping, armature, friction loss, addresses vs MuJoCo | 0 | < 1e-12 | `parse_joints_vs_mujoco` |
| actuator joint, gear, ctrlrange vs MuJoCo | 0 | < 1e-12 | `parse_actuators_vs_mujoco` |
| total mass vs MuJoCo `body_subtreemass` | 0 kg (15.2064 kg) | < 1e-12 | `total_mass_vs_mujoco_kg` |
| joint ranges vs Pinocchio | 0 rad | < 1e-12 | `joint_limits_vs_pinocchio` |
| armature and damping vs Pinocchio | 0 | < 1e-12 | `armature_damping_vs_pinocchio` |
| projected bounding box vs rendered silhouette, max over 3 views | 0.45 px | ≤ 1.5 px | `camera_projection_bbox_px` |
| body poses vs MuJoCo, 1000 states | 8.9e-16 | < 1e-12 | `fk_body_pose_vs_mujoco` |
| joint anchors and axes vs MuJoCo | 1.2e-15 | < 1e-12 | `fk_joint_anchor_axis_vs_mujoco` |
| body angular and linear velocity vs `mj_objectVelocity` | 2.7e-15 | < 1e-12 | `body_velocity_vs_mujoco` |
| centre of mass vs Pinocchio | 8.9e-16 m | < 1e-12 | `com_vs_pinocchio_m` |
| centroidal momentum vs Pinocchio | 4.3e-14 | < 1e-12 | `centroidal_momentum_vs_pinocchio` |
| $B\,\mathrm{clip}(u)$ vs `qfrc_actuator`, half the commands beyond the range | 0 N m | < 1e-12 | `actuator_force_vs_mujoco` |
| $M - M_0$ equals diag(armature), 50 configurations | 1.7e-18 | < 1e-12 | `armature_in_mass_matrix` |
| largest standing torque / limit | 0.155 (rear knee) | < 0.5 | `stand_torque_utilisation_max` |
| base rows of $B$ | 0 | ≤ 0 | `B_base_rows_max_abs` |
| base rows of MuJoCo's actuator force, 1000 random commands | 0 | ≤ 0 | `qfrc_actuator_base_rows_max_abs` |
| rank of $B$ | 12 | = 12 | `rank_B` |
| flight: $\dot h_G - (Mg, 0)$, finite difference, max | 1.8e-8 | < 1e-6 | `flight_momentum_rate_vs_Mg_fd` |
| flight: same, Pinocchio's $\dot h_G$ | 2.8e-11 | < 1e-8 | `flight_momentum_rate_vs_Mg_pinocchio` |
| stance: $\dot h_G -$ (gravity + contact wrenches), 180 samples | 1.3e-9 | < 1e-5 | `stance_momentum_rate_vs_wrench` |
| Euler angular-momentum drift, 2 ms / 1 ms | 2.00 | 1.6 to 2.4 | `flight_drift_order_euler` |
| RK4 angular-momentum drift at 2 ms | 1.9e-6 kg m²/s | < 1e-4 | `flight_drift_rk4_ang_kgm2s` |
| pitch change after 5 loops | 12.4° | > 5° | `reorient_pitch_5_cycles_deg` |
| largest $|k|$ in the four reorientation runs | 1.5e-7 kg m²/s | < 1e-4 | `reorient_max_ang_momentum` |
| reversed loop / forward loop rotation | -1.002 | -1.1 to -0.9 | `reorient_reverse_ratio` |
| 0.5 Hz / 1 Hz rotation | 1.030 | 0.9 to 1.1 | `reorient_half_speed_ratio` |
| other region / forward loop rotation, $|$ratio $- 1|$ | 0.194 | > 0.1 | `reorient_other_region_rel_diff` |
| all MP4 files < 6 MB | 4 files, largest 2.3 MB | all | `media_mp4_under_6MB` |

All 30 checks pass in the run that wrote results.json. The last row was added after an independent review pointed out that
the third Stokes prediction had no test; its threshold, three times the 3% speed effect, was set before the run that measured it.

### Flight: the motors turn the base but cannot move the centre of mass

Over 1000 random states the joint torques change the base's angular acceleration by a median of 112 rad/s² (difference between
$u$ and $u = 0$, `flight_torque_effect_on_base_ang_acc_median`), while the centre-of-mass acceleration differs from $g$ by at most
$1.1 \times 10^{-9}$ m/s² (`flight_com_acc_minus_g_max`). In the 0.9 s flight with flailing legs the base pitches over a range of
8.9° and its origin leaves the ballistic parabola by up to 3.4 cm; the centre of mass stays on the parabola to $3.9 \times 10^{-7}$ m
(`flight_*` keys).

![Flight: COM on the parabola, base attitude moving, momentum change](figures/flight_momentum.png)

[Flight video](media/flight.mp4) (3.6 s, quarter speed): the robot is drawn semi-transparent so that the centre of mass, the
blue sphere inside the base (the colour of the simulated COM in the figure above), can be seen through it; the amber dots are
the parabola it has to follow.

![Flight still at t = 0.45 s](media/flight.png)

### Stance: the contacts carry the weight and the accelerations

During the squat the summed vertical contact force ranges from 132.7 to 168.7 N around $Mg = 149.2$ N. The momentum rate equals
gravity plus the contact wrenches to $1.3 \times 10^{-9}$; leaving the contacts out gives an error of at least 132.7 N at every
sample (`stance_error_without_contacts_min_N`).

![Momentum balance during a squat](figures/stance_momentum_balance.png)

### Reorientation without angular momentum

![Pitch with zero angular momentum: forward, reversed, half-speed and other-region loops](figures/zero_momentum_reorientation.png)

All three predictions of the Stokes argument hold. The reversed loop gives +12.45° against -12.43° (ratio -1.002), and running it
at half speed gives -12.80° (ratio 1.03, the 3% being the tracking error, see below). The point reflection of the circle
through the stand pose, $q_{\text{thigh}} = 0.9 - 0.45 \sin \omega t$, $q_{\text{knee}} = -1.8 - 0.45(\cos\omega t - 1)$, has the
same area and direction but covers the region where the knee is bent less (measured knee range -1.80 to -0.90 rad,
`reorient_other_region_knee_range_rad`, inside the limit of -0.83776 rad). It turns the base by -14.84°, ratio 1.194 to the
first loop (`reorient_other_region_ratio`), with a tracking error of 0.057 rad, the same as the first loop's. Area alone does
not set the rotation; the integrand $\partial a_2/\partial q_1 - \partial a_1/\partial q_2$ varies over joint space. $|k|$ stays below $1.5 \times 10^{-7}$ kg m²/s.
Within each 1 Hz cycle the pitch swings over a peak-to-peak range of 4.94° and ends 2.05° to 2.85° below where the cycle
started (`reorient_pitch_swing_per_cycle_deg`, `reorient_pitch_net_per_cycle_deg`).

[Reorientation video](media/reorientation.mp4) (5 s, real time, zero gravity, camera follows the base position).

![Reorientation still after 4 loops](media/reorientation.png)

## Failure analysis

### Euler drifts, and RK4 is not fourth order here

![Momentum drift in flight for Euler and RK4 at 2 ms and 1 ms](figures/integrator_drift.png)

The continuous model conserves $k$ in flight; MuJoCo's default integrator does not. Over 0.9 s, semi-implicit Euler at 2 ms
drifts by 0.0107 kg m²/s in angular and 0.050 kg m/s in linear momentum; at 1 ms by 0.0054 and 0.025 (`euler_*`). The ratio 2.00
matches the first-order prediction. For scale (my estimate, not measured): the whole robot pitching at 1 rad/s has $|k|$ of
roughly 0.5 kg m²/s, so Euler's drift over 0.9 s is about 2% of that. RK4 drifts by $1.9 \times 10^{-6}$ (2 ms) and
$8.0 \times 10^{-7}$ (1 ms), a ratio of 2.4 rather than the 16 of a fourth-order method (`flight_drift_order_rk4`). Both RK4 curves grow in steps rather
than smoothly (left panel): at 2 ms one step near 0.16 s, at 1 ms steps near 0.22, 0.25, 0.28 and 0.32 s, and a small step near
0.77 s in both. The step instants differ between the two step sizes, and the Euler curves show bumps near 0.18 and 0.78 s. A possible cause, not tested: events in the non-smooth constraints (the 0.2 N m joint friction loss,
joint limits, self-contacts) that fall inside a step at different places for different step sizes; a non-smooth right-hand
side would limit RK4's order. Logging the active constraint set per step would test this. Any estimator
or controller module that reads momentum from simulation should use RK4 or a 1 ms step, or expect errors of this size.

### The momentum identity holds only to the solver tolerance

At MuJoCo's default solver settings (tolerance $10^{-8}$, 100 iterations) the flight identity $\dot h_G = Mg$ is off by
$7.2 \times 10^{-5}$ (`flight_momentum_rate_default_solver_tol`); with tolerance 0 and 1000 iterations by $1.8 \times 10^{-8}$. The
cause: MuJoCo computes $\dot v$ inside its constraint solver (friction loss, joint limits and self-contacts are all constraints,
even in flight), and stops when the cost improvement falls below the tolerance, so $M\dot v = \tau + J^T f$ is only approximately
met. The residual $M\dot v - \tau_{\text{smooth}} - J^T f$ over the same 1000 states is at most $7.2 \times 10^{-5}$ in the base
rows, which is the momentum error above, and 0.134 N m in the joint rows; with tolerance 0 it drops to $2.7 \times 10^{-11}$ and
$1.8 \times 10^{-10}$ (`flight_eom_residual_*`). The joint-row residual does not enter $\dot h_G$, which is fixed by the base
rows of the equations of motion alone.

### Reorientation depends on the loop the joints actually trace

![Low-gain tracking shrinks the loop and makes the rotation speed dependent](figures/reorientation_tracking_failure.png)

An earlier full run, with PD gains kp 60, kd 2 and loop amplitude 0.5 rad, missed `reorient_half_speed_ratio` with 1.175
(threshold 0.9 to 1.1). That run's results were not kept, so 1.175 cannot be checked against results.json. To show the cause, validate.py reruns the low
gains on the current loop (amplitude 0.45 rad). With those gains the joints lag the command by up to 0.18 rad at 1 Hz and
0.10 rad at 0.5 Hz (`reorient_lowgain_track_err_rad`, `reorient_lowgain_half_speed_track_err_rad`); the traced loop is smaller
at 1 Hz, so it encloses less area and rotates the body less: -9.87° vs -11.75° over 5 cycles, ratio 1.19
(`reorient_lowgain_half_speed_ratio`). The rotation follows the loop the joints execute, as the Stokes argument predicts, and
the commanded loop does not enter it. With kp 400, kd 6 the tracking error is 0.057 rad at 1 Hz and 0.029 rad at 0.5 Hz, and
the ratio is 1.03. The threshold was not changed. results.json keeps the kp 400 runs and the low-gain rerun, not the earlier
run.

The same earlier run (not kept either) also reported a reversal ratio of -1.44. The "reversed" loop then flipped the sign of the knee term, which
traces a different circle (knee bent less instead of more), not the same circle backwards; the region differed, so the rotation
differed. Its amplitude of 0.5 rad also drove the knee to -2.8 rad, past the -2.7227 rad limit. The loop now reverses the thigh
term and uses 0.45 rad.

### Pinocchio's MJCF import drops the torque limits

Pinocchio 4.1 reads go2.xml's joint ranges, armature and damping exactly (0 difference), but all 18 entries of
`model.effortLimit` are infinite (`pinocchio_effort_limits_finite = 0`): the actuator `ctrlrange` is not imported. A controller
that takes its torque limits from a Pinocchio model built from this file gets no limits.

### What the model leaves out

The MJCF has no motor velocity limit, no torque-speed curve, no gear ratio and no motor electrical model; it applies the commanded
torque at the joint instantly. The static payload bound above (84.9 kg) is a consequence: only the torque limits constrain it.
The foot contact is a soft sphere that sinks about 13 mm into the floor at rest (MODULE_SPEC.md); the collision geometry is boxes,
cylinders and spheres (turntable, second turn), so self-collision during the joint sweep is approximate.

## References

- MuJoCo documentation, XML reference: `default` and `childclass`, `joint` (`armature`, `damping`, `frictionloss`, `range`),
  `freejoint`, `motor`; Computation chapter: constraint solver and integrators. mujoco.readthedocs.io, version 3.x.
- E. Todorov, T. Erez, Y. Tassa, "MuJoCo: A physics engine for model-based control", IROS 2012.
- K. Zakka et al., MuJoCo Menagerie, github.com/google-deepmind/mujoco_menagerie (unitree_go2 model).
- J. Carpentier et al., "The Pinocchio C++ library", IEEE/SICE SII 2019.
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer 2008: kinematic trees stored as parent arrays, outward velocity
  recursion.
- K. M. Lynch, F. C. Park, *Modern Robotics*, Cambridge University Press 2017: ch. 4 (product-of-exponentials forward kinematics).
- D. E. Orin, A. Goswami, S.-H. Lee, "Centroidal dynamics of a humanoid robot", Autonomous Robots 35, 2013.
- T. R. Kane, M. P. Scher, "A dynamical explanation of the falling cat phenomenon", Int. J. Solids and Structures 5, 1969.
