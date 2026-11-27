# Module 00 · Rotations and rigid transforms

## Objective

Derive and implement the rotation and transform operations the rest of the stack uses (Rodrigues' formula, quaternions,
roll-pitch-yaw angles, SE(2) and SE(3), the SO(3) exponential and logarithm), then work out three results that this robot
depends on: yaw rate from the body gyro, the camera optical frame, and the height error caused by an uncorrected trunk tilt.
Every function is checked against `scipy.spatial.transform.Rotation` and Pinocchio.

## Prerequisites

Linear algebra: matrix products, orthogonal matrices, the cross product. Python with NumPy; scipy and Pinocchio only for
checking.

## Theory

Notation: $[a]$ is the skew matrix with $[a]v = a \times v$; $c_\alpha = \cos\alpha$, $s_\alpha = \sin\alpha$. Frames are
right-handed. Roll-pitch-yaw $(\phi, \theta, \psi)$ means $R = R_z(\psi) R_y(\theta) R_x(\phi)$, the URDF and Pinocchio convention.

### Rodrigues' formula

Rotate $v$ by $\theta$ about the unit axis $a$. Split $v$ into the part along the axis, which does not move, and the part
perpendicular to it, which turns by $\theta$ in the plane spanned by $v_\perp$ and $a \times v$:

$$v = v_\parallel + v_\perp,\qquad v_\parallel = (a \cdot v)\,a,\qquad v' = v_\parallel + \cos\theta\, v_\perp + \sin\theta\,(a \times v).$$

With $[a]v = a \times v$ and $[a]^2 v = a (a\cdot v) - v = -v_\perp$, substitute $v_\perp = -[a]^2 v$ and $v_\parallel = v + [a]^2 v$:

$$v' = v + [a]^2 v - \cos\theta\,[a]^2 v + \sin\theta\,[a] v \quad\Rightarrow\quad R = I + \sin\theta\,[a] + (1-\cos\theta)\,[a]^2 .$$

### Quaternion to matrix and back

A unit quaternion $q = (w, \mathbf v)$ with $w = \cos\frac\theta2$, $\mathbf v = \sin\frac\theta2\, a$ encodes the same rotation.
Use $\sin\theta = 2\sin\frac\theta2\cos\frac\theta2$ and $1-\cos\theta = 2\sin^2\frac\theta2$ in Rodrigues' formula:

$$\sin\theta\,[a] = 2w[\mathbf v],\qquad (1-\cos\theta)[a]^2 = 2[\mathbf v]^2 \quad\Rightarrow\quad R = I + 2w[\mathbf v] + 2[\mathbf v]^2 .$$

Expanding with $[\mathbf v]^2 = \mathbf v\mathbf v^\top - \lVert\mathbf v\rVert^2 I$ gives the familiar entries, for example
$R_{00} = 1 - 2(y^2+z^2)$ and $R_{01} = 2(xy - wz)$. For the inverse, read off the trace, the skew part and the symmetric part:

$$\operatorname{tr} R = 3 - 4\lVert\mathbf v\rVert^2 = 4w^2 - 1,\qquad R - R^\top = 4w[\mathbf v],\qquad R_{ii} = 2v_i^2 + 2w^2 - 1,\qquad R_{ij} + R_{ji} = 4 v_i v_j .$$

So $4w^2 = 1 + \operatorname{tr}R$ and $4v_i^2 = 1 + 2R_{ii} - \operatorname{tr}R$. Shepperd's method takes the largest of these four,
which is at least 1 since they sum to 4, and gets the other three components by dividing a skew or symmetric entry by it.
Dividing only by $w$ fails at $\theta = \pi$, where $w = 0$.

### Roll, pitch, yaw from the matrix

Multiplying out $R_z(\psi)R_y(\theta)R_x(\phi)$:

$$R = \begin{bmatrix} c_\psi c_\theta & c_\psi s_\theta s_\phi - s_\psi c_\phi & c_\psi s_\theta c_\phi + s_\psi s_\phi \\
s_\psi c_\theta & s_\psi s_\theta s_\phi + c_\psi c_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi \\
-s_\theta & c_\theta s_\phi & c_\theta c_\phi \end{bmatrix}.$$

The bottom row and first column give the angles when $c_\theta \neq 0$:

$$\theta = \operatorname{atan2}\!\big(-R_{20},\ \sqrt{R_{00}^2 + R_{10}^2}\big),\qquad \phi = \operatorname{atan2}(R_{21}, R_{22}),\qquad \psi = \operatorname{atan2}(R_{10}, R_{00}).$$

At $\theta = \pm\pi/2$ (gimbal lock) the first column and bottom row reduce to $(0, 0, \mp 1)$ and the remaining entries depend on one
angle combination only. For $\theta = +\pi/2$: $R_{01} = \sin(\phi-\psi)$, $R_{11} = \cos(\phi-\psi)$. For $\theta = -\pi/2$:
$R_{01} = -\sin(\phi+\psi)$, $R_{11} = \cos(\phi+\psi)$. Only $\phi \mp \psi$ is observable; the code sets $\psi = 0$ and
returns $\phi = \operatorname{atan2}(-R_{20}R_{01},\ R_{11})$, which covers both signs.

### SE(2) and SE(3)

A rigid transform $T = \begin{bmatrix} R & t \\ 0 & 1\end{bmatrix}$ maps $p \mapsto Rp + t$. Composition and inverse follow from
applying the maps in turn and solving $p' = Rp + t$ for $p$:

$$T_a T_b = \begin{bmatrix} R_a R_b & R_a t_b + t_a \\ 0 & 1 \end{bmatrix},\qquad T^{-1} = \begin{bmatrix} R^\top & -R^\top t \\ 0 & 1\end{bmatrix}.$$

In SE(2), with poses $(x, y, \theta)$ and $R(\theta)$ the planar rotation, this is
$(x_a + c_a x_b - s_a y_b,\ y_a + s_a x_b + c_a y_b,\ \theta_a + \theta_b)$ and
$(x,y,\theta)^{-1} = (-c\,x - s\,y,\ s\,x - c\,y,\ -\theta)$.

### Exponential and logarithm of SO(3)

For a rotation vector $\omega = \theta a$, $\exp[\omega] = \sum_k [\omega]^k / k!$. Because $[a]^3 = -[a]$, the odd powers collapse
onto $[a]$ and the even ones onto $[a]^2$:

$$\exp[\omega] = I + \Big(\theta - \tfrac{\theta^3}{3!} + \dots\Big)[a] + \Big(\tfrac{\theta^2}{2!} - \tfrac{\theta^4}{4!} + \dots\Big)[a]^2 = I + \sin\theta\,[a] + (1-\cos\theta)[a]^2,$$

which is Rodrigues' formula again. For the logarithm take the trace and the skew part of that expression,
$\operatorname{tr}R = 1 + 2\cos\theta$ and $\operatorname{vee}(R - R^\top) = 2\sin\theta\, a$, so

$$\theta = \operatorname{atan2}\big(\lVert \operatorname{vee}(R-R^\top)\rVert,\ \operatorname{tr}R - 1\big),\qquad \omega = \frac{\theta}{2\sin\theta}\operatorname{vee}(R - R^\top).$$

Near $\theta = \pi$ the skew part vanishes and the division loses digits. The symmetric part still holds the axis,
$\tfrac12(R + R^\top) = \cos\theta\, I + (1-\cos\theta)\, a a^\top$, so for $\theta > \pi/2$ the code takes the largest column of
$a a^\top = (\operatorname{sym}R - \cos\theta I)/(1-\cos\theta)$ and the sign from the skew part.

### Yaw rate from the body gyro

The IMU measures $\omega$ in the body frame, $[\omega] = R^\top \dot R$. Differentiate $R = R_zR_yR_x$ and use
$R_z^\top\dot R_z = \dot\psi[e_z]$ (likewise for $y$, $x$) and $Q^\top[u]Q = [Q^\top u]$:

$$R^\top\dot R = R_x^\top R_y^\top(\dot\psi[e_z])R_yR_x + R_x^\top(\dot\theta[e_y])R_x + \dot\phi[e_x]
\quad\Rightarrow\quad \omega = \dot\psi\, R_x^\top R_y^\top e_z + \dot\theta\, R_x^\top e_y + \dot\phi\, e_x .$$

With $R_y^\top e_z = (-s_\theta, 0, c_\theta)$, $R_x^\top R_y^\top e_z = (-s_\theta,\ s_\phi c_\theta,\ c_\phi c_\theta)$ and
$R_x^\top e_y = (0, c_\phi, -s_\phi)$:

$$\omega_x = \dot\phi - s_\theta\dot\psi,\qquad \omega_y = c_\phi\dot\theta + s_\phi c_\theta\dot\psi,\qquad \omega_z = -s_\phi\dot\theta + c_\phi c_\theta\dot\psi .$$

Multiply the second by $s_\phi$, the third by $c_\phi$ and add: the $\dot\theta$ terms cancel and $s_\phi^2 + c_\phi^2 = 1$, so

$$\dot\psi = \frac{s_\phi\,\omega_y + c_\phi\,\omega_z}{c_\theta},\qquad \dot\theta = c_\phi\omega_y - s_\phi\omega_z,\qquad \dot\phi = \omega_x + s_\theta\dot\psi .$$

Integrating the gyro z axis alone as yaw makes the error rate $\dot\psi - \omega_z = (1 - c_\phi c_\theta)\dot\psi + s_\phi\dot\theta$.
During a turn at constant tilt the second term is zero and the yaw estimate is short by the fraction $1 - c_\phi c_\theta$.
Walking at 0.2 m/s in simulation the trunk rolled between −2.1° and 3.6° and pitched between −2.0° and 1.9° (tutorial, walking
chapter, "How the trunk moves"). At 3.6° roll, $1 - \cos 3.6° = 1.97\times10^{-3}$: 0.710° of yaw lost per full turn; with 2.0°
pitch added, 0.929° per turn. The $s_\phi\dot\theta$ term has zero mean only if roll and pitch rate are uncorrelated over a gait
cycle; this module does not measure that.

### Camera optical frame

The camera link frame is x forward, y left, z up; the optical frame used by image projection is z forward, x right, y down.
A URDF joint with `rpy="-pi/2 0 -pi/2"` gives $R = R_z(-\pi/2)\,R_y(0)\,R_x(-\pi/2)$:

$$R_x(-\tfrac\pi2) = \begin{bmatrix} 1&0&0\\0&0&1\\0&-1&0\end{bmatrix},\quad
R_z(-\tfrac\pi2) = \begin{bmatrix} 0&1&0\\-1&0&0\\0&0&1\end{bmatrix},\quad
R = R_z R_x = \begin{bmatrix} 0&0&1\\-1&0&0\\0&-1&0\end{bmatrix}.$$

Row $i$ of the product is row $i$ of $R_z$ applied to $R_x$: row 1 of $R_z$ is $e_2^\top$, so row 1 of $R$ is row 2 of $R_x$,
$(0,0,1)$; row 2 is minus row 1 of $R_x$; row 3 is row 3 of $R_x$. The columns of $R$ are the optical axes written in the link
frame: $x_\text{optical} = (0,-1,0) = -y_\text{link}$, $y_\text{optical} = (0,0,-1) = -z_\text{link}$,
$z_\text{optical} = (1,0,0) = x_\text{link}$.

### Height error from an uncorrected tilt

A point at range $r$ along the sensor's forward axis, with the trunk pitched by $\theta$ but the transform assuming it level,
lands at height $r\sin\theta$ off its true value:

$$\Delta z = r\sin\theta = 3\ \text{m} \times \sin 3.6° = 0.1884\ \text{m}.$$

## Implementation

- `rotations.py`: `skew`, `Rx/Ry/Rz`, `exp_so3`, `log_so3`, `R_to_quat` (Shepperd), `rpy_to_R`, `R_to_rpy` (with the gimbal-lock
  branch), `rpy_rates`, `se2_compose`, `se2_inverse`, `se3`, `se3_inverse`, `R_LINK_OPTICAL`. Rodrigues' formula and quaternion to
  matrix are the existing `axis_angle_R` and `quat_to_R` in `../common.py`, imported rather than repeated.
- `validate.py`: random-sample comparison and the worked numbers. `test_rotations.py`: the same checks as regression tests.

## Experiment

```bash
.venv-modules/bin/python modules/m00_math/validate.py --n 100000 --json modules/m00_math/results.json
.venv-modules/bin/python -m pytest -q modules/m00_math
```

100 000 rotations drawn uniformly on SO(3) (scipy `Rotation.random`, seed 0). The yaw check integrates $\omega_z$, obtained from
the true rotation sequence with scipy's logarithm at 1 ms steps, over one full turn at 1 rad/s.

## Validation

| Check | Measured max error | Acceptance |
|---|---|---|
| `quat_to_R` vs scipy | 6.7e-16 | < 1e-14 |
| `R_to_quat` vs scipy (sign fixed to w ≥ 0) | 1.7e-16 | < 1e-14 |
| `rpy_to_R` vs Pinocchio `rpy.rpyToMatrix` | 1.2e-15 | < 1e-14 |
| `R_to_rpy` vs Pinocchio `rpy.matrixToRpy` | 7.7e-15 | < 1e-10 |
| `exp_so3` vs Pinocchio `exp3` | 8.9e-16 | < 1e-14 |
| `log_so3` vs scipy `as_rotvec` | 1.3e-15 | < 1e-9 |
| `log_so3` vs Pinocchio `log3` | 5.7e-11 | < 1e-9 |
| `se3_inverse` vs Pinocchio `SE3.inverse` | 0 | < 1e-14 |
| `rpy_rates` vs Pinocchio `rpy.computeRpyJacobian` (LOCAL), \|pitch\| < 1.5 rad | 2.7e-15 | < 1e-12 |
| Gimbal lock, pitch ±π/2: reconstructed R | < 1e-12 (test) | < 1e-12 |
| `R_LINK_OPTICAL` vs Pinocchio `rpyToMatrix(-π/2, 0, -π/2)` | exact | equal to 1e-15 |

| Yaw lost per turn integrating $\omega_z$ only | Predicted $1 - c_\phi c_\theta$ | Integrated |
|---|---|---|
| roll 1.0° | 0.055° | 0.055° |
| roll 2.1° | 0.242° | 0.242° |
| roll 3.6° | 0.710° | 0.710° |
| roll 3.6°, pitch 2.0° | 0.929° | 0.929° |
| roll 10° | 5.469° | 5.469° |

Regression tests: `.venv-modules/bin/python -m pytest -q modules/m00_math` (13 tests).

## Failure analysis

The first `log_so3` computed $\theta = \arccos((\operatorname{tr}R - 1)/2)$. It failed the 1e-9 test with an error of 2.0e-9 at
$\theta = 3.10$ rad: the derivative of arccos grows as $1/\sqrt{1-x^2}$ near $x = \pm1$, so rounding in the trace is amplified,
and the axis was then divided by a small $\sin\theta$. Switching to $\operatorname{atan2}$ and to the symmetric-part axis above
$\pi/2$ brought the error against scipy to 1.3e-15. Pinocchio's `log3` shows the same effect: its largest deviation from scipy,
5.7e-11, is at $\theta = \pi - 3.7$ mrad, while below 3 rad it agrees to 4.6e-14.

`R_to_rpy` loses accuracy as $c_\theta \to 0$ before the gimbal-lock branch takes over at $c_\theta < 10^{-9}$: $\phi$ and $\psi$ come
from entries of size $c_\theta$ with absolute rounding of about 1e-16, so their error grows like $10^{-16}/c_\theta$. `rpy_rates`
divides by $c_\theta$ and is undefined at $\pm\pi/2$; an estimator that can pitch that far must integrate the rotation (quaternion
or SO(3)) and extract angles only for display.

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017, ch. 3.
- J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics", arXiv:1812.01537, 2018.
- S. W. Shepperd, "Quaternion from rotation matrix", *Journal of Guidance and Control* 1(3), 1978.
- REP 103 (standard units and coordinate conventions) and REP 105 (coordinate frames), ROS Enhancement Proposals.
- SciPy `scipy.spatial.transform.Rotation`; J. Carpentier et al., "The Pinocchio C++ library", IEEE/SICE SII, 2019.
