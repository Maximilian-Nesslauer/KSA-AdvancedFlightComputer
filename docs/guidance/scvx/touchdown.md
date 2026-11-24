# Touchdown guidance (6-DOF)

Touchdown guidance is AFC's six-degree-of-freedom SCvx planner. It plans the last part of a landing as a rigid body: position, velocity, attitude, body rates and mass, steered by the gimbals, the roll torque, the RCS and any control surfaces. It takes over in one of four ways. Entry guidance can hand it the craft in the glide, 5 s before entry's planned ignition or 12 s before entry's planned touchdown, whichever comes first. It then plans the rest of the coast and the ignition as well as the burn. It can also take a landing burn already under way, from entry guidance's burn or from the braking burn of DEORBIT + LAND, after a standby solve has converged in the background. Or it can be engaged directly. Each plan ends with the craft upright, at the height where its first foot meets the ground, sinking at 0.3 m/s. When the plan has run out, a set of fallback rules flies the rest: the sink to contact, the climb cut, the no-plan stopping law, and terminal hover if the plan is lost near the ground. These rules are not part of the 6-DOF plan. [Touchdown guidance on the craft](../scvx-bridge.md) covers how a plan becomes actuator commands.

This page states the problem in continuous form, then says how each part becomes convex. The machinery every SCvx planner shares is on [Successive convexification](index.md).

## At a glance

| | |
| --- | --- |
| Frame | Site frame: origin at the landing site centre, $+z$ up, treated as inertial. Gravity is uniform. The air is at rest in this frame. |
| State | $\mathbf x = (\mathbf r, \mathbf v, \mathbf q, \boldsymbol\omega, m) \in \mathbb R^{14}$ |
| Controls | $7 + n_f$: lateral thrust $t_x, t_y$, axial thrust $T$, roll torque $\tau_r$, three RCS torques, and one deflection per fin ($n_f$ fins) |
| Parameters | burn duration $\sigma$, free; coast duration $\sigma_c$ when the plan starts before ignition, free until it is locked |
| Nodes | 5 to 50, on the rungs 50, 40, 30, 25, 20, 15, 10, 5, chosen for about 0.6 s between nodes |
| Discretisation | trapezoidal collocation, controls linear between nodes, uniform grid in each phase |
| Linearisation | forward-mode automatic differentiation (dual numbers); RCS columns in closed form |
| Cone solver | SCS (ADMM), with the quadratic terms passed as SCS's own $\tfrac12\mathbf y^\top P\mathbf y$ |
| Where it runs | a worker thread per craft (`SixDofThreaded`, on) |
| Re-solve cadence | every 0.1 s (`SixDofReplanSec`), from the measured state, warm-started from the last plan |

## State and controls

$$
\mathbf x = \begin{bmatrix}\mathbf r\\ \mathbf v\\ \mathbf q\\ \boldsymbol\omega\\ m\end{bmatrix}\in\mathbb R^{14},
\qquad
\mathbf u = \big(t_x,\ t_y,\ T,\ \tau_r,\ \boldsymbol\tau_{\text{rcs}},\ \delta_1,\dots,\delta_{n_f}\big)\in\mathbb R^{7+n_f},
\qquad
\sigma,\ \sigma_c \in \mathbb R_{\ge 0}.
$$

- $\mathbf r$ is the position of the centre of mass in the site frame, m.
- $\mathbf v$ is its velocity, m/s. It is also the air velocity, since the air is at rest in the site frame.
- $\mathbf q = (q_w, q_x, q_y, q_z)$ is the attitude, a unit quaternion, scalar first, body to site. $R(\mathbf q)$ is the matching rotation matrix. The body $z$ axis is the thrust axis, measured from the engines, so $\mathbf b = R(\mathbf q)\,\hat{\mathbf z}$ points the way the thrust pushes.
- $\boldsymbol\omega$ is the body rate, in body axes, rad/s.
- $m$ is the mass, kg.
- $t_x, t_y$ are the lateral thrust components the gimbals give, in body axes, N.
- $T$ is the axial thrust, N. It carries the throttle limits and the mass flow.
- $\tau_r$ is the roll torque the gimbals give on their own, N m.
- $\boldsymbol\tau_{\text{rcs}} = (\tau_{\text{rcs},x}, \tau_{\text{rcs},y}, \tau_{\text{rcs},z})$ is the RCS torque about each body axis, N m. Each is one signed control.
- $\delta_i$ is the deflection of fin $i$, degrees.
- $\sigma$ is the duration of the burn, s. $\sigma_c$ is the duration of the coast before ignition, s.

The engine sits a distance $L$ below the centre of mass along the thrust axis. The thrust vector in body axes is $\mathbf T_b = (t_x, t_y, T)$. For a small gimbal angle its length is close to $T$, so $T$ alone carries the throttle box and the mass flow. In flight the commanded thrust is $\lVert\mathbf T_b\rVert$, so the axial part is exactly $T$.

A craft without RCS keeps the three RCS controls, pinned at zero by their bounds. A craft without fins has $n_f = 0$.

## Dynamics

$$
\begin{aligned}
\dot{\mathbf r} &= \mathbf v \\
\dot{\mathbf v} &= \mathbf g + \hat{\mathbf a} + \frac{1}{m}\Big(R(\mathbf q)\big(\mathbf T_b + K\,\boldsymbol\tau_{\text{rcs}}\big) + \mathbf F_{\text{aero}}\Big) \\
\dot{\mathbf q} &= \tfrac12\,\mathbf q \otimes \begin{bmatrix}0\\ \boldsymbol\omega\end{bmatrix} \\
J\dot{\boldsymbol\omega} &= \boldsymbol\tau_{\text{gim}} + T\,\mathbf m_T + \boldsymbol\tau_{\text{rcs}} + \mathbf M_{\text{aero}} - \boldsymbol\omega\times(J\boldsymbol\omega) \\
\dot m &= -\big(\dot m_0 + \alpha\,T\big)
\end{aligned}
$$

**Gravity and bias.** $\mathbf g = (0, 0, -\mu/r_s^2)$ is the gravity at the site, from the body's $\mu$ and the site radius $r_s$. $\hat{\mathbf a}$ is the estimated acceleration bias: what the model is missing, measured in flight. See [Offset-free bias](#offset-free-bias). It is a constant inside each solve.

**Gimbal force and torque.** The engine's thrust point is at $(e_x, e_y, -L)$ from the centre of mass, in body axes. The lateral thrust acts there, so

$$
\boldsymbol\tau_{\text{gim}} = \begin{bmatrix} L\,t_y \\ -L\,t_x \\ \tau_r + e_x t_y - e_y t_x \end{bmatrix},
\qquad
(e_x, e_y) = (-m_{T,y},\ m_{T,x}).
$$

The torque is linear in the controls. The third row is the roll a sideways deflection brings when the thrust acts off the axis, such as when a landing runs on one engine of a ring. It is zero for a balanced cluster.

**Engine moment.** $\mathbf m_T$ is the moment the running engines make about the centre of mass per newton of axial thrust, with the gimbals at rest, N m per N. It is non-zero when the thrust line misses the centre of mass. It couples throttle and torque, so the plan holds the deflection that cancels it from node 0.

**RCS.** $\boldsymbol\tau_{\text{rcs}}$ is the RCS torque. $K$ is a $3\times3$ matrix of the side force each newton metre brings, N per N m, in body axes. On a booster the thrusters sit far above the centre of mass, so each newton metre comes with a sideways push of about one over that distance. $K$ is the average of what the two directions bring, which is exact for a symmetric layout. The RCS propellant (tens of kilograms against hundreds for the engine) is left out of the mass.

**Aerodynamics.** The air force and moment come from tables the aero sweep measures in the game's body axes, with rotational damping and the exact closed-form 2-term moment limiter evaluated in `Dual` arithmetic inside `BodyAero6Dof.Wrench`:

$$
\mathbf F_{\text{aero}} = R(\mathbf q)\,w\,\bar q S\,\Big(\mathbf C_F(M,\alpha,\beta) + \sum_i \Delta\mathbf C_{F,i}(\delta_i)\Big) + (1-w)\,\mathbf F_{\text{pm}}(h, \mathbf v, \mathbf b),
$$

$$
\boldsymbol\tau_{\text{damp}} = -\tfrac12\rho(h)\,v\,w\,\operatorname{diag}(C_{lp}, C_{mq}, C_{nr})\,\boldsymbol\omega_{\text{table}},
\qquad
D(\bar q, v) = 1 + \bar q\!\left(k_{\text{stiff}} + \frac{k_{\text{damp}}}{v}\right),
$$

$$
\mathbf M_{\text{aero}} = \frac{w\,\bar q S\,\Big(\ell\,\mathbf C_M(M,\alpha,\beta) + \ell\sum_i \Delta\mathbf C_{M,i}(\delta_i)\Big) + \Delta\mathbf c\times\mathbf F_{\text{body}} + \boldsymbol\tau_{\text{damp}}}{D(\bar q, v)}.
$$

- $\bar q = \tfrac12\rho(h)\lVert\mathbf v\rVert^2$ is the dynamic pressure, with $h$ the altitude of the site plus $r_z$.
- $S$ and $\ell$ are the reference area and length. Local Mach number $M = \lVert\mathbf v\rVert / a(h)$ uses altitude-varying `Atmosphere.Mach(speed, altitude)`.
- $\alpha$ and $\beta$ are the angles of the air velocity off engine-first, read in the body axes from the plan's own $\mathbf q$. So the model knows its roll.
- $\Delta\mathbf C_{F,i}$ and $\Delta\mathbf C_{M,i}$ are what fin $i$ adds at its deflection, with smooth transitions across breakpoints in `BodyAero6Dof.FinCoefficients`.
- $\Delta\mathbf c$ moves the moment from the centre of mass the sweep saw to the one the craft has now. $\mathbf F_{\text{body}}$ is the table force in body axes.
- $\boldsymbol\omega_{\text{table}} = \mathbf M_{\text{m}\to\text{t}}\boldsymbol\omega$ rotates body rates into table axes for rotational aerodynamic damping $(C_{lp}, C_{mq}, C_{nr})$.
- $k_{\text{stiff}} = \tfrac12\lVert I_{\text{asmb}}^{-1}\rVert_\infty \Delta t^2 K_{\text{sub}}$ and $k_{\text{damp}} = \lVert I_{\text{asmb}}^{-1}\rVert_\infty \Delta t\,C_{\text{sub}}$ are computed live every step with zero filter lag via `KsaAeroSweep.TryComputeMomentLimiterClosedForm` and passed via `WithMomentLimiterExact`.
- $w = w_{\text{tilt}}\,w_{\text{speed}}$ fades the tables out where they mean nothing. Both factors are smoothsteps: $w_{\text{speed}}$ from 0 at 5 m/s to 1 at 15 m/s, and $w_{\text{tilt}}$ from 1 at 20 deg off engine-first to 0 at 28 deg, where the tables end.
- $\mathbf F_{\text{pm}}$ is the point-mass drag and lift on the thrust axis $\mathbf b$ from the 3-DOF tables. It carries the share $(1-w)$ the body tables fade out of. It has no moment.

The fade keeps the force and its slopes continuous, so the solver never meets a jump.

**Rotational dynamics.** $J = \mathrm{diag}(I_x, I_y, I_z)$ is the inertia about the model body axes. It is the live tensor projected onto those axes, re-read before every solve as the propellant drains. The off-diagonal terms are dropped, which is exact for an axisymmetric craft.

**Engine back-pressure parity and mass flow.** `Dynamics6Dof.Params` carries the shared differentiable 2D cubic B-spline `EngineThrustTable` (`Engine`) and reference pressure `EngineRefPressurePa`. Inside `Dynamics6Dof.F`, the site-reference axial thrust control $T$ maps via the exact Inverse-Function-Theorem `Dual` inverse $u_{\text{th}} = \texttt{Engine.ThrottleFor}(T, P_{\text{ref}})$ to altitude-exact thrust $T_{\text{eff}} = \texttt{Engine.Thrust}(u_{\text{th}}, P(h))$ (applied to both axial thrust and off-centre `ThrustMoment`) and mass flow $\dot m = -\texttt{Engine.MassFlow}(u_{\text{th}})$. `Ksa6DofGuidance.Command` applies the identical conversion at live ambient pressure, eliminating thrust jumps at 3-DOF to 6-DOF hand-over. When no table is attached, mass flow falls back to the linear fit $\dot m_0 + \alpha T$ (or $T / (I_{sp} g_0)$ in vacuum).

**The coast.** Before ignition the engine is off: $t_x$, $t_y$ and $T$ have no effect and no propellant flows. The RCS, the fins and the air still act.

**Time dilation.** The burn runs on $\tau \in [0,1]$ with $d\mathbf x/d\tau = \sigma\,\mathbf f(\mathbf x,\mathbf u)$. A plan with a coast has a second phase before it with $d\mathbf x/d\tau = \sigma_c\,\mathbf f_{\text{off}}(\mathbf x,\mathbf u)$. See [Free final time and phases](index.md#free-final-time-and-phases).

## Boundary conditions

**Initial state.** Node 0 is the measured state of the craft, all 14 components:

$$
\mathbf x(0) = \mathbf x_0 .
$$

**Terminal state.** Mass is free. The other 13 components are pinned to

$$
\mathbf r(t_f) = \begin{bmatrix}0\\0\\ h_f\end{bmatrix},\qquad
\mathbf v(t_f) = \begin{bmatrix}0\\0\\ -0.3\end{bmatrix}\ \text{m/s},\qquad
\mathbf q(t_f) = \mathbf q_f,\qquad
\boldsymbol\omega(t_f) = \mathbf 0 ,
$$

with $t_f = \sigma_c + \sigma$.

- **The site** is the origin. A flat-ground search may move the site before the plan is built. Slopes do not enter the plan in any other way.
- **$h_f$** is the height of the centre of mass when the first foot meets the ground: the measured height of the centre of mass above the feet, plus the early contact $\Delta h$. The early contact is how much sooner than level ground a foot meets the ground under it. For each foot at radius $r_i$ from the thrust axis and depth $d_i$ below the centre of mass, it takes the highest ground $g(r_i)$ within that radius, and $\Delta h = \max_i\big(d_i + g(r_i)\big) - \max_i d_i$. This is exact with a foot pointing up the slope, and errs high otherwise. $h_f$ is updated when the node count changes, as the centre of mass moves up the craft.
- **The sink of 0.3 m/s** carries a craft that can hover on to the ground. A plan that ended at rest above lower ground left it hovering there.
- **$\mathbf q_f$** is upright at the heading the craft has now: the twist of the measured attitude about the vertical. After a glide that rolled the craft to its least rolling moment, it is upright with the wings where the glide held them.

**Roll left free.** A craft whose gimbals and RCS together give less than $0.002\,I_z$ N m of roll at full thrust cannot stop the roll it has. For such a craft the plan drops the terminal rows on $q_w$, $q_z$ and $\omega_z$. It must still arrive upright ($q_x = q_y = 0$) with no pitch or yaw rate, at any heading and roll rate. The choice is made once per landing (`TerminalRollFree`).

## Constraints

| Constraint | Continuous form | In the subproblem | Code |
| --- | --- | --- | --- |
| Dynamics | $\dot{\mathbf x} = \mathbf f(\mathbf x,\mathbf u)$ | linearised, trapezoidal, plus virtual control | `Scvx6DofSubproblemScs.AssembleEqualities` |
| Initial state | $\mathbf x(0) = \mathbf x_0$ | hard equality | `AssembleEqualities` |
| Terminal position and velocity | $\mathbf r(t_f) = \mathbf r_f$, $\mathbf v(t_f) = \mathbf v_f$ | soft, two-sided L1 slack per axis | `AssembleEqualities`, `TerminalMissWeight` |
| Terminal attitude and rates | $\mathbf q(t_f) = \mathbf q_f$, $\boldsymbol\omega(t_f) = \mathbf 0$ | hard; three rows dropped with the roll free | `AssembleEqualities`, `TerminalRollFree` |
| Unit quaternion | $\lVert\mathbf q\rVert = 1$ | tangent plane at interior nodes, reference re-projected | `AssembleEqualities`, `Scvx6DofSolver.NormaliseQuaternions` |
| Thrust bounds | $T_{\min} \le T \le T_{\max}$; $T = 0$ in the coast | convex as it is | `AssembleCone` |
| Gimbal angle | $\lVert(t_x,t_y) - T\mathbf c\rVert \le T\tan\delta_{\max}$ | second-order cone, every node | `AssembleCone` |
| Gimbal slew | $\lvert\dot t_{x}\rvert, \lvert\dot t_{y}\rvert \le s\,T\tan\delta_{\max}$ | linear rows between nodes, step length from the reference | `AssembleCone`, `GimbalSlewPerS` |
| Roll torque | $\lvert\tau_r\rvert \le \tau_{r,\max}\,T/T_{\max}$ | linear rows | `AssembleCone`, `RollTorqueFollowsThrust` |
| RCS torque | $-\tau^-_a \le \tau_{\text{rcs},a} \le \tau^+_a$ | box | `AssembleCone`, `RcsBounds` |
| Fin deflection | $\delta_i^{\min} \le \delta_i \le \delta_i^{\max}$ | box | `AssembleCone`, `FinBounds` |
| Fin rate | $\lvert\dot\delta_i\rvert \le \dot\delta_i^{\max}$ | linear rows between nodes | `AssembleCone`, `FinRate` |
| Tilt | $1 - 2(q_x^2+q_y^2) \ge \cos\theta_{\max}$ | exact second-order cone on $(q_x,q_y)$, every node | `AssembleCone`, `SinHalfTilt` |
| Ground floor | $r_z \ge -1$ m | linear row, every node | `AssembleCone`, `GroundFloor` |
| Glide slope | $\lVert\mathbf r_{xy}-\mathbf r_{f,xy}\rVert \le \cot\gamma\,(r_z - r_{f,z})$ | second-order cone from node 1, entry allowance, L1 slack | `AssembleCone`, `GlideSlopeDeg` |
| Climb rate | $v_z \le v_{z,\max}$ | linear row from node 1, L1 slack | `AssembleCone`, `VzMax` |
| Thrust acceleration | $T \le a_{\max}\,m$ | linear row at every node, L1 slack | `AssembleCone`, `AccelMax` |
| Turn rate | $\lVert\boldsymbol\omega\rVert \le \omega_{\max}$ | second-order cone from node 1, L1 slack | `AssembleCone`, `RateMaxRadS` |
| Durations | $\sigma_{\min}\le\sigma\le\sigma_{\max}$, $0\le\sigma_c\le\sigma_c^{\max}$ | box; $\sigma_c$ pinned once ignition is locked | `AssembleCone` |
| Mass | none | none; the mass is checked after the solve | `Scvx6DofSolver.CopyReferenceWithMassBalance` |

### Dynamics

The continuous form is the set of equations [above](#dynamics). Each interval $k$ of phase $p$ (coast or burn) is imposed by trapezoidal collocation:

$$
\mathbf x_{k+1} = \mathbf x_k + \frac{\Delta\tau_p}{2}\Big(\sigma_p\big(\bar{\mathbf f}_k + \bar{\mathbf f}_{k+1}\big) + \bar\sigma_p\big(A_k\delta\mathbf x_k + B_k\delta\mathbf u_k + A_{k+1}\delta\mathbf x_{k+1} + B_{k+1}\delta\mathbf u_{k+1}\big)\Big) + \boldsymbol\nu_k ,
$$

with $\Delta\tau_p$ the phase's normalised step, $A$ and $B$ the Jacobians on the reference, and $\boldsymbol\nu_k$ the virtual control. The controls are taken as linear between nodes. The ignition node closes the last coast interval with the engine off and opens the burn with it lit, so it is linearised twice. See [Linearisation](index.md#linearisation) and [Discretisation](index.md#discretisation).

### Initial state

Hard: 14 equality rows. The trust region also applies at node 0, so every seed copies $\mathbf x_0$ into node 0 exactly. A seed that disagrees there would make the subproblem infeasible.

### Terminal position and velocity

Each axis gets a pair of non-negative slacks, so the miss can go either way:

$$
\mathbf r_{N-1} - (\mathbf s^{r+} - \mathbf s^{r-}) = \mathbf r_f,\qquad
\mathbf v_{N-1} - (\mathbf s^{v+} - \mathbf s^{v-}) = \mathbf v_f,\qquad
\mathbf s \ge \mathbf 0 .
$$

Soft with an L1 price. When the target can be reached the slacks sit at zero. When it cannot, the plan lands off target or a little fast instead of failing. A craft that is falling cannot decline to land, and "no plan" only means flying an older one. Position is priced above speed: a squeezed plan arrives a little fast rather than stopping high. See [Soft constraints and slacks](index.md#soft-constraints-and-slacks).

### Terminal attitude and rates

Hard equality rows on $\mathbf q_{N-1}$ and $\boldsymbol\omega_{N-1}$. With the roll left free, the rows on $q_w$, $q_z$ and $\omega_z$ are dropped. The terminal quaternion is put on the same sign branch as the reference's last node before each solve, since $\mathbf q$ and $-\mathbf q$ are the same attitude but not the same numbers.

### Unit quaternion

$\lVert\mathbf q_k\rVert = 1$ is not convex. It becomes the tangent plane at the reference,

$$
\bar{\mathbf q}_k^\top\mathbf q_k = 1,\qquad k = 1,\dots,N-2 ,
$$

and on node $N-1$ too when the roll is free. Nodes 0 and $N-1$ are otherwise pinned, so a row there would be redundant. After each accepted step every node's quaternion is normalised and put on the same branch as the node before it. See [Linearisation](index.md#linearisation).

### Thrust bounds

$$
T_{\min} \le T_k \le T_{\max}\ \ \text{(burn nodes)},\qquad T_k = 0\ \ \text{(coast nodes)} .
$$

Convex as it is. Because the axial thrust is a scalar control, the lit engine's floor is a box and needs no relaxation.

- $T_{\max}$ is the thrust of every running engine at the pressure of the site, the densest air of the descent.
- $T_{\min}$ is the plan's floor. It sits 5 % of the thrust range above the engines' real minimum (`LandingPlanFloorMargin`), so the thrust tracker has some thrust to take off, and below the weight when the engines can go below it. If that leaves no plan, the real minimum is used.

### Gimbal angle

$$
\big\lVert (t_{x,k}, t_{y,k}) - T_k\,\mathbf c \big\rVert \le T_k \tan\delta_{\max} .
$$

A second-order cone, exact, at every node. $\mathbf c = (m_{T,y}/L,\ -m_{T,x}/L)$ is the trim: the deflection per newton that holds the engine moment. The cone is centred there, so its width is the authority the gimbals have around the trim. It is zero for a balanced cluster. In the coast $T_k = 0$, so the cone pins the lateral thrust at zero.

$\delta_{\max}$ is not the hardware's limit. It is the effective angle at which $L\,T_{\max}\tan\delta_{\max}$ equals the lateral torque the allocator really delivers, capped at the hardware angle. The plan gets 80 % of that, in torque (`SixDofTorqueMargin`, 0.2), and the attitude feedback keeps the rest.

### Gimbal slew

The gimbal moves at a finite rate. With $s$ in full deflections per second, each lateral axis is limited between nodes:

$$
\big\lvert t_{a,k+1} - t_{a,k}\big\rvert \le \tfrac12\,s\,\Delta\bar t\,\tan\delta_{\max}\,\big(T_k + T_{k+1}\big),\qquad a \in \{x, y\}.
$$

Linear rows. $\Delta\bar t = \bar\sigma\,\Delta\tau$ is the step length at the reference burn time, so the row stays linear. The step from the coast into the burn is free, since the nozzle may already be where the burn wants it. $s$ is the gimbal's own rate cap, about 2 per second for the stock engines (`GimbalSlewPerS`), widened by the torque margin so it stays the hardware rate in degrees. See [Common convexification moves](index.md#common-convexification-moves).

### Roll torque

$$
\lvert\tau_{r,k}\rvert \le \tau_{r,\max}\,\frac{T_k}{T_{\max}} .
$$

Linear rows. The roll torque comes from the gimbals, so it scales with the thrust through them (`RollTorqueFollowsThrust`, on in flight). In the coast it is zero. $\tau_{r,\max}$ is the allocator's measured roll capability at $T_{\max}$, times the 80 % torque share. It is floored at $10^{-3}$ N m so the scale stays positive.

### RCS torque

$$
-\tau^-_a \le \tau_{\text{rcs},a,k} \le \tau^+_a,\qquad a \in \{x,y,z\}.
$$

A box per axis. $\tau^\pm_a$ are the largest torques the allocator measured each way, times the 80 % torque share. A craft without RCS gets the same rows with zero bounds.

### Fin deflection and rate

$$
\delta_i^{\min} \le \delta_{i,k} \le \delta_i^{\max},\qquad
\lvert\delta_{i,k+1} - \delta_{i,k}\rvert \le \dot\delta_i^{\max}\,\Delta\bar t_k .
$$

A box and linear rate rows, in the coast as in the burn. Range and rate come from the fin's part, times the 80 % torque share. $\Delta\bar t_k$ is the reference length of interval $k$ in its phase. A fin with no rate may cross its whole range in one step.

### Tilt

The tilt of the thrust axis from vertical is $\theta$ with $\cos\theta = R_{33} = 1 - 2(q_x^2 + q_y^2)$. For a unit quaternion,

$$
\theta \le \theta_{\max}
\iff q_x^2 + q_y^2 \le \tfrac12(1 - \cos\theta_{\max})
\iff \big\lVert (q_x, q_y) \big\rVert \le \sin\tfrac{\theta_{\max}}{2} .
$$

An exact second-order cone of constant radius at every node, with no reference in it. It is exact to the same degree as the unit-norm tangent plane. The tangent of $R_{33}$ that it replaced has a zero gradient for an upright craft, so the linearised row could not see the limit.

$\theta_{\max}$ is 120 deg (`SixDofTiltDeg`). It is widened to at least the tilt at hand-over plus 5 deg, up to 175 deg. The cone includes node 0, which is pinned to the measured state, so a limit below the present tilt would be infeasible.

### Ground floor

$$
r_{z,k} \ge -1\ \text{m}.
$$

A hard linear row at every node (`GroundFloor`). It is only a guard: the target height $h_f$ and the soft limits keep the plan above the ground.

### Glide slope

The path stays in a cone that opens upward from the target, $\gamma$ above the horizontal:

$$
\lVert\mathbf r_{xy,k} - \mathbf r_{f,xy}\rVert \le \cot\gamma\,\big(r_{z,k} - r_{f,z}\big) + e_k + s^{\text{gs}}_k,
\qquad s^{\text{gs}}_k \ge 0,\qquad k \ge 1 .
$$

A second-order cone from node 1, soft with an L1 slack. $\gamma$ is 10 deg (`SixDofGlideSlopeDeg`). $e_k$ is an entry allowance for a craft handed over outside the cone. With $e_0$ the metres it starts outside and $s = k/(N-1)$,

$$
e_k = e_0\big((1-s)^2 + s\,(1-s)^4\big),
$$

which is the full violation at the start and zero at the target. The plan closes on the cone instead of asking the first interval to jump inside it.

### Climb rate

$$
v_{z,k} \le v_{z,\max} + s^{vz}_k,\qquad s^{vz}_k \ge 0,\qquad k \ge 1 .
$$

A linear row from node 1, soft with an L1 slack. $v_{z,\max}$ is 0 m/s in flight (`SixDofVzEnabled` on, `SixDofVzMaxMs` 0). The two soft limits matter together: outside the glide slope and too low, the only way back is to climb. Hard versions of both could leave no feasible plan.

### Thrust acceleration

$$
T_k \le a_{\max}\,m_k + s^{a}_k,\qquad s^{a}_k \ge 0 .
$$

Exact and linear, since $T$ and $m$ are both variables. Soft with an L1 slack at every node, node 0 included: its thrust is free even though its mass is pinned. $a_{\max}$ is 3 g0 (`LandingAccelLimitG`), raised to 1.05 times the minimum throttle's acceleration when that is higher. It keeps a powerful booster from planning its landing as a few seconds at full thrust, where engine lag costs most.

### Turn rate

$$
\lVert\boldsymbol\omega_k\rVert \le \omega_{\max} + s^{\omega}_k,\qquad s^{\omega}_k \ge 0,\qquad k \ge 1 .
$$

A second-order cone from node 1 on the whole body rate, roll included, soft with an L1 slack. $\omega_{\max}$ is 15 deg/s (`SixDofRateMaxDegS`). With RCS the craft can turn hard, and nothing else in the objective stops the cheapest plan from doing so.

### Durations

$$
\sigma_{\min} \le \sigma \le \sigma_{\max},\qquad \sigma_c^{\min} \le \sigma_c \le \sigma_c^{\max}.
$$

A box. $\sigma_{\min} = 0.15\,\sigma_0$ and $\sigma_{\max} = 4\,\sigma_0$, with $\sigma_0$ the burn-time seed. As the burn counts down, $\sigma_{\min}$ is lowered to stay under half the time left (not below 0.25 s). Otherwise the bound would hold each plan longer than the craft can stretch its last seconds. The coast runs from 0 to 60 s. Its duration is pinned once ignition is less than 1 s away (the ignition lock), and pinned at zero once the engine is lit, so the problem keeps its shape through ignition. A step whose $\sigma$ or $\sigma_c$ lands outside its bounds is refused like a failed solve.

The flight setting `SixDofFixedTime` (off) pins $\sigma$ to the burn time left on every re-solve instead.

### Mass

There is no bound on the mass, and the final mass is free. The objective keeps it high. Before a plan is published its mass history is rebuilt from the measured mass by integrating the planned flow. A plan whose mass reaches zero is refused. This stops virtual control from letting the reference gain mass under thrust.

!!! note "Torque demand"
    `Scvx6DofConfig.TorqueDemand` is a prototype that plans one torque per axis and leaves the split between gimbals and RCS to a mixer outside the plan. The flight setup does not turn it on. Its rows pin $t_x$, $t_y$ and $\tau_r$ at zero and bound the torque by the mixer's envelope.

## Objective

**Continuous.** Burn as little propellant as possible, and break the soft limits as little as possible:

$$
\min\ \frac{m_0 - m(t_f)}{m_0}
+ \sum_j W_j\,\big(\text{violation of soft limit } j\big)
+ \text{small regularisers}.
$$

**Discrete.** The cone program minimises

$$
\begin{aligned}
J_{\text{sub}} ={}& -\frac{m_{N-1}}{m_0}
+ W_{\Delta u}\sum_{k}\sum_j \Big(\frac{u_{k+1,j}-u_{k,j}}{u_{s,j}}\Big)^2
+ W_\omega\sum_k \lVert\boldsymbol\omega_k\rVert^2 \\
&+ W_c\sum_{k=1}^{K-1} \lVert\boldsymbol\omega_k\rVert^2
+ W_{\text{rcs}}\sum_k\sum_a \Big(\frac{\tau_{\text{rcs},a,k}}{u_{s,a}}\Big)^2
+ W_{\text{fin}}\sum_k\sum_i \Big(\frac{\delta_{i,k}}{u_{s,i}}\Big)^2 \\
&+ \rho_{vc}\sum_k \big\lVert\boldsymbol\nu_k\oslash\boldsymbol\nu_s\big\rVert^2
+ W_{\text{gs}}\sum_k \frac{s^{\text{gs}}_k}{L}
+ W_{vz}\sum_k \frac{s^{vz}_k}{V} \\
&+ W_r\,\frac{\mathbf 1^\top(\mathbf s^{r+}+\mathbf s^{r-})}{250\ \text{m}}
+ W_v\,\frac{\mathbf 1^\top(\mathbf s^{v+}+\mathbf s^{v-})}{100\ \text{m/s}} \\
&+ W_a\sum_k \frac{s^a_k}{T_{\max}}
+ W_{\omega\max}\sum_k \frac{s^\omega_k}{1\ \text{rad/s}} .
\end{aligned}
$$

$\mathbf x_s$ and $u_{s,j}$ are the [scales](#scaling-and-the-cone-program), $L$ and $V$ the length and speed scales, and $K$ the number of coast intervals. The smoothing sum skips the engine's controls over the last coast interval, where they step at ignition.

| Term | Config name | Default in `Scvx6DofConfig` | In flight |
| --- | --- | --- | --- |
| Control-rate smoothing $W_{\Delta u}$ | `WDu` | 0.2 | 0.05 (`SixDofControlSmooth`) |
| Body-rate damping $W_\omega$ | `WW` | 1.0 | $0.002 / (3N\,\omega_s^2)$, $\omega_s = \theta_{\text{set}} / \max(\sigma_0/2, 1\ \text{s})$ with $\theta_{\text{set}}$ the tilt setting (`SixDofRateDampShare`) |
| Coast turning $W_c$ | `CoastRateWeight` | 0 | the fuel of 1 m/s for turning at 0.05 rad/s through the coast: $\alpha / \big((K-1)\cdot 0.05^2\big)$ |
| RCS torque $W_{\text{rcs}}$ | `RcsWeight` | 0 | 0.1 (`SixDofRcsWeight`) |
| Fin deflection $W_{\text{fin}}$ | `FinWeight` | $10^{-3}$ | $10^{-3}$ |
| Proximal $W_{\text{prox}}$ | `ProximalWeight` | 0 | 0 (disabled under Clarabel to avoid merit distortion) |
| Virtual control $\rho_{vc}$ | `RhoVc` | $10^5$ | $10^5$ |
| Glide-slope slack $W_{\text{gs}}$ | `GlideSlopeWeight` | $10^4$ | $10^4$ |
| Climb-rate slack $W_{vz}$ | `VzWeight` | $10^4$ | $10^4$ |
| Terminal miss $W_r$ | `TerminalMissWeight`, `TerminalMissScaleM` | 0 (hard) | $10^5$ per 250 m |
| Terminal speed $W_v$ | `TerminalSpeedWeight`, `TerminalSpeedScaleMs` | 0 (hard) | $10^4$ per 100 m/s |
| Acceleration slack $W_a$ | `AccelWeight` | 300 | 300 |
| Turn-rate slack $W_{\omega\max}$ | `RateWeight` | $10^3$ | $10^3$ |

The `Scvx6DofConfig` defaults mirror the Python reference (`6dof.py`), so the offline comparison stays valid. `Ksa6DofSetup.TryBuild` sets the flight values.

Three choices in the weights matter:

- **Rate, not magnitude.** The smoothing prices the change of each control between nodes, not its size. $W_{\Delta u}$ is the only term that keeps the control profile continuous. Without it the minimum-fuel answer jumps its thrust direction by about 19 deg between nodes.
- **Small regularisers.** $W_{\Delta u}$ and $W_\omega$ both get cheaper as the burn gets longer. At the Python values they were 121 % of the fuel term and pinned $\sigma$ at its upper bound. In flight they are kept a small share of fuel; Clarabel's direct $LDL^\top$ solver stays well-conditioned without needing a proximal distortion term.
- **The RCS price** makes the split between gimbals and RCS unique, and leans on the gimbals, which cost no propellant. Without it the split was nearly free, and SCS needed several times the iterations.

## What this solver does differently

This section gives this solver's settings for the shared machinery, then the parts that are its own.

### Linearisation and discretisation

$A$ and $B$ come from forward-mode automatic differentiation of $\mathbf f$: one pass of dual numbers per state and control column. The RCS columns are linear in their controls and are written in closed form, which saves three passes through the aero tables. Fin columns are swept. On a swept fin deflection the slope is the mean of the two sides, so a fin at rest is not told one side of a kink.

The grid is uniform in each phase. Collocation is trapezoidal with no sub-steps: the dynamics are evaluated at the two ends of each interval only. The defect used for the merit and the gates is the same trapezoid on the true dynamics. A coast takes 40 % of the intervals (`CoastIntervalsFor`). See [Discretisation](index.md#discretisation).

### Virtual control (`vc-rv`)

$\boldsymbol\nu_k$ enters every state row of every interval. It is priced as a **squared two-norm**, $\rho_{vc}\sum_k\lVert\boldsymbol\nu_k\oslash\boldsymbol\nu_s\rVert^2$ with $\rho_{vc} = 10^5$, using warm-ramped position and velocity scaling (`vc-rv`):

- $\boldsymbol\nu_s$ uses `VirtualControlPositionScaleM = 5.0` m on position (`[0..2]`), `VirtualControlVelocityScaleMs = 5.0` m/s on velocity (`[3..5]`), and `XScale` on `[6..13]`.
- `Ksa6DofGuidance.PricingStage` keeps Stage 0 (cold solves and $t_{\text{go}} > 25$ s) loose on `XScale` so straight-line cold seeds are not trapped in high-fuel local minima, then tightens geometrically over 4 accepted warm solves toward the `vc-rv` scales as $t_{\text{go}}$ drops below 10 s.
- Because Clarabel's direct $LDL^\top$ KKT solver is insensitive to diagonal $P$ scaling, tight position weights drive terminal horizontal miss to sub-centimetre levels without conditioning loss or extra L1 slack variables.

### Outer-loop invariants

- `NormaliseQuaternions(_xbar)` executes *before* `_jRef = TrueCost(...)` is stored, preventing systematic bias in $\rho$.
- Gimbal slew-rate and fin-rate constraints include the time-dilation derivative $\partial(\mathrm{d}u/\mathrm{d}t)/\partial\sigma$.
- `ColdReferenceShift` smoothly blends observed $(q, \boldsymbol\omega)$ shifts across nodes $0 \dots k_{\text{blend}}$ rather than introducing a step discontinuity across interval 0.

### Trust region

A **hard infinity-norm box** on every scaled state, control and duration, at every node including node 0:

$$
\lvert x_{k,i}-\bar x_{k,i}\rvert \le \eta\,x_{s,i},\qquad
\lvert u_{k,j}-\bar u_{k,j}\rvert \le \eta\,u_{s,j},\qquad
\lvert\sigma-\bar\sigma\rvert \le \eta\,\sigma_0,\qquad
\lvert\sigma_c-\bar\sigma_c\rvert \le \eta\,\sigma_0 .
$$

It does not apply to the virtual control or the slacks. A pinned coast is always reachable, whatever $\eta$.

- $\eta \in [10^{-3}, 0.1]$ (`TrustRegionMin`, `TrustRegionMax`).
- A cold or spread solve starts at 0.1. Every warm re-solve starts at 0.05, a constant (`WarmTrustRegion`). Carrying $\eta$ across cycles was tried twice and dropped.
- Update: accept if $\rho > 0$. Shrink by 0.5 if $\rho < 0.25$. Grow by 1.5 if $\rho \ge 0.7$ and the step used at least 80 % of $\eta$. A subproblem that fails, is cut off at its iteration cap, or returns a duration outside its bounds counts as rejected and shrinks $\eta$.

The proximal term $W_{\text{prox}}\lVert(\mathbf x-\bar{\mathbf x})\oslash\mathbf x_s\rVert^2$ acts as a soft trust region on the state on top of the box. See [Trust region](index.md#trust-region).

### Accepting a step

The merit uses the true defect $\mathbf d_k$ of the trapezoid:

$$
\Phi = \frac{m_0 - m_{N-1}}{m_0} + J_{\text{smooth}} + \rho_{vc}\sum_k\lVert\mathbf d_k\oslash\boldsymbol\nu_s\rVert^2 + J_{\text{slacks}},
$$

where $J_{\text{smooth}}$ holds the rate, RCS and fin smoothing terms above, and $J_{\text{slacks}}$ prices all five soft limits (acceleration, turn-rate, glide-slope, climb-rate and terminal position/velocity miss). On the initial reference (`TrueCost`), $J_{\text{slacks}}$ evaluates those five violations directly on the trajectory (`SoftLimitCost`). Inside `Iterate()`, both $\Phi_{\text{lin}}$ and $\Phi$ use the subproblem's exact convex slack costs (`AccelerationSlackCost + RateSlackCost + PathSlackCost`), while $\Phi$ replaces $\boldsymbol\nu_k$ with the true defect $\mathbf d_k$, so only dynamics linearisation error moves $\rho$. The ratio $\rho$ is actual over predicted improvement, as in [Accepting a step](index.md#accepting-a-step). When the predicted improvement is below $10^{-9}$, $\rho$ is $\pm1$ by the sign of the actual one.

Only the proximal term is excluded from the merit.

### Convergence

An accepted step converges when

$$
\max\Big(\max_{k,i}\frac{\lvert x^\star_{k,i}-\bar x_{k,i}\rvert}{x_{s,i}},\ \frac{\lvert\delta\sigma\rvert}{\sigma_0}\Big) < 8\times10^{-3}
\quad\text{and}\quad
\max_{k,i}\frac{\lvert d_{k,i}\rvert}{x_{s,i}} < 10^{-3}
$$

(`StepTolerance`, `DefectTolerance`). In flight a re-solve is mostly stopped by its time budget after one iteration, and the [defect gate](#defect-gate) decides whether the plan is flown. See [Convergence](index.md#convergence).

### Scaling and the cone program

| Variable | Scale |
| --- | --- |
| position | $L = \max(\lVert\mathbf r_0-\mathbf r_f\rVert, 1\ \text{m})$, from the state at ignition for a plan with a coast |
| velocity | $V = \max(\lVert\mathbf v_0\rVert, \sqrt{L g}, 1\ \text{m/s})$ |
| quaternion, body rate | 1 |
| mass | $m_0$ |
| lateral thrust | $T_{\max}(\tan\delta_{\max} + \lVert\mathbf c\rVert)$ |
| axial thrust | $T_{\max}$ |
| roll torque | $\tau_{r,\max}$ |
| RCS torque, axis $a$ | the axis's physical RCS torque authority (`RcsTorqueScale`) |
| fin | its range, degrees |
| durations | the burn-time seed $\sigma_0$ |

Rows are then equilibrated by their largest entry (with every row of a second-order cone sharing one scale, leaving rows below $10^{-4}$ unscaled), and Clarabel applies Ruiz equilibration (`EquilibrateMaxIter = 3`) on top. See [Scaling](index.md#scaling).

The subproblem is solved through a persistent native **`ClarabelWorkspace`** (backed by `IScvxConicWorkspace`, with `ScsWorkspace` available behind a switch). Equalities, non-negative orthant and second-order cones (gimbal, glide slope, tilt, turn rate) stack into one frozen-sparsity `CcsAssembler`, and quadratic terms go into $P$ directly without epigraph cones. See [The cone program](index.md#the-cone-program).

| Clarabel setting | Value |
| --- | --- |
| tolerance (`ClarabelEpsCap`) | $10^{-4}$ |
| `PresolveEnable` / `InputSparseDropzeros` | `false` / `false` (mandatory for numeric `update_A` sparsity invariance) |
| `EquilibrateMaxIter` | `3` (fast Ruiz equilibration on real-time warm updates) |
| `IterativeRefinementEnable` | `false` (automatic one-shot `IR = true` rebuild on numerical faults) |
| SCvx warm cycle budget | 25 ms (`CycleBudgetMs`); 120 ms after a refusal (`RecoveryCycleBudgetMs`); at least one iteration always runs |
| Cold solve budget (`Ksa6DofGuidance.Finish`) | Explicit `budgetMs` when $> 0$; otherwise `max(LongBudgetMs, 5000.0)` ms for cold calls so synchronous cold solves (`Plan`, `PlanFromSeed`, `PlanSearch`) are never truncated to `CycleBudgetMs` |

A subproblem that fails or hits its cap counts as rejected and shrinks the trust region.

### Re-solving in flight

The loop follows [Re-solving in flight](index.md#re-solving-in-flight).

- **Cadence.** A re-solve is sent to the worker every 0.1 s, with up to 5 SCvx iterations inside the cycle budget. The cadence timer moves on only when a plan is published.
- **Warm seed.** The last plan is resampled at the times the new grid wants, on a horizon that shrinks onto the same end time. Position and velocity come from the cubic through each interval's end states, the rest is interpolated linearly. The whole reference is then moved to the measured state via `ColdReferenceShift` (blending $(q, \boldsymbol\omega)$ smoothly across nodes $0 \dots k_{\text{blend}}$). Every node is put on the quaternion branch of node 0.
- **Async cold initialisation (`Ksa6DofJob.InitCold`).** `StartSixDofStandby` reads only immutable vehicle/body properties on the main thread, clones mutable target arrays, and dispatches `TryDispatchInitCold` to `Ksa6DofSolveWorker` so constructing `Ksa6DofGuidance` and running `BeginCold` never hitches the main simulation thread. `StepSixDofStandbyCore` collects the produced instance, logs the `"seed"` snapshot, and skips `StepCold`/`Track` until it arrives.
- **Cold solve.** On the worker thread (default, `SixDofThreaded` on), cold iterations run back-to-back (`ColdIterationIntervalS = 0`) and re-anchor at the craft before each; when `SixDofThreaded` is off, they are paced at one iteration every 0.25 s (`SixDofColdIntervalS`). It hands over at the warm gate. After 6 refused iterations in a row it restarts from the craft's state. The seed is a straight line, the braking cubic `ColdSeed6Dof.Build`, or entry guidance's glide and burn (`ColdSeed6Dof.BuildAlongPath`) for a plan with a coast.
- **Standby.** Under another mode's burn, `Ksa6DofGuidance.Track` re-solves back to back with 2 iterations, keeping its latest trajectory whether or not it passed the gate.
- **Model refresh.** Inertia, the closed-form 2-term moment limiter, engine thrust table and bias are refreshed at the start of each solve.

### Offset-free bias

A force the model lacks is met again on every re-plan, so re-solving cannot correct it. The guidance estimates the residual acceleration

$$
\hat{\mathbf a} \leftarrow \hat{\mathbf a} + \frac{\Delta t}{2\ \text{s}}\Big(\mathbf a_{\text{meas}} - \big(\tfrac{T_{\text{del}}}{m}\mathbf b + \mathbf g + \mathbf F_{\text{aero}}/m\big) - \hat{\mathbf a}\Big),
$$

a low-pass filter with a time constant of about 2 s, clamped to 5 m/s^2 per axis, and adds it to the plan's gravity. $T_{\text{del}}$ is the thrust the last throttle delivers, read off `EngineThrustTable` at live ambient pressure. The model's aero force is subtracted first, so the bias holds only what the model misses. It is on by default (`SixDofBiasEnabled`).

### Soft path constraints from node 1

The glide slope, climb rate and turn rate start at node 1, since node 0 is the craft and cannot be changed. All soft limits carry L1 slacks, priced on their normalised violation so the weights are dimensionless. The acceleration limit applies at node 0 too, because the thrust there is still free. See [Soft constraints and slacks](index.md#soft-constraints-and-slacks).

### Defect gate

The solver's convergence test divides each defect by its state scale. The flight gate instead divides each by a tolerance in its own units:

| Channel | Tolerance |
| --- | --- |
| position | 1 m (`MaxDefectM`) |
| velocity | 0.5 m/s |
| attitude | 1 deg, as $\sin(0.5^\circ)$ per quaternion component |
| body rate | 0.01 rad/s |
| mass | $10^{-3}$ of the mass |

The intervals within the first second (`CommitHorizonS`) are judged at full strength. Beyond it the allowance rises linearly to 25 times at the last interval (`HorizonFarSlack`). An interval far out will be judged at full strength before it is flown. A cold plan's tolerances are 15 times looser (`ColdMaxDefectM`, 15 m). The plan is also refused if node 0 is more than 1 m from the craft. A refused plan leaves the craft flying the previous one.

### Two-grid schedule

Instead of a multi-rung node ladder down to coarse 5-node grids, touchdown uses a **two-grid schedule**:

- **Coast + burn (`N = 25`):** Used whenever the plan includes an unpowered coast before ignition (`ColdNodesFor`, `StartSixDofStandby`).
- **Burn-only (`N = 15`):** Clean single transition once the craft enters the powered burn, preserving terminal flare resolution while keeping every warm Clarabel solve well inside a single frame.

At the transition, the reference trajectory is resampled onto the new grid so the solver starts warm. After repeated refusals or a drift from the plan of more than 20 m or 5 % of height, the plan restarts cold.

### After the plan and safety guards

There is no planned hand-off to a hover controller: the plan itself ends at first contact. Outside the plan, the mode enforces strict safety guards:

- **Hover-aware climb cut.** Cuts the engine only when:

    $$
    \text{cut} \iff \big(\text{minTwr} > \texttt{SixDofHoverMaxMinTwr} \land v_z \ge 0\big) \;\lor\; \big(\text{noLivePlan} \land v_z > \texttt{SixDofClimbCutMs}\big),
    $$

    with `SixDofHoverMaxMinTwr` in $[0.90, 0.95]$ and `noLivePlan` true when the last published plan is absent or expired. Hoverable landers flying an active plan therefore absorb brief upward gusts via throttle modulation instead of cutting and relighting.
- **Plan end without contact.** Within 1 m of the plan's end height with no contact, a craft that can hover sinks to contact on its engine. One that cannot drops with the engine off when the drop ends below 3 m/s, and otherwise flies the no-plan law. One relight is allowed.
- **Unpowered-impact emergency backstop.** Whenever 3-DOF or 6-DOF drops its active plan and altitude reaches $h \le h_{\text{stop,margin}}$, an emergency `TerminalBraking` / G-FOLD suicide-burn backstop engages so propellant-bearing craft never impact unpowered. Below `SixDofLostPlanGateM` (50-120 m), a hoverable craft hands over to terminal hover while higher-altitude plan loss is caught by G-FOLD / `TerminalBraking`.

## Code map

| File | Main type or method | Role |
| --- | --- | --- |
| [Dynamics6Dof.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Dynamics6Dof.cs) | `Dynamics6Dof.F`, `Dynamics6Dof.Jacobian` | the dynamics and their Jacobians |
| [BodyAero6Dof.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/BodyAero6Dof.cs) | `BodyAero6Dof.Wrench` | body-axis air force and moment, fins, fades, moment limiter |
| [Scvx6DofConfig.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofConfig.cs) | `Scvx6DofConfig` | limits, weights and scales |
| [Scvx6DofSubproblemScs.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofSubproblemScs.cs) | `Scvx6DofSubproblemScs.Assemble` | builds one cone program |
| [Scvx6DofSolver.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofSolver.cs) | `Scvx6DofSolver.Iterate`, `TrueCost`, `WeightedDefect` | the SCvx loop, merit, trust region, defect |
| [ColdSeed6Dof.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/ColdSeed6Dof.cs) | `ColdSeed6Dof.Build`, `BuildWithCoast`, `BuildAlongPath` | cold seeds |
| [ColdReferenceShift.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/ColdReferenceShift.cs) | `ColdReferenceShift.Apply` | moves a reference onto the measured state |
| [PlanStateInterpolation.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/PlanStateInterpolation.cs) | `PlanStateInterpolation.SamplePositionVelocity` | cubic resampling of a plan |
| [TouchdownContact.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/TouchdownContact.cs) | `TouchdownContact.EarlyContactM` | the early contact that raises $h_f$ |
| [SiteSlope.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/SiteSlope.cs) | `SiteSlope.TipLimitRad`, `SiteSlope.FlatSearch` | site slope limits and the flat-ground search |
| [Ksa6DofSetup.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSetup.cs) | `Ksa6DofSetup.TryBuild` | measures the craft into a config and dynamics |
| [Ksa6DofGuidance.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofGuidance.cs) | `Ksa6DofGuidance.Update`, `StepCold`, `Finish` | warm and cold solves, pricing stages, the defect gate |
| [Ksa6DofSolveWorker.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSolveWorker.cs) | `Ksa6DofSolveWorker` | the worker thread |
| [SixDof.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs) | `GuidanceWindow.StepNodeGates`, `PlanEndState`, `NoPlanNearGround` | the mode: ladder, end state, bias, fallback rules |
| [SixDofStandby.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/SixDofStandby.cs) | `GuidanceWindow` (standby) | solving in the background before a hand-over |
