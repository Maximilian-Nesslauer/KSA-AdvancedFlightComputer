# Entry guidance (3-DOF)

Entry guidance flies a returning booster down through the atmosphere and plans its landing burn. It treats the craft as a point mass whose body axis is a control. One problem has two phases: an engine-off **glide**, steered by lift from the angle of attack, and then a single **burn** to the pad. The time of ignition is a free variable of the problem. Entry guidance engages **on standby** once the dynamic pressure of a falling boostback glide reaches 0.5 kPa (`ThreeDofStandbyQkPa`). From then on it plans and re-plans while the craft holds the glide the boostback planned. It **takes the glide** at 1 kPa (`ThreeDofEngageQkPa`, about 37 km on a typical return), or when the predicted time to impact falls under 45 s (`ThreeDofEngageTgoS`), whichever comes first. With touchdown guidance on (`ThreeDofSixDofHandover`), [touchdown guidance](touchdown.md) takes over the end of the descent. By default it owns the ignition (`SixDofOwnsIgnition`): it takes the craft in the glide, 5 s before entry guidance's planned ignition or 12 s before its planned touchdown, whichever is earlier. With `SixDofOwnsIgnition` off, entry guidance lights the engine itself and hands the burn over `ThreeDofSixDofTimeS` (30 s) before the burn's planned end, which for a typical landing burn means from ignition, once touchdown guidance's standby plan is ready. With touchdown guidance off, entry guidance flies the burn down to contact.

This page states the problem. The machinery that every SCvx solver in AFC shares is on the [common page](index.md).

## At a glance

| | |
|---|---|
| Frame | Landing-site frame: origin at the site, $+z$ radially up, turning with the body |
| State | $\mathbf x = (\mathbf r, \mathbf v, m) \in \mathbb R^7$ |
| Controls | $\mathbf u = (\mathbf b, \kappa) \in \mathbb R^4$: body axis (unit vector) and throttle fraction |
| Parameters | Phase durations $\sigma_G$ (glide, which is also the time to ignition) and $\sigma_B$ (burn) |
| Nodes | Two-phase problem: $N = 30$ nodes, $K = 12$ glide intervals and 17 burn intervals. Burn-only problem (after the ignition lock): 20 nodes, 19 burn intervals |
| Discretisation | Trapezoidal collocation, one pseudo-time step per phase, no sub-steps |
| Cone solver | SCS (ADMM), with a quadratic objective and second-order cones of size 4 |
| Where it runs | Its own worker thread (`Ksa3DofWorker`), one solve at a time. The sim thread reads only the immutable published plan |
| Re-solve cadence | Every 0.1 s of sim time: 3 SCvx iterations or 60 ms per cycle (200 ms after a refused cycle), whichever ends first |

## State and controls

$$
\mathbf x = \begin{bmatrix}\mathbf r\\ \mathbf v\\ m\end{bmatrix}\in\mathbb R^{7},
\qquad
\mathbf u = \begin{bmatrix}\mathbf b\\ \kappa\end{bmatrix}\in\mathbb R^{4},
\qquad
\sigma_G,\ \sigma_B \in \mathbb R_{\ge 0}.
$$

* $\mathbf r$: the position of the centre of mass (CoM) in the site frame (m). $r_z$ is the height above the site's tangent plane.
* $\mathbf v$: the velocity in the site frame (m/s). KSA's atmosphere turns with the body, so this is also the airspeed.
* $m$: the mass (kg).
* $\mathbf b$: the **body axis**, a unit vector along the thrust, pointing from the engines towards the nose. It is a control, not a state. The point-mass model has no attitude dynamics: the plan's rate and turn-acceleration limits stand in for them (see [Constraints](#constraints)).
* $\kappa$: the **throttle fraction**, not newtons. Thrust depends on the throttle and on the back pressure, so a fraction keeps the throttle box a box.
* $\sigma_G,\sigma_B$: the glide and burn durations (s).

**Attitude is the control.** The body axis $\mathbf b$ sets the thrust direction and the angle of attack. The angle of attack is measured **retrograde-first**, as the angle between the tail $-\mathbf b$ and the airflow, so it is zero when the booster flies engine-first into the wind. With $\hat{\mathbf v} = \mathbf v/\sqrt{\lVert\mathbf v\rVert^2+\varepsilon^2}$ and $\varepsilon^2 = 10^{-4}$ m^2/s^2, the body axis projected across the airflow is

$$
\mathbf p(\mathbf v,\mathbf b) = \mathbf b - (\mathbf b\cdot\hat{\mathbf v})\,\hat{\mathbf v},
\qquad
\lVert\mathbf p\rVert = \sin\alpha,
\qquad
\alpha = \operatorname{atan2}\!\big(\lVert\mathbf p\rVert,\ -\mathbf b\cdot\hat{\mathbf v}\big).
$$

Lift and drag depend on $\mathbf b$ and not on the thrust, so the attitude steers the craft with the engine off. That is why the glide is a phase worth planning.

Roll is not in the problem. A returning booster is held at a fixed roll, wings level along its ground track (`ReturnRollRefCci`). The model is given that roll reference so that it can read a body-axis aerodynamic table (see [Aerodynamics](#aerodynamics)).

## Dynamics

$$
\begin{aligned}
\dot{\mathbf r} &= \mathbf v \\
\dot{\mathbf v} &= \mathbf g(\mathbf r) \;-\; 2\,\boldsymbol\Omega\times\mathbf v \;-\; \boldsymbol\Omega\times\big(\boldsymbol\Omega\times\mathbf r_c\big) \;+\; \frac{1}{m}\Big(T(\kappa,h)\,\mathbf b + \mathbf F_{\text{aero}}(h,\mathbf v,\mathbf b)\Big) \\
\dot m &= -\dot m(\kappa)
\end{aligned}
$$

### Gravity and the rotating frame

Gravity is central, from the body's centre $\mathbf c$ (at $(0,0,-R_{\text{site}})$ in the site frame):

$$
\mathbf r_c = \mathbf r - \mathbf c,
\qquad
\mathbf g(\mathbf r) = -\mu\,\frac{\mathbf r_c}{\lVert\mathbf r_c\rVert^3},
\qquad
h = \lVert\mathbf r_c\rVert - R,
$$

with $\mu$ the gravitational parameter, $R$ the body's mean radius and $h$ the altitude that the atmosphere is read at. The site frame turns with the body at a constant $\boldsymbol\Omega$, so the site does not move and the terminal condition stays fixed whatever the landing time. The cost is the Coriolis term $-2\boldsymbol\Omega\times\mathbf v$ and the centrifugal term $-\boldsymbol\Omega\times(\boldsymbol\Omega\times\mathbf r_c)$. From tens of kilometres out, both these terms and the curvature drop matter. The same physics drives the boostback's impact prediction, so the prediction and the plan agree.

### Aerodynamics

The air comes from the game's atmosphere: density $\rho(h)$, pressure $p_a(h)$ and speed of sound, so Mach $M = \lVert\mathbf v\rVert / a(h)$. With $S$ the reference area,

$$
q = \tfrac12\,\rho(h)\,\lVert\mathbf v\rVert^2 .
$$

The coefficients come from the game's own forces, sampled by `KsaAeroSweep` over Mach and angle of attack. Between breakpoints they are C^2 cubic splines, so the linearisation sees smooth slopes. Mach is held inside each table's grid, because past the last breakpoint the fit's end slope has been seen to flip the sign of the lift.

**Whole-sphere tables.** Drag runs along $-\hat{\mathbf v}$ and lift along $-\mathbf p$:

$$
\mathbf D = -\,s_D\,q S\,C_D(M,\tilde\alpha)\,\hat{\mathbf v},
\qquad
\mathbf L = -\,s_L\,q S\,k(M,\tilde\alpha)\,\mathbf p,
\qquad
k = \frac{C_L}{\sin\alpha}.
$$

A positive $C_L$ lifts towards the side the engine end is tilted to. The engine end's tilt off the airflow is $-\mathbf p$, so the lift is along $-\mathbf p$. The lift is written through $k$, not as $C_L$ times a unit vector, so it has no direction singularity at $\alpha=0$ and keeps its slope there: a seed flown at exactly $\alpha=0$ still sees what a tilt would buy. At $\alpha=0$ the ratio is 0/0, so the value at the first non-zero breakpoint (1 deg on the sweep's grid) stands in for it.

**The corner at zero.** KSA's drag grows with $\lvert\sin\alpha\rvert$, which has a corner exactly where a booster flies. The tables are therefore read at a rounded angle $\tilde\alpha$, whose $\lVert\mathbf p\rVert$ is replaced by

$$
\lVert\mathbf p\rVert_e = \frac{\lVert\mathbf p\rVert^2}{\sqrt{\lVert\mathbf p\rVert^2 + e^2}},
\qquad e = 0.02 \ (\texttt{AlphaRounding}).
$$

This is exact at zero and smooth everywhere. Only the lookup argument is rounded; the lift's direction comes from $\mathbf p$ itself.

**Body-axis table.** When the sweep has a body-axis table and the roll is held, the model reads it instead wherever the craft is within 30 deg of engine-first (`BodyAeroMaxTiltRad`). The body axes follow from $\mathbf b$ and the held wing direction. The table gives the body-axis force coefficients $(C_x,C_y,C_z)(M,\alpha,\beta)$. The force $\mathbf F = qS(C_x\hat{\mathbf x}_b + C_y\hat{\mathbf y}_b + C_z\hat{\mathbf z}_b)$ is split into its part along the airflow, scaled by $s_D$, and its part across it, scaled by $s_L$.

**Learned corrections.** In the glide the engine is off, so the measured non-gravitational acceleration is the aerodynamic force alone. `AeroScaleEstimator` learns a drag scale $s_D$ from it and a cross-flow force coefficient $\mathbf C_\perp$ (an offset, not a ratio). Under thrust it freezes those and learns a thrust scale $s_T$. All three are low-passed and clamped, and handed to the next solve:

$$
\mathbf F_{\text{aero}} = \mathbf D + \mathbf L + q S\big(\mathbf C_\perp - (\mathbf C_\perp\cdot\hat{\mathbf v})\hat{\mathbf v}\big).
$$

$s_L$ stays at 1 in flight; the cross-flow offset does its job.

### Thrust and mass flow

Thrust acts along $\mathbf b$ only while the engine runs ($\kappa > 10^{-6}$):

$$
T(\kappa,h) = s_T\,T_{\text{tab}}\big(\kappa,\ p_a(h)\big),
\qquad
\dot m(\kappa) = \dot m_{\text{tab}}(\kappa).
$$

The throttle sets the chamber, not the thrust. Both tables are sampled from the game's engines once, at engage, over throttle (from the engines' minimum to full) and over ambient pressure (from vacuum to just past sea level), and fitted to cubic splines (`EngineThrustTable`). KSA's nozzle loses less to the air as the chamber pressure falls. A constant loss $\kappa F_{\text{vac}} - p_a A_e$ is exact only at full throttle and badly wrong at the floor, so the tables replace it. The mass flow follows the throttle and does not depend on the air. Without a table (offline checks) the model falls back to $T = s_T(\kappa F_{\text{vac}} - p_a A_e)$ and $\dot m = \kappa\,\dot m_{\max}$.

## Phases and free durations

The grid is split at node $K$. Glide intervals $0..K-1$ run engine-off, and burn intervals $K..N-2$ run with the engine lit. Each phase has its own pseudo-time $\tau\in[0,1]$ and its own duration, so the grid never moves while the durations are optimised:

$$
\frac{d\mathbf x}{d\tau} = \sigma_G\,\mathbf f(\mathbf x,\mathbf b,0)\quad\text{(glide)},
\qquad
\frac{d\mathbf x}{d\tau} = \sigma_B\,\mathbf f(\mathbf x,\mathbf b,\kappa)\quad\text{(burn)},
$$

with steps $\Delta\tau_G = 1/K$ and $\Delta\tau_B = 1/(N-1-K)$. Node $K$ is the ignition instant and belongs to both phases. The last glide interval sees it through the glide's dynamics and the first burn interval through the burn's.

**Ignition is placed by $\sigma_G$.** The time to ignition is one continuous variable, so the on/off decision needs no integers. The glide's dynamics are evaluated at $\kappa = 0$, where the thrust is switched off and the unlit nozzle has no loss. Their throttle column is zeroed, because the glide's throttle is not a control there. The burn never goes below its floor, so the engine-on switch is never crossed inside a phase. See [free final time and phases](index.md#free-final-time-and-phases).

In flight the problem changes shape once. When the time to ignition falls to `LockSeconds` (2.5 s), the ignition time is committed, and the guidance swaps to the **burn-only** problem: the same class with $K = 0$ and no $\sigma_G$. From then until ignition, node 0 is the state **predicted at the committed ignition**. This prediction carries the measured state engine-off (RK4, 0.05 s steps) along the attitude of the last two-phase plan, which is flown open loop until the engine lights. After ignition, node 0 is the measured state. See [What this solver does differently](#what-this-solver-does-differently).

## Boundary conditions

**Initial state** (hard):

$$
\mathbf x_0 = \mathbf x_{\text{meas}}\quad\text{(or the predicted ignition state while ignition is locked)}.
$$

$\mathbf b_0$ is a control, so it is not pinned. The [attitude anchor](#attitude-anchor) holds it near the commanded attitude.

**Terminal position and velocity.** The plan flies the CoM, so "legs on the pad, at rest" puts it the vehicle's height $h_{\text{veh}}$ (`VehicleHeightM`, measured from the colliders) above the site:

$$
\mathbf r_{N-1} = \mathbf r_f = (0,\,0,\,h_{\text{veh}}),
\qquad
\mathbf v_{N-1} = \mathbf v_f = \mathbf 0 .
$$

Both are **soft**, with L1 slacks:

$$
\mathbf r_{N-1} - (\mathbf s^+_r - \mathbf s^-_r) = \mathbf r_f,
\qquad
\mathbf v_{N-1} - (\mathbf s^+_v - \mathbf s^-_v) = \mathbf v_f,
\qquad
\mathbf s^\pm \ge 0 .
$$

Above a finite weight, an L1 slack is zero whenever the target can be reached. It opens only when the alternative is no plan at all. The miss is weighted ten times the speed (`TerminalMissWeight` $10^5$, `TerminalSpeedWeight` $10^4$, per unit of the channel's scale). A squeezed burn should arrive a little fast rather than stop at rest above the pad, which a booster that cannot throttle below its weight cannot recover from.

**Terminal attitude** (hard): upright, $\mathbf b_{N-1} = (0,0,1)$ (`TerminalAttitude`). The unit-attitude row is dropped at that node, where it would duplicate the pin.

**Terminal mass** is free.

## Constraints

| Constraint | Continuous form | In the subproblem | Code |
|---|---|---|---|
| Dynamics | $\dot{\mathbf x} = \sigma\,\mathbf f(\mathbf x,\mathbf u)$ per phase | Linearised, trapezoidal, virtual control $\boldsymbol\nu_k$ | `AssembleEqualities` |
| Unit attitude | $\lVert\mathbf b\rVert = 1$ | Tangent plane $\bar{\mathbf b}_k\cdot\mathbf b_k = 1$, re-projected on accept; hard | `AssembleEqualities` |
| Engine off in glide | $\kappa = 0$ for $t<\sigma_G$ | Equality; hard | `AssembleEqualities` |
| Throttle floor and ceiling | $\kappa_{\min}\le\kappa\le\kappa_{\max}$ in burn | Convex box as-is; hard | `AssembleCone` |
| Thrust acceleration | $T(\kappa,h)\le a_{\max}m$ | Thrust law linearised in $\kappa$ and $\mathbf r$, exact in $m$; soft | `AccelMax` |
| Angle of attack | $\lVert\mathbf p\rVert\le\sin\alpha_{\max}$ | SOC on linearised $\mathbf p$; soft | `AlphaMaxDeg` |
| $q\alpha$ (off) | $q\,\sin\alpha\le(q\alpha)_{\max}$ | Folded into the cone's radius at the reference $q$; soft | `QAlphaMax` |
| Retrograde | $\mathbf b\cdot\hat{\mathbf v}\le 0$ | Linear in $\mathbf b$, $\hat{\mathbf v}$ from the reference; soft | `Retrograde` |
| Ground floor | $r_z \ge h_{\text{veh}}$ | Linear bound from node 1; soft | `StateMin` |
| Dry mass | $m \ge m_{\text{dry}}$ | Linear bound from node 1; soft | `StateMin` |
| Turn rate | $\lVert\dot{\mathbf b}\rVert\le\omega_{\max}$ | Exact SOC with $\sigma$ in the head; soft | `AttitudeRateMax` |
| Fixed glide attitude | $\dot{\mathbf b} = 0$ in glide | Same cone with a zero head; hard | `GlideAttitudeFixed` |
| Turn acceleration | $\lVert\ddot{\mathbf b}\rVert\le\dot\omega_{\max}$ | SOC on the second difference, head linearised in $\sigma$; soft | `AttitudeAccelMax` |
| Attitude anchor | $\mathbf a\cdot\mathbf b(0)\ge\cos\theta_a$ | Linear; soft | `AttitudeAnchor` |
| Burn tilt (off) | $\angle(\mathbf b,\hat{\mathbf z})\le\theta_0+\theta'\,t_{\text{left}}$ | $b_z\ge\cos(\cdot)$, linearised in $\sigma_B$; soft | `BurnTiltPerS` |
| Phase durations | $\sigma\in[\sigma_{\min},\sigma_{\max}]$ | Box; hard | `SigmaRows` |
| Terminal state | $\mathbf r,\mathbf v$ at $N-1$ | Equality with L1 slack | `TerminalMissWeight` |
| Terminal attitude | $\mathbf b(t_f) = \hat{\mathbf z}$ | Equality; hard | `TerminalAttitude` |

"Soft" here means an L1 slack $s\ge 0$ in the constraint's own unit, priced at `PathSlackWeight` = $10^3$ per unit. The reason is the one on the common page: a hard constraint that the reference breaks by more than one trust radius leaves no feasible step at all. See [soft constraints and slacks](index.md#soft-constraints-and-slacks). The merit prices the same violations on the true geometry (the actual angle of attack and airflow direction), not the linearised one. A constraint whose speed condition fails ($\lVert\bar{\mathbf v}_k\rVert\le 5$ m/s, `AlphaSpeedThreshold`) is written as a vacuous row with explicit zeros, so the sparsity pattern never changes.

There is **no** heating, load-factor, dynamic-pressure or glide-slope constraint in this problem. The only g-limit is on the thrust acceleration. The aerodynamic load is limited only through the angle of attack (and the $q\alpha$ radius, when it is set).

### Dynamics

$$
\mathbf x_{k+1} = \mathbf x_k + \frac{\Delta\tau}{2}\big(\mathbf g_k + \mathbf g_{k+1}\big) + \boldsymbol\nu_k,
\qquad
\mathbf g_k = \sigma\,\bar{\mathbf f}_k + \bar\sigma\big(A_k(\mathbf x_k-\bar{\mathbf x}_k) + B_k(\mathbf u_k-\bar{\mathbf u}_k)\big),
$$

in each phase's own $\sigma$ and $\Delta\tau$. Here $A_k$ and $B_k$ are the Jacobians at the reference, computed with forward-mode dual numbers. $\sigma$ enters linearly with the coefficient $\bar{\mathbf f}$, so freeing the durations needs no extra Jacobian. $\boldsymbol\nu_k$ is the [virtual control](index.md#virtual-control). See [linearisation](index.md#linearisation) and [discretisation](index.md#discretisation).

### Unit attitude

$\lVert\mathbf b\rVert = 1$ is not convex. Each node takes the plane tangent to the sphere at the reference:

$$
\bar{\mathbf b}_k\cdot\mathbf b_k = 1 .
$$

On every accepted step, and on every seed, each $\bar{\mathbf b}_k$ is re-projected onto the unit sphere. Without that, the reference drifts off the sphere until the tangent plane and a tight trust region disagree, and the solver fails. At a fixed point ($\mathbf b = \bar{\mathbf b}$) the row is exact.

### Throttle floor and ceiling

A real engine is either off or between its minimum and maximum thrust: the set $\{0\}\cup[T_{\min},T_{\max}]$, which has a hole in it. Here the off state is the glide phase, and the direction lives in $\mathbf b$, so the burn's magnitude is a scalar in a plain interval:

$$
\underbrace{\kappa_k = 0}_{\text{glide},\ k<K},
\qquad
\underbrace{\kappa_{\min}\le\kappa_k\le\kappa_{\max}}_{\text{burn},\ k\ge K}.
$$

This is convex as it stands. It needs no lossless-convexification slack and no integers. The price is moved to the unit-attitude constraint above.

* $\kappa_{\min}$ (`ThrottleFloor`) is the engines' minimum plus 5 % of the thrust range above it at the site's pressure (`LandingPlanFloorMargin`), so the burn tracker has thrust to take off as well as to add. Where the engines can throttle below the craft's weight, it is held under the hover line.
* $\kappa_{\max}$ (`ThrottleCeiling`) is 0.88 before the ignition lock and 0.95 after it (`ThreeDofGlideCeiling`, `ThreeDofBurnCeiling`). These are two stacked margins: the headroom a committed ignition needs to absorb arriving late, and the share that `BurnTracker3Dof` may use between re-solves.

### Thrust acceleration

$$
T(\kappa_k, h_k) \le a_{\max}\,m_k \qquad (k \ge K).
$$

The thrust law is linearised about the reference, in throttle and in position (the back-pressure loss changes with altitude):

$$
T \approx T'_\kappa\,\kappa - \big(\ell_0 + \nabla\ell\cdot\mathbf r\big),
$$

where $T'_\kappa$ is the slope in throttle and $\ell_0$, $\nabla\ell$ are the loss and its gradient. The row is divided by $T'_\kappa$ to put it in throttle units, and is exact in $m$. The position term matters: without it, the row ignored that a step up or down changes the loss, while the merit measured it. Near the optimum, with the limit active, every step was then rejected. In flight, $a_{\max}$ is `LandingAccelLimitG` (3 g) for the burn-only problem. The two-phase problem, which times the ignition, uses 3 g / 1.3 (`AccelHeadroom`), so a burn lit on that timing has headroom left. Both are floored at 1.05 times what the throttle floor already gives at the present mass.

### Angle of attack

$$
\lVert\mathbf p(\mathbf v,\mathbf b)\rVert \le \sin\alpha_{\max},
\qquad \alpha_{\max} = 7.5^\circ\ (\texttt{GlideMaxAoaDeg}),
$$

at every node, glide and burn, where the reference speed exceeds 5 m/s. It is a second-order cone on the projection linearised in $\mathbf v$ and $\mathbf b$:

$$
\Big\lVert\,\bar{\mathbf p}_k + \tfrac{\partial\mathbf p}{\partial\mathbf v}\big(\mathbf v_k-\bar{\mathbf v}_k\big) + \tfrac{\partial\mathbf p}{\partial\mathbf b}\big(\mathbf b_k-\bar{\mathbf b}_k\big)\Big\rVert \le \sin\alpha_{\max} + s_{\alpha,k}.
$$

The limit bounds $\lVert\mathbf p\rVert$ rather than $\cos\alpha$, because $\mathbf p$ has a non-zero slope even when $\mathbf b$ lies along the airflow, where $\cos\alpha$ is stationary.

When `QAlphaMax` is set, the radius becomes $\min\big(\sin\alpha_{\max},\ (q\alpha)_{\max}/\bar q_k\big)$, with $\bar q_k$ read at the reference. It is off in flight.

!!! note "Planned glide angle"
    The boostback plans its glide at 2.5 deg (`GlideNominalAoaDeg`), with the lift up the trajectory. That angle is **not** a constraint here. It only sets the tilt that the cold seed's glide is flown at. The problem itself may use any angle up to 7.5 deg.

### Retrograde

The booster flies engine first:

$$
\mathbf b_k\cdot\hat{\bar{\mathbf v}}_k \le s_{r,k},
$$

with the airflow direction taken from the reference. The row is linear in $\mathbf b$ and applies where the reference speed exceeds 5 m/s.

### Ground floor and dry mass

From node 1 onward:

$$
r_{z,k} + s \ge h_{\text{veh}},
\qquad
m_k + s \ge m_{\text{dry}},
$$

with the slack in the channel's scale. These are linear as they stand. Node 0 is pinned to the measured state, so a bound that node 0 already breaks would make the problem infeasible rather than expensive; that is why the bounds start at node 1. $r_z$ is measured from the site's tangent plane, which lies above the curved ground away from the site, so the floor errs on the safe side.

### Turn rate

$$
\lVert\dot{\mathbf b}\rVert \le \omega_{\max}
\quad\longrightarrow\quad
\lVert\mathbf b_{k+1}-\mathbf b_k\rVert \le \omega_{\max}\,\sigma\,\Delta\tau + s_{\omega,k}.
$$

The interval length is affine in the variable $\sigma$, so this is an **exact** second-order cone, with no linearisation. $\omega_{\max}$ is 10 deg/s, or 0.8 of the flight computer's own rate limit if that is lower (`PlanRateMax`).

**Fixed glide attitude.** On a craft with no RCS to turn it with the engine off, and with `ThreeDofTurnAccelDegS2` at 0, the glide's cones get a zero head and no slack: $\mathbf b_{k+1} = \mathbf b_k$ through the glide (`GlideAttitudeFixed`).

### Turn acceleration

A rate limit alone lets the plan change its turn rate instantly, which no vehicle can do. The second difference is bounded at every node:

$$
\lVert\ddot{\mathbf b}\rVert\le\dot\omega_{\max}
\quad\longrightarrow\quad
\lVert\mathbf b_{k+1}-2\mathbf b_k+\mathbf b_{k-1}\rVert \le \dot\omega_{\max}\,(\sigma\Delta\tau)^2 + s_{\dot\omega,k}.
$$

$(\sigma\Delta\tau)^2$ is not affine in $\sigma$. The head takes its tangent at the reference, $\dot\omega_{\max}\Delta\tau^2(2\bar\sigma\sigma-\bar\sigma^2)$, which lies under the true value. A step is therefore held to a slightly tighter limit than the true one, and the row is exact at convergence. Node 0 continues the measured turn rate $\dot{\mathbf b}_0 = \boldsymbol\omega\times\mathbf b$, as if the axis had been at $\mathbf b_0 - \dot{\mathbf b}_0 h$ one interval before. The last node stops, as if the axis held still one interval after. At the join between glide and burn, where the two interval lengths differ, the stencil and the head use the reference lengths. $\dot\omega_{\max}$ is 0.75 of the weaker lateral RCS authority, or of the gimbal authority on a craft with no RCS (`TurnAccelLimit`).

### Attitude anchor

Node 0's body axis is a control, not a measured state, so without a limit a plan could open with an instant rotation. It is held in a cone about a direction $\mathbf a$:

$$
\mathbf a\cdot\mathbf b_0 + s_a \ge \cos\theta_a,
\qquad
\theta_a = \max\!\big(0.3\text{ s}\times\omega_{\max},\ 1^\circ\big).
$$

This is linear, and exact to the extent that $\lVert\mathbf b\rVert = 1$ is. $\mathbf a$ is the attitude last commanded. While ignition is locked, it is the locked plan's attitude at ignition. The cold solve releases the anchor.

### Burn tilt

Off by default (`ThreeDofBurnTiltPerSDeg` = 0). When it is on, a burn node with $t_{\text{left}} = \sigma_B(1-\tau_k)$ seconds of burn left may lean no more than $\theta(t) = \min(\theta_0 + \theta' t,\ 60^\circ)$ off vertical, with $\theta_0 = 2^\circ$:

$$
b_{z,k} + s \ge \cos\theta\big(\sigma_B(1-\tau_k)\big).
$$

The cosine is not affine in $\sigma_B$, so it takes its tangent at $\bar\sigma_B$. The limit bounds the sideways speed the burn can take off, so the glide has to arrive nearer vertical.

### Phase durations

$$
0 \le \sigma_G \le 400\text{ s},
\qquad
1\text{ s} \le \sigma_B \le 120\text{ s},
$$

hard boxes (`GlideSigmaMax`, `BurnSigmaMin`, `BurnSigmaMax`). Once the burn runs, its floor drops to `MinimumBurnTime` (0.25 s), so a burn counted down below 1 s is still feasible.

## Objective

**Continuous.** Minimum fuel, or equally the largest final mass:

$$
\min_{\mathbf b(\cdot),\,\kappa(\cdot),\,\sigma_G,\,\sigma_B}\ \frac{m_0 - m(t_f)}{m_0}.
$$

**Discrete.** The fuel fraction plus rate smoothing, the virtual-control penalty, the terminal slacks and the path slacks:

$$
\begin{aligned}
J ={}& \frac{m_0 - m_{N-1}}{m_0}
+ w_\kappa\sum_{k=K}^{N-2}\big(\kappa_{k+1}-\kappa_k\big)^2
+ w_b\sum_{k=0}^{N-2}\lVert\mathbf b_{k+1}-\mathbf b_k\rVert^2 \\
&+ \rho_{vc}\sum_{k=0}^{N-2}\big\lVert\boldsymbol\nu_k\oslash\mathbf x_{\text{scale}}\big\rVert^2
+ w_r\,\mathbf 1^\top\!\big(\mathbf s^+_r+\mathbf s^-_r\big)\oslash\mathbf x_{\text{scale},r}
+ w_v\,\mathbf 1^\top\!\big(\mathbf s^+_v+\mathbf s^-_v\big)\oslash\mathbf x_{\text{scale},v}
+ w_{\text{path}}\sum s_{\text{path}} .
\end{aligned}
$$

| Term | Config | Flight value |
|---|---|---|
| Throttle rate, burn only | `WThrottleRate` | 0.05 |
| Attitude rate, every interval | `WAttitudeRate` | 0.10 |
| Virtual control (quadratic) | `RhoVc` | $10^5$ |
| Terminal position (L1) | `TerminalMissWeight` | $10^5$ |
| Terminal velocity (L1) | `TerminalSpeedWeight` | $10^4$ |
| Path slacks (L1) | `PathSlackWeight` | $10^3$ |
| Proximal term | `ProximalWeight` | 0 (off) |

The smoothing prices the **rate** of the throttle and the attitude, not their magnitude. A magnitude penalty $\sum\kappa^2$ equals $I^2/T_{\text{burn}}$ for a fixed impulse $I$, so it falls as the burn gets longer: it would pay the optimiser to stretch the burn and pin $\sigma_B$ at its bound. The rate penalty has no such bias. The virtual control is priced on the state scales $\mathbf x_{\text{scale}}$ (`VirtualControlScale` is left unset here, unlike touchdown guidance's). Priced on a fixed 5 m/s, the locked burn solves ran out of ADMM iterations.

## What this solver does differently

These are this solver's settings for the [common machinery](index.md), and the tricks of its own.

**Trust region.** The region is a hard $\infty$-norm box on every scaled state, every control and both durations: $\lvert x-\bar x\rvert\le\eta\,x_{\text{scale}}$, $\lvert u-\bar u\rvert\le\eta$ (the body axis and the throttle are already of order one), and $\lvert\sigma-\bar\sigma\rvert\le\eta\,\sigma_{\text{scale}}$. $\eta\in[10^{-3}, 0.1]$. A cold solve starts at 0.1, a warm cycle at 0.05 (`WarmTrustRegion`), and a cycle after a refused one at 0.1. The update uses ratios $\rho_0 = 0$, $\rho_1 = 0.25$, $\rho_2 = 0.7$: shrink by 0.5 below $\rho_1$; grow by 1.5 at or above $\rho_2$, but only when the step used at least 0.8 of the radius. See [trust region](index.md#trust-region) and [accepting a step](index.md#accepting-a-step).

**Merit and convergence.** The merit uses the true trapezoidal defect, the terminal miss measured on the trajectory, and the path violations measured on the true geometry. It carries the same fuel, smoothing and slack terms as the subproblem, so the ratio compares like with like. A step converges when it is accepted, its largest scaled move in state and duration is under `StepTolerance` = $8\times10^{-3}$, and the largest scaled true defect is under `DefectTolerance` = $10^{-3}$. A subproblem that fails, or that only ran out of ADMM iterations, is never taken as a step: the radius shrinks, and the loop stops if it is already at the floor. See [convergence](index.md#convergence).

**Cone solver.** The subproblem runs in SCS to $\epsilon_{\text{abs}} = \epsilon_{\text{rel}} = 10^{-4}$ (`RealTimeEps`). It is warm-started from the previous solution across iterations and across receding-horizon cycles, and reset on a cold start and on the first burn-only solve. The structure is built once and refilled in place each iteration. Columns are scaled to physical units, and rows are then equilibrated, with every row of a cone sharing one scale. See [scaling](index.md#scaling) and [the cone program](index.md#the-cone-program).

**Scaling sized to the problem left.** `Scaling` = `EverySolve`. Before every solve, the length scales are set per axis: the horizontal range to the aim point (at least a tenth of the height, and at least 30 m) for $x$ and $y$, and the height for $z$. One scale for all three axes left a 2 km cross-range to be resolved in 30 km units, and that cold solve stopped converging. The speed scale is the larger of the speed and $\sqrt{gh}$, and the mass scale is the present mass. The duration scale is set twice and is never resized with the time left. The cold solve uses the seed's whole duration, clamped to 30-60 s. From the first published plan on, it uses that plan's burn time, clamped to 5-30 s. A duration scale sized from the time left let the ignition time wander two seconds either way between cycles.

**Fixed node counts.** There are always 12 glide intervals and 17 burn intervals, wherever entry guidance takes over. From 37 km each glide interval is about 8 s long; the intervals shrink back to the sizes flown for months as the horizon closes. More intervals cost 0.7-0.9 s per re-solve up high.

**Cold ADMM cap on retry.** A cold solve gets 20 000 ADMM iterations per subproblem (`ColdAdmmCap`) and up to 150 SCvx iterations (`ColdIterations`), run two at a time on the worker. A warm cycle gets 2 000 (`WarmAdmmCap`), with one escalated retry at 8 000 (`EscalatedAdmmCap`) for a subproblem that only ran out of iterations. While the glide left is longer than 40 s (`LongGlideS`), that retry gets the cold cap instead. With 8 s intervals the subproblem is harder, and warm re-solves that hit the 8 000 cap were refused 20 at a time.

**Stalled cold solve.** A cold solve whose last 10 steps were all rejected at the smallest radius can still hold a plan: its defect sits just above the normalised test but inside the physical gate. Such a plan is published if it passes the gate below.

**Speed-scaled plan gate.** A plan is published only if its true defect passes a per-channel gate: 2 m, 0.5 m/s and 5 kg (`DefectTolerance`). Intervals in the first 2 s (`CommitHorizonS`) are judged at full strength; the allowance then grows linearly to 25 times at the last interval (`HorizonFarSlack`). The position and velocity tolerances also grow with the speed at the plan's start: at least 0.01 s of it for position (`PositionToleranceS`) and 0.2 % of it for velocity (`VelocityToleranceShare`). At 600 m/s that is 6 m and 1.2 m/s. Without the speed term, a glide taken over at 45 km had its plans refused for a 0.6 m/s defect that the next re-solve took back. A refused cycle keeps the previous plan.

**Standby planning.** From 0.5 kPa the guidance solves cold and then re-solves every cycle, but the craft keeps holding the boostback's planned glide. At 1 kPa it takes the glide with a plan already in hand, and the take-over does not wait for a cold solve.

**The ignition lock.** At 2.5 s to ignition (`LockSeconds`), the ignition time is committed and the two-phase plan is flown open loop up to it. The burn-only problem is seeded from that plan's burn and gets 12 iterations for its first solve (`SwapIterations`). Until it has its own plan, the old plan's burn, re-timed to the committed ignition, is published in its place, so there is never a moment without a plan. The anchor and the turn-rate anchor are those of the locked plan at ignition. At ignition the prediction horizon is zero and node 0 becomes the measured state, so nothing switches.

**Re-solving in flight.** Each cycle seeds from the last published plan, resampled onto the new horizon. Position and velocity come from cubic Hermite segments; mass and controls are interpolated linearly. Node 0 is set to the measured state, the later masses are shifted by the same amount, and node 0's body axis is set to the last commanded attitude. The glide's duration is reduced by the time elapsed. A cold solve is seeded by `Seed3Dof`. It integrates the glide engine-off at the planned glide angle, finds the latest ignition from which a simulated retrograde burn at 70 % thrust (drag included) still stops above the target, ramps that burn onto the target, and slews the attitudes forward at the rate limit. Ten refusals in a row in the glide drop back to the boostback's glide for a cold retry. See [re-solving in flight](index.md#re-solving-in-flight).

**Least-rolling-moment roll.** This is not in the problem. When `ThreeDofLeastRollMoment` is on (off by default), the flight side rolls the held wing direction towards the roll at which the air's rolling moment is zero and restoring (`UpdateGlideRoll`). The problem sees roll only through the body-axis table's roll reference, which the model takes when entry guidance engages.

## Code map

| File | Main type or method | What it holds |
|---|---|---|
| [`Scvx3DofConfig.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Scvx3DofConfig.cs) | `Scvx3DofConfig` | Problem shape, limits, weights, scales |
| [`PointMass3Dof.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/PointMass3Dof.cs) | `PointMass3Dof.Jacobian`, `ProjectionJacobian` | State layout, dual-number Jacobians, the angle-of-attack projection |
| [`KsaPointMassModel.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/KsaPointMassModel.cs) | `KsaPointMassModel.F`, `AeroForce` | Gravity, rotating frame, aerodynamics, thrust and flow |
| [`EngineThrustTable.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/EngineThrustTable.cs) | `EngineThrustTable` | Thrust over throttle and pressure, flow over throttle |
| [`Scvx3DofSubproblemScs.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Scvx3DofSubproblemScs.cs) | `Scvx3DofSubproblemScs.Assemble` | Objective, equalities, cones, scaling |
| [`Scvx3DofSolver.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Scvx3DofSolver.cs) | `Scvx3DofSolver.Iterate`, `TrueCost` | Linearisation, ratio test, trust region, merit, convergence |
| [`Seed3Dof.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Seed3Dof.cs) | `Seed3Dof.TryBuild` | Cold seed: simulated glide, ignition search, burn |
| [`Resample3Dof.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Resample3Dof.cs) | `Resample3Dof.Resample` | Reading a plan between nodes and moving it onto a new grid |
| [`Guidance3Dof.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Guidance3Dof.cs) | `Guidance3Dof`, `AeroScaleEstimator` | Receding horizon, lock, gate, rescaling, model corrections |
| [`BurnTracker3Dof.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/BurnTracker3Dof.cs) | `BurnTracker3Dof` | Flying a burn plan between re-solves |
| [`Toy3DofModel.cs`](../../../AdvancedFlightComputer/Features/Guidance/Scvx/ThreeDof/Toy3DofModel.cs) | `Toy3DofModel` | The `3dof.py` physics, for diffing the port offline |
| [`Ksa3DofSetup.cs`](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/ThreeDof/Ksa3DofSetup.cs) | `Ksa3DofSetup.Config`, `BuildModel` | Flight configs and the model built from the live craft |
| [`ThreeDof.cs`](../../../AdvancedFlightComputer/Features/Guidance/Modes/ThreeDof.cs) | `StartThreeDofCold` | The flight mode: engage, standby, cadence, hand-over to touchdown guidance |
