# `3dof.py` — 3-DOF Powered-Descent Guidance via Successive Convexification

A minimal but complete **Successive Convexification (SCvx)** solver for a Falcon-9-style
powered-descent landing. The vehicle **coasts** (engine off, steering aerodynamically), then flies
**engine-first (retrograde)** through a single landing **burn** down to a pad — holding angle of
attack within a stall limit, respecting a **40 % throttle floor**, minimising fuel, with **free
phase durations**.

The defining idea: **we hand-write only the nonlinear dynamics $\mathbf f$; every Jacobian is
obtained by automatic differentiation (JAX).** SCvx then repeatedly linearises about a reference
trajectory, solves a convex second-order-cone program (SOCP), and updates the reference.

---

## 1. State, control, and free phase times

$$
\mathbf x = \begin{bmatrix}\mathbf r\\ \mathbf v\\ m\end{bmatrix}\in\mathbb R^{7},
\qquad
\mathbf u = (\mathbf b,\ T_m) \in\mathbb R^{3}\times\mathbb R,
\qquad
\sigma_{\text{coast}},\ \sigma_{\text{burn}} \in \mathbb R_{>0}.
$$

* $\mathbf r$ — position (m), with altitude $h = r_z$
* $\mathbf v$ — velocity (m/s)
* $m$ — mass (kg)
* $\mathbf b$ — **attitude**: the body-axis unit vector, $\lVert\mathbf b\rVert=1$ (an independent control)
* $T_m$ — **throttle magnitude** (N), a scalar
* $\sigma_{\text{coast}},\sigma_{\text{burn}}$ — the two **free phase durations** (s)

**Attitude is the control.** The body axis $\mathbf b$ sets **both** the thrust direction
($\mathbf T = T_m\,\mathbf b$) **and** the angle of attack (hence lift). Crucially, lift depends on
$\mathbf b$ alone, *not* on thrust — so the vehicle generates aerodynamic lift via attitude **even
with the engine off** (entry/coast steering, like grid fins). The trajectory has **two phases**: a
**coast** ($T_m=0$, attitude still makes lift) followed by a single **burn**.

---

## 2. System of equations (continuous, nonlinear)

$$
\begin{aligned}
\dot{\mathbf r} &= \mathbf v \\
\dot{\mathbf v} &= \mathbf g + \frac{1}{m}\left(T_m\,\mathbf b + \mathbf D + \mathbf L\right) \\
\dot{m} &= -\frac{1}{I_{sp}\,g_0}\,T_m
\end{aligned}
$$

Thrust is $\mathbf T = T_m\,\mathbf b$ (magnitude $\times$ body axis) and mass burns against $T_m$.
During the coast $T_m=0$ (no thrust, no fuel burn), but the attitude $\mathbf b$ still produces
lift and drag. With gravity $\mathbf g = [0,0,-g_0]^\top$, let

$$
\hat{\mathbf v} = \frac{\mathbf v}{\lVert\mathbf v\rVert},\qquad
\rho(h) = \rho_0\,e^{-h/H},\qquad
q = \tfrac12\,\rho(h)\,\lVert\mathbf v\rVert^2 .
$$

**Angle of attack and lift** — from the attitude $\mathbf b$. The body axis projected onto the
plane orthogonal to $\mathbf v$ is

$$
\mathbf p = \mathbf b - (\mathbf b\cdot\hat{\mathbf v})\,\hat{\mathbf v},
\qquad \lVert\mathbf p\rVert = \sin\alpha ,
$$

where $\alpha$ is the angle of attack. Lift acts perpendicular to $\mathbf v$, on the side set by
whether we fly prograde or retrograde — **and is independent of thrust** (so it is non-zero in coast):

$$
C_L = C_{L\alpha}\,\lVert\mathbf p\rVert,\qquad
\mathbf L = q\,S\,C_{L\alpha}\,\mathrm{sgn}(\mathbf b\cdot\hat{\mathbf v})\,\mathbf p .
$$

The sign factor is crucial: in **retrograde** flight ($\mathbf b\cdot\hat{\mathbf v}<0$) lift acts
**opposite** to the lateral component of the body axis, so tilting to steer costs you.

**Drag** (with induced drag through the polar):

$$
C_D = C_{D0} + k\,C_L^2,\qquad
\mathbf D = -\,q\,S\,C_D\,\hat{\mathbf v}.
$$

A small $\varepsilon$ softens $\lVert\mathbf v\rVert\to0$, and $C_D$ is written with
$\lVert\mathbf p\rVert^2$ (not a bare $\sqrt{\,\cdot\,}$) so the autodiff never sees an infinite
gradient at $\mathbf p=0$.

### Two phases with free durations (time dilation)

The horizon is split at a fixed node index $K$: nodes $0\!:\!K$ are the **coast**, nodes
$K\!:\!N\!-\!1$ are the **burn**. Each phase has its own pseudo-time $\tau\in[0,1]$ and its own free
duration, so the node grid never moves while the durations are optimised:

$$
\frac{d\mathbf x}{d\tau} = \sigma_{\text{coast}}\,\mathbf f(\mathbf x,\mathbf b,0)
\quad\text{(coast)},
\qquad
\frac{d\mathbf x}{d\tau} = \sigma_{\text{burn}}\,\mathbf f(\mathbf x,\mathbf b,T_m)
\quad\text{(burn)} .
$$

The **ignition time is just** $\sigma_{\text{coast}}$ — a single continuous variable — so the on/off
decision needs **no integers**. The *same* $\mathbf f$ serves both phases: with $T_m=0$ it is the
coast (no thrust, no fuel burn) — but because lift comes from $\mathbf b$, the coast still steers.

---

## 2b. Minimum thrust — convex, because attitude is separated

A real engine obeys $T_{\min}\le\lVert\mathbf T\rVert\le T_{\max}$ when lit ($T_{\min}=0.4\,T_{\max}$),
**or** is off — a non-convex set $\{0\}\cup[T_{\min},T_{\max}]$ with a forbidden hole.

Separating attitude from thrust dissolves the hole. The thrust *magnitude* is now the **scalar** $T_m$
(direction is carried by $\mathbf b$), so the floor is just a convex box:

$$
\underbrace{T_m = 0}_{\text{coast (off)}},
\qquad
\underbrace{T_{\min}\le T_m \le T_{\max}}_{\text{burn (on)}} .
$$

No lossless-convexification slack and no integers are needed — the **off** state is the coast phase,
and **on** is a scalar interval. (Earlier versions used an LCvx slack $\Gamma$ on a thrust *vector*;
making attitude an explicit control removed it and the ignition-seam slack that came with it.)

The cost moves to a new, milder non-convexity: the unit-attitude constraint $\lVert\mathbf b\rVert=1$,
linearised about the reference as the tangent plane

$$
\bar{\mathbf b}\cdot\mathbf b = 1 ,
$$

with $\bar{\mathbf b}$ **re-projected onto the unit sphere each iteration**
($\bar{\mathbf b}\leftarrow\bar{\mathbf b}/\lVert\bar{\mathbf b}\rVert$). Without that re-projection the
reference drifts off the sphere and the tangent-plane constraint becomes inconsistent with a tight
trust region — the solver then fails. The constraint is exact at the fixed point ($\bar{\mathbf b}=\mathbf b\Rightarrow\lVert\mathbf b\rVert=1$).

---

## 3. Linearisation

Each iteration linearises about the current reference $(\bar{\mathbf x},\bar{\mathbf u},\bar\sigma)$.
Define the Jacobians (computed by JAX, **never by hand**)

$$
A = \frac{\partial \mathbf f}{\partial \mathbf x},
\qquad
B_b = \frac{\partial \mathbf f}{\partial \mathbf b},
\qquad
B_t = \frac{\partial \mathbf f}{\partial T_m},
\qquad
\mathbf f_0 = \mathbf f(\bar{\mathbf x},\bar{\mathbf b},\bar T_m),
$$

all evaluated at the reference. The time-dilated right-hand side $\,\sigma\,\mathbf f\,$ is **bilinear**
in $(\sigma,\mathbf f)$ — and $\mathbf f$ itself is bilinear in $(T_m,\mathbf b)$ through the thrust
$T_m\mathbf b$; JAX linearises all of it. Collapsing the constant and $\sigma$-linear terms:

$$
\sigma\,\mathbf f \;\approx\; \sigma\,\mathbf f_0 \;+\; \bar\sigma\big(A\,\delta\mathbf x + B_b\,\delta\mathbf b + B_t\,\delta T_m\big).
$$

So $\sigma$ enters each dynamics constraint **linearly** with coefficient $\mathbf f_0$ — freeing the
durations costs no extra Jacobian machinery. Coast nodes use the $T_m=0$ Jacobians and
$\sigma_{\text{coast}}$ (the attitude Jacobian $B_b$ is still active there — that is what gives coast
lift); burn nodes use $\sigma_{\text{burn}}$.

The angle-of-attack projection $\mathbf p(\mathbf x,\mathbf b)$ is linearised the same way
(JAX gives $\partial\mathbf p/\partial\mathbf x,\ \partial\mathbf p/\partial\mathbf b$).

---

## 4. Discretisation and the virtual control (w)

Trapezoidal (Crank–Nicolson) integration, with **per-phase** pseudo-time steps
$\Delta\tau_{\text{coast}}=1/K$ and $\Delta\tau_{\text{burn}}=1/(N\!-\!1\!-\!K)$, and a **virtual
control** $\mathbf w_n$ added to the defect:

$$
\mathbf x_{n+1} = \mathbf x_n + \frac{\Delta\tau_{(\cdot)}}{2}\big(\mathbf g_n + \mathbf g_{n+1}\big) + \mathbf w_n,
\qquad
\mathbf g_n = \sigma_{(\cdot)}\,\mathbf f_{0,n} + \bar\sigma_{(\cdot)}\big(A_n\,\delta\mathbf x_n + B_{b,n}\,\delta\mathbf b_n + B_{t,n}\,\delta T_{m,n}\big),
$$

where $(\cdot)$ is `coast` for $n<K$ and `burn` for $n\ge K$.

$\mathbf w_n$ is an **artificial-infeasibility / exact-penalty** slack. It guarantees the convex
subproblem is **always feasible** no matter how poor the reference or how small the trust region.
At convergence $\mathbf w\to\mathbf 0$, meaning the *true* nonlinear dynamics are satisfied; if
$\mathbf w$ refuses to vanish, the problem is genuinely infeasible (see §8).

---

## 5. Constraints

Boundary conditions (mass at touchdown is free):

$$
\mathbf x_0 = [\mathbf r_0,\mathbf v_0,m_0]^\top,\qquad
\mathbf r_{N-1} = \mathbf r_f,\qquad
\mathbf v_{N-1} = \mathbf v_f .
$$

Unit attitude (linearised sphere, every node): $\ \bar{\mathbf b}_n\cdot\mathbf b_n = 1$.

Throttle. **Coast** nodes ($n<K$) have the engine off; **burn** nodes ($n\ge K$) sit in the convex box:

$$
\underbrace{T_{m,n} = 0}_{\text{coast } (n<K)}
\qquad\qquad
\underbrace{T_{\min}\le T_{m,n} \le T_{\max}}_{\text{burn } (n\ge K)} .
$$

Angle-of-attack (stall) limit, at **all** nodes with meaningful speed
($\lVert\bar{\mathbf v}_n\rVert > v_{\text{thr}}$) — including the coast, since attitude steers there.
Using the linearised projection $\mathbf p^{\mathrm{lin}}_n$:

$$
\lVert\mathbf p^{\mathrm{lin}}_n\rVert \le \sin\alpha_{\max}.
$$

This is bounded as an SOC rather than via $\cos\alpha$ because $\mathbf p$ has a non-zero Jacobian
even when $\mathbf b$ is aligned with $\mathbf v$ (where $\cos\alpha$ is stationary).

Retrograde body (the vehicle flies engine/base-first), with $\hat{\mathbf v}_n = \bar{\mathbf v}_n/\lVert\bar{\mathbf v}_n\rVert$:

$$
\mathbf b_n\cdot\hat{\mathbf v}_n \le 0 .
$$

Phase-duration bounds:

$$
0 \le \sigma_{\text{coast}} \le \sigma_{\text{coast}}^{\max},
\qquad
\sigma_{\text{burn}}^{\min} \le \sigma_{\text{burn}} \le \sigma_{\text{burn}}^{\max}.
$$

---

## 6. Objective (minimum fuel)

$$
\min\;
\underbrace{\frac{m_0 - m_{N-1}}{m_0}}_{\text{fuel fraction}}
\;+\;
\underbrace{w_{\Delta T}\!\sum_{n\ge K}\!\Big(\tfrac{T_{m,n+1}-T_{m,n}}{T_{\max}}\Big)^2
+ w_{\Delta b}\!\sum_{n}\!\lVert \mathbf b_{n+1}-\mathbf b_n\rVert^2}_{\text{RATE smoothing (anti-chatter)}}
  \;+\;
  \underbrace{\rho_{vc}\sum_{n}\big\lVert \mathbf w_n \oslash \mathbf x_{\text{scale}}\big\rVert^2}_{\text{virtual-control penalty}}
  $$

Minimum fuel $\equiv$ maximise final mass. The large $\rho_{vc}$ drives $\mathbf w\to\mathbf 0$.

The smoothing penalises the **rate** of the throttle and attitude, **not their magnitude**. This
matters: a magnitude penalty $\sum T_m^2$ equals $I^2/T_{\text{burn}}$ for fixed impulse $I$, which
*decreases* with burn length — it secretly pays the optimiser to stretch the burn (and pins
$\sigma_{\text{burn}}$ at its bound, burning more fuel). The rate penalty has no such bias: it damps
node-to-node chatter while leaving the true bang-bang min-fuel profile (a clean ramp to 100 %) intact.

---

## 7. Trust region and the radius update

All deviations (state, attitude $\mathbf b$, throttle $T_m$, and **both** durations) are
non-dimensionalised and bounded by a **single hard radius** $\eta$ (a box):

$$
\big\lVert(\mathbf x_n-\bar{\mathbf x}_n)\oslash\mathbf x_{\text{scale}}\big\rVert_\infty \le \eta,
\quad
\lVert\mathbf b_n-\bar{\mathbf b}_n\rVert_\infty \le \eta,
\quad
\frac{|T_{m,n}-\bar T_{m,n}|}{T_{\max}} \le \eta,
\quad
\frac{|\sigma_{(\cdot)}-\bar\sigma_{(\cdot)}|}{\sigma_{\text{scale}}} \le \eta .
$$

The radius is **not fixed** — it is resized each iteration by a **predicted-vs-actual ratio test**.
Define the merit (true) cost using the *nonlinear* defect $\mathbf d_n$ (computed from the real
$\mathbf f$, not the linearisation):

$$
J(\mathbf x,\mathbf u,\sigma) = \text{fuel} + \text{smoothing} + \rho_{vc}\sum_n\big\lVert\mathbf d_n\oslash\mathbf x_{\text{scale}}\big\rVert^2 .
$$

After solving for a candidate $(\mathbf x^\star,\mathbf u^\star,\sigma^\star)$:

$$
\rho \;=\; \frac{\Delta J_{\text{actual}}}{\Delta J_{\text{predicted}}}
\;=\;
\frac{J(\bar{\mathbf x}) - J(\mathbf x^\star)}{J(\bar{\mathbf x}) - J_{\text{lin}}(\mathbf x^\star)},
$$

where $J_{\text{lin}}$ uses the subproblem's linearised defect ($=\mathbf w$). Then

| ratio | meaning | action |
|------|---------|--------|
| $\rho > \rho_0$ | true cost improved | **accept** step (update reference) |
| $\rho \le \rho_0$ | linearisation overshot | **reject** step (keep reference) |
| $\rho < \rho_1$ | poor prediction | **shrink** $\eta \leftarrow \tfrac12\eta$ |
| $\rho \ge \rho_2$ **and** boundary active | excellent prediction | **grow** $\eta \leftarrow 1.5\,\eta$ |

Defaults: $\rho_0=0,\ \rho_1=0.25,\ \rho_2=0.7$, clamped to $\eta\in[10^{-3},\,0.1]$.
Growth is gated on the step actually hitting the boundary, so $\eta$ cannot run away near a flat
optimum. This replaces hand-tuned penalty weights — the safe step size is *discovered*.

---

## 8. Convergence criteria and success / failure

### Converged (success)

The loop stops and reports success when an **accepted** step satisfies both:

$$
\text{step} = \max\!\big(\lVert\delta\mathbf x\oslash\mathbf x_{\text{scale}}\rVert_\infty,\ |\delta\sigma|/\sigma_{\text{scale}}\big) < \texttt{tol}
\qquad\text{and}\qquad
\lVert\mathbf d\oslash\mathbf x_{\text{scale}}\rVert_\infty < 10^{-3}.
$$

The first says the iterate has stopped moving; the second says the **true** nonlinear dynamics are
satisfied (the virtual control has effectively vanished). Defaults: $\texttt{tol}=5\times10^{-3}$.
A converged result is a dynamically feasible, constraint-satisfying, (locally) minimum-fuel
trajectory that lands on the pad at rest, with optimal coast and burn durations
$\sigma_{\text{coast}},\sigma_{\text{burn}}$ and a $\ge 40\%$ throttle profile.

### Failure modes

1. **Subproblem solve fails / infeasible** (solver error or non-optimal status). Treated as a
   rejected step: **shrink** $\eta$ and retry the *same* reference. If $\eta$ collapses to
   $\eta_{\min}$ this way, the loop **stops** — the problem is locally infeasible at that point.

2. **Virtual control never vanishes** — `defect_n` stays large while iterating. This is the
   *diagnostic value* of $\mathbf w$: the requested maneuver is **physically infeasible** under the
   constraints (e.g. the thrust/AoA authority cannot meet the boundary conditions — at high dynamic
   pressure the AoA-lift can exceed the lateral thrust, $qSC_{L\alpha}/T>1$, making cross-range
   impossible). The solver correctly refuses to fake feasibility.

3. **Iteration cap reached** (`iters_max`) without meeting the convergence test — typically slow
   convergence or a mild limit cycle (often a scaling or trust-region tuning issue rather than
   infeasibility).

4. **Attitude reference drifts off the unit sphere** — if $\bar{\mathbf b}$ is not re-projected to
   $\lVert\mathbf b\rVert=1$ each iteration, the tangent-plane constraint $\bar{\mathbf b}\cdot\mathbf b=1$
   becomes inconsistent with a tight trust region and the solver fails. (Fixed by re-projection; see §2b.)

The per-iteration log prints `rho`, accept/reject, the radius $\eta$, the phase durations
`tc`$=\sigma_{\text{coast}}$ and `tb`$=\sigma_{\text{burn}}$, the step size, and `defect_n`.

---

## 9. Why JAX

We write *only* $\mathbf f(\mathbf x,\mathbf b,T_m)$ and the projection $\mathbf p(\mathbf x,\mathbf b)$.
`jax.jacobian` (vmapped + jitted over all nodes) returns $A,B_b,B_t,\mathbf f_0$ and the projection
Jacobians exactly, in ~0.3 ms. Changing the vehicle or the physics means editing one function — the
linearisation follows automatically, with no hand-derived partials to maintain. (One gotcha: avoid a
bare $\sqrt{\,\cdot\,}$ that can hit zero, e.g. use $\lVert\mathbf p\rVert^2$ not $\lVert\mathbf p\rVert$
inside $C_D$ — its gradient is infinite at $\mathbf p=0$, which occurs in the coast.)

---

## 10. Real-time feasibility

Per-iteration cost (N = 30, ~535 scalar variables), measured:

| phase | time | embedded? |
|---|---|---|
| JAX Jacobians | ~0.3 ms | trivial |
| cvxpy build + canonicalisation | ~700 ms | **vanishes in C/C++** (canonicalise once) |
| solver (CLARABEL) interior | ~70 ms | the real math |
| warm re-solve (cached canon.) | ~80 ms | C# / ECOS per-iter proxy |

So the **per-iteration solve is ~70–80 ms** in compiled form; the ~700 ms is Python canonicalisation
that disappears when the problem is built once with fixed sparsity (e.g. `cvxpygen` → ECOS).

The catch is **iteration count**: this model needs **~30–60 SCvx iterations** from a cold seed
(attitude coupling + the bang-bang throttle are strongly nonlinear), so a cold solve-to-convergence is
~3–5 s — *not* real-time. The real-time path is **warm-started MPC**: re-seed from the previous cycle's
solution and run **1–2 SCvx iterations per control tick** (~75–150 ms → **~7–13 Hz**), paying the full
iteration count only once at initialisation. Levers for more margin: smaller `N` (15–20), capped
iters/tick, and noting that **attitude was the expensive addition** — it grew both the problem size and
the iteration count.

---

## 11. Outputs

* `3dof_results.png` — flight path, position/velocity, throttle + lift (with the 40% floor and
  ignition marker), mass, angle of attack.
* `3dof_landing.gif` — 3-D animation with thrust / lift / drag force vectors, a per-frame magnitude
  readout, and the phase label (COAST vs BURN).

---

# `6dof.py` — 6-DOF Powered-Descent Guidance (rigid body, gimballed thrust)

A full **rigid-body** extension of the same SCvx solver. The vehicle now carries **attitude (a
quaternion) and angular velocity as states**, and steers by **gimballing a single engine**
(small-angle) plus a small artificial roll torque. Aerodynamics are dropped (negligible at these
speeds — easy to add back later). Everything else — hand-write only $\mathbf f$, JAX for every
Jacobian, trapezoidal collocation with virtual control, free final time, and the
predicted-vs-actual trust-region update — is **identical to §3–8 above**; only the physics and the
constraint set change.

Scenario: a Super-Heavy-class booster translating from **300 m altitude / 100 m downrange offset**
onto the pad, vertical and at rest, minimising fuel.

---

## 1. State, control, free time

$$
\mathbf x = \begin{bmatrix}\mathbf r\\ \mathbf v\\ \mathbf q\\ \boldsymbol{\omega}\\ m\end{bmatrix}\in\mathbb R^{14},
\qquad
\mathbf u = (t_{dx},\,t_{dy},\,T,\,\tau_{\text{roll}})\in\mathbb R^{4},
\qquad
\sigma\in\mathbb R_{>0}.
$$

* $\mathbf r,\mathbf v$ — position / velocity (inertial)
* $\mathbf q$ — **attitude quaternion**, scalar-first $\mathbf q=(q_w,q_x,q_y,q_z)$, $\lVert\mathbf q\rVert=1$, body $\to$ inertial
* $\boldsymbol{\omega}$ — **body angular velocity**
* $m$ — mass
* $(t_{dx},t_{dy})$ — **lateral (gimbal) thrust** components in the body frame; $T$ — **axial thrust**
* $\tau_{\text{roll}}$ — small **artificial roll torque** (stands in for differential gimballing of multiple engines)
* $\sigma$ — the single **free burn duration** (one phase, engine lit throughout)

---

## 2. System of equations (continuous, nonlinear)

$$
\begin{aligned}
\dot{\mathbf r} &= \mathbf v \\
\dot{\mathbf v} &= \mathbf g + \frac{1}{m}\,R(\mathbf q)\,\mathbf T_{\text{body}} \\
\dot{\mathbf q} &= \tfrac12\,\mathbf q \otimes \begin{bmatrix}0\\\boldsymbol{\omega}\end{bmatrix} \\
\dot{\boldsymbol{\omega}} &= I^{-1}\big(\boldsymbol{\tau}_{\text{body}} - \boldsymbol{\omega}\times(I\boldsymbol{\omega})\big) \\
\dot m &= -\alpha\,T,\qquad \alpha = \frac{1}{I_{sp}\,g_0}
\end{aligned}
$$

with the body-frame thrust vector $\mathbf T_{\text{body}}=[\,t_{dx},\,t_{dy},\,T\,]^\top$ and diagonal inertia
$I=\mathrm{diag}(I_x,I_y,I_z)$. The rotation (body $\to$ inertial, Hamilton, scalar-first) is

$$
R(\mathbf q)=\begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y)\\
2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x)\\
2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix},
$$

and the quaternion kinematics expand to

$$
\dot{\mathbf q}=\tfrac12\begin{bmatrix}
-q_x\omega_x-q_y\omega_y-q_z\omega_z\\
\;\;q_w\omega_x+q_y\omega_z-q_z\omega_y\\
\;\;q_w\omega_y-q_x\omega_z+q_z\omega_x\\
\;\;q_w\omega_z+q_x\omega_y-q_y\omega_x
\end{bmatrix}.
$$

Euler's equation $I\dot{\boldsymbol{\omega}}=\boldsymbol{\tau}-\boldsymbol{\omega}\times(I\boldsymbol{\omega})$ carries the
**gyroscopic** coupling $\boldsymbol{\omega}\times(I\boldsymbol{\omega})$.

---

## 3. Gimbal torque and the small-angle approximation

The engine pivots at $\mathbf r_T=[0,0,-L]^\top$ (a distance $L$ **below** the CoM along the body axis).
The torque is the moment of the thrust about the CoM, plus the artificial roll:

$$
\boldsymbol{\tau}_{\text{body}} = \mathbf r_T\times\mathbf T_{\text{body}} + \tau_{\text{roll}}\,\hat{\mathbf z}
= \begin{bmatrix} L\,t_{dy}\\ -L\,t_{dx}\\ \tau_{\text{roll}} \end{bmatrix}.
$$

Two structural facts fall out:

1. **The torque is *linear* in the controls** — $\partial\boldsymbol{\tau}/\partial(t_{dx},t_{dy},\tau_{\text{roll}})$ is constant. No gimbal-angle trigonometry.
2. **The gimbal produces *zero* roll torque** ($\tau_z=0$ from $\mathbf r_T\times\mathbf T_{\text{body}}$) — a single centreline engine cannot torque about the thrust axis. Hence the explicit $\tau_{\text{roll}}$ is the **only** roll authority, and it stays $\approx 0$ since nothing in this symmetric, aero-free model excites roll.

**The small-angle approximation.** Parameterising the thrust by its body components makes the magnitude

$$
\lVert\mathbf T_{\text{body}}\rVert=\sqrt{t_{dx}^2+t_{dy}^2+T^2}\;\approx\;T,
$$

so the **axial** component $T$ stands in for total thrust in the mass flow ($\dot m=-\alpha T$) and in the
throttle limit. The relative error is $\sec\delta-1$ (second order in gimbal angle $\delta$). The reward
is that the throttle floor stays a **convex scalar box** $T_{\min}\le T\le T_{\max}$ — whereas a
thrust-*vector* formulation has the non-convex lower bound $T_{\min}\le\lVert\mathbf T_{\text{body}}\rVert$
(a lower bound on a norm) that requires a lossless-convexification (LCvx) slack to relax.

**What it costs (measured).** At the gimbal angles actually used here (peak $\delta\approx 5.9^\circ$,
mean $1.7^\circ$) the magnitude error $\sec\delta-1$ peaks at $0.54\%$ (mean $0.08\%$). Re-propagating
the converged optimal controls through the **exact** dynamics ($\dot m=-\alpha\lVert\mathbf T_{\text{body}}\rVert$)
costs $\approx 9$ kg extra propellant ($0.06\%$) and $\approx 3$ cm of terminal drift — i.e. negligible.
The value of the assumption is therefore **formulation robustness** (a clean SOCP, no LCvx), not raw
solve speed.

---

## 4. Convexity and constraints

The smooth non-convexities — $R(\mathbf q)$ (quadratic in $\mathbf q$), $\mathbf q\otimes\boldsymbol{\omega}$
(bilinear), $\boldsymbol{\omega}\times(I\boldsymbol{\omega})$ (quadratic), and $1/m$ — are all handled
automatically by the SCvx linearisation, with $A=\partial\mathbf f/\partial\mathbf x\in\mathbb R^{14\times14}$
and $B=\partial\mathbf f/\partial\mathbf u\in\mathbb R^{14\times4}$ from JAX.

**Unit quaternion** — the one hard non-convexity, linearised as the tangent hyperplane at every node and
**re-projected each iteration** (exactly as $\lVert\mathbf b\rVert=1$ in §2b):

$$
\bar{\mathbf q}_n\cdot\mathbf q_n = 1,\qquad \bar{\mathbf q}\leftarrow\bar{\mathbf q}/\lVert\bar{\mathbf q}\rVert .
$$

**Boundary conditions** (mass free):

$$
\mathbf x_0=[\mathbf r_0,\mathbf v_0,\mathbf q_0,\boldsymbol{\omega}_0,m_0]^\top,\qquad
(\mathbf r,\mathbf v,\mathbf q,\boldsymbol{\omega})_{N-1}=(\mathbf r_f,\mathbf v_f,\mathbf q_f,\boldsymbol{\omega}_f).
$$

**Convex control set** (no relaxation needed):

$$
T_{\min}\le T_n\le T_{\max},
\qquad
\sqrt{t_{dx,n}^2+t_{dy,n}^2}\;\le\;\tan\delta_{\max}\,T_n,
\qquad
|\tau_{\text{roll},n}|\le\tau_{\text{roll}}^{\max}.
$$

The middle one is the **gimbal cone** — a second-order cone, valid because $T_n\ge T_{\min}>0$.

**Upright (tilt) limit.** The body axis in the inertial frame is $R(\mathbf q)\hat{\mathbf z}$; its vertical
component is the $(3,3)$ entry $R_{33}=1-2(q_x^2+q_y^2)$. Keeping the booster within $\theta_{\max}$ of
vertical, linearised in $\mathbf q$:

$$
\bar R_{33,n} + \nabla R_{33}\big|_{\bar{\mathbf q}_n}\!\cdot(\mathbf q_n-\bar{\mathbf q}_n)\ge\cos\theta_{\max},
\qquad
\nabla R_{33}=(0,\,-4q_x,\,-4q_y,\,0).
$$

Plus ground clearance $r_{z,n}\ge 0$.

---

## 5. Shared machinery (unchanged from the 3-DOF model)

Linearisation (§3), trapezoidal collocation with the virtual control $\mathbf w$ (§4), the
minimum-fuel objective with **rate** smoothing on the controls plus an angular-rate damping term (§6),
the trust-region radius update via the predicted-vs-actual ratio test (§7), and the convergence test
(§8) are all reused verbatim — with a **single** free duration $\sigma$ (one burn phase, no coast split)
in the time dilation $\,d\mathbf x/d\tau=\sigma\,\mathbf f(\mathbf x,\mathbf u)$.

---

## 6. Real-time feasibility

Measured at $N=30$: the convex subproblem is an **SOCP** with **947 scalar variables / 253 constraints**,
solving in **~60 ms cold** in CLARABEL. As in the 3-DOF case the headline ~700 ms/iter wall is almost all
Python canonicalisation, which **vanishes** in a code-gen C/C# stack (`cvxpygen` $\to$ ECOS, references as
`cp.Parameter`). One-shot trajectory generation is sub-second compiled; **warm-started MPC** (1–2 SCvx
iterations per tick, tens of ms each) is genuinely real-time at tens of Hz.

---

## 7. Outputs

* `6dof_results.png` — flight path, position / velocity, throttle + gimbal angle, tilt + body rates, mass.
* `6dof_landing.gif` — 3-D animation of the oriented booster body (drawn from the quaternion) with its
  thrust vector and a per-frame throttle / gimbal / tilt / speed readout.