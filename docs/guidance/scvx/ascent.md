# Convex ascent

The convex ascent plans the climb from the pad to orbit insertion: the steering and throttle through the atmosphere, every staging event, and the propellant each returnable booster keeps back to fly home. It is a port of `launch3dof.py`, a minimum-propellant multi-stage ascent solved by [successive convexification](index.md). It runs on the pad, on a worker thread, when LAUNCH is pressed without a plan that fits the launch, or when the player asks for a plan to look at first. A failed or misfitting plan can be solved again, as described [below](#recovery-and-retry). Three things use the plan. The ascent flies its attitude and throttle open loop, indexed by air-relative speed. UPFG takes over above the hand-over altitude. And the LAUNCH page shows the plan's mass to orbit, its launch timing and each booster's reserve, which the staging model then holds back.

## At a glance

| Item | This solver |
| --- | --- |
| Frame | Body-centred inertial (KSA's CCI), $+z$ along the spin axis. Canonical units: length $R$ (the mean radius), time $\sqrt{R^3/\mu}$, mass $m_0$ (the lift-off mass), so $\mu = 1$. |
| State | $\mathbf x = (\mathbf r, \mathbf v, m) \in \mathbb R^7$ |
| Controls | $\mathbf u \in \mathbb R^3$, the throttle vector: its direction is the thrust axis, its length the liquid throttle |
| Parameters | one burn time $\sigma_i$ per stage (free, or pinned); optionally a plane turn $\varphi$ (launch time or LAN); optionally one reserve change $\Delta R_j$ per booster stage |
| Phases and nodes | one phase per planned stage, 60 nodes per burn stage (`NodesPerStage`) and 25 per coast stage (`CoastNodes`); two nodes share each staging instant |
| Discretisation | trapezoidal collocation on a uniform mesh in each stage's own $\tau \in [0,1]$ |
| Cone solver | persistent `ClarabelWorkspace` backed by frozen-sparsity `CcsAssembler` (`AscentSubproblem.Workspace`), rebuilt only when the active pattern changes or stage duration shifts by $\ge 2\times$ |
| Where and when | the `afc-ascent-scvx` worker thread, once per launch on the pad (plus retries); seconds to tens of seconds, cancellable between iterations |

## State and controls

| Symbol | Meaning |
| --- | --- |
| $\mathbf r,\ \mathbf v$ | position and velocity, inertial |
| $m$ | mass |
| $\mathbf u$ | throttle vector, $\lVert\mathbf u\rVert \le 1$. $\hat{\mathbf u} = \mathbf u/\lVert\mathbf u\rVert$ is the thrust axis (the attitude), $\lVert\mathbf u\rVert$ the liquid engines' throttle |
| $\sigma_i$ | burn time of stage $i$; $\tau \in [0,1]$ is the stage's normalised time, $t = \sigma_i\tau$ inside it |
| $\varphi$ | turn of the target plane about the spin axis, east positive |
| $\Delta R_j$ | change of booster stage $j$'s reserve from the reserve it was built with, $R_{0,j}$ |
| $\mu,\ R,\ \boldsymbol\omega$ | the body's gravitational parameter, mean radius and spin, $\boldsymbol\omega = (0,0,\omega)$ |
| $h = \lVert\mathbf r\rVert - R$ | altitude above the mean radius |
| $\mathbf v_{\text{rel}} = \mathbf v - \boldsymbol\omega\times\mathbf r$ | velocity relative to the co-rotating air |
| $\rho(h),\ p(h)$ | air density and pressure |
| $q = \tfrac12\rho\lVert\mathbf v_{\text{rel}}\rVert^2$ | dynamic pressure |
| $\alpha$ | angle of attack: the angle between $\hat{\mathbf u}$ and $\mathbf v_{\text{rel}}$ |

The control is a throttle, not a thrust. `launch3dof.py` uses the thrust vector $\mathbf T$ with a constant bound per stage. In KSA the full thrust depends on back pressure, so $\lVert\mathbf T\rVert \le T_{\max}(h)$ would not be a cone. Writing $\mathbf T = T_{\max}(h)\,\mathbf u$ moves the altitude dependence into the dynamics and keeps $\lVert\mathbf u\rVert \le 1$ a cone.

## Dynamics

$$
\begin{aligned}
\dot{\mathbf r} &= \mathbf v \\
\dot{\mathbf v} &= -\frac{\mu\,\mathbf r}{\lVert\mathbf r\rVert^3}
  + \frac{T_L\big(p(h)\big)\,\mathbf u + T_S\big(t, p(h)\big)\,\hat{\mathbf u} + \mathbf F_{\text{aero}}(\mathbf r,\mathbf v,\hat{\mathbf u})}{m} \\
\dot m &= -\big(\dot m_L\,\lVert\mathbf u\rVert + \dot m_S(t)\big)
\end{aligned}
$$

**Gravity** is a point mass. There are no zonal terms.

**Rotating-body terms.** The frame is inertial, so there are no Coriolis or centrifugal terms. The body's spin enters in four places: the air co-rotates, so drag, $q$ and $\alpha$ use $\mathbf v_{\text{rel}}$; the lift-off velocity carries the pad's own motion $\boldsymbol\omega\times\mathbf r_0$; the pad's zenith turns during a planned [pad roll](#pad-roll-for-a-solid-launch); and each booster's landing site turns with the ground.

**Liquid thrust** $T_L(p)$ is the stage's full-throttle thrust at back pressure $p$: a cubic spline through 17 samples from vacuum to sea level, taken from KSA's own engine model, flow separation included. Its mass flow $\dot m_L$ does not depend on pressure, so the burn time of a load is the same at any altitude. A stage without a pressure table has constant thrust, as in the script.

**Solid motors** add thrust $T_S(t,p)$ and mass flow $\dot m_S(t)$ outside the throttle. They push along $\hat{\mathbf u}$ whatever its length, so the throttle steers them but cannot slow them. Both are read from a table sampled off KSA's burn model, against the stage's own clock and the back pressure. A stage with solids has its burn time pinned, so every node sits at a fixed second of the burn. See [Solids](#solid-motors).

**Aerodynamics (drag, lift and side force).** When the aero sweep provides a nose-first body-axis force table (`AscentAtmosphere.BodyForce` / `NoseBodyAero`), `AscentDynamics` evaluates the full aerodynamic force (axial drag plus lateral lift and side force) in the held-roll body axes (`RollAngleRad`), projecting lateral/normal forces onto the rotated plane normal $\hat{\mathbf n}(\varphi) = \texttt{PlaneNormal}(\varphi)$ whenever `PlaneTurn` is free so $\partial/\partial\varphi$ derivatives are exact. Axial $C_x$ is read at the rounded angle of attack and lateral $C_y, C_z$ at the true angles softly bounded by $x / (1 + (x/A)^8)^{1/8}$ with $A = 35^\circ$. Local Mach uses altitude-varying `KsaAscentAtmosphere.SpeedOfSound(altitude)` (delegating to `SampledAtmosphere.SpeedOfSoundAt(altitude)`). When no body-force table is available (or during the initial drag-only pass), $\mathbf F_{\text{aero}} = -\tfrac12\rho(h)\,S\,C_D(M,\alpha)\,\lVert\mathbf v_{\text{rel}}\rVert\,\mathbf v_{\text{rel}}$ acts straight against $\mathbf v_{\text{rel}}$ using the pad-swept drag table. With

$$
\mathbf w = \hat{\mathbf u} - (\hat{\mathbf u}\cdot\hat{\mathbf v}_{\text{rel}})\,\hat{\mathbf v}_{\text{rel}},
\qquad \lVert\mathbf w\rVert = \sin\alpha,
\qquad \alpha = \operatorname{atan2}\big(\lVert\mathbf w\rVert,\ \hat{\mathbf u}\cdot\hat{\mathbf v}_{\text{rel}}\big),
$$

the drag law reads $\lVert\mathbf w\rVert$ rounded to $\lVert\mathbf w\rVert^2/\sqrt{\lVert\mathbf w\rVert^2 + \epsilon^2}$, $\epsilon = 0.05$ (`AlphaRounding`). KSA's drag grows with $|\sin\alpha|$ from a corner at $\alpha = 0$, which a linearisation cannot follow. The rounding keeps nose-on drag exact and underestimates off-axis drag by at most $0.3\,\epsilon$ in $\sin\alpha$. On an airless body $\rho = p = 0$, so aerodynamics, $q$ and the aerodynamic limits drop out and every engine runs at its vacuum thrust.

Small regularisers keep the square roots smooth: $\lVert\mathbf r\rVert$ and $\lVert\mathbf v_{\text{rel}}\rVert$ take $10^{-8}$ under the root, and $\lVert\mathbf u\rVert$ takes $10^{-6}$ divided by the square of the stage's reference thrust.

## Phases and staging

The stages are KSA's own staging model, phase for phase. A phase is a stretch over which the same engines burn, so it ends wherever an engine starts or stops. Each phase becomes one stage of the plan, with its own block of nodes and its own burn time $\sigma_i$:

$$
\frac{d\mathbf x}{d\tau} = \sigma_i\,\mathbf f(\mathbf x,\mathbf u),\qquad \tau\in[0,1].
$$

Separations are instantaneous. The last node of one stage and the first node of the next are the same instant, joined by a jump: $\mathbf r$ and $\mathbf v$ carry through, and the mass drops by the structure the stage leaves behind.

**Variable-duration ballistic coast (`AscentStage.Coast`).** A stage list may include an explicit coast stage (`IsCoast = true`, `HasLiquid = false`, `IsPinned = false`, `CoastNodes = 25`, minimum 5) with free duration $\sigma_i \in [\sigma_{\min}, \sigma_{\max}]$ (default `[0, 1800]` s) and optional `LiquidCarriesOver` to chain propellant across the coast into a circularisation burn:

- Coast dynamics apply zero thrust, $\dot m = 0$, nose-first relative airflow ($\alpha = 0$), and pin $\mathbf u_k = \mathbf 0$ by equality rows; smoothing, $q\alpha$, prograde and thrust-ceiling cones are omitted while $q \le q_{\max}$, mass floor and ground floor stay active.
- Adjacent burn stages relax $\sigma_{\min}$ to $\min(\sigma_{\min,i}, 1.0 / t_u)$ to permit short circularisation burns.
- For `LiquidCarriesOver` pre-coast burns, `ApoapsisCutoff` integrates up to 95 % of combined propellant until Keplerian $r_a \ge R_{\text{target}}$ or $\varepsilon \ge \varepsilon_{\text{target}}$, and `CoastDurationSeed` computes the analytical Keplerian coast time to $R_{\text{target}}$ (or apoapsis $\pi$):

    $$
    \varepsilon = \tfrac12 v^2 - \frac{1}{r},\quad a = -\frac{1}{2\varepsilon},\quad
    e\cos E_0 = 1 - \frac{r}{a},\quad e\sin E_0 = \frac{\mathbf r\cdot\mathbf v}{\sqrt{a}},\quad M_0 = E_0 - e\sin E_0,
    $$

    $$
    E_t = \begin{cases} \arccos\!\left(\dfrac{1 - R_{\text{target}}/a}{e}\right), & r_a > R_{\text{target}} \land r < R_{\text{target}} \\ \pi, & \text{otherwise} \end{cases},
    \qquad
    \Delta t = \operatorname{clamp}\!\big((M_t - M_0)\,a\sqrt{a},\ \sigma_{\min},\ \sigma_{\max}\big),
    $$

    returning $\sigma_{\min}$ directly when the state is unbound ($\varepsilon \ge -10^{-9}$), descending ($\mathbf r\cdot\mathbf v < 0$), near-circular ($e \le 10^{-6}$), or already at $r \ge R_{\text{target}}$.

- Live flight execution currently uses single-burn `KsaAscentPlan` and UPFG hand-over; wiring `AscentStage.Coast` into a dedicated flight coast and circularisation mode is a separate follow-up.

Which durations are free:

- **A liquid stage** has a free $\sigma_i$. It is bounded by $\min(20\text{ s}, \tfrac12 t_{\text{full}}) \le \sigma_i \le \max(700\text{ s}, 1.1\,t_{\text{full}}/\theta_{\min})$, where $t_{\text{full}}$ is the full-throttle burn time and $\theta_{\min}$ the stage's throttle floor. The upper bound widens for a stage under an acceleration limit, which throttles down as it gets lighter, and for a core that burns a side booster's share of a reserve.
- **A stage with solids** is pinned to the solids' burn: it ends where the first of them burns out, or where the drain model's phase ends. A liquid core lit with the solids burns on into the next stage on the same load.
- **A core that would run dry before its boosters** at full throttle is planned throttled down to outlast them, if its floor gives at least a 15 % margin. It becomes two stages: boosters and core until the boosters burn out (pinned), then the core alone (free). Otherwise the core is held at full thrust and its stage pinned.
- **A load shared by several stages** fixes only the total burned, not how it splits across the stages.

**How many stages.** The first attempt plans the fewest stages whose ideal $\Delta v$ reaches 1.1 times the insertion speed. The stages above ride along as payload, and the last planned stage drops nothing. See [Recovery and retry](#recovery-and-retry).

**Returnable-stage reserves.** A booster that comes home keeps a reserve $R_j = R_{0,j} + \Delta R_j$ in the stage whose end drops it. That stage burns $R_j/\chi_j$ less, where $\chi_j \in (0,1]$ is the booster's share of the stage's propellant flow, and the drop at separation grows by $R_j$. With side boosters ($\chi_j < 1$) the core's part of the reserve, $R_j(1/\chi_j - 1)$, stays aboard and is burned in the next stage. In the game $R_{0,j} = 0$: the solve sizes each reserve from nothing.

## Boundary conditions

**Lift-off.** The plan starts from the live state:

$$
\mathbf x(0) = (\mathbf r_0,\ \mathbf v_0,\ m_0).
$$

$m_0$ is the live mass. On the pad the air-relative velocity is zero and the angle of attack is undefined, so a vehicle at rest starts rising at 10 m/s: $\mathbf v_0 = \boldsymbol\omega\times\mathbf r_0 + 10\,\hat{\mathbf r}_0$ m/s. A launch armed for a window starts from the pad as the spin will have carried it by then. The first thrust is pinned straight up (see [Lift-off direction](#lift-off-direction)).

**Insertion.** The plan runs to orbit insertion, not to the hand-over. It inserts at the target's periapsis, or where the target ellipse crosses the top of the atmosphere when periapsis is inside it, with radius $r^\star$, speed $v^\star$ from the vis-viva equation and flight-path angle $\gamma^\star$. Five conditions hold at $t_f = \sum_i\sigma_i$:

$$
\lVert\mathbf r_f\rVert = r^\star,\qquad
\lVert\mathbf v_f\rVert = v^\star,\qquad
\mathbf r_f\cdot\mathbf v_f = r^\star v^\star\sin\gamma^\star,\qquad
\mathbf n(\varphi)\cdot\mathbf r_f = 0,\qquad
\mathbf n(\varphi)\cdot\mathbf v_f = 0,
$$

plus a prograde condition $(\mathbf r_f\times\mathbf v_f)\cdot\mathbf n(\varphi) \ge 0$. Here $\mathbf n(\varphi) = R_z(\varphi)\,\mathbf n_0$ is the target plane's normal, from the inclination and LAN, turned about the spin axis by $\varphi$. Where on the orbit the vehicle arrives is free, and so is the final mass, which the objective maximises.

**The plane turn.** The body and its air are symmetric about the spin axis, so lifting off $\Delta t$ later is the same problem as lifting off now into a plane turned by $-\omega\,\Delta t$. With SOLVE LIFTOFF on (`ConvexOptimiseTiming`), $\varphi$ is free:

- With a target, it is the launch time, within 600 s either side of the window and no earlier than 60 s from now: $\Delta t = -\varphi/\omega$.
- Without one, it is the LAN, within $\pm\omega\cdot 600$ s of the plane over the pad, flown now.
- Launching now to a target, it is pinned at zero and none of its rows exist.

!!! note "The hand-over is not a boundary condition"
    The plan is solved all the way to insertion, so the climb through the air is shaped for the vacuum burn that follows. The flight uses only the part below the hand-over altitude (UPFG FROM, 80 km by default, never above 75 % of the target periapsis). From there UPFG closes the loop on the same target.

## Constraints

| Constraint | Continuous form | In the subproblem | Code |
| --- | --- | --- | --- |
| [Lift-off direction](#lift-off-direction) | $\mathbf u(0)\parallel\hat{\mathbf e}_{\text{up}}$ | two linear rows | `AscentSubproblem.Solve` |
| [Pad roll](#pad-roll-for-a-solid-launch) | vertical to $t_{\text{clear}} + 10$ s; $\lvert\dot{\hat{\mathbf u}}\rvert \le 5$ deg/s in stage 1 | linear rows; one cone per interval | `AscentCase` |
| [Dynamics](#dynamics-and-staging-links) | $\dot{\mathbf x} = \mathbf f$ | linearised trapezoid with virtual control | `AscentSubproblem.Solve` |
| [Staging links](#dynamics-and-staging-links) | $\mathbf r,\mathbf v$ continuous, $m^+ = m^- - m_{\text{dry}} - R$ | linear equalities, exact | `AscentSubproblem.Solve` |
| [Propellant](#propellant) | each load burned exactly, the last at most | linear rows, exact | `AscentCase.Loads` |
| [Burn times](#burn-times) | $\sigma_{\min}\le\sigma_i\le\sigma_{\max}$, or $\sigma_i = \sigma_{\text{fix}}$ | box or equality, exact | `AscentCase` |
| [Throttle ceiling](#throttle-ceiling-and-floor) | $\lVert\mathbf u\rVert \le 1$ | second-order cone, exact | `AscentSubproblem.Solve` |
| [Throttle floor](#throttle-ceiling-and-floor) | $\lVert\mathbf u\rVert \ge \theta_{\min}$ | tangent half-space in the reference direction, hard | `AscentSubproblem.Solve` |
| [Acceleration limit](#acceleration-limit) | $T_L(h)\lVert\mathbf u\rVert \le a_{\max}\,m$ | cone with $T_L$ frozen, backed off, slacked | `AscentSubproblem.Solve` |
| [Maximum dynamic pressure](#maximum-dynamic-pressure) | $q \le q_{\max}$ | tangent half-space, backed off, slacked | `AscentSubproblem.Solve` |
| [q-alpha](#q-alpha) | $q\sin\alpha \le (q\alpha)_{\max}$ | cone on the linearised $\mathbf w$, backed off, slacked | `AscentSubproblem.Solve` |
| [Thrust with the airflow](#thrust-with-the-airflow) | $\hat{\mathbf u}\cdot\hat{\mathbf v}_{\text{rel}} \ge 0$ | half-space on the reference airflow, slacked | `AscentSubproblem.Solve` |
| [Ground](#ground-and-mass-floor) | $\lVert\mathbf r\rVert \ge R_g$ | tangent half-space, hard | `AscentSubproblem.Solve` |
| [Mass floor](#ground-and-mass-floor) | $m \ge m_{\text{floor}}$ | linear, exact | `AscentSubproblem.Solve` |
| [Insertion](#insertion) | five equalities | linearised, slacked, quadratic price | `AscentCase.TerminalResidual` |
| [Prograde insertion](#insertion) | $(\mathbf r_f\times\mathbf v_f)\cdot\mathbf n \ge 0$ | linearised half-space, hard | `AscentSubproblem.Solve` |
| [Plane turn](#plane-turn) | $\varphi_{\min}\le\varphi\le\varphi_{\max}$ | box, and an epigraph for $\lvert\varphi\rvert$ | `AscentSubproblem.Solve` |
| [Booster reserve](#booster-reserve) | $R_j \ge R_{\text{req},j}(\mathbf r_s,\mathbf v_s,t_s)$, $R_{\min}\le R_j\le R_{\max}$ | linear row, finite-difference slopes, slacked | `BoostbackOracle` |
| [Peak q ceiling](#maximum-dynamic-pressure) | $\max q < 0.98\,q_{\text{breakup}}$ | not in the subproblem: checked on the plan | `AscentPlan.OverQCeiling` |

The path limits on $q$, q-alpha and the airflow apply only at the nodes where the reference dynamic pressure exceeds 100 Pa (`QActive`). When that active set, the mass-scaled floor mask, the limited-node thrust mask, and the recovery-oracle validity mask match the previous iteration, `AscentSubproblem.Workspace` refills the frozen `CcsAssembler` in place and updates the persistent `ClarabelWorkspace` without reallocating or refactorising. Backed-off limits use $b = 0.98$ (`PathBackoff`). Aerodynamic slacks are priced linearly at $w_{\text{path}} = 1$ (`RhoPath`), and thrust-acceleration slacks $s^g_k$ at $w_{\text{accel}} = 100$ (`RhoAccel`); see [Soft constraints and slacks](index.md#soft-constraints-and-slacks).

### Lift-off direction

The first thrust points straight up. The direction $\hat{\mathbf e}_{\text{up}}$ is the lift-off air-relative velocity, which for a vehicle at rest is the local vertical. It is two linear rows, $\mathbf p_1\cdot\mathbf u_0 = \mathbf p_2\cdot\mathbf u_0 = 0$, with $\mathbf p_1,\mathbf p_2$ perpendicular to $\hat{\mathbf e}_{\text{up}}$. The throttle floor and the trust region at node 0 use the same direction.

!!! note "The pad hold is flown, not planned"
    The flight holds the command straight up, with no roll, until the vehicle is 50 m above where it engaged (`PadHold`), whatever the plan says. The planner does not model that hold for a liquid first stage: it pins only the first node. A solid first stage with FORCE ROLL plans the hold, the roll and the pitch-over rate, as below.

### Pad roll for a solid launch

A solid first stage cannot throttle to wait for a roll, so with FORCE ROLL on a pad launch the plan models the roll program. In continuous form:

$$
\mathbf u(t)\parallel\hat{\mathbf e}_{\text{up}}(t)\quad\text{for } t \le t_{\text{clear}} + 10\text{ s},
\qquad
\big\lVert\dot{\hat{\mathbf u}}\big\rVert \le 5^\circ/\text{s}\quad\text{in stage 1}.
$$

$t_{\text{clear}}$ is the time to climb 50 m straight up from rest at the floor throttle, and $\hat{\mathbf e}_{\text{up}}(t)$ is the pad's zenith turning with the body. The vertical part is two linear rows per node, up to the first node at or after the deadline. The rate limit becomes a cone on each first-stage interval:

$$
\lVert\mathbf u_{k+1} - \mathbf u_k\rVert \le 2\,\theta_{\min}\sin\!\big(\tfrac12\,\omega_{\max}\,\Delta t\big).
$$

Since $\lVert\mathbf u_{k+1} - \mathbf u_k\rVert \ge 2\,\theta_{\min}\sin(\tfrac12\Delta\theta)$ for any two throttles above the floor, $\Delta\theta$ being the angle between them, the cone implies the rate limit. It is exact at the floor, where a solid stage's 99 % throttle sits. A problem whose deadline falls after the first solid burns out is refused. The reference must meet these rows exactly, and that is one of the [convergence gates](#settings-for-the-common-machinery).

### Dynamics and staging links

The dynamics are linearised about the reference and discretised by the trapezoidal rule, with a virtual control $\boldsymbol\nu_k$ on each interval inside a stage; see [Linearisation](index.md#linearisation), [Discretisation](index.md#discretisation) and [Virtual control](index.md#virtual-control). With $\mathbf g_k = \sigma_i\,\bar{\mathbf f}_k + \bar\sigma_i\,(A_k\,\delta\mathbf x_k + B_k\,\delta\mathbf u_k + D_{\varphi,k}\,\delta\varphi)$ (the $D_{\varphi,k}$ term enters the velocity rows when `BodyForce` and a free `PlaneTurn` are both active),

$$
\mathbf x_{k+1} = \mathbf x_k + \frac{\Delta\tau}{2}\,(\mathbf g_k + \mathbf g_{k+1}) + \boldsymbol\nu_k .
$$

A staging link has no virtual control. It is exact and linear:

$$
\mathbf r_{k+1} = \mathbf r_k,\qquad \mathbf v_{k+1} = \mathbf v_k,\qquad m_{k+1} = m_k - m_{\text{dry},i} - \Delta R_j\ [\text{booster stage only}] .
$$

$m_{\text{dry},i}$ already includes the reserve the stage was built with.

### Propellant

Each run of stages that shares one liquid load burns it:

$$
\sum_{i\in\text{run}}\big(m_{\text{start},i} - m_{\text{end},i}\big) \;=\; P_{\text{run}} - \frac{\Delta R_j}{\chi_j}\ [\text{booster stage}] + \Delta R_{j'}\Big(\frac{1}{\chi_{j'}} - 1\Big)\ [\text{core after side boosters}] .
$$

The row is an equality for a lower load that runs dry on a free burn time. It is an upper bound ($\le$) for the last load, whose cutoff is free, and for a load ended by a pinned burn. Each stage's drop is taken from its own first node to its last, so the jettisons between them stay out of it. A run of solids alone needs no row: the dynamics fix its drop.

### Burn times

Free burn times sit in the box of [Phases and staging](#phases-and-staging). A pinned burn time, of a stage with solids or a core held at full thrust, is a linear equality and takes no trust-region rows.

### Throttle ceiling and floor

The ceiling $\lVert\mathbf u_k\rVert \le 1$ is a second-order cone and needs no change.

The floor $\lVert\mathbf u\rVert \ge \theta_{\min}$ is not convex: it cuts a ball out of the control set. The subproblem replaces it by the tangent half-space in the reference thrust direction,

$$
\hat{\bar{\mathbf u}}_k\cdot\mathbf u_k \ge \theta_{\min},
$$

which implies the floor, so it is conservative. With the ceiling it leaves a spherical cap. At a 99 % floor the thrust can turn $\arccos 0.99 = 8.1^\circ$ per iteration; at 100 % the cap would close to a point and the direction could never move from the seed's.

The floor $\theta_{\min}$ of each stage:

- A liquid stage that can throttle takes MIN THROTTLE (`ConvexThrottleMinPct`, 80 % by default), raised to its engines' own minimum $\theta_{\text{phys}}$.
- A stage with solids, an engine that cannot throttle, or a core held at full thrust is held at 99 %.
- The library default, `AscentSettings.ThrottleMin`, is the script's 99 %.

Where the [acceleration limit](#acceleration-limit) binds on the reference, full thrust is not allowed, so the floor becomes a fraction of the ceiling the limit leaves at the node's own mass:

$$
\hat{\bar{\mathbf u}}_k\cdot\mathbf u_k \ge \theta_{\min}\,\frac{a_{\max}\,b\,m_k}{\bar T_{L,k}},
\qquad
\hat{\bar{\mathbf u}}_k\cdot\mathbf u_k \ge \theta_{\text{phys}} .
$$

The floor follows the variable $m_k$, not the reference mass, so the next reference meets it whatever the step did to the mass.

### Acceleration limit

A liquid stage that can throttle holds a thrust-acceleration limit,

$$
T_L\big(p(h)\big)\,\lVert\mathbf u\rVert \le a_{\max}\,m,
$$

where $a_{\max}$ is the lower of the player's G-LIMIT (when on) and the structural limit KSA's flight computer holds the throttle to. The limit is bilinear, because $T_L$ depends on altitude. The subproblem freezes the thrust at the reference altitude, $\bar T_{L,k} = T_L(p(\bar h_k))$, which leaves a cone:

$$
\bar T_{L,k}\,\lVert\mathbf u_k\rVert \le a_{\max}\big(b\,m_k + \bar m_k\,s^g_k\big),\qquad s^g_k \ge 0 .
$$

The slack term uses the reference mass so the cone stays linear in the variables. A stage whose peak thrust over its lightest mass never reaches $b\,a_{\max}$ drops the limit. Solids follow their grain, not the throttle, so a stage with solids cannot hold the limit; the planner says so when its full thrust would pass it.

### Maximum dynamic pressure

$$
q = \tfrac12\,\rho(h)\,\lVert\mathbf v_{\text{rel}}\rVert^2 \le q_{\max}
$$

is linearised in the state (forward-mode automatic differentiation gives $\nabla q$), backed off and slacked:

$$
\frac{\bar q_k + \nabla q_k^\top\,\delta\mathbf x_k}{q_{\max}} \le b + s^q_k,\qquad s^q_k \ge 0 .
$$

$q_{\max}$ is MAX Q (`ConvexQMaxKpa`), **195 kPa** by default. A plan is never solved above 98 % of KSA's break-up dynamic pressure rounded down to 5 kPa, which is also 195 kPa. A converged plan whose peak node $q$ still reaches 98 % of break-up is not flown. `AscentProblem.QMax` defaults to the script's 35 kPa, which applies only offline. If every seed climbs through $q_{\max}$, the problem is likely infeasible and the job retries at a higher limit; see [Recovery and retry](#recovery-and-retry).

### q-alpha

$$
q\,\sin\alpha \le (q\alpha)_{\max}
$$

uses $\sin\alpha = \lVert\mathbf w\rVert$ for the angle, and $(q\alpha)_{\max}$ is MAX Q-ALPHA (`ConvexQAlphaMax`, 3500 Pa rad). The limit is linearised through the vector $\mathbf w$, not through $\lVert\mathbf w\rVert$: at $\alpha = 0$ the gradient of $\lVert\mathbf w\rVert$ vanishes, and a scalar linearisation could not see the thrust turn. Linearising the product $q\lVert\mathbf w\rVert$ about the reference gives a cone:

$$
\big\lVert\bar{\mathbf w}_k + J_{w,x}\,\delta\mathbf x_k + J_{w,u}\,\delta\mathbf u_k\big\rVert
\le \frac{(q\alpha)_{\max}}{\bar q_k}\big(b + s^{q\alpha}_k\big) - \frac{\lVert\bar{\mathbf w}_k\rVert}{\bar q_k}\,\nabla q_k^\top\delta\mathbf x_k,
\qquad s^{q\alpha}_k \ge 0 .
$$

This is the planner's only angle-of-attack limit. It has no attitude-rate limit outside the [pad roll](#pad-roll-for-a-solid-launch): the [smoothing term](#objective) damps the node-to-node turn, and the flight's own 5 deg/s command slew limit guards the rest.

### Thrust with the airflow

$\sin\alpha$ returns to zero at $180^\circ$, so without this a stack that cannot throttle could meet the $q$ limit by firing against the airflow to bleed off speed. At the active nodes the thrust stays within $90^\circ$ of the reference airflow:

$$
-\hat{\bar{\mathbf v}}_{\text{rel},k}\cdot\mathbf u_k \le s^{\text{pro}}_k,\qquad s^{\text{pro}}_k \ge 0 .
$$

It never binds on a sane ascent.

### Ground and mass floor

The trajectory stays above $R_g = \min(R,\ \lVert\mathbf r_0\rVert - 1\text{ m})$, the mean radius, or just under the pad where the pad is below it. The outside of a ball is not convex, so the subproblem uses the tangent half-space,

$$
\hat{\bar{\mathbf r}}_k\cdot\mathbf r_k \ge R_g,
$$

which is conservative. The mass never drops below what is left when every load is burned and every jettison dropped, $m_k \ge m_{\text{floor}}$, a linear row.

### Insertion

The five insertion conditions are linearised about the reference's last node and each takes a free slack $s_j$, priced quadratically:

$$
\begin{aligned}
\hat{\bar{\mathbf r}}_f\cdot\mathbf r_f - r^\star &= s_0, &
\hat{\bar{\mathbf v}}_f\cdot\mathbf v_f - v^\star &= s_1, \\
\bar{\mathbf v}_f\cdot\mathbf r_f + \bar{\mathbf r}_f\cdot\mathbf v_f - \bar{\mathbf r}_f\cdot\bar{\mathbf v}_f - (\mathbf r\cdot\mathbf v)^\star &= s_2, &
\bar{\mathbf n}\cdot\mathbf r_f + (\hat{\mathbf z}\times\bar{\mathbf n})\cdot\bar{\mathbf r}_f\,(\varphi - \bar\varphi) &= s_3,
\end{aligned}
$$

and the same as the last for $\mathbf v_f$ ($s_4$). The plane rows use $\mathbf n(\varphi) \approx \bar{\mathbf n} + (\hat{\mathbf z}\times\bar{\mathbf n})(\varphi - \bar\varphi)$ and drop the second-order cross term. The prograde condition is linearised the same way and kept hard:

$$
(\bar{\mathbf r}_f\times\bar{\mathbf v}_f)\cdot\bar{\mathbf n} + (\bar{\mathbf v}_f\times\bar{\mathbf n})\cdot(\mathbf r_f - \bar{\mathbf r}_f) + (\bar{\mathbf n}\times\bar{\mathbf r}_f)\cdot(\mathbf v_f - \bar{\mathbf v}_f) \ge 0 .
$$

Because it is hard, a seed that ends retrograde is discarded: it would leave the first subproblem with no feasible point.

### Plane turn

When $\varphi$ is free it sits in its bounds, and an epigraph variable $t \ge \lvert\varphi\rvert$ carries a linear price in the objective. When the bounds are equal, neither variable nor any of their rows exists.

### Booster reserve

Each booster brought home needs, by its own rocket equation from its dry mass,

$$
R_{\text{req}} = m_{\text{dry}}\left(\exp\!\left(\frac{1.25\,\Delta v_{bb} + \Delta v_{\text{corr}}}{v_e} + \frac{\Delta v_{\text{land}}}{v_{e,\text{SL}}}\right) - 1\right).
$$

$\Delta v_{bb}$ is the smallest impulse, fired 15 s after separation, that puts the booster's drag-and-lift coast down on its site. It is zero for an optimal return (a drone ship). $\Delta v_{\text{land}}$ is the landing burn from terminal velocity at the landing's thrust acceleration $a$, $1.1\,v_{\text{term}}\,a/(a - g)$, or $1.3\,v_{\text{term}}$ where $a$ is not known. $\Delta v_{\text{corr}}$ is the player's allowance for corrections. The landing is priced at the sea-level exhaust velocity $v_{e,\text{SL}}$, the rest at the vacuum one.

The need depends on the separation state $(\mathbf r_s, \mathbf v_s)$, the last node of the booster's stage, and on the separation time $t_s = \sum_{i\le s}\sigma_i$, which turns the site with the body. The subproblem linearises it:

$$
R_{0,j} + \Delta R_j + s^{\text{rec}}_j \ge \bar R_{\text{req}} + \mathbf g_r^\top(\mathbf r_s - \bar{\mathbf r}_s) + \mathbf g_v^\top(\mathbf v_s - \bar{\mathbf v}_s) + g_t\,(t_s - \bar t_s),
\qquad s^{\text{rec}}_j \ge 0 .
$$

The slopes are finite differences of the whole boostback solve, with steps of 200 m, 2 m/s and 2 s, and are recomputed only when the reference moves. A need the oracle cannot solve leaves its row out for that iteration. The reserve itself is boxed, $-R_{0,j} \le \Delta R_j \le 0.98\,(\chi_j P_j + R_{0,j}) - R_{0,j}$: none at all, up to the same 98 % of the booster's own load that UPFG's reserve stops at. Boosters that leave on one stage share its reserve, and each holds it to its own site.

## Objective

The continuous objective is the most mass to orbit:

$$
\min\ \Big(1 - \frac{m(t_f)}{m_0}\Big).
$$

The subproblem's objective, in canonical units:

$$
\begin{aligned}
J_{\text{sub}} ={}& (1 - m_{N-1})
+ \lambda\sum_{k=0}^{N-2}\lVert\mathbf u_{k+1} - \mathbf u_k\rVert^2
+ w_\nu\sum_k\big\lVert\boldsymbol\nu_k\oslash\mathbf D\big\rVert^2
+ w_T\sum_{j=0}^{4}s_j^2 \\
&+ w_{\text{path}}\sum_k\big(s^q_k + s^{q\alpha}_k + s^{\text{pro}}_k\big)
+ w_{\text{accel}}\sum_k s^g_k
+ c_\varphi\,\lvert\varphi\rvert
+ w_{\text{rec}}\sum_j s^{\text{rec}}_j .
\end{aligned}
$$

| Term | Weight | Setting | Why |
| --- | --- | --- | --- |
| propellant | 1 | | the final mass over the lift-off mass |
| control smoothing | $\lambda = 4\times10^{-3}$ | `SmoothingWeight` | damps node-to-node chatter in attitude and throttle; skipped across coast boundaries |
| virtual control | $w_\nu = 10^5$ | `RhoVirtualControl` | scaled by $\mathbf D$ = (0.05 for position, $v^\star$ for velocity, 1 for mass), canonical |
| insertion slacks | $w_T = 10^5$ | `RhoTerminal` | canonical units |
| aero path slacks | $w_{\text{path}} = 1$ | `RhoPath` | linear, as a fraction of each aerodynamic limit |
| accel path slacks | $w_{\text{accel}} = 100$ | `RhoAccel` | linear, strong enough that high-TWR stages honour the g limit rather than buying fuel with slack |
| plane turn | $c_\varphi = 10^{-3}$ per radian | `PlaneTurnPrice` | a tie-break that keeps the launch where it was asked for when the propellant does not care |
| booster shortfall | $w_{\text{rec}} = 10^3$ | `RhoRecovery` | far above the propellant's price of 1, so the plan never buys mass to orbit by leaving a booster short |

The smoothing penalises the rate of the control, not its size, so it does not bias the burn times.

## What this solver does differently

### Settings for the common machinery

| Item | This solver |
| --- | --- |
| [Linearisation](index.md#linearisation) | analytic, by forward-mode automatic differentiation (`Dual`): ten seeded sweeps per node (plus one $\varphi$ sweep when `BodyForce` and a free `PlaneTurn` are active) give $\mathbf f$, $A$, $B$, $D_\varphi$, $\mathbf w$, its Jacobians, $q$ and $\nabla q$. Only the booster need is a finite difference. |
| [Discretisation](index.md#discretisation) | trapezoidal collocation, so the control enters at both ends of each interval like a first-order hold; no sub-steps in the subproblem; the true defect in the merit uses the same trapezoid with the true $\mathbf f$ |
| [Free time](index.md#free-final-time-and-phases) | one $\sigma_i$ per stage in canonical time; trust scale `StageSigScale(s)` = $\sigma_{\text{scale}}$ (300 s, `SigmaTrustSeconds`) on burn stages and $\max(\sigma_{\text{scale}}, \tfrac12(\sigma_{\max,s} - \sigma_{\min,s}))$ on coast stages |
| [Virtual control](index.md#virtual-control) | on all seven rows of every collocation interval, none on staging links; **squared two-norm**, weight $10^5$, scaled by $\mathbf D$ |
| [Trust region](index.md#trust-region) | hard box (infinity norm) on scaled $\mathbf x$, **acceleration-normalised** $\mathbf u$ with per-node scale $s_u(k) = \operatorname{clamp}(\min(u_{\text{cap}}(s,x),\ 6.0 / (T_{\text{full}}(s,x)/m_k)),\ 0.02,\ 1.0)$ (`ControlTrustScale`, preventing $180^\circ$ thrust flips on high-TWR g-limited stages), each free $\sigma_i$ (by `StageSigScale`), $\varphi$ (`PlaneTurnTrust` 0.05 rad) and $\Delta R_j$ (`RecoveryTrust` 0.2 $m_0$). $\eta_0 = 0.10$, $\eta \in [5\times10^{-4},\ 0.30]$. |
| [Accepting a step](index.md#accepting-a-step) | accept on $\rho > 0$; shrink by 0.5 on $\rho < 0.25$, accepted or not; grow by 1.5 on $\rho \ge 0.7$ when the step used at least 80 % of the box. When the model predicts no gain ($\le 10^{-12}$), the true change alone decides. A failed cone solve shrinks the box and retries the same reference. |
| Merit | the subproblem's objective term for term, with true defects, true insertion miss, true booster shortfall, aerodynamic path violations against full limits, and **`s_g`-only exact backoff merit** (`ExactBackoffMerit = true`, `RhoAccel = 100.0`): $J_{\text{accel}} = \rho_{\text{accel}}\sum_k \max(0,\ T_{\text{delivered},k}/(F_u a_{\max,s} m_k) - \beta_{\text{backoff}})$ with $\beta_{\text{backoff}} = 0.98$ |
| [Convergence](index.md#convergence) | predicted gain below $10^{-5}$ (`PredictedTolerance`) **and** every gate met by the reference: scaled defect $< 10^{-4}$, worst path violation $< 2\times10^{-3}$ of its limit, worst insertion residual $< 2\times10^{-5}$ canonical, booster shortfall $< 10^{-4}\,m_0$, pad roll met. Never on step size alone. |
| Stall and cap | six rejections or failures in a row at the smallest radius stop the solve (`StallMax`); 350 iterations at most (`MaxIterations`) |
| [Scaling](index.md#scaling) | canonical units; trust scales 500 km and 1000 m/s on Earth, carried to other bodies as the same fraction of the body radius and of the canonical speed; mass 1 |
| [Cone program](index.md#the-cone-program) | zero-allocation persistent `AscentSubproblem.Workspace` backed by `ClarabelWorkspace` (`SubproblemEps = 1e-6`, `UseClarabelPersistent = true`, `ClarabelIterativeRefinement = false`, `ClarabelEquilibrateMaxIter = -1`). Constant-signature assembly always emits all 3 `IU(k, 0..2)` entries (writing `0.0` instead of skipping) so `CcsAssembler.BeginRefill()` preserves exact CSC sparsity. Resets equilibration whenever any stage duration shifts by $\ge 2.0\times$ or $\le 0.5\times$, and enables terminal polish (`IR = true`, $\epsilon = 10^{-8}$) in the final 25 iterations when feasible. |
| [Re-solving in flight](index.md#re-solving-in-flight) | planned once on the pad; UPFG closes the loop after hand-over, with proportional energy terminal cutoff |

The trust floor sits below the script's $2\times10^{-3}$. Drag that grows with the angle of attack is curved in the thrust direction, and at the script's floor the plan crawled about a kilogram an iteration and stalled short of converging.

A stall is not always a failure. When the stalled reference passes every gate and the last step promised less than $10^{-3}$ of the lift-off mass (`StallAcceptGain`), the plan is flown: it cannot be proven optimal, but it can be flown.

### Seeding

The seed is a gravity turn flown through the same dynamics by RK4, 20 sub-steps per interval, so its defects are small from the first iteration. The vehicle rises vertically at its seed throttle. At body-scaled pitch-over speed $v_{\text{pitch,eff}} = \min(v_{\text{pitch}}, 0.02\,v_{\text{target}})$ (`PitchOverSpeed`, 130 m/s on Earth) its air-relative velocity is turned by the kick angle toward downrange. For airless bodies (`VacuumAscentAtmosphere`), the kick scan widens to $[75^\circ, 85^\circ]$. From then on it follows $\mathbf v_{\text{rel}}$, with the thrust projected into the target plane. Lower stages burn their loads, at full throttle or at the throttle the acceleration limit leaves. The last stage burns until the orbital energy reaches the target's (`SeedEnergyCutoff`), not for a fixed 80 % of its load.

The kick is searched: a log-spaced scan of 25 angles from 0.25 deg to 30 deg, slid up to four times when the best lands on an edge, then a golden section to $10^{-3}$ deg. A kick is scored on the insertion altitude and plane miss plus the path violation. The path term counts only what the best-behaved seed does not already have, so a vehicle that cannot meet $q_{\max}$ at full throttle is not driven to fly straight up. Seeds that blow up, fall 50 km below the mean radius or end retrograde are discarded. The lowest peak $q$ any seed flew is kept: it is close to the least this vehicle can manage, and the max-q retry uses it.

A plan of the same stages can stand in for the seed. Its throttle is raised to this problem's floor and its burn times clamped into this problem's bounds, or the first subproblem could be infeasible.

### Solid motors

A table, not the burn model. KSA's solid model is a chain of piecewise-linear tables and a fixed-point solve whose slope jumps at every table point. Nothing the plan decides reaches the grain: the throat is choked, so the grain burns back as a fixed function of time since ignition, and only the nozzle exit sees the air. So the game samples its own model once per stage, at 257 times and 17 back pressures, and the dynamics read that table. Pressure is an axis, not a factor on one time shape, because an overexpanded nozzle separates at low chamber pressure.

Because a stage with solids is pinned, each node sits at a fixed second of the table. Time is never differentiated, and the table's jumps cost nothing. Only the spline in back pressure is differentiated. The trapezoid over the nodes' mass flows is scaled so the stage burns exactly the table's grain; otherwise the virtual control would have to absorb the trapezoid's error.

### Recovery and retry

Boosters brought home are solved in one SCvx run from the seed, with the reserves free from the start. The library can instead solve first with each reserve held where it was set and free it from there (`RecoveryWarmStart`). On the dumped test problems that found 1 to 5 % more mass to orbit for two to six times the time, so it is off.

The solve sits inside a job (`AscentPlanJob`) that tries again when a solve fails:

1. **Stage count.** A solve that is not usable adds the next stage and solves again, while there is one. A converged plan whose last stage burns only its shortest allowed burn drops that stage and solves again.
2. **Max q.** When every seed climbed through $q_{\max}$ (the least seed peak above 98 % of it), the same stages are solved once more at $1.15$ times that peak, rounded up to 5 kPa, but never above the 195 kPa ceiling. The LAUNCH page says so.
3. **Stages held at full thrust.** An upper stage that lights above the hand-over altitude burns wholly under UPFG, which flies full thrust. If the converged plan throttles such a stage below 90 % of what it can give, it is solved again with that stage held at 99 %, starting from the converged plan. If the held solve does not converge, the throttled plan is kept and the page says which stage UPFG will fly differently. A solid launch with a pad roll that did not converge gets the same held solve, since holding the throttle can make its problem feasible.
4. **Lift and side-force continuation.** `AscentPlanJob` runs steps 1-3 with `DragOnly()` first, then runs a continuation pass with full body-axis lift and side force seeded from the converged drag-only plan. If the lift solve does not converge, the drag-only plan is kept with `LiftLeftOut = true`.

Every problem is also written to the temp folder as JSON (`AscentProblemFile`), so a plan that fails in the game can be solved again offline with the test console's `--ascent-replay`.

### The plan-fit check

A plan belongs to the vehicle, place, mass, lift-off instant, target and limits it was solved for. Before launching on it, and again when an armed launch reaches its window, `ConvexPlanFlyable` checks that:

- it is for this craft and body;
- the craft has moved less than 500 m over the ground and its mass by less than 1 %;
- the target orbit, MAX Q, MAX Q-ALPHA, MIN THROTTLE, the G-LIMIT and SOLVE LIFTOFF are unchanged;
- the pad roll program, the booster landing targets and the landing engine groups are unchanged;
- the lift-off is within 60 s of the instant it was planned for.

A converged plan that fails this when its solve lands, solved under time warp or across a change of target for example, is calculated once more. If that one does not fit either, nothing launches and the console shows **SOLVED PLAN DOES NOT FIT LAUNCH** with the reason, and offers the backup gravity turn. That is the player's choice, never made for them. At the window the same failure holds the launch with LAUNCH HELD AT WINDOW.

### How the plan is flown

The flight does not replay the plan's clock. It flies pitch and azimuth against air-relative speed, stage by stage: within a stage the index is the speed gained since that stage lit, and where speed dips the plan time runs on with the clock. A planned pad roll follows the elapsed solid burn instead. The throttle is the plan's fraction of the liquid engines' full thrust at the altitude flown, turned into a setting through the engines' own curve, and full whenever the plan is within 0.5 % of full. Above the hand-over altitude, once UPFG has converged, or when the plan runs out, the command blends onto UPFG's steering over 15 s. A solid launch with a planned pad roll stays on the plan until its first stage ends.

## Closed-loop UPFG and terminal cutoff

After hand-over from the open-loop convex ascent profile, `UpfgGuidance` closes the loop to orbit insertion, and `Modes/Ascent.cs` / `Modes/Autopilot.cs` enforce terminal energy cutoff and escape protection.

### UPFG singularity guards and sensed velocity

- **Non-gravitational sensed velocity:** Each cycle subtracts trapezoidal gravity from velocity change before updating velocity-to-go:

    $$
    \Delta\mathbf v_{\text{sensed}} = (\mathbf v - \mathbf v_{\text{prev}}) - \tfrac12(\mathbf g_{\text{prev}} + \mathbf g_{\text{curr}})\,\Delta t,
    \qquad
    \mathbf v_{\text{go}} \leftarrow \mathbf v_{\text{go}} - \Delta\mathbf v_{\text{sensed}}.
    $$

- **Radial range-to-go and denominator guards:** When $|\hat{\boldsymbol\lambda}\cdot\hat{\mathbf i}_z| < 0.15$, radial range-to-go clamps to $\operatorname{clamp}(\hat{\mathbf i}_z\cdot\mathbf r_{\text{go}}, -|S|, |S|)$ and clears `Converged = false` instead of dividing by a near-zero projection:

    $$
    r_{\text{go},z} = \begin{cases}
    \dfrac{S - \hat{\boldsymbol\lambda}\cdot\mathbf r_{\text{go},xy}}{\hat{\boldsymbol\lambda}\cdot\hat{\mathbf i}_z}, & |\hat{\boldsymbol\lambda}\cdot\hat{\mathbf i}_z| \ge 0.15 \\[6pt]
    \operatorname{clamp}(\hat{\mathbf i}_z\cdot\mathbf r_{\text{go}},\ -|S|,\ |S|), & |\hat{\boldsymbol\lambda}\cdot\hat{\mathbf i}_z| < 0.15
    \end{cases}
    $$

    Denominator guards protect $\lambda_{\text{de}} = Q - SJ/L$ ($|\lambda_{\text{de}}| > 10^{-6}$), $\dot\phi = -\phi L / J$ ($|J| > 10^{-9}$) and the turning term in $\mathbf r_{\text{thrust}}$ ($\lVert\dot{\boldsymbol\lambda}\rVert > 10^{-12}$). `Converged` is cleared on any singularity, non-finite value or divergence.

### Orbital escape cutoff invariant

During `AscentPhase.ClosedLoop` or `AscentPhase.Terminal` for any bound target orbit ($a_{\text{target}} = R_{\text{body}} + 500(\text{Pe}_{\text{km}} + \text{Ap}_{\text{km}})$, $E_{\text{target}} = -\mu / (2\max(a_{\text{target}}, R_{\text{body}})) < 0$), `Modes/Ascent.cs` checks specific orbital energy $E_{\text{now}} = \tfrac12\lVert\mathbf v\rVert^2 - \mu/\lVert\mathbf r\rVert$ every step and cuts thrust immediately if

$$
E_{\text{now}} \ge 0 \quad\lor\quad \Big(E_{\text{target}} < -0.05\,\frac{\mu}{r} \;\land\; E_{\text{now}} > 0.25\,E_{\text{target}}\Big).
$$

The $-0.05\,\mu/r$ gate allows high-apogee near-escape transfers while strictly blocking accidental hyperbolic escape.

### Proportional energy terminal cutoff

Discrete physics-step cutoff quantization ($0.5\text{--}1.0\,\Delta t\,a_{\text{burn}}$) dominates high-warp orbit-insertion overshoot, with a smaller contribution from KSA's 1-step lagged Velocity Verlet gravity acceleration $\mathbf a_{\text{err}} \approx \Delta t\,(\mu/r^3)\mathbf v$. Within the final $3.0\text{ s}$ of `AscentPhase.Terminal` ($\Delta t = \operatorname{clamp}(\text{simSpeed}/60,\ 1/120,\ 1.0)$), `TerminalCutoffThrottle` computes:

$$
\Delta E_{\text{verlet}} = \Delta t\cdot\Big(\tfrac12\Delta t\,\frac{\mu}{r^3}v\Big)v,
\qquad
\Delta E_{\text{thrust}} = \Delta t\cdot a_{\text{burn}}\cdot\max(\mathbf v\cdot\hat{\mathbf u},\ 0.1 v),
$$

$$
E_{\text{need}} = E_{\text{target}} - E_{\text{now}} - \Delta E_{\text{verlet}},
\qquad
f = \frac{E_{\text{need}}}{\Delta E_{\text{thrust}}}.
$$

- If $E_{\text{now}} \ge E_{\text{target}}$ or $E_{\text{need}} \le 0$: cut immediately (`EngineOn = false`, `CutoffDone = true`).
- If $0 < E_{\text{need}} < \Delta E_{\text{thrust}}$: cut if $f < 0.5\,u_{\min}$; otherwise command $u = \operatorname{clamp}(u_{\text{base}} f,\ u_{\min},\ u_{\text{base}})$.
- On degenerate/non-orbital cutoffs with $t_{\text{rem}} < \Delta t$, the same nearest-energy rule applies with $f = t_{\text{rem}} / \Delta t$.

This holds apoapsis and semi-major-axis error well below $0.1\text{ km}$ across $1\times$, $4\times$ and $10\times$ time warp.

## Code map

| File | Main type or method |
| --- | --- |
| [AscentScvx.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentScvx.cs) | `AscentScvx.Solve`: the SCvx loop, merit (`TrueCost`), gates and warm start; `AscentSolution` |
| [AscentSubproblem.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentSubproblem.cs) | `AscentSubproblem.Solve`: every row and cone of the subproblem; `Linearization` |
| [AscentDynamics.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentDynamics.cs) | `AscentDynamics`: the dynamics, $q$, $\mathbf w$ and their Jacobians by forward-mode AD; `SolidPoint` |
| [AscentModel.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentModel.cs) | `AscentProblem`, `AscentStage`, `AscentSettings` (every default above), `KsaAscentAtmosphere` |
| [AscentCase.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentCase.cs) | `AscentCase`: canonical units, the node mesh, loads, reserves, burn-time bounds, pad roll, `TerminalResidual` |
| [AscentPhasePlan.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentPhasePlan.cs) | `AscentPhasePlan.Build`: KSA's phases to stages, solids and throttled cores; `Truncate` |
| [AscentSolid.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentSolid.cs) | `AscentSolidBurn`: the sampled solid table |
| [AscentSeed.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentSeed.cs) | `AscentSeed.Search`: the flown gravity-turn seed and kick search |
| [AscentRecovery.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentRecovery.cs) | `AscentRecovery`, `BoostbackOracle.Evaluate`: a booster's need and its slopes |
| [AscentMaxQ.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentMaxQ.cs) | `AscentMaxQ.Relaxed`: the max-q retry |
| [BoosterReturn.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/BoosterReturn.cs) | `BoosterReturn.Compute`: each booster's way down, for the pictures |
| [AscentProblemFile.cs](../../../AdvancedFlightComputer/Features/Guidance/Scvx/Ascent/AscentProblemFile.cs) | `AscentProblemFile.ToJson`, `FromJson`: problem dumps for replay |
| [KsaAscentPlan.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/Ascent/KsaAscentPlan.cs) | `AscentPlanJob`: retries on the worker; `AscentPlan`; `ConvexAscentProfile`: the speed-indexed profile |
| [KsaAscentStages.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/Ascent/KsaAscentStages.cs) | `KsaAscentStages.Gather`: phases and solid motors from the live vehicle |
| [ConvexAscent.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/ConvexAscent.cs) | `TryStartAscentPlan`: the problem from the game; `ConvexPlanFlyable`, `ContinueConvexLaunch`, `ConvexProfileDone`, `ConvexThrottle` |
| [Ascent.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/Ascent.cs) | the ascent's Profile phase, `PadHold`, the roll program and the blend onto UPFG |
