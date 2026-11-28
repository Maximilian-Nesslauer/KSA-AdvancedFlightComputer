# Orbital entry and bellyflop

Orbital entry guidance brings a belly-first craft from orbit down to a few kilometres above its landing site. It has two planners. The **entry planner** chooses the vertical lift over drag against energy, from the first touch of the air to a hand-over at about Mach 5. The **bellyflop planner** then chooses the belly's pitch, bank and yaw against the airflow over a free flight time, so the craft falls onto a point just short of the site. [Touchdown guidance](scvx/touchdown.md) takes the end: the flip and the landing burn.

Both planners are SCvx in the same light form: RK4 multiple shooting, Jacobians by central differences, virtual control and slacks priced as exact penalties, an $L_\infty$ trust region accepted on the true merit, and one Clarabel QP per iteration. They live in `Scvx/OrbitalEntry` and `Scvx/Bellyflop` and know nothing of the game. The machinery that every SCvx solver in AFC shares is on the [common page](scvx/index.md). The player's view is in the [User Guide](../user-guide/powered-guidance.md#orbital-entry-and-the-flip-landing); the history of each change, with its flight and harness evidence, is in the [changelog](CHANGELOG.md) and the [branch record](orbital-entry-branch.md).

## The chain

| Step | Mode | Ends when | Then |
|---|---|---|---|
| Aimed deorbit | DEORBIT ONLY with AIM AT SITE (on) | The burn ends | ORBITAL ENTRY, if ENTRY AFTER DEORBIT is on |
| Entry | ORBITAL ENTRY | Bellyflop's first converged plan from BELLYFLOP FROM MACH (10); at the latest the hand-over energy (HAND-OVER ALTITUDE 38 km, HAND-OVER MACH 5) | Bellyflop, retro burn, parachutes or hold |
| Bellyflop | last phase of ORBITAL ENTRY | 1 s before touchdown's plan turns 15 deg from the attitude flown; at the latest 5 s before touchdown's ignition or 12 s before bellyflop's ground | Touchdown guidance |
| Flip and landing burn | Touchdown guidance (6-DOF) | Contact | |

**Deorbit to entry.** `PlanEntryBurn` hands DEORBIT ONLY's burn to `PlanAimedEntryBurn` when AIM AT SITE (`OrbitalEntryAimDeorbit`) is on, whether or not the entry follows. At the end of the burn `MonitorStockDeorbit` sets `OrbitalEntry.EngagePending` when `OrbitalEntryAfterDeorbit` is on; the entry engages on the next step, ahead of the claim.

**Engage.** `EngageOrbitalEntry` refuses a craft that cannot be controlled, a body with no atmosphere, or a craft with an engine burning. It starts a belly-first aero sweep and sits in `Sweeping` until the sweep lands. It also records whether the craft has an engine to land on (`OrbitalEntryBellyflop = LandEngineOnCraft`).

**After the entry.** What follows depends on the craft and on the LAND page's GLIDE row (`OrbitalEntryFlip`, set from FLIP AFTER ENTRY or LAND NOW's FLIP LANDING):

* **Bellyflop** (`EntryFlips`: an engine to land on and FLIP LANDING). Bellyflop solves in the background once Mach falls to BELLYFLOP FROM MACH. It takes the craft at its first converged plan, during the entry or while the entry holds after the hand-over. HAND-OVER MACH is only the latest point.
* **Retro burn** (`EntryRetro`: an engine to land on and RETRO BURN). At the hand-over energy the entry asks once for the 3-DOF (`ThreeDofEngagePending`) and holds its attitude until the 3-DOF claims the craft. The entry aims its hand-over RETRO AIM SHORT (30 km) short of the site.
* **Parachutes** (no engine to land on). At the hand-over the entry arms every parachute (`KsaParachutes.ArmAll`). Each opens at its own deploy altitude, and the entry lets go once one is open.
* **Hold.** Otherwise the entry slews its bank to zero and holds the entry attitude.

**BELLYFLOP DIRECT.** If ORBITAL ENTRY is engaged already past the hand-over energy and has never planned, the entry is skipped. The craft holds the attitude it had (`HoldNoseCci`, `HoldBellyCci`) until bellyflop's first plan.

## Entry planner at a glance

| | |
|---|---|
| Independent variable | Normalised energy $e = 1/r - V^2/2$, which rises through the entry |
| State | $\mathbf x = (r, \gamma, s) \in \mathbb R^3$: radius, flight path angle, range to go (rad of arc) |
| Controls | $u = (L/D)\cos\sigma$, the vertical lift over drag. Bank does not appear |
| Parameters | None. The final energy is fixed by the hand-over altitude and Mach |
| Nodes | $N = 60$, spaced evenly in speed at a mean radius |
| Discretisation | RK4 multiple shooting, 6 sub-steps an interval, first-order hold on $u$ |
| Cone solver | Clarabel, one QP per iteration (quadratic objective, linear constraints only) |
| Where it runs | Its own worker thread (`Ksa3DofWorker`). The step reads only the published `EntryPlan` |
| Re-solve cadence | Every RE-SOLVE EVERY (2 s) warm from the plan before, 15 iterations (60 cold). Before the plan's start drag, every 10 s from a predicted coast |

## Entry: the problem

### Model

The planner works in normalised units: length by the body radius $R_0$, speed by $\sqrt{\mu/R_0}$, acceleration by $g_0 = \mu/R_0^2$. `EntryModel` gives density and speed of sound from the sweep's atmosphere, and drag coefficient and L/D as functions of Mach alone, read along the angle-of-attack schedule (see [Angle of attack](#angle-of-attack)). Each re-solve applies the estimator's density scale and L/D scale to every evaluation.

With $D$ the drag acceleration, the dynamics are Lu's (2008) longitudinal equations with energy as the independent variable:

$$
\begin{aligned}
r' &= \frac{\sin\gamma}{D_e} \\
\gamma' &= \frac{u D + (V^2 - 1/r)\cos\gamma / r + V\dot\gamma_\Omega}{D_e V^2} \\
s' &= -\frac{\cos\gamma}{r D_e}
\end{aligned}
\qquad
D_e = \max(D - \dot V_\Omega,\ 10^{-5}).
$$

$V$ is not a state; it follows from $e$ and $r$. The rotating-body terms $\dot V_\Omega$ and $V\dot\gamma_\Omega$ (`EntryModel.RotationTerms`, Lu's Eqs. 4 and 5) carry the Coriolis and centrifugal terms at a latitude and heading held fixed for the solve:

$$
\begin{aligned}
\dot V_\Omega &= \omega^2 r\cos\phi\,(\sin\gamma\cos\phi - \cos\gamma\sin\phi\cos\psi) \\
V\dot\gamma_\Omega &= 2\omega V\cos\phi\sin\psi + \omega^2 r\cos\phi\,(\cos\gamma\cos\phi + \sin\gamma\cos\psi\sin\phi)
\end{aligned}
$$

with $\omega$ the body's rotation in planner units, $\phi$ the latitude and $\psi$ the heading from north. On a shallow coast these terms are not small: without them the planner predicted the coast to meet the air too soon.

The range to go is measured **along the ground track**, signed, so it goes negative once the site is behind. The planner is longitudinal. Crossrange is the tracker's job, through bank reversals.

**Before the plan.** The plan starts where drag first reaches `PlanStartDrag` (0.5 m/s²). Before that, `PredictStart` coasts the craft in the time domain (RK4, 0.5 s steps, up to 4 h) at the bank it flies there, `PreStartBankDeg` (100 deg), and the plan starts from the predicted state. In air that thin the energy hardly changes while the altitude falls tens of kilometres, so a plan started earlier does not converge.

### Constraints

| Constraint | Form | How it is kept |
|---|---|---|
| Initial state | $\mathbf x_0$ = measured or predicted start | Hard |
| Dynamics | $\mathbf x_{k+1} = \Phi(\mathbf x_k, u_k, u_{k+1})$, linearised | Virtual control $w$, priced at `PenaltyWeight` (1000) per scaled unit |
| Vertical L/D | $(L/D)\cos\sigma_{\max} \le u_k \le (L/D)\cos\sigma_{\min}$ | Hard. PLAN MIN BANK 15 deg, PLAN MAX BANK 100 deg |
| Roll rate | $\lvert u_{k+1} - u_k\rvert \le \lvert L/D\rvert\,\dot\sigma_{\text{plan}}\,\Delta t_k$ | Hard. $\dot\sigma_{\text{plan}}$ is half of ROLL RATE (`PlanRollRateShare` 0.5), so reversals still fit |
| Path limits | $r_k \ge r_{\text{floor}}(e_k)$ | Slack, priced at `FloorShare` (0.1) of the penalty |
| Hand-over box | $\lvert h_N - h_{\text{hand}}\rvert \le 3$ km | Slack, priced at `TargetShare` (1.0) of the penalty |
| Terminal range | $s_N$ in bellyflop's reach band | Slack outside the band at `RangeBandShare` (0.5), and a pull to the aim inside it at `RangePullShare` (0.05) |
| Trust region | $\lvert \mathbf x_k - \bar{\mathbf x}_k\rvert \le \eta\,\mathbf s_x$, $\lvert u_k - \bar u_k\rvert \le \eta\,0.1$ | Hard, $L_\infty$ |

The scales are 5 km of radius, 1 deg of flight path angle and 20 km of range; vertical L/D is scaled by 0.1. $\Delta t_k$ is the interval's energy step over $D V$ at its midpoint.

**Path limits become altitude floors.** Heating, dynamic pressure and load all grow as the craft flies lower at a fixed energy, because density and speed both rise. So each limit becomes the highest radius at which it is reached, `EntryDynamics.FloorRadius`: a 64-point scan down from the top of the air, then 40 bisections. The floor depends on energy alone, so it is worked out once per solve. The limits are:

* **Dynamic pressure** $q = \tfrac12\rho V^2 \le$ MAX DYNAMIC PRESSURE (16 kPa).
* **Load** $D\sqrt{1 + (L/D)^2}/g_{\text{std}} \le$ MAX LOAD (2.5 g).
* **Stagnation heating**, Sutton-Graves: $\dot Q = k\sqrt{\rho / R_n}\,V^3 \le$ MAX HEATING (440 kW/m²), with $k = 1.7415\times10^{-4}$ (SI) and $R_n$ the NOSE RADIUS (1 m). The game does not heat craft yet, so this only shapes the trajectory.
* A fixed floor of 5 km (`AltitudeFloor`).

A limit of zero is off. The floors are priced at a tenth of the dynamics so that a limit the craft cannot meet is missed as little as it can be, and the plan never bends its dynamics to it. A heating limit priced like the dynamics once had every plan refused from the heating peak on.

**Least heating.** `EntryDynamics.LeastPeakHeating` flies the equilibrium glide at the plan's most lift up ($L/D\cos 15°$) over 400 energy steps and returns the highest heating on it. No plan peaks lower. When MAX HEATING is below it, the status says so (LEAST HEATING), and the log says it again when the figure moves by more than 5 %.

### The reach band and the corridor

**The reach band.** When bellyflop follows, the entry does not aim its hand-over at the site. Before each entry re-plan, `UpdateBellyflopRangeTarget` predicts bellyflop's fall from the entry plan's own hand-over state with `BellyflopPlanner.FallRange`. It flies the bellyflop model with the pitch easing at the angle rate from the entry's angle of attack to a held pitch:

* the aim, `TerminalRangeM`: the fall held at 20 deg of pitch (`BellyflopAimPitchDeg`), a little longer than flat, so the site sits inside bellyflop's reach;
* the short edge: the fall held at $-0.75 \times 20$ deg (nose down, `LimitMargin` of BELLYFLOP PITCH DOWN);
* the long edge: the fall held at $+0.75 \times 55$ deg (nose up, `LimitMargin` of BELLYFLOP PITCH UP).

Each has BELLYFLOP AIM SHORT (200 m) added. The planner prices the terminal range only outside $[s_{\text{lo}}, s_{\text{hi}}]$, with a light pull towards the aim inside. A point target closed the corridor on a spot that moved 9 km as bellyflop's prediction changed, and every later plan was refused.

**The corridor.** `EntryPlanner.ReachBand` gives the range the craft can still fly from each node to the hand-over, flown with the planner's own dynamics from that node's state:

* long: full lift up ($\sigma_{\min}$) all the way;
* short: full lift down ($\sigma_{\max}$) until the craft reaches the floor it can hold, then along that floor, at range per energy $1/D$.

The floor it can hold (`HeldFloor`) is the highest of the path limits' own floors that lies at or under the full-lift glide at that energy. A limit the craft cannot hold there does not bound the corridor, since every flight breaks it.

### Objective

$$
J = w_Q\frac{Q}{Q_{\text{seed}}}
+ w_{\Delta}\sum_k\Big(\frac{\Delta u_k}{0.1}\Big)^2
+ w_{\kappa}\sum_k\Big(\frac{\Delta^2 u_k}{0.1}\Big)^2
+ \frac{w_c}{N}\sum_k\Big(\frac{u_k - u^{\text{prev}}_k}{0.1}\Big)^2
+ \frac{w_m}{N}\sum_{k<N-1}\Big(\frac{s_k - s^{\text{mid}}_k}{h_k}\Big)^2
+ \text{penalties}
$$

The heat load per unit energy is $1/(C_D\sqrt\rho)$, in which speed cancels. It enters the QP as its second-order expansion in $r$ about the reference, with the curvature $f'^2/f$, which is exact for an exponential atmosphere and never concave. $Q_{\text{seed}}$ is the seed's heat load. The corridor margin's middle $s^{\text{mid}}_k$ and half-width $h_k$ come from the seed's corridor, the half-width floored at the larger of a quarter of the widest half-width and 50 km.

| Weight | Value | Config field | Why |
|---|---|---|---|
| Heat load $w_Q$ | 1.0 | `HeatWeight` | Least heat load, normalised by the seed's |
| Change $w_\Delta$ | 0.05 | `SmoothWeight` | Smooth vertical L/D between nodes |
| Curvature $w_\kappa$ | 0.5 | `CurvatureWeight` | A reference that bends gently, so the tracker and the flaps do not chase corners |
| Consistency $w_c$ | 0.2 | `ConsistencyWeight` | Warm re-solves stay near the plan being flown |
| Corridor margin $w_m$ | 1.0 | `CorridorWeight` (CORRIDOR MARGIN) | The heat load alone pushes the plan to the high-energy edge, which leaves no drag to spare at the end |
| Dynamics defect | 1000 per scaled unit | `PenaltyWeight` | Exact penalty |
| Floors | 0.1 of the penalty | `FloorShare` | Below the dynamics, so an unreachable limit is missed, not obeyed by bending the dynamics |
| Hand-over box | 1.0 of the penalty | `TargetShare` | |
| Range outside the band | 0.5 of the penalty | `RangeBandShare` | |
| Pull to the aim inside the band | 0.05 of the penalty | `RangePullShare` | |
| Proximal | $10^{-3}$ per scaled unit² | `ProximalWeight` | A unique step |

### Seed, iterations and the gate

The cold seed flies a constant $\cos\sigma$ and bisects it (30 steps) on the final range. A warm seed takes the plan before at each node's energy. The trust region starts at $\eta = 2$ cold or 1 warm. A step is refused below a ratio of 0.1, $\eta$ halves below 0.25 and doubles above 0.7, up to 20. The solve stops early once the defect is under a quarter of the gate, the step under 0.02 and the range under a quarter of the range gate.

A plan is **converged** when its largest scaled defect is at most `GateDefect` (0.25: 1.25 km, 0.25 deg, 5 km of range) and it ends at most `GateRangeM` (25 km) outside the band. A refused plan is flown only while there is no plan at all.

## Entry: the flying side

### The tracker

`EntryTracker` flies the plan. Its form is the Shuttle entry guidance's: a damped spring on the drag error, from $\delta D/D = -\delta h/H$.

$$
u_{\text{cmd}} = u_{\text{ref}}(e) + \operatorname{clamp}\!\Big(\frac{\omega^2 H}{\bar D^2}(D_f - D_{\text{ref}}) - \frac{2\zeta\omega}{\bar D}(\dot h_f - \dot h_{\text{ref}}),\ \pm 0.35\,(L/D)\Big),
\qquad
\lvert\sigma\rvert = \arccos\frac{u_{\text{cmd}}}{(L/D)_{\text{meas}}}
$$

* $\omega$ is DRAG TRACKING (`TrackOmega`, 0.03 rad/s) and $\zeta$ is `TrackZeta` (0.7). $H$ is the local scale height from the atmosphere table.
* $\bar D = \max(D_{\text{ref}}, 10)$ m/s² (GAIN DRAG FLOOR, `GainDragFloor`). The gains go as $1/D$ and $1/D^2$, and are taken at the plan's smooth reference drag, not the measured one. Below the floor the loop keeps its damping and slows in proportion.
* $D_f$ and $\dot h_f$ are the measured drag and sink rate through a first-order low-pass of DRAG FILTER (`MeasureSmoothS`, 4 s).
* The feedback is clamped to `FeedbackShare` (35 %) of the craft's measured L/D either way. The plan, re-solved every few seconds, does the work; the feedback only trims it.
* The command is held to what the craft can give, $(L/D)\cos\sigma_{\text{over}} \le u \le L/D$, then low-passed by COMMAND SMOOTHING (`CommandSmoothS`, 0 s, off).

The measured drag and L/D come from the game's own aero force on the craft (`KsaAeroSweep.TryLiveAero`). The entry also learns a density scale and an L/D scale from them against its model: a 5 s low-pass, clamped to 0.5-2 and 0.5-1.5, and only above 0.3 m/s² of drag. Both stand still once bellyflop flies, because the entry's model does not describe a belly-first craft.

**Over the plan's bank.** The tracker may bank to TRACKER MAX BANK (`OverbankDeg`, 120 deg), further than the plan's 100 deg, to absorb what the plan did not foresee. Past the plan's limit is ordinary tracking, only flagged in the log.

**Bank reversals.** The bank's side turns the lift towards the site. It reverses when the heading error to the site (the angle between the ground track and the great circle to the site) leaves a band:

$$
\delta(V) = \text{REVERSAL BAND} \times \operatorname{clamp}\!\Big(\frac{V/\sqrt{\mu/R_0}}{0.25},\ 0.2,\ 1\Big),
$$

5 deg at speed, narrowing below about 2 km/s on Earth so the craft lines up on the site at the end. A reversal rolls at ROLL RATE (5 deg/s) through lift-up; tracking slews at TRACKING ROLL RATE (5 deg/s). Where the attitude controller reports the acceleration it can spare about the airflow, the slew is also a reference governor: its rate changes by no more than half that acceleration per second, and never exceeds $\sqrt{2a\,\Delta\sigma}$, the rate it can stop from in the bank left.

**Before the plan.** The tracker takes hold, latched, once drag reaches START DRAG (0.02 m/s²) or, with ENGAGE IN AIR on, once the craft is below the top of the atmosphere. Before that it holds wings level, lift up. In the air but short of the plan's start energy it flies `PreStartBankDeg` (100 deg) on the side towards the site, the bank the coast was predicted with.

**Reach warnings.** The status sets the site against the plan's corridor at the current energy (`EntryPlan.RangeShortM`, `RangeLongM`). Closer than the short edge is HIGH ENERGY, further than the long edge LOW ENERGY. Either warning is dropped while a converged plan reaches the site within the range gate. ATTITUDE NOT HELD is shown when the angle of attack flown is more than 10 deg off the one asked.

### Angle of attack

By default the entry flies ENTRY AoA (55 deg from nose-first) at every Mach: TRIM AoA is off (`OrbitalEntryTrimAoa`), and `EntryTrim.Fixed` reads drag and L/D from the first sweep's belly-first rows at that angle.

With TRIM AoA on, `EntryTrim.Build` takes at each Mach row the angle where $C_m$ falls through zero, restoring, inside TRIM AoA MIN to TRIM AoA MAX (40 to 70 deg), nearest ENTRY AoA. A row with no such zero takes the angle of least $\lvert C_m\rvert$, read every 0.5 deg, and is marked untrimmed. Once the belly-first body and flap tables are in, `RetrimWithFlaps` redoes the schedule every 1 deg: each row takes the angle where the body's pitch moment lies inside what the flaps can add either way, the most margin each side winning.

The belly side is the one BELLY AXIS asks for (0, the craft's own belly, by default). When BELLY AXIS lets the sweep choose, the side with the best score wins: ten points a trimmed row, less a thousand times the residual $\lvert C_m\rvert$, plus the mean L/D. The entry refuses a side with L/D of 0.02 or less at Mach 10.

**The attitude.** With the angle of attack $\alpha$ and the bank $\sigma$:

$$
\hat{\mathbf l} = \cos\sigma\,\hat{\mathbf u}_\perp + \sin\sigma\,(\hat{\mathbf v}\times\hat{\mathbf u}_\perp),
\qquad
\text{nose} = \cos\alpha\,\hat{\mathbf v} + \sin\alpha\,\hat{\mathbf l},
\qquad
\text{belly} = \sin\alpha\,\hat{\mathbf v} - \cos\alpha\,\hat{\mathbf l},
$$

with $\hat{\mathbf u}_\perp$ the local up across the airflow. The target's turning rate is fed forward to the attitude controller, smoothed over 0.3 s, and zeroed after a jump of more than 10 deg in one step.

AFC's attitude controller flies the target with **OWN ATTITUDE** on (the default): the authority-aware capture, each axis owned by the flaps or the RCS, and the RCS on the phase plane across **RCS DEAD BAND** (4 deg), with **ENTRY RCS** letting it fire at all. See [Authority-aware capture](attitude/index.md#authority-aware-capture), [Per-axis ownership](attitude/index.md#per-axis-ownership) and [Phase-plane RCS](attitude/rcs.md#phase-plane-rcs). Bellyflop flies through the same controller.

## Aimed deorbit

A DEORBIT ONLY burn on its own goes at the next chance, so the entry starts wherever the orbit happens to be. `PlanAimedEntryBurn` (in `OrbitalEntryAim.cs`) times it so the site sits at AIM IN CORRIDOR (0.5, the middle) of the corridor when the craft reaches the air.

1. **Measure.** The same belly-first sweep and schedule as the entry takes. It is swept again if the body, the mesh hash or the belly side changes, and the schedule is rebuilt if the AoA settings change.
2. **Corridor.** From one reference burn, the coast from the entry interface to the start drag (`PredictStart`) and the corridor there (`EntryPlanner.Corridor`, flown as `ReachBand` from that one state). Every burn time gives about the same entry energy, so this is done once. The terminal range is added: bellyflop's predicted fall (or BELLYFLOP AIM SHORT before there is one), or RETRO AIM SHORT.
3. **Search.** Departures are scanned from 120 s ahead over AIM HORIZON (24 h), in steps of a 180th of the orbital period (10 to 60 s). A pass counts only if its ground track comes within AIM MAX CROSSRANGE (200 km) of the site. Where the site's range ahead at the start of the air crosses the aim, the departure is bisected to 0.05 s. Burns are horizontal, down to the entry periapsis, so the crossrange is the pass's.
4. **Lambert.** With that departure and flight time, `DeorbitPlanner.TryLambert` aims at the entry point itself: at the top of the air, on the great circle to the site, the coast and the aimed range short of it. It is taken only if it costs at most the larger of 25 % more or 10 m/s more than the horizontal burn, and meets the air within 0.5 deg of the horizontal burn's flight path angle, for which the corridor was worked out.
5. **Check.** One cold plan from the predicted start, with twice the cold iterations (at least 50). It passes when it misses by no more than the range gate (25 km) and its defect is within five times the flight gate. Otherwise the aim moves, by the start error if the burn put the site more than the gate away from where it was aimed, else by the plan's miss, and searches again, up to three tries.

Ranges are measured over the ground in the body's turning frame, from where the site is when the craft reaches the top of the air.


## Bellyflop planner at a glance

| | |
|---|---|
| Frame | Landing-site frame: origin at the site's ground, $+z$ up, turning with the body. The plan's positions are about the aim point, BELLYFLOP AIM SHORT (200 m) short of the site along the bearing |
| State | $\mathbf x = (\mathbf r, \mathbf v) \in \mathbb R^6$. No mass: nothing burns |
| Controls | $\mathbf u = (\alpha, \beta, \psi) \in \mathbb R^3$: pitch and bank (the airflow's angles on the belly) and yaw about the airflow |
| Parameters | Flight time $\sigma \in [5, 900]$ s, free |
| Nodes | $N = 30$, even in time |
| Discretisation | RK4 multiple shooting, 4 sub-steps an interval, first-order hold on $\mathbf u$, time scaled by $\sigma$ |
| Cone solver | Clarabel, one QP per iteration (linear constraints only) |
| Where it runs | Its own worker (`BellyflopState.Worker`), from BELLYFLOP FROM MACH (10) |
| Re-solve cadence | Every BELLYFLOP RE-SOLVE (1 s), warm, 12 iterations (40 cold) |

## Bellyflop: the problem

### Attitude from the airflow

The angles are relative to the airflow, not inertial Euler angles. `BellyflopModel.Axes` builds a reference frame at each state from its own velocity $\hat{\mathbf f} = \mathbf v/\lVert\mathbf v\rVert$:

* belly normal along $\hat{\mathbf f}$: the belly leads;
* reference nose $\mathbf n_0$: the part of $\hat{\mathbf c} + \hat{\mathbf u}$ across $\hat{\mathbf f}$, with $\hat{\mathbf c}$ the horizontal bearing to the site and $\hat{\mathbf u}$ local up. In shallow flight the nose stands up the vertical plane through the bearing; falling near vertically it lies along the bearing. It is singular only for a climb at 45 deg along the bearing;
* yaw $\psi$ turns the nose about $\hat{\mathbf f}$.

The body is then the reference turned by the smallest rotation that puts the airflow at $(\tan\alpha, \tan\beta, 1)/\lVert\cdot\rVert$ in body axes, so the tables are read at exactly the angles the solver chose. $\alpha$ is towards the nose (nose up, positive), $\beta$ towards the span. They are the belly-first table's own angles: $\alpha = \operatorname{atan2}(\mathbf v\cdot\hat{\mathbf n}, \mathbf v\cdot\hat{\mathbf b})$ and $\beta = \operatorname{atan2}(\mathbf v\cdot\hat{\mathbf s}, \mathbf v\cdot\hat{\mathbf b})$. The bearing $\hat{\mathbf c}$ follows the site until bellyflop takes over, and is held from then.

Angles, rather than a unit vector or a quaternion, keep every limit a box. The flip, where the attitude swings 90 deg, belongs to touchdown guidance.

### Dynamics

$$
\dot{\mathbf r} = \mathbf v,
\qquad
\dot{\mathbf v} = -\mu\frac{\mathbf r_c}{\lVert\mathbf r_c\rVert^3} - 2\boldsymbol\Omega\times\mathbf v - \boldsymbol\Omega\times(\boldsymbol\Omega\times\mathbf r_c) + \frac{s_F\,qS}{m}\big(C_n\hat{\mathbf n} + C_s\hat{\mathbf s} + C_b\hat{\mathbf b}\big),
$$

with $\mathbf r_c$ the position from the body's centre (the site's radius below the origin), $q$ from the sweep's density at the altitude above the site's ground, and $C_n, C_s, C_b$ the belly-first body table (`BellyBodyAero`) projected onto bellyflop's nose, span and belly normal (`KsaBellyflopSetup.Coefficients`). Mach is held inside the table's grid. $s_F$ is bellyflop's own force scale (see [the flying side](#bellyflop-the-flying-side)).

### Constraints

| Constraint | Form | How it is kept |
|---|---|---|
| Initial state | $\mathbf x_0$ = measured, shifted to the aim | Hard |
| Dynamics | $\mathbf x_{k+1} = \Phi(\mathbf x_k, \mathbf u_k, \mathbf u_{k+1}, \sigma)$, linearised | Virtual control, `PenaltyWeight` ($10^4$) per scaled unit (1 km, 10 m/s) |
| Terminal altitude | $h(\mathbf r_N) = 0$, linearised | Slack pair at the dynamics' price |
| Pitch | $-\bar\alpha^{-}_k \le \alpha_k \le \bar\alpha^{+}_k$ | Hard. BELLYFLOP PITCH DOWN 20 deg, BELLYFLOP PITCH UP 55 deg |
| Bank | $\lvert\beta_k\rvert \le \bar\beta_k$ | Hard. BELLYFLOP BANK 45 deg, held to what the flaps can roll-trim (below) |
| Yaw | $\lvert\psi_k\rvert \le \bar\psi_k$ | Hard. `YawMaxDeg` 90 deg off the bearing |
| Start | $\lvert \mathbf u_0 - \mathbf u_{\text{flown}}\rvert \le \dot u \times 0.5$ s | Hard (`StartSlackS`) |
| Rates | $\lvert \mathbf u_{k+1} - \mathbf u_k\rvert \le \dot u\,\sigma/(N-1)$ | Hard, linear in $\sigma$. $\dot u$ is ROLL RATE (at least 1 deg/s) for all three angles |
| Approach cone | Below 12 km, horizontal offset $\le \tan 15°\,z + 300$ m, on an octagon | Slack per node |
| Lateral cone | $\lvert c_k\rvert \le \tan 60°\,d_k + 25$ m, $d_k$ the distance still to go along the bearing | Slack on the worst node |
| Trust region | $L_\infty$: 1 km, 10 m/s, 5 deg, 10 s, times $\eta$ | Hard |

**Limits widened at the start.** Each angle's limit at node $k$ is the larger of the setting and the attitude flown less what the angle rate can close by then, so a take-over at the entry's attitude is feasible. Since #325 the time used is node $k$'s time at the trust region's lower bound on $\sigma$, recomputed each iteration, so a shrinking flight time stays feasible.

**Authority reserve.** Beyond the first 20 s (`MarginAfterS`) the pitch and bank limits are cut to 75 % (`LimitMargin`). The next plan, a second later, has the whole limit again, so a plan never spends the authority the craft needs to correct with.

**The bank the flaps can hold.** Before each solve, `LimitBellyflopBankToRollTrim` lowers the bank limit to the largest bank, either way, at which the flaps hold the body's rolling moment with 60 % of their reach (`RollTrimShare`). It reads the belly-first body and flap moment tables at the Mach and pitch flown now, in 2.5 deg steps, and never goes under 5 deg. A craft with no flap tables keeps BELLYFLOP BANK.

**The cones.** The approach cone keeps the last of the fall steep, so the miss comes out while there is speed to take it out with. The lateral cone lies on its side, its apex at the aim, opening back along the bearing. It stops a plan passing the aim and swinging back: past the aim the distance to go is negative and nothing is inside.

### Objective

$$
J = \frac{w_m}{1000}\big(\lvert x_N\rvert + \lvert y_N\rvert\big)
+ \frac{w_x}{1000\,N}\sum_{k\ge1}\lvert c_k\rvert
+ \frac{1}{N}\sum_k\big(w_{\alpha,k}\alpha_k^2 + w_{\beta,k}\beta_k^2 + w_{\psi^2,k}\psi_k^2\big)
+ \frac{1}{N}\sum_k w_{\psi,k}\lvert\psi_k\rvert
+ w_r\sum_k\lVert\Delta\mathbf u_k\rVert^2
+ \text{cone and penalty terms},
$$

angles in degrees, distances in metres, $c_k$ the crossrange off the vertical plane through the site along the bearing.

| Weight | Value | Config field | Why |
|---|---|---|---|
| Miss $w_m$ | 1000 per km (L1 in $x$ and $y$) | `MissWeight` | 1 per metre: a degree of yaw held all the way down is worth 2 m of miss |
| Crossrange $w_x$ | 300 per km of its mean | `CrossWeight` | Priced at every node, so crossrange comes out early instead of being spread over the whole descent |
| Pitch $w_\alpha$ | 0.01 per deg² (mean) | `EffortWeight` | Fly as near belly first as the range allows |
| Bank $w_\beta$ | 0.1 per deg² (mean) | `BankWeight` (NaN prices it as pitch) | **Bank to turn**: bank is sideslip held against the flaps' roll trim, so it is priced ten times pitch and the plan turns its pitch's lift with yaw instead, which needs no trim |
| Yaw $w_\psi$ | 0.05 per deg (mean, L1) | `YawWeight` | Low, so yaw is how the plan steers (it was 2, far above bank's, and yaw turned only once bank was pinned) |
| Yaw, square $w_{\psi^2}$ | 0.01 per deg² (mean) | `YawQuadWeight` | Keeps a free yaw from wandering |
| Level by the end | on pitch $+0.5\,f_k$, on bank and yaw $+1\,f_k$ per deg² | `EndLevelS` 30 s, `EndLevelPitchWeight` 0.5, `EndLevelWeight` 1 | **Level by the end**: $f_k = \operatorname{clamp}\big((30\,\text{s} - t_{\text{go},k})/15\,\text{s},\ 0,\ 1\big)$ rises from nothing 30 s before the plan's end to all of it 15 s before, so the glide is done early and the craft falls the last of the way belly flat and vertical onto its aim, wings level for the flip |
| Yaw while gliding | 0.1 of $w_\psi$ from 25 deg of pitch | `YawGlideScale`, `YawGlideFromDeg` | Gliding, yaw turns the lift as the entry's bank does. The share falls linearly from 1 at 20 deg |
| Rate $w_r$ | 0.01 per deg² | `RateWeight` | Smooth angles between nodes |
| Approach cone | 500 per km, capped | `ConeWeight`, `ConeShareOfPenalty` | Per node, capped so that all the cone's nodes together cost at most 0.5 of one node's position defect |
| Lateral cone | 5000 per km of the worst excess | `LateralConeWeight` | Half one node's position defect, so no defect buys its way into the cone |
| Dynamics, altitude | $10^4$ per scaled unit | `PenaltyWeight` | 10 per metre, far above the miss: a miss is never closed by bending the dynamics |
| Proximal | $10^{-4}$ | `ProximalWeight` | A unique step |

### Seed, iterations and the gate

The cold seed eases each angle to zero at the angle rate and falls to the ground; $\sigma$ is the time that takes. A warm seed takes the plan before at each node's time, with its first node at the attitude flown and $\sigma$ its time left. The trust region and ratio test are the entry planner's ($\eta$ 2 cold, 1 warm, up to 20; thresholds 0.1, 0.25, 0.7). The published states are the shot trajectory of the final controls, so a plan says what flying it gives. A plan is converged at a scaled defect of at most 0.05 (`GateDefect`) and a miss of at most 500 m (`GateMissM`).

## Bellyflop: the flying side

**Force scale.** The entry's density scale compares drag with the entry's own model, and read 1.9 belly first. Bellyflop measures its own: the game's aero acceleration over its model's at the angles flown, clamped to 0.5-2, taken as it is the first time and low-passed over 5 s after that. Until it has one it uses the entry's.

**Take-over.** Bellyflop takes the craft at its first converged plan once Mach is at or below BELLYFLOP FROM MACH, whether the entry is still flying or holding at the hand-over. The entry's tracker is reset and its bank set to zero.

**Refusals.** A plan whose defects closed seeds the next solve even when it was refused for its miss (`BellyflopState.Warm`). After three refusals in a row the next solve starts cold. While bellyflop flies, a refused plan is flown if it lands nearer than the plan being flown would from the same state (`BellyflopPlanner.ShotMiss`). Flying a stale converged plan through a run of refusals once took the craft 1.4 km off.

**Roll first.** Under bellyflop the attitude controller weighs the roll's shortfall 4 against pitch and yaw's 1 in the flaps' and RCS's allocation (`BellyflopRollPriority`, `AttitudeController.Settings.RollPriority`; a rocket's 0.3 everywhere else). Belly first the roll is the sideslip, and the air's rolling moment grows with it: with the flaps saturated and roll last, the allocation gave the roll away (once 150 kN m the wrong way against 450 asked), and the craft departed in bank.

**Experiments.** `AFC_XBF_<name>` sets any `BellyflopConfig` property for every plan bellyflop builds (`BellyflopExperiment`, logged at the take-over), and `AFC_XBF_RollPriority`, `AFC_XBF_TurnAwayDeg`, `AFC_XBF_HandoverBeforeIgnitionS` and `AFC_XBF_HandoverBeforeImpactS` the flight-side numbers. Nothing changes when they are unset.

**The command.** `BellyflopAttitude` passes the plan's angles through a critically damped second-order filter, $\omega = 2$ rad/s (`BellyflopCommandOmega`), stepped semi-implicitly in 0.02 s steps. It starts from the angles flown at the take-over, at rest, and reads the plan $2/\omega = 1$ s ahead, the filter's lag on a ramp. Yaw wraps across $\pm180$ deg. The command, its rate and its acceleration are all continuous, so a new plan each second does not step the target. The filtered angles become a nose and a belly for AFC's attitude controller, as the entry's do.

## Hand-over to touchdown

With bellyflop's touchdown hand-over on (`OrbitalEntryBellyflopTouchdown`, on by default), `StepBellyflopTouchdown` starts once bellyflop's plan is TOUCHDOWN FROM (45 s) from the ground:

1. **Landing group.** The engines chosen on LAND are switched on first (`PrepareSelectedLandingEngines`). Bellyflop flies on while they switch.
2. **Can it stop?** Refused while the active engines' full thrust at the local pressure gives less than 0.5 m/s² over gravity.
3. **The flaps' tables.** Touchdown does not plan until the shared sweep's control-surface tables are in (`TouchdownWaitsForSurfaces`, up to 90 s after the sweep was taken).
4. **Standby.** Touchdown solves on standby, with a coast to an ignition it chooses (`SixDofCoastPending`). Its cold seed follows bellyflop's plan (`BellyflopSeedPath`): the coast along bellyflop's states with its nose as the wanted body axis, then a straight braking burn. The burn seed is the stop at 70 % of thrust less gravity (at least 30 % of the spare), at least 2 s, plus 6 s for the flip.
5. **Due.** The hand-over is due at the earliest of: 1 s before touchdown's plan first turns more than 15 deg from the attitude flown (`SixDofTurnAwayDue`, `TurnAwayDeg`; it was 5, which handed a Starship over 31-36 s before ignition); 5 s before touchdown's ignition (`ThreeDofTakeoverBeforeIgnitionS`); 12 s before bellyflop's ground (`ThreeDofTakeoverBeforeImpactS`). The standby plan may be up to 2.5 s old (`BellyflopHandoverPlanAgeS`).
6. **Take.** `CheckPendingBellyflopTouchdownHandoff` engages touchdown guidance on the next step, ahead of the claim. If touchdown cannot take the craft, or the engines have changed, bellyflop keeps it and the standby waits 5 s before it starts again.

The engine stays off until touchdown's plan lights it.

## Aero data

Everything above flies on one sweep of the craft, taken by `KsaAeroSweep` on its own thread from the game's own aero forces.

| Tables | Grid | Used by |
|---|---|---|
| Belly-first trim rows (`Result.Belly`), one set for each of the four belly sides | 22 angles of attack from nose-first, 0-180 deg, dense from 40 to 90 | The entry's schedule: $C_D$, $C_L$, $C_m$ against Mach |
| Belly-first body and surface tables (`BellyBodyAero`, `SurfaceTable.BellyForce`), on the chosen side | $\alpha$ -20 to 80 deg (15 points), $\beta$ ±45 deg (13 points); Mach 8 at most | Bellyflop's model, the roll-trim bank limit, the entry's flap re-trim, the attitude controller |
| Whole sphere (`SphereBodyAero`) | Longitude every 15 deg, latitude every 15 deg: 266 directions a Mach row. Surfaces every 30 deg: 62 | Touchdown's flip, and the controller past the other tables |
| Flip band (`BandBodyAero`) | Longitude -15 to 100 deg every 5 deg, latitude ±45 deg every 7.5 deg: 24 × 13 directions, body and each surface | Touchdown's flip between the engine-first and belly-first tables |

`TryPickTables` picks, for the air as it is now, belly first where the air is inside those tables, else nose first, else engine first, else the flip band, else the whole sphere.

**One sweep for every mode.** `TryStart` always sweeps every table: tail, nose, belly trim rows and the belly side BELLY AXIS names. The aim, the entry and touchdown then share one result. Asked for piecemeal, a Starship was once swept three times in a minute.

**In-memory cache.** `KsaAeroSweepCache` keeps the last 8 sweeps, keyed by the aero mesh hash (taken with every surface at rest, so a flap's angle does not change it), a fingerprint of the control surfaces and their limits, the body (by identity, since #322) and, for a booster, its name. The same craft with its centre of mass within 5 cm (`CacheComToleranceM`) is handed the same result. Any other is handed a view (`ViewOf`): its surface tables re-linked to the craft's parts by `PartKey`, and every moment moved to the centre of mass the craft has now:

$$
\mathbf C_{M,1} = \mathbf C_{M,0} + \frac{(\mathbf c_0 - \mathbf c_1)\times\mathbf C_F}{L},
$$

node by node, for the body, nose, belly, sphere and band tables and each surface's per-deflection changes, with $L$ the reference length. A miss sweeps what was asked for plus what the newest sweep of that geometry held. The cache is cleared on a star-system swap or mod unload (#322).

**Disk cache.** `KsaAeroSweepStore` saves each finished sweep as gzipped JSON under `afc-aero-tables` in KSA's documents folder, one file per craft, mesh hash, mesh fingerprint, surface fingerprint, body and belly side. The atmosphere is not stored; it is sampled again from the body. A file of another `StoreVersion` (now 2), or one that cannot be read, is deleted and the craft swept again.

**Stale sweeps.** The entry takes the craft's own sweep only if it has the sphere and belly tables on the right side, for the same body, and is not stale. A sweep is stale when the live mesh hash differs from the sweep's, or, with no live hash, when the craft's bounding box has changed (`SweepStale`).

**Waiting.** A sweep hands over its body tables first and its control surfaces after. Until the surfaces are in, the attitude flies without them, and touchdown waits. While a sweep runs, the amber band under the altitude counter says GENERATING AERO DATA: READY IN ~... (ESTIMATE), or LOADING AERO DATA for a cached or saved sweep (`AeroDataBandWindow`). TUNE > DIAG > AERO SWEEPS lists the craft's sweeps: stage, time, surfaces done and missing.

## Diagnostics and checks

**Logs.** With `EntryLogging` on (the default), `OrbitalEntryLog` writes two files to `navbox-logs`:

* `<run>-orbital-entry.csv`: a row every 0.5 s. Where the craft is against the site and the corridor, what the tracker asked and why, what the attitude flew (demand, air, surface and RCS torques, fins), bellyflop's angles, miss, fall and force scale, and the plan it flew on.
* `<run>-orbital-entry-plans.csv`: every published entry plan, node by node.

**ENTRY TRAJ** (`Ui/Gauges/EntryTrajWindow.cs`) is the ORBITAL ENTRY picture and, while the entry flies, a display-only window: range to go against speed, paged by speed band on axes fixed from the first plan, between the HIGH and LOW ENERGY lines, with the bug, its trailers and the Shuttle's guidance box for the drag error.

The game log has each refused solve, the schedule, reach changes, LEAST HEATING, the bellyflop take-over, the roll-trim bank limit and the touchdown hand-over. **AERO CHECK**: once a second while touchdown runs on a craft with whole-sphere tables, touchdown's model force and moment (body and flaps apart) against the game's.

**Offline checks** are flags of the Scvx console, `dotnet run --project tests/AdvancedFlightComputer.Guidance.Tests/Scvx -- <flag>`:

| Flag | What it checks |
|---|---|
| `--orbital-entry` (CI) | The entry planner and tracker in closed loop on synthetic vehicles, the heating floor, the equilibrium glide's bounds and the corridor's first node |
| `--orbital-entry-gains` | A sweep of the tracker's gains with measured drag noise |
| `--bellyflop` | Bellyflop from hand-overs as flown, including a steep one with a shrinking flight time |

**Harness.** `afc-guidance-flopper-entry` (`GuidanceFlopperEntryTest.cs`) is opt-in and needs the KSAero content mod. It sets an entry state from `AFC_FLOPPER_*` variables, engages ORBITAL ENTRY and checks the engage, bellyflop's take-over, the miss and the heating peak.

## Main files

| Path | Role |
|---|---|
| `Features/Guidance/Scvx/OrbitalEntry/EntryPlanner.cs`, `EntryDynamics.cs`, `EntryModel.cs`, `EntryConfig.cs`, `EntryPlan.cs` | Entry planner: energy-domain problem, corridor, coast prediction |
| `Features/Guidance/Scvx/OrbitalEntry/EntryTracker.cs`, `EntryTrim.cs` | Drag tracker and bank reversals; angle-of-attack schedule |
| `Features/Guidance/Scvx/Bellyflop/BellyflopPlanner.cs`, `BellyflopModel.cs`, `BellyflopConfig.cs`, `BellyflopPlan.cs` | Bellyflop planner and model |
| `Features/Guidance/Modes/OrbitalEntry.cs`, `OrbitalEntryState.cs` | The mode: phases, re-plans, tracker, attitude, hand-overs after the entry |
| `Features/Guidance/Modes/Bellyflop.cs` | Bellyflop's range target, take-over, command filter, roll-trim bank, hand-over to touchdown |
| `Features/Guidance/Modes/OrbitalEntryAim.cs`, `DeorbitPlanner.cs`, `StockDeorbit.cs`, `Deorbit.cs` | Aimed deorbit burn, Lambert option, hand-over to the entry at burn end |
| `Features/Guidance/Adapters/Scvx/Bellyflop/KsaBellyflopSetup.cs` | Bellyflop's model from the craft and the sweep; angles flown |
| `Features/Guidance/Adapters/KsaAeroSweep.cs`, `KsaAeroSweepCache.cs`, `KsaAeroSweepStore.cs`, `KsaAeroSweepStatus.cs` | Sweep, table pick, in-memory and disk cache, progress |
| `Features/Guidance/Adapters/OrbitalEntryLog.cs` | The two CSV logs |
| `Features/Guidance/Modes/SixDofStandby.cs`, `SixDof.cs` | Touchdown's standby, turn-away rule, flap-table wait, AERO CHECK |
| `Features/Ui/Logic/GuidanceSettingSpecs.cs` | Setting labels, defaults and tiers |
| `tests/AdvancedFlightComputer.Guidance.Tests/Scvx/OrbitalEntryCheck.cs`, `BellyflopCheck.cs` | Offline checks |

Paths in the first column are under `AdvancedFlightComputer/` unless they start with `tests/`.
