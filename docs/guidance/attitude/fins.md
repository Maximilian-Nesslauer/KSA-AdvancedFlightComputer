# Fin mixing

`KsaAeroTorqueControl` commands a craft's aerodynamic control surfaces, called fins here, for a torque, and says how much torque they can give.
It does for the fins what [RCS mixing](rcs.md) does for the thrusters, and it is usable by any mode.
The stock flight computer allocates torque to gimbals and RCS only, so this is the only allocator AFC has for fins.

A fin's torque is the air's, so it changes with the flight.
The mixer therefore needs three things: a list of the fins (the control-surface interface), what each fin does at each deflection and flight condition (the fin tables), and a programme that picks deflections from those curves.
The shared problem is set out in [The common torque-allocation problem](index.md#the-common-torque-allocation-problem).

## The control-surface interface

AFC learns of fins from whoever provides them, through `ControlSurfaces.Register`.
A parts mod registers by reflection, with KSA and .NET types only, so it needs no reference to AFC's assembly and the load order does not matter.
KSAero registers its all-moving fins this way today; when KSA has control surfaces of its own, AFC can register a provider for them and nothing that steers with them changes.

A provider gives four functions, all called on the main thread except where noted:

| Function | What it does |
| --- | --- |
| `collect` | the fins on a vehicle, each as (part, name, min deg, max deg, rate deg/s) |
| `deflection` | the deflection a fin has reached, degrees |
| `command` | ask a fin for a deflection; the provider moves it at its own rate |
| `poser` | for the aero sweep: a binder that finds the fin in a private copy of the aero mesh and returns an action that turns it to a deflection there. The binder and the action run on the sweep's thread. |

Each fin becomes a `ControlSurface`: its part, name, range and rate.
A fin whose range is empty is skipped, and one that reports no rate is treated as reaching any deflection at once.

## The fin tables

The aero sweep (`KsaAeroSweep`) measures the craft's aerodynamics by calling the game's own force function on a copy of the craft, with the airspeed set per sample.
It runs on its own background thread (`afc-aero-sweep`, below normal priority), on a private aero mesh gathered from the craft's parts, so it holds up neither the main thread nor the shared thread pool.

It works in two passes.
The body table comes first and is handed over as soon as it is fitted, so a boostback or a SOLVE waiting on the craft's aero goes on with it.
The fins follow on the same private mesh and are attached to the same result when they are all done.
On one test craft the body took 2 s and four fins 5.2 s.

For each fin, alone, the sweep:

1. turns it to each deflection of its grid (`KsaAeroSweep.SurfaceDeflections`): rest, every 15 deg outward from rest (`SurfaceStepDeg`), and both ends of its range. A +/-45 deg fin is swept at -45, -30, -15, 0, 15, 30 and 45 deg;
2. flies the body table's grid again at each deflection: the same Mach rows (0 to 5 in the default grid, then 6, 7 and 8) and the same $\alpha$ and $\beta$ breakpoints (every degree to +/-5 deg, then +/-8, +/-12, +/-16 and +/-20 deg);
3. records, per node, the **change** the fin makes to the body's force coefficients (per $qS$) and moment coefficients (per $qSL$, about the centre of mass the sweep saw);
4. turns it back to rest, whatever happens, so the next fin is swept against the craft as built.

Each deflection's table is a cubic B-spline over (Mach, $\alpha$, $\beta$), like the body table (`BodyAeroTable`).
A fin is swept with the body angles because it is panels of the craft's mesh like any other, shielded by the body and the other fins as the flow turns: what it does depends on $\alpha$ and $\beta$, not only on its own angle.
The fins are swept one at a time and added in flight; the interference between two deflected fins is left out.

## From tables to torque curves

Each step, `KsaAeroTorqueControl.Measure` reads every fin's tables at the condition flown now and builds one curve per fin.
At each swept deflection $\delta_k$:

$$
\mathbf t_i(\delta_k) = \frac{q S L\,\Delta\mathbf C_{M,i}(\delta_k) + (\mathbf r_\text{sweep} - \mathbf r_\text{now}) \times q S\,\Delta\mathbf C_{F,i}(\delta_k)}{1 + k\,q},
\qquad
\mathbf f_i(\delta_k) = q S\,\Delta\mathbf C_{F,i}(\delta_k)
$$

with every coefficient read at the Mach number, $\alpha$ and $\beta$ flown.
The terms are those of the air-moment feed-forward on the [overview](index.md#air-moment-feed-forward): the moment moved to the present centre of mass, and divided by the game's moment limiter $1 + kq$.
Between swept deflections the curve is straight; past its ends it is held.
Every curve is zero at rest.

### Authority limits

The fins report **no authority**, rather than a guess, when

- the dynamic pressure is under 50 Pa (`MinQPa`),
- the airspeed is under 1 m/s,
- the craft flies nose first, or
- $|\alpha|$ or $|\beta|$ is over 20 deg (`KsaAeroSweep.BodyAngleMaxDeg`), the edge of the tables.

Then every fin is sent home and the whole demand goes to the RCS.

## The programme

### Segments

The fins' curves are not straight, so the programme works on pieces of them.
Each fin's travel from where it is now is split into **segments**, outward each way, at its swept deflections, at rest, and at its reach.
Segment $j$ has a width $w_j$ in degrees and a slope $\mathbf b_j$, its torque per degree.
The command is how far along each segment the fin travels, $0 \le u_j \le w_j$, and the fin's deflection is where its segments add up to.

| Common term | Fins |
| --- | --- |
| commands $\mathbf u$ | travel along each segment of each fin's curve, degrees |
| effectiveness $B$ | each segment's slope $\mathbf b_j$, N m per degree |
| bounds | $0 \le u_j \le w_j$, inside what the fin reaches by the next step |
| command cost $\mathbf c$ | a price per degree away from rest, and a price per degree moved |
| shortfall price $p$ | 1 per N m at priority 1 |
| priorities $w_a$ | $(1, 1, 0.3)$ from the attitude controller |
| side force | $\sum_i \mathbf f_i(\delta_i)$, left free and reported |
| backup columns | the RCS thrusters (below) |

$$
\begin{aligned}
\min\quad & \sum_j \big(\pm c_\delta \pm c_\Delta\big)\,u_j + \sum_k c_k\,d_k + \sum_a w_a\,(e^{+}_a + e^{-}_a) \\
\text{s.t.}\quad & \boldsymbol\tau_0 + \sum_j \mathbf b_j\,u_j + \sum_k \mathbf t_k\,d_k + \mathbf e^{+} - \mathbf e^{-} = \boldsymbol\tau \\
& 0 \le u_j \le w_j,\quad 0 \le d_k \le 1,\quad \mathbf e^{\pm} \ge 0
\end{aligned}
$$

$\boldsymbol\tau_0$ is what the fins give where they are now.
The $d_k$ are the RCS backup columns, absent when there is no backup.

- $c_\delta$, the **price of a degree away from rest**, is $10^{-3}$ of the best segment's slope (`DegreePriceShare`). It is earned back moving towards rest. It makes the answer unique and keeps fins home that the demand does not need.
- $c_\Delta$, the **price of a degree moved**, is half that (`ChangePriceShare`), and earned back moving towards where the fin is now.

A degree that buys a newton metre of torque costs about a thousandth of a newton metre of shortfall, so the fins give everything they can before any demand goes unmet.

### Rate reach

Each fin is asked only for a deflection it reaches by the next step:

$$
\delta_i \in \big[\max(\delta_{\text{min},i},\ \delta_{\text{now},i} - r_i\,\Delta t),\ \min(\delta_{\text{max},i},\ \delta_{\text{now},i} + r_i\,\Delta t)\big]
$$

$r_i$ is the fin's rate and $\Delta t$ the time to the next command (the attitude controller's step, or 0.05 s on its first).
So what the mixer reports the fins give is what they do give at the next step, and the RCS, which is fired for the rest, is told the truth.
A fin asked for a deflection it cannot reach in time would leave a gap that nothing filled.

### Sticky, and going home

With the price of a degree moved, a fin stays where it is unless moving buys torque.
A demand that four fins can meet in many ways does not swap between them from one step to the next.
A fin the demand does not need goes home at its rate, since a degree towards rest is worth more than a degree moved costs.

### Straightened where they steepen

A real fin must pass through the inside of its travel before the outside.
A linear programme does not know that: it would happily take a steep outer segment alone.
So each direction's segments are **straightened where they steepen** (`AddDirection`): a segment steeper than the one inside it is merged with it into one straight piece, and again with the next one in, until the slopes only get shallower outward.
The programme then fills them inside first.

This is exact for the usual curve, where a fin's effectiveness falls off as it stalls.
Where a curve steepens outward, the straight piece is a chord that promises a little more than the curve gives on its way out.

### Trust-region refinement

The first pass is the programme above.
Where the curves bend the wrong way, its answer can count on torque they do not give; from a deflection on a bend it can even fill a steep piece one way and a shallow one back, moving nowhere for torque it does not get.
On one craft's +/-45 deg fins, whose drag grows faster outward, it gave 48 of 51 kN m of pitch with 12 kN m of roll and 14 of yaw nobody asked for.

So `AeroTorqueAllocator.Allocate` refines its own answer.
Each further pass gives each fin **one straight slope**, the secant of its curve across a window of $\pm W$ around the best answer so far, the same slope both ways so there is nothing to play off:

1. Solve the programme on those secants.
2. Read the torque the new deflections really give off the curves, and the true cost in the programme's own prices.
3. If the cost fell, keep the answer and double the window, up to 15 deg. If not, halve it.
4. Stop when the window is under 0.05 deg, after 24 passes, or once the torque the pass planned is within $1 + 0.005\,|\boldsymbol\tau|$ N m of what the curves give.

The window starts at 10 deg.
A window that could only shrink settled 8 % short on scaled-up fins, where one large coordinated move of several fins was needed, so it can grow back.
If the first start does not settle, a second one runs from where the fins are now, and the better answer is kept.

<!-- viz: fin-mixing 620 -->
*Drag the demand and step time forward: watch each fin travel at its rate, the refinement window shrink and grow, and the RCS take what the fins cannot reach yet.*

## Fins and RCS in one programme

The attitude controller passes the RCS thrusters into the fins' programme as **backup columns** (`KsaAeroTorqueControl.Command`'s `backup`).
Column $k$ is thruster $k$'s full-duty torque $\mathbf t_k$, bounded $0 \le d_k \le 1$, at a cost

$$
c_k = 0.02\,\frac{\dot m_k}{\min_l\,(\dot m_l/|\mathbf t_l|)}
$$

so the best thruster costs 0.02 per newton metre (`BackupPricePerNm`) and the others more, by their propellant.
The prices then line up as: a fin about $10^{-3}$ per newton metre, a thruster 0.02 or more, and a shortfall 1 (0.3 in roll).

- The fins take every axis they can, whichever it is; the RCS takes the axes the fins cannot reach or find dear.
- A demand beyond the fins goes to the RCS before any of it goes unmet.

The programme's duties are not flown.
The controller reads what the fins will give and fires the RCS through [RCS mixing](rcs.md) for the rest.

### Slow demand to the fins, fast to the RCS

Where RCS backs the fins, the attitude controller can lag the fins behind the programme's answer (`AttitudeController.Settings.SurfaceSmoothS`, passed to `KsaAeroTorqueControl.Command` as `smoothS`).
Orbital entry uses it, with $T = 0.5/\omega_n = 0.5$ s at the controller's $\omega_n = 1$ rad/s; the other modes leave it at 0.
Without it, the RCS's flutter about a held attitude swung the flaps 10 deg every half second for next to no torque (2026-11-21).

Each step, in order:

1. The programme above gives each fin its best deflection $\delta^*_i$, inside its rate reach.
2. Each fin moves only part of the way there from its last command $\delta_{\text{prev},i}$ (its measured deflection on the first step), and stays inside its reach (below).
3. The torque the fins give is read off their curves at those deflections, $\boldsymbol\tau_{\text{fins}} = \sum_i \boldsymbol\tau_i(\delta_{\text{cmd},i})$ (`AeroTorqueAllocator.Evaluate`).
4. The RCS is fired for the rest, $\boldsymbol\tau_{\text{rcs}} = \boldsymbol\tau - \boldsymbol\tau_{\text{gimbals}} - \boldsymbol\tau_{\text{fins}}$.

Step 2 in full:

$$
\begin{aligned}
k &= \frac{\Delta t}{T + \Delta t} \\
\delta_{\text{cmd},i} &= \delta_{\text{prev},i} + k\,\big(\delta^*_i - \delta_{\text{prev},i}\big) \\
\delta_{\text{cmd},i} &\leftarrow \operatorname{clamp}\big(\delta_{\text{cmd},i},\ \delta_{\text{now},i} - r_i\,\Delta t,\ \delta_{\text{now},i} + r_i\,\Delta t\big)
\end{aligned}
$$

At 60 frames a second, $\Delta t \approx 0.017$ s and $k \approx 0.032$: each step a fin closes about 3 % of the gap to its target.

**It is a first-order lag.** Step 2 is the backward-Euler step of $T\,\dot\delta = \delta^* - \delta$:

$$
\frac{\delta_k - \delta_{k-1}}{\Delta t} = \frac{\delta^*_k - \delta_k}{T}
\quad\Longrightarrow\quad
\delta_k = \delta_{k-1} + \frac{\Delta t}{T + \Delta t}\,\big(\delta^*_k - \delta_{k-1}\big)
$$

**It prices the fins' rate, not their deflection.** Step 2 is also the deflection that minimises, fin by fin,

$$
J(\delta) = (\delta - \delta^*)^2 + \frac{T}{\Delta t}\,(\delta - \delta_{\text{prev}})^2
= (\delta - \delta^*)^2 + T\,\Delta t\,\dot\delta^{\,2},
\qquad \dot\delta = \frac{\delta - \delta_{\text{prev}}}{\Delta t}
$$

since $J'(\delta) = 0$ gives exactly the update above.
That is dynamic control allocation (Harkegard) with diagonal weights: a quadratic price on the fin's rate, of weight $T\,\Delta t$ against the distance from the programme's answer, and none on the deflection itself, so the fins still end where the programme wants them.
The programme's own price of a degree moved, $c_\Delta$, stays: it is linear and small, and stops the fins swapping work; the lag is the strong price on top.

**It splits the demand by frequency.** For demand varying at $\omega$, the fins pass a low-pass share and the RCS the complementary high-pass share:

$$
H_{\text{fins}}(j\omega) = \frac{1}{1 + j\omega T}, \qquad
H_{\text{rcs}}(j\omega) = \frac{j\omega T}{1 + j\omega T}, \qquad
H_{\text{fins}} + H_{\text{rcs}} = 1
$$

| Demand varying at | $\lvert H_{\text{fins}}\rvert$ | $\lvert H_{\text{rcs}}\rvert$ |
| --- | --- | --- |
| 0.5 rad/s, a slow trim change | 0.97 | 0.24 |
| 1 rad/s, the attitude loop's bandwidth | 0.89 | 0.45 |
| 2 rad/s | 0.71 | 0.71 |
| 6 rad/s, about 1 Hz flicker | 0.32 | 0.95 |
| 12 rad/s, about 2 Hz | 0.16 | 0.99 |

The magnitudes do not add to 1; the complex shares do.
Steady demand ends on the fins, and the fast flicker almost all on the RCS.

Where it is approximate:

- The lag is on deflection, so the split is exact only where a fin's torque is straight in its deflection; step 3 reads the true torque, so the RCS is always told the truth.
- $\delta^*$ already counts on the RCS, which shares the programme as backup columns; the lag only delays the fins' part of that answer.
- With no RCS on the craft there is nothing to take the fast part, so there is no lag: the fins follow the programme directly.
- A fast, large demand that the RCS cannot cover goes partly unmet while the fins travel. The attitude controller's authority-aware capture (orbital entry) keeps its demands to what the craft can stop from, which keeps this rare.

## What the fins can reach

`AeroTorqueAllocator.Characterise(axis, sign)` is the common characterise step, run from rest over each fin's whole range on the straightened segments.
Because those segments are chords where the curves steepen, the programme's maximum can promise more than the fins give.
So the reach is checked: the mixer allocates that maximum and keeps what the curves really give about the axis.
It then allocates half the reach and reports the side force per newton metre.
`KsaAeroTorqueControl.TryCharacterise` gives all six directions at the condition flown now.

## Touchdown guidance

Touchdown guidance does not use this programme.
Its plan carries the fins (`BodyAero6Dof.Fins`, built from the same tables), keeps back a share of each fin's range and rate for the attitude feedback, and the flight commands the plan's own deflections fin by fin (`GuidanceWindow.FlyPlanFins`).
The fins go home when a plan has none.

## What you see in flight

- **The fins move smoothly and settle.** They would chatter if they chased a jittering demand or swapped work between them every step; the filtered rate error on the [overview](index.md#rate-error-and-its-filter), the price of a degree moved, and asking each fin only for what it reaches stop that.
- **The RCS fires when the fins cannot reach.** That happens in thin air, while the fins are still travelling to a new deflection, when they are at their stops, and when the craft is more than 20 deg off engine first.
- **Idle fins drift home** at their own rate when the demand does not need them.
- **Fins take roll too**, by deflecting against each other, so the RCS rolls the craft only when the fins have no roll left.

## Code map

| File | Main type or method |
| --- | --- |
| [ControlSurfaces.cs](../../../AdvancedFlightComputer/Features/AeroControl/ControlSurfaces.cs) | `ControlSurfaces.Register`, `Collect`; `ControlSurface` |
| [AeroTorqueAllocator.cs](../../../AdvancedFlightComputer/Features/AeroControl/AeroTorqueAllocator.cs) | `AeroTorqueAllocator.Allocate`, `SolvePass`, `AddDirection`, `SecantsWithin`, `Characterise` |
| [KsaAeroTorqueControl.cs](../../../AdvancedFlightComputer/Features/AeroControl/KsaAeroTorqueControl.cs) | `KsaAeroTorqueControl.Measure`, `Build`, `Command`, `TryCharacterise`, `Release` |
| [KsaAeroSweep.cs](../../../AdvancedFlightComputer/Features/Guidance/Adapters/KsaAeroSweep.cs) | `KsaAeroSweep.SurfaceTable`, `SurfaceDeflections`, `Job` |
| [BodyAeroTable.cs](../../../AdvancedFlightComputer/Features/Guidance/Numerics/Flight/BodyAeroTable.cs) | `BodyAeroTable.Coefficients` |
| [RcsLpSolver.cs](../../../AdvancedFlightComputer/Features/RcsTranslation/RcsLpSolver.cs) | `RcsLpSolver.Solve` |
| [AttitudeController.cs](../../../AdvancedFlightComputer/Features/Guidance/Control/AttitudeController.cs) | `AttitudeController.Step`, which passes the RCS in as backup |
| [SixDof.cs](../../../AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs) | `GuidanceWindow.FlyPlanFins` |
