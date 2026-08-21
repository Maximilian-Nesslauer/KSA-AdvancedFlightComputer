# 6-DOF SCvx — from the algorithm to a thing that flies

This directory is the **bridge** between a successive-convexification solver and a
rocket in Kitten Space Agency. The maths is documented elsewhere; what follows is
everything that had to be true *besides* the maths, most of it learned by flying it
and finding out.

| Layer | Where | Job |
|---|---|---|
| Reference maths | [`scvx/README.md`](../../scvx/README.md), `scvx/python_ref/` | The algorithm, in JAX, as a spec |
| Solver | `scvx/Scvx.Core/` | SCvx loop, subproblem assembly, SCS bindings |
| **Bridge (here)** | `ksamod/Scvx/` | Frames, vehicle measurement, MPC, actuator commands |
| Host | `ksamod/PoweredGuidance6Dof.cs` | UI, stepping, telemetry, hand-off |

**Files here**

- **`KsaFrameBridge.cs`** — inertial frame, body-axis and quaternion conversions.
- **`Ksa6DofSetup.cs`** — builds the solver config by *measuring* the live vehicle.
- **`Ksa6DofGuidance.cs`** — the MPC loop: plan, re-solve, read commands back.

---

## Contents

- [1. What the solver actually solves](#1-what-the-solver-actually-solves)
- [2. Scaling is load-bearing](#2-scaling-is-load-bearing)
- [3. Node 0 is the vehicle — a whole family of bugs](#3-node-0-is-the-vehicle--a-whole-family-of-bugs)
- [4. Reading the answer back out](#4-reading-the-answer-back-out)
- [5. The actuator boundary](#5-the-actuator-boundary)
- [6. Making it real-time](#6-making-it-real-time)
- [7. Model error: what MPC cannot fix](#7-model-error-what-mpc-cannot-fix)
- [8. Host integration](#8-host-integration)
- [9. Diagnostics](#9-diagnostics)
- [10. Still open](#10-still-open)

---

## 1. What the solver actually solves

SCvx linearises the nonlinear dynamics about a reference trajectory, solves a convex
subproblem, accepts or rejects the step on a ratio test, and repeats. Three things
about the *subproblem* shape drive most of what follows.

### The dynamics are a SOFT constraint

Collocation is imposed as

```
X[k+1] = X[k] + 0.5*dtau*sigma*(g_k + g_k+1) + Wv[k]
```

where `Wv` is **virtual control** — a slack variable, penalised by `RhoVc`, *never
constrained to zero*. Until SCvx converges, `Wv != 0` and **the trajectory does not
obey the dynamics**: the state teleports between nodes on fictitious forces. No
thrust can reproduce it.

> [!WARNING]
> A plan that has not converged is not merely *suboptimal*, it is **unflyable**. The
> vehicle chases a trajectory no force can produce, saturates thrust, and falls
> further behind every cycle. Gate on the **defect**, not on the solver status.

`Ksa6DofGuidance.Finish()` refuses any plan whose defect exceeds `MaxDefectM` and
keeps the previous one, saying why.

### The problem is a cone program, in SCS's native form

Equalities, the positive orthant and the SOC blocks all stack into **one** matrix.
Row order is fixed: zero cone, then non-negative, then SOC blocks in `q` order.
`Scvx6DofSubproblemScs` freezes the sparsity pattern on the first assemble and only
refills values afterwards.

### Free final time makes it bilinear

`sigma` multiplies `f(x,u)` in the collocation, so free burn time is a genuine
nonconvexity — and the source of several separate pathologies (§7).

---

## 2. Scaling is load-bearing

The solver works on `x~ = x/scale`, and **the trust region is in those units**.

> [!IMPORTANT]
> `TrustRegionMax = 0.1` with `XScale = 300 m` means the solver may move ~30 m of
> altitude per iteration — *regardless of how far away the target is*. Engage from
> 3 km with a scale sized for 300 m and the problem is 10× larger while the step
> stays fixed. The iteration budget simply cannot walk there.

This presented as **"cold solve failed"**, and looked exactly like a constraint or
seed problem. It was neither — the problem was perfectly feasible. Relaxing tilt,
throttle and terminal constraints did nothing, which was the clue: *if loosening
every constraint doesn't help, the solver isn't being blocked, it's being
out-walked.* `Ksa6DofSetup` now sizes `XScale` per-axis from the actual `x0 -> xf`
extent.

### Row equilibration, and one row that must be clamped

Column scaling to physical units is not enough; ADMM is *more* sensitive to row-scale
disparity than an interior-point method. But the lower bound on row norms is
load-bearing rather than defensive:

The **tilt row** linearises `R22 >= cos(tilt_max)` about the reference quaternion,
giving gradient `(-4qx, -4qy)` — which is **exactly zero for a perfectly vertical
booster** and tiny for a nearly vertical one, i.e. for the whole flight. Its constant
term stays O(0.13). Dividing that row by a ~1e-3 norm amplifies the right-hand side a
thousandfold and wrecks the conditioning of an otherwise well-scaled problem.

Only the *lower* bound is applied. SCS pairs it with an upper clamp of 1e4, but that
is inside its own Ruiz iteration on already-scaled data; imposing it on raw rows is
actively harmful, because the mass trust-region rows legitimately have norm ~2.5e5.

### Penalties must be scaled too

Adding soft path constraints, I penalised the **raw** slack. That puts a coefficient
of `weight * XScale = 1e6` next to objective terms of order 1e-2, and SCS does not
merely solve that slowly — it returns **`unbounded`**, because a cost that lopsided
makes the dual infeasible to working precision. Penalise the **normalised** slack so
the weight is dimensionless.

---

## 3. Node 0 is the vehicle — a whole family of bugs

Node 0 is pinned by an equality to the measured state. Almost everything that also
touches node 0 has been a bug at some point.

### The trust region applies at node 0 too

The box is `|X[k][i] - xbar[k][i]| <= tr * XScale[i]` for **all** `k`, including 0 —
while node 0 is *simultaneously* pinned to `x0`. Together these demand

```
|x0[i] - xSeed[0][i]| <= tr * XScale[i]
```

The cold seed wrote **identity quaternion and zero body rates at every node**. With
`XScale = 1` on those channels and `tr = 0.1`, the cold solve is **literally
infeasible for any vehicle more than ~11.5° off vertical or rotating faster than
0.1 rad/s.** Not slow — infeasible, immediately, and untouched by relaxing any
physical constraint.

Fix: slerp attitude from the measured state, spin rates down from measured, and copy
`x0` into seed node 0 verbatim.

### Path constraints must start at node 1

A hard constraint that node 0 *already violates* makes the problem infeasible **by
construction** — no plan at all, precisely when one is most needed.

> [!CAUTION]
> The sharper trap is two constraints *together*. Outside a glideslope cone and too
> low, the only way back inside is to **climb** — which a climb-rate constraint
> forbids. Hard versions of both can trap the vehicle in a region with no feasible
> exit. Soft versions cannot.

So the glideslope (a SOC per node) and the climb-rate limit (one linear row per node)
both **start at node 1** *and* carry a **penalised slack**.

### L1, not L2, for the slacks

L1 is an **exact** penalty: above a finite weight the solution is identical to the
hard-constrained one, so the slack sits at exactly zero whenever the corridor is
reachable. A quadratic penalty always trades a little violation for a little
objective, and would quietly fly just outside the cone forever.

---

## 4. Reading the answer back out

> [!IMPORTANT]
> `solver.ReferenceX` is the SCvx **reference** — it only advances on an **accepted**
> step. It is *not* the subproblem's answer.

The subproblem does pin node 0 to `x0`, so any *solved* step has node 0 at the
vehicle. But `Reseed` sets the reference to the previous plan shifted forward, whose
node 0 is the old plan's node `shift` — **not** the vehicle. A cycle where the ratio
test accepts nothing hands back a plan anchored an interval away, and the controls
get read at the wrong point on it.

`AnchorOffsetM` measures exactly this, and a plan more than a metre off is refused.

### A refused re-solve flies the old plan

When `Finish()` returns false, `_planX` and `_solveTime` are untouched — so `Command`
keeps reading further and further along an **ageing open-loop plan**. This is the
single most dangerous failure mode in the whole system, because everything looks
healthy while the feedback is silently switched off.

It caused a "sensible straight line that becomes a loop as we get closer", via a
defect gate that was accidentally tightening (see below). Every refusal is now logged
with its reason, and plan age is on the HUD.

### The defect gate must be in metres

`DefectNorm` is `max|defect| / XScale`, and XScale's position entries are **L, the
range to the target** — the one quantity guaranteed to shrink on an approach. A fixed
scaled tolerance therefore means `1e-3 * L` **metres**:

| range to go | allowed defect | actual defect at N=50 |
|---|---|---|
| 1000 m | 1.07 m | 0.04 m |
| 235 m | 0.26 m | 0.13 m |
| 50 m | **0.04 m** | 0.06 m ← refused |

The absolute defect is flat all the way down. Nothing degraded except the ruler — and
inside 100 m the solver was producing centimetre-accurate plans and having them
thrown away. The flight gate is now absolute (`MaxDefectM`, 1 m); the solver's own
convergence test still uses the scaled figure.

---

## 5. The actuator boundary

This is where the model meets the game, and it has been the richest source of
systematic error — because **MPC cannot correct a model error**. Re-solving fixes the
*state* estimate, not the actuator mapping, so every replan is executed just as
wrongly as the last.

### Thrust is not proportional to throttle in an atmosphere

Throttle sets **combustion pressure**, not thrust (`throttle * CombustionPressureMax`
into a gas-property LUT). Nozzle thrust is momentum plus pressure:

```
F = mdot*Ve + (Pe - Pa)*Ae
```

The momentum term scales with throttle. The ambient term `-Pa*Ae` **does not**. So

```
F(t) = t*F(1) - Pa*Ae*(1 - t)
```

and the deficit is a near-constant force, **largest at low throttle** — which is
where a descent lives. Measured in flight: `Pa*Ae` ≈ 334 kN, ~270 kN missing at the
~30% throttle flown, 2.2 m/s² on a 122 t vehicle.

`KsaEnginePerf.ThrottleForThrust` inverts KSA's own curve by bisection rather than
dividing by a full-throttle figure.

### Command newtons, divide by *live* capability

`throttle = demand / cfg.Tmax` freezes the divisor at plan time. Any drift in the
vehicle's real capability — pressure, an engine out, propellant starvation — scales
every commanded thrust by that ratio. `Command()` therefore returns **newtons**, and
the conversion happens at the KSA boundary.

### The control is a vector

The model's control is `u = (tdx, tdy, T, tau_roll)`: `T` is *axial* thrust and
`tdx/tdy` are the lateral components the gimbal produces. KSA's throttle sets the
**total** magnitude along the gimballed nozzle, so commanding `T` alone delivers a
vector of length `T` whose axial part is `T*cos(delta)`. Command `|u|`.

*(Measured deflections were 0.1–2.2°, so this is worth ~0.1% — fixed because it is
wrong, not because it is large.)*

### One gimbal setting does two jobs

In the model a gimbal deflection **tilts the thrust vector** *and* **torques the
vehicle** — `tdx/tdy` carry both. Any controller that re-tasks the gimbal as a pure
torque actuator (attitude supplying thrust direction instead) **cannot feed the
plan's gimbal command forward unchanged**: it commands rotation the attitude loop
never asked for, and `|torqueFf|` alone reaches the entire lateral actuator budget,
so anything on top saturates.

### Never tune a gain that multiplies inertia

Shipping `KAtt = 6, KRate = 4` looks reasonable and means `wn = 2.4 rad/s` — then
gets multiplied by `Ixx ~ 1e8 kg m²`. The demand was orders of magnitude past the
vehicle's authority and the loop went bang-bang. Parameterise by the closed-loop
frequency you actually want (`kAtt = wn²`, `kRate = 2*zeta*wn`); a booster attitude
loop lives at 0.3–0.5 rad/s.

### Don't over-constrain the terminal attitude

Pinning the terminal quaternion to identity constrains **yaw** as well as tilt — and
yaw about the thrust axis is the one attitude freedom a landing does not care about.
It is a **roll** in the body frame, the axis with least authority by far (roll comes
only from off-axis vernier gimbals, measured ~700× weaker than pitch/yaw). A vehicle
that happens to be yawed 180° is then required to roll 180° on its weakest axis for
no purpose. Terminal attitude is now *upright at whatever yaw the vehicle has*.

### Everything must be measured, not assumed

`Ksa6DofSetup` derives gravity, thrust, Isp, inertia, engine arm, gimbal limit, roll
authority and throttle floor from the live vehicle. **Getting these wrong does not
error** — the solver happily plans a perfectly feasible trajectory for the wrong
vehicle.

Two that bite:

- **Inertia is live.** It is rebuilt as propellant drains, so a value captured at
  engage goes stale over a burn that spends a meaningful fraction of wet mass. That is
  a *systematic* torque error, so MPC cannot correct it. Re-read every cycle.
- **The diagonal inertia approximation is exact for an axisymmetric booster** —
  verified to 8e-16 over 5000 random orientations. The transverse inertia is
  degenerate, so *any* perpendicular pair are principal axes and the arbitrary roll
  reference `BodyAxes` picks is provably harmless. `Inertia()` reports
  `offDiagonalRatio` and `transverseAsymmetry` so the assumption stays checkable.

---

## 6. Making it real-time

Offline validation numbers are not flight numbers, and shipping them caused hangs.

| Setting | Offline | Flight | Why |
|---|---|---|---|
| ADMM iterations | 100 000 | time-budgeted | 100k on the sim thread is a freeze, then a watchdog kill |
| Subproblem `eps` | 1e-7 | **1e-4** | 1e-5/1e-4/1e-3 give the *same* trajectory |
| SCvx iterations | 150 | 5 | real-time iteration; 1 is measurably as good |

### An iteration cap does not bound time

One ADMM iteration costs ~2.8× more at N=80 than at N=30, so a fixed budget is a
different wall-clock cost at every node count. The cap is now derived each cycle from
a measured cost-per-iteration against a **wall-clock** budget.

### Tolerance: the tail is almost the entire bill

At 1e-5 roughly **half** of all solves overran the 40 ms budget and were truncated; at
1e-4, under a tenth do. p50 went 36.5 → 12.7 ms, p90 251 → 25.6 ms, with an identical
trajectory.

> [!TIP]
> **Measure the distribution, not the mean.** At N=80 the mean was 82 ms while p50 was
> 36 ms and the max was 964 ms. "Fast with periodic hitches" and "uniformly slow" have
> the same mean and need completely different fixes.

### A truncated iterate poisons the warm start

SCS reports truncation as `SolvedInaccurate`, which passes `IsUsable()`. Storing it
seeds the *next* solve from a half-converged iterate, making that one more likely to
truncate too — which is why long solves arrive in **bursts** rather than singly.

### Our SCS build had its accelerator switched off

All of `aa.c` (Anderson acceleration) is wrapped in `#ifndef USE_LAPACK`, and the
fallback is not a slower path — it is a **no-op**. SCS enables acceleration by default
(`ACCELERATION_LOOKBACK = 10`) and it is the mechanism that holds ADMM iteration count
down on ill-conditioned problems, which every SCvx subproblem is.

`aa.c` is now compiled alone with `-DUSE_LAPACK` against a six-routine shim
(`scvx/native_src/blas_shim.c`), because `struct ACCEL_WORK` is private to that
translation unit so no other file's ABI depends on the macro.

### Where the time actually goes

`scs_init` is only **2–6%** of solve time; ADMM is the other 94–98%. An earlier plan
to reuse the symbolic factorisation would have recovered ~5%. *Measure before
optimising* — that one cost a day of misdirected effort.

### Re-solve cadence

Shorter is **cheaper**, which is the opposite of the intuition: the warm start is only
good while the vehicle is near its previous plan, so a longer interval makes each
solve *harder* as well as the plan staler. Past ~2 plan nodes of advance the warm
start is too stale and the solver thrashes.

### Node gates

Node count steps down at fixed altitudes — **ten nodes per step, 50 down to 10
between 1000 m and 100 m**, never outside `[10, 50]`. Never continuously: every change
rebuilds the solver and discards the ADMM warm start, since the sparsity pattern is
frozen at the node count. The **reference trajectory survives** by interpolation, and
that is the seed that matters — measured 5–90× cheaper than a cold solve at the gate.

The floor of 10 is not arbitrary. Collocation defect grows with node spacing, and at
N=10 it is 2.36 m at 235 m altitude against the 1 m flight gate — so N=10 is only
safe once the horizon is short, which is why that rung sits at 100 m (0.41 m there).

Hold node **spacing** roughly constant, not node count: collocation error grows with
spacing, and an earlier "N=20 gives 4 m of plan jump" was measured on a full-length
trajectory where N=20 meant coarse spacing.

---

## 7. Model error: what MPC cannot fix

> [!IMPORTANT]
> MPC re-anchors the **state** every cycle but keeps planning with the same **model**.
> A persistent force it does not know about is met identically on every replan: the
> plan promises to arrive, the vehicle falls short, the next plan promises again.

This is the single most important idea in this codebase. Every symptom below was
originally misdiagnosed as a solver or tuning problem.

`Ksa6DofGuidance.SetAccelBias` implements **offset-free MPC**: estimate the residual
acceleration (measured minus `thrust/m + gravity`), filter it (~2 s), clamp it
(5 m/s²), and add it to the planner's gravity. Estimating the *residual* rather than
any individual term needs no theory about which of gravity, thrust calibration or
aerodynamics is responsible — and it picks up drag for free, including drag falling
away as speed comes off.

### A constant force is indistinguishable from a gravity error

Fitting `az = k*(T/m)*zb_z - g` for **both** `k` and `g` returned `k = 0.897`,
`g = 10.74` with a tight residual (rms 0.194) and stable split-halves — and was
**wrong**. The real gravity was 9.81 and all of it was thrust. Pin what you know
before fitting; a two-parameter fit will happily trade a constant force against
gravity and look convincing doing it.

### The objective can bias the answer

Min-fuel should prefer a *shorter* burn, so burn time running to its upper bound means
the objective is rewarding length. It was: `W_DU` and `W_W` both get **cheaper as
sigma grows**, and at the reference solution came to 121% of the fuel term.

Cutting them fixed the bias and **tripled the ADMM iteration count**, because those
same terms were adding positive-definite mass to `P`. The fix is a **proximal** term —
`rho*||(X - Xbar)/Xscale||²` — which restores exactly that conditioning *without* the
bias: centred on the current reference, so it prefers neither slow rotation nor long
burns, and vanishes at convergence.

`W_DU` and `W_W` are not interchangeable. `W_DU` is the only thing keeping the control
profile **continuous** — min-fuel with no control-rate penalty is bang-bang, and the
optimum genuinely has thrust direction jumping between nodes (19.1° node-to-node at
`W_DU = 0`, 3.9° at 0.05).

### An over-powered vehicle is a real physical situation

The model has a convex throttle box `Tmin <= T <= Tmax`, so a lit engine can never go
below `Tmin = floor * Tmax`. **If `Tmin` exceeds weight the vehicle cannot hold
altitude pointing up** — the only way to shed the excess is to *tilt* until the
vertical component falls to 1 g, needing `acos(1/TWRmin)` of tilt. Past the tilt limit
no descent exists and the solver reports infeasible; just inside it, the path curves
away sideways. That is one origin of spiral trajectories, and it is not a bug to be
modelled around — the feasibility panel reports it.

### Reachability is not conditioning

Thrust only buys deceleration **above** hover, so usable authority is `(TWR-1)*g`.
From 300 m at 50 m/s, stopping needs 127 m at TWR 2.0, 255 m at 1.5 and **510 m at
1.25** — more altitude than exists. The solver returns its best effort, which is a
long curving miss that looks exactly like a controller bug.

Higher gravity is *not* a conditioning problem: at equal TWR, Earth tracks comparably
to the Moon and solves an order of magnitude faster.

---

## 8. Host integration

### An exception in an ImGui draw names the wrong function

ImGui keeps a **window stack**, so an exception that unwinds past `End()` fails at
end-of-frame as `window Powered Guidance: missing End` — then every frame after,
always naming the window, never the fault. `Draw` now ends the window in a `finally`;
the exception still propagates with its stack trace, but the ImGui stack stays
balanced so the *real* error surfaces.

### Format strings

The C# overloads take one `ImString`, so formatting happens native-side:

| Call | Behaviour | Escape |
|---|---|---|
| `ImGui.Text` | maps to `igTextUnformatted` | literal `%` |
| `ImGui.TextColored` | printf | `%%` |
| `ImGui.TextWrapped` | printf | `%%` |

A lone `%)` is an invalid conversion specifier reading a vararg that was never pushed.

### KSA nullability, and validity predicates that lie

`AmbientPressureAt` gated on `PhysicalAtmosphereReference.IsValid()`, which requires
`ScaleHeight.IsValid()` — and `DistanceReference.IsValid()` is
`Math.Abs(value) > 100000.0`, i.e. **over 100 km**. Atmospheric scale heights are
single-digit kilometres, so that predicate is **false for every atmosphere in the
game**. It is a test written for orbital distances applied to a scale height.

Zero was not a harmless answer: `ComputeActivePerformance` returns `VacuumData` when
pressure ≤ 0, so both the planner's `Tmax` and the throttle divisor silently became
**vacuum thrust**.

> [!TIP]
> **Validate the computed result, not the container.** And when a game API's semantics
> matter, decompile it — `ComputeActiveThrust` sounds like a measurement and is
> actually a full-throttle *capability* (`ComputeConditions(1f)` is hardcoded).

`AtmosphereReference` and `EngineController.Cores` are both nullable; `?.` on the
parent alone does not protect the chain.

---

## 9. Diagnostics

Every one of these exists because a symptom could not be attributed from the outside.

**Console** (`scvx/Scvx.Console`):

| Mode | Question it answers |
|---|---|
| `--loop`, `--sub-scs` | Do we still reproduce the Python reference? |
| `--mpc` | Closed-loop MPC with dispersions — the only thing that reproduced flight symptoms |
| `--mpc --split` / `--tail` | Where does solve time go; what does tolerance buy |
| `--mpc --grav` | Gravity vs thrust margin, at dynamic similarity |
| `--path`, `--gates`, `--defect` | Constraints bind; node transitions carry; defect gate sanity |
| `--spiral` | Reproduce a logged engage state offline |

**Flight telemetry** (`SixDofLog` → `tools/readlog.py`) writes `-cycle.csv`,
`-plan.csv` and `-events.log`.

> [!NOTE]
> The plan snapshots earn their place. A cycle row only records where the vehicle
> *got to*, so on its own it can never distinguish "the plan is a loop and the vehicle
> followed it" from "the plan was straight and the vehicle diverged" — and those have
> opposite causes.

Two rules the logger must obey: **nothing may throw** (see §8) and **nothing may
stall** (buffer, flush on an interval). Smoke-test it under a comma-decimal culture —
that caught event timestamps written as `13,00`.

### How to read a spiral

- **Decompose horizontal velocity** against the line to the target into *closing* and
  *cross*. An orbit is closing → 0 while cross stays large.
- **Look at velocity error, not position error.** One-second position prediction was
  good to 0.6 m while velocity was off by 2.37 m/s — and sustained over 15 s that was
  exactly the 28 m/s of cross-range accumulated.
- **A path/direct ratio is useless on a steep descent.** A 100 m lateral excursion on
  a 1264 m mostly-vertical path barely moves it. Measure the horizontal track
  separately.

---

## 10. Still open

- **The plan itself spirals.** From a logged engage state the flight solver returned
  `IterationLimit` at sigma 27.2 s and swept **+319°** of bearing around the target,
  while the vehicle tracked it to 4° of thrust direction. The same initial condition
  reproduced offline converges to a direct descent under every variation tried. The
  reconstruction is missing something — the leading suspects are the *measured*
  inertia, roll authority and gimbal limit, none of which the offline case uses.
- **The tilt constraint binds hard**: the plan demands 63.7–64.6° against a 60° cap.
- **Hand-off happens at −25.9 m/s** at 30 m, which the hover controller will struggle
  with regardless.
- The acceleration-bias estimator should now settle near **zero**; it sitting at a
  constant offset means another systematic force is still unaccounted for.
