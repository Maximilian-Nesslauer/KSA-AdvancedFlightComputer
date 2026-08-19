# 6-DOF Powered-Descent Guidance

This is a real-time optimal-control autopilot for landing a rocket booster. Every
tenth of a second it solves a trajectory-optimisation problem from the vehicle's
current state and flies the first fraction of the answer.

This document explains the chain end to end: how a landing becomes a convex cone
program, how the cone program becomes a trajectory, and how the trajectory becomes a
throttle setting and a gimbal deflection. Where the design departs from textbook
successive convexification, that is called out explicitly.

For the development history and the specific traps encountered along the way, see
[DEVNOTES.md](DEVNOTES.md).

---

## Contents

1. [The problem](#1-the-problem)
2. [Why it isn't convex, and what to do about it](#2-why-it-isnt-convex-and-what-to-do-about-it)
3. [Discretisation: a trajectory as a finite list of numbers](#3-discretisation-a-trajectory-as-a-finite-list-of-numbers)
4. [The convex subproblem](#4-the-convex-subproblem)
5. [The SCvx loop](#5-the-scvx-loop)
6. [From plan to vehicle commands](#6-from-plan-to-vehicle-commands)
7. [Receding horizon: this is an MPC, not a planner](#7-receding-horizon-this-is-an-mpc-not-a-planner)
8. [Departures from textbook SCvx](#8-departures-from-textbook-scvx)
9. [Code map](#9-code-map)

---

## 1. The problem

A booster is falling. Given where it is now, find the thrust and gimbal history that
lands it on the pad, upright and stopped, using as little propellant as possible —
while respecting everything the vehicle physically cannot do.

**State** (14 numbers):

| | Symbol | Meaning |
|---|---|---|
| Position | `r` (3) | metres, in a frame fixed to the landing site, `+z` up |
| Velocity | `v` (3) | m/s |
| Attitude | `q` (4) | unit quaternion, scalar-first |
| Body rates | `ω` (3) | rad/s |
| Mass | `m` (1) | kg, falls as propellant burns |

**Control** (4 numbers):

| | Symbol | Meaning |
|---|---|---|
| Axial thrust | `T` | along the body's thrust axis |
| Lateral thrust | `tdx`, `tdy` | the sideways component produced by deflecting the engine |
| Roll torque | `τ` | about the thrust axis, from off-axis vernier engines |

The first three are one physical thing: a **thrust vector** in body coordinates. The
engine is on a gimbal, so pointing it slightly off-axis produces a sideways force
*and*, because the engine sits below the centre of mass, a torque that rotates the
whole vehicle. Roll is separate because a single centreline engine cannot produce it.

**Constraints:**

- **Throttle box**: `Tmin ≤ T ≤ Tmax`. A lit engine has a minimum thrust — it cannot
  be turned down arbitrarily far.
- **Gimbal cone**: `‖(tdx, tdy)‖ ≤ tan(δmax)·T`. The engine deflects only so far.
- **Tilt limit**: the vehicle may not lean more than some angle from vertical.
- **Roll authority**: `|τ| ≤ τmax`.
- **Ground**: altitude stays above zero.
- Plus optional **glideslope** and **descent-rate** corridors (§4).

**Objective**: maximise final mass, i.e. burn as little propellant as possible.

---

## 2. Why it isn't convex, and what to do about it

If this problem were convex, we could solve it once, exactly, and be done. It isn't,
for three reasons:

1. **Rotational dynamics.** `q̇` depends on `q ⊗ ω` and `ω̇` on `I⁻¹(τ − ω × Iω)` —
   products of unknowns.
2. **The thrust vector rotates with the vehicle.** Body-frame thrust becomes
   world-frame acceleration through `q`, another product of unknowns.
3. **Free final time.** We do not know in advance how long the burn takes, and the
   burn duration `σ` multiplies the dynamics.

**Successive convexification (SCvx)** handles this by iterating:

> Guess a trajectory. Linearise the dynamics *about that guess*. The result is convex,
> so solve it exactly. Use the answer as the next guess. Repeat.

The linearisation is only valid near the guess, so each step is confined to a **trust
region**. If a step turns out to predict the true nonlinear behaviour well, it is
accepted and the region grows; if not, it is rejected and the region shrinks. This is
the same accept/reject machinery as a trust-region Newton method, applied to a whole
trajectory instead of a single point.

---

## 3. Discretisation: a trajectory as a finite list of numbers

A trajectory is a continuous function of time; a solver needs a finite vector. The
trajectory is represented by its values at `N` **nodes**, evenly spaced in normalised
time `τ ∈ [0,1]`, and the actual duration is the separate variable `σ`.

Between nodes the dynamics are enforced by **trapezoidal collocation** — the state at
one node must equal the state at the previous node plus the average of the derivatives
at both ends:

```
X[k+1] = X[k] + ½·dτ·σ·( f(X[k],U[k]) + f(X[k+1],U[k+1]) ) + Wv[k]
```

`f` is the true nonlinear dynamics; the linearisation supplies its Jacobians `A`, `B`
about the current guess.

Two consequences matter throughout:

**Node spacing sets accuracy.** The real spacing is `dt = σ/(N−1)` seconds. Collocation
error grows with `dt`, so the same node count is generous close to the pad and coarse
at altitude, where `σ` is large. Measured across a descent, `dt < 2 s` keeps the error
under 0.1 m while `dt > 8 s` pushes it past 0.5 m. **The meaningful knob is spacing,
not node count** — see §8.

**`Wv` is a slack variable.** That is the *virtual control* term, and it is the single
most important thing to understand about a partially converged plan (§5).

---

## 4. The convex subproblem

Each SCvx iteration builds one cone program in the form the solver
([SCS](https://github.com/cvxgrp/scs)) accepts natively:

```
minimise    ½ xᵀPx + cᵀx
subject to  Ax + s = b,   s ∈ K
```

where `K` is a product of cones: an equality (zero) cone, a non-negative orthant, and
a set of second-order cones. Everything stacks into **one** matrix `A`, in that order.

### The variables

| Block | Count | What |
|---|---|---|
| `X` | `N × 14` | state at every node |
| `U` | `N × 4` | control at every node |
| `Wv` | `(N−1) × 14` | virtual control, one per interval |
| `σ` | 1 | burn duration |
| slacks | 0–2 per node | glideslope and descent-rate relaxations (§8) |

### The equality rows

| Rows | Constraint |
|---|---|
| 14 | initial state = the measured vehicle state |
| 13 | terminal state = the target (mass is free) |
| `(N−1) × 14` | the collocation identity above |
| `N−2` | **quaternion norm**, linearised as `q̄·q = 1` at interior nodes |

That last one is worth a note: a unit-norm constraint is not convex, but *linearised
about the current guess* it becomes a plane, and the iteration pulls the quaternion
back onto the sphere. Endpoints are excluded because they are already pinned.

### The inequality rows

Per node: the throttle box, roll-torque limits, the linearised tilt limit, and a ground
floor. Then a **trust region** box on every state, control and on `σ`, and the bounds on
`σ` itself.

The tilt constraint is `R₂₂ ≥ cos(tilt_max)` where `R₂₂ = 1 − 2(qx² + qy²)` — again not
convex, again linearised about the guess.

### The second-order cones

One per node for the **gimbal**: `tan(δmax)·T ≥ ‖(tdx, tdy)‖`. This is the one
constraint that is genuinely conic rather than merely linearised — the set of
achievable thrust vectors really is an ice-cream cone, and the solver handles it
exactly. Optionally a second cone per node for the glideslope corridor (§8).

### The objective

- **Linear**: `−m_final / m_initial` — maximise the propellant left. Plus L1 penalties
  on the path-constraint slacks.
- **Quadratic** (`P`): a penalty `ρ_vc‖Wv‖²` on virtual control, small penalties on
  control rate and body rate, and a **proximal** term (§8).

### Scaling

The solver never sees SI units. Every variable is divided by a characteristic scale —
length by the distance to the target, speed by a characteristic speed, mass by the
initial mass — so all variables are order 1. This is not cosmetic: **the trust region
is expressed in scaled units**, so the scale determines how far the solver may move per
iteration. Rows are separately equilibrated, because first-order methods like ADMM are
even more sensitive to row-scale disparity than interior-point methods are.

---

## 5. The SCvx loop

```
seed a reference trajectory
repeat:
    linearise the dynamics about the reference        →  A, B, f0
    assemble and solve the convex subproblem          →  candidate X, U, σ
    integrate the TRUE nonlinear dynamics along it    →  actual cost and defect
    ρ = actual improvement / predicted improvement
    if ρ is good:  accept the candidate as the new reference, grow the trust region
    else:          reject it, shrink the trust region and retry
until the step and the defect are both small
```

The ratio test `ρ` is what keeps the linearisation honest. The subproblem always
believes its own prediction; comparing that against the true dynamics is the only
thing that detects a step that went too far.

### Virtual control, and why a converged plan matters

`Wv` is a free variable in the collocation identity. The solver can always satisfy the
dynamics by setting `Wv` to whatever is needed — it just pays `ρ_vc‖Wv‖²` for it. That
is deliberate and necessary: without it, an early linearisation could make the problem
infeasible and the iteration would die before it started.

But it means an unconverged trajectory **does not obey physics**. The state jumps
between nodes on a force that does not exist. Such a plan is not slightly wrong; it is
*unflyable* — no thrust setting reproduces it.

So the **defect** — the size of that fictitious force, converted to metres — is the
measure of whether a plan is real. This implementation refuses to fly any plan whose
defect exceeds a threshold, keeping the previous plan instead and reporting why. That
is a departure from the textbook algorithm, which simply returns whatever it has when
the iteration budget runs out.

---

## 6. From plan to vehicle commands

The output is a trajectory: state and control at `N` nodes over `σ` seconds. Turning
that into actuator commands is a short chain, but every link has a convention that must
match the game's.

### Frames

The model works in a **site frame** — origin at the landing target, `+z` up — and treats
it as inertial. The game works in a body-centred inertial frame and uses a
scalar-**last** quaternion; the model uses scalar-**first**. `KsaFrameBridge` converts
in both directions, routing through rotation matrices so the conversion cannot depend on
either convention being guessed right.

The model's body axes are derived from the *measured thrust direction* rather than
assumed, so a vehicle assembled with any orientation still works. A round-trip check
(convert out, convert back, compare) is exposed in the UI as the single number that
would catch an axis swap, a handedness error or a sign flip.

### Reading the control at the current instant

The plan's control is sampled at `t = now − solve_time` and interpolated linearly
between nodes — matching the assumption the collocation itself made. Immediately after
a solve this returns node 0's control, which is the control the optimiser chose *for the
vehicle's actual current state*.

### Torque

The model's lateral thrust becomes a body torque directly, from the geometry of an
engine hanging below the centre of mass:

```
τ_body = r_engine × T_body = ( L·tdy, −L·tdx, τ_roll )
```

That torque is converted to the game's body axes and handed to an allocator, which
solves a small least-squares problem for the per-gimbal deflections that produce it —
including the roll verniers. The allocator clamps each axis independently, because roll
authority on this class of vehicle is hundreds of times weaker than pitch and yaw, and a
single over-large roll demand would otherwise scale down the whole solution.

### Thrust

The optimiser asks for **newtons**, not a throttle fraction. Converting to a throttle
setting requires knowing what the engines can actually produce *right now*, which only
the game side knows, so that conversion happens at the boundary rather than inside the
guidance.

It is not a division. In an atmosphere, nozzle thrust is

```
F = ṁ·Ve + (Pe − Pa)·Ae
```

The momentum term scales with throttle, but the ambient back-pressure term `−Pa·Ae`
does not — throttle sets *combustion pressure*, not thrust. So

```
F(t) = t·F(1) − Pa·Ae·(1 − t)
```

which is a near-constant deficit, largest at low throttle, exactly where a descent
lives. The commanded throttle is therefore obtained by **inverting the engine's real
thrust curve numerically**, so the newtons requested are the newtons delivered.

---

## 7. Receding horizon: this is an MPC, not a planner

The trajectory is never flown open loop. Every cycle:

1. Measure the vehicle state.
2. Re-solve, seeded with the previous plan shifted forward in time.
3. Apply the first fraction of the new plan.
4. Discard the rest.

There is no trajectory-tracking controller — no PD loop, no attitude reference, no
gain schedule. **The feedback is the re-solve.** Because node 0 is pinned to the
measured state, every plan begins where the vehicle actually is, and disturbances are
absorbed by re-planning rather than corrected against a stored reference.

Two properties follow, and they cut in opposite directions:

- Anything wrong with the vehicle's *state* is corrected automatically, every cycle.
- Anything wrong with the *model* is not corrected at all. Every re-plan meets the same
  error and makes the same mistake. §8 covers the mechanism added for that.

Warm starting matters enormously here. The shifted previous plan is a very good guess,
so a re-solve typically takes a few milliseconds against a few hundred for a cold start.

---

## 8. Departures from textbook SCvx

Everything so far is standard. These are the additions this implementation makes, and
why.

### Offset-free MPC: an estimated acceleration bias

Standard MPC re-anchors the *state* but keeps planning with the same *model*. If a
persistent force is missing from the model — a thrust calibration error, drag, a wrong
gravity constant — every re-plan encounters it identically: the plan promises to
arrive, the vehicle falls short, the next plan promises again. Re-solving cannot fix a
model error.

So the guidance estimates the residual acceleration

```
bias = measured acceleration − ( thrust/m + gravity )
```

low-pass filters it, clamps it, and **adds it to the planner's gravity vector**. The
optimiser then plans *around* the disturbance instead of rediscovering it every cycle.

Estimating the lumped residual rather than any individual term is deliberate: it
requires no theory about which effect is responsible, and it tracks drag falling away as
speed comes off.

### A proximal term instead of large regularisers

The reference formulation penalises control rate and body rate to keep the trajectory
smooth. Those penalties also, incidentally, add positive-definite mass to `P` and hold
the subproblem's conditioning together — so tuning them down to fix the trajectory
triples the solver's iteration count.

A **proximal** term `ρ‖(X − X̄)/scale‖²` restores exactly that conditioning without the
side effect. Being centred on the current reference rather than on zero, it expresses no
preference about the answer, and it vanishes at convergence where `X = X̄`.

### Soft path constraints that start at node 1

Optional glideslope and descent-rate corridors are added as a second-order cone and a
linear row per node. Two design choices make them safe:

**They skip node 0.** Node 0 is pinned by equality to the measured state. A hard
constraint that the vehicle is *already violating* makes the problem infeasible by
construction — no plan at all, precisely when one is most needed.

**They carry penalised slacks.** The sharper hazard is two corridors interacting:
outside a glideslope cone and too low, the only way back inside is to climb — which a
descent-rate constraint forbids. Hard versions of both can trap the vehicle in a region
with no feasible exit. Soft versions cannot.

The penalties are **L1, not quadratic**. An L1 penalty is *exact*: above a finite
weight, the solution is identical to the hard-constrained one, so the slack sits at
exactly zero whenever the corridor is reachable. A quadratic penalty always trades a
little violation for a little objective, and would quietly fly just outside the corridor
forever.

### A wall-clock budget, not an iteration cap

Textbook SCvx runs to a tolerance. Running inside a game's simulation thread, a solve
that takes a second is a visible freeze.

An iteration cap does not bound time — one ADMM iteration costs several times more at
80 nodes than at 30. So the cap is derived each cycle from a *measured* cost per
iteration against a wall-clock budget, with a larger escalated budget for the rare hard
subproblem.

The flight tolerance is also far looser than the validation tolerance. Measured across a
descent, `1e-5`, `1e-4` and `1e-3` produce the same trajectory at very different cost;
the tail of the ADMM iteration is almost the entire bill and buys nothing.

### A defect gate in metres

Whether a plan is flyable is an absolute question — a 10 cm discrepancy over a 20 second
trajectory is fine whether the target is 2 km away or 50 m. But the solver's internal
defect is normalised by the problem scale, whose length is the *range to target*, so a
fixed scaled tolerance silently tightens as the vehicle closes in.

The flight gate is therefore expressed in **metres**. The solver's own convergence test
still uses the scaled figure, where it belongs.

### A node ladder

Node count steps down at fixed altitudes rather than being held constant. Since spacing
is `σ/(N−1)` and `σ` shrinks on approach, a fixed count buys steadily finer resolution
than the problem needs.

It is a ladder rather than a continuously tracked value because changing `N` changes the
problem's dimensions: the sparsity pattern is frozen at construction, so a new count
means a new solver and the loss of the warm start. The reference *trajectory* survives
by interpolation onto the new node count, and that is the seed that matters — measured
5–90× cheaper than a cold solve at the transition.

### Hand-off to a hover controller

The last few metres are the worst case for this solver and the easiest for a simple
controller. The horizon has collapsed to a handful of nodes over a second or two, so the
trust region rather than the physics is what binds; meanwhile the terminal state —
near-zero velocity, upright, holding a point — is exactly what a hover PID is for.
Optimising a descent is the wrong question by then, so the guidance hands over.

---

## 9. Code map

| File | Role |
|---|---|
| `scvx/Scvx.Core/Dynamics6Dof.cs` | The nonlinear dynamics and their Jacobians |
| `scvx/Scvx.Core/Scvx6DofSubproblemScs.cs` | Builds one cone program per iteration |
| `scvx/Scvx.Core/Scvx6DofSolver.cs` | The SCvx loop: linearise, solve, ratio test, trust region |
| `scvx/Scvx.Core/ScsWorkspace.cs` | SCS bindings and iterate-level warm starting |
| `ksamod/Scvx/KsaFrameBridge.cs` | Frame, body-axis and quaternion conversions |
| `ksamod/Scvx/Ksa6DofSetup.cs` | Measures the live vehicle into a solver configuration |
| `ksamod/Scvx/Ksa6DofGuidance.cs` | The MPC loop and the command interface |
| `ksamod/KsaTvcAllocator.cs` | Body torque → per-gimbal deflections |
| `ksamod/KsaEnginePerf.cs` | Engine capability and the thrust-curve inversion |
| `ksamod/PoweredGuidance6Dof.cs` | UI, stepping, telemetry, hand-off |

### Validation

`scvx/Scvx.Console` runs the solver headless against a JAX reference implementation and
a set of behavioural checks — closed-loop MPC with injected dispersions, path-constraint
enforcement, node-ladder transitions, defect behaviour against range, and solve-time
distributions. `dotnet run --project scvx/Scvx.Console -c Release -- --help` lists them.

In flight, `SixDofLog` writes per-cycle telemetry, periodic whole-plan snapshots and an
event log; `tools/readlog.py` summarises a run.
