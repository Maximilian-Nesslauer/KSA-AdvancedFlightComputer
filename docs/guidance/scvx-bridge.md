# Touchdown guidance on the craft

Touchdown guidance re-solves a trajectory-optimisation problem from the craft's measured state every tenth of a second and flies the first part of the answer. This page covers what happens around the solver: how a plan becomes a throttle setting and gimbal deflections, the receding-horizon loop, and where the flight design departs from textbook successive convexification.

The problem itself - state, dynamics, every constraint and how it is made convex, the objective and this solver's settings - is on [Touchdown guidance (6-DOF)](scvx/touchdown.md). The machinery all of AFC's SCvx planners share is on [Successive convexification](scvx/index.md).

For the development history and the specific traps encountered along the way, see [scvx-bridge-devnotes.md](scvx-bridge-devnotes.md).

!!! note "Older than the problem page"
    Some details below predate the current code. Where they differ, [Touchdown guidance (6-DOF)](scvx/touchdown.md) is current. In particular: a thrust tracker and an attitude feedback now act between re-solves; the defect gate now judges each state channel in its own units, weighted by how far ahead the interval is; the node count now follows a node-spacing target rather than altitude gates; and there is no planned hand-off to a hover controller, only fallback rules after the plan.

---

## Contents

1. [From plan to vehicle commands](#1-from-plan-to-vehicle-commands)
2. [Receding horizon: this is an MPC, not a planner](#2-receding-horizon-this-is-an-mpc-not-a-planner)
3. [Departures from textbook SCvx](#3-departures-from-textbook-scvx)
4. [Code map](#4-code-map)

---

## 1. From plan to vehicle commands

The output is a trajectory: state and control at `N` nodes over `sigma` seconds. Turning that into actuator commands is a short chain, but every link has a convention that must match the game's.

### Frames

The model works in a **site frame** - origin at the landing target, `+z` up - and treats it as inertial. The game works in a body-centred inertial frame and uses a scalar-**last** quaternion; the model uses scalar-**first**. `KsaFrameBridge` converts in both directions, routing through rotation matrices so the conversion cannot depend on either convention being guessed right.

The model's body axes are derived from the *measured thrust direction* rather than assumed, so a vehicle assembled with any orientation still works. A round-trip check (convert out, convert back, compare) is exposed in the UI as the single number that would catch an axis swap, a handedness error or a sign flip.

### Reading the control at the current instant

The plan's control is sampled at `t = now - solve_time` and interpolated linearly between nodes - matching the assumption the collocation itself made. Immediately after a solve this returns node 0's control, which is the control the optimiser chose *for the vehicle's actual current state*.

### Torque

The model's lateral thrust becomes a body torque directly, from the geometry of an engine hanging below the centre of mass:

```
tau_body = r_engine x T_body = ( L*tdy, -L*tdx, tau_roll )
```

That torque is converted to the game's body axes and handed to an allocator, which solves a small least-squares problem for the per-gimbal deflections that produce it - including the roll verniers. The allocator clamps each axis independently, because roll authority on this class of vehicle is hundreds of times weaker than pitch and yaw, and a single over-large roll demand would otherwise scale down the whole solution.

### Thrust

The optimiser asks for **newtons**, not a throttle fraction. Converting to a throttle setting requires knowing what the engines can actually produce *right now*, which only the game side knows, so that conversion happens at the boundary rather than inside the guidance.

It is not a division. In an atmosphere, nozzle thrust is

```
F = m-dot*Ve + (Pe - Pa)*Ae
```

The momentum term scales with throttle, but the ambient back-pressure term `-Pa*Ae` does not - throttle sets *combustion pressure*, not thrust. So

```
F(t) = t*F(1) - Pa*Ae*(1 - t)
```

which is a near-constant deficit, largest at low throttle, exactly where a descent lives. The commanded throttle is therefore obtained by **inverting the engine's real thrust curve numerically**, so the newtons requested are the newtons delivered.

---

## 2. Receding horizon: this is an MPC, not a planner

The trajectory is never flown open loop. Every cycle:

1. Measure the vehicle state.
2. Re-solve, seeded with the previous plan shifted forward in time.
3. Apply the first fraction of the new plan.
4. Discard the rest.

There is no trajectory-tracking controller - no PD loop, no attitude reference, no gain schedule. **The feedback is the re-solve.** Because node 0 is pinned to the measured state, every plan begins where the vehicle actually is, and disturbances are absorbed by re-planning rather than corrected against a stored reference.

Two properties follow, and they cut in opposite directions:

- Anything wrong with the vehicle's *state* is corrected automatically, every cycle.
- Anything wrong with the *model* is not corrected at all. Every re-plan meets the same error and makes the same mistake. Section 3 covers the mechanism added for that.

Warm starting matters enormously here. The shifted previous plan is a very good guess, so a re-solve typically takes a few milliseconds against a few hundred for a cold start.

---

## 3. Departures from textbook SCvx

These are the additions this implementation makes to textbook SCvx, and why. Each one, with its current settings, is also in [What this solver does differently](scvx/touchdown.md#what-this-solver-does-differently).

### Offset-free MPC: an estimated acceleration bias

Standard MPC re-anchors the *state* but keeps planning with the same *model*. If a persistent force is missing from the model - a thrust calibration error, drag, a wrong gravity constant - every re-plan encounters it identically: the plan promises to arrive, the vehicle falls short, the next plan promises again. Re-solving cannot fix a model error.

So the guidance estimates the residual acceleration

```
bias = measured acceleration - ( thrust/m + gravity )
```

low-pass filters it, clamps it, and **adds it to the planner's gravity vector**. The optimiser then plans *around* the disturbance instead of rediscovering it every cycle.

Estimating the lumped residual rather than any individual term is deliberate: it requires no theory about which effect is responsible, and it tracks drag falling away as speed comes off.

### A proximal term instead of large regularisers

The reference formulation penalises control rate and body rate to keep the trajectory smooth. Those penalties also, incidentally, add positive-definite mass to `P` and hold the subproblem's conditioning together - so tuning them down to fix the trajectory triples the solver's iteration count.

A **proximal** term `rho||(X - X-bar)/scale||^2` restores exactly that conditioning without the side effect. Being centred on the current reference rather than on zero, it expresses no preference about the answer, and it vanishes at convergence where `X = X-bar`.

### Soft path constraints that start at node 1

Optional glideslope and descent-rate corridors are added as a second-order cone and a linear row per node. Two design choices make them safe:

**They skip node 0.** Node 0 is pinned by equality to the measured state. A hard constraint that the vehicle is *already violating* makes the problem infeasible by construction - no plan at all, precisely when one is most needed.

**They carry penalised slacks.** The sharper hazard is two corridors interacting: outside a glideslope cone and too low, the only way back inside is to climb - which a descent-rate constraint forbids. Hard versions of both can trap the vehicle in a region with no feasible exit. Soft versions cannot.

The penalties are **L1, not quadratic**. An L1 penalty is *exact*: above a finite weight, the solution is identical to the hard-constrained one, so the slack sits at exactly zero whenever the corridor is reachable. A quadratic penalty always trades a little violation for a little objective, and would quietly fly just outside the corridor forever.

### A wall-clock budget, not an iteration cap

Textbook SCvx runs to a tolerance. Running inside a game's simulation thread, a solve that takes a second is a visible freeze.

An iteration cap does not bound time - one ADMM iteration costs several times more at 80 nodes than at 30. So the cap is derived each cycle from a *measured* cost per iteration against a wall-clock budget, with a larger escalated budget for the rare hard subproblem.

The flight tolerance is also far looser than the validation tolerance. Measured across a descent, `1e-5`, `1e-4` and `1e-3` produce the same trajectory at very different cost; the tail of the ADMM iteration is almost the entire bill and buys nothing.

### A defect gate in metres

Whether a plan is flyable is an absolute question - a 10 cm discrepancy over a 20 second trajectory is fine whether the target is 2 km away or 50 m. But the solver's internal defect is normalised by the problem scale, whose length is the *range to target*, so a fixed scaled tolerance silently tightens as the vehicle closes in.

The flight gate is therefore expressed in **metres**. The solver's own convergence test still uses the scaled figure, where it belongs.

### A node ladder

Node count steps down at fixed altitudes rather than being held constant. Since spacing is `sigma/(N-1)` and `sigma` shrinks on approach, a fixed count buys steadily finer resolution than the problem needs.

It is a ladder rather than a continuously tracked value because changing `N` changes the problem's dimensions: the sparsity pattern is frozen at construction, so a new count means a new solver and the loss of the warm start. The reference *trajectory* survives by interpolation onto the new node count, and that is the seed that matters - measured 5-90x cheaper than a cold solve at the transition.

### Hand-off to a hover controller

The last few metres are the worst case for this solver and the easiest for a simple controller. The horizon has collapsed to a handful of nodes over a second or two, so the trust region rather than the physics is what binds; meanwhile the terminal state - near-zero velocity, upright, holding a point - is exactly what a hover PID is for. Optimising a descent is the wrong question by then, so the guidance hands over.

---

## 4. Code map

| File | Role |
|---|---|
| `AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Dynamics6Dof.cs` | The nonlinear dynamics and their Jacobians |
| `AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofSubproblemScs.cs` | Builds one cone program per iteration |
| `AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofSolver.cs` | The SCvx loop: linearise, solve, ratio test, trust region |
| `AdvancedFlightComputer/Features/Guidance/Conic/ScsWorkspace.cs` | SCS bindings and iterate-level warm starting |
| `AdvancedFlightComputer/Features/Guidance/Adapters/KsaFrameBridge.cs` | Frame, body-axis and quaternion conversions |
| `AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSetup.cs` | Measures the live vehicle into a solver configuration |
| `AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofGuidance.cs` | The MPC loop and the command interface |
| `AdvancedFlightComputer/Features/Guidance/Control/TvcAllocator.cs` | Body torque -> per-gimbal deflections |
| `AdvancedFlightComputer/Features/Guidance/Control/EnginePerf.cs` | Engine capability and the thrust-curve inversion |
| `AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs` | UI, stepping, telemetry, hand-off |

### Validation

`tests/AdvancedFlightComputer.Guidance.Tests/Scvx` runs the solver headless against a JAX reference implementation and a set of behavioural checks - closed-loop MPC with injected dispersions, path-constraint enforcement, node-ladder transitions, defect behaviour against range, and solve-time distributions. `dotnet run --project tests/AdvancedFlightComputer.Guidance.Tests/Scvx -c Release -- --help` lists them.

In flight, `SixDofLog` writes per-cycle telemetry, periodic whole-plan snapshots and an event log; `tools/readlog.py` summarises a run.
