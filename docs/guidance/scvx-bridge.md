# Touchdown guidance on the craft

This page covers the **integration bridge** between the 6-DOF SCvx solver and a live vehicle in KSA: coordinate conversions, reading controls from the published plan, converting torque and thrust into actuator commands, and running the asynchronous receding-horizon worker loop.

- For the mathematical problem, constraints, scaling, `vc-rv` virtual control, two-grid schedule (`N = 25` to `N = 15`), defect gates, and safety guards, see [Touchdown (6-DOF)](scvx/touchdown.md).
- For the shared persistent `ClarabelWorkspace` and SCvx machinery, see [Successive convexification](scvx/index.md).
- For historical engineering lessons learned during initial flight bring-up, see [Dev notes](scvx-bridge-devnotes.md).

## 1. Frame conventions (`KsaFrameBridge`)

The 6-DOF solver operates in a **site frame** with origin at the landing target and `+z` up, treated as inertial during each solve.
KSA works in body-centred inertial (CCI) with scalar-last quaternions, while the solver uses scalar-first quaternions.
`KsaFrameBridge` converts states, vectors and quaternions in both directions through rotation matrices so neither convention can be misread.

Model body axes are derived from the measured thrust direction (`+z` along thrust) so any vehicle assembly orientation works.
A round-trip check (convert out, convert back, compare) is exposed in diagnostics to catch any axis swap, handedness error, or sign flip.

## 2. From plan to actuator commands

Every simulation step, `Ksa6DofGuidance.Command` evaluates the latest accepted plan at $t = t_{\text{now}} - t_{\text{solve}}$ by linear interpolation between nodes, matching the solver's first-order-hold collocation:

- **Attitude feedback between re-solves (`AttitudeFeedback6Dof`):** adds a closed-loop torque correction $\boldsymbol\tau_{\text{fb}}$ against the plan's attitude and rate at the same instant, sharing the correction across gimbals and RCS in proportion to their remaining room (`SixDofTorqueMargin`, 20 % reserve).
- **Gimbal torque (`KsaTvcAllocator` + `KsaGimbalControl`):** converts the model's lateral thrust and coupled off-centre roll into body torque $\boldsymbol\tau_{\text{gim}}$ and solves the circular-cone TVC allocation ($\sqrt{u_{c_0}^2 + u_{c_1}^2} \le 1$) for per-gimbal deflections on the vehicle worker.
- **Engine throttle (`EngineThrustTable`):** the solver outputs site-reference axial thrust in newtons, not throttle fraction. Because ambient back pressure $-P_a A_e$ causes a non-linear low-throttle deficit, `Ksa6DofGuidance.Command` inverts the shared differentiable 2D cubic B-spline `EngineThrustTable(u, P)` at live ambient pressure so commanded newtons match delivered thrust (matching `Dynamics6Dof.F` and eliminating 3-DOF to 6-DOF hand-over thrust jumps).
- **RCS and fins:** plan RCS torques route through `KsaRcsTorqueControl` (LP allocation + pulse modulation), and plan fin deflections route fin-by-fin through `GuidanceWindow.FlyPlanFins`.

## 3. Asynchronous receding-horizon loop (`Ksa6DofSolveWorker`)

Touchdown guidance never flies open loop. Every `0.1 s` (`10 Hz`), the main thread publishes a fresh state snapshot and dispatches work to the background `Ksa6DofSolveWorker`:

1. **Async cold start (`Ksa6DofJob.InitCold`):** `StartSixDofStandby` reads immutable vehicle/body properties on the main thread, clones mutable target arrays, and dispatches `Ksa6DofGuidance` construction and `BeginCold` to the worker thread so standby engagement never hitches the main simulation thread.
2. **Warm re-solve:** resamples the previous plan onto the shrinking horizon (cubic Hermite for position/velocity, linear for the rest), smoothly blends $(q, \boldsymbol\omega)$ onto the measured state via `ColdReferenceShift`, refreshes live inertia, closed-form 2-term `MomentLimiter`, `EngineThrustTable`, and offset-free acceleration bias, and solves through persistent `ClarabelWorkspace` (`IScvxConicWorkspace`, `SCS` optional).
3. **Publication gate:** a candidate plan replaces the active plan only if its nonlinear defects pass the per-channel SI defect gate (`MaxDefectM = 1 m` in the commit horizon) and its anchor offset is within `1 m`. Refused cycles keep the previous plan; with no live plan, the no-plan stopping-curve law flies above `SixDofLostPlanGateM` (100 m) and a hoverable craft hands over to hover below it.

## 4. Code map

| File | Role |
| --- | --- |
| [Dynamics6Dof.cs](../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Dynamics6Dof.cs) | Nonlinear 6-DOF dynamics, `EngineThrustTable` back-pressure parity, dual-number Jacobians |
| [BodyAero6Dof.cs](../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/BodyAero6Dof.cs) | Body-axis aero tables, fins, rotational damping, closed-form 2-term `MomentLimiter` |
| [Scvx6DofSubproblemScs.cs](../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofSubproblemScs.cs) | Frozen-sparsity cone program assembly (`CcsAssembler`) |
| [Scvx6DofSolver.cs](../../AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/Scvx6DofSolver.cs) | SCvx loop, merit, trust region, quaternion normalization |
| [ClarabelWorkspace.cs](../../AdvancedFlightComputer/Features/Guidance/Conic/ClarabelWorkspace.cs) | Persistent native Clarabel workspace (`IScvxConicWorkspace`) |
| [ScsWorkspace.cs](../../AdvancedFlightComputer/Features/Guidance/Conic/ScsWorkspace.cs) | Optional SCS backend behind `IScvxConicWorkspace` |
| [KsaFrameBridge.cs](../../AdvancedFlightComputer/Features/Guidance/Adapters/KsaFrameBridge.cs) | Site/CCI frame, body-axis and quaternion conversions |
| [Ksa6DofSetup.cs](../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSetup.cs) | Measures live vehicle properties into solver config and `GuidanceModelSnapshot` |
| [Ksa6DofGuidance.cs](../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofGuidance.cs) | MPC loop, `vc-rv` pricing stages, `Command` actuator boundary |
| [Ksa6DofSolveWorker.cs](../../AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofSolveWorker.cs) | Background worker thread (`InitCold`, warm re-solves) |
| [TvcAllocator.cs](../../AdvancedFlightComputer/Features/Guidance/Control/TvcAllocator.cs) | Circular-cone body torque to per-gimbal deflections |
| [SixDof.cs](../../AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs) | Mode driver, two-grid transitions, hover-aware climb-cut and safety guards |
| [SixDofStandby.cs](../../AdvancedFlightComputer/Features/Guidance/Modes/SixDofStandby.cs) | Asynchronous background standby pre-planning before hand-over |
