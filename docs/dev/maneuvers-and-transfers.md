# Maneuvers, transfers and burn planning

AFC extends KSA's stock orbital planning with six subsystems under `AdvancedFlightComputer/Features/`: quick-tools (`ManeuverTools`), multi-pass burn splitting (`MultiPass`), flyby B-plane targeting (`Flyby`), porkchop transfers and moon course corrections (`Transfer`), lunar launch-window geometry (`MissionPlanner`), hyperbolic comet support (`HyperbolicTargets`), and pre-burn turn-warp / pointing diagnostics (`Burns`).

## Quick-tool solvers (`ManeuverTools`)

`OrbitManeuvers` computes impulsive maneuvers returning `ManeuverResult(DvCci, DvVlf, BurnTime)` and refines them for finite burn duration via `FiniteBurns.Refine`:

- **Apse and circularization tools:** `ComputeSetPeriapsis` / `ComputeSetApoapsis` apply vis-viva at the opposite apse on bound orbits; `ComputeCircularize` delegates to `OrbitalTransfers.DvCciToCircularize` for `Eccentricity >= 0.001`.
- **Plane changes:** `ComputeMatchInclination` and `ComputeSetInclination` apply a speed-preserving quaternion rotation (`QuaternionEx.AngleAxis`) at AN/DN (`relInc >= 0.001 rad`).
- **Burn chaining (`PlanningBasis.For`):** anchors new single burns onto the post-burn conic of `BurnPlan.FindFinalNontrivialBurn()`, clamping `Earliest` to `max(finalBurn.ImpulseTime, patch.ConicStartTime)`.
- **Orbit right-click integration (`Patch_BurnContextMenu_Launcher`):** a Harmony transpiler on `BurnContextMenu.Draw()` injects the AFC launcher inline and into the `"At Periapsis"` / `"At Apoapsis"` submenus.

## Multi-pass burn splitting (`MultiPass`)

`Splitter` splits any maneuver (`MaxPasses = 10`, `EqualDv` or `EqualBurnTime`) across `SequenceDrain` (multi-stage Tsiolkovsky propellant accounting) without exceeding available propellant.
Steering loss for a constant-inertial-attitude burn spanning orbital fraction $r$ is estimated by the Robbins sinc formula:

$$
\text{loss}(r) = 1 - \frac{\sin(\pi r)}{\pi r} \in [0, 1].
$$

### Forward-chain vs. Hohmann K-schedule

- **Apse and plane-change intents (`MultiPassForwardChainPlanner`):** iteratively solves each pass via `PassStepFactory`, refines finite burn duration, propagates a detached `FlightPlan` (8 patches, order 8), aborts on escape/encounter/impact, and advances `earliestTime = max(burnTime + 1 s, burnPatch.ConicStartTime)`.
- **Transfers (`HohmannMultiPassPlanner`):**
  1. **Real K-schedule (`BuildRealKSchedule`):** computes intermediate orbital period ratios $K_k = T_k / T_{\text{park}}$ from cumulative periapsis speed, clamped below the SOI ceiling $a_{\max} = (r_p + 0.95\,R_{\text{SOI}}) / 2$.
  2. **Integer-period phasing (`BuildSchedule`, `AdjustLastPeriod`):** rounds $\sum K_k$ to an integer number of parking periods so the final departure lands on the target window $T_{\text{final}}$, absorbing rounding into $K_{\text{priors}-1}$ while enforcing $K_0 > 1.01$, strict monotonicity $K_k > K_{k-1}$, and typed `PassPlanFailure` diagnostics (`FuelShort`, `SoiCeiling`, `ParabolicVp`, `TimeBudget`, `KFloor`, `NonMonotonicK`).
  3. **Departure shifting (`PrepareShiftedInput`):** for same-parent transfers where $T_{\text{final}}$ is too close (`< now + kTotal * T_park + 30 s`), shifts departure forward by integer parking periods and scans `64` Lambert flight times for a clean minimum-delta-v window.
  4. **Out-of-plane allocation (`AllocateAbsoluteThetaSchedule`):** allocates absolute plane-rotation shares across passes with zero radial delta-v on prior passes to preserve periapsis direction.
- **Execution (`PassCompletionPatch`, `MultiPassRegistry`):** ticked in `SharedVehicleHooks.TickVehicles` and notified on `FlightComputer.RaisePendingAlerts`. Re-engages Auto on subsequent passes, handles post-load burn reconciliation and `MaxAwaitingMaterializationTicks = 4`, cancels cleanly on external delete or zero usable propellant, and persists active runs atomically in `<LocalMods>/AdvancedFlightComputer/multipass.toml`.

## Flyby targeting and moon course corrections (`Flyby` and `Transfer`)

### Closed-form B-plane and patched-conic refinement (`FlybyTargeting`)

For a target periapsis radius $r_p$ (`Surface`, `Center`, or `Atmosphere`) and approach speed $v_{\text{SOI}}$, the B-plane impact parameter is:

$$
v_p^2 = v_{\text{SOI}}^2 - \frac{2\mu_{\text{target}}}{r_{\text{SOI}}} + \frac{2\mu_{\text{target}}}{r_p},
\qquad
b = \frac{r_p\,v_p}{v_{\text{SOI}}}.
$$

`SolveOffsetTransfer` seeds from a center-aimed `SuperiorLambert` transfer, runs `3` fixed-point iterations updating closest approach time, $v_{\text{SOI}}$, and aim point $\mathbf r_{\text{aim}} = \mathbf r_{\text{target}} + b\,\hat{\mathbf d}_{\text{side}}$ (`Inner`, `Outer`, `North`, `South`, rejecting sides with alignment `< 0.15`), and then runs up to `4` `PeriapsisCorrections` against `PatchedPeriapsis` (60-step SOI entry bisection) to remove the ~6 % patched-conic hand-off bias.
`HohmannFlybyUI.BuildPreview` propagates the finite burn and blocks `CREATE` if predicted periapsis drops below `MinFlybyRadius`.

### Porkchop sessions and moon course corrections (`Transfer`)

- **Porkchop grid (`TransferSession`, `PorkchopScores`):** evaluates a `292 x 292` Lambert grid in parallel without propagating full trajectories, flagging analytical `DepartureImpact` via `PassesPeriapsisBelow`.
- **Moon injection (`MoonInjectionPlanner`):** coarse Lambert scan (`1.5` rev, `144 x 24`) -> fine refinement of top `6` candidates (`16 x 16`, rejecting `> 1.25 * HohmannDv`) -> secant periapsis aim (`max(2 km, 3 % alt)`).
- **Mid-course corrections (`CourseCorrector`, `FlybyCourse`):** after a moon flyby departure completes and settles (`30 s` / `30` steps), checks whether periapsis is within `max(10 km, 10 % alt)`. If an encounter exists, `TryNudge` computes a central-difference gradient (`0.05 m/s` step) of hyperbolic impact parameter $b = r_p\sqrt{(e+1)/(e-1)}$ in VLF axes plus up to `6` secant steps (capped at `250 m/s`, `>= 3600 s` flight time); otherwise falls back to `TryAim`. `FlybyCourse` schedules up to `2` Manual correction nodes across candidate leads (`1800 s`, 25/50/75 %, late trims 87.5/93.75/96.875 %).

## Lunar launch geometry (`MissionPlanner`)

`LunarTransferGeometry.TryNodesForInclination` solves $\hat{\mathbf n}(i, \Omega)\cdot\hat{\mathbf u} = 0$ ($A\sin\Omega + B\cos\Omega = C$) in closed form for the two orbital planes of inclination $i \ge \lvert\delta(\hat{\mathbf u})\rvert$ containing the moon arrival unit vector $\hat{\mathbf u}$.
`LunarLaunchPlanner` pairs this with `GuidanceWindow.TryLaunchWindow` (`InsertionAfterIgnitionS = 600 s`): `InPlane` targets the moon's own plane when $\lvert\text{siteLat}\rvert \le \text{MoonMaxDeclinationDeg}$; `SoonestArrival` steps forward up to `2` moon periods (200 steps) and bisects (`24` steps, `1 s` tolerance) until the parking coast holds at least one full parking orbit.

## Hyperbolic targets (`HyperbolicTargets`)

`Content/HyperbolicBodies.xml` supplies mass and SOI for interstellar comets (`Oumuamua`, `2I/Borisov`, `3I/ATLAS`).
Six Harmony patches (`HyperbolicTargets.ApplyPatches`) adapt stock's bound-orbit assumptions (`Orbit.IsBound() == false`):

1. `Patch_PopulateWithPlanets`: appends same-frame unbound targets with valid SOI.
2. `Patch_HohmannFlight`: substitutes `Periapsis` for `SemiMajorAxis` (which is `NaN` on hyperbolas).
3. `Patch_SetTransferInfo` (finalizer): catches `ArgumentException` from subtracting `NaN` period and sets `Min/MaxTransferTimeOfFlight = Hohmann * [0.3, 4.0]`.
4. `Patch_AlignmentTime`: places ideal departure at `max(TimeAtPeriapsis - HohmannToF, startTime)`.
5. `Patch_TryFindIntercept`: skips stock's +/-5 % delta-v sweep and measures closest approach via 2-pass sampling (`86400 s` coarse, `60 s` fine).
6. `Patch_FindClosestApproaches`: skips encounter searches against unbound second bodies on finite patches to avoid `NaN` period faults.

## Turn-warp and pointing diagnostics (`Burns`)

- **`TurnWarp`:** warps uncapped to `AlignmentStartTime - 30 s` (`WarpLeadSeconds`) whenever the turn is more than `90 s` away (`30 + 60`) and stock AUTO is armed. `TurnWarp.Guard()` cancels only its own warp (`Universe.AutoWarpTime == owned.End`) if the target changes or alignment drifts within `15 s`.
- **`BurnPointing` and `BurnTurnRules`:** models stock's RCS switching-line dead zone (`RestingStallEdge = 0.5 * deadband + max(rateBit^2 / authority, 10 * rateBit)`) and caches `ComputeTvcTrackSettleDuration` to classify pre-burn cautions in priority order: `RcsOff` -> `NoRcs` -> `CannotTurn` -> `StallBand` (`>= 5 deg`) -> `ShortBurn`.

## Code map

| File | Main type or method | Role |
| --- | --- | --- |
| [OrbitManeuvers.cs](../../AdvancedFlightComputer/Features/ManeuverTools/OrbitManeuvers.cs) | `ComputeSetPeriapsis`, `ComputeCircularize`, `ComputeSetInclination` | Impulsive quick-tool solvers and finite-burn refinement |
| [HohmannMultiPassPlanner.cs](../../AdvancedFlightComputer/Features/MultiPass/HohmannMultiPassPlanner.cs) | `BuildRealKSchedule`, `BuildSchedule`, `PrepareShiftedInput` | K-schedule phasing, SOI clamping, departure shifting |
| [MultiPassForwardChainPlanner.cs](../../AdvancedFlightComputer/Features/MultiPass/MultiPassForwardChainPlanner.cs) | `Plan` | Detached flight-plan forward chaining for apse/plane splits |
| [FlybyTargeting.cs](../../AdvancedFlightComputer/Features/Flyby/FlybyTargeting.cs) | `ImpactParameterForPeriapsis`, `SolveOffsetTransfer` | B-plane targeting and patched-conic periapsis refinement |
| [MoonInjectionPlanner.cs](../../AdvancedFlightComputer/Features/Transfer/MoonInjectionPlanner.cs) | `TryPlan`, `TryPlanCorrection` | Lunar Lambert grid search and VLF impact-parameter nudging |
| [CourseCorrector.cs](../../AdvancedFlightComputer/Features/Transfer/CourseCorrector.cs) | `CourseCorrector`, `FlybyCourse` | Post-departure settle check and automatic correction scheduling |
| [LunarLaunchPlanner.cs](../../AdvancedFlightComputer/Features/MissionPlanner/LunarLaunchPlanner.cs) | `LunarTransferGeometry`, `LunarLaunchPlanner` | Closed-form launch-plane nodes and soonest-arrival bisection |
| [HyperbolicTargets.cs](../../AdvancedFlightComputer/Features/HyperbolicTargets/HyperbolicTargets.cs) | `ApplyPatches` | Six Harmony patches enabling unbound comet transfers |
| [TurnWarp.cs](../../AdvancedFlightComputer/Features/Burns/TurnWarp.cs) | `TurnWarp`, `BurnPointing` | Pre-turn warp ownership and RCS/TVC pointing diagnostics |
