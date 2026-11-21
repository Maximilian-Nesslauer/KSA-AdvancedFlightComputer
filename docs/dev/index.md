# Developer guide

This section explains how AFC works inside, for contributors and for anyone who wants to know why the craft does what it does.
The [User Guide](../user-guide/index.md) explains what each feature does from the player's side. Start there if a term here is new.

## How the code is laid out

Every feature lives in its own folder under `AdvancedFlightComputer/Features/`.

| Folder | What it does |
| --- | --- |
| `Ui` | The AFC console: its pages, the canvas pictures and the pure console rules. |
| `ManeuverTools` | The quick-tools on **MANEUVER**: circularize, set Pe/Ap, match and set inclination. |
| `MultiPass` | Splits a burn into passes near periapsis and flies them one after another. |
| `Transfer` | The **TRANSFER** page: porkchop plot, transfer commit, moon injection and course correction. |
| `Flyby` | Aims a transfer at a chosen flyby periapsis instead of the body centre. |
| `HyperbolicTargets` | Lets the planners target bodies on unbound orbits, such as interstellar comets. |
| `MissionPlanner` | The lunar launch planning behind **THEN** on **LAUNCH**: the launch plane and window. |
| `RcsTranslation` | Flies a planned burn with RCS thrusters only. |
| `RcsTorque` | Shares an attitude torque out among the RCS thrusters. |
| `AeroControl` | Shares an attitude torque out among the control surfaces (fins). |
| `AutoStage` | Automatic staging, the spent-stage drop and the staging delays. |
| `AutoRemove` | Removes the last finished burn from the plan. |
| `Guidance` | Powered guidance: ascent, boostback, entry and landing. See [AFC guidance](../guidance/README.md). |

Every write AFC makes to a craft's controls goes through one set of rules, described in [Control ownership](../control-ownership.md).

## The guidance chain

Each box is a guidance mode. An arrow is a hand-over from one mode to the next.

```mermaid
flowchart TD
    pad([On the pad]) --> ascent[Ascent<br/>convex plan + UPFG]
    ascent --> orbit([Orbit])
    ascent -- booster separates --> boostback[Boostback burn]
    boostback --> glide[Boostback glide]
    glide --> entry[Entry guidance<br/>3-DOF glide + burn]
    orbit --> deorbit[Deorbit burn]
    orbit -- THEN: a moon --> moon[Moon injection<br/>and corrections]
    deorbit --> touchdown[Touchdown guidance<br/>6-DOF SCvx]
    entry --> touchdown
    touchdown -- no plan for 6 s --> gfold[G-FOLD<br/>point-mass fallback]
    touchdown --> hover[Terminal hover]
    gfold --> hover
    touchdown --> ground([Landed])
    hover --> ground
```

| Solver | Where it is written up |
| --- | --- |
| What all three SCvx planners share | [Successive convexification](../guidance/scvx/index.md) |
| Convex ascent (SCvx) | [Convex ascent](../guidance/scvx/ascent.md) |
| Entry guidance, 3-DOF SCvx (glide and burn) | [Entry (3-DOF)](../guidance/scvx/entry.md) |
| Touchdown guidance, 6-DOF SCvx | [Touchdown (6-DOF)](../guidance/scvx/touchdown.md), [on the craft](../guidance/scvx-bridge.md) and the [dev notes](../guidance/scvx-bridge-devnotes.md) |
| G-FOLD, the point-mass landing | [G-FOLD](../guidance/gfold.md) |
| Impact prediction and boostback steering | [Impact steering](../guidance/reference-material/impact-steering.md) |
| The attitude controller and the gimbal, RCS and fin mixers | [Attitude control](../guidance/attitude/index.md) |

## Still to write

These parts have no page yet. Each one will follow the same layers: what it is for, a diagram, something to try, the maths, then where the code is.

- UPFG and the hand-over from the convex ascent
- Boostback and the returnable-stage reserve
- Landing engine groups and the landing legs
- AutoStage and the spent-stage drop
- Multi-pass burns, flyby targeting and the mission planner
- The AFC console: pages, commits and the stop
- The HeadlessHarness tests in `tests/AdvancedFlightComputer.HarnessTests`
