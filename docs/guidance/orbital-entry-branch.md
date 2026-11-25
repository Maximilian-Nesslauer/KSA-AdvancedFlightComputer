# The `feature/orbital-entry` branch

A summary of the work on `feature/orbital-entry` (2026-11-21 to 2026-11-24, status updated after the first landing) and how it relates to the open GitHub issues. The detailed record of each change, with flight and harness evidence, is the [Unreleased] section of [CHANGELOG.md](CHANGELOG.md). The bellyflop design is in [bellyflop-design.md](bellyflop-design.md).

## Purpose and status

The branch builds a Starship-style flight chain from orbit to the pad: orbital entry guidance flies a belly-first entry on an energy-domain planner inside heating, dynamic-pressure and load limits; BELLYFLOP flies the belly-first terminal descent and aims short of the site; touchdown guidance (6-DOF SCvx) takes the craft for the flip and the landing burn.

What works: the aimed deorbit, the entry and bellyflop. In the harness the KSAero "flopper" flies from 100 km to 100 m over the site; with BELLYFLOP AIM SHORT at 200 m it ends 216 m from the site and 11-20 m from bellyflop's aim (f09d793). The entry holds a heating limit to within a few percent where the site allows it (261b71d). A Starship (KSAStarship) has flown the entry and bellyflop in windowed flights and reached touchdown guidance.

First landing (2026-11-24, windowed): a Starship engaged from a bellyflop save with BELLYFLOP DIRECT, landing on a one-engine landing group (the centre sea-level Raptor), lit at 510 m, flipped under power and touched down at 1.6 m/s, 1.9 m from the target, with no refused re-solves. Every earlier flight lit all three sea-level Raptors, whose throttle floor is 2.3 times the craft's weight. Touchdown could then light only about 100 m up and had to flip in the coast on the air and flaps, which broke its plan. ORBITAL ENTRY did not apply the LAND landing group until 1c1f49e.

Not yet flown: the whole chain from orbit to touchdown in one flight, a landing on all three engines, and the harness checks after the touchdown pricing changes (7e401d6 onwards).

Branch state: up to date with `origin/main` through #306 (merges 2a72baa, 230b092 and 7697c62), so it builds for KSA 2026.11.25.5949 and no longer for 5901. Harness on 5949 (2026-11-25, harness 1.2.2): the Moon retarget passes; the flopper and the booster landing fail as before; the suite has 109 pass and 21 fail, five of them new on 5949 in code the branch did not touch (flyby course, ascent arg. Pe, booster aero, moon chain, RCS attitude yield), not yet checked against a main baseline.

The LAND page rework (#303) and the entry's retro burn, parachutes and Lambert-aimed burn (2026-11-25) are built and offline-checked but not flown.

## The flight chain

| Phase | Mode | What it does | Hands over when |
| --- | --- | --- | --- |
| Deorbit | DEORBIT ONLY with AIM AT SITE | Times the deorbit burn so the site sits mid-corridor of the coming entry, also with ENTRY AFTER DEORBIT off. | At burn end, if ENTRY AFTER DEORBIT is on. |
| Entry | ORBITAL ENTRY | Plans vertical L/D against energy (SCvx, Clarabel QP per iteration), flies it with a Shuttle-style drag and sink-rate tracker, bank reversals on the heading error to the site, angle of attack from a trim schedule. | To bellyflop at bellyflop's first converged plan from BELLYFLOP FROM MACH (10); HAND-OVER MACH (5) is the latest. |
| Terminal descent | BELLYFLOP (last phase of ORBITAL ENTRY) | 3-DOF SCvx in belly axes (pitch, bank, yaw about the airflow), free flight time, falling onto a point BELLYFLOP AIM SHORT (200 m) short of the site. | To touchdown 1 s before touchdown's standby plan turns more than 5 deg from the attitude flown, at the latest 5 s before its ignition or 12 s before bellyflop's ground. |
| Flip and landing burn | Touchdown guidance (6-DOF SCvx, Clarabel) | Solves on standby from TOUCHDOWN FROM (45 s before bellyflop's ground), then plans the coast, the flip on the whole-sphere aero, and the burn to the pad. | Lands (not achieved yet). |

BELLYFLOP DIRECT (21fa816): ORBITAL ENTRY engaged past the hand-over, for example from a save in the low air, skips the entry and starts at bellyflop. It is for testing the flip and landing without flying the entry.

### Main files

Where a cell lists several paths, the later ones are relative to the first one's feature folder (`AdvancedFlightComputer/Features/Guidance/` or `AdvancedFlightComputer/Features/`).

| Path | Role |
| --- | --- |
| `AdvancedFlightComputer/Features/Guidance/Modes/OrbitalEntry.cs`, `OrbitalEntryState.cs` | Orbital entry mode: phases (SWEEP, ENTRY, HOLD, BELLYFLOP), re-plans, tracker, hand-over to bellyflop. |
| `AdvancedFlightComputer/Features/Guidance/Modes/OrbitalEntryAim.cs`, `Modes/DeorbitPlanner.cs`, `Modes/StockDeorbit.cs` | Site-aimed DEORBIT ONLY: pass search, corridor, burn time; hand-over to orbital entry at burn end. |
| `AdvancedFlightComputer/Features/Guidance/Scvx/OrbitalEntry/*` | Game-independent entry planner: `EntryModel`, `EntryDynamics`, `EntryPlanner`, `EntryPlan`, `EntryTracker`, `EntryTrim`, `EntryConfig`. |
| `AdvancedFlightComputer/Features/Guidance/Modes/Bellyflop.cs` | Bellyflop mode, its attitude command, and the hand-over to touchdown (`StepBellyflopTouchdown`). |
| `AdvancedFlightComputer/Features/Guidance/Scvx/Bellyflop/*`, `Adapters/Scvx/Bellyflop/KsaBellyflopSetup.cs` | Bellyflop planner (`BellyflopPlanner`, `BellyflopModel`, `BellyflopPlan`, `BellyflopConfig`) and its game setup. |
| `AdvancedFlightComputer/Features/Guidance/Modes/SixDof.cs`, `Modes/SixDofStandby.cs` | Touchdown guidance's flying loop and standby; take-over rules. |
| `AdvancedFlightComputer/Features/Guidance/Adapters/Scvx/SixDof/Ksa6DofGuidance.cs`, `Ksa6DofSetup.cs`, `Ksa6DofInputs.cs` | Touchdown solve cycle, re-solve seeding, virtual-control pricing, plan surface share, moment bias. |
| `AdvancedFlightComputer/Features/Guidance/Scvx/SixDof/BodyAero6Dof.cs`, `Dynamics6Dof.cs` | Touchdown's air model: blending of engine-first, flip-band, belly-first and whole-sphere tables. |
| `AdvancedFlightComputer/Features/Guidance/Adapters/KsaAeroSweep.cs`, `Numerics/Flight/BodyAeroTable.cs` | Aero sweep: belly-first, whole-sphere and flip-band tables; `TryPickTables`; `BodyAeroFrame`. |
| `AdvancedFlightComputer/Features/Guidance/Control/AttitudeController.cs` | Authority-aware capture, per-axis ownership between RCS and surfaces, RCS trim moved onto the flaps, rate filter. |
| `AdvancedFlightComputer/Features/RcsTorque/PhasePlaneRcs.cs`, `RcsTorque/KsaRcsTorqueControl.cs`, `AeroControl/KsaAeroTorqueControl.cs`, `RcsTranslation/RcsLpSolver.cs` | Phase-plane RCS, RCS and flap allocation, the RCS LP's final-check fix. |
| `AdvancedFlightComputer/Features/Guidance/Adapters/OrbitalEntryLog.cs` | `<run>-orbital-entry.csv` and `<run>-orbital-entry-plans.csv`. |
| `AdvancedFlightComputer/Features/Guidance/Ui/Gauges/EntryTrajWindow.cs`, `Ui/EditorBalanceWindow.cs`, `Ui/Console/OrbitalEntryMode.cs`, `Ui/Console/OrbitalEntryForm.cs`, `Ui/Console/LandForm.cs` | ENTRY TRAJ display, editor BELLY-FLOP BALANCE window, LAND > ENTRY console mode and form. |
| `AdvancedFlightComputer/Features/Ui/Logic/GuidanceSettingSpecs.cs` | The new settings and their tiers. |
| `tests/AdvancedFlightComputer.Guidance.Tests/Scvx/OrbitalEntryCheck.cs`, `BellyflopCheck.cs`, `PhasePlaneCheck.cs`, `RcsRefusalCheck.cs`, `SixDofTableBlendCheck.cs` | Offline checks. |
| `tests/AdvancedFlightComputer.HarnessTests/Features/Guidance/GuidanceFlopperEntryTest.cs` | Harness test `afc-guidance-flopper-entry`. |

## Changes by area

### Entry planner and tracker

- **Planner and tracker** (924012b): vertical L/D against energy to reach the site inside q, load and optional heating limits, turned into altitude floors, with the least heat load; re-solved every 2 s warm. Shuttle-style tracker; ENTRY TRAJ window.
- **Trim schedule** (674f475, bec10c8, 810e02b, ce8dc00): least-moment angle where no exact trim exists, AoA floor 40 deg and cap 70 deg (no crossrange near 90), re-trim with the flaps' reach once belly-first flap tables exist. From the first three flights, where the craft could not hold the angle asked.
- **Site-aimed deorbit** (e5cb884, bec10c8): the first entry flew past the site because DEORBIT ONLY burned at the next chance. The burn is now timed on a 24 h pass search so the site is mid-corridor, checked with one cold plan.
- **Plan quality** (cb9fa5a, 92df5d3): smoothing in the optimisation (curvature term, pull to the previous plan) instead of a tracker low-pass; corridor-margin term so the plan does not creep to the high-energy edge; feedback clamped to 35 % of L/D; gate 0.25.
- **Rotating body** (d46412f): planner and coast predictions carry Coriolis and centrifugal terms (Lu 2008). A shallow eastward coast had been predicted to reach the air about half as far as it did.
- **Tracker tuning** (233a946, e3f4fc3, 6a7f0cc, 8798e9b, aaf66e1, 451a775): bank reversal on the heading error to the site (5 deg band), overbank as ordinary tracking to 120 deg, attitude-rate feedforward, gains floored at 10 m/s² of drag and retuned from an offline sweep with measured drag noise, reversals at 5 deg/s, belly axis +Z.
- **Heating** (ffc410b, 1a5cd5a, 261b71d): MAX HEATING 440 kW/m² at a 1 m nose (Shuttle STS-2 reference, Sutton-Graves estimate). The floors are priced at a tenth of the dynamics so an unreachable limit no longer refuses every plan. LEAST HEATING shows the lowest peak the craft can fly and warns when the limit is below it. The game does not heat craft yet; the limit only shapes the trajectory.
- **Aim at bellyflop's reach** (1d97319): the entry prices its hand-over range only outside a band of bellyflop's reach, not on one point. Entry refusals in the harness went from 36-44 a flight to none.

### Bellyflop

- **New planner** (653fce9; design in [bellyflop-design.md](bellyflop-design.md)): 3-DOF SCvx in belly axes, RK4 shooting plus one Clarabel QP per iteration, solved in the background.
- **Reach and take-over** (6e269ef, be96385): own force scale measured against its own model (the entry's density scale read 1.9 once belly first), miss priced 1 per metre, bank 45 deg, pitch up 55 deg so it can take over at the entry's attitude; approach cone, 75 % authority reserve beyond the first 20 s, cheaper yaw while gliding; solve from Mach 10, take over by Mach 5.
- **Refusals in the harness** (c107cdb, 4197191, c3cb612): cold re-solve after three refusals, yaw within 90 deg of the bearing, defects priced ten times the miss, cone capped below one node's defect price, crossrange priced at every node, a refused plan flown when it lands nearer than the current one.
- **Aim short, bearing fix** (f09d793): BELLYFLOP AIM SHORT 200 m. Fixed the crossrange term, which priced the site frame's y axis instead of the distance off the bearing.
- **Command and bank** (9f1c656, 5081e60, d9023b3): attitude command low-passed, then a second-order filter (the first-order one rocked a Starship 33 deg in bank); bank limited to what the flaps can roll-trim on 60 % of their reach.
- **Lateral approach cone** (08c3c62): crossrange within tan 60 deg of the distance to go plus 25 m, so plans no longer pass the aim and swing back. Offline, closed loops went from 0.2-2.9 km past the site to 0-16 m, except two cases with more energy than they can lose (2.8 and 6.1 km past).
- **BELLYFLOP DIRECT** (21fa816): see above.

### Hand-over to touchdown

- **Bellyflop hands the flip to touchdown** (9f1c656): standby from TOUCHDOWN FROM (45 s), take-over at its first plan, engine off until touchdown lights it; refused only if full thrust cannot hold the craft up (70 % refused the flopper's 753 kN).
- **Seeds** (5081e60, d9023b3): the burn seed includes the flip (6 s); the standby is seeded along bellyflop's plan (BellyflopSeedPath), since a free seed planned its own coast attitude and every re-solve was refused.
- **Timing** (d9023b3, 4180bd6, 08c3c62): taking the craft at the first plan, 27 s before ignition, lost the belly-first attitude; taking it just before ignition let the re-solves drift and the standby went stale. Now: 1 s before touchdown's plan first turns 5 deg from the attitude flown, at the latest 5 s before ignition or 12 s before bellyflop's ground; a standby plan up to 2.5 s old is accepted. The turn-away rule applies to entry guidance's glide too.

### Touchdown model and aero

- **Whole-sphere aero** (9f1c656): past the engine-first table's 20 deg, touchdown had a roll-averaged drag and no moment, so a flip from belly first was planned on next to nothing. The sphere fills the rest.
- **One shared sweep** (08c3c62): a belly-flopper keeps a belly side and one sweep, shared by orbital entry and touchdown and kept between modes. Belly first, touchdown's model then matched the game's force within 1 % (harness).
- **Half the RCS and flaps** (21fa816): plans may use 50 % of the RCS torque and of the surfaces' travel (`Ksa6DofSetup.PlanSurfaceShare`), so turns that need more go to the gimbals with the engine lit. A Starship's coast flip on flaps and RCS at their limits fell 26 deg behind its plan and lit at 60 m at 46 m/s.
- **Flip band and moment error** (7f6bbaa): a fine table over the flip (5 by 7.5 deg, every surface deflection), the belly-first tables, and the sphere, blended with faded edges; mid-flip the old model credited the flaps three times what the game gave. A moment-error integrator (`Ksa6DofInputs.MomentBiasPerPa`) feeds the game's moment less the model's, per pascal, into the plan, as `AccelBias` does for force. Not flown.

### Touchdown solver and refusals

- **Carry on after a refusal** (0767fa7): the flying re-solve starts from the refused attempt's trajectory, shifted to now and re-anchored at the measured state, as the standby's Track does. Seeded from the published plan each cycle, a Starship's re-solves were refused 15 times in a row at 1.2-1.6 m against the 1 m gate. Not flown.
- **Virtual-control pricing** (7e401d6): 5 m position scale, near-term intervals priced tight on warm re-solves, no loosening after a gate refusal.
- **Flaps' tables first** (bc59fcd): touchdown waits for the sweep's control surfaces before it plans, flying on meanwhile; a direct engage had planned without flaps and stalled every half second.
- **PLAN RCS AND FLAPS** (bc59fcd, 49464a7): the plans' share of the RCS torque and the surfaces' travel is a setting, default 0.75 (was a fixed 0.5).

### Attitude control and RCS

- **Authority-aware capture** (8798e9b): square-root capture from the acceleration the surfaces and RCS can spare after trim, conditional integration while saturated, bank slew limited to what the craft can follow. Opt-in, used by orbital entry and bellyflop.
- **Cheaper reach measurement** (3a0ff62, 76e2954, 2eb2bfa): RCS reach measured once and again only on a 2 % mass or CoM change; the flaps' reach on a 5 % Mach or 2 deg airflow change, scaled by q in between. Six RCS LPs every half second of game time had run on the main thread.
- **Flap allocation** (7da82a2, 623b392, e688c43): a fix to flap-rate smoothing that held flaps to about 1 deg/s; a frequency split by reach that was later dropped (2.4 times the propellant); the ENTRY RCS switch (flaps alone when off).
- **Per-axis ownership and phase-plane RCS** (819cee3): each axis is owned by the surfaces or the RCS, as stock gives an axis to gimbals or RCS; on its own axes the RCS flies stock's phase plane (`PhasePlaneRcs`, from `FlightComputer.ComputeRcsTrackAxis`). Fixed `RcsLpSolver`'s final check, which rejected 1566 of 3735 reachable pure-axis demands on round-off.
- **Flaps take the steady work** (5081e60, 8ebf9c3): RCS DEAD BAND 4 deg for entry and bellyflop; surfaces take an axis on a fixed 1 deg need; the RCS's steady torque is integrated into the flaps' trim over 10 s. The RCS had held 80-117 kN m of steady pitch through the whole entry, 2 t of propellant by Mach 19.
- **Whole sphere for the controller** (d9023b3): twice a Starship banked past the belly-first tables once its RCS ran dry, the controller lost the air's moment and the craft tumbled.
- **Rate filter 0.05 s** (65d8cf1) for the glide, orbital entry and bellyflop (was 0.2 s; #291). Not flown.
- **GLIDE + BURN's glide** may fly this controller (NEW ATTITUDE CONTROL, 6e269ef). Not flown.

### Aero sweep

- Belly-first sweep over four candidate belly axes (924012b), then belly-first body and control-surface tables about a `BodyAeroFrame`, picked by `KsaAeroSweep.TryPickTables` for the controller and the fin allocator (b6280ab).
- Coverage widened from the flights: 10 to 110 deg from nose first (810e02b), 45 deg of sideslip (6e269ef), read at Mach 8 above Mach 8 (1d97319).
- Whole-sphere tables, 266 directions a Mach row for the body and 62 for each surface (9f1c656); one sweep shared and kept between modes (08c3c62); the fine flip band (7f6bbaa). Each addition lengthens the sweep.

### UI and settings

- ENTRY TRAJ window after the Shuttle's displays: paged range plot on fixed axes, guidance box, Mach axis, plots kept inside their frames (924012b, 94a839f, 233a946, 98cb019, e3f4fc3, 6a7f0cc).
- LAND as the path of modes (#303, 800d6b0 to 9bae2c2): tabs DEORBIT, BOOSTBACK, ENTRY/BRAKING, GLIDE, LAND NOW, HOVER, each starting the path at its step; the path built from each mode's declared steps and next mode (`GuidanceModeDescriptor.Steps`, `.Next`), each step a button for its settings, the flying step marked and followed; the GLIDE table (FLIP LANDING or RETRO BURN); the hand-over switches, the solver table and the AFTER DEORBIT and METHOD rows gone. GLIDE + BURN is RETRO BURN in the console. Before it: LAND > ENTRY with a METHOD row, ORBITAL ENTRY and GLIDE + BURN (the 3-DOF, renamed in the UI; setting keys unchanged). Orbital entry is a registered console mode; its settings are tiered across the page, HAND-OVERS, GNC, TUNE > LANDING > ORBITAL ENTRY, AFTER DEORBIT and DEORBIT's GNC; the ENTRY TRAJ window became a display-only pop-out (653fce9).
- Editor BELLY-FLOP BALANCE window: centre of pressure against centre of mass belly first, per angle of attack (c2c7b7a, ce8dc00).
- New settings include BELLYFLOP, BELLYFLOP FROM MACH, HAND-OVER MACH, BELLYFLOP PITCH UP/DOWN, BELLYFLOP BANK, BELLYFLOP AIM SHORT, BELLYFLOP TO TOUCHDOWN, TOUCHDOWN FROM, MAX HEATING, NOSE RADIUS, ENTRY RCS, RCS DEAD BAND, AIM AT SITE, CORRIDOR MARGIN and DRAG TRACKING.

### Tests and harness

- Offline: `--orbital-entry` (in CI), `--orbital-entry-gains`, `--bellyflop`, `--phase-plane`, `--rcs-refusal`, `--6dof-blend` (in CI). Details under [Testing](#testing).
- Harness: `afc-guidance-flopper-entry` (ae335a3, ec427f9, 72e42ff and later), opt-in.

### Diagnostics

- Orbital entry CSV logs every 0.5 s and per plan (674f475), with RCS ownership and firing columns (819cee3).
- Debug PerfTracker scopes through the autopilot step, controller, allocation and entry step; GC counts; any call over 50 ms (623b392). Release builds get a no-op scope.
- AERO CHECK: once a second while touchdown runs on a craft with whole-sphere tables, the model's force and moment (body and flaps apart) against the game's (08c3c62).
- LEAST HEATING in the status and log (261b71d); BELLYFLOP DIRECT logged (21fa816).

### Merged from main

2a72baa merges `origin/main` through #294: Clarabel SCvx (#261) and its follow-ups (#292), the deep-review fixes #274 to #278, and docs. Conflicts were resolved keeping both sides.

## Open problems and next steps

1. **Touchdown re-solves refused during the coast flip (addressed, 0767fa7 and 7e401d6).** The landing flight had no refused re-solves. Before: On the latest Starship flights the flying re-solves left 1-2 m position defects against the 1 m publish gate. This traces to loose virtual-control pricing: position is priced on XScale, velocity is priced tight only under 10 s to go (`PricingTightBelowS`), and repeated refusals loosen the price a further stage. 0767fa7 (carry on from the last attempt) is committed but not flown. 7e401d6 adds a 5 m position virtual-control scale, as proposed in #265, no loosening after gate refusals, and tight pricing on the near-term intervals the gate checks.
2. **Flap aero mid-flip.** Between 75 and 45 deg of tilt the air resisted the flip with 1.6-1.7 MN m where the model had 0.5-1.1 (AERO CHECK), and the old model credited the flaps three times what the game gave. The flip band and the moment-error integrator (7f6bbaa) are meant to close this; not flown.
3. **Timing margin.** The coast flip fell 26 deg behind its plan and lit at 60 m at 46 m/s. The 50 % RCS and flap planning share (21fa816) should leave margin for the gimbals; not flown. The standby's plans are often stale while bellyflop flies, so the hand-over accepts plans up to 2.5 s old (4180bd6).
4. **Bellyflop has no pitch moment in its planner**, so it can choose attitudes the flaps can barely hold (noted in 9f1c656). Not done.
5. **Entry with too much energy.** Two offline bellyflop closed loops that start with more energy than they can lose before the site still pass it (08c3c62).
6. **Heating limit.** The flopper cannot hold the default 440 kW/m²; the limit is soft and loses to the range, so the craft may fly hotter than the limit to reach the site. LEAST HEATING says so.
7. **Engines to land on.** The chain now lands on the LAND landing group (1c1f49e). Open: switching the number of engines in flight (flip on several, land on one, as the real Starship does), so a craft whose full set cannot hover can still light high. The hand-over checks only the active engines' thrust against weight.
8. **Merge with main (done, 230b092 and 7697c62).** The three conflicts in the RCS and aero torque controls and the attitude controller keep both sides; main's minimum-throttle exhaust velocity is applied to the aimed entry burn too.
9. **Retro burn after an entry.** Asked once at the entry's hand-over (Mach 5 at 38 km, about 6-7 kPa) and started cold; the entry aims RETRO AIM SHORT (30 km), a guess. A standby like the boostback's, with its prediction of the ground covered in place of the guess, is the next step. Not flown.
10. **Parachutes.** Armed at the hand-over for a craft with no engine to land on; KSA opens them at their own deploy altitude. Not flown.
11. **The Lambert-aimed deorbit burn.** Taken when within 25 % (10 m/s) of the horizontal burn and 0.5 deg of its flight path angle. Not flown.
12. **User guide.** `docs/user-guide/console.md`, `powered-guidance.md` and `docs/guidance/boostback.md` still describe the old LAND page (ENTRY BURN ONLY, ENTRY ON GLIDE, the solver table).

## Relation to open issues

| Issue | Title (short) | Relation | Notes |
| --- | --- | --- | --- |
| #219 | Aerodynamic entry from orbit | Partly addresses | Orbital entry guidance after a site-aimed DEORBIT ONLY, with heating, q and g limits, and a powered landing chain. Its done-when is not met: no landing at the site yet, and no parachute capsule flown (offline the capsule case ends 21-26 km off). |
| #241 | Touchdown re-solves refused in the engine-off coast | Partly addresses | The same failure is the Starship's current blocker. 0767fa7 and the in-progress pricing fix target it; neither has been checked on the booster case in the issue. |
| #265 | SCvx Clarabel tuning (warm position VC, AL-SCvx, two-grid) | Partly addresses (in progress) | The in-progress fix takes item 1, a 5 m position virtual-control scale, for touchdown. Items 2 to 4 are not touched. |
| #266 | 6-DOF standby engage hitch, 25 ms cold budget truncation | Touches the same code | Bellyflop drives the standby (`SixDofStandby.cs`, `Ksa6DofGuidance.cs`) with new seeds and plan-age rules, but does not fix the hitch or the cold budget. The truncation may contribute to the standby's refusals under bellyflop; not checked. |
| #272 | Mode state machine and hand-over guards | Touches the same code (merge risk) | Adds two modes and new hand-overs (entry to bellyflop, bellyflop to touchdown, the turn-away rule, BELLYFLOP DIRECT), which grows the chain the issue proposes to consolidate. Its four guard fixes are not done here. |
| #291 | AttitudeController rate filter, anti-windup, gimbal residual | Partly addresses | Rate filter 0.05 s for the glide, entry and bellyflop (65d8cf1; the issue suggests 0 to 0.04 s); conditional integration while saturated for entry and bellyflop (8798e9b). Gimbal residual hand-off and the finned-booster validation are not done. |
| #282 (closed) | RcsTorque and AeroControl races, leaks, allocations | Merge risk | Fixed on main by #295, not yet merged into the branch, which changed the same files heavily (phase plane, ownership, RCS trim onto flaps). |
| #267 | KSA 5901 aero fidelity (moment limiter, C_mq damping, sampling cache) | Works around; touches the same code | The moment-error integrator and AERO CHECK learn and show the model's moment error in flight, which would include missing damping, without modelling it. The sweep changes overlap with item 3's `KsaAeroSweep` sampling code. |
| #234 | Booster entry loses its plan in the glide | Mostly unrelated | Its cause, 3-DOF cold solves giving up, is not touched. The glide-to-touchdown take-over now also uses the turn-away rule (08c3c62), so re-check after a merge. |
| #196 | Lunar landing lost when the game runs slow | Unrelated | Same flying re-solve loop: 0767fa7 changes its seed after a refusal, which may help, but nothing here addresses wall-time iteration caps. |
| #103 | SCvx retarget robustness, terminal hover | Touches the same code | Item 1's refused re-solve cycles are close to what 0767fa7 changes (seed from the last attempt, re-anchored in time and state); not tested on a retarget. Item 2 is unrelated. |
| #186 | Cap the angle of attack at what the controls can hold | Partly addresses, for orbital entry and bellyflop | The entry's trim schedule uses the flaps' reach and caps AoA at 70 deg; bellyflop's bank is limited to the flaps' roll trim. The boostback glide and the 3-DOF entry are not touched. |
| #222 | Plan touchdown with control-surface torque | Partly addresses (extends #238) | Touchdown now plans a belly-flopper's flaps on the belly-first, flip-band and whole-sphere surface tables, on half their travel, with half the RCS. |
| #283 | Guidance lifecycle: StockDeorbit completion race and others | Touches the same code | The branch hands DEORBIT ONLY's burn end to orbital entry in `StockDeorbit.cs`; the race in item 1 would also stop that hand-over. Not fixed here. |
| #184 | Aero tables on KSA 5901, shorter sweep | Touches the same code | The branch adds belly-first, sphere and flip-band tables, which lengthen the sweep for belly-floppers; the issue's density and speed-of-sound items are not addressed. |
| #170 | Booster entry burn reserve under g, Q or heating limits | Unrelated | The branch's heating limit is orbital entry's only. |

Other open issues (#177, #245, #248 and the planner, ascent and UI issues) are unrelated to this branch.

## Testing

Offline checks (no game) are flags of the Scvx console in `tests/AdvancedFlightComputer.Guidance.Tests/Scvx`, run with `dotnet run --project tests/AdvancedFlightComputer.Guidance.Tests/Scvx -- <flag>`; `build/ci/test-guidance.ps1` runs the CI ones the same way with a configuration and runtime identifier.

| Flag | What it checks |
| --- | --- |
| `--orbital-entry` (CI) | Three vehicles (L/D 0.3 to 1.0) under 700 and 440 kW/m², air denser and L/D lower than planned; heating peaks above LEAST HEATING; table frames; sphere angles round-trip. |
| `--orbital-entry-gains` | Tracker gain sweep with measured drag noise. |
| `--bellyflop` | Hand-overs as flown (Mach 3 to 5, 10 to 20 km crossrange, force 15 % weak); the same fall along +x and +y; closed loops to the aim. |
| `--phase-plane` | Phase-plane RCS against the old laws on one axis and an entry axis with flaps, RCS and air torque. |
| `--rcs-refusal` | The logged 48-thruster RCS case, rebuilt and fuzzed (0 unsolved). |
| `--6dof-blend` (CI) | Touchdown's table shares sum to one over the sphere and blend smoothly along a flip. |
| `--6dof-aero`, `--6dof-fins` | Touchdown aero and fins unchanged for a craft without a sphere. |
| `--6dof-coast`, `--standby` | Touchdown's coast and standby checks (existing). |

UI rules are checked in `tests/AdvancedFlightComputer.UiLogic.Tests` (ConsolePagesTest).

Harness: `afc-guidance-flopper-entry` in `tests/AdvancedFlightComputer.HarnessTests` is opt-in and needs the content mods (`-ContentMods KSAero`, plus `KSAStarship` for a Starship; it skips with a hint without KSAero). It spawns the flopper, or AFC_FLOPPER_VEHICLE, on a set entry state, engages ORBITAL ENTRY as the console does, and flies at AFC_FLOPPER_PACE to 100 m over the site, or on to contact with AFC_FLOPPER_TOUCHDOWN. It checks that the entry engages, that bellyflop takes over in its Mach band, the miss (AFC_FLOPPER_MISS_M, AFC_FLOPPER_LAND_MISS_M), and the flown heating peak against AFC_FLOPPER_HEAT_KWM2. The start state is set with AFC_FLOPPER_ALT_M, AFC_FLOPPER_SPEED_MS, AFC_FLOPPER_GAMMA_DEG, AFC_FLOPPER_RANGE_M, AFC_FLOPPER_CROSS_M and AFC_FLOPPER_AOA_DEG. It ends at the ground if touchdown never takes over.
