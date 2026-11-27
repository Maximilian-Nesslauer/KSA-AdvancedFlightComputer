# Touchdown: one terminal phase (design)

Status: design, 2026-11-27. The cut supervisor is built (#335); the terminal law is not. Replaces the post-plan rules in `Modes/SixDof.cs` and, in a second step, the separate end-of-landing laws in `TerminalHover.cs` and `TerminalBurn.cs`.

## Why

The last metres of a powered landing are flown by whichever of several rules fires first. Each was added after a different crash, they use different numbers for the same quantities, and they keep producing new failures:

| Flight | What happened | Rule that did it |
|---|---|---|
| 2026-11-20 booster | Cut and relight every 1-2 s for 6.6 s, drifting 14.9 m off | `SinkToContact` / no-plan law toggling around the minimum thrust; the one-relight cap was added for it |
| 2026-11-21 11:24 (#245) | Cut at 16 m on a single 0.2 m/s reading with a live plan; relit 17 ms later, spending the only relight | Climb cut |
| 2026-11-27 10:06 Starship | Engine cut 43 m above the plan end; free fall | Climb cut for a craft classed "cannot brake a climb" fires at any height as soon as it stops descending |
| 2026-11-27 10:19 Starship | Contact at 12.8 m/s sideways | Nothing controls sideways speed near the end |
| 2026-11-27 10:21 Starship | Engine cut and relit at the plan end, then contact 15 m from the target while sliding sideways | `SinkToContact` cuts on any climb below minimum thrust (after the plan's last braking pulse); stock holds the craft straight up with no sideways control |

## What is there now

After the plan, touchdown runs this, in this order, every step (`NoPlanNearGround`, `StepSixDofToContact`):

1. **Climb cut** (`GuidanceModeGuards.ShouldCutSixDofOnClimb`): minimum thrust-to-weight over 0.9 and vertical speed at or above 0 cuts the engine, at any height, live plan or not; under 0.9, a climb over 0.1 m/s with no live plan cuts it.
2. **Arrival**: within 1 m of the plan end and (plan expired, below the end, or vertical speed under 0.1 m/s) switches to flying to contact.
3. **Lost plan within 100 m**: hands over to the terminal hover mode, if minimum thrust-to-weight is 0.9 or less.
4. **Flying to contact**, chosen by minimum thrust-to-weight against 0.9:
   - 0.9 or less: `SinkToContact`, a P law on sink rate that turns the engine off whenever the craft climbs and the demand is below minimum thrust.
   - Over 0.9: drop with the engine off if the fall ends under 3 m/s at the legs, else the no-plan law (`FlyWithoutPlan`: stop-on-the-pad thrust, lit at 100 % and cut at 90 % of minimum thrust).
   - One relight allowed (`LimitContactRelights`); the next cut is final.
5. **Attitude**: AFC lets go of the gimbal and asks stock to point straight up (`HoldTouchdownAttitude`).
6. **Contact**: engine cut, guidance ends.

Elsewhere the same job is done again: terminal hover (`TerminalHover.cs`, a sink-rate profile and sideways velocity PID with its own climb cut at 0.3 m/s) and the terminal burn (`TerminalBurn.cs` / `TerminalBraking`, a coast-and-hoverslam law for craft that cannot hover), plus the G-FOLD fallback.

### The faults

1. **Two minimum thrust-to-weights.** The cut decisions use `TerminalMinThrottleTwr` (the game's minimum throttle, all thrust, pressure at the craft); the plan and the cycle log use the landing group's floor (`SixDofThrottleFloor`). They differ by about 9 % (0.885 against 0.965 at 10:21, 1.01 against 1.10 at 10:06). Every rule switches at 0.9, and a Starship on one Raptor sits right on that line.
2. **Height-blind cuts.** The climb cut ignores height; the 10:06 cut was 43 m up.
3. **Bang-bang engine logic around minimum thrust.** Both post-plan laws turn the engine off on any climb or on a demand just under minimum thrust. With minimum thrust about equal to the weight the demand sits inside the band. The one-relight cap turns the next toggle into a final cut at whatever height it happens.
4. **Three ground references.** The drop test measures to the legs (`VehicleHeightM`), the no-plan law to the plan end (`SixDofTargetComAltM`), terminal hover to the terrain under the craft.
5. **No sideways control after the plan.** Stock holds the craft upright; any sideways speed at the plan end reaches the ground.
6. **No record of the end.** The cycle log stops at the plan end; the after-plan contact line has no sideways speed, tilt or rates.
7. **Order-dependent.** Overlapping conditions checked in a fixed order decide which law flies, on small differences in vertical speed.

## Built first: the cut supervisor (#335)

The commonest failure is simpler than the rest: the plan brings the craft to rest a little above the ground (10:21: 1.9 m, then 0.5 m) and nothing lets it down. A plain cut would have landed it. So the first change is a **cut supervisor**, a watchdog beside touchdown guidance rather than part of it (`CutSupervisor` in `Ui/Logic`, stepped by `GuidanceWindow.StepCutSupervisor` after the modes):

- **Inputs:** the legs' height above the ground under the craft (`LegsHeightM`, a radio altitude) and the vertical speed. Nothing from the plan.
- **Armed** once the craft has descended faster than 1 m/s with the engine lit, so a craft on the pad is never cut.
- **Integrates**, while the legs are within CUT WATCH HEIGHT (5 m) of the ground, the target sink (CUT SINK, 0.5 m/s) less the actual sink: the height it has failed to descend. A slight stall builds slowly, a hover at 0.5 m/s, a climb faster; sinking faster pays it back, never below zero.
- **Cuts** with the legs within CUT HEIGHT (2 m) at CUT SHORTFALL (0.3 m there, less in proportion lower down, never under 0.1 m), only while the craft is still short of CUT SINK, and holds the engine off every step after, whatever guidance asks; touchdown guidance flies the rest to contact with no relight left. A drop from 2 m lands at about 6.3 m/s.
- **Logged:** the first step of each shortfall and the cut, with height, vertical and sideways speed, tilt and the drop's landing speed. The cycle log now keeps writing after the plan's end, with the supervisor's height, shortfall and action.
- **Settings** on TUNE > TOUCHDOWN > CUT SUPERVISOR.

Later, a learned supervisor could be trained on the logged inputs; the integrator comes first because a handful of landings is no training set and its decisions can be read.

**Also built: the progressive glideslope** (see the changelog), which shapes the approach so the last stretch comes down nearly straight, from the craft's length and flip time rather than per-craft tuning. Ending the plan at a gate above the pad, with the terminal law below flying the last metres, is the next step.

The terminal law below is still to build.

## The design

### One terminal phase, one state machine

Touchdown gets explicit phases, each entered in one place with its reason logged:

```
PLAN ──► TERMINAL ──► CONTACT ──► DONE
  │          ▲
  └─ lost ───┘ (within the terminal gate)
```

- **PLAN**: the SCvx plan flies, as now.
- **TERMINAL**: entered when any of these holds, with the craft descending or near the end:
  - the plan has ended or expired;
  - the plan is lost (refusal run, restart) within the terminal gate (height above contact `H_gate`, default 30 m, or plan time-to-go under `T_gate`, default 3 s);
  - (option, see decisions) re-solves are refused with under `T_gate` left.
- **CONTACT**: the game's contact flag. Engine cut, logged with sink, sideways speed, tilt, rates and distance.
- **DONE**: guidance released.

Above the terminal gate a lost plan keeps today's ladder (warm restart, cold restart, G-FOLD).

### The terminal law

One controller, run every step in TERMINAL:

- **Height** `h`: the centre of mass above first contact, measured against the ground **under the craft** (terrain or landing deck below it, minus the measured CoM-to-legs height), not the plan's end, so drift off the site is handled. One function, used by the law, the drop test and the log.
- **Vertical**: a sink-rate setpoint `v_set(h)` = touchdown sink (0.5 m/s) + `k h^2`, capped (as terminal hover's profile), and a PI on the vertical speed error, gravity fed forward.
- **Sideways**: drive the horizontal velocity to zero with a PD on velocity (no position chase below the gate by default), giving a sideways acceleration.
- **Thrust vector**: the sum, its direction clamped to a tilt cone (`TiltMax`, default 10 deg); throttle from its magnitude along the thrust axis.
- **Attitude**: flown by AFC's `AttitudeController` (authority-aware, gimbals, RCS and fins, as bellyflop and hand flying use), not handed to stock.

### Engine on and off: one rule, by capability not threshold

One minimum thrust-to-weight, computed one way: the lit landing engines' thrust at the floor throttle at the ambient pressure, over the weight (the plan's figure). From it:

- **Can descend on the engine** (`TWRmin < 1 - margin`, margin 0.03): the engine stays lit until contact. At minimum thrust the craft cannot climb for long; a transient climb is taken out by gravity with the engine at minimum. **No airborne cut.**
- **Cannot** (`TWRmin >= 1 - margin`): the terminal burn's law (`TerminalBraking.Evaluate`, already written and tested for this case) decides coast and hoverslam: engine off while the fall from here can still be stopped on the remaining relights, lit at the ignition height. A cut is allowed only when the fall from here reaches contact at no more than `v_safe` (3 m/s), or when a relight is left and the braking law has planned it.
- **Relights are a budget**, set per craft (`LANDING RELIGHTS`, default 1) and counted from the landing burn's ignition, read by the braking law, not a cap applied after the fact.
- A **sustained** climb (over +0.3 m/s for 0.5 s at minimum throttle) is the only climb signal, and it hands to the braking law rather than cutting.

### Logging

- The cycle log keeps writing through TERMINAL to DONE, with the phase, `h`, the setpoints, the tilt command and the engine decision.
- Every phase change and every engine on/off is an event with its reason and the numbers behind it.
- Contact logs vertical and sideways speed, tilt, body rates and distance from the target, on every path.

### Shared with the modes

Step 2 (separate change): terminal hover becomes this controller with the player's setpoint nudges, and the terminal burn becomes its "cannot descend" branch, so all three end the same way.

## What goes

`ShouldCutSixDofOnClimb`, `CutSixDofEngine`, `SinkToContact`, the drop branch and the no-plan law in `StepSixDofToContact`, `LimitContactRelights` / `SixDofMaxContactRelights`, the 0.9 `SixDofHoverMaxMinTwr` split in touchdown, `HoldTouchdownAttitude`'s stock hold, and touchdown's use of `TerminalMinThrottleTwr`.

## Tests

- **Offline** (`--touchdown-terminal`, Guidance.Tests): the law on a 6-DOF point-mass-plus-attitude model, entered at 2-25 m with 0-8 m/s sideways and -3 to +0.5 m/s vertical, for `TWRmin` 0.6, 0.97, 1.02 and 1.1, a climb transient, and drift over a 2 m ledge. Pass: contact under 2 m/s down and 0.5 m/s sideways, tilt under 5 deg, no airborne cut for `TWRmin < 0.97`, relights within budget.
- **Unit**: the engine rule's decisions as a pure function (UiLogic, as `GuidanceModeGuards` is now).
- **Harness**: `afc-guidance-flopper-entry` and the booster landing, before and after.
- **Windowed**: the one-engine Starship from the bellyflop save.

## Decisions needed

1. **When TERMINAL takes over:** at the plan end and on a plan lost within the gate only (recommended), or also on refused re-solves with under 3 s left (10:21 would have been caught 2 s earlier).
2. **Sideways:** zero the velocity only (recommended), or also creep back towards the target above some height.
3. **Relight budget:** a per-craft setting, default 1 (recommended), or read from the engine where KSA exposes it.
4. **Tilt limit in TERMINAL:** 10 deg (recommended) or another value.
5. **Scope:** touchdown first, terminal hover and terminal burn in a second change (recommended), or all at once.

Separate from this design: the refused re-solves in the last 3 s of the 10:21 flight (attitude mismatch at interval 0), which let the craft drift before the plan ended.
