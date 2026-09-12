# AdvancedFlightComputer [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Extra maneuver planning tools for [Kitten Space Agency](https://ahwoo.com/app/100000/kitten-space-agency).

Adds quick-tools to the Transfer Planner (set Pe/Ap, match/set inclination, circularize), flyby targeting so a Hohmann transfer arrives as a flyby instead of an impact, multi-pass burn splitting for Oberth-efficient departures, RCS-only burn execution, automatic staging with spent-booster drop and configurable staging delays, automatic removal of finished burns, and enables the planner to target interstellar comets on hyperbolic orbits (Oumuamua, 2I/Borisov, 3I/ATLAS).

This mod is written against the [StarMap loader](https://github.com/StarMapLoader/StarMap).

Validated against KSA build version 2026.9.7.5402.

## Features

### Maneuver Quick-Tools

New plan types in the stock Transfer Planner dropdown:

- **Set Periapsis / Set Apoapsis** - single burn at the opposite apse to raise or lower one apse to a target altitude.
- **Match Inclination** - plane-change burn at AN or DN to align with a target orbit's plane.
- **Set Inclination** - plane-change burn at AN or DN to set an absolute inclination angle. The reference plane is selectable: **Ecliptic** (matches `Orbit.Inclination`, KSA's system-wide inertial Z) or **Equatorial** (parent body's equator, standard astrodynamics convention). For Earth the two differ by the ~23.4 degree obliquity.

Right-click the controlled vehicle's orbit and open **Advanced Flight Computer** to select a quick-tool.
The **At Periapsis** submenu offers **AFC: Set Apoapsis...**, and **At Apoapsis** offers **AFC: Set Periapsis...**.
These shortcuts open the planner and they do not place a burn at the clicked point.

Quick-tools can plan a single burn after the last planned burn that changes the trajectory.
For example, plan a burn to raise apoapsis, then use **Set Periapsis** on the resulting orbit.
The preview and created node use that post-burn trajectory.
Multi-pass cannot start on a pending burn's trajectory.
Shortcuts wait until a stock transfer calculation or active multi-pass execution has finished.

### Flyby Targeting

The stock planner aims every transfer at the target body's center, so a well-timed Hohmann arrives as an impact and the flyby has to be set up afterwards as a separate correction. Tick **Target flyby periapsis** in the Transfer Planning window and the departure burn is aimed to arrive at a periapsis you choose instead, so **Create** fires the flyby directly.

- **Periapsis** is entered against a selectable reference: **Surface** (altitude above the mean radius), **Center** (radius straight from the body center), or **Atmosphere** (altitude above the atmosphere boundary). The Atmosphere option only appears for bodies that have one, and a request below the safe floor is refused.
- **Flyby side** picks which side of the body you pass, named in the target's own orbital frame: **Inner** (toward its parent), **Outer** (away from it), **North**, or **South**. The aim offset has to stay perpendicular to the approach, so a side whose axis lies along the approach direction cannot be reached and is greyed out. That is also why there is no leading/trailing option for a Hohmann-style arrival.
- Works for moon flybys and for interplanetary targets, either as a single burn or split across multi-pass passes.
- The section reports the approach speed, the impact parameter, the departure delta-V next to the impact-aimed one, and the periapsis the propagated trajectory actually reaches. While a flyby is armed the preview shows that retargeted trajectory in place of stock's center-aimed one.
- For multiple passes, the readout uses the selected departure after any schedule shift and the periapsis from the final preview trajectory.
- If propagation cannot confirm a periapsis, the readout shows **No prediction** and its reason, such as a departure time outside the flight plan or no resolved target encounter.

**Limitations:**
- The plan is impulsive, the burn is not. On a near-escape departure the apoapsis moves by thousands of km per m/s of periapsis velocity, so the periapsis actually flown drifts from the requested one by roughly the finite-burn loss (order of one percent of a multi-km/s injection). Expect to trim it with a small correction burn, or split the departure across several passes to cut the loss.

### Multi-Pass Burns

![LEO to Luna multi-pass transfer](images/LEO_to_Luna_5_passes.png)

Any of the above plan types (including stock Hohmann and circularize Apse transfers) can be split into multiple burns across successive orbits.
Instead of one long burn that sweeps a large arc away from periapsis, the engine fires in shorter bursts near periapsis on each orbit, reducing finite-burn loss.

**Supported plan types that can be split:**
- Hohmann transfers
- Set Periapsis / Set Apoapsis
- Match Inclination / Set Inclination
- Circularize Apoapsis / Periapsis

**How to use:**
1. Select a plan type and configure the maneuver as usual.
2. Use the **PASSES** slider to choose how many passes (2-10).
3. Click **Create**. The first pass burn is placed in the burn plan.
4. Enable **Auto** burn mode. Each pass fires automatically, and the next pass is scheduled after completion.
5. The plan window shows "Multi-pass active: pass X of N" with remaining pass details and a **Cancel remaining passes** button.

**Why it helps:**
When burn duration is a significant fraction of the orbital period, a single burn wastes fuel by thrusting far from periapsis. Splitting across N passes keeps each burn near periapsis where the Oberth effect is strongest.
This is the same technique used by real missions: lunar kick stages that perform multiple perigee burns over several days to gradually raise their orbit before the final trans-lunar injection, because a single burn would spend too long thrusting away from periapsis. Particularly useful for low-TWR spacecraft (ion engines, small kick stages, nuclear tugs) where a single departure burn can take tens of minutes and sweep a large fraction of the orbit.

**Hands-free execution:**
Multi-pass works best with the built-in [automatic staging](#automatic-staging) switched on (handles staging between passes) and the built-in [automatic burn removal](#automatic-burn-removal) left on (cleans up completed burns). A multi-pass execution then runs hands-free from first ignition to final departure.

**Limitations:**
- Automatic pass advancement currently requires stock engine Auto mode. RCS translation completion does not advance the multi-pass sequence.
- Same-parent transfers (e.g., LEO to Luna) shift the final burn forward by a few parking periods to fit the K-schedule. The shift is shown in the plan window.
- A change to the flyby side or altitude updates the selected departure even during thrust.
- Very high-energy departures from small SOIs (e.g., low Mars orbit to Saturn) may auto-clamp to fewer passes because intermediate orbits would escape the SOI.

### RCS Translation Burns

![RCS burn options in the burn editor](images/RCS_burn_panel.png)

Execute a planned burn with RCS thrusters only, no main engine. Useful for small correction burns (rotating the whole vehicle for the main engine can cost more than just translating) and for vehicles without an active main engine like small probes.

- The stock **Auto** burn button is still the single trigger: it executes the next burn with its resolved method. A burn resolves to the main engine when an active, fueled engine exists, otherwise to RCS; a per-burn override is available in the burn editor window ("Execution: Default | Engine | RCS").
- Two attitude strategies, selectable per burn: **Hold** (keep the current attitude, fire the axis mix that points at the burn vector) and **Align** (rotate the strongest thruster axis onto the burn vector first). **Auto** (default) compares propellant estimates for both, including the slew cost, and picks the cheaper one. The estimates derive from the bang-off-bang slew cost model standard in the attitude control literature.
- Execution is closed-loop against the game's own delta-V accounting: pulses shrink as the remaining delta-V approaches zero, and the burn stops inside the thrusters' minimum impulse of the target. The engine autopilot is suppressed for the whole run, so a misclick can never ignite the main engine on an RCS-armed burn.
- Burns themselves stay in the stock save format; removing the mod keeps every planned burn. The RCS arming metadata lives in `mods/AdvancedFlightComputer/rcs-exec.toml` next to the mod and survives save/load, including mid-burn.
- The burn editor warns when a burn resolves to RCS but no thruster can translate (no propellant, none active) and when the estimated propellant exceeds what the thrusters can actually reach. Auto also shows an alert and refuses the burn before it creates execution state or changes the controls when the burn has no delta-V, no usable translation, or no axis that can serve its direction.
- Estimates for a later planned burn use the current vehicle as an approximation and show **Estimate basis: Current vehicle** in the burn editor or **Current vehicle** in the gauge. Stock only loads the first executable burn as its active burn target, so these estimates do not forecast earlier burns or staging.
- Completed RCS burns raise a public event (`RcsBurnCompletions.Completed`) other mods can consume; the built-in [automatic burn removal](#automatic-burn-removal) uses it to clean up finished RCS burns the same way it cleans up engine auto-burns.
- The **allocator** is selectable per burn (default **Groups**). Groups fires signed-axis groups and uses attitude control to counter residual torque. **LP** solves a fuel-optimal jet-selection problem over individual thruster forces and torques (the Bergmann/Draper formulation). It prices residual torque on axes with rotation authority and requires zero net torque on the other axes. LP can require costly counter-thrust, so it is opt-in. It falls back to Groups when the constraints are infeasible.

### Automatic Staging

<table>
  <tr>
    <th align="center">Stock</th>
    <th align="center">With AdvancedFlightComputer</th>
  </tr>
  <tr valign="top">
    <td><img src="images/AutoStage_stock.png" alt="Stock engine gauge panel" width="420" /></td>
    <td><img src="images/AutoStage_button.png" alt="Engine gauge panel with AUTOSTAGE toggle" width="420" /></td>
  </tr>
</table>

Activates the next sequence whenever the active engines run out of propellant, and drops burnt-out boosters while the rest of the stage keeps firing. Works during auto-burns (continues the burn instead of aborting) and manual burns. Formerly the separate AutoStage mod; remove that mod when you install this version, because two stagers on one burnout would activate two sequences. The settings written by AutoStage are imported on the first load.

- **AUTOSTAGE toggle button** on the EngineControl gauge panel, in the free slot under RCS. The switch belongs to the vehicle, so an armed craft keeps staging when you control another one. The same switch is available on the Mods settings page for installs without KittenExtensions.
- **Control-module guard** - a sequence that would separate the last control module is never staged automatically; stage it by hand if that is what you want.
- **Auto-burn continuation** - keeps the burn mode at Auto through staging so planned burns do not abort.
- **Cascade staging** - stages again if the next stage is empty or only has decouplers.
- **Spent stage drop** - sheds burnt-out boosters as soon as they quit, without waiting for the core stage to run dry.
- **Configurable staging delays** - independent delays for decouplers and engines, simulating separation and engine spool-up time.
- **On-screen countdowns** - "Decouple in X.Xs" and "Ignition in X.Xs" alerts during a delayed stage.

**Spent stage drop.** A launch stage that mixes solid boosters with a liquid core does not run out of propellant all at once. The boosters burn out first, but a staging trigger that waits for *every* active engine to go dry never fires while the core is still burning, so the empty booster casings ride along as dead mass. The mod therefore also stages when the next sequence would jettison nothing but burnt-out hardware. Before staging it works out which parts each decoupler in that sequence would separate, and only fires when the sequence activates no engine, every active engine in the jettisoned parts is spent, at least one engine that stays with the vehicle is still firing, and nothing in the jettisoned parts is an engine that has never been activated. It also refuses when the parts to be jettisoned still hold propellant a retained engine can draw from, or when an enabled fuel link crosses the separation, so crossfeed setups are not cut off mid-burn. Turn it off with "Drop spent stages early" on the Mods settings page if staging should wait for a full burnout.

**Staging delays.** Two delays are configurable per part variant, both measured from the staging trigger: the engine ignition delay (default values per stock engine variant, the small EngineA1 ignites after 2 s, EngineA3 after 3 s) and the decoupler delay (default 0 s, which matches stock). Set the decoupler delay shorter than the engine delay if the lower stage should drop away before the upper stage lights up.

- **Settings window (Settings > Mods > AUTOSTAGE):** the two switches, then the "Engine Ignition Delays" and "Decoupler Delays" tables listing every known part variant. Every setting takes effect immediately; click SAVE to persist it.
- **Part window (right-click part > Window):** override the delay for a specific sequence on the current vehicle. A part gets one block per sequence it fires something in, each naming the module it covers. Per-vehicle overrides take priority over the global config.

The global config is `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\autostage.toml`:

```toml
[staging]
drop_spent_stages = true

[engine_delays]
CorePropulsionA_Prefab_EngineA2 = 2.0
CorePropulsionA_Prefab_EngineA3 = 5.0

[decoupler_delays]
CoreFairingA_Prefab_Interstage3W3HB = 1.0
```

Per-vehicle sequence overrides are stored next to it in `autostage-vehicles\<vehicle-id>.toml`, created automatically when you set an override in the part window, with separate `[sequence_delays]` (engines) and `[decoupler_delays]` sections. Nothing is written to a game save.

### Automatic Burn Removal

In stock KSA, when an auto-burn completes the flight computer flips the burn mode to Manual but leaves the burn entry in the plan, so you have to click "Delete" before the next maneuver can take focus. This feature cleans up completed burns automatically. Formerly the separate AutoRemoveFinishedBurns mod; remove that mod when you install this version. Its saved switch is imported on the first load.

- **Auto-burns** are removed as soon as the flight computer flips out of Auto mode on completion. Completion is confirmed through the same delta-V vector reversal the stock flight computer uses, so a burn that flamed out before reaching its target stays in the plan and can be resumed after staging.
- **RCS burns** executed by this mod are removed on their completion event.
- **Manual burns are never touched**, and only the vehicle you control is watched. A burn that finishes on a background vehicle stays in its plan.
- **Switch** on the Mods settings page, on by default, persisted in `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\autoremove.toml`.

### Hyperbolic Targets

The stock Transfer Planner filters out bodies with eccentricity >= 1. This mod lets it target interstellar comets (Oumuamua, 2I/Borisov, 3I/ATLAS) by patching the planner's time-of-flight and alignment math to handle unbound orbits.

## Installation

1. Install [StarMap](https://github.com/StarMapLoader/StarMap) and [KittenExtensions](https://github.com/tsholmes/KittenExtensions) (the latter is only required for hyperbolic targets and the AUTOSTAGE gauge button).
2. Download the latest release from the [Releases](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/releases) tab.
3. Extract into `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\`.
4. The game auto-discovers new mods and prompts you to enable them. Alternatively, add to `Documents\My Games\Kitten Space Agency\manifest.toml`:

```toml
[[mods]]
id = "AdvancedFlightComputer"
enabled = true
```

## Dependencies

| Package | Purpose | Tested version |
| --- | --- | --- |
| [StarMap](https://github.com/StarMapLoader/StarMap) | Mod loader, required at runtime (see [Installation](#installation)) | 0.4.6 |
| [KittenExtensions](https://github.com/tsholmes/KittenExtensions) | Optional, required at runtime for the hyperbolic-targets and AUTOSTAGE button XML patches | v0.4.0 |

## Build dependencies

Required only to build the mod from source. Targets **.NET 10**.

| Package | Source | Tested Version |
| --- | --- | --- |
| [StarMap.API](https://github.com/StarMapLoader/StarMap) | NuGet | 0.3.6 |
| [Lib.Harmony](https://www.nuget.org/packages/Lib.Harmony) | NuGet | 2.4.2 |

## Experimental guidance

The guidance source is integrated under `AdvancedFlightComputer/Features/Guidance/`.
The AFC Guidance menu reports that execution is unavailable until control ownership is integrated.
This build does not install guidance actuator hooks or expose execution controls.
The numerical libraries and their console checks remain independent of the game.
See [the guidance layout and build instructions](docs/guidance/README.md).

If a standalone PoweredGuidance mod is installed, disable it before a future combined guidance flight test.
This build does not change the game manifest.

## Mod compatibility

- [AutoStage](https://github.com/Maximilian-Nesslauer/KSA-AutoStage) is now part of this mod. Delete its folder under `mods`, do not only disable it: KittenExtensions applies the XML patches of every manifest entry, so a disabled AutoStage still puts its own dead AUTOSTAGE button on top of this one, and an enabled one keeps the built-in staging off, which the log says.
- [AutoRemoveFinishedBurns](https://github.com/Maximilian-Nesslauer/KSA-AutoRemoveFinishedBurns) is now part of this mod. Remove the standalone mod; with both installed each removes the burn the other already took care of, which is harmless but logged.
- Known conflicts: none

## License

MIT - see [`LICENSE`](LICENSE). This applies to everything in the current tree.

The guidance source under `AdvancedFlightComputer/Features/Guidance/` was imported from
[cairn5/PoweredGuidance](https://github.com/cairn5/PoweredGuidance) with its full history
rather than as a squashed snapshot, so its commits stay reachable from this repo with
their original authorship intact.

PoweredGuidance releases up to and including **v0.3.1** linked
[ECOS](https://github.com/embotech/ecos), which is GPLv3, and were therefore distributed
under GPLv3 as a whole. Those commits remain reachable here and those releases stay
available under GPLv3 - a licence already granted cannot be withdrawn - but they are the
only versions to which that applies. ECOS was removed and replaced by Clarabel
(Apache-2.0) before the import, so **nothing in the current tree is GPL**.

Attribution for the vendored solvers and their own transitive dependencies is in
[`THIRD-PARTY-NOTICES.md`](THIRD-PARTY-NOTICES.md); all
of them are permissive, and none impose copyleft on this work.

## Community

Thread on the KSA forums: https://forums.ahwoo.com/threads/advanced-flight-computer.783/

## Check out my other mods

- [MeasureTools](https://github.com/Maximilian-Nesslauer/KSA-MeasureTools) - click-to-measure ruler, protractor, and surface measuring in the map view ([forum thread](https://forums.ahwoo.com/threads/measuretools.992/))
