# Burns and staging

## RCS Translation Burns

Execute a planned burn with RCS thrusters only, no main engine. Useful for small correction burns (rotating the whole vehicle for the main engine can cost more than just translating) and for vehicles without an active main engine like small probes.

- The stock **Auto** burn button is still the trigger, and **FLY BURN** in the footer of the **BURNS** page sends the same command: it executes the next burn with its resolved method. A burn resolves to the main engine when an active, fueled engine exists, otherwise to RCS; **BURN WITH** on the **BURNS** page sets a per-burn override (**ENGINE/RCS** or **RCS ONLY**), and **BURN WITH** on the docked block under the burn editor shows and cycles the same choice.
- Two attitude strategies, selectable per burn as **ATTITUDE** on the **BURNS** page while RCS flies the burn: **Hold** (keep the current attitude, fire the axis mix that points at the burn vector) and **Align** (rotate the strongest thruster axis onto the burn vector first). **FREE** (default) compares propellant estimates for both, including the slew cost, and picks the cheaper one.
- Execution is closed-loop against the game's own delta-V accounting: pulses shrink as the remaining delta-V approaches zero, and the burn stops inside the thrusters' minimum impulse of the target. The engine autopilot is suppressed for the whole run, so a misclick can never ignite the main engine on an RCS-armed burn.
- Burns themselves stay in the stock save format; removing the mod keeps every planned burn. The RCS arming metadata lives in `mods/AdvancedFlightComputer/rcs-exec.toml` next to the mod and survives save/load, including mid-burn.
- The **BURNS** page warns when a burn resolves to RCS but no thruster can translate (no propellant, none active) and when the estimated propellant exceeds what the thrusters can actually reach. Auto also shows an alert and refuses the burn before it creates execution state or changes the controls when the burn has no delta-V, no usable translation, or no axis that can serve its direction.
- The **BURNS** page lists every planned burn, and a press on a row picks it. **OPEN AFC** on the docked block under the burn editor and in the rendezvous burn infobox opens the page on that burn. The docked block shows **BURN WITH**, the state and **CANCEL** of a running RCS burn, and the warnings of its burn. Estimates for a later planned burn use the current vehicle as an approximation and say so on the **BURNS** page. Stock only loads the first executable burn as its active burn target, so these estimates do not forecast earlier burns or staging, and the footer verb shows only for the next burn.
- An RCS burn ends like an engine auto-burn: time warp stops, the game shows its alert and the next burn loads.
- The allocator is selectable per burn (default **BY AXIS**), as **THRUSTER MIX** in the **GNC** fold of **BURNS**. **BY AXIS** fires signed-axis groups and uses attitude control to counter residual torque. **MIN FUEL** solves a fuel-optimal jet-selection problem over individual thruster forces and torques, as a linear program (the Bergmann/Draper formulation). It can require costly counter-thrust, so it is opt-in, and it falls back to **BY AXIS** when the constraints are infeasible.

## Warp to the turn

While stock **AUTO** flies the next burn, the console offers **WARP TO TURN** when the turn is more than 90 s away. It warps to 30 s before stock AUTO starts to turn the craft, so the turn and the burn run at 1x. Stock AUTO stays on. The stock **WARP** runs to the ignition and turns the craft under warp. **STOP WARP** stops the warp to the turn, and it does not stop a warp that you started elsewhere.

AFC also checks before the burn that the craft can aim it, and shows a warning on the burn. The warning is a caution, not a refusal:

- **RCS OFF: CRAFT DOES NOT TURN** and **NO RCS FOR PITCH AND YAW**: stock AUTO turns the craft to the burn with RCS before the engine lights. With RCS off, the craft does not turn, and the engine lights on time pointed wherever the craft points. With no thruster that has propellant and turns the craft in pitch and yaw, the engine lights pointed wherever the craft points, or the burn waits.
- **RCS TOO STRONG TO AIM**: one RCS pulse turns a light craft so fast that stock AUTO does not start the turn from rest, and the burn waits. Smaller thrusters or more mass fix it.
- **MAY NOT TURN FROM n DEG OFF**: from rest less than that angle off the burn attitude, stock AUTO does not fire the RCS of the craft, and the burn waits until the craft drifts out. Point the craft farther away before the turn starts.
- **BURN s, AIM NEEDS s**: the engine needs that time to settle its aim, and stock AUTO cannot stretch the burn further. The burn ends with an error in direction, so check the orbit after it.

## Automatic Staging

<table>
  <tr>
    <th align="center">Stock</th>
    <th align="center">With AdvancedFlightComputer</th>
  </tr>
  <tr valign="top">
    <td><img src="../../images/AutoStage_stock.png" alt="Stock engine gauge panel" width="420" /></td>
    <td><img src="../../images/AutoStage_button.png" alt="Engine gauge panel with AUTOSTAGE toggle" width="420" /></td>
  </tr>
</table>

Activates the next sequence whenever the active engines run out of propellant, and drops burnt-out boosters while the rest of the stage keeps firing. Works during auto-burns (continues the burn instead of aborting) and manual burns. Do not install the standalone AutoStage mod next to AdvancedFlightComputer, because two stagers on one burnout would activate two sequences. The settings written by AutoStage are imported on the first load.

- **AUTOSTAGE toggle button** on the EngineControl gauge panel, in the free slot under RCS. The switch belongs to the vehicle, so an armed craft keeps staging when you control another one. The **AUTOSTAGE THIS CRAFT** switch at the top of the **STAGING** page of the console, the **AUTOSTAGE** switch in the head and in its drawer set the same switch.
- **Control-module guard** - a sequence that would separate the last control module is never staged automatically; stage it by hand if that is what you want.
- **Auto-burn continuation** - keeps the burn mode at Auto through staging, also when you stage by hand, so planned burns do not abort.
- **Cascade staging** - stages again if the next stage is empty or only has decouplers.
- **Spent stage drop** - sheds burnt-out boosters as soon as they quit, without waiting for the core stage to run dry.
- **Payload fairing release** - drops stock payload fairings in a staging row of their own once the air is thin enough, apart from engine staging.
- **Configurable staging delays** - independent delays for decouplers and engines, simulating separation and engine spool-up time.
- **On-screen countdowns** - "Decouple in X.Xs" and "Ignition in X.Xs" alerts during a delayed stage.

**Spent stage drop.** When solid boosters burn out before a liquid core, a trigger that waits for every engine to go dry would carry the empty casings along as dead mass. The mod therefore also stages when the next sequence would jettison nothing but burnt-out hardware: the sequence activates no engine, every active engine it separates is spent, an engine that stays with the vehicle is still firing, and nothing it separates is an engine that has never been activated. It refuses when the separated parts still hold propellant a retained engine can draw from, or when an enabled fuel link crosses the separation, so crossfeed setups are not cut off mid-burn. Turn it off with **DROP SPENT STAGES EARLY** on the **STAGING** page if staging should wait for a full burnout.

**Payload fairings.** A staging row that holds only stock payload fairings (aeroshrouds on stock radial decouplers) is released by AutoStage on its own, not by engine staging. In air, the row is released when the dynamic pressure falls and is at or below **FAIRING RELEASE Q** (default 3 kPa) and the whole vehicle is at least **FAIRING CLEARANCE** (default 30 km) above the terrain. Without air, it is released while climbing once the vehicle is 20 m clear of the terrain. Both settings are on the **LAUNCH** page, per craft and per body. Engine staging, the booster reserve and the separation countdown pass such a row over while AutoStage flies the craft. A row that mixes fairings with other parts stays with normal staging, and LAUNCH shows which kind each row is. If AutoStage is disarmed or a release fails, the row goes back to the stock staging key. Modded fairing parts are not released yet.

**Solid motors in the staging readout.** The game's staging model gives a solid motor the burn time of a full grain on every refresh, so a nearly spent booster is shown as a trickle that outlasts the core. The mod paces the motor at the mean flow of the burn it has left, read off its thrust curve, so the in-flight staging window and the guidance stage list end the booster phase where the boosters do.

**Staging delays.** Two delays are configurable per part variant, both measured from the staging trigger: the engine ignition delay (default values per stock engine variant, the small EngineA1 ignites after 2 s, EngineA3 after 3 s) and the decoupler delay (default 0 s, which matches stock). Set the decoupler delay shorter than the engine delay if the lower stage should drop away before the upper stage lights up.

- **STAGING page of the console:** the switches, and in the **DELAYS** fold the delays of this craft per sequence, the default delays of the parts of this craft, and under **ALL PARTS** the "ENGINE IGNITION DELAYS" and "DECOUPLER DELAYS" tables of every known part variant, with a filter. Every setting takes effect immediately; click **SAVE DEFAULTS** to persist the defaults. In the editor the page holds the defaults only.
- **Part window (right-click part > Window):** override the delay for a specific sequence on the current vehicle, or open the STAGING page with **OPEN AFC**. A part gets one block per sequence it fires something in, each naming the module it covers. Per-vehicle overrides take priority over the global config.

The global config is `mods\AdvancedFlightComputer\autostage.toml` in the game profile folder, by default `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\autostage.toml`:

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

## Automatic Burn Removal

KSA removes a finished auto-burn from the plan when another burn follows it, but keeps the last one. This feature removes that last finished burn, engine or RCS, on every vehicle. Manual burns and burns that stopped for lack of propellant stay in the plan. A switch saved by the old standalone AutoRemoveFinishedBurns mod is imported on the first load.

- **AUTO REMOVE FINISHED BURNS** on the **BURNS** page of the console, on by default, persisted in `Documents\My Games\Kitten Space Agency\mods\AdvancedFlightComputer\autoremove.toml`.
