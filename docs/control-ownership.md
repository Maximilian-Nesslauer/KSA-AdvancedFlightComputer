# Vehicle control writes

`VehicleControlOwnership` coordinates claims between AFC's RCS executor and Guidance.
It is internal to AFC and provides no shared ownership protocol for other mods.
Conflicting writes from another mod are outside this arbitration.

RCS control and the guidance driver are active in the current build.
Planning features also edit burn plans; this page focuses on control, release and staging.

## Active RCS control

`VehicleCommandSink.Run` calls the RCS writer after stock `FlightComputer.ComputeControl`.
The output commands are rebuilt each step, while the executor's saved settings need explicit release.

| State | Writer | Behavior |
| --- | --- | --- |
| `FlightComputerOutput.Thrusters[].State.CommandPulseTime` | `RcsComputeControlPatch.FireGroups` and `FireLpPattern` | Increase pulse time for the selected thrusters. |
| `FlightComputerOutput.Engines[].State.CommandThrottle` and `CommandBurnTime` | `RcsComputeControlPatch.ZeroEngineCommands` | Clear main-engine commands while the RCS worker command is active. |
| `FlightComputerOutput.AnyActuatorCommanded` and `NextWakeupDeltaTime` | `VehicleCommandSink.Run` | Apply RCS receipt requests without clearing stock's command flag or delaying an earlier wake request. |
| `BurnTarget.BurnDuration` and `IgnitionTime` | `RcsComputeControlPatch.Command` | Replace stock engine timing with RCS timing on the current target. These are not saved settings to restore. |
| `FlightComputer.LastThrustTime` | `RcsComputeControlPatch.Command` | Record commanded RCS pulses. The timestamp is not restored. |
| `FlightComputer.RCSMode` | `RcsExecutor.ForceRcsOn` and `RestoreRcsMode` | Enable RCS at acquisition when needed and restore Disabled if AFC changed it. A running burn that finds RCS switched off stands down, because the player owns the actuator. |
| `FlightComputer.BurnMode` | `RcsExecutor.ForceBurnManual` and `RestoreBurnMode` | Set Manual at acquisition. Restore an earlier Auto only after cleanup succeeds and while the field still matches Manual. Explicit stops and completion discard the saved Auto. This is not a periodic Manual write. |
| `FlightComputer.AttitudeMode`, `AttitudeFrame`, `AttitudeTrackTarget` and `AttitudeTarget` | `RcsExecutor.EnsureBurnControl` through `FlightComputer.RateHold` | Select Auto and null the rotation when the burn takes control from Manual. This is the first write to the mode, so the release records here what to hand back. |
| `FlightComputer.AttitudeMode`, `AttitudeFrame`, `AttitudeTrackTarget` and `CustomAttitudeTarget` | `RcsExecutor.CommandAlignAttitude` | Select Auto and a burn-relative target for Align. Non-X axes use custom Euler angles. Later steps compare mode and tracker, plus frame and coordinates for None and Custom. |
| `FlightComputer.AttitudeTrackTarget`, `AttitudeFrame`, `AttitudeTarget`, `AttitudeMode` and `CustomAttitudeTarget` | `RcsExecutor.EndExecution` through `FlightComputer.SetNullRot` and its own restore | Select None in BurnBody and zero the computed target, hand Manual back when acquisition replaced it, and clear the custom coordinates because tracking is None. The frame stays at BurnBody. |
| Navball frame | `RcsExecutor.BeginControl` through `Vehicle.SetNavBallFrame` | Select BurnBody. Specific SetEnum cancellation and activation-failure paths select the vehicle-region frame; generic release does not restore a captured frame. |

## Yielding the attitude

AFC takes the attitude once, at acquisition, and never takes it again for the same execution.
Rate hold and Align record a command for comparison on later steps.
The comparison checks the fields the live tracker reads for the recorded target.
Mode and tracker always count. The frame and the coordinates count under None and under Custom, because `FlightComputer.UpdateAttitudeTarget` reads the coordinates as a rotation rate under None and as an orientation under Custom, and reads the frame in both.
They do not count for a built-in target, where that method derives the frame itself and a change is the game rather than a takeover.
A value that no longer matches means a player click or another mod points the craft, so AFC yields, drops Align to Hold, stops requiring an attitude for its firing gate, stops claiming a mode to hand back, and leaves the new tracker alone at release.
Selecting another tracked target leaves the mode at Auto, so checking the mode alone is insufficient.
The yield is recorded in `attitude_yielded` and survives a save, so a load cannot take the attitude back from the player.
A burn that finds RCS disabled after acquisition cancels with a visible message and leaves RCS disabled.
The executor detects changes on its next step.
Cleanup also checks for a takeover before restoring attitude, because cancellation can bypass the step check.
A save does not carry the recorded command, so an execution that has not yielded writes its align target again after a load.
A hold burn has no such rewrite, so reconciliation adopts the loaded values as its own command, but only when it has nothing recorded, it still holds the attitude, and the loaded mode is the Auto its rate hold leaves behind.
Reconciliation rebuilds control state from all ownership markers to prevent another acquisition after loading.
`afc-rcs-attitude-yield` checks target changes, a rate and a frame change under None, a changed euler target under Custom, Manual selections, release, and simulated load recovery.
The tests call `Vehicle.SetEnum` directly rather than queue UI input, and load cases reset transient state rather than save and load the game.

Selecting None does not by itself establish a neutral rotation command.
`FlightComputer.UpdateAttitudeTarget` reads `CustomAttitudeTarget` as a rotation rate while tracking is None, and both `RateHold` at acquisition and the release select None.
Coordinates left there are a standing turn whoever wrote them, and the tracker they pointed with is gone once AFC took the attitude, so the release clears them.
`afc-control-write-surface` sets a custom target, takes control through the executor, cancels, and measures the commanded rate after a game step.

## Guidance stopping on a player takeover

Guidance writes one attitude command, Custom in a chosen frame, and `AttitudeOwnership` compares that whole command on every step.
A mismatch means a player or another mod steers now, so the modes that need that attitude stop.
Stopping is not a shutdown request. The release hands back the rate and the gimbal override as it always does, keeps the player's attitude fields, leaves the engine command where the flight left it, and says why guidance stopped.
A takeover moves the attitude and nothing else, so an engine cut guidance already decided on still happens.
That covers the queued one-shot cut, which touchdown, a failed landing solve, an engine-wait failure, a boostback abort and a 6-DOF disengage all set, and it covers a release that shuts down by itself, which the ascent release records in `ShutdownRequested`.
Only a takeover with neither of those pending leaves the engine command alone.
The release makes that call, not the takeover, so a cut decided after the takeover still happens. That matters when a cleanup failed, because the no-cut request waits for the retry and an abort can arrive between the two.

A step that hands the craft to another guidance mode is not a stop either. The 6-DOF dispatch releases only when the step left no mode running, because the handover to terminal hover claims the craft again, and a release in the same step would cut the engine the handover kept lit and reset the mode that just started.
The no-cut request and the reason both survive a failed cleanup, so a retry can neither deliver a late shutdown nor lose why guidance stopped.
Holding the last throttle is not a claim that the trajectory is safe, and continuing an automatic throttle under player steering would be a separate mode capability with its own tests.
`afc-guidance-player-takeover` covers the stop, the queued cut, and the failed cleanup with its retry.

## Guidance control

`GuidanceFeature` installs a prefix on `Vehicle.PrepareWorker` that runs `GuidanceWindow.ApplyAutopilot` for every vehicle, after `InputEvents.ApplyInputEvents` has drained the player's input and before the worker snapshot, so an engine command written there is the one the worker sees.
The same block installs the rate postfix on `FlightComputer.UpdateAttitudeTarget`, and `VehicleCommandSink.Run` dispatches the gimbal writer after the RCS writer.
A step that throws releases the craft through `GuidanceWindow.FailAutopilot` and reports once.
The game menu's Enabled switch queues a release for every held craft, applied on each craft's next step.

| State or action | Writer | Release behavior |
| --- | --- | --- |
| `FlightComputer.BurnMode` | `GuidanceWindow.AcquireControl` and `RestoreBurnMode` | Set Manual when control is acquired, because `Vehicle.PrepareWorker` clears `EngineOn` on every step while the mode is Auto. An Auto that appears while guidance holds the craft is a takeover of the engine, so guidance stops without a cut and leaves it armed. Restore an earlier Auto only on a release that cuts the engine, after the rest of the release succeeded, while the field still reads Manual and the loaded burn target is the one acquisition saw, because stock also writes Manual when a burn is loaded, unloaded or completed. |
| `FlightComputer.AttitudeMode`, `AttitudeFrame`, `AttitudeTrackTarget`, `CustomAttitudeTarget` and `RollMode` | `GuidanceWindow.CommandAttitude` with `AttitudeOwnership` | Capture before writing. Restore frame, tracking and custom coordinates as one group if still unchanged; restore mode and roll conditionally. The complete computed `AttitudeTarget` is not captured. |
| `FlightComputer.AttitudeTarget.RatesCci` | `KsaAttitudeRate.OnUpdateAttitudeTarget` | Add the published rate after stock computes its target. Clear disables future additions; it does not restore the current field. Stock rebuilds it on the next update. |
| `FlightComputerOutput.Gimbals[].State.CommandY` and `CommandZ` | `KsaGimbalControl.OnComputeControl` and `ApplyLsq` | Write output commands from the per-vehicle override and report them through the sink receipt, which sets `AnyActuatorCommanded`. Disengage clears the override, and the Enabled switch stops the writer on the next control step. |
| `Vehicle._manualControlInputs.EngineOn` and `EngineThrottle` | Guidance mode steps and `GuidanceWindow.ApplyAutopilot` | Write through the validated handle in `GameReflection`. HandBackVehicle requests MainShutdown after acquired control, preserving the throttle setting. An explicit no-cut handover skips shutdown, and so does a stop that a player attitude takeover caused. |
| Sequence activation and part-tree refresh | `GuidanceWindow.AutoSequence` | Call `SequenceList.ActivateNextSequence` and `Vehicle.UpdateAfterPartTreeModification`. Staging is not a reversible setting restored on release. |

`afc-guidance-driver` installs these hooks and steps the universe with the ascent step replaced. It checks the claim, the attitude and engine writes, the rate on the worker, the burn-mode hold with its takeover, replaced-burn and no-cut cases, the sink dispatch, the Enabled switch and a failed step.

The release paths keep the existing flight computer and burn plan objects.
That does not mean AFC never changes plans, parts or staging through other features and stock APIs.

## Identity and persistence

RCS and MultiPass registries use save and vehicle IDs.
`SharedVehicleHooks.OnRenamed` moves their entries, stored vehicle IDs, MultiPass burn-mode history and the claim's recorded ID.
Disposal removes the vehicle's registry state.
The execution registries persist to TOML; ownership claims remain runtime state.

## Verification limits

`afc-control-write-surface` compares public `FlightComputer` field values around one RCS activation and cancellation.
Its shallow snapshots do not observe mutations inside referenced objects, private inputs, properties, per-step outputs, staging, Guidance or writes that leave a value unchanged.
The field list is maintained separately from this document.
It is a regression check for that scenario, not an automatic audit of every write or release path.
