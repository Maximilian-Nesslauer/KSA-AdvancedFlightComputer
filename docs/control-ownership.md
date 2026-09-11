# Vehicle control writes

`VehicleControlOwnership` coordinates claims between AFC's RCS executor and Guidance.
It is internal to AFC and provides no shared ownership protocol for other mods.
Conflicting writes from another mod are outside this arbitration.

RCS control is active in the current build.
Guidance's command hooks are not installed, so its control paths below describe code that is not yet enabled.
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

## Guidance code that is not enabled

`GuidanceFeature` installs diagnostics and lifecycle cleanup, but no vehicle command driver or actuator hooks.
The sink currently dispatches only RCS.
These paths must be integrated with ownership before Guidance is enabled.

| State or action | Writer | Release behavior |
| --- | --- | --- |
| `FlightComputer.AttitudeMode`, `AttitudeFrame`, `AttitudeTrackTarget`, `CustomAttitudeTarget` and `RollMode` | `GuidanceWindow.CommandAttitude` with `AttitudeOwnership` | Capture before writing. Restore frame, tracking and custom coordinates as one group if still unchanged; restore mode and roll conditionally. The complete computed `AttitudeTarget` is not captured. |
| `FlightComputer.AttitudeTarget.RatesCci` | `KsaAttitudeRate.OnUpdateAttitudeTarget` | Add the published rate after stock computes its target. Clear disables future additions; it does not restore the current field. Stock rebuilds it on the next update. |
| `FlightComputerOutput.Gimbals[].State.CommandY` and `CommandZ`, plus `AnyActuatorCommanded` | `KsaGimbalControl.Apply` and `ApplyLsq` | Write output commands from the per-vehicle override. These paths set the flag directly and do not use the sink receipt yet. Disengage clears the override. |
| `Vehicle._manualControlInputs.EngineOn` and `EngineThrottle` | Guidance mode steps and `GuidanceWindow.ApplyAutopilot` | Write through the shared field accessor. HandBackVehicle requests MainShutdown after acquired control, preserving the throttle setting. An explicit no-cut handover skips shutdown. |
| Sequence activation and part-tree refresh | `GuidanceWindow.AutoSequence` | Call `SequenceList.ActivateNextSequence` and `Vehicle.UpdateAfterPartTreeModification`. Staging is not a reversible setting restored on release. |

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
