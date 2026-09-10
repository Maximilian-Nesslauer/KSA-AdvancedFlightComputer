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
| `FlightComputer.RCSMode` | `RcsExecutor.ForceRcsOn` and `RestoreRcsMode` | Enable RCS when needed and restore Disabled if AFC changed it. |
| `FlightComputer.BurnMode` | `RcsExecutor.ForceBurnManual` and `RestoreBurnMode` | Set Manual at acquisition. Restore an earlier Auto only after cleanup succeeds and while the field still matches Manual. Explicit stops and completion discard the saved Auto. This is not a periodic Manual write. |
| `FlightComputer.AttitudeMode`, `AttitudeFrame`, `AttitudeTrackTarget` and `CustomAttitudeTarget` | `RcsExecutor.CommandAlignAttitude` | Select Auto and a burn-relative target for Align. Non-X axes use custom Euler angles. |
| `FlightComputer.AttitudeTrackTarget`, `AttitudeFrame` and `AttitudeTarget` | `RcsExecutor.EndExecution` through `FlightComputer.SetNullRot` | Select None in BurnBody and zero the computed target. This does not restore the previous mode, frame or custom coordinates. |
| Navball frame | `RcsExecutor.BeginControl` through `Vehicle.SetNavBallFrame` | Select BurnBody. Specific SetEnum cancellation and activation-failure paths select the vehicle-region frame; generic release does not restore a captured frame. |

RCS attitude release has a known gap.
`SetNullRot` does not restore `AttitudeMode` or clear `CustomAttitudeTarget`.
After a custom-axis Align, `FlightComputer.UpdateAttitudeTarget` can interpret the retained Euler angles as a rotation-rate command because tracking is now None.
Selecting None therefore does not by itself establish a neutral rotation command.

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
