using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTranslation;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Runs AFC command writers after stock control and combines their physics and wake requests.
/// Each call has its own receipt because vehicle workers can run in parallel.
/// Writer failures are caught so the remaining receipt can still be applied.
/// </summary>
internal static class VehicleCommandSink
{
    internal static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(ComputeControlPatch)).Patch();
    }

    /// <summary>
    /// Records actuator writes and the requested time to the next control step, in seconds.
    /// The game's minimum control interval still limits when that step can run.
    /// </summary>
    internal ref struct Receipt
    {
        internal bool Commanded;
        internal double WakeupSec = double.PositiveInfinity;

        public Receipt()
        {
        }

        internal void Command() => Commanded = true;

        internal void Wake(double seconds)
        {
            if (seconds < WakeupSec)
                WakeupSec = seconds;
        }
    }

    internal static void Run(FlightComputer fc, in FlightComputerNavigation nav,
                             ref FlightComputerOutput outputs)
    {
        Receipt receipt = new();

        if (SharedVehicleHooks.RcsEnabled)
        {
            try
            {
                RcsComputeControlPatch.Command(fc, in nav, ref outputs, ref receipt);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce($"command-sink-rcs:{ex.GetType().Name}",
                    $"[AFC] RCS command writer failed on the vehicle worker: {ex}");
            }
        }

        if (SharedVehicleHooks.GuidanceEnabled)
        {
            try
            {
                KsaGimbalControl.OnComputeControl(fc, ref outputs, ref receipt);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce($"command-sink-guidance:{ex.GetType().Name}",
                    $"[AFC] Guidance gimbal writer failed on the vehicle worker: {ex}");
            }
        }

        if (receipt.Commanded)
            outputs.AnyActuatorCommanded = true;
        if (receipt.WakeupSec < outputs.NextWakeupDeltaTime)
            outputs.NextWakeupDeltaTime = receipt.WakeupSec;
    }

    [HarmonyPatch(typeof(FlightComputer), nameof(FlightComputer.ComputeControl),
        new Type[] { typeof(FlightComputerNavigation), typeof(ManualControlInputs), typeof(FlightComputerOutput) },
        new ArgumentType[] { ArgumentType.Ref, ArgumentType.Ref, ArgumentType.Ref })]
    private static class ComputeControlPatch
    {
        static void Postfix(FlightComputer __instance, in FlightComputerNavigation nav,
                            ref FlightComputerOutput outputs)
            => Run(__instance, in nav, ref outputs);
    }
}
