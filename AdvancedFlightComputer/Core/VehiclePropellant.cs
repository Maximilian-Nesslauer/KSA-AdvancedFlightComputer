using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Whether engines can still draw propellant, judged per core the way <c>Rocket.UpdateRockets</c> judges it.
/// The stock answers on the module states only cover active engines, because <c>Rocket.UpdateRockets</c> writes <c>EngineControllerState.IsPropellantAvailable</c> false for every inactive one, while the per-core answer exists for every core, lit or not.
/// </summary>
internal static class VehiclePropellant
{
    /// <summary>
    /// True when any core of the engine can draw propellant. <paramref name="burning"/> says whether a core is lit, and <paramref name="broken"/> whether a solid motor's grain stack failed to resolve, which leaves it without propellant for the life of the vehicle.
    /// The caller runs <c>PartTree.EnsureDerived(DerivedData.SolidMotorStacks)</c> first, because a solid motor reads its stack.
    /// </summary>
    internal static bool IsFueled(EngineController engine,
        ReadOnlySpan<MoleState> moleStates, ReadOnlySpan<RocketCoreState> coreStates,
        out bool burning, out bool broken)
    {
        bool fueled = false;
        burning = false;
        broken = false;
        foreach (RocketCore core in engine.Cores)
        {
            // Mirrors Rocket.UpdateRockets: a core burns above zero throttle.
            // A lit solid motor counts its remaining grain as propellant, while a quenched one falls back to the equilibrium-pressure check and reads as spent.
            bool isBurning = coreStates[core.StatesIdx].Throttle > 0f;
            burning |= isBurning;
            fueled |= core.ComputePropellantAvailable(moleStates, isBurning);
            broken |= core is SolidMotor { Stack.IsValid: false };
        }
        return fueled;
    }

    /// <summary>
    /// True when any engine, lit or not, or with <paramref name="includeRcs"/> any RCS thruster, can still draw propellant.
    /// An engine the player has not staged yet counts, because staging or switching it on lets the craft thrust again.
    /// </summary>
    internal static bool AnyUsable(Vehicle vehicle, bool includeRcs)
    {
        PartTree tree = vehicle.Parts;
        tree.EnsureDerived(DerivedData.SolidMotorStacks);
        ReadOnlySpan<MoleState> moleStates = tree.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = tree.RocketCores.States;

        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            if (IsFueled(engines[i], moleStates, coreStates, out _, out _))
                return true;
        }

        if (!includeRcs)
            return false;
        Span<ThrusterController> thrusters = tree.Modules.Get<ThrusterController>();
        for (int i = 0; i < thrusters.Length; i++)
        {
            foreach (RocketCore core in thrusters[i].Cores)
            {
                if (core.ComputePropellantAvailable(moleStates, coreStates[core.StatesIdx].Throttle > 0f))
                    return true;
            }
        }
        return false;
    }
}
