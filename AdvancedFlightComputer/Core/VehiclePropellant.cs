using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Whether engines can still draw propellant, judged per rocket the way <c>Rocket.UpdateRockets</c> judges it.
/// The stock answers on the module states only cover active engines, because <c>Rocket.UpdateRockets</c> writes <c>EngineControllerState.IsPropellantAvailable</c> false for every inactive one, while the per-rocket answer exists for every rocket, lit or not.
/// </summary>
internal static class VehiclePropellant
{
    /// <summary>
    /// True when any rocket of the engine can draw propellant. <paramref name="burning"/> says whether a rocket is lit, and <paramref name="broken"/> whether a solid motor's grain stack failed to resolve, which leaves it without propellant for the life of the vehicle.
    /// The caller runs <c>PartTree.EnsureDerived(DerivedData.SolidMotorStacks)</c> first, because a solid motor reads its stack.
    /// </summary>
    internal static bool IsFueled(EngineController engine,
        ReadOnlySpan<MoleState> moleStates, ReadOnlySpan<RocketState> rocketStates,
        out bool burning, out bool broken)
    {
        bool fueled = false;
        burning = false;
        broken = false;
        foreach (Rocket rocket in engine.Rockets)
        {
            // Mirrors Rocket.UpdateRockets: a rocket burns above zero throttle.
            // A lit solid motor counts its remaining grain as propellant, while a quenched one falls back to the equilibrium-pressure check and reads as spent.
            bool isBurning = rocketStates[rocket.StatesIdx].Throttle > 0f;
            burning |= isBurning;
            fueled |= IsFueled(rocket, moleStates, isBurning);
            foreach (RocketCore core in rocket.Cores)
                broken |= core is SolidMotor { Stack.IsValid: false };
        }
        return fueled;
    }

    /// <summary>
    /// True when every core of the rocket can draw propellant, the answer of the rocket's own <c>UpdatePropellantAvailability</c>, which is internal: a gas generator cycle runs only while both its chamber and its gas generator are fed.
    /// </summary>
    internal static bool IsFueled(Rocket rocket, ReadOnlySpan<MoleState> moleStates, bool burning)
    {
        foreach (RocketCore core in rocket.Cores)
        {
            if (!core.ComputePropellantAvailable(moleStates, burning))
                return false;
        }
        return true;
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
        ReadOnlySpan<RocketState> rocketStates = RocketStates.Of(tree);

        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            if (IsFueled(engines[i], moleStates, rocketStates, out _, out _))
                return true;
        }

        if (!includeRcs)
            return false;
        Span<ThrusterController> thrusters = tree.Modules.Get<ThrusterController>();
        for (int i = 0; i < thrusters.Length; i++)
        {
            foreach (Rocket rocket in thrusters[i].Rockets)
            {
                if (IsFueled(rocket, moleStates, rocketStates[rocket.StatesIdx].Throttle > 0f))
                    return true;
            }
        }
        return false;
    }
}
