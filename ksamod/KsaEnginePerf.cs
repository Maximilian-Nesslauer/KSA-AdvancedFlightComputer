using System;
using Brutal.Numerics;
using KSA;

// Vehicle-wide propulsion totals for the main engines.
//
// KSA's FlightComputer.VehicleConfigInfo used to expose TotalEngineVacuumThrust
// and TotalEngineExhaustVelocity; the August 2026 build dropped both (it now
// carries only the RCS thrusters and gimbals), so we sum the same quantities
// ourselves. EngineController.VacuumData is the game's own precomputed vacuum
// performance for that controller's cores — thrust vector and max mass flow at
// full throttle — refreshed by PartTree whenever the part tree changes.
//
// The Vacuum* members are deliberately NOT filtered by EngineController.IsActive.
// The callers (G-FOLD planning, terminal hover) size a burn that may be commanded
// from a coast: both cut the engine when the commanded throttle falls near zero,
// so an active-only total would read zero exactly when the next solve needs it.
// They are vehicle-configuration figures, not live-thrust ones.
//
// They are also VACUUM figures, so they overstate thrust in any atmosphere. Use
// AtPressure for anything flown on a world with air — see the comment there.
internal static class KsaEnginePerf
{
    // Full-throttle vacuum thrust (N) and mass flow (kg/s) of the vehicle's
    // engines. Both zero if the vehicle has no engine controllers.
    internal static (double thrust, double massFlow) Vacuum(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return (0.0, 0.0);

        double thrust = 0.0, massFlow = 0.0;
        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            thrust += engines[i].VacuumData.ThrustMax.Length();
            massFlow += engines[i].VacuumData.MassFlowRateMax;
        }
        return (thrust, massFlow);
    }

    // Full-throttle vacuum thrust (N) alone, for callers that don't need Isp.
    internal static double VacuumThrust(Vehicle vehicle) => Vacuum(vehicle).thrust;

    // Effective exhaust velocity (m/s) = thrust / mass flow. Zero if the vehicle
    // has no engines, which callers treat as "nothing to plan with".
    internal static double VacuumExhaustVelocity(Vehicle vehicle)
    {
        (double thrust, double massFlow) = Vacuum(vehicle);
        return massFlow > 0.0 ? thrust / massFlow : 0.0;
    }

    // ---------------------------------------------------------------- pressure

    // Full-throttle thrust (N) and mass flow (kg/s) AT A GIVEN AMBIENT PRESSURE,
    // counting only the engines that will actually be lit.
    //
    // Vacuum() overstates thrust everywhere there is air — a booster's sea-level
    // thrust is typically ~80% of its vacuum figure, so a planner fed the vacuum
    // number believes in ~25% more thrust than exists. It never arrests as
    // planned, each re-solve starts lower than the last, and the trajectory
    // eventually curls into a loop because that is the only feasible shape left.
    // Vacuum figures being exactly right on an airless world is why this looked
    // like a Moon-works/Earth-fails bug rather than a thrust bug.
    //
    // RocketControllerData.ComputeFromCores is the game's own routine and is a
    // CAPABILITY at full throttle, not a measurement of current thrust. That
    // distinction matters: Vehicle.ComputeActiveThrust(p) reports what the engines
    // are producing right now, and feeding a live figure back in as Tmax builds a
    // loop — throttle down, Tmax reads low, the plan believes it has less
    // authority, it throttles down further.
    internal static (double thrust, double massFlow) AtPressure(Vehicle vehicle, double ambientPressure)
    {
        PartTree tree = vehicle?.Parts;
        FlightComputer fc = vehicle?.FlightComputer;
        if (tree == null || fc == null)
            return (0.0, 0.0);

        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        if (engines.Length == 0)
            return (0.0, 0.0);

        float3 com = fc.CenterOfMassAsmb;
        var p = (float)Math.Max(ambientPressure, 0.0);

        // Prefer the engines that are actually lit, but fall back to all of them
        // when none are. Both cases are real: a replan mid-burn wants the live
        // subset (an over-powered booster lands on some of its engines, and
        // counting the dark ones inflates Tmax), while the FIRST plan is built
        // before ignition, when an active-only sum would be zero and would fail
        // the solve outright.
        double thrust = 0.0, massFlow = 0.0;
        for (int pass = 0; pass < 2; pass++)
        {
            bool activeOnly = pass == 0;
            for (int i = 0; i < engines.Length; i++)
            {
                if (activeOnly && !engines[i].IsActive)
                    continue;
                // Cores is a RocketCore[] and can be null on a part that has not
                // finished building. Same reasoning as the atmosphere lookup above:
                // this runs from the ImGui draw, so an exception here does not
                // surface as a stack trace, it surfaces as a corrupted ImGui frame.
                RocketCore[] cores = engines[i].Cores;
                if (cores == null || cores.Length == 0)
                    continue;
                RocketControllerData d = RocketControllerData.ComputeFromCores(cores, com, p);
                thrust += d.ThrustMax.Length();
                massFlow += d.MassFlowRateMax;
            }
            if (thrust > 0.0 && massFlow > 0.0)
                break;
            thrust = massFlow = 0.0;
        }
        return (thrust, massFlow);
    }

    // The vehicle's thrust capability RIGHT NOW, in newtons: full throttle, current
    // ambient pressure, counting only engines that are lit and fed.
    //
    // This is the number a thrust command must be divided by. KSA's EngineThrottle
    // is a FRACTION of whatever the engines can currently produce, so dividing a
    // demand by anything else — a figure from plan time, a vacuum figure, a
    // different altitude's figure — delivers the wrong thrust by exactly that ratio.
    //
    // Safe to use as a divisor despite the name: ComputeActivePerformance calls
    // ComputeFromCores, whose ComputeConditions(1f) hardcodes FULL throttle, so this
    // is a capability and NOT scaled by the current throttle. There is therefore no
    // feedback loop in dividing by it. (It does return VacuumData when pressure <= 0,
    // so airless bodies are handled by the same call.) Zero if nothing is lit, which
    // the caller must treat as "no authority" rather than dividing by it.
    internal static double ActiveThrustCapability(Vehicle vehicle, double ambientPressure)
    {
        if (vehicle == null)
            return 0.0;
        float t = vehicle.ComputeActiveThrust((float)Math.Max(ambientPressure, 0.0));
        return double.IsFinite(t) && t > 0.0 ? t : 0.0;
    }

    // Thrust (N) the lit engines produce at an ARBITRARY throttle and pressure.
    //
    // THRUST IS NOT PROPORTIONAL TO THROTTLE IN AN ATMOSPHERE, which is why this has
    // to exist rather than scaling a full-throttle figure. Throttle sets COMBUSTION
    // PRESSURE (CombustorConfig.ComputeConditions: `throttle * CombustionPressureMax`
    // into a gas-property LUT), and nozzle thrust is momentum plus pressure:
    //
    //     F = mdot*Ve + (Pe - Pa)*Ae
    //
    // The momentum term scales with throttle. The ambient term -Pa*Ae does NOT — it
    // is the same at 30% throttle as at 100%. So
    //
    //     F(t) = t*F(1) - Pa*Ae*(1 - t)
    //
    // and the deficit is LARGEST at low throttle. Assuming linearity therefore
    // over-estimates delivered thrust by a near-constant amount, which is exactly
    // the signature the flight logs showed and which I first misread as a gravity
    // error, since a constant offset looks like one.
    internal static double ThrustAtThrottle(Vehicle vehicle, double throttle, double ambientPressure)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 0.0;
        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        if (engines.Length == 0)
            return 0.0;

        var th = (float)Math.Clamp(throttle, 0.0, 1.0);
        var p = (float)Math.Max(ambientPressure, 0.0);

        // Same active-first, fall-back-to-all rule as AtPressure, for the same reason.
        for (int pass = 0; pass < 2; pass++)
        {
            bool activeOnly = pass == 0;
            double total = 0.0;
            bool sawAny = false;
            for (int i = 0; i < engines.Length; i++)
            {
                if (activeOnly && !engines[i].IsActive)
                    continue;
                RocketCore[] cores = engines[i].Cores;
                if (cores == null)
                    continue;
                foreach (RocketCore core in cores)
                {
                    RocketNozzle[] nozzles = core?.Rocket?.Nozzles;
                    if (nozzles == null)
                        continue;
                    sawAny = true;
                    RocketCoreConditions cond = core.ComputeConditions(th);
                    foreach (RocketNozzle nz in nozzles)
                        total += nz.ComputePerformance(in cond, p).GetTotalThrust();
                }
            }
            if (sawAny)
                return total > 0.0 ? total : 0.0;
        }
        return 0.0;
    }

    /// <summary>
    /// The throttle that actually delivers <paramref name="demandN"/> newtons, by
    /// inverting the real thrust curve. Returns -1 if there is nothing to invert and
    /// the caller should fall back.
    ///
    /// Bisection rather than algebra: the curve is very nearly affine above the
    /// minimum throttle, but it is exactly zero below it, and inverting KSA's own
    /// function needs no assumption about shape at all.
    /// </summary>
    internal static double ThrottleForThrust(Vehicle vehicle, double demandN, double ambientPressure)
    {
        if (!(demandN > 0.0))
            return 0.0;
        double full = ThrustAtThrottle(vehicle, 1.0, ambientPressure);
        if (!(full > 1.0))
            return -1.0;
        if (demandN >= full)
            return 1.0;

        double lo = 0.0, hi = 1.0;
        for (int i = 0; i < 20; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (ThrustAtThrottle(vehicle, mid, ambientPressure) < demandN) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    // Ambient pressure (Pa) at an altitude above the body's sea level datum, or 0
    // on an airless world. Anything missing reads as vacuum, which is the safe
    // direction for every caller except a planner — see AtPressure's callers for
    // which altitude they choose.
    internal static double AmbientPressureAt(IParentBody parent, double altitudeAslM)
    {
        // EVERY link here is nullable and an airless body breaks the FIRST one.
        // AtmosphereReference is a class, so GetAtmosphereReference() returns null
        // on a body with no atmosphere — which is most of them, and is exactly the
        // case this function exists to report 0 for. Guarding only `parent` (as an
        // earlier version did) throws a NullReferenceException instead, and because
        // this is called from the ImGui draw the exception unwinds past the window's
        // End() and leaves ImGui mid-window: the game reports "missing End" and the
        // panel is unusable, with nothing pointing back at the atmosphere lookup.
        AtmosphereReference atmo = parent?.GetAtmosphereReference();
        PhysicalAtmosphereReference phys = atmo?.Physical;
        if (phys == null)
            return 0.0;

        // DO NOT gate this on phys.IsValid(). It requires ScaleHeight.IsValid(), and
        // DistanceReference.IsValid() is `Math.Abs(value) > 100000.0` — over 100 km.
        // Atmospheric scale heights are single-digit kilometres (Earth's is ~8.5 km),
        // so that predicate is FALSE for every atmosphere in the game. The check is
        // written for orbital distances and is simply the wrong test here.
        //
        // Trusting it made this function return 0 on every world with air, and 0 is
        // not a harmless answer: EngineController.ComputeActivePerformance returns
        // VacuumData when pressure <= 0, so both the planner's Tmax and the throttle
        // divisor silently became VACUUM thrust. The vehicle then planned against
        // thrust it did not have and under-throttled by the same ratio all the way
        // down — which is exactly the constant shortfall the flight logs showed.
        //
        // Validating the RESULT is the honest check: a body with no atmosphere has
        // zero or NaN sea-level pressure and lands on 0 here anyway, and a zero
        // scale height divides to infinity and is caught by IsFinite.
        double p = phys.GetAtmosphericPressureAtAltitude(altitudeAslM);
        return double.IsFinite(p) && p > 0.0 ? p : 0.0;
    }
}
