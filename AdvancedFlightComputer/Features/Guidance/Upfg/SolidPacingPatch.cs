#nullable disable

using System;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

/// <summary>
/// Paces a solid motor in the staging drain simulation at the mean mass flow of the burn it has
/// left.
///
/// <c>SequencePerformanceList.ComputeSolidPacingMassFlowRate</c> divides the usable grain by the
/// burn time the thrust curve reports for a grain that fills its geometry, and that divisor never
/// shortens. A grain holds less when it is partly full, or while its <c>GrainVolume</c> is still
/// sized from <c>GrainGeometryLibrary.Default</c>, which only <c>SolidGrainSegment.Refill</c>
/// resizes. The postfix replaces only the divisor, with the burn left on the curve.
/// </summary>
internal static class SolidPacingPatch
{
    private const double G0 = 9.80665;

    private const int CurvePoints = 65;

    /// <summary>True while the postfix is installed, so the adapter's own correction stands down.</summary>
    internal static bool Active { get; private set; }

    // Sampling a curve solves the chamber pressure 256 times, and the drain simulation can run every
    // physics step.
    private static ConditionalWeakTable<SolidMotor, GrainCurve> _curves = new();

    internal static void Apply(Harmony harmony)
    {
        harmony.Patch(GameReflection.SequencePerformanceList_ComputeSolidPacingMassFlowRate!,
            postfix: new HarmonyMethod(typeof(SolidPacingPatch), nameof(Postfix)));
        Active = true;
    }

    internal static void Disable()
    {
        Active = false;
        _curves = new ConditionalWeakTable<SolidMotor, GrainCurve>();
    }

    // Stock returns ignitionMassFlowRate when it gives up, and then its pacing stands.
    private static void Postfix(SolidMotor solid, float ignitionMassFlowRate, ref float __result)
    {
        try
        {
            if (__result != ignitionMassFlowRate && TryPacingMassFlowRate(solid, __result, out float flow))
                __result = flow;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"solid-pacing:{ex.GetType().Name}",
                $"[AFC] Solid pacing read failed, the game's own pacing stands: {ex}");
        }
    }

    private static bool TryPacingMassFlowRate(SolidMotor solid, float stockPacing, out float flow)
    {
        flow = 0f;
        if (!(stockPacing > 0f) || !TryCurve(solid, out GrainCurve curve))
            return false;

        // The drain simulation drains its own copy of the mole masses per sequence index, in a span
        // parameter the postfix does not declare, so the simulated grain comes from the stock result.
        double usable = (double)stockPacing * curve.BurnSeconds;
        double left = curve.SecondsLeft(usable);
        if (!(left > 0.0))
            return false;

        double paced = usable / left;
        if (!double.IsFinite(paced) || !(paced > 0.0))
            return false;

        flow = (float)paced;
        return true;
    }

    /// <summary>Reads the live grain, not the simulated grain the pacing uses.</summary>
    internal static bool TryBurnSecondsLeft(SolidMotor solid, out double seconds)
    {
        seconds = 0.0;
        if (!TryUsableGrainMass(solid, out double usable) || !TryCurve(solid, out GrainCurve curve))
            return false;
        seconds = curve.SecondsLeft(usable);
        return seconds > 0.0;
    }

    internal static bool TryBurnEndSeconds(SolidMotor solid, out double seconds)
    {
        seconds = 0.0;
        if (!TryCurve(solid, out GrainCurve curve))
            return false;
        seconds = curve.BurnEndSeconds;
        return seconds > 0.0;
    }

    internal static bool TryUsableGrainMass(SolidMotor solid, out double usable)
    {
        usable = 0.0;
        Rocket rocket = solid?.Rocket;
        PartTree tree = rocket?.Parent?.FullPart?.Tree;
        if (tree?.Moles == null || !solid.Stack.IsValid)
            return false;

        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        SolidGrainSegment[] segments = solid.Stack.Segments;
        for (int i = 0; i < segments.Length; i++)
        {
            Mole grain = segments[i].Grain;
            if (grain != null)
                usable += Math.Max(0.0, moles[grain.StatesIdx].Mass - segments[i].UnburnableGrainMass);
        }
        return usable > 0.0;
    }

    private static bool TryCurve(SolidMotor solid, out GrainCurve curve)
    {
        curve = null;
        if (solid == null || !solid.Stack.IsValid || solid.Rocket?.Nozzles == null
            || !(solid.InitialBurnableGrainMass > 0f))
            return false;
        if (_curves.TryGetValue(solid, out curve) && curve.Describes(solid))
            return true;
        if (!TryBuildCurve(solid, out curve))
            return false;

        // The drain simulation also runs from a job, so another thread can add the key first.
        _curves.AddOrUpdate(solid, curve);
        return true;
    }

    private static bool TryBuildCurve(SolidMotor solid, out GrainCurve curve)
    {
        curve = null;
        Span<float> thrust = stackalloc float[CurvePoints];
        Span<float> isp = stackalloc float[CurvePoints];
        Span<float> pressure = stackalloc float[CurvePoints];
        var samples = new SolidMotor.ThrustCurveSamples
        {
            ThrustNewtons = thrust,
            IspSeconds = isp,
            ChamberPressurePascals = pressure,
        };
        if (!solid.TrySampleThrustCurve(samples, out SolidMotor.ThrustCurvePreview preview)
            || !(preview.BurnSeconds > 0f))
            return false;

        // The samples are on an even time grid, so their integrated mass flow maps grain burned to time.
        float[] burned = new float[CurvePoints];
        double stepSeconds = (double)preview.BurnSeconds / (CurvePoints - 1);
        double running = 0.0;
        double previous = MassFlowRate(thrust[0], isp[0]);
        for (int i = 1; i < CurvePoints; i++)
        {
            double next = MassFlowRate(thrust[i], isp[i]);
            running += 0.5 * (previous + next) * stepSeconds;
            burned[i] = (float)running;
            previous = next;
        }
        if (!(running > 0.0))
            return false;

        curve = new GrainCurve(preview.BurnSeconds, burned, solid.InitialBurnableGrainMass, solid.AreaRatio);
        return true;
    }

    private static double MassFlowRate(double thrustNewtons, double ispSeconds) =>
        ispSeconds > 0.0 ? thrustNewtons / (ispSeconds * G0) : 0.0;

    private sealed class GrainCurve
    {
        // Grain burned in kg at each time step of the reported burn.
        private readonly float[] _burnedByStep;
        private readonly float _burnableMass;
        private readonly float _areaRatio;
        private readonly double _stepSeconds;

        internal float BurnSeconds { get; }

        // Where the curve has burned the burnable grain, or BurnSeconds when it never does.
        internal double BurnEndSeconds { get; }

        internal GrainCurve(float burnSeconds, float[] burnedByStep, float burnableMass, float areaRatio)
        {
            BurnSeconds = burnSeconds;
            _burnedByStep = burnedByStep;
            _burnableMass = burnableMass;
            _areaRatio = areaRatio;
            _stepSeconds = (double)burnSeconds / (burnedByStep.Length - 1);
            BurnEndSeconds = ElapsedSeconds(burnableMass);
        }

        // A part tree edit to the grain, the propellant or the nozzles changes one of these two values.
        internal bool Describes(SolidMotor solid) =>
            _burnableMass == solid.InitialBurnableGrainMass && _areaRatio == solid.AreaRatio;

        internal double SecondsLeft(double usableGrainMass)
        {
            if (!(_burnableMass > 0f) || !(BurnSeconds > 0f))
                return 0.0;
            // Time is linear in grain within a step, so grain over time left tends to that step's
            // flow at burnout and needs no floor.
            double burned = Math.Max(0.0, _burnableMass - Math.Max(usableGrainMass, 0.0));
            double left = BurnEndSeconds - ElapsedSeconds(burned);
            return left > 0.0 ? left : 0.0;
        }

        private double ElapsedSeconds(double burnedGrainMass)
        {
            int last = _burnedByStep.Length - 1;
            for (int i = 1; i <= last; i++)
            {
                if (burnedGrainMass > _burnedByStep[i])
                    continue;
                double step = _burnedByStep[i] - _burnedByStep[i - 1];
                double within = step > 0.0 ? (burnedGrainMass - _burnedByStep[i - 1]) / step : 0.0;
                return (i - 1 + within) * _stepSeconds;
            }
            return BurnSeconds;
        }
    }
}
