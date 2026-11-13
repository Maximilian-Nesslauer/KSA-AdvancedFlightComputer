using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Transfer;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>Where the follow-up of a flown flyby departure stands.</summary>
internal enum FlybyCourseStage
{
    Idle,
    /// <summary>The departure that CREATE made is planned or underway.</summary>
    Departure,
    /// <summary>A burn is done and the flown orbit waits for its check against the periapsis.</summary>
    Check,
    /// <summary>A correction node is planned or underway.</summary>
    Correction,
}

/// <summary>
/// Follows the single flyby departure that CREATE made to a moon of the craft's parent. Once stock reports the burn complete, the periapsis on the craft's own flight plan is compared with the requested one, and when it is off a correction node is added (CourseCorrector), which is checked the same way after it is flown.
/// It writes only the burn plan. A correction node is loaded in Manual and is the player's to fly or delete, like the departure, and nothing is added while the player has a burn of their own planned.
/// The follow-up ends without a trace when the player removes or changes the burn it waits for, and with a save load. One craft is followed at a time, the one of the last CREATE.
/// </summary>
internal static class FlybyCourse
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // A burn that stock has not reported complete this long after its node time is taken as flown by hand or left, s.
    private const double BurnOverdueS = 1800.0;

    // CREATE queues its node through InputEvents.BurnUpdateBuffer, which Program.PrepareFrame drains after Universe.ApplyVehicleSolvers, so the step that follows the click still sees a plan without it. The wait counts steps and not seconds, because at a high simulation speed one frame is many seconds. A node that is not in the plan after this many steps was never added.
    private const int NodeArrivalSteps = 3;

    private static Vehicle? _vehicle;
    private static Celestial? _moon;
    private static double _altitudeM;
    private static Burn? _node;
    private static double _nodeTime;
    private static double3 _nodeDv;
    private static bool _nodeSeen;
    private static int _stepsWithoutNode;
    private static double _checkAfter;
    private static int _stepsInCheck;

    internal static FlybyCourseStage Stage { get; private set; }

    /// <summary>The corrections planned since the departure.</summary>
    internal static int Corrections { get; private set; }

    /// <summary>What the last check found. The TRANSFER footer shows it while a correction node waits, and the final result stays here for the log and the alert. Empty before the first check.</summary>
    internal static string LastResult { get; private set; } = "";

    internal static bool Watching => Stage != FlybyCourseStage.Idle;

    /// <summary>True while the follow-up of <paramref name="vehicle"/> has something to say on the TRANSFER footer.</summary>
    internal static bool Follows(Vehicle vehicle) => Watching && ReferenceEquals(vehicle, _vehicle);

    /// <summary>
    /// Called by CREATE for the single flyby departure it made. Only a destination that orbits the craft's own parent is followed, because the corrector aims within one sphere of influence.
    /// </summary>
    internal static void OnDepartureCreated(Vehicle vehicle, IOrbiter? destination, double periapsisRadius, Burn node)
    {
        Reset();
        if (destination is not Celestial moon || moon is not IParentBody moonBody
            || moon.Orbit?.Parent == null || vehicle.Orbit?.Parent == null || moon.Orbit.Parent.Id != vehicle.Orbit.Parent.Id)
            return;
        _vehicle = vehicle;
        _moon = moon;
        _altitudeM = periapsisRadius - moonBody.MeanRadius;
        Watch(node, FlybyCourseStage.Departure);
    }

    private static void Watch(Burn node, FlybyCourseStage stage)
    {
        _node = node;
        _nodeTime = node.ImpulseTime.Seconds();
        _nodeDv = node.DeltaVVlf;
        _nodeSeen = false;
        _stepsWithoutNode = 0;
        Stage = stage;
    }

    /// <summary>The completion of a stock burn, from FlightComputer.RaisePendingAlerts. The burn can already be out of the plan here.</summary>
    internal static void OnBurnCompleted(Vehicle vehicle, Burn burn)
    {
        if (!ReferenceEquals(vehicle, _vehicle) || !ReferenceEquals(burn, _node)
            || (Stage != FlybyCourseStage.Departure && Stage != FlybyCourseStage.Correction))
            return;
        Stage = FlybyCourseStage.Check;
        _checkAfter = Universe.GetElapsedSeconds() + CourseCorrector.SettleS;
        _stepsInCheck = 0;
    }

    /// <summary>One step for <paramref name="vehicle"/>, on the main thread after the vehicle workers have been joined (the Universe.ApplyVehicleSolvers postfix).</summary>
    internal static void TickVehicle(Vehicle vehicle)
    {
        if (!ReferenceEquals(vehicle, _vehicle) || Stage == FlybyCourseStage.Idle)
            return;
        try
        {
            if (Stage == FlybyCourseStage.Check)
                CheckCourse(vehicle);
            else
                WatchBurn(vehicle);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("flyby-course:" + ex.GetType().Name, $"[AFC] Flyby course follow-up failed on '{vehicle.Id}': {ex}");
            Reset();
        }
    }

    // A node that the player removed or changed is no longer the burn that was planned here, and a burn nobody reports complete was flown by hand or left. Either ends the follow-up.
    private static void WatchBurn(Vehicle vehicle)
    {
        Burn node = _node!;
        double now = Universe.GetElapsedSeconds();
        bool planned = vehicle.FlightComputer.BurnPlan.TryGetBurn(node);
        _nodeSeen |= planned;
        if (!planned)
            _stepsWithoutNode++;
        bool gone = !planned && (_nodeSeen || _stepsWithoutNode >= NodeArrivalSteps);
        bool changed = Math.Abs(node.ImpulseTime.Seconds() - _nodeTime) > 0.001 || (node.DeltaVVlf - _nodeDv).Length() > 0.001;
        if (gone || changed || now > _nodeTime + BurnOverdueS)
            Reset();
    }

    private static void CheckCourse(Vehicle vehicle)
    {
        double now = Universe.GetElapsedSeconds();
        // The flight plan of the flown orbit arrives with the vehicle worker results of later frames, so the wait also counts steps, because at a high simulation speed the settle time is over within one frame.
        if (++_stepsInCheck < CourseCorrector.SettleSteps || now < _checkAfter)
            return;
        // A craft that an AFC controller flies by now is not given a node, and its burn plan is left as it is.
        ControlClaimant holder = VehicleControlOwnership.HolderOf(vehicle);
        if (holder != ControlClaimant.None)
        {
            DefaultCategory.Log.Info($"[AFC] FlybyCourse: vehicle='{vehicle.Id}' no course check, because {VehicleControlOwnership.Describe(holder)} has the craft.");
            Reset();
            return;
        }
        Celestial moon = _moon!;
        var moonBody = (IParentBody)moon;
        string moonName = moon.Id.ToUpperInvariant();
        FlightComputer fc = vehicle.FlightComputer;
        // FlightComputer.EndAutoBurn removes a completed node only when another burn follows it, so the flown node can still be the one node of the plan.
        if (fc.BurnPlan.BurnCount == 1 && _node != null && fc.BurnPlan.TryGetBurn(_node) && fc.BurnMode != FlightComputerBurnMode.Auto)
            fc.RemoveBurn(_node);
        _node = null;
        if (vehicle.Orbit?.Parent == null || vehicle.Orbit.Parent.Id != moon.Orbit.Parent.Id || CourseCorrector.BurnPlanned(vehicle))
        {
            Reset();
            return;
        }

        double predicted = MoonInjectionPlanner.PredictedPeriapsisRadius(vehicle, moonBody);
        string course = double.IsNaN(predicted)
            ? $"NO ENCOUNTER WITH {moonName}"
            : string.Create(Inv, $"PERIAPSIS AT {moonName} {(predicted - moonBody.MeanRadius) / 1000.0:F0} KM");
        string asked = string.Create(Inv, $"{_altitudeM / 1000.0:F0} KM ASKED");
        if (CourseCorrector.OnCourse(predicted, moonBody, _altitudeM))
        {
            Finish(vehicle, $"FLYBY ON COURSE: {course}, {asked}.", warn: false);
            return;
        }
        if (Corrections >= CourseCorrector.MaxCorrections)
        {
            Finish(vehicle, $"FLYBY OFF COURSE AFTER {Corrections} CORRECTIONS: {course}, {asked}.", warn: true);
            return;
        }

        string refusal = CourseCorrector.TryPlan(vehicle, moon, moonBody, _altitudeM, now, out MoonInjectionPlanner.MoonInjection plan);
        Burn? node = null;
        if (refusal.Length == 0)
            refusal = CourseCorrector.TryAddNode(vehicle, plan.BurnTime, plan.DvVlf, out node);
        if (refusal.Length > 0 || node == null)
        {
            Finish(vehicle, $"FLYBY OFF COURSE: {course}, {asked}. NO CORRECTION: {refusal}.", warn: true);
            return;
        }
        Corrections++;
        Watch(node, FlybyCourseStage.Correction);
        double plannedKm = (plan.PredictedPeriapsisRadius - moonBody.MeanRadius) / 1000.0;
        LastResult = string.Create(Inv, $"{course}, {asked}. CORRECTION {plan.DvVlf.Length():F1} M/S PLANNED.");
        AfcAlerts.Info($"Flyby: {LastResult} Fly it like the departure.", Color.Yellow, 8.0, vehicle);
        DefaultCategory.Log.Info(string.Create(Inv,
            $"[AFC] FlybyCourse: vehicle='{vehicle.Id}' {course}, {asked}; correction {Corrections} at {plan.BurnTime.Seconds():F1} s, delta-v {plan.DvVlf.Length():F2} m/s, predicted periapsis {plannedKm:F1} km."));
    }

    private static void Finish(Vehicle vehicle, string result, bool warn)
    {
        int corrections = Corrections;
        Reset();
        LastResult = result;
        AfcAlerts.Info(result, warn ? Color.Yellow : Color.Cyan, 8.0, vehicle);
        DefaultCategory.Log.Info($"[AFC] FlybyCourse: vehicle='{vehicle.Id}' after {corrections} correction(s): {result}");
    }

    internal static void OnVehicleDisposed(Vehicle vehicle)
    {
        if (ReferenceEquals(vehicle, _vehicle))
            Reset();
    }

    internal static void Reset()
    {
        _vehicle = null;
        _moon = null;
        _node = null;
        _altitudeM = 0.0;
        _nodeTime = double.NaN;
        _nodeDv = default;
        _nodeSeen = false;
        _stepsWithoutNode = 0;
        _checkAfter = double.NaN;
        _stepsInCheck = 0;
        Stage = FlybyCourseStage.Idle;
        Corrections = 0;
        LastResult = "";
    }
}
