using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>
/// What the console knows about the subject craft in one frame: the craft itself, what AFC flies on
/// it, its status lines and the one stop. Built once per frame before anything draws, so the head,
/// the strip, the drawer, the page dots and the AFC menu all read the same answer.
/// </summary>
internal sealed class ConsoleFrame
{
    /// <summary>The controlled craft, or null in the editor, during an EVA, or before a flight has a craft.</summary>
    public Vehicle? Subject;
    public ModeReadout Mode;
    public bool GuidanceOn;
    public bool Editor;
    public bool Eva;
    public StopSpec Stop = StopSpec.None;

    /// <summary>
    /// Without a subject (the editor, an EVA, no craft): the stop of the one other craft AFC still
    /// flies, which the head shows in place of the subject's stop, and that craft's id. With several
    /// such craft both stay empty and their stops are in the drawer.
    /// </summary>
    public StopSpec OtherStop = StopSpec.None;
    public string OtherStopCraft = "";

    /// <summary>The multi-pass on the subject belongs to a transfer, so its page is TRANSFER and not ORBIT.</summary>
    public bool TransferPasses;
    public readonly List<StatusLine> Lines = new();

    public SituationFacts Situation;

    public CraftActivity Activity => Mode.Activity;

    public bool Idle => Subject != null && Mode.ModeName.Length == 0 && !Activity.StockAutoOnNode && Stop.Target == StopTarget.None;

    public bool SubjectBusy => Subject != null && Activity.AnyLive;

    public string FailureWhat = "";

    public string FailureDetail = "";

    public int FailureSerial;

    public bool FailureAnswered;

    /// <summary>The strip stays on screen while AFC flies, after a hand-back until the pilot acknowledged the band, and from there on while the craft still falls (FailureBand.StripStays).</summary>
    public bool StripWanted => FailureBand.StripStays(SubjectBusy, FailureWhat.Length > 0, FailureAnswered, in Situation);

    public bool FallAfterFailure => Subject != null && StripWanted && !SubjectBusy && FailureWhat.Length == 0;

    /// <summary>Who flies the subject now, for the chip at the left of the head.</summary>
    public string WhoFlies
    {
        get
        {
            CraftActivity a = Activity;
            return FlownBy.Now(in a);
        }
    }

    public void Clear()
    {
        Subject = null;
        Mode = default;
        GuidanceOn = false;
        Editor = false;
        Eva = false;
        Stop = StopSpec.None;
        OtherStop = StopSpec.None;
        OtherStopCraft = "";
        TransferPasses = false;
        Lines.Clear();
        Situation = default;
        FailureWhat = "";
        FailureDetail = "";
        FailureSerial = 0;
        FailureAnswered = false;
    }

    public void ReadSituation()
    {
        Vehicle? v = Subject;
        if (v == null)
            return;
        Situation = ReadFacts(v);
        if (!SituationText.IsFalling(Situation.Where))
            return;
        try
        {
            Situation = Situation with { HeightM = GuidanceWindow.GroundHeightM(v), ImpactS = GuidanceWindow.ImpactSeconds(v, GuidanceOn) };
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-fall:" + ex.GetType().Name, $"[AFC] Reading the fall failed: {ex.Message}");
        }
    }

    public BandFacts FailureFacts() => new(FailureWhat, GuidanceWindow.EngineNowOf(Subject!), WhoFlies,
        Situation.Where == CraftSituation.Ground ? double.NaN : GuidanceWindow.ImpactSeconds(Subject!, GuidanceOn));

    /// <summary>IParentBody.GetNearSurfaceRadius is the top of the atmosphere on a body with air and the highest terrain on any other.</summary>
    public static SituationFacts ReadFacts(Vehicle v)
    {
        try
        {
            IParentBody parent = v.Parent;
            Orbit orbit = v.Orbit;
            bool hasAir = parent.GetPhysicalAtmosphere() is { Height: > 0.0 };
            double radialSpeed = double3.Dot(orbit.StateVectors.VelocityCci, orbit.StateVectors.PositionCci);
            CraftSituation where = SituationText.Classify(v.Situation.HasAnyContact(), orbit.IsBound(), orbit.Periapsis, parent.GetNearSurfaceRadius(), radialSpeed, hasAir);
            return new SituationFacts(where, parent.Id, orbit.PeriapsisAltitude / 1000.0, orbit.ApoapsisAltitude / 1000.0, AnyEngineBurns(v), hasAir);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-situation:" + ex.GetType().Name, $"[AFC] Situation check failed: {ex.Message}");
            return default;
        }
    }

    public static bool EngineKnownOff(Vehicle? v)
    {
        if (v == null)
            return false;
        try
        {
            return !AnyEngineBurns(v);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-engine-off:" + ex.GetType().Name, $"[AFC] Engine check failed: {ex.Message}");
            return false;
        }
    }

    public static double? EnginePercent(Vehicle? v)
    {
        if (v == null)
            return null;
        try
        {
            return HighestThrottle(v) * 100.0;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-engine-percent:" + ex.GetType().Name, $"[AFC] Engine throttle read failed: {ex.Message}");
            return null;
        }
    }

    private static bool AnyEngineBurns(Vehicle v) => HighestThrottle(v) > 0f;

    // The throttle of a rocket core is what it burns now (RocketCore.UpdateState, SolidMotor.UpdateState), under the manual inputs and under stock AUTO alike.
    private static float HighestThrottle(Vehicle v)
    {
        if (v.Parts?.States == null
            || !ModuleStateful<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>.TryGetFrom(v.Parts.States, out var cores))
            return 0f;
        float highest = 0f;
        foreach (EngineController engine in v.Parts.Modules.Get<EngineController>())
        {
            foreach (RocketCore core in engine.Cores)
                highest = Math.Max(highest, cores.States[core.StatesIdx].Throttle);
        }
        return highest;
    }

    /// <summary>The stop of whatever runs on <paramref name="vehicle"/>, with its action. Shared by the head, the strip and the AFC menu.</summary>
    public static StopSpec StopFor(Vehicle vehicle, in CraftActivity a)
    {
        StopTarget target = StopResolver.Resolve(in a);
        if (target == StopTarget.None)
            return StopSpec.None;
        Action? act = StopAction(vehicle, target);
        return new StopSpec(target, StopResolver.Label(target, in a), StopResolver.OutcomeLine(target, in a),
            StopResolver.EngineOutcome(target, in a), act);
    }

    /// <summary>The stop call of each kind of running thing. The guidance stops call the abort entry point of the mode that runs.</summary>
    public static Action? StopAction(Vehicle vehicle, StopTarget target)
    {
        switch (target)
        {
            case StopTarget.SixDof:
            case StopTarget.Landing:
            case StopTarget.Hover:
            case StopTarget.Boostback:
            case StopTarget.Ascent:
            case StopTarget.LaunchArmed:
            case StopTarget.Registered:
                if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
                    return () => GuidanceWindow.RequestStop(vehicle, target);
                return () => GuidanceWindow.StopGuidance(vehicle, target);
            case StopTarget.Rcs:
                return () => RcsStop.Cancel(vehicle);
            case StopTarget.MultiPass:
                return () => MultiPassController.Cancel(vehicle);
            case StopTarget.StockAuto:
                return () => BurnModeInput.StopAuto(vehicle);
            default:
                return null;
        }
    }
}
