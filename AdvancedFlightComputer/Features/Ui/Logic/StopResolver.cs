using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>What the one stop ends. <see cref="Registered"/> is a mode that registered later than the built-in ones, named by CraftActivity.RegisteredModeLive.</summary>
internal enum StopTarget { None, SixDof, Landing, Hover, Boostback, Ascent, LaunchArmed, Rcs, MultiPass, StockAuto, Registered }

/// <summary>
/// Which running thing the one stop of the console ends. It follows what flies the craft, not the
/// page on screen: a guidance mode first, then an RCS burn, then a multi-pass, then a node that stock
/// AUTO flies. The guidance part reads the registered modes of <see cref="GuidanceModes"/> in their
/// stop order. A mode that registers later stops as <see cref="StopTarget.Registered"/>, with the
/// label and outcome of its descriptor.
/// </summary>
internal static class StopResolver
{
    public static StopTarget Resolve(in CraftActivity a)
    {
        foreach (GuidanceModeDescriptor mode in GuidanceModes.ByStopOrder)
        {
            if (mode.IsLive(a))
                return mode.Stop(a);
        }
        if (a.RcsFlying)
            return StopTarget.Rcs;
        if (a.MultiPassActive)
            return StopTarget.MultiPass;
        if (a.StockAutoOnNode)
            return StopTarget.StockAuto;
        return StopTarget.None;
    }

    public static bool IsGuidance(StopTarget target)
        => target is StopTarget.SixDof or StopTarget.Landing or StopTarget.Hover or StopTarget.Boostback
            or StopTarget.Ascent or StopTarget.LaunchArmed or StopTarget.Registered;

    // The descriptor behind StopTarget.Registered on this craft.
    private static GuidanceModeDescriptor? RegisteredMode(in CraftActivity a)
        => a.RegisteredModeLive is GuidanceModeId id ? GuidanceModes.Of(id) : null;

    /// <summary>The stop label, with the label a registered mode gave for <see cref="StopTarget.Registered"/>.</summary>
    public static string Label(StopTarget target, in CraftActivity a)
        => target == StopTarget.Registered ? NonEmpty(RegisteredMode(in a)?.StopLabel, Label(target)) : Label(target);

    private static string NonEmpty(string? text, string fallback) => string.IsNullOrEmpty(text) ? fallback : text;

    public static string Label(StopTarget target) => target switch
    {
        StopTarget.SixDof => "STOP TOUCHDOWN",
        StopTarget.Landing => "STOP LANDING",
        StopTarget.Hover => "STOP HOVER",
        StopTarget.Boostback => "STOP BOOSTBACK",
        StopTarget.Ascent => "STOP ASCENT",
        StopTarget.LaunchArmed => "CANCEL LAUNCH",
        StopTarget.Rcs => "STOP RCS BURN",
        StopTarget.MultiPass => "STOP PASSES",
        StopTarget.StockAuto => "STOP STOCK AUTO",
        StopTarget.Registered => "STOP GUIDANCE",
        _ => "",
    };

    public static readonly string[] BuiltInLabels = Array.ConvertAll(Enum.GetValues<StopTarget>()[1..], Label);

    /// <summary>The engine is said to be cut only when the stop cuts it: guidance holds the controls and an engine burns, or stock AUTO flies the deorbit burn of a landing. The stop of a deorbit that stock AUTO flies also removes its burn (GuidanceWindow.ReleaseStockDeorbit). The stock engine command in MANUAL reads the manual switch, which Vehicle.PrepareWorker holds off while stock AUTO flies, so an end of stock AUTO leaves the engine off.</summary>
    public static string OutcomeShort(StopTarget target, in CraftActivity a) => target switch
    {
        StopTarget.LaunchArmed => "No launch, engine not touched",
        StopTarget.Ascent or StopTarget.Boostback or StopTarget.SixDof => !a.ControlAcquired ? "Engine not touched" : a.EngineOff ? "Engine stays off" : "Engine cut",
        StopTarget.Landing => a.LandingAirborne ? EngineKept(in a)
            : a.LandingStockAuto ? (a.EngineOff ? "Engine stays off, burn deleted" : "Engine cut, burn deleted")
            : a.ControlAcquired ? (a.EngineOff ? "Engine stays off" : "Engine cut")
            : "Engine not touched",
        StopTarget.Hover => EngineKept(in a),
        StopTarget.Rcs => "RCS stops, engine not touched",
        StopTarget.MultiPass => a.StockAutoOnNode ? "Deletes the passes left, engine off" : "Deletes the passes left",
        StopTarget.StockAuto => a.LandingStockAuto ? "Landing ends too, burn stays" : "Engine off, burn stays",
        StopTarget.Registered => NonEmpty(a.RegisteredStopOutcome, NonEmpty(RegisteredMode(in a)?.StopOutcome, "The mode ends")),
        _ => "",
    };

    private static string EngineKept(in CraftActivity a)
        => a.EngineOff ? "Engine stays off"
            : a.EnginePercent is double percent ? string.Create(Fmt.Inv, $"Engine stays at {percent:F0} %")
            : "Engine stays as it is";

    private static string EngineKeptSentence(in CraftActivity a)
        => a.EngineOff ? "Engine stays off. You fly, and the craft falls." : EngineKept(in a) + ". You fly.";

    private static string EngineHeld(in CraftActivity a)
        => !a.ControlAcquired ? "Engine not touched." : a.EngineOff ? "Engine stays off." : "Engine cut.";

    public static string OutcomeLine(StopTarget target, in CraftActivity a) => OnStop(OutcomeShort(target, in a));

    public static string OnStop(string outcome)
    {
        if (outcome.Length == 0)
            return "";
        bool sentence = outcome.Length > 1 && char.IsUpper(outcome[0]) && char.IsLower(outcome[1]);
        return "On stop: " + (sentence ? char.ToLowerInvariant(outcome[0]) + outcome.Substring(1) : outcome);
    }

    /// <summary>The full sentence for the stop tooltip.</summary>
    public static string EngineOutcome(StopTarget target, in CraftActivity a) => target switch
    {
        StopTarget.LaunchArmed => "The launch is canceled. Nothing has launched, so the engine is not touched.",
        StopTarget.Ascent => EngineHeld(in a) + " You fly.",
        StopTarget.Boostback => EngineHeld(in a) + (a.ControlAcquired ? " AFC stops steering. You fly." : " You fly."),
        StopTarget.SixDof => EngineHeld(in a) + (a.ControlAcquired ? " You fly, and the craft falls." : " You fly."),
        StopTarget.Landing => a.LandingAirborne ? EngineKeptSentence(in a)
            : a.LandingStockAuto ? (a.EngineOff ? "Engine stays off." : "Engine cut.") + " AFC removes the deorbit burn it made, unless you changed it."
            : a.ControlAcquired ? EngineHeld(in a) + " You fly."
            : "Engine not touched, because AFC does not hold the craft yet.",
        StopTarget.Hover => EngineKeptSentence(in a),
        StopTarget.Rcs => "RCS stops. Stock AUTO is not given back. The engine is not touched.",
        StopTarget.MultiPass => a.StockAutoOnNode
            ? "The queued pass and the passes after it are gone, so its stock AUTO burn ends and the engine goes off. Burns already flown stay."
            : "The queued pass and the passes after it are gone. Burns already flown stay.",
        StopTarget.StockAuto => a.LandingStockAuto
            ? "Stock AUTO goes back to MANUAL, so its burn ends and the engine goes off. The landing ends too, because it needs that burn. The burn stays in the plan."
            : "Stock AUTO goes back to MANUAL, so its burn ends and the engine goes off. The burn stays in the plan.",
        StopTarget.Registered => NonEmpty(a.RegisteredStopDetail,
            NonEmpty(RegisteredMode(in a)?.StopDetail, "Ends the mode that flies this craft.")),
        _ => "Nothing runs on this craft.",
    };

    public const string ReleaseOutcome =
        "Ends every AFC mode on this craft and cuts the engine. An engine that a stop left lit stays lit. Stock AUTO comes back with the cut.";

    public const string ReleaseIdleOutcome =
        "Nothing flies and guidance holds no control of this craft, so RELEASE only clears its guidance state. The engine is not touched.";

    /// <summary>The RELEASE tooltip. ReleaseVehicle cuts the engine only while guidance holds the controls, so an idle craft gets the plain text.</summary>
    public static string ReleaseOutcomeFor(in CraftActivity a)
        => a.AnyGuidanceLive || a.ControlAcquired ? ReleaseOutcome : ReleaseIdleOutcome;

    /// <summary>RELEASE ALL GUIDANCE shows only while it undoes something: a guidance mode flies, guidance still holds the controls, or guidance state is left to free.</summary>
    public static bool NothingToRelease(in CraftActivity a) => !a.AnyGuidanceLive && !a.ControlAcquired && !a.ReleaseWouldAct;
}
