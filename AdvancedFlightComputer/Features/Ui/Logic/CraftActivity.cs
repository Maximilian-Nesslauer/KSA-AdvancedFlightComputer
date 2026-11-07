namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// What AFC does on one craft right now, as plain flags. The game side fills it from the craft's
/// guidance state, the multi-pass and RCS registries and the stock burn mode, so the stop, commit and
/// status rules can be decided and tested without the game.
/// </summary>
internal readonly record struct CraftActivity
{
    public bool AscentRunning { get; init; }
    public bool LaunchArmed { get; init; }
    public bool ConvexLaunchPending { get; init; }
    public bool BoostbackLive { get; init; }
    public bool DescentLive { get; init; }
    public bool LandingMachineLive { get; init; }
    public bool TerminalBurnActive { get; init; }
    public bool HoverLive { get; init; }
    public bool SixDofEngaged { get; init; }
    public bool LandingAirborne { get; init; }
    public bool ControlAcquired { get; init; }

    public bool HoldsNoControl { get; init; }

    public bool EngineOff { get; init; }

    /// <summary>The highest throttle an engine of the craft burns at now, in percent, or null when it cannot be read.</summary>
    public double? EnginePercent { get; init; }

    public bool LandingStockAuto { get; init; }
    public bool MultiPassActive { get; init; }
    public bool RcsFlying { get; init; }

    /// <summary>The stock flight computer flies a planned node in AUTO. AFC does not fly it, but the one stop covers it.</summary>
    public bool StockAutoOnNode { get; init; }

    /// <summary>A release still frees something on this craft: guidance state that outlives a mode, such as a claimed deorbit node, a worker or the staging guidance armed.</summary>
    public bool ReleaseWouldAct { get; init; }

    /// <summary>
    /// The mode that registered through GuidanceModes.Registered and flies this craft, or null. The
    /// built-in modes have their own flags above; a mode that registers later reports itself here, so
    /// the stop, the commit rules and the head need no new flag for it.
    /// </summary>
    public GuidanceModeId? RegisteredModeLive { get; init; }

    /// <summary>
    /// The short stop outcome and the full sentence of the mode in <see cref="RegisteredModeLive"/>
    /// when they depend on what that mode does now, such as whether its engine burns. Null or empty
    /// falls back to the texts of its descriptor.
    /// </summary>
    public string? RegisteredStopOutcome { get; init; }
    public string? RegisteredStopDetail { get; init; }

    /// <summary>A landing flies that has not reached the powered descent or the hover yet: the deorbit and braking chain.</summary>
    public bool DeorbitChainLive => DescentLive && !LandingMachineLive && !HoverLive;

    public bool LandingFlies => BoostbackLive || DescentLive || HoverLive || SixDofEngaged || RegisteredModeLive != null;

    public bool AnyGuidanceLive => AscentRunning || LaunchArmed || ConvexLaunchPending
        || BoostbackLive || DescentLive || SixDofEngaged || RegisteredModeLive != null;

    public bool PilotHolds => AnyGuidanceLive && HoldsNoControl;

    /// <summary>Something AFC flies or holds on this craft: a guidance mode, a multi-pass or an RCS burn.</summary>
    public bool AnyLive => AnyGuidanceLive || MultiPassActive || RcsFlying;
}
