using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// The quick orbit tools: circularize, set apoapsis, set periapsis, match inclination and set
/// inclination, each with its passes. The MANEUVER page of the AFC console hosts them. A frame first
/// prepares the plan from the planning basis of the craft the tool plans for (so the picture at the top
/// of the page and the footer read the same maneuver), then draws the form. The commit creates the
/// node or starts a multi-pass the same way for every tool.
/// </summary>
internal static partial class OrbitToolsPanel
{
    /// <summary>False when the ORBIT tools block failed, so the page says the tools are unavailable.</summary>
    internal static bool Enabled;

    internal static string ToolKey { get; private set; } = ManeuverTools.KeyCircularizeApoapsis;

    internal static Vehicle? Source { get; private set; }

    private static bool _toolPicked;
    private static bool _startPending = true;
    private static SeedPhase _startPhase;
    private static bool _madeLastFrame;

    internal static bool ToolPicked => _toolPicked;

    internal static bool PreviewVisible;

    internal static bool PlanKept;

    private static Burn? _ourBurn;
    private static OrbitalTransfers.PorkChopEntry? _lastEntry;
    private static Vehicle? _lastSource;
    private static bool _showOrbitPreview;

    private static OrbitManeuvers.ManeuverResult? _refined;
    private static Orbit? _refinedOrbit;
    private static OrbitManeuvers.ManeuverResult _refinedImpulsive;
    private static FiniteBurnGoal _refinedGoal;
    // FlightComputer.SetStagedBurn keeps at most 16 phases.
    private static readonly FiniteBurnPhase[] _refinedPhases = new FiniteBurnPhase[16];
    private static int _refinedPhaseCount;

    /// <summary>
    /// What the MANEUVER page shows this frame: the maneuver of the tool from the planning basis as the engines fly it, its preview entry, the commit state, and the impulsive maneuver the passes are split from.
    /// </summary>
    internal readonly record struct Plan(
        OrbitManeuvers.ManeuverResult? Maneuver, PlanningBasis Basis, OrbitalTransfers.PorkChopEntry? Entry, CommitState State,
        OrbitManeuvers.ManeuverResult? Impulsive = null)
    {
        public bool Made => State is CommitState.NodeCreated or CommitState.MultiPassRunning;
    }

    internal static bool ShowOrbitPreview
    {
        get => _showOrbitPreview;
        set => _showOrbitPreview = value;
    }


    /// <summary>The node this page created and that is still in the plan, so the footer can fly or delete it.</summary>
    internal static Burn? OurBurn => _ourBurn;

    /// <summary>
    /// Selects a tool and a source. A change of either resets the inputs to the defaults of the new
    /// pair, because a target altitude typed for one craft or tool means nothing for another.
    /// <paramref name="force"/> resets them even without a change, as a shortcut from the orbit menu does.
    /// </summary>
    internal static void Select(string key, Vehicle? source, bool force = false)
    {
        if (!OrbitToolCatalog.IsTool(key))
            return;
        bool keyChanged = key != ToolKey;
        bool sourceChanged = !ReferenceEquals(source, Source);
        ToolKey = key;
        Source = source;
        if (force)
            _toolPicked = true;
        if (keyChanged || force)
            OrbitToolForm.OnTypeChanged();
        if (sourceChanged || force)
            OrbitToolForm.OnSourceChanged();
        if (keyChanged || sourceChanged || force)
            OnManeuverContextChanged();
    }

    internal static void Pick(string key, Vehicle? source)
    {
        _toolPicked = true;
        Select(key, source);
    }

    internal static void OnSubjectChanged(Vehicle subject)
    {
        _toolPicked = false;
        _startPending = true;
        Select(ToolKey, subject);
    }

    internal static bool ApplyStartTool(Vehicle source, CraftSituation where)
    {
        SeedPhase phase = ManeuverSeed.PhaseOf(where);
        if (phase != SeedPhase.Unknown && phase != _startPhase)
        {
            _startPhase = phase;
            _toolPicked = false;
            _startPending = true;
        }
        if (_toolPicked || !_startPending || _ourBurn != null || source.Orbit == null || MultiPassRegistry.Has(source.Id))
            return false;
        _startPending = false;
        string key = OrbitToolCatalog.StartKey(PlanningBasis.For(source).Orbit.Eccentricity);
        if (key == ToolKey)
            return false;
        Select(key, source);
        return true;
    }

    internal static string RunningToolKey(Vehicle source)
    {
        if (!MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec) || !OrbitToolCatalog.IsTool(exec.Intent.TypeKey))
            return ToolKey;
        if (!_toolPicked)
            Select(exec.Intent.TypeKey, source);
        return exec.Intent.TypeKey;
    }

    internal static void PlanNext()
    {
        _ourBurn = null;
        OrbitToolForm.OnTypeChanged();
    }

    internal static Plan Prepare(string key, Vehicle source, CraftSituation where)
    {
        if (source.Orbit == null)
            return default;
        _lastSource = source;
        CleanupStaleBurn(source);

        // The controls, calculation and preview must use the same planning trajectory.
        PlanningBasis basis = PlanningBasis.For(source);
        CommitState state = ResolveCommitState(source, key);
        var plan = new Plan(null, basis, null, state);
        if (_madeLastFrame && !plan.Made)
            OrbitToolForm.RecheckSeed();
        _madeLastFrame = plan.Made;
        if (plan.Made)
        {
            _lastEntry = null;
            ClearRefinement();
            return plan;
        }

        OrbitToolForm.ApplyInputs(key, source, basis, where);
        OrbitManeuvers.ManeuverResult? impulsive = ComputeManeuver(key, source, basis);
        if (impulsive == null)
        {
            _lastEntry = null;
            ClearRefinement();
            return plan;
        }
        // The single burn is created, previewed and reported as the engines fly it. The multi-pass planners split the impulsive maneuver and refine each pass themselves.
        OrbitManeuvers.ManeuverResult maneuver = RefineForFiniteBurn(key, source, basis, impulsive.Value);
        _lastEntry = BuildTransferEntry(source, maneuver, basis);
        return new Plan(maneuver, basis, _lastEntry, state, impulsive);
    }

    internal static void DrawForm(string key, Vehicle source, in Plan plan, CraftSituation where)
    {
        if (plan.Made)
        {
            OrbitToolForm.ClearFrame();
            if (plan.State == CommitState.NodeCreated)
                DrawCreatedBurn();
            else
                MultiPassUI.DrawRunning(source, key);
            return;
        }

        OrbitToolForm.DrawInline(key, source, plan.Basis, plan.Maneuver);
        if (plan.Maneuver is not OrbitManeuvers.ManeuverResult maneuver)
            return;

        MultiPassUI.Draw(source, plan.Impulsive ?? maneuver, key, countRow: where != CraftSituation.Ground);
    }

    internal static Orbit? OrbitAfter(Burn burn) => PlanningBasis.ConicAfterBurn(burn.FlightPlan)?.Orbit;

    private static void DrawCreatedBurn()
    {
        if (_ourBurn == null)
            return;
        if (ConsoleUi.NextRow(PageText.CreatedNext, PageText.NextToolLink, "AfcOrbitNextTool", PageText.NextToolTip))
            PlanNext();
    }

    /// <summary>The markers of the orbit preview, drawn after the console window has ended so they do not nest inside it.</summary>
    internal static void DrawAfterWindow(IGameViewport viewport)
    {
        if (_showOrbitPreview && _lastSource != null)
        {
            if (MultiPassUI.HasMultiPassPreview)
                MultiPassUI.RenderMarkers(viewport, _lastSource);
            else if (_lastEntry != null)
                DrawOrbitMarkers(viewport);
        }
    }

    /// <summary>
    /// <summary>Drops the preview once neither the page nor the table of its plan is on screen.</summary>
    /// </summary>
    internal static void Tick()
    {
        if (PreviewVisible || PlanKept)
            return;
        OrbitToolForm.RecheckSeed();
        _lastEntry = null;
        _lastSource = null;
        ClearRefinement();
    }

    #region Helpers

    private static OrbitManeuvers.ManeuverResult? ComputeManeuver(
        string key, Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        // Search after the preceding burn when chaining maneuvers.
        UniverseTime now = basis.Earliest;

        if (key == ManeuverTools.KeySetPeriapsis)
            return OrbitManeuvers.ComputeSetPeriapsis(
                orbit, OrbitToolForm.TargetAltitude, parentRadius, now);

        if (key == ManeuverTools.KeySetApoapsis)
            return OrbitManeuvers.ComputeSetApoapsis(
                orbit, OrbitToolForm.TargetAltitude, parentRadius, now);

        if (key == ManeuverTools.KeyMatchInclination)
        {
            Orbit? targetOrbit = OrbitToolForm.GetSelectedTargetOrbit();
            if (targetOrbit == null) return null;
            return OrbitManeuvers.ComputeMatchInclination(
                orbit, targetOrbit, OrbitToolForm.UseDescendingNode, now);
        }

        if (key == ManeuverTools.KeySetInclination)
        {
            return OrbitManeuvers.ComputeSetInclination(
                orbit, OrbitToolForm.TargetInclinationRad,
                OrbitToolForm.UseDescendingNode, now,
                OrbitToolForm.InclinationRef);
        }

        if (key == ManeuverTools.KeyCircularizeApoapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: true, now);

        if (key == ManeuverTools.KeyCircularizePeriapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: false, now);

        return null;
    }

    // Stock refines Circularize with FiniteBurnRefiner.RefineToCircular and Match Target Plane with FiniteBurnRefiner.RefineToPlane. A plane goal can move the burn time. Set Periapsis and Set Apoapsis hold the apsis they set.
    internal static FiniteBurnGoal FiniteBurnGoalFor(string key)
    {
        if (key == ManeuverTools.KeyCircularizeApoapsis || key == ManeuverTools.KeyCircularizePeriapsis)
            return FiniteBurnGoal.Circular;
        if (key == ManeuverTools.KeyMatchInclination)
            return FiniteBurnGoal.Plane(OrbitToolForm.GetSelectedTargetOrbit());
        if (key == ManeuverTools.KeySetInclination)
            return FiniteBurnGoal.Plane(null);
        return FiniteBurnGoal.Apsis;
    }

    // The page redraws every frame, so the last refinement is reused until its inputs change, as TransferPlanner.SelectedPlan does for the transfer burn.
    private static OrbitManeuvers.ManeuverResult RefineForFiniteBurn(
        string key, Vehicle source, PlanningBasis basis, in OrbitManeuvers.ManeuverResult impulsive)
    {
        ReadOnlySpan<FiniteBurnPhase> phases = FiniteBurns.FullThrottlePhases(source);
        FiniteBurnGoal goal = FiniteBurnGoalFor(key);
        if (_refined is OrbitManeuvers.ManeuverResult cached
            && ReferenceEquals(_refinedOrbit, basis.Orbit)
            && _refinedImpulsive == impulsive
            && _refinedGoal == goal
            && phases.SequenceEqual(_refinedPhases.AsSpan(0, _refinedPhaseCount)))
            return cached;

        OrbitManeuvers.ManeuverResult refined =
            OrbitManeuvers.RefineForFiniteBurn(basis.Orbit, impulsive, phases, goal, basis.Earliest);
        _refinedOrbit = basis.Orbit;
        _refinedImpulsive = impulsive;
        _refinedGoal = goal;
        _refinedPhaseCount = Math.Min(phases.Length, _refinedPhases.Length);
        phases.Slice(0, _refinedPhaseCount).CopyTo(_refinedPhases);
        _refined = refined;
        return refined;
    }

    private static void ClearRefinement()
    {
        _refined = null;
        _refinedOrbit = null;
        _refinedGoal = default;
    }

    // Burn.Equals compares time and delta V, so plan membership alone does not prove that the burn belongs to the selected vehicle.
    private static void CleanupStaleBurn(Vehicle source)
    {
        if (_ourBurn == null) return;
        if (_ourBurn.Vehicle.Id != source.Id
            || !source.FlightComputer.BurnPlan.TryGetBurn(_ourBurn))
            _ourBurn = null;
    }

    private static void DropPlanState()
    {
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
        ClearRefinement();
    }

    internal static void Reset()
    {
        DropPlanState();
        _showOrbitPreview = false;
        ToolKey = ManeuverTools.KeyCircularizeApoapsis;
        Source = null;
        PreviewVisible = false;
        PlanKept = false;
        _toolPicked = false;
        _startPending = true;
        _startPhase = SeedPhase.Unknown;
        _madeLastFrame = false;
    }

    // Keep the preview toggles when execution is cancelled and disable them when it completes successfully.
    internal static void OnMultiPassCompleted()
    {
        _showOrbitPreview = false;
    }

    #endregion
}
