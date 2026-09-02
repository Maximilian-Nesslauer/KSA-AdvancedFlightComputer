using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Prefix on TransferPlanner.DrawPlanWindow that takes over the entire window
/// when one of our plan types is selected. Draws the Plan Type dropdown, Source
/// dropdown and type-specific controls in the window body, with Create in the
/// footer where stock's own Create button sits.
///
/// Returns false (skip original) for our types, true for stock types.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow
{
    // Window placement and identity, kept to match stock
    // TransferPlanner.DrawPlanWindow and DrawSelectedTransferFlightPlan. The
    // id, title and signature are stock's own: this prefix replaces that window
    // for our plan types, so sharing them keeps the position and size the player
    // set when they switch between a stock and an AFC plan type.
    private const string WindowId = "transfer-planning";
    private const string WindowTitle = "TRANSFER PLANNING";
    private const string WindowSignature = "KSA-TRJ";
    private const string FlightPlanWindowId = "afc-maneuver-flightplan";
    private const string FlightPlanWindowTitle = "MANEUVER FLIGHT PLAN";

    // Base values are stock's, scaled the way stock scales its own, so the
    // window keeps the size and position of a stock plan type at any interface
    // scale setting.
    private static float MainWindowOffsetXPx => 440f * ImGuiHelper.InterfaceScale;
    private static float MainWindowOffsetYPx => 50f * ImGuiHelper.InterfaceScale;
    private static float MainWindowWidthPx => 400f * ImGuiHelper.InterfaceScale;
    private static float MainWindowHeightPx => 1050f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowOffsetXPx => 620f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowOffsetYPx => 40f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowWidthPx => 460f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowHeightPx => 620f * ImGuiHelper.InterfaceScale;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static readonly List<TransferObject> _sourceListBuffer = new();

    private static Burn? _ourBurn;
    private static OrbitalTransfers.PorkChopEntry? _lastEntry;
    private static Vehicle? _lastSource;
    private static bool _showFlightPlanPreview;
    private static bool _showOrbitPreview;

    static bool Prefix(IGameViewport inViewport)
    {
        TransferType transferType;
        try
        {
            if (StockPlanner.TransferType is not TransferType current)
                return true;
            transferType = current;
        }
        catch (Exception ex)
        {
            // Deduped: this runs per ImGui frame, and a persistent throw would
            // otherwise write a stack trace at frame rate into a log the next
            // game start overwrites.
            LogHelper.WarnOnce("maneuvertools-type-lookup:" + ex.GetType().Name,
                $"[AFC] ManeuverTools Prefix (type lookup): {ex}");
            return true;
        }

        if (!ManeuverTools.IsHandledType(transferType.GetKey()))
        {
            DropPlanState();
            return true;
        }

#if DEBUG
        using var _perf = new PerfTracker.Scope("Patch_DrawPlanWindow.Prefix");
#endif

        try
        {
            DrawWindow(inViewport, transferType);
        }
        catch (Exception ex)
        {
            // Keyed by plan type as well as exception type: one plan type
            // throwing must not silence a different one for the rest of the
            // session, and the type is the first thing to reproduce with.
            string typeKey = transferType.GetKey();
            LogHelper.WarnOnce($"maneuvertools-draw:{typeKey}:{ex.GetType().Name}",
                $"[AFC] ManeuverTools Prefix (plan type '{typeKey}', "
                + $"source '{_lastSource?.Id ?? "none"}'): {ex}");
            // ImGui state may be partially set; do not let stock also Begin the
            // same window this frame, that produces ID-stack conflicts.
        }
        return false;
    }

    private static void DrawWindow(IViewport inViewport, TransferType transferType)
    {
        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(inViewport.Size.X - MainWindowOffsetXPx, MainWindowOffsetYPx),
            ImGuiCond.Appearing, (float2?)null);

        bool open = StockPlanner.ShowPlanWindow;
        // BeginWindow ends the underlying ImGui window itself when it returns
        // false, so only the true branch owes an EndWindow.
        if (!ConsoleStyle.BeginWindow(WindowId, WindowTitle, WindowSignature, ref open,
                new float2(MainWindowWidthPx, MainWindowHeightPx), ImGuiWindowFlags.NoScrollbar))
            return;

        Commit commit = default;
        try
        {
            if (!open)
            {
                HandleWindowClose();
                return;
            }

            StockPlanner.ShowPlanWindow = true;

            ConsoleStyle.BeginBody();
            ConsoleStyle.PushWidgetStyle();
            try
            {
                commit = DrawBody(transferType);
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
                ConsoleStyle.EndBody();
            }

            ConsoleStyle.BeginFooter();
            bool create = DrawFooter(commit.State);
            ConsoleStyle.EndFooter();

            // Outside the footer, matching stock: the commit runs ImGui-free
            // work and queues a burn, which has no business happening between
            // BeginFooter and EndFooter. DrawFooter only returns true for
            // CommitState.Ready, which DrawBody produces only with a resolved
            // source, so the source is non-null here.
            if (create)
                CreateSingleOrMultiPass(commit);
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }

        if (_showOrbitPreview && _lastSource != null)
        {
            if (MultiPassUI.HasMultiPassPreview)
                MultiPassUI.RenderMarkers(inViewport, _lastSource);
            else if (_lastEntry != null)
                DrawOrbitMarkers(inViewport);
        }

        if (_showFlightPlanPreview)
            DrawFlightPlanWindow(inViewport);
    }

    /// <summary>Window body. Returns what the footer needs to offer the commit
    /// action, so the primary button can sit in the footer where stock's own
    /// Create button sits.</summary>
    private static Commit DrawBody(TransferType transferType)
    {
        if (!DrawPlanTypeDropdown(ref transferType))
            return default;

        Vehicle? source = DrawSourceDropdown();
        if (source?.Orbit == null)
            return default;
        _lastSource = source;
        CleanupStaleBurn(source);

        ImGui.Separator();

        // Resolved once per frame and threaded through: the window, the maneuver
        // math and the preview all have to agree on which trajectory is being
        // planned against, or the readout describes a different orbit than the
        // Create button would burn on.
        PlanningBasis basis = PlanningBasis.For(source);
        string typeKey = transferType.GetKey();

        ManeuverToolsWindow.DrawInline(typeKey, source, basis);

        var result = ComputeManeuver(typeKey, source, basis);
        if (result == null)
        {
            _lastEntry = null;
            return default;
        }

        _lastEntry = BuildTransferEntry(source, result.Value, basis);

        ImGui.Separator();
        DrawManeuverInfo(result.Value);

        MultiPassUI.Draw(source, result.Value, typeKey);

        CommitState state = ResolveCommitState(source, typeKey);
        if (state == CommitState.MultiPassRunning)
        {
            ImGui.Spacing();
            MultiPassController.DrawStatus(source);
        }

        // Hidden only when a single-burn maneuver node has been
        // created (stock then owns the rendering); during active
        // multi-pass execution the checkboxes stay visible so
        // the user can toggle the future-passes overlay.
        if (_ourBurn == null)
        {
            ConsoleWidgets.Rule();
            ConsoleUi.CheckboxRow("PREVIEW ORBIT".AsSpan(), "AfcMtPreviewOrbit".AsSpan(),
                ref _showOrbitPreview);
            ConsoleUi.CheckboxRow("PREVIEW FLIGHT PLAN".AsSpan(), "AfcMtPreviewPlan".AsSpan(),
                ref _showFlightPlanPreview);
        }

        return new Commit(state, source, result.Value, typeKey, basis);
    }

    #region Dropdowns

    private static bool DrawPlanTypeDropdown(ref TransferType transferType)
    {
        TransferType prev = transferType;
        if (ConsoleUi.ComboRow("PLAN TYPE".AsSpan(), "PlanType".AsSpan(), ref transferType,
                TransferPlanner.TransferTypes)
            && transferType.GetKey() != prev.GetKey())
        {
            StockPlanner.TransferType = transferType;
            StockPlanner.TransferCalculated = false;
            ManeuverToolsWindow.OnTypeChanged();
            OnManeuverContextChanged();

            if (!ManeuverTools.IsHandledType(transferType.GetKey()))
            {
                GameReflection.TransferPlanner_SetTransferInfo!.Invoke(null, null);
                return false;
            }
        }
        return true;
    }

    private static Vehicle? DrawSourceDropdown()
    {
        TransferObject sourceBody = StockPlanner.SourceBody;

        // Rebuild every frame, as stock's own PopulateWithVehicles does: a
        // TransferObject holds only a LookupIndex, and LookupCollection.Deregister
        // swap-removes, so an entry kept across frames can resolve to a different
        // vehicle once one is destroyed. Program.VehiclesInFrame is every vehicle in
        // the current system (Program.RefreshVehiclesInFrame applies no filter
        // beyond the type), which is the same set stock offers as a source.
        ReadOnlySpan<Vehicle> vehiclesInFrame = Program.VehiclesInFrame;
        Span<TransferObject> list = stackalloc TransferObject[vehiclesInFrame.Length];
        for (int i = 0; i < vehiclesInFrame.Length; i++)
            list[i] = new TransferObject(vehiclesInFrame[i]);

        if (ImGui.IsWindowAppearing() || sourceBody.GetKey() == "N/A")
        {
            // new TransferObject(-1), not default: a TransferObject holds one int, so
            // default is LookupIndex 0 and resolves to the first registered
            // astronomical (normally the star), whose key is its Id and never "N/A".
            // With default here the "N/A" fallback below would be dead, an unrelated
            // body would end up in stock's _sourceBody, and stock's own re-pick could
            // not repair it either, since that is keyed on "N/A" too. Only a negative
            // index is the none sentinel.
            sourceBody = new TransferObject(-1);
            if (Program.ControlledVehicle != null)
            {
                for (int i = 0; i < list.Length; i++)
                {
                    if (list[i].GetKey() == Program.ControlledVehicle.Id)
                    {
                        sourceBody = list[i];
                        break;
                    }
                }
            }
            if (sourceBody.GetKey() == "N/A" && list.Length > 0)
                sourceBody = list[0];

            StockPlanner.SourceBody = sourceBody;
        }

        TransferObject prev = sourceBody;
        // A stack Span cannot cross into the helper's IReadOnlyList parameter, so
        // the per-frame list is copied into a reusable buffer for the combo.
        _sourceListBuffer.Clear();
        for (int i = 0; i < list.Length; i++)
            _sourceListBuffer.Add(list[i]);
        if (ConsoleUi.ComboRow("SOURCE".AsSpan(), "Source".AsSpan(), ref sourceBody, _sourceListBuffer)
            && sourceBody.GetKey() != prev.GetKey())
        {
            StockPlanner.SourceBody = sourceBody;
            ManeuverToolsWindow.OnSourceChanged();
        }

        return sourceBody.Body as Vehicle;
    }

    #endregion

    #region Maneuver Info + Create

    private static void DrawManeuverInfo(OrbitManeuvers.ManeuverResult maneuver)
    {
        double dvMag = maneuver.DvCci.Length();
        double timeToNode = (maneuver.BurnTime - Universe.GetElapsedTime()).Seconds();

        ConsoleWidgets.Readout("REQUIRED DELTA V".AsSpan(),
            string.Format(Inv, "{0:F1} m/s", dvMag).AsSpan());

        if (timeToNode > 0)
        {
            ConsoleWidgets.Readout("TIME TO BURN".AsSpan(),
                FormatHelper.FormatDuration(timeToNode).AsSpan());
        }
    }

    private static void DrawFlightPlanWindow(IViewport inViewport)
    {
        // Multi-pass: show the final-pass trajectory.
        FlightPlan? fp = MultiPassUI.HasMultiPassPreview
            ? MultiPassUI.LastPassFlightPlan
            : _lastEntry?.FlightPlan;
        if (fp == null) return;

        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(FlightPlanWindowOffsetXPx, FlightPlanWindowOffsetYPx),
            ImGuiCond.Appearing, (float2?)null);

        // Mirrors stock's own DrawSelectedTransferFlightPlan shell, footer included.
        if (!ConsoleStyle.BeginWindow(FlightPlanWindowId, FlightPlanWindowTitle, WindowSignature,
                ref _showFlightPlanPreview,
                new float2(FlightPlanWindowWidthPx, FlightPlanWindowHeightPx),
                ImGuiWindowFlags.NoFocusOnAppearing))
            return;

        try
        {
            ConsoleStyle.BeginBody();
            ConsoleStyle.PushWidgetStyle();
            try
            {
                fp.DrawPatchInfo();
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
                ConsoleStyle.EndBody();
            }
            ConsoleStyle.BeginFooter();
            ConsoleStyle.EndFooter();
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    private enum CommitState
    {
        /// <summary>No maneuver to commit (no source, no solution).</summary>
        None,
        MultiPassRunning,
        NodeCreated,
        /// <summary>Multi-pass selected but its preview failed. Offering Create
        /// would silently fall back to a full-dV single burn.</summary>
        Blocked,
        Ready,
    }

    private readonly record struct Commit(
        CommitState State, Vehicle? Source, OrbitManeuvers.ManeuverResult Maneuver,
        string TypeKey, PlanningBasis Basis);

    /// <summary>Decides what the footer may offer. Also retires an expired
    /// <see cref="_ourBurn"/>, which is what lets the Create button come back
    /// once the node it created is in the past.</summary>
    private static CommitState ResolveCommitState(Vehicle source, string typeKey)
    {
        if (MultiPassRegistry.Has(source.Id))
            return CommitState.MultiPassRunning;

        if (_ourBurn != null)
        {
            if (_ourBurn.Time < Universe.GetElapsedTime())
                _ourBurn = null;
            else
                return CommitState.NodeCreated;
        }

        return MultiPassUI.WantsMultiPassButCannot(typeKey)
            ? CommitState.Blocked
            : CommitState.Ready;
    }

    /// <summary>Returns true when the player clicked Create.</summary>
    private static bool DrawFooter(CommitState state)
    {
        switch (state)
        {
            case CommitState.MultiPassRunning:
                ConsoleStyle.FooterStatus("MULTI-PASS RUNNING".AsSpan(), pending: true);
                return false;
            case CommitState.NodeCreated:
                ConsoleStyle.FooterStatus("NODE CREATED".AsSpan(), pending: false);
                return false;
            case CommitState.Blocked:
                ConsoleStyle.FooterWarning("MULTI-PASS PREVIEW FAILED".AsSpan());
                return false;
            case CommitState.Ready:
                ConsoleStyle.FooterStatus("MANEUVER READY".AsSpan(), pending: true);
                ConsoleStyle.FooterRightAlign(ConsoleWidgets.ButtonWidth("CREATE".AsSpan()));
                return ConsoleWidgets.PrimaryButton("CREATE".AsSpan());
            default:
                ConsoleStyle.FooterStatus("NO MANEUVER".AsSpan(), pending: false);
                return false;
        }
    }

    private static void CreateSingleOrMultiPass(in Commit commit)
    {
        if (MultiPassUI.IsArmed(commit.TypeKey))
        {
            // The intents recompute every pass from the vehicle's live orbit,
            // so a multi-pass started on a chained basis would execute against
            // a different trajectory than the window just displayed.
            if (commit.Basis.IsChained)
                TimedAlert.Create(
                    "Multi-pass cannot start on a pending burn's trajectory; " +
                    "use a single pass or clear the plan first.", Color.Yellow, 4.0);
            else
                MultiPassController.Start(commit.Source!, commit.TypeKey);
        }
        else
            CreateSingleBurn(commit.Source!, commit.Maneuver, commit.Basis);
    }

    /// <summary>Drops the "we already created this node" marker because the user is
    /// now configuring a different maneuver. The marker only exists to stop a second
    /// click duplicating the same node across the frame gap between queueing a burn
    /// and it appearing in the plan, so holding it past a plan-type change would
    /// suppress the Create button, both preview checkboxes and the orbit preview for
    /// a maneuver that has not been created at all - which is what chaining a second
    /// tool onto a pending burn does.</summary>
    internal static void OnManeuverContextChanged()
    {
        _ourBurn = null;
    }

    private static void CreateSingleBurn(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, PlanningBasis basis)
    {
        // Routed through MultiPassCommitter so single-burn and the first
        // multi-pass pass take the same Burn.Create -> buffer-Add path.
        Burn? burn = MultiPassCommitter.QueueAddBurn(
            source, maneuver.BurnTime, maneuver.DvVlf, basis.Plan);
        if (burn == null) return;

        _ourBurn = burn;
    }

    /// <summary>
    /// Builds a PorkChopEntry for <paramref name="maneuver"/> so the
    /// flight-plan preview / orbit-marker rendering pipelines (which
    /// expect a PorkChopEntry like the stock Hohmann path produces)
    /// can run unchanged.
    /// </summary>
    private static OrbitalTransfers.PorkChopEntry BuildTransferEntry(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, PlanningBasis basis)
    {
        var transferData = new OrbitalTransfers.TransferData
        {
            Start = maneuver.BurnTime,
            Point = basis.Orbit.GetPointAt(maneuver.BurnTime),
            DeltaVelocityCci = maneuver.DvCci,
            TransferDvVlf = maneuver.DvVlf
        };

        // Chained: propagate off the patch the preceding burn produces. The stock
        // BuildFlightPlan path below starts from the vehicle's live state, which for
        // a chained maneuver is the orbit before that burn.
        if (basis.IsChained && basis.Patch != null)
        {
            var (chainedPlan, _) = MultiPassForwardChainPlanner.BuildPassFlightPlan(
                source, basis.Patch, maneuver.BurnTime, maneuver.DvVlf);
            return new OrbitalTransfers.PorkChopEntry(transferData, chainedPlan);
        }

        FlightPlan flightPlan = FlightPlan.CreateUninitialized(source.Hash);
        // Committed burns get this margin stamped by Burn.Create, so the preview's
        // impact test agrees with what the created burn's plan will compute.
        flightPlan.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
        var info = new OrbitalTransfers.TransferInfo(source, source, source, usePorkChopData: false);
        // BuildFlightPlan forwards info.Target as encounterFilter, which restricts
        // SOI-encounter detection to that one body. Apse / inclination maneuvers
        // have no target, so null it to detect all high-SOI siblings.
        info.Target = null!;
        OrbitalTransfers.BuildFlightPlan(
            ref flightPlan, info, transferData.Start, transferData.TransferDvVlf,
            out _, out _);
        // BuildFlightPlan leaves the terrain-impact search incremental and nothing
        // ever advances a detached preview plan's frontier, so finish the search here
        // or the preview can omit an impact the created burn's plan will find. 5/8
        // match the limits BuildFlightPlan itself computed the plan with.
        if (flightPlan.ImpactSearchUnresolved)
            flightPlan.ComputeCompleteTrajectory(out _, 5, 8, null,
                resolveImpactsCompletely: true);

        return new OrbitalTransfers.PorkChopEntry(transferData, flightPlan);
    }

    #endregion

    #region Visual Orbit Preview

    /// <summary>Background-drawlist markers (encounter, escape, impact,
    /// closest approach, Ap/Pe) for the preview orbit.</summary>
    private static void DrawOrbitMarkers(IViewport inViewport)
    {
        var uiContext = new Astronomical.UiContext(
            inViewport, _lastSource!, Color.Green,
            TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0),
            ManeuverToolsWindow.GetSelectedTargetOrbiter());
        _lastEntry!.FlightPlan.DrawUi(inViewport, uiContext, tintDanger: true);
    }

    /// <summary>3D-view post-burn orbit (single-burn or multi-pass), drawn
    /// from Patch_OnPreRender when "Preview Orbit" is on.</summary>
    internal static void RenderOrbitPreview(IViewport inViewport)
    {
        // Program.OnDrawUiThreadSafe calls DrawPlanWindow only while
        // TransferPlanner.ShowPlanWindow is true, so neither the prefix nor
        // HandleWindowClose runs once the window is closed from the
        // ToggleTransferPlan keybind or the View menu - both of which flip the
        // property and notify nobody. CelestialSystem.OnPreRender keeps calling
        // TransferPlanner.OnPreRender either way, which is why stock re-tests
        // ShowPlanWindow on both of its own overlay branches. Drop the plan state
        // rather than only skipping the draw, so a closed window cannot leave a
        // burn reference or a previous world's plan behind for the reopen.
        if (!StockPlanner.ShowPlanWindow)
        {
            DropPlanState();
            return;
        }

        if (_ourBurn != null || _lastSource == null) return;
        if (!_showOrbitPreview) return;

        if (MultiPassUI.HasMultiPassPreview)
        {
            MultiPassUI.Render(inViewport, _lastSource);
            return;
        }

        if (_lastEntry == null) return;

        FlightPlan fp = _lastEntry.FlightPlan;
        if (fp.Patches.Count == 0)
            return;

        if (fp.Patches[0].Orbit.IsMissingPoints())
        {
            foreach (PatchedConic patch in fp.Patches)
            {
                patch.HidePatch = false;
                MemoryOwner<OrbitPointCce> points = UpdateTaskUtils.GenerateSpacedPoints(patch);
                patch.Orbit.UpdateCachedPoints(points);
            }
        }

        fp.AddLineInstances(inViewport, _lastSource, isActive: true,
            drawVehiclePosition: false, TrueAnomaly.NaN, TrueAnomaly.NaN,
            isPostBurnOrbit: true);
    }

    #endregion

    #region Helpers

    private static OrbitManeuvers.ManeuverResult? ComputeManeuver(
        string key, Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        // Not "now": on a chained maneuver every apsis and node has to be sought
        // after the burn this one follows, or the result lands before it.
        UniverseTime now = basis.Earliest;

        if (key == ManeuverTools.KeySetPeriapsis)
            return OrbitManeuvers.ComputeSetPeriapsis(
                orbit, ManeuverToolsWindow.TargetAltitude, parentRadius, now);

        if (key == ManeuverTools.KeySetApoapsis)
            return OrbitManeuvers.ComputeSetApoapsis(
                orbit, ManeuverToolsWindow.TargetAltitude, parentRadius, now);

        if (key == ManeuverTools.KeyMatchInclination)
        {
            Orbit? targetOrbit = ManeuverToolsWindow.GetSelectedTargetOrbit();
            if (targetOrbit == null) return null;
            return OrbitManeuvers.ComputeMatchInclination(
                orbit, targetOrbit, ManeuverToolsWindow.UseDescendingNode, now);
        }

        if (key == ManeuverTools.KeySetInclination)
        {
            return OrbitManeuvers.ComputeSetInclination(
                orbit, ManeuverToolsWindow.TargetInclinationRad,
                ManeuverToolsWindow.UseDescendingNode, now,
                ManeuverToolsWindow.InclinationRef);
        }

        if (key == ManeuverTools.KeyStockCircularizeApoapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: true, now);

        if (key == ManeuverTools.KeyStockCircularizePeriapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: false, now);

        return null;
    }

    /// <summary>Drops <see cref="_ourBurn"/> unless it is still a live burn on the
    /// vehicle currently selected as the source.
    ///
    /// Anchoring on the current source is what stock does: the first thing
    /// <c>TransferPlanner.DrawPlanWindow</c> does is clear its own
    /// <c>_transferBurn</c> when <c>Source.FlightComputer.BurnPlan.TryGetBurn</c>
    /// says it is gone, where Source is the selected source, not the burn's own
    /// vehicle. Asking the burn's own vehicle instead keeps answering "still
    /// there" after the source dropdown moves to a different vehicle, and after a
    /// save load, because <c>Vehicle.Dispose</c> leaves the destroyed vehicle's
    /// FlightComputer and BurnPlan fully intact.
    ///
    /// The id comparison is not redundant with the plan lookup:
    /// <c>Burn.Equals</c> compares Time and DeltaVVlf only, so
    /// <c>BurnPlan.TryGetBurn</c> also matches an unrelated burn that happens to
    /// coincide in both.</summary>
    private static void CleanupStaleBurn(Vehicle source)
    {
        if (_ourBurn == null) return;
        if (_ourBurn.Vehicle.Id != source.Id
            || !source.FlightComputer.BurnPlan.TryGetBurn(_ourBurn))
            _ourBurn = null;
    }

    /// <summary>The cross-frame state a drawn window rebuilds every frame: the
    /// burn we created, the porkchop entry the preview renders, and the vehicle
    /// both are about. Every path that stops drawing the window clears all three
    /// together, so none of them can outlive the world they describe.</summary>
    private static void DropPlanState()
    {
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
    }

    private static void HandleWindowClose()
    {
        // Use the public setter so stock state (_transferBurn, _correctionBurn,
        // _selectedEntry, _lambertPatch, _transferCalculated) is cleared too;
        // setting only _showPlanWindow via reflection would leak that state.
        TransferPlanner.ShowPlanWindow = false;
        DropPlanState();
    }

    internal static void Reset()
    {
        DropPlanState();
        _showFlightPlanPreview = false;
        _showOrbitPreview = false;
    }

    /// <summary>Called from PassCompletionPatch when a multi-pass execution
    /// finishes all passes cleanly (not on user cancel). Auto-disables the
    /// preview toggles since the goal orbit has been reached and the
    /// overlay is no longer informative.</summary>
    internal static void OnMultiPassCompleted()
    {
        _showOrbitPreview = false;
        _showFlightPlanPreview = false;
    }

    #endregion
}
