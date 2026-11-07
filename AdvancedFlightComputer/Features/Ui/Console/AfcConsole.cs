using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Transfer;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>The AFC console, one window for every AFC function. It docks where the stock Transfer Planner opens, and its height follows its content.</summary>
internal static partial class AfcConsole
{
    internal const string KeyPage = "console_page";
    internal const string KeyWide = "console_wide";
    // Read once to move a saved booster page to LAUNCH or LAND > BOOSTBACK, and dropped on the next save.
    private const string KeyLegacyBoosterStep = "console_recover_step";
    internal const string KeyLandWay = "console_land_way";
    internal const string KeyTuneSet = "console_tune_set";
    internal const string KeyTuneLanding = "console_tune_landing";
    internal const string KeyTuneDiagnostics = "console_tune_diagnostics";
    internal const string KeyOpenAtStart = "console_open_at_start";
    internal const string KeyExpertOpen = "console_expert_open";
    internal const string KeyFirstOpenSeen = "console_first_open_seen";

    /// <summary>The draw gate. Off while the mod loads its patches and from the start of an unload.</summary>
    internal static bool Enabled;

    internal static bool IsOpen;
    internal static bool Wide;
    internal static bool DrawerOpen;
    internal static ConsolePage Page = ConsolePage.Launch;
    internal static LandWay LandWay = LandWay.FromOrbit;
    internal static TuneSet TuneSet = TuneSet.Ascent;
    internal static TuneLanding TuneLanding = TuneLanding.Approach;
    internal static TuneDiagnostics TuneDiagnostics = TuneDiagnostics.Ascent;

    /// <summary>The facts of this frame. Rebuilt first in every draw.</summary>
    internal static readonly ConsoleFrame Frame = new();

    private static bool _pageUsedThisFlight;
    private static bool _flightStarted;
    private static bool _firstOpenNotice;
    private static bool _applySize;
    private static string? _subjectId;
    private static int _drawFaults;
    private static bool _pagesShown;

    // A console that throws in this many draws in a row gives the planner key back to stock for the session, because the only other ways back to the stock planner are inside the console.
    private const int MaxDrawFaults = 30;

    /// <summary>The notice that the planner key opens AFC. It shows only while the key is routed.</summary>
    private static bool ShowFirstOpenNotice => _firstOpenNotice && PlannerRoute.Active;

    /// <summary>The console is on screen: open, and not stepped aside for the stock planner.</summary>
    internal static bool IsShown => IsOpen && !StockHoldsDock;

    #region Opening

    /// <summary>
    /// Opens the console. A null page keeps the page used in this flight, or picks one from the situation
    /// when none was used yet. When the stock planner holds the dock, it closes, so the console is seen.
    /// </summary>
    internal static void Open(ConsolePage? page = null)
    {
        if (!Enabled)
            return;
        if (StockHoldsDock)
            TransferPlanner.ShowPlanWindow = false;
        bool wasOpen = IsOpen;
        IsOpen = true;
        if (page is ConsolePage p)
            SetPage(p);
        else if (!_pageUsedThisFlight)
            Page = FirstPage();
        if (!wasOpen)
        {
            _shrinkFrames = ShrinkFrames;
            if (!UiConfig.GetBool(KeyFirstOpenSeen, false))
                _firstOpenNotice = true;
        }
    }

    internal static void OpenRunning(ConsolePage? page)
    {
        Open(page);
        CraftActivity a = Frame.Activity;
        if (page == ConsolePage.Land && FlightView.FlyingWay(in a) is LandWay way)
            LandWay = way;
    }

    internal static void Close()
    {
        IsOpen = false;
        TakeOverAsk.Clear();
    }

    internal static void SetPage(ConsolePage page)
    {
        Page = page;
        _pageUsedThisFlight = true;
        SavePreferences();
    }

    /// <summary>Opens MANEUVER with a quick tool and the craft it plans for, for the orbit right-click entries.</summary>
    internal static void OpenOrbitTool(string toolKey, Vehicle vehicle)
    {
        Open(ConsolePage.Orbit);
        Pages.OrbitPage.SyncFromKey(toolKey);
        OrbitToolsPanel.Select(toolKey, vehicle, force: true);
    }

    /// <summary>Opens a page with its sub-page, from a cue target "PAGE" or "PAGE/SUBPAGE". The old booster cues open LAND > BOOSTBACK, or LAUNCH for the stage sites before launch.</summary>
    internal static void Navigate(string cueTarget)
    {
        if (cueTarget == GuidanceWindow.DrawerTarget)
        {
            Open();
            DrawerOpen = true;
            return;
        }
        if (!ConsolePages.TryParseCue(cueTarget, out ConsolePage page, out string sub))
            return;
        DrawerOpen = false;
        int slash = cueTarget.IndexOf('/');
        if (ConsolePages.ResolveLegacy(slash >= 0 ? cueTarget.Substring(0, slash) : cueTarget, sub, out ConsolePage legacy, out LandWay? legacyWay))
        {
            if (legacyWay is LandWay w)
                LandWay = w;
            Open(legacy);
            return;
        }
        if (page == ConsolePage.Land && sub.Length > 0)
            LandWay = ConsolePages.EnumFromKey(sub, LandWay);
        if (page == ConsolePage.Orbit && OrbitToolCatalog.KeyOfCue(sub) is string tool && Frame.Subject is Vehicle craft)
        {
            OpenOrbitTool(tool, craft);
            return;
        }
        Open(page);
    }

    /// <summary>What one routed planner key press does, see RouteDecision.Toggle.</summary>
    private static void ApplyToggle(bool editor)
    {
        bool strip = !IsOpen && Frame.StripWanted;
        switch (RouteDecision.Toggle(IsOpen, strip, editor))
        {
            case ConsoleToggle.Close:
                Close();
                break;
            case ConsoleToggle.OpenRunningPage:
                OpenRunning(ConsolePages.PageOf(Frame.Stop.Target, Frame.TransferPasses, Frame.Activity.RegisteredModeLive) ?? Page);
                break;
            case ConsoleToggle.OpenEditor:
                Open(ConsolePages.WorksInEditor(Page) ? Page : ConsolePage.Staging);
                break;
            default:
                Open();
                break;
        }
    }

    private static ConsolePage FirstPage()
    {
        Vehicle? v = Frame.Subject;
        Frame.ReadSituation();
        bool landed = Frame.Situation.Where == CraftSituation.Ground;
        bool inOrbit = false, engines = false, target = false;
        if (v != null)
        {
            try
            {
                inOrbit = !landed && v.Orbit.Periapsis > v.Parent.MeanRadius;
                engines = v.Parts?.Modules.Get<EngineController>().Length > 0;
                target = v.Target is Celestial c && !ReferenceEquals(c, v.Parent);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("afc-first-page:" + ex.GetType().Name, $"[AFC] First page check failed: {ex.Message}");
            }
        }
        var facts = new FlightFacts(Program.IsEditorOpen, Frame.Stop.Target, landed, engines, inOrbit, target, Frame.TransferPasses,
            Frame.Activity.RegisteredModeLive, Frame.Situation.Where);
        return FlightContext.FirstPage(in facts);
    }

    #endregion

    #region Preferences

    internal static void LoadPreferences()
    {
        UiConfig.SetDefault(KeyPage, ConsolePages.Key(ConsolePage.Launch));
        UiConfig.SetDefault(KeyWide, false);
        UiConfig.SetDefault(KeyOpenAtStart, false);
        UiConfig.SetDefault(KeyExpertOpen, false);
        UiConfig.SetDefault(UiConfig.KeyHoverNumpad, false);
        UiConfig.SetDefault(UiConfig.KeyShowDiagnostics, false);
        PlannerRoute.LoadPreference();

        Wide = UiConfig.GetBool(KeyWide, false);
        string savedPage = UiConfig.GetString(KeyPage, "");
        LandWay = ConsolePages.EnumFromKey(UiConfig.GetString(KeyLandWay, ""), LandWay.FromOrbit);
        // With no saved step, the legacy key opens LAUNCH, the page of the stage sites.
        if (ConsolePages.ResolveLegacy(savedPage, UiConfig.GetString(KeyLegacyBoosterStep, "BEFORELAUNCH"), out ConsolePage legacy, out LandWay? legacyWay))
        {
            Page = legacy;
            if (legacyWay is LandWay w)
                LandWay = w;
        }
        else
            Page = ConsolePages.PageFromKey(savedPage, ConsolePage.Launch);
        TuneSet = ConsolePages.EnumFromKey(UiConfig.GetString(KeyTuneSet, ""), TuneSet.Ascent);
        TuneLanding = ConsolePages.EnumFromKey(UiConfig.GetString(KeyTuneLanding, ""), TuneLanding.Approach);
        TuneDiagnostics = ConsolePages.EnumFromKey(UiConfig.GetString(KeyTuneDiagnostics, ""), TuneDiagnostics.Ascent);
    }

    internal static void SavePreferences()
    {
        UiConfig.Set(KeyPage, ConsolePages.Key(Page));
        UiConfig.Set(KeyWide, Wide);
        UiConfig.Remove(KeyLegacyBoosterStep);
        UiConfig.Set(KeyLandWay, ConsolePages.EnumKey(LandWay));
        UiConfig.Set(KeyTuneSet, ConsolePages.EnumKey(TuneSet));
        UiConfig.Set(KeyTuneLanding, ConsolePages.EnumKey(TuneLanding));
        UiConfig.Set(KeyTuneDiagnostics, ConsolePages.EnumKey(TuneDiagnostics));
    }

    internal static void SetWide(bool wide)
    {
        if (Wide == wide)
            return;
        Wide = wide;
        _applySize = true;
        SavePreferences();
    }

    /// <summary>Forgets the saved place, so the console docks at the stock planner place again.</summary>
    internal static void ResetWindowPlace()
    {
        Dock.ForgetPlacement(WindowKey);
        _applySize = true;
    }

    internal static void DismissFirstOpenNotice()
    {
        _firstOpenNotice = false;
        UiConfig.Set(KeyFirstOpenSeen, true);
    }

    #endregion

    #region Reset

    /// <summary>Everything the console holds about the world, for a save load.</summary>
    internal static void ResetWorldState()
    {
        _subjectId = null;
        _pageUsedThisFlight = false;
        _flightStarted = false;
        DrawerOpen = false;
        TakeOverAsk.Clear();
        ConsoleUi.ForgetFold(FlightView.PlanFoldId);
        Frame.Clear();
        ConsoleHead.Reset();
        ConsoleFooter.Reset();
        ConsoleMessages.Reset();
        HeadDrawer.Reset();
        GuidanceWindow.ResetWorldRequests();
        Pages.RcsPage.Reset();
    }

    internal static void Reset()
    {
        ResetWorldState();
        IsOpen = false;
        Wide = false;
        _firstOpenNotice = false;
        _applySize = false;
        ResetFollowing();
        ConsoleUi.ResetRows();
        _drawFaults = 0;
        Page = ConsolePage.Launch;
        LandWay = LandWay.FromOrbit;
        TuneSet = TuneSet.Ascent;
        TuneLanding = TuneLanding.Approach;
        TuneDiagnostics = TuneDiagnostics.Ascent;
        Pages.OrbitPage.Reset();
        Pages.TunePage.Reset();
        Pages.TransferPage.Reset();
        GuidanceWindow.ResetConsoleState();
        AfcAlerts.History = null;
    }

    /// <summary>The first unload step: no frame between unload steps may route the key or draw.</summary>
    internal static void Unload()
    {
        Enabled = false;
        PlannerRoute.Reset();
        TransferSession.CancelAll();
        UiConfig.Flush();
    }

    #endregion

    #region Draw entry

    /// <summary>Called once per frame from the loader's after-GUI hook, also while the console is closed.</summary>
    public static void Draw(IGameViewport viewport)
    {
        if (!Enabled)
            return;

        try
        {
            PlannerRoute.FrameCheck();
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-route-check:" + ex.GetType().Name, $"[AFC] Planner key frame check failed: {ex}");
        }

        BuildFrame();
        if (PlannerRoute.ConsumeToggle(out bool editorToggle))
            ApplyToggle(editorToggle);

        TickWork();

        if (Frame.Editor)
        {
            SetPageFlags(pagesShown: false);
            if (IsOpen && !StockHoldsDock)
                DrawWindowGuarded();
            return;
        }

        if (Program.ControlledVehicle == null)
        {
            SetPageFlags(pagesShown: false);
            return;
        }

        if (!_flightStarted)
        {
            _flightStarted = true;
            if (UiConfig.GetBool(KeyOpenAtStart, false))
                Open();
        }

        bool consoleShown = IsOpen && !StockHoldsDock;
        SetPageFlags(consoleShown);

        if (consoleShown)
            DrawWindowGuarded();
        else if (Frame.StripWanted)
            DrawStripGuarded(besideStock: StockHoldsDock);

        DrawFailureBandGuarded();
        DrawAfterWindows(viewport);
    }

    /// <summary>The stock planner is open on purpose (see PlannerRoute.StockLinkActive), so it owns the dock and the console steps aside.</summary>
    internal static bool StockHoldsDock => PlannerRoute.StockLinkActive && TransferPlanner.ShowPlanWindow;

    private static void BuildFrame()
    {
        Frame.Clear();
        Frame.Editor = Program.IsEditorOpen;
        if (Frame.Editor)
        {
            RefreshOtherCraft();
            return;
        }

        Vehicle? controlled = Program.ControlledVehicle;
        Frame.Eva = controlled is KittenEva;
        Vehicle? subject = controlled != null && controlled is not KittenEva && controlled.Orbit?.Parent != null ? controlled : null;
        Frame.Subject = subject;
        Frame.GuidanceOn = SharedVehicleHooks.GuidanceEnabled && GuidanceWindow.ModActive;

        if (subject?.Id != _subjectId)
        {
            _subjectId = subject?.Id;
            TakeOverAsk.Clear();
            ConsoleUi.ForgetFold(FlightView.PlanFoldId);
            GuidanceWindow.OnSubjectChanged();
            ConsoleHead.OnSubjectChanged(subject);
            if (subject != null && OrbitToolsPanel.Enabled)
                OrbitToolsPanel.OnSubjectChanged(subject);
            // TRANSFER plans for the new craft too, as stock's planner resets its SOURCE to the controlled craft when its window appears.
            if (subject != null)
                TransferSession.SetSource(subject);
        }

        if (subject == null)
        {
            RefreshOtherCraft();
            return;
        }

        if (Frame.GuidanceOn)
            GuidanceWindow.BeginConsoleFrame(subject);
        Frame.Mode = GuidanceWindow.ReadMode(subject);
        Frame.Stop = ConsoleFrame.StopFor(subject, Frame.Mode.Activity);
        if (Frame.Idle)
            ConsoleUi.ForgetFold(FlightView.PlanFoldId);
        Frame.ReadSituation();
        Frame.TransferPasses = Frame.Activity.MultiPassActive
            && MultiPassRegistry.TryGet(subject.Id, out MultiPassExecution? exec) && !OrbitToolCatalog.IsTool(exec.Intent.TypeKey);
        HeadDrawer.Refresh(subject, force: false);
        ConsoleHead.BuildLines(Frame);
        // LAND follows a hand-over of the landing with its way, as the solver follows it. The page itself never changes.
        if (GuidanceWindow.ConsumeFollowedLandWay() is LandWay followed && followed != LandWay)
        {
            LandWay = followed;
            SavePreferences();
        }
        // The head is on screen in the console and in the strip, so a toast is raised only when neither draws.
        bool headShown = (IsOpen && !StockHoldsDock) || Frame.StripWanted;
        ConsoleMessages.Observe(Frame.Lines, subject.Id, headShown);
    }

    // Without a subject (the editor, an EVA, no craft) every craft AFC still flies is listed under OTHER CRAFT with its stop, also the one the player left. When it is one craft, the head keeps its stop.
    private static void RefreshOtherCraft()
    {
        try
        {
            HeadDrawer.Refresh(null, force: false);
            if (HeadDrawer.TryGetSingleOtherStop(out Vehicle? other, out StopSpec stop))
            {
                Frame.OtherStop = stop with { Detail = "On " + other!.Id + ": " + stop.Detail };
                Frame.OtherStopCraft = other.Id;
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-other-craft:" + ex.GetType().Name, $"[AFC] Other craft scan failed: {ex}");
        }
    }

    // Work that runs every frame whether or not the console draws: the transfer calculation collects its workers, the hover nudges read the numpad, and the demand flags of the step expire.
    private static void TickWork()
    {
        try
        {
            TransferSession.Tick();
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-transfer-tick:" + ex.GetType().Name, $"[AFC] Transfer calculation tick failed: {ex}");
        }

        if (Frame.Subject != null && Frame.GuidanceOn)
        {
            try
            {
                GuidanceWindow.PollHoverNumpad();
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("afc-numpad:" + ex.GetType().Name, $"[AFC] Hover numpad failed: {ex.Message}");
            }
        }
    }

    // The flags the guidance step and the orbit preview read: which page is on screen this frame.
    private static void SetPageFlags(bool pagesShown)
    {
        bool shown = pagesShown && Frame.Subject != null;
        GuidanceWindow.GuidanceUiVisible = shown
            && Page is ConsolePage.Launch or ConsolePage.Land or ConsolePage.Tune or ConsolePage.Staging or ConsolePage.Orbit or ConsolePage.Transfer or ConsolePage.Rcs;
        GuidanceWindow.PageShownProbe = Frame.Subject != null ? PageProbe : null;
        OrbitToolsPanel.PreviewVisible = shown && Page == ConsolePage.Orbit && OrbitToolsPanel.Enabled;
        OrbitToolsPanel.PlanKept = shown && Page == ConsolePage.Tune && OrbitToolsPanel.Enabled && Pages.TunePage.FlightPlanShown(Frame);
        OrbitToolsPanel.Tick();
        TransferSession.PageVisible = shown && Page == ConsolePage.Transfer;
        _pagesShown = shown;
    }

    private static readonly Func<GuidanceWindow.GuidancePage, bool> PageProbe = IsGuidancePageShown;

    private static bool IsGuidancePageShown(GuidanceWindow.GuidancePage page)
    {
        if (!_pagesShown)
            return false;
        return page switch
        {
            GuidanceWindow.GuidancePage.Ascent => Page == ConsolePage.Launch,
            GuidanceWindow.GuidancePage.Boostback => (Page == ConsolePage.Land && LandWay == LandWay.Boostback)
                || (Page == ConsolePage.Tune && TuneSet == TuneSet.Diag
                    && TuneDiagnostics is TuneDiagnostics.Boostback or TuneDiagnostics.Aero),
            GuidanceWindow.GuidancePage.Deorbit => Page == ConsolePage.Land && LandWay == LandWay.FromOrbit,
            GuidanceWindow.GuidancePage.Land => Page == ConsolePage.Land && LandWay == LandWay.FromHere,
            GuidanceWindow.GuidancePage.Hover => Page == ConsolePage.Land && LandWay == LandWay.Hover,
            _ => false,
        };
    }

    private static void DrawWindowGuarded()
    {
        try
        {
            DrawWindow();
            _drawFaults = 0;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-console:" + ex.GetType().Name, $"[AFC] AFC console draw failed: {ex}");
            if (++_drawFaults >= MaxDrawFaults && PlannerRoute.Active)
            {
                PlannerRoute.Active = false;
                IsOpen = false;
                DefaultCategory.Log.Warning($"[AFC] The AFC console failed to draw {MaxDrawFaults} times in a row, so the planner key opens the stock Transfer Planner again for this session.");
            }
        }
    }

    private static void DrawStripGuarded(bool besideStock)
    {
        try
        {
            DrawStrip(besideStock);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-strip:" + ex.GetType().Name, $"[AFC] AFC strip draw failed: {ex}");
        }
    }

    private static void DrawFailureBandGuarded()
    {
        try
        {
            FailureBandWindow.Draw(Frame);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-failure-band:" + ex.GetType().Name, $"[AFC] AFC failure band draw failed: {ex}");
        }
    }

    internal static void AcknowledgeFailure() => ConsoleHead.AcknowledgeFailure(Frame);

    // World markers and overlays are their own ImGui windows, so they draw after the console window has ended.
    private static void DrawAfterWindows(IGameViewport viewport)
    {
        if (OrbitToolsPanel.PreviewVisible)
        {
            try
            {
                OrbitToolsPanel.DrawAfterWindow(viewport);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("afc-orbit-after:" + ex.GetType().Name, $"[AFC] ORBIT map markers failed: {ex}");
            }
        }

        try
        {
            Pages.TransferPage.DrawAfterWindow(viewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("afc-transfer-after:" + ex.GetType().Name, $"[AFC] TRANSFER map markers failed: {ex}");
        }

        if (Frame.GuidanceOn && Frame.Subject != null)
        {
            try
            {
                GuidanceWindow.DrawTrailing(viewport, Frame.Subject);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("afc-guidance-trailing:" + ex.GetType().Name,
                    $"[AFC] Guidance overlay draw failed for '{Frame.Subject.Id}': {ex}");
            }
        }
    }

    #endregion
}
