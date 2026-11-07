using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>DIAG holds the live diagnostics and the stock table of the flight plan.</summary>
internal static class TunePage
{
    private static bool _pickedHere;

    public static void Reset() => _pickedHere = false;

    /// <summary>The plan of the tool stays while that view is on screen (AfcConsole.SetPageFlags), so the table still reads it.</summary>
    public static void ShowFlightPlan()
    {
        _pickedHere = true;
        AfcConsole.Open(ConsolePage.Tune);
        AfcConsole.TuneSet = TuneSet.Diag;
        AfcConsole.TuneDiagnostics = TuneDiagnostics.FlightPlan;
        GuidanceWindow.ShowBuiltInDiagnostics();
        AfcConsole.SavePreferences();
    }

    private static bool SetsWork(ConsoleFrame f) => f.Subject != null && f.GuidanceOn && !f.Editor;

    public static bool FlightPlanShown(ConsoleFrame f)
        => TuneRules.Shown(AfcConsole.TuneSet, SetsWork(f), _pickedHere) == TuneSet.Diag && AfcConsole.TuneDiagnostics == TuneDiagnostics.FlightPlan && !GuidanceWindow.ModeDiagnosticsOnScreen;

    public static CommitSpec Draw(in PageContext ctx)
    {
        Vehicle? subject = ctx.Frame.Subject;
        bool setsWork = SetsWork(ctx.Frame);
        if (setsWork)
            _pickedHere = false;
        TuneSet shown = TuneRules.Shown(AfcConsole.TuneSet, setsWork, _pickedHere);
        DrawSetRow(shown, setsWork);
        shown = TuneRules.Shown(AfcConsole.TuneSet, setsWork, _pickedHere);

        if (shown == TuneSet.Afc)
        {
            DrawPreferences();
            return CommitSpec.None;
        }
        if (!setsWork)
        {
            // The stock table needs no guidance, so the link from MANEUVER works with guidance off too.
            if (subject != null && shown == TuneSet.Diag && AfcConsole.TuneDiagnostics == TuneDiagnostics.FlightPlan)
                DrawFlightPlanView(subject);
            else
                ConsoleUi.MutedWrapped(SharedVehicleHooks.GuidanceEnabled ? PageText.TuneSetsNeed(ctx.Frame.Editor, subject != null) : GuidanceFeature.UnavailableReason);
            return CommitSpec.None;
        }

        TuneLanding landing = AfcConsole.TuneLanding;
        TuneDiagnostics diagnostics = AfcConsole.TuneDiagnostics;
        GuidanceWindow.DrawTuneGuidanceSet(subject!, ctx.Width, shown, ref landing, ref diagnostics);
        if (landing != AfcConsole.TuneLanding || diagnostics != AfcConsole.TuneDiagnostics)
        {
            AfcConsole.TuneLanding = landing;
            AfcConsole.TuneDiagnostics = diagnostics;
            AfcConsole.SavePreferences();
        }

        if (shown == TuneSet.Diag)
            return CommitSpec.None;

        int changed = GuidanceWindow.CountTuneChanged(shown, landing);
        if (!TuneRules.ResetSetShows(changed, GuidanceWindow.TuneRowResets))
            return CommitSpec.None;
        string resetLock = GuidanceWindow.TuneSetResetLock(shown, landing);
        Vehicle vehicle = subject!;
        return new CommitSpec("RESET SET", FlownBy.Nobody, resetLock, "", false,
            resetLock.Length > 0 ? resetLock : "Resets the changed values of this set to their defaults. Asks first.",
            Act: () => GuidanceWindow.ConfirmResetTuneSet(vehicle, shown, landing), Owner: Owner.Settings, Outcome: PageText.ResetSetOutcome(changed), Quiet: true);
    }

    private static void DrawSetRow(TuneSet shown, bool setsWork)
    {
        TuneSet[] order = TuneRules.Order;
        Span<string> labels = new string[order.Length];
        Span<string> cards = new string[order.Length];
        Span<int> counts = new int[order.Length];
        Span<bool> dim = stackalloc bool[order.Length];
        ConsoleFrame f = AfcConsole.Frame;
        string need = SharedVehicleHooks.GuidanceEnabled ? PageText.TuneSetsNeed(f.Editor, f.Subject != null) : GuidanceFeature.UnavailableReason;
        int active = 0;
        for (int i = 0; i < order.Length; i++)
        {
            TuneSet set = order[i];
            labels[i] = ConsolePages.TuneSetLabel(set);
            counts[i] = !TuneRules.HoldsValues(set) ? -1 : setsWork ? GuidanceWindow.CountTuneSetChanged(set) : 0;
            dim[i] = !setsWork && set != TuneSet.Afc;
            cards[i] = dim[i] ? need + "\n\n" + PageText.TuneSetCard(set) : PageText.TuneSetCard(set);
            if (set == shown)
                active = i;
        }
        int picked = ConsoleUi.PlaceTabs("", "AfcTuneSet", labels, active, cards, -1, TuneRules.DividerAt, dim, counts);
        if (picked < 0)
            return;
        if (!setsWork)
            _pickedHere = true;
        if (order[picked] != AfcConsole.TuneSet)
        {
            AfcConsole.TuneSet = order[picked];
            AfcConsole.SavePreferences();
        }
    }

    private static void DrawPreferences()
    {
        if (SharedVehicleHooks.GuidanceEnabled)
        {
            bool active = GuidanceWindow.ModActive;
            if (ConsoleUi.CheckboxRow("GUIDANCE ENABLED".AsSpan(), "AfcTuneGuidance".AsSpan(), ref active,
                    ("Off releases every craft under AFC and stops all guidance drawing. The AFC menu has the same switch.\n" + PageText.TagGameSession).AsSpan()))
                GuidanceWindow.SetModActive(active);
        }
        else
            ConsoleUi.MutedWrapped(GuidanceFeature.UnavailableReason);

        string key = Input.GetAssignmentString(InputAction.ToggleTransferPlan);
        bool route = PlannerRoute.Active;
        if (ConsoleUi.CheckboxRow((key + " OPENS AFC").AsSpan(), "AfcTuneRoute".AsSpan(), ref route,
                ("On: " + key + " and View > Transfer Planner open the console, and TRANSFER > STOCK PLANNER opens the stock planner. Off: they open the stock planner, and the AFC menu opens the console.\n" + PageText.TagEveryCraft).AsSpan()))
            PlannerRoute.SetRouting(route);
        if (!PlannerRoute.Active && PlannerRoute.RoutingPreference)
            ConsoleUi.MutedWrapped("OFF FOR THIS SESSION: CONSOLE FAILED TO DRAW. SWITCH ON TO RETRY.");
        else if (PlannerRoute.Active && PlannerRoute.MenuMissing && !PlannerRoute.RoutingPreference)
            ConsoleUi.MutedWrapped("ON FOR THIS SESSION: AFC MENU NOT LOADED. THE KEY OPENS THE CONSOLE.");

        Switch("OPEN AFC AT FLIGHT START", "AfcTuneOpenAtStart", AfcConsole.KeyOpenAtStart,
            "Opens the console when a flight starts or a save loads.");
        Switch("NUMPAD NUDGES THE HOVER", "AfcTuneNumpad", UiConfig.KeyHoverNumpad,
            "The numpad keys nudge the aim of the craft on screen while it hovers. The game also uses the numpad digits for its HUD layouts, so one press does both.");
        Switch("FOLDS OPEN", "AfcTuneExpertOpen", AfcConsole.KeyExpertOpen,
            "Folds of every page start open.");
        Switch("SHOW STAGING DIAGNOSTICS", "AfcTuneDiagnostics", UiConfig.KeyShowDiagnostics,
            "Shows the rows of the staging bar with their provenance, and the KSA cross-check.");

        if (ConsoleUi.ButtonRow("WINDOW POSITION", PageText.ResetVerb, "AfcTuneResetPlace", "Docks the console where the stock Transfer Planner opens."))
            AfcConsole.ResetWindowPlace();
    }

    private static void Switch(string label, string id, string key, string tooltip)
    {
        bool value = UiConfig.GetBool(key, false);
        if (ConsoleUi.CheckboxRow(label.AsSpan(), id.AsSpan(), ref value, (tooltip + "\n" + PageText.TagEveryCraft).AsSpan()))
            UiConfig.Set(key, value);
    }

    /// <summary>DIAG > FLIGHT PLAN: the stock table of the burn that MANEUVER plans or made, else of the flight plan of the craft.</summary>
    internal static void DrawFlightPlanView(Vehicle craft)
    {
        FlightPlan? planned = OrbitToolsPanel.KeptFlightPlan(Universe.GetElapsedTime());
        FlightPlan? shown = planned ?? (ConsoleFrame.ReadFacts(craft).Where == CraftSituation.Ground ? null : craft.FlightPlan);
        if (shown == null || shown.Patches.Count == 0)
        {
            ConsoleUi.MutedWrapped(PageText.DiagEmpty(TuneDiagnostics.FlightPlan));
            return;
        }
        float linkW = ConsoleUi.LinkWidth(ConsolePages.Label(ConsolePage.Orbit));
        ConsoleUi.BeginRow(FoldNames.FlightPlan.AsSpan(), linkW);
        ConsoleUi.RowChip(FoldNames.FlightPlan, Owner.Stock, "STOCK");
        ConsoleUi.RowRight(linkW);
        if (ConsoleUi.Link(ConsolePages.Label(ConsolePage.Orbit), "AfcFlightPlanBack", PageText.FlightPlanBackTip))
            AfcConsole.SetPage(ConsolePage.Orbit);
        ConsoleUi.EndRow();
        ConsoleUi.MutedWrapped(planned != null ? PageText.FlightPlanOfBurn : PageText.FlightPlanOfCraft);
        shown.DrawPatchInfo();
    }
}
