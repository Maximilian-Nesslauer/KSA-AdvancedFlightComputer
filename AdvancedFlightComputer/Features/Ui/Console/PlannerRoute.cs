using System;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>
/// Routes the planner key to the AFC console. The planner key (InputAction.ToggleTransferPlan, F4 by
/// default in Input.GenerateDefault) and View > Transfer Planner both toggle the public setter
/// TransferPlanner.ShowPlanWindow (Program.DispatchKeyEvent and Program.DrawMenuBar). A prefix on that
/// setter turns a false-to-true write that AFC did not ask for into a console toggle and leaves the
/// stock window closed and its fields untouched. Every close passes, and so does the per-frame write
/// TransferPlanner.DrawPlanWindow makes while its own window is open.
/// The setter is small, so a caller compiled before the patch may hold an inlined copy that the
/// prefix never sees. Whether the JIT does that is not verified in the decompiled source, so a frame
/// check runs every frame as well: an open stock window that AFC did not open is closed again and
/// becomes a console toggle.
/// </summary>
internal static class PlannerRoute
{
    internal const string KeyRoutePlannerKey = "route_planner_key";

    /// <summary>The F4 OPENS AFC switch together with a console that loaded. Off gives the key back to stock.</summary>
    internal static bool Active;

    /// <summary>
    /// The AFC menu did not load, so the planner key is the one sure way into the console. A stored
    /// switch that is off is then ignored for the session, or the player could not reach the switch again.
    /// </summary>
    internal static bool MenuMissing;

    /// <summary>
    /// True while the stock window is open on purpose: from the STOCK PLANNER link, or from a key press
    /// the console did not take (routing off, or no craft for the console). The stock window then holds
    /// the dock and the console steps aside.
    /// </summary>
    internal static bool StockLinkActive { get; private set; }

    private static bool _stockLinkPending;
    private static bool _togglePending;
    private static bool _toggleEditor;
    private static bool _frameCheckLogged;
    private static bool _prefixApplied;

    internal static bool PrefixApplied => _prefixApplied;

    /// <summary>The public property setter the prefix sits on, or null when it is gone.</summary>
    internal static MethodInfo? Setter => GameReflection.TransferPlanner_set_ShowPlanWindow;

    /// <summary>Applies the setter prefix. A failure is logged and leaves the frame check to route alone.</summary>
    internal static void ApplyPatch(Harmony harmony)
    {
        MethodInfo? setter = Setter;
        if (setter == null)
        {
            DefaultCategory.Log.Warning("[AFC] TransferPlanner.ShowPlanWindow has no setter, so the planner key is routed by the frame check only.");
            return;
        }
        try
        {
            harmony.Patch(setter, prefix: new HarmonyMethod(typeof(PlannerRoute), nameof(SetterPrefix)));
            _prefixApplied = true;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] The planner key prefix did not apply, so the frame check routes the key alone: {ex.Message}");
        }
    }

    internal static void LoadPreference()
    {
        UiConfig.SetDefault(KeyRoutePlannerKey, true);
        Active = UiConfig.GetBool(KeyRoutePlannerKey, true);
    }

    /// <summary>
    /// Turns the switch on or off and saves it. A stock window that is open when routing turns on stays
    /// open as if the STOCK PLANNER link had opened it, so the frame check does not take it for a key
    /// press that went past the prefix.
    /// </summary>
    internal static void SetRouting(bool on)
    {
        UiConfig.Set(KeyRoutePlannerKey, on);
        Active = (on || MenuMissing) && AfcConsole.Enabled;
        if (Active && TransferPlanner.ShowPlanWindow)
            StockLinkActive = true;
    }

    internal static bool RoutingPreference => UiConfig.GetBool(KeyRoutePlannerKey, true);

    /// <summary>Whether a key press goes to the console now, see RouteDecision.RoutesKey.</summary>
    internal static bool RoutesNow => RouteDecision.RoutesKey(Active, Program.IsEditorOpen, Program.ControlledVehicle != null);

    // Writes no stock field: it only reads the public getter and raises a request the console applies at its next draw.
    private static bool SetterPrefix(bool value)
    {
        try
        {
            if (!value)
            {
                OnStockWindowClosed();
                return true;
            }
            bool current = TransferPlanner.ShowPlanWindow;
            RouteAction action = RouteDecision.Decide(value, current, _stockLinkPending, RoutesNow);
            if (action == RouteAction.Pass)
            {
                // An open that passes, from the link or from a key the console did not take, gives the stock window the dock.
                if (!current)
                {
                    _stockLinkPending = false;
                    StockLinkActive = true;
                }
                return true;
            }
            RequestToggle(Program.IsEditorOpen);
            return false;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("planner-route:" + ex.GetType().Name, $"[AFC] Planner key routing failed, the stock planner opens: {ex}");
            return true;
        }
    }

    internal static void RequestToggle(bool editor)
    {
        _togglePending = true;
        _toggleEditor = editor;
    }

    /// <summary>The request a key press left, consumed once by the console.</summary>
    internal static bool ConsumeToggle(out bool editor)
    {
        editor = _toggleEditor;
        if (!_togglePending)
            return false;
        _togglePending = false;
        return true;
    }

    /// <summary>Opens the stock Transfer Planner on purpose, the one path to it from the console.</summary>
    internal static void OpenStockPlanner()
    {
        if (TransferPlanner.ShowPlanWindow)
        {
            StockLinkActive = true;
            return;
        }
        _stockLinkPending = true;
        TransferPlanner.ShowPlanWindow = true;
        // The prefix moves the pending flag to active. Without the prefix the write went through as it was.
        if (_stockLinkPending)
        {
            _stockLinkPending = false;
            StockLinkActive = TransferPlanner.ShowPlanWindow;
        }
    }

    internal static void OnStockWindowClosed()
    {
        _stockLinkPending = false;
        StockLinkActive = false;
    }

    /// <summary>
    /// Runs every frame from the console tick. A stock window that is open although the console routes
    /// the key, and that AFC did not open, was opened past the prefix, so it is closed through the public
    /// setter and becomes a console toggle. Program.DrawMenuBar runs after this check in the same frame
    /// (Program.OnDrawUiThreadSafe), so a View > Transfer Planner write that went past the prefix draws
    /// the stock window for one frame before it closes.
    /// </summary>
    internal static void FrameCheck()
    {
        bool open = TransferPlanner.ShowPlanWindow;
        if (!open && StockLinkActive)
            StockLinkActive = false;
        bool routes = RoutesNow;
        if (RouteDecision.FrameCheckAdopts(open, StockLinkActive, routes))
        {
            StockLinkActive = true;
            return;
        }
        if (!RouteDecision.FrameCheckRoutes(open, StockLinkActive, routes))
            return;
        TransferPlanner.ShowPlanWindow = false;
        RequestToggle(Program.IsEditorOpen);
        if (!_frameCheckLogged)
        {
            _frameCheckLogged = true;
            DefaultCategory.Log.Info("[AFC] Planner key routed by the frame check.");
        }
    }

    internal static void Reset()
    {
        Active = false;
        MenuMissing = false;
        StockLinkActive = false;
        _stockLinkPending = false;
        _togglePending = false;
        _toggleEditor = false;
        _frameCheckLogged = false;
        _prefixApplied = false;
    }
}
