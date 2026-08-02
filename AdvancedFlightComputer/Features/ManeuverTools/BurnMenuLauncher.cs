using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// AFC shortcuts inside stock's orbit right-click menu.
///
/// The four quick-tools cannot be point-anchored the way stock's own Manual and
/// Circularize entries are: each one derives its own burn time (the opposite apse
/// for the apse tools, an ascending or descending node for the inclination tools)
/// and needs a typed input the menu has no room for. These entries therefore open
/// the Transfer Planning window on the matching plan type instead of creating a
/// burn at the clicked point.
/// </summary>
internal static class BurnMenuLauncher
{
    public const int PeriapsisSubmenu = 0;
    public const int ApoapsisSubmenu = 1;

    public static bool Enabled;

    /// <summary>Injected before the EndMenu of stock's "At Periapsis" and "At
    /// Apoapsis" submenus. The tool offered is the one that actually BURNS there:
    /// raising apoapsis is cheapest at periapsis and vice versa, so AFC's apse tools
    /// cross over relative to the submenu they appear under.</summary>
    public static void DrawApsisEntry(int submenu)
    {
        if (!Enabled)
            return;

        try
        {
            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle is KittenEva)
                return;

            ImGui.Separator();
            if (submenu == PeriapsisSubmenu)
            {
                if (ImGui.MenuItem("AFC: Set Apoapsis..."u8))
                    OpenPlanner(ManeuverTools.KeySetApoapsis, vehicle);
            }
            else
            {
                if (ImGui.MenuItem("AFC: Set Periapsis..."u8))
                    OpenPlanner(ManeuverTools.KeySetPeriapsis, vehicle);
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("burn-menu-apsis", $"[AFC] BurnMenuLauncher apsis entry: {ex}");
        }
    }

    /// <summary>Injected before the final EndPopup of
    /// <see cref="BurnContextMenu.Draw"/>. The state check is the real gate rather
    /// than the anchor: the IL may share one EndPopup epilogue across the menu's
    /// modes, and a later build could move the last one into a different branch, so
    /// a misplaced anchor has to degrade to a shortcut in the wrong menu instead of
    /// to a wrong action.</summary>
    public static void DrawInline()
    {
        if (!Enabled)
            return;

        try
        {
            Vehicle? vehicle = Program.ControlledVehicle;
            if (vehicle == null || vehicle is KittenEva)
                return;

            ImGui.Separator();
            if (!ImGui.BeginMenu("Advanced Flight Computer"u8))
                return;

            try
            {
                if (ImGui.MenuItem("Set Apoapsis..."u8))
                    OpenPlanner(ManeuverTools.KeySetApoapsis, vehicle);
                if (ImGui.MenuItem("Set Periapsis..."u8))
                    OpenPlanner(ManeuverTools.KeySetPeriapsis, vehicle);
                if (ImGui.MenuItem("Match Inclination..."u8))
                    OpenPlanner(ManeuverTools.KeyMatchInclination, vehicle);
                if (ImGui.MenuItem("Set Inclination..."u8))
                    OpenPlanner(ManeuverTools.KeySetInclination, vehicle);
            }
            finally
            {
                // BeginMenu returned true, so the matching EndMenu is owed even if
                // an item handler throws; skipping it would nest every later menu
                // inside this one.
                ImGui.EndMenu();
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("burn-menu-launcher", $"[AFC] BurnMenuLauncher: {ex}");
        }
    }

    private static void OpenPlanner(string typeKey, Vehicle vehicle)
    {
        if (FindType(typeKey) is not TransferType type)
            return;

        // Stock cancels a running porkchop whenever its own combo changes the plan
        // type, and its polling block lives in the window body that AFC's prefix
        // replaces for AFC types. Switching here instead would leave the worker
        // unobserved, and the eventual resolution restores the TransferInfo it was
        // built with over whatever source is selected by then. AFC holds no handle
        // on stock's CancellationTokenSource, so refuse the switch rather than
        // strand the calculation.
        if (StockPlanner.TransferBeingCalculated)
        {
            TimedAlert.Create("Transfer calculating; wait for it to finish.", Color.Yellow, 3.0);
            return;
        }

        // During a multi-pass execution the only committed trajectory is the
        // current pass's, and the next pass commit will replace it; the true
        // final orbit exists only as a preview no burn can anchor on. Chaining
        // reopens once the run has finished or been cancelled.
        if (MultiPassRegistry.Has(vehicle.Id))
        {
            TimedAlert.Create("Multi-pass running; cancel it or let it finish first.", Color.Yellow, 3.0);
            return;
        }

        // Only the false path of stock's setter touches its selection state, so
        // setting it true here does not clear a calculated transfer by itself.
        TransferPlanner.ShowPlanWindow = true;
        StockPlanner.SourceBody = new TransferObject(vehicle);
        StockPlanner.TransferType = type;
        StockPlanner.TransferCalculated = false;

        // The window body keys its per-type input defaults off these notifications,
        // not off the values, so a type switched from outside the dropdown would
        // otherwise reopen holding the previous tool's altitude or angle.
        ManeuverToolsWindow.OnTypeChanged();
        ManeuverToolsWindow.OnSourceChanged();
        Patch_DrawPlanWindow.OnManeuverContextChanged();
    }

    /// <summary>Resolves a plan type from the live dropdown list. Returns null once
    /// <see cref="ManeuverTools.RemoveTransferTypes"/> has run, which is what makes
    /// a click during unload a no-op rather than a window with no body.</summary>
    private static TransferType? FindType(string key)
    {
        foreach (TransferType candidate in TransferPlanner.TransferTypes)
        {
            if (candidate.GetKey() == key)
                return candidate;
        }
        return null;
    }
}
