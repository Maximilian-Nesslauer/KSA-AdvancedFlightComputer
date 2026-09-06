using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.PlanWindow;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// Each quick tool calculates its own burn time and needs input from the user, so the shortcut opens the planner instead of placing a burn at the clicked point.
internal static class BurnMenuLauncher
{
    public const int PeriapsisSubmenu = 0;
    public const int ApoapsisSubmenu = 1;

    public static bool Enabled;

    // An apse burn changes the opposite apse. The periapsis submenu therefore offers Set Apoapsis, and the apoapsis submenu offers Set Periapsis.
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
                // Balance the menu even if an item handler throws.
                ImGui.EndMenu();
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("burn-menu-launcher", $"[AFC] BurnMenuLauncher: {ex}");
        }
    }

    internal static void OpenPlanner(string typeKey, Vehicle vehicle)
    {
        if (FindType(typeKey) is not TransferType type)
            return;

        // TransferPlanner.DrawPlanWindow polls the stock worker only in its own body. Do not switch to an AFC body while that worker is running.
        if (StockPlanner.TransferBeingCalculated)
        {
            TimedAlert.Create("Transfer calculating; wait for it to finish.", Color.Yellow, 3.0);
            return;
        }

        // The next pass replaces the committed trajectory, so another maneuver can be planned only after all passes finish.
        if (MultiPassRegistry.Has(vehicle.Id))
        {
            TimedAlert.Create("Multi-pass running; cancel it or let it finish first.", Color.Yellow, 3.0);
            return;
        }

        // TransferPlanner.ShowPlanWindow clears stock selection state only when set to false.
        TransferPlanner.ShowPlanWindow = true;
        StockPlanner.SourceBody = new TransferObject(vehicle);
        StockPlanner.TransferType = type;
        StockPlanner.TransferCalculated = false;

        // Reset input defaults when a shortcut changes the source or type outside the dropdown.
        ManeuverToolsWindow.OnTypeChanged();
        ManeuverToolsWindow.OnSourceChanged();
        Patch_DrawPlanWindow.OnManeuverContextChanged();
    }

    // Resolve from the live list so a shortcut cannot reopen a removed plan type.
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
