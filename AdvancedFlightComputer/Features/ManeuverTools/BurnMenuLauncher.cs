using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Console;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// Each quick tool calculates its own burn time and needs input from the user, so the shortcut opens the tool on the MANEUVER page of the AFC console instead of placing a burn at the clicked point.
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
                    OpenOrbitTool(ManeuverTools.KeySetApoapsis, vehicle);
            }
            else
            {
                if (ImGui.MenuItem("AFC: Set Periapsis..."u8))
                    OpenOrbitTool(ManeuverTools.KeySetPeriapsis, vehicle);
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("burn-menu-apsis:" + ex.GetType().Name,
                $"[AFC] BurnMenuLauncher apsis entry: {ex}");
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
                if (ImGui.MenuItem("Circularize..."u8))
                    OpenOrbitTool(ManeuverTools.KeyCircularizeApoapsis, vehicle);
                if (ImGui.MenuItem("Set Apoapsis..."u8))
                    OpenOrbitTool(ManeuverTools.KeySetApoapsis, vehicle);
                if (ImGui.MenuItem("Set Periapsis..."u8))
                    OpenOrbitTool(ManeuverTools.KeySetPeriapsis, vehicle);
                if (ImGui.MenuItem("Match Inclination..."u8))
                    OpenOrbitTool(ManeuverTools.KeyMatchInclination, vehicle);
                if (ImGui.MenuItem("Set Inclination..."u8))
                    OpenOrbitTool(ManeuverTools.KeySetInclination, vehicle);
                if (ImGui.MenuItem("Open AFC"u8))
                    AfcConsole.Open();
            }
            finally
            {
                // Balance the menu even if an item handler throws.
                ImGui.EndMenu();
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("burn-menu-launcher:" + ex.GetType().Name,
                $"[AFC] BurnMenuLauncher: {ex}");
        }
    }

    /// <summary>Opens the AFC console on the MANEUVER page with the tool and the craft selected.</summary>
    internal static void OpenOrbitTool(string toolKey, Vehicle vehicle)
    {
        // The next pass replaces the committed trajectory, so another maneuver can be planned only after all passes finish.
        if (MultiPassRegistry.Has(vehicle.Id))
        {
            AfcAlerts.Info("Multi-pass running; cancel it or let it finish first.", Color.Yellow, 3.0, vehicle);
            AfcConsole.Open();
            return;
        }
        AfcConsole.OpenOrbitTool(toolKey, vehicle);
    }
}
