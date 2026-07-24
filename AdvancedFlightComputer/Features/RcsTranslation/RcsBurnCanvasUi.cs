using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Draws the per-burn RCS block (see <see cref="RcsBurnUi"/>) for the flight
/// burn editor. The 4980 update moved that editor from an ImGui infobox to a
/// gauge canvas: <see cref="BurnCanvasHost.Draw"/> renders the dV/time cells
/// over the fixed, full 'Burn' gauge when <see cref="Program.ActiveBurn"/> is
/// set. There is no room in that gauge for extra rows, so this postfix
/// anchors an AFC panel directly above the gauge (which sits bottom-right),
/// using the pixel rect the host is handed. The panel appears and disappears
/// with the editor, so it reads as part of it rather than a floating window.
/// </summary>
[HarmonyPatch(typeof(BurnCanvasHost), nameof(BurnCanvasHost.Draw))]
internal static class RcsBurnCanvasUi
{
    static void Postfix(float2 canvasMinPixels, float2 canvasSizePixels)
    {
        Burn? burn = Program.ActiveBurn;
        if (burn == null || burn.ParentEjectBurn)
            return;
        Vehicle vehicle = burn.Vehicle;

        // Top-right of the panel pinned to the gauge's bottom-right corner, so
        // it stacks straight below the burn gauge, right edges aligned;
        // AlwaysAutoResize then grows it down and to the left.
        float2 anchor = new float2(
            canvasMinPixels.X + canvasSizePixels.X,
            canvasMinPixels.Y + canvasSizePixels.Y);
        ImGui.SetNextWindowPos(in anchor, ImGuiCond.Always, new float2(1f, 0f));
        // The host popped its own font/style before returning, so this window
        // draws with the default ImGui styling. A new top-level window nested
        // inside the still-open gauge window is a standard ImGui pattern; the
        // matching End must always run, hence the finally.
        ImGui.Begin("RCS###AfcRcsBurnPanel"u8,
            ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse
            | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoSavedSettings
            | ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoNav);
        try
        {
            RcsBurnUi.DrawBlock(burn, vehicle, vehicle.FlightComputer);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("rcs-burn-canvas",
                $"[AFC] RcsBurnCanvasUi failed for vehicle='{vehicle.Id}': {ex}");
        }
        finally
        {
            ImGui.End();
        }
    }
}
