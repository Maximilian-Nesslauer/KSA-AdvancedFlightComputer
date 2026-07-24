using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Draws the per-burn RCS block below the flight burn editor's gauge, anchored
/// to the gauge rect <see cref="BurnCanvasHost.Draw"/> is handed, so it appears
/// and disappears with the editor.
///
/// Two renderers, picked by whether the canvas is docked: normally the
/// gauge-styled <see cref="RcsGaugePanel"/>, but ImGauge only ever draws into
/// the main viewport, so a canvas the player detached onto its own OS window
/// falls back to the plain ImGui block.
/// </summary>
[HarmonyPatch(typeof(BurnCanvasHost), nameof(BurnCanvasHost.Draw))]
internal static class RcsBurnCanvasUi
{
    static void Postfix(GaugeCanvas canvas, float2 canvasMinPixels, float2 canvasSizePixels)
    {
        Burn? burn = Program.ActiveBurn;
        if (burn == null || burn.ParentEjectBurn)
            return;
        Vehicle vehicle = burn.Vehicle;
        try
        {
            if (canvas.Detached)
                DrawImGuiFallback(burn, vehicle, canvasMinPixels, canvasSizePixels);
            else
                RcsGaugePanel.Draw(burn, vehicle, vehicle.FlightComputer,
                    canvasMinPixels, canvasSizePixels);
        }
        catch (Exception ex)
        {
            // Once per load: this runs every frame the editor is open, and a
            // persistent draw failure would otherwise flood the log.
            LogHelper.WarnOnce("rcs-burn-canvas",
                $"[AFC] RcsBurnCanvasUi failed for vehicle='{vehicle.Id}': {ex}");
        }
    }

    private static void DrawImGuiFallback(
        Burn burn, Vehicle vehicle, float2 canvasMinPixels, float2 canvasSizePixels)
    {
        // Top-right pinned to the gauge's bottom-right corner; AlwaysAutoResize
        // then grows the window down and to the left.
        float2 anchor = new float2(
            canvasMinPixels.X + canvasSizePixels.X,
            canvasMinPixels.Y + canvasSizePixels.Y);
        ImGui.SetNextWindowPos(in anchor, ImGuiCond.Always, new float2(1f, 0f));
        ImGui.Begin("RCS###AfcRcsBurnPanel"u8,
            ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse
            | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoSavedSettings
            | ImGuiWindowFlags.NoFocusOnAppearing | ImGuiWindowFlags.NoNav);
        try
        {
            RcsBurnUi.DrawBlock(burn, vehicle, vehicle.FlightComputer);
        }
        finally
        {
            // The matching End must run even if the block throws, or every
            // later ImGui window nests inside this one.
            ImGui.End();
        }
    }
}
