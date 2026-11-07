using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Draws the compact RCS block below the flight burn editor's gauge, anchored
/// to the gauge rect <see cref="BurnCanvasHost.Draw"/> is handed, so it appears
/// and disappears with the editor. It is the gauge-styled
/// <see cref="RcsGaugePanel"/>. ImGauge only ever draws into the main viewport,
/// so a canvas the player detached onto its own OS window gets no block; the RCS
/// page of the AFC console holds the same options for the planned nodes either way.
/// </summary>
[HarmonyPatch(typeof(BurnCanvasHost), nameof(BurnCanvasHost.Draw),
    new[] { typeof(GaugeCanvas), typeof(float2), typeof(float2) })]
internal static class RcsBurnCanvasUi
{
    static void Postfix(GaugeCanvas canvas, float2 canvasMinPixels, float2 canvasSizePixels)
    {
        Burn? burn = Program.ActiveBurn;
        if (burn == null || burn.ParentDepartureBurn)
            return;
        Vehicle vehicle = burn.Vehicle;
        if (canvas.Detached)
            return;
        try
        {
            RcsGaugePanel.Draw(burn, vehicle, vehicle.FlightComputer,
                canvasMinPixels, canvasSizePixels);
        }
        catch (Exception ex)
        {
            // Once per load: this runs every frame the editor is open, and a persistent draw failure would otherwise flood the log.
            LogHelper.WarnOnce("rcs-burn-canvas:" + ex.GetType().Name,
                $"[AFC] RcsBurnCanvasUi failed for vehicle='{vehicle.Id}': {ex}");
        }
    }

}
