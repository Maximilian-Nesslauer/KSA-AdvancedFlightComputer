using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Injects the per-burn RCS block (see <see cref="RcsBurnUi"/>) into the
/// stock rendezvous burn infobox with a postfix on
/// <see cref="Burn.DrawBurnEditorWindowContent"/>. TargetTrackWindow is the
/// only stock caller of that infobox, and the flight burn editor on the gauge
/// canvas is covered by <see cref="RcsBurnCanvasUi"/>.
/// </summary>
[HarmonyPatch(typeof(Burn), nameof(Burn.DrawBurnEditorWindowContent),
    new Type[] { typeof(Vehicle), typeof(FlightComputer), typeof(bool), typeof(bool) })]
internal static class RcsBurnWindowUi
{
    static void Postfix(Burn __instance, Vehicle vehicle, FlightComputer flightComputer)
    {
        try
        {
            RcsBurnUi.DrawBlock(__instance, vehicle, flightComputer);
        }
        catch (Exception ex)
        {
            // Once per load: this runs every frame the editor is open, and a persistent draw failure would otherwise flood the log.
            LogHelper.WarnOnce("rcs-burn-window:" + ex.GetType().Name,
                $"[AFC] RcsBurnWindowUi failed for vehicle='{vehicle.Id}': {ex}");
        }
    }
}
