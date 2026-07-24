using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Injects the per-burn RCS block (see <see cref="RcsBurnUi"/>) into the
/// stock rendezvous burn infobox: a postfix on
/// <see cref="Burn.DrawBurnEditorWindowContent"/>. As of 4980 the game only
/// draws that infobox from TargetTrackWindow (the normal flight burn editor
/// moved to the gauge canvas - see <see cref="RcsBurnCanvasUi"/>), so this
/// surface now covers the rendezvous case only.
/// </summary>
[HarmonyPatch(typeof(Burn), nameof(Burn.DrawBurnEditorWindowContent))]
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
            // Once per load: this runs every frame the editor is open, and a
            // persistent draw failure would otherwise flood the log.
            LogHelper.WarnOnce("rcs-burn-window",
                $"[AFC] RcsBurnWindowUi failed for vehicle='{vehicle.Id}': {ex}");
        }
    }
}
