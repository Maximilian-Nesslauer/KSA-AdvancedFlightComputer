#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Ascent tab's page for the gravity-turn ascent, shown in place of the CAT-S / UPFG sections when it is selected. See Modes/GravityTurn.cs.
public static partial class GuidanceWindow
{
    private static void DrawGravityTurnTabContent(float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Gravity turn",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        var dim = new float4(0.7f, 0.7f, 0.7f, 1f);
        GaugeRowText("Status", "placeholder, not implemented yet", dim);
        GaugeRowText("EXECUTE", "does nothing while Gravity turn is selected", dim);

        ImGuiHelper.EndRegion();
    }
}
