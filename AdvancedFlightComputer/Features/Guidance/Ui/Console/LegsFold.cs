#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The LEGS fold of the LAND page: whether a landing deploys the landing legs by itself, and at which height. A craft with no legs has the fold as not available.
public static partial class GuidanceWindow
{
    private const string LegsFoldId = "LandLegs";

    private static void DrawLegsFold(Vehicle vehicle, bool openByDefault)
    {
        int legs = KsaLandingLegs.Count(vehicle);
        ConsoleUi.NoteShape(LegsFoldId, legs > 0 ? 1 : 0);
        if (legs == 0)
        {
            ConsoleUi.FoldEmpty(LegsRules.Label(_s.LegsAutoDeploy, _s.LegsDeployHeightM, 0), PageText.LegsNoneTip);
            return;
        }
        if (!ConsoleUi.Fold(LegsRules.Label(_s.LegsAutoDeploy, _s.LegsDeployHeightM, legs), LegsFoldId, openByDefault, PageText.LegsTip))
            return;
        ImGui.PushID(LegsFoldId);
        try
        {
            GaugeRowCheck(Specs.LegsAutoDeploy, ref _s.LegsAutoDeploy);
            using (new ImGuiDisabledScope(!_s.LegsAutoDeploy))
                GaugeRow(Specs.LegsDeployHeightM, ref _s.LegsDeployHeightM);
            GaugeRowText("DEPLOY TIME", PageText.LegsDeployTime(KsaLandingLegs.DeploySeconds(vehicle)));
            string done = LegsRules.Done(_s.LegsFired, _s.LegsFiredAtM, _s.LegsCommanded);
            ConsoleUi.NoteShape("AfcLandLegsDone", done.Length > 0 ? 1 : 0);
            if (done.Length > 0)
                GaugeRowText("THIS LANDING", done);
        }
        finally
        {
            ImGui.PopID();
        }
    }
}
