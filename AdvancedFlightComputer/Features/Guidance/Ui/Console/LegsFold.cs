#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The LEGS fold of the LAND page: whether a landing deploys the landing legs by itself, and at which time to the touchdown. A craft with no legs has the fold as not available.
public static partial class GuidanceWindow
{
    private const string LegsFoldId = "LandLegs";

    private static void DrawLegsFold(Vehicle vehicle, bool openByDefault)
    {
        int legs = KsaLandingLegs.Count(vehicle);
        ConsoleUi.NoteShape(LegsFoldId, legs > 0 ? 1 : 0);
        if (legs == 0)
        {
            ConsoleUi.FoldEmpty(LegsRules.Label(_s.LegsAutoDeploy, double.NaN, 0), PageText.LegsNoneTip);
            return;
        }
        double legsS = KsaLandingLegs.DeploySeconds(vehicle);
        double deployAt = LegsRules.DeployAtS(legsS, _s.LegsDeployBufferS);
        if (!ConsoleUi.Fold(LegsRules.Label(_s.LegsAutoDeploy, deployAt, legs), LegsFoldId, openByDefault, PageText.LegsTip))
            return;
        ImGui.PushID(LegsFoldId);
        try
        {
            GaugeRowCheck(Specs.LegsAutoDeploy, ref _s.LegsAutoDeploy);
            GaugeRowText("DEPLOY TIME", PageText.LegsDeployTime(legsS));
            using (new ImGuiDisabledScope(!_s.LegsAutoDeploy))
            {
                GaugeRow(Specs.LegsDeployBufferS, ref _s.LegsDeployBufferS);
                GaugeRowText("DEPLOYS AT", LegsRules.ToGo(deployAt));
            }
            string done = LegsRules.Done(_s.LegsFired, _s.LegsFiredToGoS, _s.LegsFiredAtM, _s.LegsCommanded);
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
