#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Linq;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.LandingSites;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The named landing sites of mods (LandingSiteRegistry) as a list beside PICK SITE: RETURN TO on each booster row of LAUNCH, LAND AT under the site row of LAND. A pick writes the same target that PICK SITE writes, so it is kept the same way.
public static partial class GuidanceWindow
{
    /// <summary>The RETURN TO list of the stage rooted at <paramref name="rootId"/>, measured from <paramref name="fromCcf"/> (body-fixed, m or any length). A site that another booster holds names that booster.</summary>
    internal static List<SiteChoice> StageSiteChoices(VehicleAutopilotState state, IParentBody parent, double3 fromCcf, uint rootId)
    {
        IReadOnlyList<LandingSite> sites = LandingSiteRegistry.On(parent);
        if (sites.Count == 0)
            return [];
        double3 lla = parent.GetLlaFromCcf(fromCcf);
        var points = StageTargetPoints(state).ToList();
        return LandingSiteRules.Order(sites, lla.X, lla.Y, parent.MeanRadius,
            site => LandingSiteRules.Owner(site, sites, points, rootId, parent.MeanRadius) is uint owner ? StageIdText(state, owner) : "");
    }

    /// <summary>The LAND AT list, measured from <paramref name="fromCcf"/>. With a known impact time and surface speed, each site carries the reach of the far-site rule of LAND.</summary>
    internal static List<SiteChoice> LandSiteChoices(IParentBody parent, double3 fromCcf, double impactS, double surfaceSpeedMs)
    {
        IReadOnlyList<LandingSite> sites = LandingSiteRegistry.On(parent);
        if (sites.Count == 0)
            return [];
        double3 lla = parent.GetLlaFromCcf(fromCcf);
        return LandingSiteRules.Order(sites, lla.X, lla.Y, parent.MeanRadius,
            inReach: m => LandingSiteRules.InReachOfFall(m, impactS, surfaceSpeedMs));
    }

    internal static string StageIdText(VehicleAutopilotState state, uint rootId)
    {
        var list = state.ReturnableStages;
        int index = list.FindIndex(stage => stage.RootId == rootId);
        return index >= 0 ? ReturnableStageIdText(list, index) : PageText.AnotherBooster;
    }

    private static double3 CraftCcf(Vehicle vehicle)
        => vehicle.Orbit.StateVectors.PositionCci.Transform(vehicle.Orbit.Parent.GetCci2Ccf());

    // The launch site: the craft while it stands on it, and the start of the plan in flight.
    private static double3 LaunchOriginCcf(Vehicle vehicle)
        => _s.Running && _s.AscentPlan is { } plan && plan.StartCcf.LengthSquared() > 0.0 ? plan.StartCcf : CraftCcf(vehicle);

    /// <summary>RETURN TO under the name row of a booster: the named sites of the body by distance from the launch site.</summary>
    private static void DrawStageSiteRow(Vehicle vehicle, ReturnableStage stage)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null || LandingSiteRegistry.On(parent).Count == 0)
            return;
        LandingSite on = stage.HasTarget ? LandingSiteRegistry.Holding(parent, stage.TargetLatDeg, stage.TargetLonDeg) : null;
        string preview = on != null ? on.Name : stage.HasTarget ? PageText.SiteListPickedPoint : PageText.SiteListChoose;
        BeginKitRow(PageText.ReturnTo);
        bool rowHovered = ConsoleWidgets.RowHovered;
        bool open = ConsoleUi.BeginCombo($"AfcStageSites{stage.RootId}".AsSpan(), preview);
        if (!open && rowHovered)
            ConsoleUi.Card(PageText.ReturnTo, PageText.ReturnToTip);
        if (open)
        {
            List<SiteChoice> choices = StageSiteChoices(_s, parent, LaunchOriginCcf(vehicle), stage.RootId);
            LandingSite picked = DrawSiteChoices(choices, on, $"AfcStageSite{stage.RootId}");
            if (picked != null)
                SetStageTarget(_s, stage.RootId, picked.LatDeg, picked.LonDeg, parent);
            ConsoleWidgets.EndComboControl();
        }
        ConsoleUi.EndRow();
    }

    /// <summary>LAND AT under the site row: the named sites of the body by distance from the craft, those in reach of a fall marked.</summary>
    private static void DrawLandSiteList(Vehicle vehicle, IParentBody parent, LandWay way)
    {
        if (LandingSiteRegistry.On(parent).Count == 0)
            return;
        LandingSite on = NoSitePicked(parent) ? null : LandingSiteRegistry.Holding(parent, _s.SiteLatDeg, _s.SiteLonDeg);
        BeginKitRow(PageText.LandAt);
        bool rowHovered = ConsoleWidgets.RowHovered;
        using (new ImGuiDisabledScope(DeorbitTargetLocked))
        {
            bool open = ConsoleUi.BeginCombo("AfcLandSites".AsSpan(), on != null ? on.Name : PageText.SiteListChoose);
            if (!open && rowHovered)
                ConsoleUi.Card(PageText.LandAt, DeorbitTargetLocked ? PageText.SiteLockedByDeorbit : PageText.LandAtTip);
            if (open)
            {
                bool falling = way == LandWay.FromHere && !vehicle.Situation.HasAnyContact();
                double impact = falling ? ImpactSeconds(vehicle, guidanceBound: true) : double.NaN;
                List<SiteChoice> choices = LandSiteChoices(parent, CraftCcf(vehicle), impact, vehicle.GetSurfaceSpeed());
                LandingSite picked = DrawSiteChoices(choices, on, "AfcLandSite");
                if (picked != null)
                    RetargetVehicleSite(picked.LatDeg, picked.LonDeg);
                ConsoleWidgets.EndComboControl();
            }
        }
        ConsoleUi.EndRow();
    }

    // A group name stands over its sites; a site that another booster holds stands dim with that booster's name.
    private static LandingSite DrawSiteChoices(List<SiteChoice> choices, LandingSite current, string idPrefix)
    {
        LandingSite picked = null;
        foreach (SiteChoice choice in choices)
        {
            if (choice.GroupStart)
                ImGui.TextDisabled(ConsoleUi.EscapeFormat(choice.Site.Group));
            bool taken = choice.TakenBy.Length > 0;
            string label = (choice.Site.Group.Length > 0 ? "  " : "") + PageText.SiteChoice(choice.Site.Name.Replace("##", "#"), choice.DistanceM, choice.TakenBy, choice.InReach)
                + "##" + idPrefix + choice.Site.Id;
            using (new ImGuiDisabledScope(taken))
            {
                if (ImGui.Selectable(label, ReferenceEquals(choice.Site, current)) && !taken)
                    picked = choice.Site;
            }
        }
        return picked;
    }
}
