using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Features.Ui.Pictures;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>MANEUVER (key ORBIT): the quick orbit tools on the TOOL tabs. Under them stand the inputs of the tool, PASSES, and CRAFT when another craft can be picked; then the orbit sketch, or the plane sketch for the plane tools, with the fuel of the burn or the rows of the passes.</summary>
internal static class OrbitPage
{
    private static readonly List<TransferObject> _sourceBuffer = new();
    private static readonly List<OrbitSketch.Track> _tracks = new();
    private static readonly List<BurnBar> _bars = new();
    private static CircularizeAt _at = CircularizeAt.Apoapsis;
    private static string _fuelFact = "";

    // What the craft holds for this plan, from the fuel rows of this frame, for the answer row.
    private static double _budgetMs = double.NaN;
    private static readonly Picture.Swatch[] _legend = new Picture.Swatch[4];

    public const string UnavailableText = "Orbit tools are unavailable because their game hooks did not load.";

    public static CommitSpec Draw(in PageContext ctx)
    {
        if (!OrbitToolsPanel.Enabled)
        {
            ConsoleUi.MutedWrapped(UnavailableText);
            return CommitSpec.StatusOnly("Orbit tools unavailable");
        }

        ConsoleUi.ClearPageWarning();
        Vehicle subject = ctx.Subject;
        Vehicle? source = OrbitToolsPanel.Source;
        if (source == null || source.IsDisposed)
        {
            OrbitToolsPanel.OnSubjectChanged(subject);
            source = subject;
        }

        bool otherSource = !ReferenceEquals(source, subject);
        SituationFacts situation = otherSource ? ConsoleFrame.ReadFacts(source) : ctx.Frame.Situation;
        if (OrbitToolsPanel.ApplyStartTool(source, situation.Where))
            ConsoleUi.ShapeChanged = true;

        if (!ctx.Flying)
            ctx.ToInputs();
        string key = ctx.Flying ? OrbitToolsPanel.RunningToolKey(source) : DrawToolChooser(source);
        OrbitToolsPanel.Plan plan = OrbitToolsPanel.Prepare(key, source, situation.Where);

        OrbitToolsPanel.DrawForm(key, source, in plan, situation.Where);
        // A pick counts from the next frame.
        if (!ctx.Flying)
            DrawSourceRow(source, subject);
        ctx.ToPicture();
        DrawPicture(in ctx, key, source, in plan, otherSource, situation.Where);

        ConsoleUi.Gap();
        if (ctx.Flying)
        {
            (string goalWords, int goalPasses) = RunningGoal(source);
            if (ConsoleUi.Fold(FlightView.PlanLabel(PageText.PassesPlan(goalWords, goalPasses)), FlightView.PlanFoldId, false, PageText.PlanTip) && ctx.ToPlanForm())
            {
                ctx.PlanHead();
                string goal = PageText.PassesGoal(goalWords, goalPasses);
                ConsoleUi.NoteShape("AfcPassGoal", goal.Length > 0 ? 1 : 0);
                if (goal.Length > 0)
                    ConsoleUi.Sentence(goal.AsSpan(), in ConsoleStyle.TextPrimary);
                ConsoleUi.MutedWrapped(FlightView.PassesPlanLine);
                using (ConsoleUi.Dim(true))
                {
                    DrawSourceRow(source, subject);
                    DrawToolChooser(source);
                    DrawFolds(in ctx, key, source, situation.Where);
                }
            }
        }
        else
        {
            ctx.ToInputs();
            DrawFolds(in ctx, key, source, situation.Where);
        }

        CommitSpec commit = OrbitToolsPanel.BuildCommit(key, source, in plan, Refusal(in plan), _budgetMs);
        if (plan.State == OrbitToolsPanel.CommitState.NodeCreated && commit.Verb.Length > 0)
            commit = commit with { Caution = _fuelFact };
        CraftActivity activity = ctx.Frame.Activity;
        bool ascentFlies = !otherSource && activity.AscentRunning;
        bool otherFlies = !otherSource && !ascentFlies && activity.AnyGuidanceLive;
        bool holdsGoal = ctx.Frame.GuidanceOn && !otherSource && !OrbitToolsPanel.ToolPicked && OrbitToolCatalog.ToolIndex(key) == 0 && GuidanceWindow.GoalOrbitHolds(in situation);
        return WithWorstFact(commit, key, source, in plan, situation.Where, ascentFlies, otherFlies, holdsGoal);
    }

    internal static (string Goal, int Passes) RunningGoal(Vehicle source)
    {
        if (!MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
            return ("", 0);
        double radius = source.Parent?.MeanRadius ?? 0.0;
        string goal = exec.Intent switch
        {
            ApseIntent apse => (apse.IsSetApoapsis ? "Ap " : "Pe ") + Fmt.Km((apse.TargetRadiusMeters - radius) / 1000.0),
            SetInclinationIntent inc => "Inc " + Fmt.Deg(inc.TargetInclinationRad * 180.0 / Math.PI),
            CircularizeIntent => "circular orbit",
            MatchInclinationIntent match => "the plane of " + match.TargetId,
            HohmannTransferIntent transfer => "to " + transfer.TargetId,
            _ => "",
        };
        return (goal, exec.PassCountTotal);
    }

    private static void DrawFolds(in PageContext ctx, string key, Vehicle source, CraftSituation where)
    {
        bool ground = where == CraftSituation.Ground;
        PageKit.Numbers("orbit",
        [
            .. ground ? [] : OrbitToolForm.DetailParts(),
            new FoldPart(FoldNames.FlightPlan, !ground && OrbitToolsPanel.PlannedFlightPlan != null, DrawFlightPlanLink),
        ], ctx.FoldsOpen);

        bool match = key == ManeuverTools.ManeuverTools.KeyMatchInclination;
        bool split = MultiPassUI.SplitRowShown(source, key);
        int changed = (split && MultiPassUI.CurrentSplitMode != SplitMode.EqualBurnTime ? 1 : 0)
            + (match && OrbitToolForm.SetsCraftTarget ? 1 : 0);
        PageKit.HowItFlies("orbit",
        [
            new FoldPart(FoldNames.Split, split, () => MultiPassUI.DrawSplitRow(source, key), MultiPassUI.SplitSummary),
            new FoldPart(FoldNames.CraftTarget, match, () => OrbitToolForm.DrawSetTargetRow(source), OrbitToolForm.SetsCraftTarget ? "Craft target on" : "Craft target off"),
        ], changed, ctx.FoldsOpen);
    }

    // The stock table of the plan is raw stock UI with clock times, so it stands on TUNE > DIAG with its STOCK chip, and this row leads there.
    private static void DrawFlightPlanLink()
    {
        float w = ConsoleUi.LinkWidth(PageText.FlightPlanLink);
        ConsoleUi.BeginRow(FoldNames.FlightPlan.AsSpan(), w);
        ConsoleUi.RowRight(w);
        if (ConsoleUi.Link(PageText.FlightPlanLink, "OrbitFlightPlanTune", PageText.FlightPlanLinkTip))
            TunePage.ShowFlightPlan();
        ConsoleUi.EndRow();
    }

    private static CommitSpec WithWorstFact(CommitSpec commit, string key, Vehicle source, in OrbitToolsPanel.Plan plan, CraftSituation where, bool ascentFlies, bool otherFlies, bool holdsGoal)
    {
        if (plan.State != OrbitToolsPanel.CommitState.Ready)
            return commit;
        VerbFit fit = SituationVerbs.ForManeuver(where, ascentFlies, otherFlies);
        CommitAction? next = PageKit.NextLink(in fit);
        if (!commit.Enabled)
            return fit.Fits ? commit : commit with { Refusal = fit.Text, Secondary = next, Calm = fit.Calm };
        if (!fit.Fits)
            return commit with { Caution = fit.Text, Secondary = next, Calm = fit.Calm };
        if (holdsGoal)
            return commit with { Caution = PageText.GoalOrbitReached, Calm = true };
        string caution = OrbitToolForm.Problem;
        if (plan.Maneuver is OrbitManeuvers.ManeuverResult m)
        {
            if (caution.Length == 0 && BurnAfterImpact(plan.Basis.Plan ?? source.FlightPlan, m.BurnTime))
            {
                var land = new VerbFit(PageText.BurnAfterImpact, ConsolePage.Land, LandWay.FromHere);
                return commit with { Caution = land.Text, Secondary = PageKit.NextLink(in land), Calm = land.Calm };
            }
            if (caution.Length == 0)
                caution = _fuelFact;
            if (caution.Length == 0)
                caution = MultiPassUI.FuelShortFact(key, (plan.Impulsive ?? m).DvCci.Length());
        }
        if (caution.Length == 0 && ConsoleUi.PageWarning.Length > 0)
            return commit with { Quiet = true };
        return commit with { Caution = caution };
    }

    // The trajectory is the one the burn starts from, so a plan chained after a burn reads Burn.FlightPlan.
    private static bool BurnAfterImpact(FlightPlan trajectory, UniverseTime burn)
    {
        foreach (PatchedConic patch in trajectory.Patches)
        {
            if (patch.EndTransition == PatchTransition.Impact)
                return burn > patch.EndTime;
        }
        return false;
    }

    private static string DrawToolChooser(Vehicle source)
    {
        string key = OrbitToolsPanel.ToolKey;
        OrbitToolCatalog.CircularizeSide(key, ref _at);
        int picked = ConsoleUi.PlaceTabs("", "AfcOrbitTool", OrbitToolCatalog.ToolLabels, OrbitToolCatalog.ToolIndex(key),
            OrbitToolCatalog.ToolDescriptions);
        int tool = picked >= 0 ? picked : OrbitToolCatalog.ToolIndex(key);
        if (tool == 0)
        {
            int at = ConsoleUi.ChoiceRow(OrbitToolForm.BurnAtLabel, "AfcOrbitAt", OrbitToolCatalog.AtLabels, _at == CircularizeAt.Apoapsis ? 0 : 1, feedsPicture: true);
            if (at >= 0)
            {
                _at = at == 0 ? CircularizeAt.Apoapsis : CircularizeAt.Periapsis;
                picked = tool;
            }
        }
        string chosen = OrbitToolCatalog.KeyForTool(tool, _at);
        if (chosen != key || picked >= 0)
        {
            OrbitToolsPanel.Pick(chosen, source);
            key = chosen;
        }
        return key;
    }

    private static Burn? MadeBurn(Vehicle source, in OrbitToolsPanel.Plan plan)
    {
        if (plan.State == OrbitToolsPanel.CommitState.NodeCreated)
            return OrbitToolsPanel.OurBurn;
        return plan.State == OrbitToolsPanel.CommitState.MultiPassRunning && MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec)
            ? exec.CurrentBurn
            : null;
    }

    private static void DrawPicture(in PageContext ctx, string key, Vehicle source, in OrbitToolsPanel.Plan plan, bool otherSource, CraftSituation where)
    {
        bool plane = OrbitToolCatalog.IsPlaneTool(key);
        bool hasBurn = plan.Maneuver != null || plan.Made;
        Orbit? basis = plan.Basis.Orbit;
        string noSketch = NoSketch(in ctx, source, in plan, otherSource);
        bool shows = Picture.Shows("AfcOrbitPic", basis != null && where != CraftSituation.Ground && noSketch.Length == 0);
        ConsoleUi.NoteShape("AfcOrbitPicHead", hasBurn ? 1 : 0);
        if (shows)
        {
            if (hasBurn)
                DrawPictureHead(plane, in plan);
            using (PictureScope pic = Picture.Open(ctx.PictureHeight))
            {
                string head = otherSource ? "For " + source.Id : "";
                if (plane)
                    DrawPlaneSketch(in pic.Frame, key, source, basis!, in plan, head);
                else
                    DrawOrbitSketch(in pic.Frame, key, source, basis!, in plan, head);
            }
        }
        else if (where != CraftSituation.Ground)
            Picture.EmptyState(noSketch.Length > 0 ? noSketch : PageText.NoOrbit);

        if (Picture.LegendLine("", default, default, "AfcOrbitLegend", default, "PLANNER", TransferPage.PlannerTipFor(where), linkStock: true) == 0)
            PlannerRoute.OpenStockPlanner();

        if (!shows)
        {
            _fuelFact = "";
            _budgetMs = double.NaN;
            return;
        }
        CollectBars(source, in plan, key);
        bool chained = plan.Basis.IsChained && !plan.Made;
        CraftActivity a = ctx.Frame.Activity;
        bool burnFits = otherSource || (SituationVerbs.ForManeuver(where, a.AscentRunning, a.AnyGuidanceLive).Fits
            && !(plan.Maneuver is OrbitManeuvers.ManeuverResult m && BurnAfterImpact(plan.Basis.Plan ?? source.FlightPlan, m.BurnTime)));
        _fuelFact = BurnBars.Draw(source, _bars, null, burnFits);
        _budgetMs = BurnBars.LastOnBoardMs;
    }

    private static string NoSketch(in PageContext ctx, Vehicle source, in OrbitToolsPanel.Plan plan, bool otherSource)
    {
        if (otherSource || ctx.Flying)
            return "";
        if (ctx.Frame.Activity.AscentRunning)
            return PageText.NoSketchClimb;
        return plan.Maneuver is OrbitManeuvers.ManeuverResult m && BurnAfterImpact(plan.Basis.Plan ?? source.FlightPlan, m.BurnTime) ? PageText.NoSketchFall : "";
    }

    private static void DrawPictureHead(bool plane, in OrbitToolsPanel.Plan plan)
    {
        bool lines = Patch_CelestialSystem_OnPreRender.Applied;
        // OrbitToolsPanel.RenderOrbitPreview draws nothing while the burn exists, because stock draws it.
        string worldOff = !lines ? Patch_CelestialSystem_OnPreRender.UnavailableTip
            : plan.State == OrbitToolsPanel.CommitState.NodeCreated ? PageText.StockDrawsBurn
            : "";
        int noView = 0;
        if (Picture.Header(plane ? "PLANE SKETCH" : "ORBIT SKETCH", default, ref noView, "AfcOrbitPic",
                new[] { PageText.ShowInWorld }, new[] { lines && OrbitToolsPanel.ShowOrbitPreview },
                new[] { "Draws the orbit after the burn and every pass orbit in the world and on the map." },
                new[] { worldOff }) == 0)
            OrbitToolsPanel.ShowOrbitPreview = !OrbitToolsPanel.ShowOrbitPreview;
    }

    private static void DrawOrbitSketch(in PictureFrame f, string key, Vehicle source, Orbit basis, in OrbitToolsPanel.Plan plan, string head)
    {
        // A plan that starts past a change of body has another frame, so there the sketch stays on the planning orbit.
        Orbit now = ReferenceEquals(source.Orbit.Parent, basis.Parent) ? source.Orbit : basis;
        Burn? made = MadeBurn(source, in plan);
        Orbit? madeAfter = made != null ? OrbitToolsPanel.OrbitAfter(made) : null;
        bool chained = plan.Basis.IsChained && !plan.Made && !ReferenceEquals(now, basis);

        _tracks.Clear();
        int entries = 0;
        _tracks.Add(new OrbitSketch.Track(now, Palette.Flown.A(0.7f), 1.5f, false));
        _legend[entries++] = new Picture.Swatch("Now", Palette.Flown);
        if (chained)
        {
            _tracks.Add(new OrbitSketch.Track(basis, Palette.Plan.A(0.55f), 1f, true));
            _legend[entries++] = new Picture.Swatch("After last burn", Palette.Plan, SwatchKind.Dashed);
        }
        Orbit? finalPass = null;
        if (MultiPassUI.HasMultiPassPreview)
        {
            PassPreview[] passes = MultiPassPreviewCache.PreviewPasses;
            for (int i = 0; i < passes.Length; i++)
            {
                FlightPlan? fp = passes[i].FlightPlan;
                if (fp != null && fp.Patches.Count > 0)
                {
                    bool last = i == passes.Length - 1;
                    _tracks.Add(new OrbitSketch.Track(fp.Patches[0].Orbit, last ? Palette.Plan.Solid : Palette.Plan.A(0.55f), last ? 2f : 1f, !last));
                    if (last)
                        finalPass = fp.Patches[0].Orbit;
                }
            }
            if (passes.Length > 1)
                _legend[entries++] = new Picture.Swatch("Between passes", Palette.Plan, SwatchKind.Dashed);
            _legend[entries++] = new Picture.Swatch("After the passes", Palette.Plan);
        }
        else if (madeAfter != null)
        {
            _tracks.Add(new OrbitSketch.Track(madeAfter, Palette.Plan.Solid, 2f, false));
            _legend[entries++] = new Picture.Swatch("After the burn", Palette.Plan);
        }
        else if (plan.Entry?.FlightPlan is FlightPlan fp && fp.Patches.Count > 0)
        {
            _tracks.Add(new OrbitSketch.Track(fp.Patches[0].Orbit, Palette.Plan.Solid, 2f, false));
            _legend[entries++] = new Picture.Swatch("After the burn", Palette.Plan);
        }

        double3? burn = null;
        Orbit? result = null;
        if (made != null)
        {
            burn = made.Patch?.Orbit.GetStateVectorsAt(made.Time).PositionCci;
            result = plan.State == OrbitToolsPanel.CommitState.NodeCreated ? madeAfter
                : MultiPassUI.LastPassFlightPlan is FlightPlan last && last.Patches.Count > 0 ? last.Patches[0].Orbit
                : madeAfter;
        }
        else if (plan.Maneuver is OrbitManeuvers.ManeuverResult m)
        {
            burn = basis.GetStateVectorsAt(m.BurnTime).PositionCci;
            if (plan.Entry?.FlightPlan is FlightPlan planned && planned.Patches.Count > 0)
                result = planned.Patches[0].Orbit;
        }
        result = finalPass ?? result;

        bool resultShort = finalPass != null && MultiPassUI.FuelShortFact(key, plan.Impulsive?.DvCci.Length() ?? 0.0).Length > 0;
        OrbitSketch.Draw(in f, now, chained ? basis : now, _tracks, burn, "", result, head, _legend.AsSpan(0, entries), resultShort: resultShort);
    }

    /// <summary>The orbit sketch of one burn, for the page of a burn that stock AUTO flies, which has no form to read the plan from.</summary>
    public static void DrawBurnSketch(in PageContext ctx, Vehicle craft, Burn burn)
    {
        Orbit now = craft.Orbit;
        Orbit? after = OrbitToolsPanel.OrbitAfter(burn);
        _tracks.Clear();
        int entries = 0;
        _tracks.Add(new OrbitSketch.Track(now, Palette.Flown.A(0.7f), 1.5f, false));
        _legend[entries++] = new Picture.Swatch("Now", Palette.Flown);
        if (after != null)
        {
            _tracks.Add(new OrbitSketch.Track(after, Palette.Plan.Solid, 2f, false));
            _legend[entries++] = new Picture.Swatch("After the burn", Palette.Plan);
        }
        double3? burnAt = burn.Patch?.Orbit.GetStateVectorsAt(burn.Time).PositionCci;
        using (PictureScope pic = Picture.Open(ctx.PictureHeight))
            OrbitSketch.Draw(in pic.Frame, now, now, _tracks, burnAt, "", after, "", _legend.AsSpan(0, entries));
    }

    private static void DrawPlaneSketch(in PictureFrame f, string key, Vehicle source, Orbit basis, in OrbitToolsPanel.Plan plan, string head)
    {
        double change;
        string other;
        string nowLabel = "Now";
        string changeText = "";
        if (MadeBurn(source, in plan) is Burn made)
        {
            change = OrbitToolsPanel.OrbitAfter(made) is Orbit after ? source.Orbit.GetRelativeInclination(after).Value() * 180.0 / Math.PI : 0.0;
            other = "After the burn";
            changeText = PageText.PlaneChange(change);
        }
        else if (key == ManeuverTools.ManeuverTools.KeyMatchInclination)
        {
            Orbit? target = OrbitToolForm.GetSelectedTargetOrbit();
            change = target != null ? basis.GetRelativeInclination(target).Value() * 180.0 / Math.PI : 0.0;
            other = OrbitToolForm.GetSelectedTargetOrbiter()?.Id ?? "Target plane";
        }
        else
        {
            double targetDeg = OrbitToolForm.TargetInclinationRad * 180.0 / Math.PI;
            change = targetDeg - OrbitToolForm.CurrentInclinationDeg;
            other = "Goal " + Fmt.Deg(targetDeg);
            nowLabel = "Now " + Fmt.Deg(OrbitToolForm.CurrentInclinationDeg);
            changeText = PageText.PlaneChange(change);
        }
        OrbitSketch.DrawPlane(in f, change, OrbitToolForm.UseDescendingNode, nowLabel, other, "", changeText, head);
    }

    // One bar per pass: the passes still to fly of a running multi-pass, the passes of the preview, or the one burn.
    private static void CollectBars(Vehicle source, in OrbitToolsPanel.Plan plan, string key)
    {
        _bars.Clear();
        int first = 1;
        if (MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec))
            first = exec.PassIndex + 1;
        if (MultiPassUI.HasMultiPassPreview)
            PageKit.AddPassBars(_bars, MultiPassPreviewCache.PreviewPasses, first, exec != null, MultiPassUI.FuelShortFact(key, plan.Impulsive?.DvCci.Length() ?? 0.0).Length > 0);
        else if (plan.Maneuver is OrbitManeuvers.ManeuverResult m)
            _bars.Add(new BurnBar("BURN", m.DvCci.Length(), double.NaN, false));
        else if (MadeBurn(source, in plan) is Burn made)
            _bars.Add(new BurnBar("BURN", made.DeltaVVlf.Length(), double.NaN, BurnModeInput.BurnFlies(source)));
    }

    // The footer refusal, the same predicate the form shows as its warning.
    private static string Refusal(in OrbitToolsPanel.Plan plan)
    {
        Orbit? orbit = plan.Basis.Orbit;
        if (orbit == null || orbit.Eccentricity >= 1.0)
            return "Needs a bound orbit";
        if (plan.Maneuver == null)
            return OrbitToolForm.Problem.Length > 0 ? OrbitToolForm.Problem : "No maneuver";
        return "";
    }

    private static void DrawSourceRow(Vehicle source, Vehicle subject)
        => PageKit.CraftRow(source, subject, _sourceBuffer, "AfcOrbitPlanFor", "AfcOrbitSource", static picked => OrbitToolsPanel.Pick(OrbitToolsPanel.ToolKey, picked));

    /// <summary>Makes the circularize side follow a tool opened from outside the page.</summary>
    public static void SyncFromKey(string key) => OrbitToolCatalog.CircularizeSide(key, ref _at);

    public static void Reset()
    {
        _at = CircularizeAt.Apoapsis;
        _fuelFact = "";
        _budgetMs = double.NaN;
    }
}
