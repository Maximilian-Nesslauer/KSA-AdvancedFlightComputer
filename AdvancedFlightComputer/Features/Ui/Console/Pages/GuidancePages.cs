using System;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>LAUNCH: the orbit and its plane, the stock target, the launch window, and the ASCENT choice from the mode registry with the CLIMB choice; then the flight profile picture with its views, the stage line and the stage bar; then the LIMITS, BOOSTERS and HOW IT FLIES folds.</summary>
internal static class LaunchPage
{
    public static CommitSpec Draw(in PageContext ctx)
    {
        if (!PageKit.GuidanceReady(in ctx, out CommitSpec refusal))
            return refusal;
        if (ctx.Flying)
        {
            GuidanceWindow.SeedLaunchLan(ctx.Subject);
            GuidanceWindow.DrawLaunchPicture(ctx.Subject, ctx.PictureHeight);
            if (GuidanceWindow.DrawLaunchPlanRow() && ctx.ToPlanForm())
            {
                ctx.PlanHead();
                GuidanceWindow.DrawLaunchPlanLine();
                GuidanceWindow.DrawLaunchInputs(ctx.Subject, ctx.Width);
                GuidanceWindow.DrawLaunchPage(ctx.Subject, ctx.Width, ctx.FoldsOpen);
            }
        }
        else
        {
            bool after = ctx.Frame.Situation.Where is not (CraftSituation.Ground or CraftSituation.Unknown);
            string flown = after ? GuidanceWindow.FlownLine(in ctx.Frame.Situation) : "";
            Core.ConsoleUi.NoteShape("AfcLaunchFlown", flown.Length > 0 ? 1 : 0);
            if (flown.Length > 0)
                Core.ConsoleUi.Sentence(flown, in ConsoleStyle.TextPrimary, GuidanceWindow.FlownGoodWord());
            ctx.ToInputs();
            using (Core.ConsoleUi.Dim(after))
                GuidanceWindow.DrawLaunchInputs(ctx.Subject, ctx.Width);
            ctx.ToPicture();
            GuidanceWindow.DrawLaunchPicture(ctx.Subject, ctx.PictureHeight);
            bool planDraws = GuidanceWindow.LaunchPictureHasData;
            if (planDraws)
                ctx.ToInputs();
            using (Core.ConsoleUi.Dim(after))
            {
                GuidanceWindow.DrawLaunchLimits(ctx.Subject, ctx.FoldsOpen);
                GuidanceWindow.DrawLaunchBoosters(ctx.Subject, ctx.FoldsOpen);
            }
            if (!planDraws && AfcConsole.HasPanes)
            {
                ctx.ToInputs();
                Core.ConsoleUi.Gap();
            }
            using (Core.ConsoleUi.Dim(after))
                GuidanceWindow.DrawLaunchHow(ctx.Subject, ctx.Width, ctx.FoldsOpen);
        }
        GuidanceModeId? mode = ConsolePages.ModeIdFor(ConsolePage.Launch, AfcConsole.LandWay,
            GuidanceWindow.LandSolverRoute, GuidanceWindow.CurrentAscentChoice());
        string status = GuidanceWindow.LaunchStatusText(ctx.Subject, out bool pending);
        PageText.OutcomeFacts facts = GuidanceWindow.CommitOutcomeFacts(ctx.Subject);
        CommitSpec commit = PageKit.GuidanceCommit(in ctx, mode, status, facts.MayFly ? PageText.ClimbFliesCard : "", readyPending: pending);
        if (GuidanceWindow.LaunchWaits(out bool atWindow))
        {
            Vehicle subject = ctx.Subject;
            return new CommitSpec(PageText.LaunchWaitVerb(atWindow), FlownBy.Afc, Tooltip: PageText.LaunchWaitTip(atWindow), Owner: Owner.Settings, HoldsFooter: true,
                Secondary: new CommitAction(PageText.CancelVerb, "", PageText.CancelWaitTip, () => GuidanceWindow.CancelLaunchPlan(subject), Danger: false));
        }
        bool arms = CommitResolver.LaunchVerbArms(commit.Verb);
        if (pending && commit.Verb.Length > 0 && !commit.NeedsTakeOver)
            return commit with { Verb = PageText.WhenReadyVerb(arms), Quiet = true, Status = "", StatusPending = false, Outcome = PageText.NoLaunchFollows, Tooltip = PageText.CalculatingVerbTip(arms), Owner = Owner.Settings };
        if (commit.Verb == CommitResolver.BackupLaunchVerb)
            return commit with { Status = PageText.BackupPlanState(commit.Status) };
        string answer = commit.Status == PageText.PlanOld ? (arms ? PageText.PlanOldArms : PageText.PlanOld)
            : commit.Status == PageText.NoPlanYet ? PageText.NoPlanAnswer(arms)
            : "";
        return answer.Length > 0 ? commit with { Status = answer, Outcome = facts.AutoStageArms ? PageText.AutoStageArmsNote : "" } : commit;
    }
}

/// <summary>
/// LAND: the four ways down, from high energy to the ground, each one phase lower than the one before
/// it. DEORBIT flies the whole chain from orbit, BOOSTBACK turns a spent booster back to its site,
/// LAND NOW flies the powered descent from where the craft is (entry guidance, touchdown guidance or
/// G-FOLD) and HOVER sinks to the ground.
/// </summary>
internal static class LandPage
{
    private static readonly string[] WayCards = Array.ConvertAll(LandWays.DisplayOrder, PageText.WayCard);

    private static readonly string[] WayCardsNow = new string[LandWays.DisplayOrder.Length];

    public static CommitSpec Draw(in PageContext ctx)
    {
        if (!PageKit.GuidanceReady(in ctx, out CommitSpec refusal))
            return refusal;

        LandWay way = AfcConsole.LandWay;
        if (ctx.Flying)
        {
            LandRead live = GuidanceWindow.ReadLandForm(ctx.Subject, way);
            GuidanceWindow.DrawLandPicture(ctx.Subject, ctx.PictureHeight, way);
            GuidanceWindow.DrawLandLive(ctx.Subject, ctx.Width, way);
            if (GuidanceWindow.DrawLandPlanRow(way, ctx.Frame.Activity) && ctx.ToPlanForm())
            {
                ctx.PlanHead();
                GuidanceWindow.DrawLandPlanLine(way);
                DrawWayRow(in ctx);
                GuidanceWindow.DrawLandInputRows(ctx.Subject, way, in ctx.Frame.Situation, in live, pickSite: false);
                GuidanceWindow.DrawLandPage(ctx.Subject, ctx.Width, way, ctx.FoldsOpen, in live, flying: true);
            }
        }
        else
        {
            DrawWayRow(in ctx);
            way = AfcConsole.LandWay;
            LandRead read = GuidanceWindow.ReadLandForm(ctx.Subject, way);
            ctx.ToInputs();
            GuidanceWindow.DrawLandSiteRows(ctx.Subject, way, in ctx.Frame.Situation, pickSite: true);
            ctx.ToPicture();
            GuidanceWindow.DrawLandMethodRows(ctx.Subject, way, in ctx.Frame.Situation, in read);
            GuidanceWindow.DrawLandPicture(ctx.Subject, ctx.PictureHeight, way, sentence: !AfcConsole.HasPanes || !LandWays.HasSolver(way));
            ctx.ToInputs();
            GuidanceWindow.DrawLandPage(ctx.Subject, ctx.Width, way, ctx.FoldsOpen, in read);
        }

        GuidanceModeId? mode = ConsolePages.ModeIdFor(ConsolePage.Land, way,
            GuidanceWindow.LandSolverRoute, GuidanceWindow.CurrentAscentChoice());
        CommitSpec commit = Commit(in ctx, mode);
        GuidanceWindow.NoteFooterPicksSite(commit.NextLeads && commit.Secondary!.Verb is PageText.PickSite or PageText.PickArmed);
        return GuidanceWindow.LandWayCannotFly(ctx.Subject, way, in ctx.Frame.Situation) ? commit with { MarkDim = true } : commit;
    }

    private static CommitSpec Commit(in PageContext ctx, GuidanceModeId? mode)
    {
        if (mode == GuidanceModeId.Boostback)
        {
            CommitSpec boostback = PageKit.GuidanceCommit(in ctx, mode, "", GuidanceWindow.BoostbackCommitHintText());
            return ctx.Frame.Situation.Where == CraftSituation.Ground && GuidanceWindow.NoStageHome
                ? boostback with { Tooltip = PageText.BoostbackTakesCraft + "\n\n" + boostback.Tooltip }
                : boostback;
        }
        if (mode == GuidanceModeId.DeorbitLand)
            return PageKit.GuidanceCommit(in ctx, mode, GuidanceWindow.PressTimingText(), "");
        if (mode != GuidanceModeId.EntryHere)
            return PageKit.GuidanceCommit(in ctx, mode, "", "");
        return GuidanceWindow.EntryBodyAirless(ctx.Subject)
            ? PageKit.GuidanceCommit(in ctx, mode, "", EntryGuidance.NoAirText + "\n\n" + EntryGuidance.CommitHint)
            : PageKit.GuidanceCommit(in ctx, mode, "", EntryGuidance.CommitHint);
    }

    private static void DrawWayRow(in PageContext ctx)
    {
        CraftSituation where = ctx.Frame.Situation.Where;
        bool air = ctx.Frame.Situation.HasAir, below = GuidanceWindow.BelowBrakingStart(ctx.Subject);
        LandWay? fits = LandWays.FitsNow(where, air, below);
        int marked = fits is LandWay fit && !ctx.Frame.Activity.AscentRunning ? LandWays.DisplayIndex(fit) : -1;
        for (int i = 0; i < WayCards.Length; i++)
        {
            string why = SituationVerbs.WayNow(LandWays.DisplayOrder[i], where, air, below, GuidanceWindow.NoStageHome);
            WayCardsNow[i] = why.Length > 0 ? WayCards[i] + "\n\n" + why
                : i == marked ? WayCards[i] + "\n\n" + PageText.FitsNowNote
                : WayCards[i];
        }

        int picked = Core.ConsoleUi.PlaceTabs("", "AfcLandWay", LandWays.Labels, LandWays.DisplayIndex(AfcConsole.LandWay), WayCardsNow, marked: marked);
        if (picked >= 0 && LandWays.DisplayOrder[picked] != AfcConsole.LandWay)
        {
            AfcConsole.LandWay = LandWays.DisplayOrder[picked];
            AfcConsole.SavePreferences();
        }
    }
}
