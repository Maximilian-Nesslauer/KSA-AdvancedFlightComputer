#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The form of the LAND page, in the order a pilot decides it. Above the picture: the site, the table of solvers with the obstacle of each row on that row and the chain of the way (DrawLandSiteRows, DrawLandMethodRows). Under the picture: the note about stowed legs near the ground, the PARAMETERS fold with one short value, what the way shows while it flies, the HAND-OVERS, the LEGS and the GNC fold (DrawLandPage). While the way flies, the picture and what it shows lead (DrawLandLive), and the PLAN row opens the same form (DrawLandPlanRow). The WAY row is in LandPage, the commit is the console footer and the stop is the console head.
public static partial class GuidanceWindow
{
    private const string LandNumbersFoldId = "LandNumbers";
    private const string HandOversFoldId = "LandHandOvers";

    private static bool _landHoverFlies = true;
    private static bool _landHoverKnown;

    internal static bool LandHoverRefused => _landHoverKnown && !_landHoverFlies;

    private static bool HoverThrustKnown(Vehicle vehicle) => vehicle?.Orbit?.Parent != null && KsaEnginePerf.SupportsThrottleControl(vehicle);

    private static bool HoverFliesByThrust(Vehicle vehicle)
        => HoverThrustKnown(vehicle) && LandSolvers.HoverFlies(TerminalMinThrottleTwr(vehicle, vehicle.Orbit, vehicle.Orbit.Parent.Mu));

    private static void ReadHoverAnswer(Vehicle vehicle, in DescentFacts descent)
    {
        if (EntryLive(_s) && Ui.Console.ConsoleFrame.EngineKnownOff(vehicle))
        {
            _landHoverKnown = false;
            _landHoverFlies = false;
            return;
        }
        _landHoverKnown = descent.Known || HoverThrustKnown(vehicle);
        _landHoverFlies = descent.Known ? LandSolvers.HoverFlies(in descent) : HoverFliesByThrust(vehicle);
    }

    private static readonly List<TableMark> _landTableMarks = new();

    // A site row has the room for a degree value to three decimals, the precision of Fmt.LatLon. The narrowest field at interface scale 1 that shows a value such as "-80.600" whole, so a long body name yields first.
    private const string SiteCellFormat = "%.3f";
    private const float SiteFieldLeast = 78f;

    private static bool LandFlies => _s.Active || _s.EngagePending || _s.SixDofStandby || EntryLive(_s) || DescentLive || BoostbackLive;

    private static LandSolver LandSolverOf(LandWay way)
        => way == LandWay.FromOrbit ? LandSolvers.Shown(LandSolvers.Chosen(false, _s.UseSixDofLanding), LandingMachineLive) : LandSolverRoute;

    // LandSolvers.EntryApplies for this craft, with the time of its fall from the sink rate, the height over the ground below and the local gravity.
    private static bool EntryFliesFirst(Vehicle vehicle)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit.Parent;
        SituationFacts facts = Ui.Console.ConsoleFrame.ReadFacts(vehicle);
        if (!facts.HasAir)
            return false;
        double3 position = orbit.StateVectors.PositionCci;
        double gravity = parent.Mu / position.LengthSquared();
        double sink = -double3.Dot(orbit.StateVectors.VelocityCci, double3.Normalize(position));
        double height = Math.Max(TerminalHeight(orbit, parent, parent.MeanRadius), 0.0);
        double fall = (-sink + Math.Sqrt(Math.Max(sink * sink + 2.0 * gravity * height, 0.0))) / gravity;
        return LandSolvers.EntryApplies(facts.HasAir, facts.Where, facts.Thrust, fall);
    }

    /// <summary>What the form reads for its rows and the facts of the footer. It runs on every frame, also while the rows are folded away.</summary>
    internal static LandRead ReadLandForm(Vehicle vehicle, LandWay way)
    {
        if (vehicle.Orbit?.Parent == null)
            return default;
        ClearLandFact();
        // Entry guidance is not a pick of the player. While no landing mode flies, the page sets it from the situation of the craft, and a flying mode sets it through FollowLiveLanding.
        if (!LandFlies)
            _s.UseThreeDofLanding = EntryFliesFirst(vehicle);
        LandSolver solver = LandSolverOf(way);
        DescentFacts descent = LandWays.HasSolver(way) ? ReadDescentFacts(vehicle) : default;
        DescentFacts judged = _s.Active || _s.SixDofStandby ? default : descent;
        ReadHoverAnswer(vehicle, in judged);
        NoteLandFacts(way, solver, in judged, HasPhysicalAtmosphere(vehicle.Orbit.Parent));
        return new LandRead(descent, LandFlies);
    }

    internal static void DrawLandInputRows(Vehicle vehicle, LandWay way, in SituationFacts situation, in LandRead read, bool pickSite)
    {
        DrawLandSiteRows(vehicle, way, in situation, pickSite);
        DrawLandMethodRows(vehicle, way, in situation, in read);
    }

    internal static void DrawLandSiteRows(Vehicle vehicle, LandWay way, in SituationFacts situation, bool pickSite)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null || way == LandWay.Hover)
            return;
        using (ConsoleUi.Dim(DeorbitTargetLocked || (way == LandWay.Boostback && situation.Where == CraftSituation.Ground && NoStageHome)))
            DrawLandSiteRow(parent, pickSite);
        ConsoleUi.Gap();
    }

    internal static void DrawLandMethodRows(Vehicle vehicle, LandWay way, in SituationFacts situation, in LandRead read)
    {
        if (vehicle.Orbit?.Parent == null)
            return;
        LandSolver solver = LandSolverOf(way);
        bool table = LandWays.HasSolver(way);
        if (table)
            DrawLandSolverTable(vehicle, way, solver, in situation, read.Descent);
        ChainStep[] chain = LandChain.For(way, solver, situation.Where, situation.HasAir, _s.ThreeDofFromGlide, _s.ThreeDofSixDofHandover, BelowBrakingStart(vehicle), _landHoverFlies);
        bool shows = LandChain.Shows(chain);
        if (shows)
            ConsoleUi.ChainStrip(chain, WayCannotFly(vehicle, way, solver, in situation));
        else if (table)
            ConsoleUi.ChainRowRoom();
        if (shows || table)
            ConsoleUi.Gap();
    }

    private static bool WayCannotFly(Vehicle vehicle, LandWay way, LandSolver solver, in SituationFacts situation)
    {
        CraftActivity activity = Ui.Console.AfcConsole.Frame.Activity;
        return LandSolvers.ChainCannotFly(way, solver, situation.HasAir, situation.Where, NoStageHome, BelowBrakingStart(vehicle), FlightView.FlyingWay(in activity) == way);
    }

    internal static bool LandWayCannotFly(Vehicle vehicle, LandWay way, in SituationFacts situation) => WayCannotFly(vehicle, way, LandSolverOf(way), in situation);

    internal static void DrawLandLive(Vehicle vehicle, float innerW, LandWay way)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        DrawLegsNote(vehicle, way);
        DrawLandWay(vehicle, orbit, parent, parent.MeanRadius, innerW, way);
    }

    internal static bool DrawLandPlanRow(LandWay way, in CraftActivity activity)
    {
        string reads = FlightView.LandReads(way, DeorbitTargetLocked, _s.SiteLatDeg, _s.SiteLonDeg);
        bool pickOnRow = FlightView.ReadsNewSite(way, in activity, DeorbitTargetLocked);
        float buttonW = pickOnRow ? ConsoleWidgets.ButtonWidth(PageText.PickSite.AsSpan()) : 0f;
        ConsoleUi.Gap();
        bool open = ConsoleUi.Fold(FlightView.PlanLabel(reads), FlightView.PlanFoldId, false, PageText.PlanTip, reserve: pickOnRow ? buttonW + CellGap : 0f);
        if (pickOnRow)
        {
            ImGui.SameLine(0f, CellGap);
            DrawPickSiteButton(PageText.PickSite, PageText.PickSite, "AfcPickSiteRow", new float2(buttonW, ConsoleStyle.FrameHeightPx));
        }
        return open;
    }

    internal static void DrawLandPlanLine(LandWay way) => ConsoleUi.MutedWrapped(FlightView.LandPlanLine(way, way == LandWay.FromHere ? LandDrawnSolver : LandSolverOf(way), DeorbitTargetLocked, _landHoverFlies));

    // The footer of the last frame holds PICK SITE with the gold fill, so the eye finds one press and this one stands dim.
    private static bool _footerPicksSite;

    internal static void NoteFooterPicksSite(bool picks) => _footerPicksSite = picks;

    private static void DrawPickSiteButton(string label, string armedLabel, string id, float2 size)
    {
        bool armed = PickSiteArmed;
        bool locked = DeorbitTargetLocked && !armed;
        bool clicked;
        using (ConsoleUi.Dim(_footerPicksSite && !armed))
        using (new ImGuiDisabledScope(locked))
            clicked = ConsoleWidgets.Button((armed ? armedLabel : label).AsSpan(), id.AsSpan(), size);
        if (clicked && !locked)
            TogglePickSite();
        ConsoleUi.NoteItem(id);
        if (armed)
            ImGui.GetWindowDrawList().AddRect(ImGui.GetItemRectMin(), ImGui.GetItemRectMax(), ConsoleStyle.ApplyAlpha(Ui.Canvas.Palette.Afc.U32),
                ConsoleStyle.FrameRoundingPx, ImDrawFlags.None, 1.5f * ImGuiHelper.InterfaceScale);
        if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ConsoleUi.Tooltip((locked ? PageText.SiteLockedByDeorbit : armed ? PageText.PickArmedTip : PageText.PickSiteTip).AsSpan());
    }

    private static void NoteLandFacts(LandWay way, LandSolver solver, in DescentFacts f, bool hasAir)
    {
        if (way == LandWay.FromOrbit)
        {
            if (double.IsFinite(ClosestPassFarKm()))
                NoteLandFact(PageText.PassesFar, danger: false, next: ConsolePage.Orbit, calm: true);
            return;
        }
        if (way != LandWay.FromHere || !f.Known)
            return;
        if (solver == LandSolver.Gfold)
        {
            LandVerdict verdict = LandVerdict.ForDescent(solver, in f, checkStop: true);
            if (verdict.Severity >= Severity.Warning)
                NoteLandFact(verdict.Text, verdict.Severity == Severity.Error);
            return;
        }
        if (solver != LandSolver.Touchdown)
            return;
        if (f.CannotStopNow)
            NoteLandFact(FormattableString.Invariant($"NO SOLUTION: TWR {f.TwrNeeded:F2} NEEDED, {f.TwrMax:F2} AVAILABLE."), danger: true);
        else if (f.MarginalNow)
            NoteLandFact(FormattableString.Invariant($"MARGINAL: STOP USES {f.StopM / f.AltToGoM * 100.0:F0} % OF ALTITUDE LEFT."), danger: false);
        if (f.NoGimbal)
            NoteLandFact(PageText.NoGimbalFact, danger: true, calm: true);
        else if (f.OverPowered)
            NoteLandFact(PageText.OverPoweredFact(f.FeasibleFloor), danger: true, next: ConsolePage.Tune, calm: true);
    }

    /// <summary>
    /// The table of the powered-descent solver, on DEORBIT and LAND NOW. The method word of a row has the colour that the same word has on the chain under it. It shows TOUCHDOWN, with ENTRY above it on LAND NOW where entry guidance flies first (LandSolvers.EntryApplies), and G-FOLD while its fallback descent flies. The player picks none of them. What stands against a row stands on that row (LandSolvers.NoteOf) in the warning colour, full on the chosen row and dim on any other (LandSolvers.MarksOf). A held row says why on its hover card, and one sentence stands under the table (LandSolvers.HeldLine). A row that is held or cannot fly on this body is dim unless it is the chosen row.
    /// </summary>
    private static void DrawLandSolverTable(Vehicle vehicle, LandWay way, LandSolver chosen, in SituationFacts situation, in DescentFacts descent)
    {
        bool hasAir = situation.HasAir;
        string body = situation.Body;
        bool entryFirst = chosen == LandSolver.Entry;
        string[] heldBySolver = LandSolvers.Held(way, DeorbitTargetLocked, LandingMachineLive, hasAir, entryFirst);
        LandSolver[] solvers = LandSolvers.RowsOf(way, LandingMachineLive, entryFirst);
        var facts = new LandNowFacts(way, chosen, hasAir, DeorbitTargetLocked, LandingMachineLive, descent, body);
        bool hoverFlies = _landHoverFlies;
        var rows = new string[solvers.Length][];
        var cards = new string[solvers.Length];
        var held = new string[solvers.Length];
        var dim = new bool[solvers.Length];
        _landTableMarks.Clear();
        for (int i = 0; i < solvers.Length; i++)
        {
            LandSolver solver = solvers[i];
            RowNote note = LandSolvers.NoteOf(solver, in facts);
            rows[i] = LandSolvers.TableRow(solver, _s.GfoldHoverHandoffAltM, hoverFlies, hasAir, note.Text);
            cards[i] = LandSolvers.Card(solver, _s.GfoldHoverHandoffAltM, _s.ThreeDofSixDofHandover, _s.ThreeDofSixDofTimeS, hoverFlies, hasAir, _landHoverKnown);
            held[i] = heldBySolver[(int)solver];
            dim[i] = LandSolvers.RowIsDim(solver, hasAir, held[i].Length > 0);
            LandSolvers.MarksOf(solver, i, solver == chosen, in note, hasAir, _landTableMarks);
            _landTableMarks.Add(LandSolvers.MethodMark(i, WayCannotFly(vehicle, way, solver, in situation), held[i].Length > 0 && solver != chosen));
        }
        int picked = ConsoleUi.ChoiceTable("AfcLandSolver", LandSolvers.TableHeads, rows, Array.IndexOf(solvers, chosen), held, cards, PageText.SolverTip, dim, CollectionsMarshal.AsSpan(_landTableMarks), radio: true);
        if (picked >= 0)
            (_s.UseThreeDofLanding, _s.UseSixDofLanding) = LandSolvers.Apply(solvers[picked], _s.UseSixDofLanding);

        string line = LandSolvers.HeldLine(in facts);
        ConsoleUi.NoteShape("AfcLandHeld", line.Length);
        if (line.Length > 0)
            ConsoleUi.Sentence(line.AsSpan(), ConsoleStyle.TextLabel);
        string fallback = LandSolvers.FallbackLine(in facts);
        ConsoleUi.NoteShape("AfcLandFallback", fallback.Length);
        if (fallback.Length > 0)
            ConsoleUi.MutedWrapped(fallback);
    }

    internal static void PickLandSolver(Vehicle vehicle, LandSolver solver)
    {
        Use(vehicle);
        // A committed deorbit reads the solver through its braking gate and stops on a change, so a link cannot pick a row that the table holds.
        if (DeorbitTargetLocked && LandSolvers.LockedByDeorbit(solver))
            return;
        (_s.UseThreeDofLanding, _s.UseSixDofLanding) = LandSolvers.Apply(solver, _s.UseSixDofLanding);
        ConsoleUi.ShapeChanged = true;
    }

    private static bool _siteTyping;

    /// <summary>
    /// Where to land, on one row: the small square of the site mark of the pictures, the word SITE and the body, the latitude and longitude, and PICK SITE, which arms a click on the map or in the world that moves the site. The fields keep the ids, the clamp and the lock of a whole row. A long body name goes first, because the head names the body. A pane with no room for the word gives the fields a row of their own, as the form with no site does. With no site picked on this body the row says so with PICK SITE as what to do, and the fields stand dim on a row of their own, because they hold what a press would fly to.
    /// </summary>
    private static void DrawLandSiteRow(IParentBody parent, bool pickSite)
    {
        float width = ConsoleStyle.ContentAvailWidth();
        float h = ConsoleStyle.FrameHeightPx;
        float gapToLabel = ConsoleWidgets.MinControlGapPx + ConsoleWidgets.RowLabelInsetPx;
        float buttonW = pickSite ? Math.Max(ConsoleWidgets.ButtonWidth(PageText.PickSite.AsSpan()), ConsoleWidgets.ButtonWidth(PageText.PickArmed.AsSpan())) : 0f;
        float button = pickSite ? buttonW + CellGap : 0f;
        float leastCells = CellsWithoutFields([("LAT", ""), ("LON", "")]) + 2f * SiteFieldLeast * ImGuiHelper.InterfaceScale;

        void PickButton() => DrawPickSiteButton(PageText.PickSite, PageText.PickArmed, "AfcPickSite", new float2(buttonW, h));

        void PickSiteRow(string label)
        {
            ConsoleUi.BeginRow(label, pickSite ? buttonW : 1f, leadSwatch: ColorRules.Target);
            if (pickSite)
            {
                ConsoleUi.RowRight(buttonW);
                PickButton();
            }
            ConsoleUi.EndRow();
        }

        bool SiteCellsRow(string label, float room, bool none)
        {
            float field = CellFieldWidth(room, out float row, ("LAT", ""), ("LON", ""));
            BeginKitRow(label, controlWidth: row);
            bool rowHovered = ConsoleWidgets.RowHovered;
            float2 at = ImGui.GetCursorScreenPos();
            float x = at.X + ConsoleWidgets.RowControlWidth - row;
            bool typing;
            using (ConsoleUi.Dim(none))
                typing = DrawSiteCells(ref x, at.Y, field);
            if (none && rowHovered)
                ConsoleUi.Tooltip(PageText.SiteDefaultTip.AsSpan());
            ConsoleUi.EndRow();
            return typing;
        }

        bool none = NoSitePicked(parent) || _siteTyping;
        ConsoleUi.NoteShape("AfcLandSite", none ? 1 : 0);
        if (none)
        {
            PickSiteRow(PageText.SiteNonePicked(parent.Id));
            string typeLabel = Ui.Console.AfcConsole.HasPanes ? PageText.SiteTypeRowShort : PageText.SiteTypeRow;
            _siteTyping = SiteCellsRow(typeLabel, width - ConsoleUi.LabelWidth(typeLabel) - gapToLabel, none: true);
            return;
        }
        _siteTyping = false;

        string full = "SITE " + parent.Id;
        float fullW = ConsoleUi.LabelWidth(full);
        float wordW = ConsoleUi.LabelWidth(FoldNames.Site);
        float labelRoom = width - button - leastCells - gapToLabel - ConsoleUi.SwatchRoom;
        if (wordW > labelRoom)
        {
            PickSiteRow(full);
            SiteCellsRow("", width, none: false);
            return;
        }
        string label = fullW <= labelRoom ? full : FoldNames.Site;
        float labelW = (fullW <= labelRoom ? fullW : wordW) + ConsoleUi.SwatchRoom;

        float fieldW = CellFieldWidth(width - labelW - gapToLabel - button, out float rowWidth, ("LAT", ""), ("LON", ""));
        ConsoleUi.BeginRow(label, rowWidth + button, leadSwatch: ColorRules.Target);
        float2 control = ImGui.GetCursorScreenPos();
        float right = control.X + ConsoleWidgets.RowControlWidth;
        float x = right - button - rowWidth;

        DrawSiteCells(ref x, control.Y, fieldW);

        if (pickSite)
        {
            ImGui.SetCursorScreenPos(new float2(right - buttonW, control.Y));
            PickButton();
        }
        ConsoleUi.EndRow();
    }

    private static bool DrawSiteCells(ref float x, float y, float fieldW)
    {
        using (new ImGuiDisabledScope(DeorbitTargetLocked))
        {
            GaugeCell(Specs.SiteLatDeg, "LAT", ref x, y, fieldW, ref _s.SiteLatDeg, SiteCellFormat, showUnit: false, marksChange: false);
            bool active = ImGui.IsItemActive();
            GaugeCell(Specs.SiteLonDeg, "LON", ref x, y, fieldW, ref _s.SiteLonDeg, SiteCellFormat, showUnit: false, marksChange: false);
            return active || ImGui.IsItemActive();
        }
    }

    private static bool NoSitePicked(IParentBody parent) => parent != null && LandSite.NonePicked(_s.SiteLatDeg, _s.SiteLonDeg, parent.Id);

    /// <summary>The rows under the picture. With <paramref name="flying"/> the legs note and the live rows stand above the PLAN row already.</summary>
    internal static void DrawLandPage(Vehicle vehicle, float innerW, LandWay way, bool foldsOpen, in LandRead read, bool flying = false)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        ConsoleUi.NoteShape("AfcLandFlies", read.Flying ? 1 : 0);
        if (!flying)
        {
            DrawLegsNote(vehicle, way);
            if (way == LandWay.Boostback || way == LandWay.FromHere)
                DrawLiftNote(vehicle, way, roomOnly: way == LandWay.FromHere && LandDrawnSolver != LandSolver.Entry);
        }
        else if (way == LandWay.FromOrbit && DescentLive && Ui.Console.ConsoleFrame.ReadFacts(vehicle).Where == CraftSituation.Orbit)
            DrawPassStrip(big: false);
        DrawLandNumbersFold(vehicle, orbit, parent, way, foldsOpen, in read);
        if (!flying)
            DrawLandWay(vehicle, orbit, parent, parent.MeanRadius, innerW, way, liftNote: false);
        ConsoleUi.Gap();
        DrawHandOversFold(way, parent, foldsOpen, !LandHoverRefused);
        DrawLegsFold(vehicle, foldsOpen);
        DrawLandHowFold(vehicle, innerW, way, foldsOpen);
    }

    private static void DrawLandHowFold(Vehicle vehicle, float innerW, LandWay way, bool foldsOpen)
    {
        SettingSpec[] doubles;
        BoolSpec[] bools = null;
        FoldPart[] wayParts;
        switch (way)
        {
            case LandWay.FromOrbit:
                doubles = Specs.LandFromOrbitExpert;
                bools = Specs.LandFromOrbitExpertBools;
                wayParts = [new FoldPart(FoldNames.Approach, true, InExpert(() => DrawApproachSection(innerW)))];
                break;
            case LandWay.Boostback:
                doubles = Specs.LandBoostbackExpert;
                wayParts = [new FoldPart(FoldNames.Burn, true, InExpert(DrawBoostbackBurnSection)), new FoldPart(FoldNames.Glide, true, InExpert(DrawBoostbackGlideSection))];
                break;
            case LandWay.FromHere when LandSolverRoute == LandSolver.Entry:
                doubles = Specs.LandEntryExpert;
                bools = Specs.LandEntryExpertBools;
                wayParts = [new FoldPart(FoldNames.BurnLimit, true, InExpert(DrawBurnLimitRow))];
                break;
            case LandWay.FromHere when LandSolverRoute == LandSolver.Touchdown:
                doubles = Specs.LandTouchdownExpert;
                wayParts = [new FoldPart(FoldNames.BurnLimit, true, InExpert(DrawBurnLimitRow))];
                break;
            case LandWay.FromHere:
                doubles = Specs.LandGfoldExpert;
                wayParts = [];
                break;
            default:
                doubles = Specs.HoverExpert;
                wayParts = [new FoldPart(FoldNames.HoverLimits, true, InExpert(DrawHoverLimitsSection))];
                break;
        }
        string id = "land-" + ConsolePages.EnumKey(way);
        int changed = SettingAccess.CountChanged(_s, doubles, bools);
        Ui.Console.Pages.PageKit.HowItFlies(id,
        [
            ResetPart(id, vehicle, doubles, bools, changed),
            new FoldPart(FoldNames.VehicleHeight, true, VehicleHeightRows),
            .. wayParts,
            new FoldPart("", TelemetryLogShows(way), DrawTelemetryLogSummary),
            new FoldPart(FoldNames.InWorld, WorldSwitchShows(vehicle, way), () => DrawWorldSwitchRow(vehicle, way)),
        ], changed, foldsOpen);
    }

    private static void DrawBurnLimitRow() => GaugeRow(Specs.LandingAccelLimitG, ref _s.LandingAccelLimitG);

    private static string SitePlace(IParentBody parent)
        => NoSitePicked(parent) ? string.Create(Fmt.Inv, $"{_s.SiteLatDeg:F3}, {_s.SiteLonDeg:F3}") : "SITE";

    private static string LandNumbersValue(Vehicle vehicle, IParentBody parent, LandWay way, in LandRead read)
    {
        if (way == LandWay.Boostback)
            return BoostbackVerdict(vehicle).Text.TrimEnd('.');
        return LandVerdict.TwrAtSite(read.Descent, SitePlace(parent));
    }

    private static void DrawLandNumbersFold(Vehicle vehicle, Orbit orbit, IParentBody parent, LandWay way, bool foldsOpen, in LandRead read)
    {
        string value = LandNumbersValue(vehicle, parent, way, in read);
        LandSolver solver = LandSolverOf(way);
        DescentFacts descent = read.Descent;
        bool noSite = NoSitePicked(parent);
        bool siteRows = way != LandWay.Hover && !noSite;
        bool ground = vehicle.Situation.HasAnyContact();
        bool impact = way == LandWay.Boostback && !ground && (_s.HasImpact || _s.Impact.Status == ImpactStatus.NoImpactWithinHorizon);
        bool deorbit = way is LandWay.FromOrbit or LandWay.FromHere && (DescentLive || _s.DeorbitRequest != null);
        bool feasibilityAlone = !siteRows && !impact && !deorbit;
        ConsoleUi.PartsFold(PageText.Numbers, LandNumbersFoldId,
        [
            new FoldPart(FoldNames.Distance, siteRows, () => GaugeRowText("DISTANCE TO SITE", Fmt.Distance(AngleBetween(orbit.StateVectors.PositionCci, SiteDirCciAt(parent, 0)) * parent.MeanRadius))),
            new FoldPart(FoldNames.Terrain, siteRows, () => GaugeRowText("SITE TERRAIN", Fmt.Distance(SiteTerrainHeight(parent)))),
            new FoldPart(FoldNames.Impact, impact, DrawBoostbackImpactDetails),
            new FoldPart(FoldNames.Feasibility, LandWays.HasSolver(way) && (descent.Known || FirstSolveStops(way, solver)), () => DrawDescentRows(way, solver, in descent, noSite, feasibilityAlone)),
            new FoldPart(FoldNames.Deorbit, deorbit, DrawDeorbitFigures),
        ], 0, foldsOpen, PageText.NumbersTip, value.Length > 0 ? PageText.Numbers + ": " + value : "");
    }

    private static bool WorldSwitchShows(Vehicle vehicle, LandWay way)
        => way == LandWay.Boostback ? !BoostbackPictureHasData(vehicle)
            : way == LandWay.FromHere && LandSolverRoute != LandSolver.Entry && !LandPictureHasData(way);

    private static void DrawWorldSwitchRow(Vehicle vehicle, LandWay way)
    {
        bool impact = way == LandWay.Boostback;
        ref bool show = ref impact ? ref _showImpactOverlay : ref LandSolverRoute == LandSolver.Touchdown ? ref _show6DofOverlay : ref _showGfoldOverlay;
        if (GaugeRowCheck(UiTier.Player, PageText.ShowInWorld, "AfcLandInWorldRow", ref show) && impact && show)
            _s.ImpactTick = 0;
    }

    private static bool FirstSolveStops(LandWay way, LandSolver solver)
        => way == LandWay.FromHere && solver == LandSolver.Touchdown && !_s.SixDofThreaded && !_s.SixDofSpreadCold;

    // The figures behind the verdict. They use the SAME pressure-corrected thrust the planner does: a readout of vacuum thrust while the plan uses something else hides the atmospheric shortfall, because "max TWR" would look healthy with a number the craft can only reach in space.
    private static void DrawDescentRows(LandWay way, LandSolver solver, in DescentFacts f, bool noSite, bool alone)
    {
        if (FirstSolveStops(way, solver))
            ConsoleUi.Muted("The first plan stops the simulation for about 1.7 s. Start in a coast, or set SOLVE ON WORKER THREAD or SPREAD FIRST PLAN on TUNE.".AsSpan());
        if (!f.Known)
            return;

        if (!alone)
            ConsoleUi.SectionHead(FoldNames.Feasibility);
        GaugeRowText("THRUST USED", Fmt.Mn(f.ThrustN));
        GaugeRowText("WEIGHT", Fmt.Mn(f.WeightN));
        GaugeRowText("MAX TWR", Fmt.Ratio(f.TwrMax));

        // With air at the target this is the number that decides whether the plan is flyable, and an airless world has nothing to say here.
        if (f.AmbientPa > 1.0)
        {
            double frac = f.VacuumN > 0.0 ? f.ThrustN / f.VacuumN : 1.0;
            GaugeRowText(noSite ? "AIR PRESSURE AT POINT" : "AIR PRESSURE AT SITE", Fmt.Kpa(f.AmbientPa));
            GaugeRowText("SHARE OF VACUUM THRUST", Fmt.Percent(frac) + " of " + Fmt.Mn(f.VacuumN));
            if (frac < 0.97)
                ConsoleUi.Muted(FormattableString.Invariant($"Plans with the thrust at {(noSite ? "that point" : "the site")}. Vacuum thrust is {(1.0 / Math.Max(frac, 1e-6) - 1.0) * 100.0:F0} % too high.").AsSpan());
        }
        bool touchdown = solver == LandSolver.Touchdown;
        double twrLow = touchdown ? f.TwrAtFloor : f.TwrAtCraftMin;
        GaugeRowText("TWR AT MIN THROTTLE", Fmt.Ratio(twrLow), twrLow > 1.0 && solver != LandSolver.Entry ? Ui.Canvas.Palette.WarnDim.F4 : ConsoleStyle.TextPrimary);
        if (touchdown)
            GaugeRowText("THROTTLE FLOOR", Fmt.Percent(f.Floor) + ", CRAFT MIN " + Fmt.Percent(f.CraftFloor));

        bool checkStop = way == LandWay.FromHere && solver != LandSolver.Entry;
        ConsoleUi.NoteShape("AfcLandSinking", f.Sinking ? 1 : 0);
        if (checkStop && f.Sinking)
        {
            GaugeRowText("DESCENT", Fmt.Speed(f.SinkMs) + ", " + Fmt.Distance(f.HeightM) + " TO GO");
            GaugeRowText("STOPS IN", double.IsInfinity(f.SinkStopM) ? "NEVER" : Fmt.Distance(f.SinkStopM));
        }
        foreach (LandVerdict note in LandVerdict.Notes(solver, f, checkStop))
        {
            float4 ink = note.Severity == Severity.Error ? ConsoleStyle.Danger
                : note.Severity == Severity.Warning ? ConsoleUi.WarnInk
                : note.Soft ? Ui.Canvas.Palette.WarnDim.F4
                : ConsoleStyle.TextLabel;
            ConsoleUi.Sentence(note.Text.AsSpan(), in ink);
        }
    }

    private static bool TelemetryLogShows(LandWay way)
    {
        bool flown = way == LandWay.FromHere ? LandSolverRoute != LandSolver.Gfold
            : way == LandWay.FromOrbit && _s.UseSixDofLanding;
        return flown && (_s.SixDofLogging || (SixDofLog.Enabled && ReferenceEquals(SixDofLog.Owner, _s)));
    }

    private static void DrawTelemetryLogSummary()
    {
        bool recordsThis = SixDofLog.Enabled && ReferenceEquals(SixDofLog.Owner, _s);
        string text = recordsThis ? $"Recording, {SixDofLog.RowsWritten} rows" : "ON";
        string now = recordsThis ? "Recording now."
            : SixDofLog.Enabled ? "Another craft holds the log."
            : "It starts when touchdown guidance starts.";
        using (ConsoleUi.Dim(true))
            ConsoleUi.Readout(FoldNames.TelemetryLog.AsSpan(), text.AsSpan());
        if (ImGui.IsItemHovered())
            ConsoleUi.Tooltip((now + " Touchdown guidance writes telemetry to log files. Switch and log state: TUNE > DIAG > TOUCHDOWN.").AsSpan());
    }

    private static void DrawLandWay(Vehicle vehicle, Orbit orbit, IParentBody parent, double bodyRadius,
                                    float innerW, LandWay way, bool liftNote = true)
    {
        switch (way)
        {
            case LandWay.FromOrbit:
                DrawDescentContent(innerW);
                break;
            case LandWay.Boostback:
                DrawBoostbackContent(vehicle, orbit, parent, innerW, liftNote);
                break;
            case LandWay.FromHere:
                switch (LandDrawnSolver)
                {
                    case LandSolver.Entry:
                        Draw3DofLandingContent(vehicle, innerW, liftNote);
                        break;
                    case LandSolver.Touchdown:
                        Draw6DofLandingContent(vehicle, innerW);
                        break;
                    default:
                        DrawGfoldLandingContent(innerW);
                        break;
                }
                break;
            default:
                DrawHoverContent(vehicle, orbit, parent, parent.Mu, innerW);
                break;
        }
    }

    private static void DrawHandOversFold(LandWay way, IParentBody parent, bool openByDefault, bool hoverCanFly)
    {
        LandSolver solver = LandSolverOf(way);
        bool entryRows = way == LandWay.Boostback || (way == LandWay.FromHere && solver == LandSolver.Entry);
        bool shows = entryRows ? HasPhysicalAtmosphere(parent) : LandWays.HasSolver(way) && solver == LandSolver.Gfold;
        ConsoleUi.NoteShape(HandOversFoldId, shows ? 1 : 0);
        if (!shows)
        {
            if (LandWays.HasSolver(way))
                ConsoleUi.FoldEmpty(PageText.NoHandOversLabel, PageText.NoHandOversTip(solver));
            return;
        }
        string label = entryRows
            ? PageText.HandOversLabel(_s.ThreeDofFromGlide, _s.ThreeDofEngageTgoS, _s.ThreeDofSixDofHandover, _s.ThreeDofSixDofTimeS)
            : PageText.HoverHandOverLabel(_s.GfoldHoverHandoffAltM, hoverCanFly, _landHoverKnown);
        if (!ConsoleUi.Fold(label, HandOversFoldId, openByDefault, entryRows ? PageText.HandOversTip : PageText.HoverHandOverTip))
            return;
        ImGui.PushID(HandOversFoldId);
        try
        {
            if (!entryRows)
            {
                // The hover handoff decides where G-FOLD stops flying, so it is part of what the landing does, and it gets changed per craft. On a craft that cannot hover it does nothing now, and a lighter craft can use it.
                ConsoleUi.NoteShape("AfcLandHoverUnused", hoverCanFly ? 0 : 1);
                if (!hoverCanFly)
                    ConsoleUi.MutedWrapped(PageText.HoverNotUsed);
                using (ConsoleUi.Dim(!hoverCanFly))
                    GaugeRow(Specs.GfoldHoverHandoffAltM, ref _s.GfoldHoverHandoffAltM);
                return;
            }
            using (new ImGuiDisabledScope(ThreeDofLive))
            {
                GaugeRowCheck(Specs.ThreeDofFromGlide, ref _s.ThreeDofFromGlide);
                using (new ImGuiDisabledScope(!_s.ThreeDofFromGlide))
                    GaugeRow(Specs.ThreeDofEngageTgoS, ref _s.ThreeDofEngageTgoS);
            }
            GaugeRowCheck(Specs.ThreeDofSixDofHandover, ref _s.ThreeDofSixDofHandover);
            using (new ImGuiDisabledScope(!_s.ThreeDofSixDofHandover))
                GaugeRow(Specs.ThreeDofSixDofTimeS, ref _s.ThreeDofSixDofTimeS);
        }
        finally
        {
            ImGui.PopID();
        }
    }
}
