#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Features.LandingSites;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
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

    // A site row has the room for a degree value to three decimals, the precision of Fmt.LatLon. The narrowest field at interface scale 1 that shows a value such as "-80.600" whole, so a long body name yields first.
    private const string SiteCellFormat = "%.3f";
    private const float SiteFieldLeast = 78f;

    private static bool LandFlies => _s.Active || _s.EngagePending || _s.SixDofStandby || EntryLive(_s) || DescentLive || BoostbackLive;

    private static LandSolver LandSolverOf(LandWay way)
        => way == LandWay.FromOrbit || (way == LandWay.Entry && !PageHasAir) ? LandSolvers.Shown(LandSolvers.Chosen(false, _s.UseSixDofLanding), LandingMachineLive)
            : way is LandWay.Entry or LandWay.Glide ? LandSolver.Entry
            : LandSolverRoute;

    /// <summary>Whether the body under the craft has air, as the console read it this frame: it decides orbital entry or the braking burn.</summary>
    private static bool PageHasAir => Ui.Console.AfcConsole.Frame.Situation.HasAir;

    /// <summary>The way's GLIDE row chose FLIP LANDING, for the commit and the flying page.</summary>
    internal static bool GlideFlipChosen(LandWay way) => GlideFlips(way);

    /// <summary>FLIP LANDING flies: on GLIDE, or on LAND NOW where a glide flies first. Orbital entry flies it, from bellyflop past its hand-over.</summary>
    private static bool FlipWay(LandWay way) => (way == LandWay.FromHere && LandSolverRoute == LandSolver.Entry && _s.LandFlip) || (way == LandWay.Glide && _s.EntryFlip);

    /// <summary>The way's path starts with orbital entry: ENTRY/BRAKING where there is air, or LAND NOW's FLIP LANDING.</summary>
    private static bool OrbitalEntryWay(LandWay way) => (way == LandWay.Entry && PageHasAir) || FlipWay(way);

    /// <summary>LAND NOW's RETRO BURN: the 3-DOF's glide and landing burn.</summary>
    private static bool RetroWay(LandWay way) => (way == LandWay.FromHere && LandSolverRoute == LandSolver.Entry && !_s.LandFlip) || (way == LandWay.Glide && !_s.EntryFlip);

    /// <summary>The picture of orbital entry, for a way whose path starts with it.</summary>
    internal static bool OrbitalEntryPictureFor(Vehicle vehicle, LandWay way) => OrbitalEntryWay(way);

    /// <summary>
    /// The craft has an engine to land on, active or not: an entry hands it to bellyflop and touchdown. With none (a capsule), orbital entry holds its attitude after the hand-over and the pilot flies the rest.
    /// </summary>
    private static bool LandEngineOnCraft(Vehicle vehicle)
    {
        if (vehicle?.Parts?.States == null
            || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>.TryGetFrom(vehicle.Parts.States, out var engines))
            return false;
        foreach (var engine in engines.ModulesAndStates)
            if (engine.Module.Rockets is { Length: > 0 })
                return true;
        return false;
    }

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
        // On LAND NOW a glide is not a pick of the player: while no landing mode flies, the page sets it from the situation of the craft, and a flying mode sets it through FollowLiveLanding. The player picks how it falls (FLIP LANDING or RETRO BURN).
        // The deorbit's next step is the body's: orbital entry where there is air, the braking burn where there is none, and ENTRY/BRAKING flies it with no deorbit burn first. An entry ends in bellyflop and touchdown when the craft has an engine to land on.
        if (!LandFlies)
        {
            // GLIDE with RETRO BURN is the player's pick of the 3-DOF, from any fall.
            _s.UseThreeDofLanding = (way == LandWay.Glide && !_s.EntryFlip) || EntryFliesFirst(vehicle);
            bool air = HasPhysicalAtmosphere(vehicle.Orbit.Parent);
            if (way is LandWay.FromOrbit or LandWay.Entry && !DeorbitTargetLocked)
            {
                _s.DeorbitOnly = air;
                _s.DeorbitBrakingOnly = way == LandWay.Entry;
            }
            if (!OrbitalEntryLive && !_s.OrbitalEntry.EngagePending)
            {
                _s.OrbitalEntryBellyflop = LandEngineOnCraft(vehicle);
                // LAND NOW engages orbital entry only for its FLIP LANDING; DEORBIT and ENTRY/BRAKING by their own GLIDE row.
                _s.OrbitalEntryFlip = way is LandWay.FromHere or LandWay.Glide || _s.EntryFlip;
            }
        }
        LandSolver solver = LandSolverOf(way);
        DescentFacts descent = LandWays.HasSolver(way) || way is LandWay.Entry or LandWay.Glide ? ReadDescentFacts(vehicle) : default;
        DescentFacts judged = _s.Active || _s.SixDofStandby ? default : descent;
        ReadHoverAnswer(vehicle, in judged);
        NoteLandFacts(way, solver, in judged, HasPhysicalAtmosphere(vehicle.Orbit.Parent));
        StepFlatSearch(vehicle.Orbit.Parent, FlatSearchBudgetMs);
        if (way != LandWay.Hover && !NoSitePicked(vehicle.Orbit.Parent))
        {
            SlopeVerdict verdict = SiteSlopeText.Verdict(SiteSlopeNowDeg(vehicle.Orbit.Parent), _s.SiteTipDeg, _s.SiteStandDeg);
            string fact = SiteSlopeText.Fact(verdict, _s.SiteSlopeDeg, _s.SiteTipDeg, _s.SiteStandDeg);
            if (fact.Length > 0)
                NoteLandFact(fact, danger: verdict == SlopeVerdict.CannotStand);
        }
        return new LandRead(descent, LandFlies);
    }

    internal static void DrawLandInputRows(Vehicle vehicle, LandWay way, in SituationFacts situation, in LandRead read, bool pickSite)
    {
        DrawDeorbitFinishRows(way);
        DrawLandSiteRows(vehicle, way, in situation, pickSite);
        DrawLandEngineRows(vehicle);
        DrawLandMethodRows(vehicle, way, in situation, in read);
    }

    /// <summary>The rows of the deorbit burn where an orbital entry follows it: the periapsis the burn lowers the orbit to, and whether its time is chosen for the site (OrbitalEntryAim). A braking burn's approach is in the GNC fold.</summary>
    internal static void DrawDeorbitFinishRows(LandWay way)
    {
        if (way != LandWay.FromOrbit || !PageHasAir)
            return;
        using (new ImGuiDisabledScope(DeorbitTargetLocked))
        {
            ConsoleUi.InputDoubleRow("ENTRY PERIAPSIS".AsSpan(), "##AfcEntryPeriapsis"u8,
                ref _s.DeorbitEntryPeriapsisKm, 1.0, 10.0, "%.1f"u8, unit: "km");
            GaugeRowCheck(Specs.OrbitalEntryAimDeorbit, ref _s.OrbitalEntryAimDeorbit);
        }
    }

    internal static void DrawLandSiteRows(Vehicle vehicle, LandWay way, in SituationFacts situation, bool pickSite)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null || way == LandWay.Hover)
            return;
        using (ConsoleUi.Dim(DeorbitTargetLocked || (way == LandWay.Boostback && situation.Where == CraftSituation.Ground && NoStageHome)))
        {
            DrawLandSiteRow(parent, pickSite);
            DrawLandSiteList(vehicle, parent, way);
        }
        DrawSiteSlopeRows(vehicle, parent);
        ConsoleUi.Gap();
    }

    /// <summary>
    /// The ground at the site under the site row: its slope over the footprint of the legs, in the warning colour above the slope on which the craft goes over and in the danger colour above the slope on which it cannot stand, with the limit beside it and FIND FLAT, which searches the ground around the site. Once the search has its answer, a second row names the flattest point with its slope, distance and direction, and MOVE SITE takes it. While that answer still tips the craft over, the button searches three times as far, and after the widest search it stays off.
    /// </summary>
    private static void DrawSiteSlopeRows(Vehicle vehicle, IParentBody parent)
    {
        bool shows = !NoSitePicked(parent) && double.IsFinite(SiteSlopeNowDeg(parent)) && double.IsFinite(_s.SiteTipDeg);
        ConsoleUi.NoteShape("AfcLandSlope", shows ? 1 : 0);
        if (!shows)
            return;
        SlopeVerdict verdict = SiteSlopeText.Verdict(_s.SiteSlopeDeg, _s.SiteTipDeg, _s.SiteStandDeg);
        bool found = FlatSearchFits(parent);
        bool searching = found && !double.IsFinite(_s.FlatSiteSlopeDeg);
        float h = ConsoleStyle.FrameHeightPx;
        double nextRadius = NextFlatSearchRadiusM(parent);
        bool searchedWidest = !(nextRadius > 0.0);
        string findVerb = searching ? SiteSlopeText.SearchingVerb : SiteSlopeText.FindVerbFor(searchedWidest ? FlatSearchWidestM : nextRadius, FlatSearchRadiusM);
        float findW = Math.Max(ConsoleWidgets.ButtonWidth(SiteSlopeText.FindVerbFor(FlatSearchWidestM, FlatSearchRadiusM).AsSpan()), ConsoleWidgets.ButtonWidth(SiteSlopeText.SearchingVerb.AsSpan()));
        uint? ink = verdict switch
        {
            SlopeVerdict.CannotStand => ConsoleStyle.DangerU32,
            SlopeVerdict.TipsOver => Ui.Canvas.Palette.Warn.U32,
            _ => null,
        };
        ConsoleUi.BeginRow(SiteSlopeText.SlopeLabel(_s.SiteSlopeDeg), findW, ink);
        bool hovered = ConsoleWidgets.RowHovered;
        ConsoleUi.RowDetail(SiteSlopeText.LimitDetail(_s.SiteTipDeg), findW);
        ConsoleUi.RowRight(findW);
        bool find;
        using (new ImGuiDisabledScope(searching || searchedWidest || DeorbitTargetLocked))
            find = ConsoleWidgets.Button(findVerb.AsSpan(), "AfcFindFlat".AsSpan(), new float2(findW, h));
        ConsoleUi.NoteItem("AfcFindFlat");
        bool wider = (searchedWidest || nextRadius > FlatSearchRadiusM) && ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled);
        if (wider)
            ConsoleUi.Tooltip((searchedWidest ? SiteSlopeText.WidestCard(_s.FlatSiteSlopeDeg, _s.SiteTipDeg, FlatSearchWidestM) : SiteSlopeText.WiderCard(_s.FlatSiteSlopeDeg, _s.SiteTipDeg, nextRadius)).AsSpan());
        else if (hovered)
            ConsoleUi.Tooltip(SiteSlopeText.Card(_s.SiteSlopeDeg, _s.SiteTipDeg, _s.SiteStandDeg, _s.SiteFootprintM).AsSpan());
        if (find && !searching && !searchedWidest && !DeorbitTargetLocked && StartFlatSearch(vehicle))
            (found, searching) = (true, true);
        ConsoleUi.EndRow();

        ConsoleUi.NoteShape("AfcLandFlat", !found ? 0 : searching ? 1 : 2);
        if (!found)
            return;
        if (searching)
        {
            SiteSlope.FlatSearch search = _s.FlatSearch;
            ConsoleUi.Muted(SiteSlopeText.Searching(search.RadiusM, search.Count > 0 ? (double)search.Evaluated / search.Count : 0.0).AsSpan());
            return;
        }
        string text = SiteSlopeText.Found(_s.FlatSiteSlopeDeg, _s.FlatSiteDistanceM, _s.FlatSiteBearingDeg, _s.SiteSlopeDeg, _s.FlatSearch.RadiusM);
        if (!SiteSlopeText.IsBetter(_s.FlatSiteSlopeDeg, _s.SiteSlopeDeg))
        {
            ConsoleUi.Muted(text.AsSpan());
            return;
        }
        float moveW = ConsoleWidgets.ButtonWidth(SiteSlopeText.MoveVerb.AsSpan());
        ConsoleUi.BeginRow(text, moveW);
        ConsoleUi.RowRight(moveW);
        bool move;
        using (new ImGuiDisabledScope(DeorbitTargetLocked))
            move = ConsoleWidgets.Button(SiteSlopeText.MoveVerb.AsSpan(), "AfcMoveFlat".AsSpan(), new float2(moveW, h));
        ConsoleUi.NoteItem("AfcMoveFlat");
        if (move)
            MoveToFlatSite(parent);
        ConsoleUi.EndRow();
    }

    /// <summary>
    /// The path of the way, and above it the one choice the player makes in it: on LAND NOW, where a glide flies first, how the craft falls to its landing burn (FLIP LANDING or RETRO BURN). What stands against the landing burn stands under the path, as does the G-FOLD fallback.
    /// </summary>
    internal static void DrawLandMethodRows(Vehicle vehicle, LandWay way, in SituationFacts situation, in LandRead read)
    {
        if (vehicle.Orbit?.Parent == null || way == LandWay.Hover)
            return;
        LandSolver solver = LandSolverOf(way);
        bool canLand = LandEngineOnCraft(vehicle);
        // The glide is a choice where one follows: on LAND NOW where the craft falls through air, and after an orbital entry on a craft with an engine to land on.
        if (situation.HasAir && (way == LandWay.FromHere ? solver == LandSolver.Entry : way == LandWay.Glide || way is LandWay.FromOrbit or LandWay.Entry && canLand))
            DrawGlideRow(way);
        ChainStep[] chain = LandChain.For(way, solver, situation.Where, situation.HasAir, GlideFlips(way), canLand, BelowBrakingStart(vehicle), _landHoverFlies, KsaParachutes.Count(vehicle) > 0);
        if (LandChain.Shows(chain))
            DrawPathStrip(way, chain, WayCannotFly(vehicle, way, solver, in situation), situation.HasAir);
        DrawPathNotes(way, solver, in situation, read.Descent);
        ConsoleUi.Gap();
    }

    /// <summary>
    /// The path while the way flies, above its live rows, with the step flying now in brackets: the same path the form drew before the press. The steps are not buttons here; the PLAN row opens the form with them.
    /// </summary>
    private static void DrawLivePath(Vehicle vehicle, LandWay way)
    {
        if (way == LandWay.Hover)
            return;
        SituationFacts situation = Ui.Console.ConsoleFrame.ReadFacts(vehicle);
        LandSolver solver = LandSolverOf(way);
        ChainStep[] chain = LandChain.For(way, solver, situation.Where, situation.HasAir, LiveGlideFlips(way), LandEngineOnCraft(vehicle), BelowBrakingStart(vehicle), _landHoverFlies, KsaParachutes.Count(vehicle) > 0);
        if (LandChain.Shows(chain))
            ConsoleUi.ChainStrip(chain, live: LiveStepIndex(chain));
    }

    /// <summary>
    /// The glide of the path that flies: none while the retro burn flies (it is the glide), the one orbital entry was engaged with while it flies or waits, else the way's own GLIDE choice. The page's choice can differ from what flies: GLIDE picks the after-entry row, which an entry took at its engage.
    /// </summary>
    private static bool LiveGlideFlips(LandWay way)
        => EntryLive(_s) ? false
            : OrbitalEntryLive || _s.OrbitalEntry.EngagePending ? _s.OrbitalEntryFlip
            : GlideFlips(way);

    /// <summary>Where in the path the craft is: the step of the mode that flies now, found by its name in <paramref name="chain"/>; -1 while nothing flies or the step is not in this way's path.</summary>
    private static int LiveStepIndex(ChainStep[] chain)
    {
        string live = LiveStepText();
        if (live.Length == 0)
            return -1;
        for (int i = 0; i < chain.Length; i++)
            if (chain[i].Text == live)
                return i;
        return -1;
    }

    /// <summary>The name in the path of what flies now, the mode closest to the ground first: touchdown guidance, the hover, the retro burn, orbital entry and its bellyflop, the deorbit's phases, the boostback's.</summary>
    private static string LiveStepText()
    {
        if (_s.Active || _s.EngagePending)
            return LandSolvers.Label(LandSolver.Touchdown);
        if (_s.LandingPhase is LandingPhase.TerminalHover or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake)
            return "HOVER";
        if (EntryLive(_s))
            return EntryMethods.GlideBurnLabel;
        if (OrbitalEntryLive || _s.OrbitalEntry.EngagePending)
            return _s.OrbitalEntry.Phase switch
            {
                OrbitalEntryPhase.Bellyflop => LandChain.Bellyflop,
                OrbitalEntryPhase.Holding when _s.OrbitalEntry.ChutesArmed => LandChain.Parachutes,
                OrbitalEntryPhase.Holding when _s.OrbitalEntryBellyflop => EntryMethods.OrbitalLabel,
                OrbitalEntryPhase.Holding => LandChain.Hold,
                _ => EntryMethods.OrbitalLabel,
            };
        switch (_s.LandingPhase)
        {
            case LandingPhase.DeorbitPlanning or LandingPhase.DeorbitCoast or LandingPhase.DeorbitNodePending or LandingPhase.DeorbitBurn:
                return SituationText.IsFalling(Ui.Console.AfcConsole.Frame.Situation.Where) ? LandChain.StockBurnIfNeeded : LandChain.StockBurn;
            case LandingPhase.TransferPlanning or LandingPhase.TransferCoast or LandingPhase.Coast:
                return "COAST";
            case LandingPhase.Prep or LandingPhase.Burn:
                return EntryMethods.BrakingLabel;
            case LandingPhase.GfoldDescent:
                return LandSolvers.Label(LandSolver.Gfold);
        }
        return _s.BoostbackPhase switch
        {
            BoostbackPhase.Separation => "SETTLE",
            BoostbackPhase.Rotation => "SLEW",
            BoostbackPhase.Boostback => "BURN",
            BoostbackPhase.Glide => "GLIDE",
            _ => "",
        };
    }

    // The step of the path whose settings show under it, by its name; empty for none. Not kept: the page opens with the path alone.
    private static string _landStepOpen = "";

    /// <summary>
    /// The path, each step with settings a button: a click shows that step's settings under the path, the same values as in the folds below (HANDOVER, GNC), and a second click hides them. The Player rows draw first, then the Expert rows of the step, as a GNC fold draws them.
    /// </summary>
    private static void DrawPathStrip(LandWay way, ChainStep[] chain, bool cannotFly, bool hasAir)
    {
        Span<bool> clickable = stackalloc bool[chain.Length];
        int open = -1;
        for (int i = 0; i < chain.Length; i++)
        {
            clickable[i] = LandChain.HasSettings(chain[i].Text);
            if (clickable[i] && chain[i].Text == _landStepOpen)
                open = i;
        }
        int clicked = ConsoleUi.ChainStrip(chain, cannotFly, "AfcLandPath", open, clickable, LiveStepIndex(chain));
        if (clicked >= 0)
        {
            open = clicked == open ? -1 : clicked;
            _landStepOpen = open >= 0 ? chain[open].Text : "";
            ConsoleUi.ShapeChanged = true;
        }
        ConsoleUi.NoteShape("AfcLandStep", open + 1);
        if (open < 0)
            return;
        string before = open > 0 ? chain[open - 1].Text : "";
        ImGui.PushID("AfcLandStep");
        try
        {
            ConsoleUi.SectionHead(chain[open].Text);
            DrawStepRows(way, chain[open].Text, before, hasAir);
            using (TierScope.Push(UiTier.Expert))
                DrawStepRows(way, chain[open].Text, before, hasAir);
        }
        finally
        {
            ImGui.PopID();
        }
    }

    /// <summary>The settings of one step of the path, in the scope of the tier drawn now. <paramref name="before"/> is the step before it, whose hand-over to this one this step's rows set.</summary>
    private static void DrawStepRows(LandWay way, string step, string before, bool hasAir)
    {
        float innerW = ConsoleStyle.ContentAvailWidth();
        switch (step)
        {
            case LandChain.StockBurn or LandChain.StockBurnIfNeeded when hasAir:
                if (TierScope.IsPrimaryPass)
                    DrawDeorbitFinishRows(LandWay.FromOrbit);
                DrawEntryAimRows();
                break;
            case LandChain.StockBurn or LandChain.StockBurnIfNeeded:
            case EntryMethods.BrakingLabel:
                DrawApproachSection(innerW);
                break;
            case EntryMethods.OrbitalLabel:
                if (EntryRetro)
                    GaugeRow(Specs.OrbitalEntryRetroAimShortKm, ref _s.OrbitalEntryRetroAimShortKm);
                GaugeRow(Specs.OrbitalEntryQMaxKPa, ref _s.OrbitalEntryQMaxKPa);
                GaugeRow(Specs.OrbitalEntryGMax, ref _s.OrbitalEntryGMax);
                GaugeRow(Specs.OrbitalEntryHeatMaxKWm2, ref _s.OrbitalEntryHeatMaxKWm2);
                GaugeRow(Specs.OrbitalEntryHandoverMach, ref _s.OrbitalEntryHandoverMach);
                GaugeRow(Specs.OrbitalEntryHandoverAltKm, ref _s.OrbitalEntryHandoverAltKm);
                DrawOrbitalEntryAttitudeRows();
                DrawOrbitalEntryBankRows();
                break;
            case LandChain.Bellyflop:
                GaugeRow(Specs.OrbitalEntryBellyflopArmMach, ref _s.OrbitalEntryBellyflopArmMach);
                GaugeRow(Specs.OrbitalEntryBellyflopAimUprangeM, ref _s.OrbitalEntryBellyflopAimUprangeM);
                GaugeRow(Specs.OrbitalEntryBellyflopPitchUpDeg, ref _s.OrbitalEntryBellyflopPitchUpDeg);
                GaugeRow(Specs.OrbitalEntryBellyflopAngleMaxDeg, ref _s.OrbitalEntryBellyflopAngleMaxDeg);
                GaugeRow(Specs.OrbitalEntryBellyflopBankMaxDeg, ref _s.OrbitalEntryBellyflopBankMaxDeg);
                break;
            case EntryMethods.GlideBurnLabel:
                if (way == LandWay.Boostback)
                    using (new ImGuiDisabledScope(ThreeDofLive))
                    {
                        GaugeRow(Specs.ThreeDofStandbyQkPa, ref _s.ThreeDofStandbyQkPa);
                        GaugeRow(Specs.ThreeDofEngageQkPa, ref _s.ThreeDofEngageQkPa);
                        GaugeRow(Specs.ThreeDofEngageTgoS, ref _s.ThreeDofEngageTgoS);
                    }
                DrawBurnLimitRow();
                DrawGlideAttitudeRows();
                break;
            case "TOUCHDOWN":
                if (before == EntryMethods.GlideBurnLabel)
                    GaugeRow(Specs.ThreeDofSixDofTimeS, ref _s.ThreeDofSixDofTimeS);
                else if (before == LandChain.Bellyflop)
                    GaugeRow(Specs.OrbitalEntryBellyflopTouchdownS, ref _s.OrbitalEntryBellyflopTouchdownS);
                DrawBurnLimitRow();
                break;
            case "G-FOLD":
            case "HOVER":
                GaugeRow(Specs.GfoldHoverHandoffAltM, ref _s.GfoldHoverHandoffAltM);
                break;
            case "BURN":
                DrawBoostbackBurnSection();
                break;
            case "GLIDE":
                DrawBoostbackGlideSection();
                break;
        }
    }

    /// <summary>The way's GLIDE row chose FLIP LANDING: LAND NOW's own (RETRO BURN by default), or the glide's own, on GLIDE and after an orbital entry (FLIP LANDING by default).</summary>
    private static bool GlideFlips(LandWay way) => way == LandWay.FromHere ? _s.LandFlip : _s.EntryFlip;

    /// <summary>The GLIDE row: FLIP LANDING or RETRO BURN, kept with the craft, one for LAND NOW and one for after an orbital entry, each with its own default. On LAND NOW either can be picked while the other flies: the commit then takes the craft over, after the question of the footer.</summary>
    private static void DrawGlideRow(LandWay way)
    {
        bool flip = GlideFlips(way);
        // MODE / USE / END, as the landing table names a mode: END is the mode each hands to, as it declares it.
        var rows = new string[EntryMethods.GlideLabels.Length][];
        Span<TableMark> marks = stackalloc TableMark[rows.Length];
        for (int i = 0; i < rows.Length; i++)
        {
            var facts = new PathFacts(way, HasAir: true, Flip: i == 1);
            GuidanceModeId mode = i == 1 ? GuidanceModeId.OrbitalEntry : GuidanceModeId.EntryHere;
            string end = GuidanceModes.NextOf(mode, in facts) is GuidanceModeId next ? GuidanceModes.Of(next)?.MethodLabel ?? "" : "";
            rows[i] = EntryMethods.GlideRow(i, end);
            marks[i] = LandSolvers.MethodMark(i, false, false);
        }
        int picked = ConsoleUi.ChoiceTable("AfcLandGlide", LandSolvers.TableHeads, rows, flip ? 1 : 0, cards: EntryMethods.GlideCards, marks: marks, radio: true);
        if (picked >= 0 && (picked == 1) != flip)
        {
            if (way == LandWay.FromHere)
                _s.LandFlip = picked == 1;
            else
                _s.EntryFlip = picked == 1;
            ConsoleUi.ShapeChanged = true;
        }
    }

    /// <summary>Under the path: what stands against the powered end (LandSolvers.NoteOf), and why touchdown guidance is held while G-FOLD flies.</summary>
    private static void DrawPathNotes(LandWay way, LandSolver solver, in SituationFacts situation, in DescentFacts descent)
    {
        bool powered = LandWays.HasSolver(way) && solver != LandSolver.Entry;
        var facts = new LandNowFacts(way, solver, situation.HasAir, DeorbitTargetLocked, LandingMachineLive, _s.Active || _s.SixDofStandby ? default : descent, situation.Body);
        RowNote note = powered ? LandSolvers.NoteOf(solver, in facts) : RowNote.None;
        string text = note.Text.Length > 0 && note.Text != LandSolvers.HeldNote ? LandSolvers.Label(solver) + ": " + note.Text : "";
        ConsoleUi.NoteShape("AfcLandPathNote", text.Length);
        if (text.Length > 0)
            ConsoleUi.Sentence(text.AsSpan(), note.Stops ? ConsoleUi.WarnInk : Ui.Canvas.Palette.WarnDim.F4);
        string line = powered ? LandSolvers.HeldLine(in facts) : "";
        ConsoleUi.NoteShape("AfcLandHeld", line.Length);
        if (line.Length > 0)
            ConsoleUi.Sentence(line.AsSpan(), ConsoleStyle.TextLabel);
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
        DrawLivePath(vehicle, way);
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

    internal static void DrawLandPlanLine(LandWay way) => ConsoleUi.MutedWrapped(FlightView.LandPlanLine(way, way == LandWay.FromHere ? LandDrawnSolver : LandSolverOf(way), DeorbitTargetLocked, _landHoverFlies, OrbitalEntryWay(way)));

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
        if (f.NoSteering)
            NoteLandFact(PageText.NoSteeringFact, danger: true, calm: true);
        else if (f.OverPowered)
            NoteLandFact(PageText.OverPoweredFact(f.FeasibleFloor), danger: true, next: ConsolePage.Tune, calm: true);
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
            else if (way == LandWay.Glide && RetroWay(way))
                DrawLiftNote(vehicle, LandWay.FromHere, roomOnly: false);
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
                wayParts = [new FoldPart(FoldNames.Approach, !PageHasAir, InExpert(() => DrawApproachSection(innerW))), new FoldPart(FoldNames.EntryAim, PageHasAir && _s.OrbitalEntryAimDeorbit, InExpert(DrawEntryAimRows))];
                break;
            case LandWay.Entry when !PageHasAir:
                doubles = Specs.LandFromOrbitExpert;
                bools = Specs.LandFromOrbitExpertBools;
                wayParts = [new FoldPart(FoldNames.Approach, true, InExpert(() => DrawApproachSection(innerW)))];
                break;
            case LandWay.Boostback:
                doubles = Specs.LandBoostbackExpert;
                wayParts = [new FoldPart(FoldNames.Burn, true, InExpert(DrawBoostbackBurnSection)), new FoldPart(FoldNames.Glide, true, InExpert(DrawBoostbackGlideSection))];
                break;
            case LandWay.Entry:
            case LandWay.FromHere or LandWay.Glide when FlipWay(way):
                doubles = Specs.LandOrbitalEntryExpert;
                bools = Specs.LandOrbitalEntryExpertBools;
                wayParts = [new FoldPart(FoldNames.Attitude, true, InExpert(DrawOrbitalEntryAttitudeRows)), new FoldPart(FoldNames.Bank, true, InExpert(DrawOrbitalEntryBankRows))];
                break;
            case LandWay.Glide:
            case LandWay.FromHere when LandSolverRoute == LandSolver.Entry:
                doubles = Specs.LandEntryExpert;
                bools = Specs.LandEntryExpertBools;
                wayParts = [new FoldPart(FoldNames.BurnLimit, true, InExpert(DrawBurnLimitRow)), new FoldPart(FoldNames.Attitude, true, InExpert(DrawGlideAttitudeRows))];
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

    // A fixed named site is a deck, whose height the file gives, so it reads DECK and not the sea under it.
    private static string SiteTerrainText(IParentBody parent)
    {
        string height = Fmt.Distance(SiteTerrainHeight(parent));
        if (LandingSiteRegistry.TryFixedHeight(parent, _s.SiteLatDeg, _s.SiteLonDeg, out _))
            return height + " (DECK)";
        return KsaSurfaceHeight.IsWater(parent, SiteDirCcf(_s.SiteLatDeg, _s.SiteLonDeg)) ? height + " (SEA)" : height;
    }

    private static void DrawBurnLimitRow() => GaugeRow(Specs.LandingAccelLimitG, ref _s.LandingAccelLimitG);

    /// <summary>GNC > ATTITUDE of RETRO BURN: which attitude control flies the glide.</summary>
    private static void DrawGlideAttitudeRows()
    {
        GaugeRowCheck(Specs.ThreeDofNewAttitude, ref _s.ThreeDofNewAttitude);
        using (new ImGuiDisabledScope(!_s.ThreeDofNewAttitude))
            GaugeRow(Specs.ThreeDofRcsDeadbandDeg, ref _s.ThreeDofRcsDeadbandDeg);
    }

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
        bool deorbit = way is LandWay.FromOrbit or LandWay.Entry or LandWay.FromHere && (DescentLive || _s.DeorbitRequest != null);
        bool feasibilityAlone = !siteRows && !impact && !deorbit;
        ConsoleUi.PartsFold(PageText.Numbers, LandNumbersFoldId,
        [
            new FoldPart(FoldNames.Distance, siteRows, () => GaugeRowText("DISTANCE TO SITE", Fmt.Distance(AngleBetween(orbit.StateVectors.PositionCci, SiteDirCciAt(parent, 0)) * parent.MeanRadius))),
            new FoldPart(FoldNames.Terrain, siteRows, () => GaugeRowText("SITE TERRAIN", SiteTerrainText(parent))),
            new FoldPart(FoldNames.Impact, impact, DrawBoostbackImpactDetails),
            new FoldPart(FoldNames.Feasibility, (LandWays.HasSolver(way) || way is LandWay.Entry or LandWay.Glide) && (descent.Known || FirstSolveStops(way, solver)), () => DrawDescentRows(way, solver, in descent, noSite, feasibilityAlone)),
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
        if (!GaugeRowCheck(UiTier.Player, PageText.ShowInWorld, "AfcLandInWorldRow", ref show))
            return;
        SaveOverlayPreferences();
        if (impact && show)
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
            case LandWay.Entry when PageHasAir:
            case LandWay.FromHere or LandWay.Glide when FlipWay(way):
                DrawOrbitalEntryContent(vehicle, innerW);
                break;
            case LandWay.Glide:
                Draw3DofLandingContent(vehicle, innerW, liftNote);
                break;
            case LandWay.Entry:
                DrawDescentContent(innerW);
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
        bool orbital = OrbitalEntryWay(way);
        bool entryRows = way == LandWay.Boostback || RetroWay(way);
        bool shows = entryRows || orbital ? HasPhysicalAtmosphere(parent) : LandWays.HasSolver(way) && solver == LandSolver.Gfold;
        ConsoleUi.NoteShape(HandOversFoldId, shows ? 1 : 0);
        if (!shows)
        {
            if (LandWays.HasSolver(way))
                ConsoleUi.FoldEmpty(PageText.NoHandOversLabel, PageText.NoHandOversTip(solver));
            return;
        }
        string label = orbital ? PageText.OrbitalHandOversLabel(_s.OrbitalEntryHandoverMach, _s.OrbitalEntryHandoverAltKm, EntryFlips ? LandChain.Bellyflop : EntryRetro ? EntryMethods.GlideBurnLabel : KsaParachutes.Count(Program.ControlledVehicle) > 0 ? LandChain.Parachutes : "HOLD ATTITUDE")
            : entryRows
            ? PageText.HandOversLabel(_s.ThreeDofEngageTgoS, _s.ThreeDofSixDofTimeS, _s.ThreeDofEngageQkPa)
            : PageText.HoverHandOverLabel(_s.GfoldHoverHandoffAltM, hoverCanFly, _landHoverKnown);
        if (!ConsoleUi.Fold(label, HandOversFoldId, openByDefault, orbital ? PageText.OrbitalHandOversTip : entryRows ? PageText.HandOversTip : PageText.HoverHandOverTip))
            return;
        ImGui.PushID(HandOversFoldId);
        try
        {
            if (orbital)
            {
                DrawOrbitalEntryHandOverRows();
                return;
            }
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
            // RETRO BURN takes a boostback glide, and TOUCHDOWN the end of its burn, always: the path says so. These rows say when.
            if (way == LandWay.Boostback)
                using (new ImGuiDisabledScope(ThreeDofLive))
                {
                    GaugeRow(Specs.ThreeDofStandbyQkPa, ref _s.ThreeDofStandbyQkPa);
                    GaugeRow(Specs.ThreeDofEngageQkPa, ref _s.ThreeDofEngageQkPa);
                    GaugeRow(Specs.ThreeDofEngageTgoS, ref _s.ThreeDofEngageTgoS);
                }
            GaugeRow(Specs.ThreeDofSixDofTimeS, ref _s.ThreeDofSixDofTimeS);
        }
        finally
        {
            ImGui.PopID();
        }
    }
}
