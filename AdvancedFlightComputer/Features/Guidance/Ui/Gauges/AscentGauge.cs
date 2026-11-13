#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private static bool AscentPlanBusy => _s.AscentPlanJob != null || _s.AscentPlanRequested;

    private const string FromPositionVerb = "FROM POSITION";

    private const float OrbitFieldLeast = 56f;

    // A row of the page. Picking a target turns the three orbit inputs into OUTPUTS of that pick, continuously recomputed and greyed out, because a chase orbit that disagreed with the target it was chasing was never anything but a mistake. Returns the width of its fields, so the row under it can line up.
    private static float DrawOrbitRow(Orbit orbit, IParentBody parent, bool driven, bool planeOnly)
    {
        float avail = ConsoleStyle.ContentAvailWidth();
        float cells = avail - ConsoleUi.LabelColumnWidth;
        float fieldW = CellFieldWidth(cells, out float rowWidth, ("PE", Specs.PeKm.Unit), ("AP", Specs.ApKm.Unit), ("INC", Specs.IncDeg.Unit));
        // In a narrow pane the fields need the room of the label, because PE, AP and INC name the row by themselves.
        bool label = fieldW >= OrbitFieldLeast * ImGuiHelper.InterfaceScale;
        if (!label)
        {
            cells = avail;
            fieldW = CellFieldWidth(cells, out rowWidth, ("PE", Specs.PeKm.Unit), ("AP", Specs.ApKm.Unit), ("INC", Specs.IncDeg.Unit));
        }
        // The name takes the violet of the band of the goal orbit in the picture, because these values set that band. With no room for the name PE and AP take it, and with no band no name has it.
        uint? bandInk = LaunchOrbitBandDraws ? Palette.Target.U32 : null;
        uint? cellInk = label ? null : bandInk;
        ConsoleUi.BeginRow(label ? "ORBIT" : "", cells, bandInk);
        float2 control = ImGui.GetCursorScreenPos();
        float x = control.X + ConsoleWidgets.RowControlWidth - rowWidth;

        // A plane target sets only the plane, so the altitudes stay the player's.
        using (new ImGuiDisabledScope((driven && !planeOnly) || AscentPlanBusy))
        {
            // Periapsis is always the lower: a pair entered the other way round swaps once the field is left (see OrderApsides).
            GaugeCell(Specs.PeKm, "PE", ref x, control.Y, fieldW, ref _s.PeKm, nameInk: cellInk);
            if (RowEditEnded(Specs.PeKm))
                OrderApsides();
            GaugeCell(Specs.ApKm, "AP", ref x, control.Y, fieldW, ref _s.ApKm, nameInk: cellInk);
            if (RowEditEnded(Specs.ApKm))
                OrderApsides();
        }
        using (new ImGuiDisabledScope(driven || AscentPlanBusy))
        {
            // A new inclination makes the old LAN meaningless, because the plane it named no longer passes overhead, so re-seed it from where the vehicle is.
            if (GaugeCell(Specs.IncDeg, "INC", ref x, control.Y, fieldW, ref _s.IncDeg))
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
        }
        ConsoleUi.EndRow();
        return fieldW;
    }

    private static void DrawPlaneRow(Orbit orbit, IParentBody parent, bool driven, float fieldW)
    {
        BeginKitRow("PLANE", controlWidth: ConsoleStyle.ContentAvailWidth() - ConsoleUi.LabelColumnWidth);
        float2 control = ImGui.GetCursorScreenPos();
        float x = control.X + ConsoleWidgets.RowControlWidth - CellsWithoutFields([("LAN", Specs.LanDeg.Unit)]) - fieldW;
        bool overPad = !driven && _s.LanFollowsPad;
        if (overPad)
        {
            float h = ConsoleStyle.FrameHeightPx;
            ConsoleStyle.PushLabelFont();
            float noteW = ImGui.CalcTextSize(PageText.LanOverPad).X;
            float2 at = new(x - CellGap - noteW, control.Y);
            ImGui.GetWindowDrawList().AddText(new float2(at.X, at.Y + (h - ImGui.GetFontSize()) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.TextMutedU32), PageText.LanOverPad);
            ConsoleStyle.PopFont();
            if (ImGui.IsMouseHoveringRect(at, new float2(at.X + noteW, at.Y + h)))
                ConsoleUi.Tooltip(PageText.LanOverPadTip.AsSpan());
        }
        else if (!driven)
        {
            float buttonW = ConsoleWidgets.ButtonWidth(FromPositionVerb.AsSpan());
            ImGui.SetCursorScreenPos(new float2(x - CellGap - buttonW, control.Y));
            bool fromPosition;
            using (new ImGuiDisabledScope(AscentPlanBusy))
                fromPosition = ConsoleWidgets.Button(FromPositionVerb.AsSpan(), "AfcLanFromPosition".AsSpan(), new float2(buttonW, ConsoleStyle.FrameHeightPx));
            if (fromPosition && !AscentPlanBusy)
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
            if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
                ConsoleUi.Tooltip(PageText.LanFromPositionTip.AsSpan());
        }
        using (new ImGuiDisabledScope(driven || overPad || AscentPlanBusy))
            GaugeCell(Specs.LanDeg, "LAN", ref x, control.Y, fieldW, ref _s.LanDeg);
        ConsoleUi.EndRow();
    }

    private static void DrawTargetFacts(ChaseStatus status, in ChasePlan plan)
    {
        switch (status)
        {
            case ChaseStatus.NotFound:
                GaugeRowText("TARGET", "NOT FOUND", ConsoleStyle.Danger);
                break;
            case ChaseStatus.DifferentBody:
                GaugeRowText("TARGET", "OTHER BODY", ConsoleStyle.Danger);
                break;
            case ChaseStatus.Ok when plan.Target == null:
                GaugeRowText("LAUNCH PLANE", FormattableString.Invariant($"{_s.PlaneTarget?.Label.ToUpperInvariant()}: INC {plan.IncDeg:F1} deg"));
                break;
            case ChaseStatus.Ok:
                GaugeRowText("TARGET ORBIT", PageText.TargetOrbit(plan.TargetPeKm, plan.TargetApKm, plan.IncDeg));
                break;
            case ChaseStatus.NoTarget when _s.MoonTransferId.Length > 0 && !_s.Running && _s.MoonChain == MoonChainStage.Idle:
                GaugeRowText("LAUNCH PLANE", "NO PLANE TO " + _s.MoonTransferId.ToUpperInvariant() + " FROM THIS SITE", ConsoleStyle.Danger);
                break;
        }
    }

    // Above this ground-relative speed the craft is flying, and a launch site has no meaning for the plane.
    private const double ReachGroundSpeedMs = 50.0;

    /// <summary>
    /// The plane change the launch would fly, before it flies. The site is taken where the body carries it in LanLeadSeconds, as LanOverhead and the launch window take it. A calculated climb with no target solves its own node over the pad (ConvexAscent), so only the part that no node avoids counts there.
    /// </summary>
    private static bool TryReachFacts(Orbit orbit, IParentBody parent, bool hasTarget, ChaseStatus status, in ChasePlan plan, out ReachFacts facts)
    {
        facts = default;
        // A target the site can reach is flown from its launch window, where the plane passes over the site.
        if (hasTarget && status != ChaseStatus.PlaneUnreachable)
            return false;
        if (!TryLaunchSite(orbit, parent, out double3 site))
            return false;
        double latDeg = UpfgTarget.RadToDeg(Math.Asin(Math.Clamp(site.Z / site.Length(), -1.0, 1.0)));

        double incDeg = hasTarget ? plan.IncDeg : _s.IncDeg;
        double lanDeg = hasTarget ? plan.LanDeg : _s.LanDeg;
        double peKm = hasTarget ? plan.PeKm : _s.PeKm;
        double apKm = hasTarget ? plan.ApKm : _s.ApKm;
        double least = LaunchReach.LeastPlaneChangeDeg(latDeg, incDeg);
        double now = least;
        // The calculated climb with a free launch time solves its own node over the pad (ConvexAscent), and the gravity turn has its LAN set over the pad at LAUNCH (SetGravityTurnPlane), so for both only the part that no node avoids counts.
        bool nodeOverPad = !hasTarget && (GravityTurnFlies || _s.ConvexOptimiseTiming);
        if (!nodeOverPad)
        {
            double3 n = UpfgTarget.OrbitNormal(UpfgTarget.DegToRad(incDeg), UpfgTarget.DegToRad(lanDeg));
            now = LaunchReach.PlaneChangeDeg(n.X, n.Y, n.Z, site.X, site.Y, site.Z);
        }
        double rBody = parent.MeanRadius;
        double ms = LaunchReach.PlaneChangeMs(parent.Mu, rBody + peKm * 1000.0, rBody + apKm * 1000.0, now);
        facts = new ReachFacts(latDeg, incDeg, now, least, ms, hasTarget);
        return true;
    }

    /// <summary>The LAUNCH commit flies the vertical rise and the gravity turn, not the calculated climb.</summary>
    private static bool GravityTurnFlies => !_s.FlyConvexAscent || BackupLaunchChosen;

    /// <summary>
    /// Where the launch site will be in <see cref="LanLeadSeconds"/>, in CCI, as LanOverhead and the launch window take it. False when the craft moves faster than <see cref="ReachGroundSpeedMs"/> over the ground, because a flying craft has no launch site.
    /// </summary>
    private static bool TryLaunchSite(Orbit orbit, IParentBody parent, out double3 site)
    {
        site = default;
        double3 r = orbit.StateVectors.PositionCci;
        double omega = parent.GetAngularVelocity();
        if (!double.IsFinite(omega) || r.Length() < 1.0)
            return false;
        if ((orbit.StateVectors.VelocityCci - double3.Cross(new double3(0, 0, omega), r)).Length() > ReachGroundSpeedMs)
            return false;
        double turn = omega * LanLeadSeconds;
        double c = Math.Cos(turn), s = Math.Sin(turn);
        site = new double3(c * r.X - s * r.Y, s * r.X + c * r.Y, r.Z);
        return true;
    }

    private static bool TryLaunchSiteLatitude(Orbit orbit, IParentBody parent, out double latDeg)
    {
        latDeg = double.NaN;
        if (!TryLaunchSite(orbit, parent, out double3 site))
            return false;
        latDeg = UpfgTarget.RadToDeg(Math.Asin(Math.Clamp(site.Z / site.Length(), -1.0, 1.0)));
        return true;
    }

    // The REACH row: a plane change the launch would fly, what it costs, and the nearest inclination the site reaches directly. Nothing shows for a plane the site reaches.
    private static void DrawReachRow(Orbit orbit, IParentBody parent, ChaseStatus status, in ChasePlan plan)
    {
        if (_s.Running || !TryReachFacts(orbit, parent, HasLaunchTarget, status, in plan, out ReachFacts facts) || !facts.Shows)
            return;
        GaugeRowText(PageText.ReachLabel, PageText.Reach(in facts), facts.OutOfReach ? ConsoleStyle.Danger : ConsoleUi.WarnInk);
        if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ConsoleUi.Tooltip(PageText.ReachTip(in facts, GravityTurnFlies).AsSpan());
        double offer = facts.OfferIncDeg;
        if (!double.IsFinite(offer) || _s.LaunchArmed || AscentPlanBusy)
            return;
        if (ConsoleUi.LinkRight(PageText.ReachFix(offer), "AfcReachFix", PageText.ReachFixTip))
        {
            _s.IncDeg = offer;
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
        }
    }

    // The TWR row: a first stage too weak for its craft, before the launch. Nothing shows for one that is not.
    private static void DrawTwrRow()
    {
        LaunchTwrFacts facts = _s.PadTwr;
        if (_s.Running || !facts.Shows)
            return;
        GaugeRowText(PageText.TwrLabel, PageText.Twr(in facts), facts.CannotLiftOff ? ConsoleStyle.Danger : ConsoleUi.WarnInk);
        if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ConsoleUi.Tooltip(PageText.TwrTip(in facts, GravityTurnFlies).AsSpan());
    }

    private static void DrawLaunchWindowRows(ChaseStatus status, in ChasePlan plan, bool nodeChoiceExists)
    {
        LaunchWindowRow row = FormRules.WindowRow(HasLaunchTarget, status == ChaseStatus.Ok && double.IsFinite(plan.WaitSec),
            status == ChaseStatus.PlaneUnreachable, nodeChoiceExists && (!_s.FlyConvexAscent || BackupLaunchChosen));
        ConsoleUi.NoteShape("AfcLaunchWindow", (int)row);
        switch (row)
        {
            case LaunchWindowRow.NodeChoice:
                int picked = ConsoleUi.ChoiceRow("WINDOW", "AfcLaunchNode", PageText.NodeLabels, _s.LaunchDescending ? 1 : 0, default, PageText.NodeCards, "",
                    Specs.LaunchDescending.Help);
                if (picked >= 0)
                    _s.LaunchDescending = picked == 1;
                break;
            case LaunchWindowRow.Countdown:
                // To IGNITION, which leads the plane crossing by LanLeadSeconds, or, once the calculated climb chose its own launch time for this window, to that.
                GaugeRowText("WINDOW", PageText.WindowCountdown(plan.WaitSec, LaunchNodeDescending(in plan), LanLeadSeconds, ExecuteLaunchInstant() - WindowInstant()));
                GaugeRowCheck(Specs.AutoLaunch, ref _s.AutoLaunch);
                DrawAutoLaunchArming();
                break;
            case LaunchWindowRow.NoCrossing:
                GaugeRowText("WINDOW", PageText.NoCrossing, ConsoleStyle.Danger);
                break;
        }
    }

    // The LIMITS fold of the page: the limits of the climb that the plan and the flight read. MAX Q shows with either climb, because SOLVE solves a plan with GRAVITY TURN too. The limit is part of the problem, so it is frozen while the problem is being solved.
    private static void DrawLimitsFold(IParentBody parent, bool openByDefault)
    {
        bool air = HasPhysicalAtmosphere(parent);
        if (!ConsoleUi.Fold(PageText.LimitsLabel(air, _s.ConvexQMaxKpa, _s.GLimitEnabled, _s.GLimitG), LimitsFoldId, openByDefault, PageText.LimitsTip(air)))
            return;
        ImGui.PushID(LimitsFoldId);
        try
        {
            using (ConsoleUi.Dim(_s.Running))
            using (new ImGuiDisabledScope(AscentPlanBusy))
            {
                if (air)
                    GaugeRow(Specs.ConvexQMaxKpa, ref _s.ConvexQMaxKpa);
                else
                    GaugeRowText(Specs.ConvexQMaxKpa.Tier, Specs.ConvexQMaxKpa.Label, PageText.NoAirRow, ConsoleStyle.TextMuted);
            }
            GaugeRowCheck(Specs.GLimitEnabled, ref _s.GLimitEnabled);
            // Greyed rather than hidden, because the limit is still worth reading when it is off.
            using (new ImGuiDisabledScope(!_s.GLimitEnabled))
                GaugeRow(Specs.GLimitG, ref _s.GLimitG);
        }
        finally
        {
            ImGui.PopID();
        }
    }

    // How the orbit plane is held, in the GNC fold. The LAN stands on the page, because it changes which orbit the craft goes to.
    private static void DrawOrbitPlaneSection(Vehicle vehicle, Orbit orbit, IParentBody parent, double bodyRadius)
    {
        if (!GuidanceRegion.Begin(FoldNames.Plane, GuidanceRegion.Expert))
            return;

        // Read only, because the rows grey out on it. The launch window step writes the chase result into the state (StepLaunchWindow), so the draw never does.
        ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, bodyRadius, out ChasePlan plan);
        bool driven = status == ChaseStatus.Ok || status == ChaseStatus.PlaneUnreachable;
        if (driven && plan.Target != null)
        {
            GaugeRow(Specs.ChaseOffsetKm, ref _s.ChaseOffsetKm);
            using (new ImGuiDisabledScope(double.IsNaN(plan.ArgPeDeg)))
                GaugeRowCheck(Specs.MatchTargetArgPe, ref _s.MatchTargetArgPe);
        }

        using (new ImGuiDisabledScope(driven || AscentPlanBusy))
        {
            GaugeRowCheck(Specs.ArgPeFixed, ref _s.ArgPeFixed);
            using (new ImGuiDisabledScope(!_s.ArgPeFixed))
                GaugeRow(Specs.ArgPeDeg, ref _s.ArgPeDeg);
        }

        GuidanceRegion.End();
    }

    private static void DrawInsertionSection()
    {
        if (!GuidanceRegion.Begin(FoldNames.Insertion, GuidanceRegion.Expert))
            return;

        // Outside the target's greying, since it is how to fly rather than where to go. Greyed instead while the argument of periapsis is fixed, which decides the insertion point by itself.
        using (new ImGuiDisabledScope(_s.ArgPeFixed))
        {
            GaugeRowCheck(Specs.OptimiseInsertion, ref _s.OptimiseInsertion);
            GaugeRowCheck(Specs.InsertBeforeApoapsis, ref _s.InsertBeforeApoapsis);
        }

        GuidanceRegion.End();
    }

    // Where the insertion row turns amber. Not limits, because nothing is refused and what an insertion costs depends on the vehicle, but where the cost has stopped being small: far round the ellipse from periapsis, or climbing steeply onto it.
    private const double InsertionWarnAnomalyDeg = 45.0;
    private const double InsertionWarnFpaDeg = 5.0;

    // Below this the insertion is before periapsis in any sense worth saying, rather than hovering over it.
    private const double InsertionDescendingFpaDeg = -0.5;

    /// <summary>
    /// Where the running ascent is aiming to insert on the target ellipse, whenever that is anywhere but periapsis: how far round from periapsis, at what altitude, and at what flight-path angle. These are the numbers a fixed argument of periapsis spends dV on, and the insertion search saves dV with. Drawn only while there is a solution to read them from.
    /// </summary>
    private static void DrawInsertionRows(double bodyRadius)
    {
        var aim = _s.Upfg.Aim;
        // A free insertion off periapsis that the floor did not put there is the search's doing.
        bool searched = !_s.ArgPeFixed && !aim.FloorLimited && aim.TrueAnomaly != 0.0;
        if (!_s.Running || !aim.Valid || !(_s.ArgPeFixed || aim.FloorLimited || searched))
            return;

        double nuDeg = aim.TrueAnomaly * (180.0 / Math.PI);
        double fpaDeg = aim.Fpa * (180.0 / Math.PI);
        // Only a fixed argument of periapsis can put the insertion somewhere dear: the search moves it only to save dV, and the floor only to keep the cutoff out of the air.
        bool far = _s.ArgPeFixed && (Math.Abs(nuDeg) > InsertionWarnAnomalyDeg || Math.Abs(fpaDeg) > InsertionWarnFpaDeg);
        bool descending = _s.ArgPeFixed && fpaDeg < InsertionDescendingFpaDeg;
        float4 warn = ConsoleUi.WarnInk;
        float4 fine = ConsoleStyle.TextPrimary;
        const UiTier D = UiTier.Diagnostics;

        GaugeRowText(D, "INSERTION",
            FormattableString.Invariant($"{nuDeg:+0.0;-0.0} deg from PE, ALT {(aim.Radius - bodyRadius) / 1000.0:F0} km, FPA {fpaDeg:+0.0;-0.0}"),
            far || descending || aim.FloorLimited ? warn : fine);
        if (aim.FloorLimited)
            GaugeRowText(D, "", "PE IN ATMOSPHERE. INSERTION AT ATMOSPHERE TOP.", warn);
        if (far)
            GaugeRowText(D, "", "FAR PAST PE. COSTS DV.", warn);
        else if (descending)
            GaugeRowText(D, "", "BEFORE PE. BURN FROM ORBIT MAY NOT CONVERGE.", warn);
        if (searched && double.IsFinite(_s.InsertionSearch.SavingMs))
            GaugeRowText(D, "", FormattableString.Invariant($"PLACED FOR DV: SAVES {Math.Max(_s.InsertionSearch.SavingS, 0.0):F1} s ({Math.Max(_s.InsertionSearch.SavingMs, 0.0):F0} m/s) VS INSERTING {(_s.InsertionSearch.AtApoapsis ? FormattableString.Invariant($"{UpfgInsertionSearch.ApoapsisLeadDeg:F0} deg BEFORE AP") : "AT PE")}"), fine);
    }

    private static void DrawTargetPicker(Vehicle vehicle)
    {
        BeginKitRow("TARGET");
        bool rowHovered = ConsoleWidgets.RowHovered;
        ConsoleUi.RowChip("TARGET", Owner.Stock, "STOCK");
        bool open = ConsoleUi.BeginCombo("AfcAscentTarget".AsSpan(), _s.TargetId.Length > 0 ? _s.TargetId : "None");
        if (!open && rowHovered)
            ConsoleUi.Card("TARGET", "Craft whose orbit LAUNCH targets. LAUNCH waits for its window. " + PageText.CraftTargetChipTip);
        if (open)
        {
            // Dropping the target frees the argument of periapsis again, because the target's was only ever copied for the chase.
            if (ImGui.Selectable("None", _s.TargetId.Length == 0) && _s.TargetId.Length > 0)
            {
                _s.TargetId = "";
                _s.ArgPeFixed = false;
            }

            CelestialSystem system = Universe.CurrentSystem;
            if (system != null)
            {
                ReadOnlySpan<Astronomical> all = system.All.AsSpan();
                for (int i = 0; i < all.Length; i++)
                {
                    if (all[i] is Vehicle v && !ReferenceEquals(v, vehicle)
                        && ImGui.Selectable(v.Id, v.Id == _s.TargetId))
                    {
                        _s.TargetId = v.Id;
                        // Mirror into the game's own targeting, so the map and the rendezvous gauge agree with us. Through the input buffer, because the draw overlaps the vehicle solver and a direct SetTarget rebuilds the flight plan it reads.
                        Universe.SetTargetCommand(vehicle.Id, v.Id);
                    }
                }
            }
            ConsoleWidgets.EndComboControl();
        }
        ConsoleUi.EndRow();
    }

    /// <summary>THEN: what follows a completed ascent. Stay in the parking orbit, fly by a moon of the launch body, or go into an orbit of it (StepMoonTransfer). With a moon picked and no target vessel, the launch plane is set for the moon (RefreshMoonLaunchPlane).</summary>
    private static void DrawMoonTransferPicker(Vehicle vehicle)
    {
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent == null)
            return;
        var moons = MoonsOf(parent);
        if (moons.Count == 0 && _s.MoonTransferId.Length == 0)
            return;
        bool chainRuns = _s.MoonChain != MoonChainStage.Idle;
        BeginKitRow("THEN");
        bool rowHovered = ConsoleWidgets.RowHovered;
        bool open = ConsoleUi.BeginCombo("AfcAscentThen".AsSpan(), _s.MoonTransferId.Length == 0 ? "Stay in orbit"
            : _s.MoonTransferId + (_s.MoonTransferCapture ? " orbit" : " flyby"));
        if (!open && rowHovered)
            ConsoleUi.Card("THEN", "After the ascent: stay in the parking orbit, fly by a moon, or go into an orbit of it. "
                + "For a moon, LAUNCH waits for the window of a plane that meets it, unless TARGET sets the plane. After the ascent AFC plans the injection burn, "
                + "a correction on the way when the periapsis is off, and for an orbit the capture burn at the periapsis. Stock AUTO flies each burn and AFC warps to it. "
                + "A burn you plan, or AUTO switched off, stops the chain. THEN goes back to Stay in orbit when the chain ends.");
        if (open)
        {
            // While the chain runs the pick only stops it, because its moon and its periapsis were latched at the end of the ascent.
            if (ImGui.Selectable("Stay in orbit", _s.MoonTransferId.Length == 0) && _s.MoonTransferId.Length > 0)
            {
                if (chainRuns)
                    _s.MoonChainStopRequested = true;
                else
                {
                    _s.MoonTransferId = "";
                    _s.MoonTransferCapture = false;
                }
            }
            if (!chainRuns)
                foreach (Celestial moon in moons)
                {
                    // An armed launch keeps the window of the plane it was armed for, so it takes no other moon.
                    using var armed = new ImGuiDisabledScope(_s.LaunchArmed && moon.Id != _s.MoonTransferId);
                    if (ImGui.Selectable(moon.Id + " flyby", moon.Id == _s.MoonTransferId && !_s.MoonTransferCapture))
                    {
                        _s.MoonTransferId = moon.Id;
                        _s.MoonTransferCapture = false;
                    }
                    if (ImGui.Selectable(moon.Id + " orbit", moon.Id == _s.MoonTransferId && _s.MoonTransferCapture))
                    {
                        _s.MoonTransferId = moon.Id;
                        _s.MoonTransferCapture = true;
                    }
                }
            ConsoleWidgets.EndComboControl();
        }
        ConsoleUi.EndRow();
        if (_s.MoonTransferId.Length > 0)
            using (new ImGuiDisabledScope(chainRuns))
                GaugeRow(Specs.MoonTransferPeKm, ref _s.MoonTransferPeKm);
    }

    // How the vehicle flies the ascent, as opposed to where it is going.
    private static void DrawAscentSettingsSection(float innerW)
    {
        if (!GuidanceRegion.Begin(FoldNames.Turn, GuidanceRegion.Expert))
            return;

        GaugeRow(Specs.TurnStartAltKm, ref _s.TurnStartAltKm);
        GaugeRow(Specs.TurnRateDegS, ref _s.TurnRateDegS);

        // FORCE ROLL is on by default: the ascent commands a roll as well as a thrust direction and switches the flight computer out of decoupled roll so it tracks it (see VehicleAutopilotState.ForceRoll). Off, the ascent holds whatever roll the vehicle lifted off with.
        GaugeRowCheck(Specs.ForceRoll, ref _s.ForceRoll);
        using (new ImGuiDisabledScope(!_s.ForceRoll))
            GaugeRow(Specs.ForceRollDeg, ref _s.ForceRollDeg);

        GuidanceRegion.End();
    }

    /// <summary>The booster reserve the calculated climb sized for the booster with a stage site, kg; NaN with no usable plan for that site.</summary>
    private static double PlannedReserveBoostbackDv(out double landingDvMs)
    {
        landingDvMs = double.NaN;
        AscentPlan p = _s.AscentPlan;
        AscentSolution sol = p?.Solution;
        if (p == null || !p.Usable || !double.IsFinite(sol.RecoveryReserveKg) || p.RecoverySite != RecoverySiteKey(BoostbackTarget()))
            return double.NaN;
        landingDvMs = sol.LandingDv;
        return sol.BoostbackDv;
    }

    /// <summary>
    /// What the booster with a stage site keeps for its way home. There is nothing to set: the stage site arms the reserve, and the calculated climb sizes it for the boostback to that site and the landing burn (RefreshReserve). Idle is normal for part of a flight, because a strap-on stack does not arm until the strap-ons are gone, so the row says why (NextSeparationDropsAllEngines).
    /// </summary>
    private static void DrawBoosterReserve()
    {
        double boostbackDv = PlannedReserveBoostbackDv(out double landingDv);
        string text = PageText.BoosterReserve(BoostbackTarget() != null, _s.ReserveArmed, _s.ReserveStaged, _s.ReserveKg, boostbackDv, landingDv, _s.ReserveNote, out bool warn);
        if (text.Length == 0)
            return;
        GaugeRowText(UiTier.Player, "RESERVE", text, warn ? ConsoleUi.WarnInk : _s.ReserveStaged ? ConsoleStyle.TextMuted : ConsoleStyle.TextPrimary);
    }

    /// <summary>
    /// The stages that can fly themselves home, one row each with its SITE.
    ///  CONTROLLABILITY IS THE TEST, not size: a stage can be flown if the subtree that separates carries a command pod, which is the same thing Vehicle.IsControllable asks. An interstage does not qualify however large it is, and a stage that separates once the climb is over is going orbital speed and is not listed.
    ///  A SITE ON A STAGE ASKS FOR ITS RESERVE: the calculated climb sizes what that booster keeps for the boostback and the landing, which the RESERVE row shows.
    /// </summary>
    private static void DrawReturnableStageRows()
    {
        var list = _s.ReturnableStages;
        if (list.Count == 0)
            return;

        float4 dim = ConsoleStyle.TextMuted;

        for (int i = 0; i < list.Count; i++)
        {
            GuidanceWindow.ReturnableStage stage = list[i];

            // The name row carries the set button, because the button is about the stage rather than about any one of its numbers.
            bool arming = _retargetStageId == stage.RootId;
            string pick = arming ? "CLICK SITE" : "PICK SITE";
            float clearW = stage.HasTarget ? ConsoleWidgets.ButtonWidth("X".AsSpan()) + ImGui.GetStyle().ItemSpacing.X : 0f;
            float buttonsW = ConsoleWidgets.ButtonWidth(pick.AsSpan()) + clearW;
            BeginKitRow(stage.Name.ToUpperInvariant() + (stage.IsNext ? "  NEXT" : ""), controlWidth: buttonsW);
            ConsoleUi.RowRight(buttonsW);
            if (ConsoleWidgets.Button(pick.AsSpan(), $"AfcStageSet{stage.RootId}".AsSpan(), default))
            {
                // Arms the world click for THIS stage. Same click handler the landing retarget uses; the binding is what sends the answer somewhere else.
                _retargetStageId = arming ? 0u : stage.RootId;
                _retargetArmed = !arming;
            }
            if (stage.HasTarget)
            {
                ImGui.SameLine();
                if (ConsoleWidgets.Button("X".AsSpan(), $"AfcStageClear{stage.RootId}".AsSpan(), default))
                {
                    _s.StageTargets.Remove(stage.RootId);
                    _s.StageModelDirty = true;
                }
            }
            ConsoleUi.EndRow();

            GaugeRowText(stage.HasTarget ? "SITE" : "", stage.HasTarget ? Fmt.Sentence(Fmt.LatLon(stage.TargetLatDeg, stage.TargetLonDeg)) : "NO SITE", dim);
        }
    }

    private static void DrawAscentOptionsSection(double bodyRadius)
    {
        if (!GuidanceRegion.Begin("LIVE##ascent", GuidanceRegion.Diagnostics))
            return;

        if (_s.Running && _s.Upfg.Tgo > 0.0)
        {
            GaugeRowText(UiTier.Diagnostics, "SPEED TO GAIN", Fmt.DvWhole(_s.Upfg.VgoMag));
            GaugeRowText(UiTier.Diagnostics, "DISTANCE TO GO", Fmt.Km(_s.Upfg.Rgo.Length() / 1000.0));
        }
        DrawInsertionRows(bodyRadius);

        if (_s.TraceCount > 0)
        {
            ConsoleUi.SectionRow();
            if (ConsoleUi.ButtonRight("CLEAR TRACK", "AfcClearTrack"))
                ResetTrace();
        }

        GuidanceRegion.End();
    }
}
