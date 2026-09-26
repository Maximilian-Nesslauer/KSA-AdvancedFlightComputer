using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MissionPlanner;

/// <summary>
/// The MISSION PLANNER window, opened from the AFC Guidance menu. For now it plans one thing: a launch from the home body to meet one of its moons at a chosen arrival (see <see cref="LunarLaunchPlanner"/>).
/// The player picks the arrival and one of the parking orbit plane's two angles, and the other follows, so the plane always contains the moon at arrival.
/// SEND TO ASCENT hands the plane and the parking orbit to the ascent as its target, and EXECUTE there arms for the window quoted here.
///  A console window like the guidance panel, drawn from the loader's after-GUI hook. Everything it holds is the plan's inputs, so a save load resets it.
/// </summary>
internal static class MissionPlannerWindow
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private const double Day = 86400.0;
    private const double DefaultParkingKm = 200.0;

    /// <summary>How far the default inclination sits above the least that reaches both the site and the moon, deg. Exactly at it the plane only grazes the site's latitude.</summary>
    private const double DefaultInclinationMarginDeg = 0.5;

    private static float WidthPx => 440f * ImGuiHelper.InterfaceScale;
    private static float HeightPx => 720f * ImGuiHelper.InterfaceScale;

    /// <summary>Whether the window is drawn. The menu entry and the window's close button write it.</summary>
    internal static bool Visible;

    // The plan's inputs. Seeded the first time the window draws for a body with a moon, and again for another body or moon.
    private static string _homeId = "";
    private static string _moonId = "";
    private static double _arrivalTime = double.NaN;
    private static PlaneControl _control = PlaneControl.Inclination;
    private static double _incDeg = double.NaN;
    private static double _lanDeg;
    private static bool _southbound;
    private static bool _planeChosen;
    private static double _parkingPeKm = DefaultParkingKm;
    private static double _parkingApKm = DefaultParkingKm;

    private static readonly List<Celestial> _moons = new();
    private static readonly List<string> _moonNames = new();

    internal static void Reset()
    {
        _homeId = "";
        _moonId = "";
        _arrivalTime = double.NaN;
        _control = PlaneControl.Inclination;
        _incDeg = double.NaN;
        _lanDeg = 0.0;
        _southbound = false;
        _planeChosen = false;
        _parkingPeKm = DefaultParkingKm;
        _parkingApKm = DefaultParkingKm;
        _moons.Clear();
        _moonNames.Clear();
    }

    /// <summary>The loader's after-GUI hook. A fault costs this frame's window and one log line per kind, never the rest of the GUI.</summary>
    internal static void DrawGui()
    {
        try
        {
            Draw();
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"mission-planner-draw:{ex.GetType().Name}", $"[AFC] Mission planner draw failed: {ex}");
        }
    }

    private static void Draw()
    {
        if (!Visible || Program.IsEditorOpen)
            return;

        // ConsoleStyle.BeginWindow closes the ImGui window itself when it returns false. The size applies on first use only, so a resize sticks for the session.
        bool open = true;
        if (!ConsoleStyle.BeginWindow("afc-mission-planner".AsSpan(), "MISSION PLANNER".AsSpan(), "AFC-MPL".AsSpan(),
                ref open, new float2(WidthPx, HeightPx), ImGuiWindowFlags.None, ImGuiCond.FirstUseEver,
                pinToMainViewport: false))
            return;

        try
        {
            if (!open)
            {
                Visible = false;
                return;
            }

            Vehicle? vehicle = Program.ControlledVehicle;
            Celestial? moon = null;
            LunarLaunchPlan? plan = null;
            ConsoleStyle.BeginBody();
            ConsoleStyle.PushWidgetStyle();
            try
            {
                plan = DrawBody(vehicle, out moon);
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
                ConsoleStyle.EndBody();
            }

            ConsoleStyle.BeginFooter();
            try
            {
                DrawFooter(vehicle, moon, plan);
            }
            finally
            {
                ConsoleStyle.EndFooter();
            }
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    private static LunarLaunchPlan? DrawBody(Vehicle? vehicle, out Celestial? moon)
    {
        moon = null;
        IParentBody? home = vehicle?.Orbit?.Parent;
        if (vehicle == null || home == null)
        {
            ConsoleUi.MutedWrapped("No vehicle to plan for. Take control of one on the ground.");
            return null;
        }

        CollectMoons(home);
        if (_moons.Count == 0)
        {
            ConsoleUi.MutedWrapped($"Nothing orbits {home.Id} to plan a transfer to.");
            return null;
        }
        if (home.Id != _homeId)
        {
            _homeId = home.Id;
            SeedFor(DefaultMoon());
        }
        moon = _moons.Find(m => m.Id == _moonId) ?? DefaultMoon();
        if (moon.Id != _moonId)
            SeedFor(moon);

        DrawLaunchSection(vehicle, ref moon);
        DrawParkingSection();

        // After the two sections that can change them: the destination reseeds, and the parking orbit moves the soonest arrival.
        double now = Universe.GetElapsedSeconds();
        double earliest = LunarLaunchPlanner.EarliestArrival(moon, home.Mu,
            home.MeanRadius + ParkingLowKm * 1000.0, now);
        if (double.IsNaN(_arrivalTime))
            _arrivalTime = earliest;
        if (double.IsNaN(_incDeg))
            _incDeg = DefaultInclination(vehicle, moon);

        DrawArrivalSection(moon, now, earliest);

        LunarLaunchPlan plan = Solve(vehicle, moon, now);
        // The first plan of a seed takes whichever of the inclination's two planes has the sooner window. After that the choice is the player's.
        if (!_planeChosen && plan.HasPlane && _control == PlaneControl.Inclination)
        {
            _planeChosen = true;
            bool southSooner = plan.WaitSouthbound < plan.WaitNorthbound
                               || (double.IsNaN(plan.WaitNorthbound) && !double.IsNaN(plan.WaitSouthbound));
            if (southSooner != _southbound)
            {
                _southbound = southSooner;
                plan = Solve(vehicle, moon, now);
            }
        }

        if (DrawPlaneSection(vehicle, moon, plan, now))
            plan = Solve(vehicle, moon, now);

        DrawWindowSection(plan, now);

        string refusal = GuidanceWindow.PlaneTargetRefusal(vehicle);
        if (refusal.Length > 0)
        {
            ConsoleWidgets.Rule();
            ConsoleUi.WarningWrapped(refusal);
        }
        return plan;
    }

    // --- Launch -------------------------------------------------------------

    private static void DrawLaunchSection(Vehicle vehicle, ref Celestial moon)
    {
        ConsoleWidgets.RegionHeader("LAUNCH".AsSpan());
        ConsoleWidgets.Readout("VEHICLE".AsSpan(), vehicle.Id.AsSpan());
        double3 site = vehicle.Orbit.StateVectors.PositionCci;
        double latDeg = Math.Asin(Math.Clamp(site.Z / site.Length(), -1.0, 1.0)) * (180.0 / Math.PI);
        ConsoleWidgets.Readout("SITE LATITUDE".AsSpan(),
            string.Format(Inv, "{0:F2} deg {1}", Math.Abs(latDeg), latDeg >= 0.0 ? "N" : "S").AsSpan());
        if (!vehicle.Situation.HasAnyContact())
            ConsoleUi.MutedWrapped("Not on the ground: the windows are for where it is now.");

        int active = _moons.IndexOf(moon);
        int picked = ConsoleUi.ComboRow("DESTINATION".AsSpan(), "afcmp-moon".AsSpan(), active, _moonNames);
        if (picked >= 0 && picked != active)
        {
            moon = _moons[picked];
            SeedFor(moon);
        }
    }

    // --- Parking orbit ------------------------------------------------------

    // The lower of the two is the periapsis whichever field it was typed in, so a pair entered the other way round, or half typed, is never planned or sent back to front.
    private static double ParkingLowKm => Math.Min(_parkingPeKm, _parkingApKm);
    private static double ParkingHighKm => Math.Max(_parkingPeKm, _parkingApKm);

    private static void DrawParkingSection()
    {
        ConsoleWidgets.RegionHeader("PARKING ORBIT".AsSpan());
        ConsoleUi.InputDoubleRow("PERIAPSIS (KM)".AsSpan(), "##afcmp-pe"u8, ref _parkingPeKm, 10.0, 100.0, "%.1f"u8);
        ConsoleUi.InputDoubleRow("APOAPSIS (KM)".AsSpan(), "##afcmp-ap"u8, ref _parkingApKm, 10.0, 100.0, "%.1f"u8);
        _parkingPeKm = Math.Max(_parkingPeKm, 0.0);
        _parkingApKm = Math.Max(_parkingApKm, 0.0);
    }

    // --- Arrival ------------------------------------------------------------

    private static void DrawArrivalSection(Celestial moon, double now, double earliest)
    {
        ConsoleWidgets.RegionHeader("ARRIVAL".AsSpan());
        ConsoleWidgets.Readout("ARRIVAL (UT)".AsSpan(), FormatUt(_arrivalTime).AsSpan());
        ConsoleWidgets.Readout("ARRIVES IN".AsSpan(), FormatSpan(_arrivalTime - now).AsSpan());

        // Coarse, over one of the moon's orbits, which is every position it can be met in. The buttons below fine-tune.
        float span = (float)Math.Max(moon.Orbit.Period, Day);
        float later = (float)Math.Clamp(_arrivalTime - earliest, 0.0, span);
        ConsoleWidgets.BeginRow("AFTER SOONEST".AsSpan());
        bool hovered = ConsoleWidgets.RowHovered;
        if (ConsoleWidgets.SliderFloat("afcmp-later".AsSpan(), ref later, 0f, span, FormatSpan(later).AsSpan(), pending: false))
            _arrivalTime = earliest + later;
        ConsoleWidgets.EndRow();
        if (hovered)
            ConsoleWidgets.Tooltip("How much later than a transfer injected now.".AsSpan());

        DrawNudgeButtons(earliest);

        if (_arrivalTime < earliest)
            ConsoleUi.WarningWrapped($"Sooner than a transfer from now can arrive, at {FormatUt(earliest)}. Move it later, or SOONEST.");
    }

    private static readonly (string Label, double Seconds)[] Nudges =
    {
        ("-1D", -Day), ("-1H", -3600.0), ("-10M", -600.0), ("+10M", 600.0), ("+1H", 3600.0), ("+1D", Day),
    };

    private static void DrawNudgeButtons(double earliest)
    {
        float spacing = ImGui.GetStyle().ItemSpacing.X;
        float height = ConsoleWidgets.ButtonHeight;
        float soonestWidth = ConsoleWidgets.ButtonWidth("SOONEST".AsSpan());
        float width = MathF.Floor((ConsoleStyle.ContentAvailWidth() - soonestWidth - spacing * Nudges.Length) / Nudges.Length);
        for (int i = 0; i < Nudges.Length; i++)
        {
            if (ConsoleWidgets.Button(Nudges[i].Label.AsSpan(), ("afcmp-nudge" + i).AsSpan(), new float2(width, height)))
                _arrivalTime += Nudges[i].Seconds;
            ImGui.SameLine();
        }
        if (ConsoleWidgets.Button("SOONEST".AsSpan(), "afcmp-soonest".AsSpan(), new float2(soonestWidth, height)))
            _arrivalTime = earliest;
        if (ImGui.IsItemHovered())
            ConsoleWidgets.Tooltip("Arrive as soon as a transfer injected now can.".AsSpan());
    }

    // --- Plane --------------------------------------------------------------

    private static readonly string[] ControlSegments = { "SET INCLINATION", "SET LAN" };

    /// <summary>The plane's rows. True when an input changed, so the plan is solved again before it is read.</summary>
    private static bool DrawPlaneSection(Vehicle vehicle, Celestial moon, LunarLaunchPlan plan, double now)
    {
        ConsoleWidgets.RegionHeader("PARKING ORBIT PLANE".AsSpan());
        bool changed = false;

        int picked = ConsoleWidgets.Segmented("afcmp-control".AsSpan(), ControlSegments.AsSpan(),
            _control == PlaneControl.Inclination ? 0 : 1);
        if (picked >= 0 && (PlaneControl)picked != _control)
        {
            _control = (PlaneControl)picked;
            // Setting the inclination again: of its two planes, keep the one flown now. Read before solving, which writes the node.
            double lanBefore = _lanDeg;
            if (_control == PlaneControl.Inclination)
                SelectPlaneNearest(Solve(vehicle, moon, now), lanBefore);
            changed = true;
        }

        using (new ImGuiDisabledScope(_control != PlaneControl.Inclination))
        {
            if (ConsoleUi.InputDoubleRow("INCLINATION (DEG)".AsSpan(), "##afcmp-inc"u8, ref _incDeg, 0.1, 1.0, "%.2f"u8))
                changed = true;
            _incDeg = Math.Clamp(_incDeg, 0.0, 180.0);
        }
        using (new ImGuiDisabledScope(_control != PlaneControl.Node))
        {
            if (ConsoleUi.InputDoubleRow("LAN (DEG)".AsSpan(), "##afcmp-lan"u8, ref _lanDeg, 0.1, 1.0, "%.2f"u8))
                changed = true;
            _lanDeg = WrapDeg(_lanDeg);
        }

        // An inclination is met by two planes: pick which, by its window.
        if (_control == PlaneControl.Inclination && plan.HasPlane)
        {
            string[] planes =
            {
                PlaneChoice(plan.LanNorthboundDeg, plan.WaitNorthbound),
                PlaneChoice(plan.LanSouthboundDeg, plan.WaitSouthbound),
            };
            float2 min = ImGui.GetCursorScreenPos();
            float2 max = min + new float2(ConsoleStyle.ContentAvailWidth(), ConsoleStyle.FrameHeightPx);
            int plane = ConsoleWidgets.Segmented("afcmp-plane".AsSpan(), planes.AsSpan(), _southbound ? 1 : 0);
            if (plane >= 0 && (plane == 1) != _southbound)
            {
                _southbound = plane == 1;
                changed = true;
            }
            if (ImGui.IsWindowHovered() && ImGui.IsMouseHoveringRect(min, max))
                ConsoleWidgets.Tooltip(($"Two planes at this inclination contain {moon.Id} at arrival.\n"
                    + "The first meets it heading north, the second heading south.").AsSpan());
        }

        float buttonWidth = ConsoleStyle.ContentAvailWidth();
        if (ConsoleWidgets.Button("LAUNCH NOW".AsSpan(), "afcmp-launchnow".AsSpan(), new float2(buttonWidth, ConsoleWidgets.ButtonHeight)))
        {
            if (LunarLaunchPlanner.TryLaunchNowPlane(vehicle, moon, _arrivalTime, out double inc, out double lan))
            {
                _lanDeg = WrapDeg(lan);
                if (_control == PlaneControl.Inclination)
                {
                    _incDeg = inc;
                    SelectPlaneNearest(Solve(vehicle, moon, now), lan);
                }
                changed = true;
            }
        }
        if (ImGui.IsItemHovered())
            ConsoleWidgets.Tooltip($"The plane through the site and {moon.Id} at arrival, so the window is now.".AsSpan());
        return changed;
    }

    private static string PlaneChoice(double lanDeg, double wait)
        => string.Format(Inv, "LAN {0:F1}, {1}", lanDeg, double.IsFinite(wait) ? "T-" + FormatSpan(wait) : "NO WINDOW");

    private static void SelectPlaneNearest(LunarLaunchPlan plan, double lanDeg)
    {
        if (!plan.HasPlane || double.IsNaN(plan.LanNorthboundDeg))
            return;
        _southbound = AngleApart(plan.LanSouthboundDeg, lanDeg) < AngleApart(plan.LanNorthboundDeg, lanDeg);
        _planeChosen = true;
    }

    // --- Launch window ------------------------------------------------------

    private static void DrawWindowSection(LunarLaunchPlan plan, double now)
    {
        ConsoleWidgets.RegionHeader("TRANSFER".AsSpan());
        ConsoleWidgets.Readout("HOHMANN TRANSFER".AsSpan(), FormatSpan(plan.TransferTime).AsSpan());
        ConsoleWidgets.Readout("INJECTION (UT)".AsSpan(), FormatUt(plan.InjectionTime).AsSpan());
        ConsoleWidgets.Readout("MOON AT ARRIVAL".AsSpan(), string.Format(Inv, "{0:N0} km, dec {1:F2} deg",
            plan.MoonDistance / 1000.0, plan.MoonDeclinationDeg).AsSpan());
        ConsoleWidgets.Readout("MOON MOVES".AsSpan(), string.Format(Inv, "{0:F1} deg, {1:N0} km",
            plan.MoonTravelDeg, plan.MoonTravelDeg * (Math.PI / 180.0) * plan.MoonDistance / 1000.0).AsSpan());
        if (ImGui.IsItemHovered())
            ConsoleWidgets.Tooltip("How far it travels while the transfer flies: the injection aims at where it will be.".AsSpan());

        ConsoleWidgets.RegionHeader("LAUNCH WINDOW".AsSpan());
        if (plan.Retrograde)
            ConsoleUi.WarningWrapped($"Retrograde, a westward launch. The same plane flown eastward has its node at LAN {WrapDeg(plan.LanDeg + 180.0):F2}.");
        if (plan.Problem.Length > 0)
        {
            ConsoleUi.DangerWrapped(plan.Problem);
            return;
        }
        if (!double.IsFinite(plan.WaitSec))
        {
            ConsoleUi.MutedWrapped("The body does not turn, so the site never comes under a plane it is not in.");
            return;
        }

        double ignition = now + plan.WaitSec;
        ConsoleWidgets.Readout("NEXT WINDOW".AsSpan(), ("T-" + FormatSpan(plan.WaitSec)).AsSpan());
        ConsoleWidgets.Readout("IGNITION (UT)".AsSpan(), FormatUt(ignition).AsSpan());
        ConsoleWidgets.Readout("HEADING".AsSpan(), string.Format(Inv, "{0:F1} deg, {1} crossing",
            WrapDeg(plan.AzimuthDeg), plan.Descending ? "descending" : "ascending").AsSpan());

        if (plan.InjectionTime < now)
        {
            ConsoleUi.DangerWrapped($"The injection would have been {FormatSpan(now - plan.InjectionTime)} ago. Arrive later.");
            return;
        }
        if (plan.CoastSec < 0.0)
        {
            ConsoleUi.DangerWrapped($"The injection comes {FormatSpan(-plan.CoastSec)} before this window reaches orbit. Arrive later.");
            return;
        }
        ConsoleWidgets.Readout("PARKING COAST".AsSpan(), string.Format(Inv, "about {0}, {1:F1} orbits",
            FormatSpan(plan.CoastSec), plan.CoastSec / plan.ParkingPeriod).AsSpan());
        if (ImGui.IsItemHovered())
            ConsoleWidgets.Tooltip(string.Format(Inv, "From orbit insertion, about {0:F0} min after ignition, to the injection burn.",
                LunarLaunchPlanner.InsertionAfterIgnitionS / 60.0).AsSpan());
    }

    // --- Footer -------------------------------------------------------------

    /// <summary>SEND TO ASCENT, and whether the ascent already has this plan. Why a send would be refused is said in the body, where there is room for it.</summary>
    private static void DrawFooter(Vehicle? vehicle, Celestial? moon, LunarLaunchPlan? plan)
    {
        bool ready = vehicle != null && moon != null && plan is { HasPlane: true, SiteReachesPlane: true }
                     && GuidanceWindow.PlaneTargetRefusal(vehicle).Length == 0;

        if (vehicle != null && plan is { HasPlane: true }
            && VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState state) && state.PlaneTarget != null)
        {
            bool current = Matches(state.PlaneTarget, plan);
            ConsoleStyle.FooterStatus((current ? "The ascent has this plan." : "The ascent has an earlier plan.").AsSpan(),
                pending: !current);
        }

        ConsoleStyle.FooterRightAlign(ConsoleWidgets.ButtonWidth("SEND TO ASCENT".AsSpan()));
        using (new ImGuiDisabledScope(!ready))
        {
            if (ConsoleWidgets.PrimaryButton("SEND TO ASCENT".AsSpan()) && ready)
            {
                var target = new AscentPlaneTarget(plan!.IncDeg, plan.LanDeg, ParkingLowKm, ParkingHighKm,
                    $"{moon!.Id}, arrive {FormatUt(_arrivalTime)}");
                GuidanceWindow.TrySendPlaneTarget(vehicle!, target, out _);
            }
        }
    }

    private static bool Matches(AscentPlaneTarget sent, LunarLaunchPlan plan)
        => Math.Abs(sent.IncDeg - plan.IncDeg) < 1e-6 && AngleApart(sent.LanDeg, plan.LanDeg) < 1e-6
           && Math.Abs(sent.PeKm - ParkingLowKm) < 1e-6 && Math.Abs(sent.ApKm - ParkingHighKm) < 1e-6;

    // --- Helpers ------------------------------------------------------------

    private static LunarLaunchPlan Solve(Vehicle vehicle, Celestial moon, double now)
    {
        LunarLaunchPlan plan = LunarLaunchPlanner.Solve(vehicle, moon, now,
            new LunarLaunchInputs(_arrivalTime, _control, _incDeg, _lanDeg, _southbound, ParkingLowKm, ParkingHighKm));
        // The angle not set follows the one that is.
        if (plan.HasPlane)
        {
            if (_control == PlaneControl.Inclination)
                _lanDeg = WrapDeg(plan.LanDeg);
            else
                _incDeg = plan.IncDeg;
        }
        return plan;
    }

    private static void CollectMoons(IParentBody home)
    {
        _moons.Clear();
        _moonNames.Clear();
        foreach (IOrbiter child in home.Children)
        {
            if (child is Celestial moon && moon.Orbit != null)
            {
                _moons.Add(moon);
                _moonNames.Add(moon.Id);
            }
        }
    }

    // The first with a sphere of influence to arrive in.
    private static Celestial DefaultMoon()
        => _moons.Find(m => m is IParentBody body && body.SphereOfInfluence > 0.0) ?? _moons[0];

    private static void SeedFor(Celestial moon)
    {
        _moonId = moon.Id;
        _arrivalTime = double.NaN;
        _incDeg = double.NaN;
        _planeChosen = false;
    }

    /// <summary>
    /// Just above the least inclination that serves: the site's latitude, for the most easterly launch the site can make, unless the moon's declination at arrival asks for more.
    /// </summary>
    private static double DefaultInclination(Vehicle vehicle, Celestial moon)
    {
        double3 site = vehicle.Orbit.StateVectors.PositionCci;
        double lat = Math.Abs(Math.Asin(Math.Clamp(site.Z / site.Length(), -1.0, 1.0)));
        double3 moonAt = LunarLaunchPlanner.MoonAt(moon, _arrivalTime);
        double dec = Math.Abs(LunarTransferGeometry.Declination(moonAt / moonAt.Length()));
        double least = Math.Max(lat, dec) * (180.0 / Math.PI) + DefaultInclinationMarginDeg;
        return Math.Min(Math.Ceiling(least * 10.0) / 10.0, 90.0);
    }

    private static double WrapDeg(double deg)
    {
        deg %= 360.0;
        return deg < 0.0 ? deg + 360.0 : deg;
    }

    private static double AngleApart(double aDeg, double bDeg)
    {
        double d = WrapDeg(aDeg - bDeg);
        return Math.Min(d, 360.0 - d);
    }

    /// <summary>Sim time as the game's own clock shows it: years, day of the year, and the time of day.</summary>
    internal static string FormatUt(double seconds)
    {
        if (!double.IsFinite(seconds))
            return "N/A";
        long whole = (long)Math.Floor(seconds);
        long years = whole / 31536000L;
        long days = whole / 86400L % 365L;
        long hours = whole / 3600L % 24L;
        long minutes = whole / 60L % 60L;
        long secs = whole % 60L;
        return string.Format(Inv, "Y{0} D{1:000} {2:00}:{3:00}:{4:00}", years, days, hours, minutes, secs);
    }

    /// <summary>A duration to the minute past a day, and to the second under an hour.</summary>
    internal static string FormatSpan(double seconds)
    {
        if (!double.IsFinite(seconds))
            return "N/A";
        string sign = seconds < 0.0 ? "-" : "";
        long s = (long)Math.Round(Math.Abs(seconds));
        long d = s / 86400L, h = s / 3600L % 24L, m = s / 60L % 60L, sec = s % 60L;
        if (d > 0)
            return string.Format(Inv, "{0}{1}d {2:00}h {3:00}m", sign, d, h, m);
        if (h > 0)
            return string.Format(Inv, "{0}{1}h {2:00}m {3:00}s", sign, h, m, sec);
        return m > 0
            ? string.Format(Inv, "{0}{1}m {2:00}s", sign, m, sec)
            : string.Format(Inv, "{0}{1}s", sign, sec);
    }
}
