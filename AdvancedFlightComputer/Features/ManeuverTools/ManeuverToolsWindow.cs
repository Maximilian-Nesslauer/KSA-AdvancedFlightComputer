using System;
using System.Collections.Generic;
using System.Globalization;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Type-specific UI controls for maneuver quick-tools. Drawn inline within the
/// Transfer Planning window by DrawPlanWindowPatch.
///
/// - Set Periapsis / Set Apoapsis: altitude input, current orbit info, post-burn orbit
/// - Match Inclination: target dropdown, Set Target, AN/DN radio buttons with tooltips
///
/// Static state (TargetAltitude, UseDescendingNode) is read by DrawPlanWindowPatch
/// to compute the maneuver in the same frame.
/// </summary>
static class ManeuverToolsWindow
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    #region Shared State (read by DrawPlanWindowPatch)

    public static double TargetAltitude;
    public static bool UseDescendingNode;

    #endregion

    #region Internal State

    private static double _inputAltitudeKm;
    private static bool _defaultsInitialized;
    private static string? _lastSourceId;

    private static TransferObject _selectedTarget;
    private static List<TransferObject>? _targetList;
    private static bool _setTarget;

    #endregion

    public static void DrawInline(string typeKey, Vehicle source)
    {
        if (_lastSourceId != source.Id)
        {
            _lastSourceId = source.Id;
            _defaultsInitialized = false;
            _targetList = null;
        }

        if (typeKey == ManeuverTools.KeySetPeriapsis)
            DrawSetPeriapsis(source);
        else if (typeKey == ManeuverTools.KeySetApoapsis)
            DrawSetApoapsis(source);
        else if (typeKey == ManeuverTools.KeyMatchInclination)
            DrawMatchInclination(source);
    }

    public static Orbit? GetSelectedTargetOrbit()
    {
        return (_selectedTarget.Body as IOrbiter)?.Orbit;
    }

    public static void OnTypeChanged()
    {
        _defaultsInitialized = false;
        _targetList = null;
    }

    public static void OnSourceChanged()
    {
        _defaultsInitialized = false;
        _targetList = null;
    }

    public static void Reset()
    {
        TargetAltitude = 0.0;
        UseDescendingNode = false;
        _inputAltitudeKm = 0.0;
        _defaultsInitialized = false;
        _lastSourceId = null;
        _selectedTarget = default;
        _targetList = null;
        _setTarget = false;
    }

    #region Set Periapsis

    private static void DrawSetPeriapsis(Vehicle source)
    {
        Orbit orbit = source.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        double currentPeAlt = Math.Max(0.0, orbit.Periapsis - parentRadius);
        double currentApAlt = Math.Max(0.0, orbit.Apoapsis - parentRadius);

        if (!_defaultsInitialized)
        {
            _inputAltitudeKm = currentPeAlt / 1000.0;
            _defaultsInitialized = true;
        }

        if (orbit.Eccentricity >= 1.0)
        {
            ImGui.Text("Requires a bound (elliptical) orbit."u8);
            return;
        }

        DrawAltitudeInput("Target Periapsis (km):");
        TargetAltitude = _inputAltitudeKm * 1000.0;

        ImGui.Spacing();
        ImGuiHelper.DrawTextWidget("Current Periapsis:"u8, FormatDistance(currentPeAlt));
        ImGuiHelper.DrawTextWidget("Current Apoapsis:"u8, FormatDistance(currentApAlt));
        ImGuiHelper.DrawTextWidget("Burn Location:"u8, "Apoapsis");
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip("Burns at apoapsis change the periapsis on the opposite\nside of the orbit. This is the most fuel-efficient point\nto lower or raise your periapsis."u8);

        if (_inputAltitudeKm >= currentApAlt / 1000.0 - 1.0)
        {
            ImGui.Spacing();
            ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 200, 60, 255));
            ImGui.Text("Target must be below current apoapsis."u8);
            ImGui.PopStyleColor();
            return;
        }

        DrawPostBurnOrbitInfo(orbit.Apoapsis, TargetAltitude + parentRadius, orbit.Mu, parentRadius);
    }

    #endregion

    #region Set Apoapsis

    private static void DrawSetApoapsis(Vehicle source)
    {
        Orbit orbit = source.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        double currentPeAlt = Math.Max(0.0, orbit.Periapsis - parentRadius);
        double currentApAlt = Math.Max(0.0, orbit.Apoapsis - parentRadius);

        if (!_defaultsInitialized)
        {
            _inputAltitudeKm = currentApAlt / 1000.0;
            _defaultsInitialized = true;
        }

        if (orbit.Eccentricity >= 1.0)
        {
            ImGui.Text("Requires a bound (elliptical) orbit."u8);
            return;
        }

        DrawAltitudeInput("Target Apoapsis (km):");
        TargetAltitude = _inputAltitudeKm * 1000.0;

        ImGui.Spacing();
        ImGuiHelper.DrawTextWidget("Current Periapsis:"u8, FormatDistance(currentPeAlt));
        ImGuiHelper.DrawTextWidget("Current Apoapsis:"u8, FormatDistance(currentApAlt));
        ImGuiHelper.DrawTextWidget("Burn Location:"u8, "Periapsis");
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip("Burns at periapsis change the apoapsis on the opposite\nside of the orbit. This is the most fuel-efficient point\nto lower or raise your apoapsis."u8);

        if (_inputAltitudeKm <= currentPeAlt / 1000.0 + 1.0)
        {
            ImGui.Spacing();
            ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(255, 200, 60, 255));
            ImGui.Text("Target must be above current periapsis."u8);
            ImGui.PopStyleColor();
            return;
        }

        DrawPostBurnOrbitInfo(TargetAltitude + parentRadius, orbit.Periapsis, orbit.Mu, parentRadius);
    }

    #endregion

    #region Match Inclination

    private static void DrawMatchInclination(Vehicle source)
    {
        Orbit orbit = source.Orbit;
        SimTime now = Universe.GetElapsedSimTime();

        DrawTargetSelector(source);

        Orbit? targetOrbit = GetSelectedTargetOrbit();
        if (targetOrbit == null)
        {
            ImGui.Text("Select a target body."u8);
            return;
        }

        IOrbiter? targetOrbiter = _selectedTarget.Body as IOrbiter;

        ImGui.Spacing();
        ImGuiHelper.BeginColumns(2, new float[] { 0.9f });
        bool prevSetTarget = _setTarget;
        if (ImGuiHelper.DrawCheckbox("Set Target"u8, ref _setTarget, isChanged: false))
        {
            if (_setTarget != prevSetTarget)
            {
                if (_setTarget && targetOrbiter != null)
                    Universe.SetTarget(source, targetOrbiter);
                else
                    Universe.UnsetTarget(source);
            }
        }
        ImGuiHelper.EndColumns();

        double relIncDeg = orbit.GetRelativeInclination(targetOrbit).Value() * (180.0 / Math.PI);
        ImGui.Spacing();
        ImGuiHelper.DrawTextWidget("Relative Inclination:"u8,
            string.Format(Inv, "{0:F2} deg", relIncDeg));

        if (relIncDeg < 0.06)
        {
            ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(80, 220, 80, 255));
            ImGui.Text("Orbits are already nearly coplanar."u8);
            ImGui.PopStyleColor();
            return;
        }

        TrueAnomaly anTa = orbit.GetAscendingNode(targetOrbit);
        TrueAnomaly dnTa = orbit.GetDescendingNode(targetOrbit);
        SimTime anTime = orbit.TimeOfTrueAnomaly(anTa, now);
        SimTime dnTime = orbit.TimeOfTrueAnomaly(dnTa, now);
        double timeToAn = anTime.Seconds() - now.Seconds();
        double timeToDn = dnTime.Seconds() - now.Seconds();

        double speedAtAn = orbit.GetStateVectorsAt(anTime).VelocityCci.Length();
        double speedAtDn = orbit.GetStateVectorsAt(dnTime).VelocityCci.Length();

        var anResult = OrbitManeuvers.ComputeMatchInclination(orbit, targetOrbit, false, now);
        var dnResult = OrbitManeuvers.ComputeMatchInclination(orbit, targetOrbit, true, now);
        double dvAn = anResult?.DvCci.Length() ?? 0.0;
        double dvDn = dnResult?.DvCci.Length() ?? 0.0;

        if (!_defaultsInitialized)
        {
            // Default to whichever node costs less dV (plane changes at higher
            // altitude are cheaper because orbital speed is lower there)
            UseDescendingNode = dvDn < dvAn;
            _defaultsInitialized = true;
        }

        ImGui.Spacing();

        bool useAn = !UseDescendingNode;
        if (ImGui.RadioButton("Ascending Node"u8, useAn))
            UseDescendingNode = false;
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip("The point where your orbit crosses upward through the target's\norbital plane. A normal burn here aligns your orbital plane\nwith the target. Lower speed at the node means cheaper dV."u8);
        ImGui.Indent();
        ImGuiHelper.DrawTextWidget("Time to Burn:"u8, FormatTimeSpan(timeToAn));
        ImGuiHelper.DrawTextWidget("Required Delta V:"u8, string.Format(Inv, "{0:F1} m/s", dvAn));
        ImGuiHelper.DrawTextWidget("Speed at Node:"u8, string.Format(Inv, "{0:F1} m/s", speedAtAn));
        ImGui.Unindent();

        ImGui.Spacing();

        bool useDn = UseDescendingNode;
        if (ImGui.RadioButton("Descending Node"u8, useDn))
            UseDescendingNode = true;
        if (ImGui.IsItemHovered())
            ImGui.SetTooltip("The point where your orbit crosses downward through the target's\norbital plane. Same plane change as ascending node but at the\nopposite side of the orbit. May cost less dV if speed is lower here."u8);
        ImGui.Indent();
        ImGuiHelper.DrawTextWidget("Time to Burn:"u8, FormatTimeSpan(timeToDn));
        ImGuiHelper.DrawTextWidget("Required Delta V:"u8, string.Format(Inv, "{0:F1} m/s", dvDn));
        ImGuiHelper.DrawTextWidget("Speed at Node:"u8, string.Format(Inv, "{0:F1} m/s", speedAtDn));
        ImGui.Unindent();
    }

    private static void DrawTargetSelector(Vehicle source)
    {
        _targetList ??= BuildTargetList(source);

        if (_targetList.Count == 0)
        {
            ImGui.Text("No targets available in current SOI."u8);
            return;
        }

        if (_selectedTarget.GetKey() == "N/A" && _targetList.Count > 0)
            _selectedTarget = _targetList[0];

        TransferObject prevTarget = _selectedTarget;
        if (ImGuiHelper.DrawCombo("Target:"u8, ref _selectedTarget, _targetList)
            && _selectedTarget.GetKey() != prevTarget.GetKey())
        {
            _defaultsInitialized = false;
            if (_setTarget && _selectedTarget.Body is IOrbiter newOrbiter)
                Universe.SetTarget(source, newOrbiter);
        }
    }

    private static List<TransferObject> BuildTargetList(Vehicle source)
    {
        var list = new List<TransferObject>();
        IParentBody? parent = source.Parent;
        if (parent == null || Universe.CurrentSystem == null)
            return list;

        foreach (Astronomical astro in Universe.CurrentSystem.All.GetList())
        {
            if (astro == source) continue;
            if (astro is StellarBody) continue;
            if ((astro as IOrbiter)?.Orbit == null) continue;

            if ((astro is Celestial celestial && celestial.Parent?.Id == parent.Id)
                || (astro is Vehicle vehicle && vehicle.Parent?.Id == parent.Id))
            {
                list.Add(new TransferObject(astro));
            }
        }

        return list;
    }

    #endregion

    #region Post-Burn Orbit Info

    /// <summary>
    /// Shows the resulting orbital parameters after the burn.
    /// Used by Set Pe/Ap to display new orbit characteristics.
    /// </summary>
    private static void DrawPostBurnOrbitInfo(double apRadius, double peRadius, double mu,
        double parentRadius)
    {
        double sma = (apRadius + peRadius) / 2.0;
        if (sma <= 0.0) return;

        double ecc = (apRadius - peRadius) / (apRadius + peRadius);
        double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);

        ImGui.Spacing();
        ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyleColorVec4(ImGuiCol.TextDisabled));
        ImGui.Text("Resulting Orbit:"u8);
        ImGuiHelper.DrawTextWidget("  Periapsis:"u8, FormatDistance(peRadius - parentRadius));
        ImGuiHelper.DrawTextWidget("  Apoapsis:"u8, FormatDistance(apRadius - parentRadius));
        ImGuiHelper.DrawTextWidget("  Eccentricity:"u8, string.Format(Inv, "{0:F4}", ecc));
        ImGuiHelper.DrawTextWidget("  Period:"u8, FormatTimeSpan(period));
        ImGui.PopStyleColor();
    }

    #endregion

    #region UI Helpers

    private static void DrawAltitudeInput(string label)
    {
        ImGui.Text(label);
        ImGui.SameLine(220f);
        ImGui.PushItemWidth(-1f);
        ImGui.InputDouble("##altInput"u8, ref _inputAltitudeKm, 10.0, 100.0,
            default(ImString), ImGuiInputTextFlags.CharsDecimal);
        if (_inputAltitudeKm < 0.0)
            _inputAltitudeKm = 0.0;
        ImGui.PopItemWidth();
    }

    #endregion

    #region Formatting

    internal static string FormatDistance(double meters)
    {
        if (double.IsNaN(meters) || double.IsInfinity(meters))
            return "N/A";
        if (meters >= 1e9)
            return string.Format(Inv, "{0:F1} Gm", meters / 1e9);
        if (meters >= 1e6)
            return string.Format(Inv, "{0:F1} Mm", meters / 1e6);
        if (meters >= 1000.0)
            return string.Format(Inv, "{0:F1} km", meters / 1000.0);
        return string.Format(Inv, "{0:F0} m", meters);
    }

    internal static string FormatTimeSpan(double seconds)
    {
        if (double.IsNaN(seconds) || double.IsInfinity(seconds))
            return "N/A";
        if (seconds < 0)
            return "past";
        if (seconds < 60.0)
            return string.Format(Inv, "{0:F0}s", seconds);
        if (seconds < 3600.0)
        {
            int m = (int)(seconds / 60.0);
            int s = (int)(seconds % 60.0);
            return s > 0 ? $"{m}m {s}s" : $"{m}m";
        }
        if (seconds < 86400.0)
        {
            int h = (int)(seconds / 3600.0);
            int min = (int)((seconds % 3600.0) / 60.0);
            return min > 0 ? $"{h}h {min}m" : $"{h}h";
        }
        int d = (int)(seconds / 86400.0);
        int hr = (int)((seconds % 86400.0) / 3600.0);
        return hr > 0 ? $"{d}d {hr}h" : $"{d}d";
    }

    #endregion
}
