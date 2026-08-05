using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Type-specific UI controls for maneuver quick-tools. Drawn inline within the
/// Transfer Planning window by DrawPlanWindowPatch.
///
/// - Set Periapsis / Set Apoapsis: altitude input, current orbit info, post-burn orbit
/// - Match Inclination: target matching with AN/DN selection
/// - Set Inclination: arbitrary angle with AN/DN relative to equatorial plane
///
/// Static state is read by DrawPlanWindowPatch to compute the maneuver in the same frame.
/// </summary>
internal static class ManeuverToolsWindow
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    #region Shared State (read by DrawPlanWindowPatch)

    public static double TargetAltitude;
    public static bool UseDescendingNode;
    public static double TargetInclinationRad;
    public static OrbitManeuvers.InclinationReference InclinationRef =
        OrbitManeuvers.InclinationReference.Equatorial;

    #endregion

    private static readonly string[] InclinationRefLabels = { "Ecliptic", "Equatorial" };
    private static readonly string[] NodeLabels = { "ASCENDING", "DESCENDING" };

    /// <summary>Min separation in km between target apsis input and the opposite apsis.</summary>
    private const double MinApseSeparationKm = 1.0;

    #region Internal State

    private static double _inputAltitudeKm;
    private static double _inputInclinationDeg;
    private static bool _defaultsInitialized;
    private static bool _nodeDefaultInitialized;
    private static string? _lastSourceId;

    // The target is held by id, never as a TransferObject: that struct stores only
    // a LookupIndex, and LookupCollection.Deregister swap-removes, so an index kept
    // across frames either resolves to a different body or, once it is past the end
    // of the collection, makes CelestialSystem.GetIndex throw.
    private static string? _selectedTargetId;
    private static readonly List<TransferObject> _targetListBuffer = new();
    private static string? _lastTargetParentId;
    private static bool _setTarget;

    #endregion

    public static void DrawInline(string typeKey, Vehicle source, PlanningBasis basis)
    {
        if (_lastSourceId != source.Id)
        {
            _lastSourceId = source.Id;
            ResetForContextChange();
        }

        if (basis.IsChained)
        {
            ConsoleUi.MutedWrapped("Planning from the trajectory after your last planned burn.");
            ImGui.Spacing();
        }

        if (typeKey == ManeuverTools.KeySetPeriapsis)
            DrawSetApse(source, isSetPeriapsis: true, basis);
        else if (typeKey == ManeuverTools.KeySetApoapsis)
            DrawSetApse(source, isSetPeriapsis: false, basis);
        else if (typeKey == ManeuverTools.KeyMatchInclination)
            DrawMatchInclination(source, basis);
        else if (typeKey == ManeuverTools.KeySetInclination)
            DrawSetInclination(source, basis);
        else if (typeKey == ManeuverTools.KeyStockCircularizeApoapsis)
            DrawCircularize(source, useApoapsis: true, basis);
        else if (typeKey == ManeuverTools.KeyStockCircularizePeriapsis)
            DrawCircularize(source, useApoapsis: false, basis);
    }

    public static Orbit? GetSelectedTargetOrbit() => GetSelectedTargetOrbiter()?.Orbit;

    public static IOrbiter? GetSelectedTargetOrbiter()
        => TargetSelection.Resolve(_selectedTargetId, _lastTargetParentId);

    public static void OnTypeChanged() => ResetForContextChange();
    public static void OnSourceChanged() => ResetForContextChange();

    private static void ResetForContextChange()
    {
        _defaultsInitialized = false;
        _nodeDefaultInitialized = false;
        _lastTargetParentId = null;
    }

    public static void Reset()
    {
        TargetAltitude = 0.0;
        UseDescendingNode = false;
        TargetInclinationRad = 0.0;
        _inputAltitudeKm = 0.0;
        _inputInclinationDeg = 0.0;
        _defaultsInitialized = false;
        _nodeDefaultInitialized = false;
        _lastSourceId = null;
        _selectedTargetId = null;
        _targetListBuffer.Clear();
        _lastTargetParentId = null;
        _setTarget = false;
    }

    #region Set Periapsis / Set Apoapsis

    private static void DrawSetApse(Vehicle source, bool isSetPeriapsis, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        double currentPeAlt = Math.Max(0.0, orbit.Periapsis - parentRadius);
        double currentApAlt = Math.Max(0.0, orbit.Apoapsis - parentRadius);

        if (!_defaultsInitialized)
        {
            _inputAltitudeKm = (isSetPeriapsis ? currentPeAlt : currentApAlt) / 1000.0;
            _defaultsInitialized = true;
        }

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        ConsoleUi.InputDoubleRow(
            isSetPeriapsis ? "TARGET PERIAPSIS (KM)".AsSpan() : "TARGET APOAPSIS (KM)".AsSpan(),
            "##AfcAltInput"u8, ref _inputAltitudeKm, 10.0, 100.0, "%.2f"u8);
        if (_inputAltitudeKm < 0.0)
            _inputAltitudeKm = 0.0;
        TargetAltitude = _inputAltitudeKm * 1000.0;

        ConsoleWidgets.Rule();
        ConsoleWidgets.Readout("CURRENT PERIAPSIS".AsSpan(), FormatDistance(currentPeAlt).AsSpan());
        ConsoleWidgets.Readout("CURRENT APOAPSIS".AsSpan(), FormatDistance(currentApAlt).AsSpan());
        ConsoleWidgets.Readout("BURN LOCATION".AsSpan(),
            (isSetPeriapsis ? "APOAPSIS" : "PERIAPSIS").AsSpan());
        if (ImGui.IsItemHovered())
        {
            ConsoleWidgets.Tooltip(isSetPeriapsis
                ? "Burns at apoapsis change the periapsis on the opposite side of the orbit. This is the most fuel-efficient point to lower or raise your periapsis.".AsSpan()
                : "Burns at periapsis change the apoapsis on the opposite side of the orbit. This is the most fuel-efficient point to lower or raise your apoapsis.".AsSpan());
        }

        bool invalid = isSetPeriapsis
            ? _inputAltitudeKm >= currentApAlt / 1000.0 - MinApseSeparationKm
            : _inputAltitudeKm <= currentPeAlt / 1000.0 + MinApseSeparationKm;

        if (invalid)
        {
            ImGui.Spacing();
            ConsoleUi.Warning(isSetPeriapsis
                ? "Target must be below current apoapsis.".AsSpan()
                : "Target must be above current periapsis.".AsSpan());
            return;
        }

        double newRadius = TargetAltitude + parentRadius;

        if (!isSetPeriapsis)
        {
            double parentSoi = source.Parent?.SphereOfInfluence ?? 0.0;
            if (parentSoi > 0.0 && newRadius > parentSoi)
            {
                ImGui.Spacing();
                ConsoleUi.WarningWrapped(
                    "Target apoapsis is above SOI; vehicle will escape after the burn.");
            }
        }

        if (isSetPeriapsis)
            DrawPostBurnOrbitInfo(orbit.Apoapsis, newRadius, orbit.Mu, parentRadius);
        else
            DrawPostBurnOrbitInfo(newRadius, orbit.Periapsis, orbit.Mu, parentRadius);
    }

    #endregion

    #region Circularize

    private static void DrawCircularize(Vehicle source, bool useApoapsis, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        double currentPeAlt = Math.Max(0.0, orbit.Periapsis - parentRadius);
        double currentApAlt = Math.Max(0.0, orbit.Apoapsis - parentRadius);
        double targetRadius = useApoapsis ? orbit.Apoapsis : orbit.Periapsis;
        double targetAlt = Math.Max(0.0, targetRadius - parentRadius);

        ConsoleWidgets.Readout("CURRENT PERIAPSIS".AsSpan(), FormatDistance(currentPeAlt).AsSpan());
        ConsoleWidgets.Readout("CURRENT APOAPSIS".AsSpan(), FormatDistance(currentApAlt).AsSpan());
        ConsoleWidgets.Readout("BURN LOCATION".AsSpan(),
            (useApoapsis ? "APOAPSIS" : "PERIAPSIS").AsSpan());
        if (ImGui.IsItemHovered())
        {
            ConsoleWidgets.Tooltip(useApoapsis
                ? "Burns at apoapsis raise the periapsis to the apoapsis radius, producing a circular orbit at the current apoapsis altitude.".AsSpan()
                : "Burns at periapsis lower the apoapsis to the periapsis radius, producing a circular orbit at the current periapsis altitude.".AsSpan());
        }
        ConsoleWidgets.Readout("TARGET ALTITUDE".AsSpan(), FormatDistance(targetAlt).AsSpan());

        if (orbit.Eccentricity < 0.001)
        {
            ImGui.Spacing();
            ConsoleUi.Positive("Orbit is already nearly circular.".AsSpan());
            return;
        }

        DrawPostBurnOrbitInfo(targetRadius, targetRadius, orbit.Mu, parentRadius);
    }

    #endregion

    #region Match Inclination

    private static void DrawMatchInclination(Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        SimTime now = basis.Earliest;

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        DrawTargetSelector(source);

        Orbit? targetOrbit = GetSelectedTargetOrbit();
        if (targetOrbit == null)
        {
            ConsoleUi.Muted("Select a target body.".AsSpan());
            return;
        }

        IOrbiter? targetOrbiter = GetSelectedTargetOrbiter();

        bool prevSetTarget = _setTarget;
        if (ConsoleUi.CheckboxRow("SET TARGET".AsSpan(), "AfcMtSetTarget".AsSpan(), ref _setTarget)
            && _setTarget != prevSetTarget)
        {
            QueueTargetChange(source, _setTarget ? targetOrbiter : null);
        }

        double relIncDeg = orbit.GetRelativeInclination(targetOrbit).Value() * (180.0 / Math.PI);
        ConsoleWidgets.Rule();
        ConsoleWidgets.Readout("RELATIVE INCLINATION".AsSpan(),
            string.Format(Inv, "{0:F2} deg", relIncDeg).AsSpan());

        if (relIncDeg < 0.06)
        {
            ConsoleUi.Positive("Orbits are already nearly coplanar.".AsSpan());
            return;
        }

        TrueAnomaly anTa = orbit.GetAscendingNode(targetOrbit);
        TrueAnomaly dnTa = orbit.GetDescendingNode(targetOrbit);
        SimTime anTime = orbit.TimeOfTrueAnomaly(anTa, now);
        SimTime dnTime = orbit.TimeOfTrueAnomaly(dnTa, now);

        var anResult = OrbitManeuvers.ComputeMatchInclination(orbit, targetOrbit, false, now);
        var dnResult = OrbitManeuvers.ComputeMatchInclination(orbit, targetOrbit, true, now);

        DrawNodeSelection(orbit, anTime, dnTime, anResult, dnResult);
    }

    #endregion

    #region Set Inclination

    private static void DrawSetInclination(Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        SimTime now = basis.Earliest;

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        int picked = ConsoleUi.ComboRow("REFERENCE PLANE".AsSpan(), "AfcMtIncRef".AsSpan(),
            (int)InclinationRef, InclinationRefLabels);
        if (picked >= 0 && (OrbitManeuvers.InclinationReference)picked != InclinationRef)
        {
            InclinationRef = (OrbitManeuvers.InclinationReference)picked;
            _defaultsInitialized = false;
        }

        double currentIncDeg = OrbitManeuvers.GetInclinationAgainst(orbit, InclinationRef)
            * (180.0 / Math.PI);

        if (!_defaultsInitialized)
        {
            _inputInclinationDeg = currentIncDeg;
            _defaultsInitialized = true;
        }

        ConsoleUi.InputDoubleRow("TARGET INCLINATION".AsSpan(), "##AfcIncInput"u8,
            ref _inputInclinationDeg, 1.0, 10.0, "%.2f"u8);
        _inputInclinationDeg = Math.Clamp(_inputInclinationDeg, 0.0, 180.0);
        TargetInclinationRad = _inputInclinationDeg * (Math.PI / 180.0);

        ConsoleWidgets.Rule();
        ConsoleWidgets.Readout("CURRENT INCLINATION".AsSpan(),
            string.Format(Inv, "{0:F2} deg", currentIncDeg).AsSpan());

        double incDiff = Math.Abs(_inputInclinationDeg - currentIncDeg);
        if (incDiff < 0.06)
        {
            ConsoleUi.Positive("Already at target inclination.".AsSpan());
            return;
        }

        var (_, _, anTime, dnTime) = OrbitManeuvers.GetReferenceNodes(orbit, now, InclinationRef);

        var anResult = OrbitManeuvers.ComputeSetInclination(
            orbit, TargetInclinationRad, false, now, InclinationRef);
        var dnResult = OrbitManeuvers.ComputeSetInclination(
            orbit, TargetInclinationRad, true, now, InclinationRef);

        DrawNodeSelection(orbit, anTime, dnTime, anResult, dnResult);
    }

    /// <summary>
    /// Draws the AN/DN radio buttons with time, dV, and speed info.
    /// Shared between Match Target and Set Angle modes.
    /// </summary>
    private static void DrawNodeSelection(Orbit orbit,
        SimTime anTime, SimTime dnTime,
        OrbitManeuvers.ManeuverResult? anResult, OrbitManeuvers.ManeuverResult? dnResult)
    {
        SimTime now = Universe.GetElapsedSimTime();
        double timeToAn = anTime.Seconds() - now.Seconds();
        double timeToDn = dnTime.Seconds() - now.Seconds();
        double speedAtAn = orbit.GetStateVectorsAt(anTime).VelocityCci.Length();
        double speedAtDn = orbit.GetStateVectorsAt(dnTime).VelocityCci.Length();
        double dvAn = anResult?.DvCci.Length() ?? 0.0;
        double dvDn = dnResult?.DvCci.Length() ?? 0.0;

        if (!_nodeDefaultInitialized)
        {
            UseDescendingNode = dvDn < dvAn;
            _nodeDefaultInitialized = true;
        }

        // Inside a row, and hover-tested through RowHovered: Segmented ends by
        // rewinding the cursor and emitting a zero-width Dummy, so an
        // IsItemHovered after it can never report the segments as hovered.
        ConsoleWidgets.BeginRow("BURN NODE".AsSpan());
        int picked = ConsoleWidgets.Segmented("AfcMtNode".AsSpan(), NodeLabels,
            UseDescendingNode ? 1 : 0);
        if (ConsoleWidgets.RowHovered)
            ConsoleWidgets.Tooltip(
                "Where the orbit crosses the reference plane: ascending upward, descending downward. Same plane change either way; the cheaper one is the node with the lower orbital speed.".AsSpan());
        ConsoleWidgets.EndRow();
        if (picked >= 0)
            UseDescendingNode = picked == 1;

        // Both nodes stay on screen: picking between them is the whole decision
        // this section supports, and it is made on the dV and the time to burn.
        double timeToSel = UseDescendingNode ? timeToDn : timeToAn;
        double dvSel = UseDescendingNode ? dvDn : dvAn;
        double speedSel = UseDescendingNode ? speedAtDn : speedAtAn;
        double timeToAlt = UseDescendingNode ? timeToAn : timeToDn;
        double dvAlt = UseDescendingNode ? dvAn : dvDn;

        ConsoleWidgets.Readout("TIME TO BURN".AsSpan(),
            FormatHelper.FormatDuration(timeToSel).AsSpan());
        ConsoleWidgets.Readout("REQUIRED DELTA V".AsSpan(),
            string.Format(Inv, "{0:F1} m/s", dvSel).AsSpan());
        ConsoleWidgets.Readout("SPEED AT NODE".AsSpan(),
            string.Format(Inv, "{0:F1} m/s", speedSel).AsSpan());
        ConsoleWidgets.Readout(
            (UseDescendingNode ? "ASCENDING INSTEAD" : "DESCENDING INSTEAD").AsSpan(),
            string.Format(Inv, "{0:F1} m/s in {1}", dvAlt,
                FormatHelper.FormatDuration(timeToAlt)).AsSpan());
    }

    private static void DrawTargetSelector(Vehicle source)
    {
        string? parentId = source.Parent?.Id;
        if (parentId != _lastTargetParentId)
        {
            // After SOI transition the previous selection is in a different
            // context, so re-pick from the fresh list.
            _lastTargetParentId = parentId;
            _selectedTargetId = null;
        }

        TargetSelection.BuildList(source, _targetListBuffer);
        if (TargetSelection.Reconcile(_targetListBuffer, ref _selectedTargetId)
            is not TransferObject reconciled)
        {
            ConsoleUi.Muted("No targets available in the current SOI.".AsSpan());
            return;
        }

        TransferObject selected = reconciled;
        if (ConsoleUi.ComboRow("TARGET".AsSpan(), "AfcMtTarget".AsSpan(), ref selected, _targetListBuffer)
            && selected.GetKey() != reconciled.GetKey())
        {
            _defaultsInitialized = false;
            if (_setTarget && selected.Body is IOrbiter newOrbiter)
                QueueTargetChange(source, newOrbiter);
        }
        _selectedTargetId = selected.GetKey();
    }

    private static void QueueTargetChange(Vehicle source, IOrbiter? target)
    {
        // Stock pattern from DrawPlanWindow's "Set Target" checkbox: mutations
        // from the ImGui pass go through the queue, applied at frame boundary.
        InputEvents.ChangeTargetBuffer.Add(new InputEvents.ChangeTargetData
        {
            Vehicle = source,
            Target = target,
        });
    }

    #endregion

    #region Post-Burn Orbit Info

    private static void DrawPostBurnOrbitInfo(double apRadius, double peRadius, double mu,
        double parentRadius)
    {
        double sma = (apRadius + peRadius) / 2.0;
        if (sma <= 0.0) return;

        double ecc = (apRadius - peRadius) / (apRadius + peRadius);
        double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);

        ConsoleWidgets.RegionHeader("RESULTING ORBIT".AsSpan());
        ConsoleWidgets.Readout("PERIAPSIS".AsSpan(), FormatDistance(peRadius - parentRadius).AsSpan());
        ConsoleWidgets.Readout("APOAPSIS".AsSpan(), FormatDistance(apRadius - parentRadius).AsSpan());
        ConsoleWidgets.Readout("ECCENTRICITY".AsSpan(), string.Format(Inv, "{0:F4}", ecc).AsSpan());
        ConsoleWidgets.Readout("PERIOD".AsSpan(), FormatHelper.FormatDuration(period).AsSpan());
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

    #endregion
}
