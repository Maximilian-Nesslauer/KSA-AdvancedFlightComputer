using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// The inputs and readouts of the MANEUVER page form, one set per orbit tool. The input values are
/// shared state, because OrbitToolsPanel computes the maneuver from them and MultiPassController and
/// MultiPassPreviewCache plan the passes from them. The player form holds the inputs; the orbit now, the burn and the resulting
/// orbit are collected while the form draws and shown in the PARAMETERS fold through DetailParts.
/// </summary>
internal static class OrbitToolForm
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    #region Shared State (read by OrbitToolsPanel)

    public static double TargetAltitude;
    public static bool UseDescendingNode;
    public static double TargetInclinationRad;
    public static OrbitManeuvers.InclinationReference InclinationRef =
        OrbitManeuvers.InclinationReference.Ecliptic;

    #endregion

    // Enum order. The names are swapped, see OrbitManeuvers.GetReferenceNormalCci.
    private static readonly string[] InclinationRefLabels = { "EQUATORIAL", "ECLIPTIC" };
    private static readonly string[] NodeLabels = { "ASCENDING", "DESCENDING" };

    private const double MinApseSeparationKm = 1.0;

    #region Internal State

    private static double _inputAltitudeKm;
    private static double _inputInclinationDeg;
    private static bool _defaultsInitialized;
    private static bool _nodeDefaultInitialized;
    private static string? _lastSourceId;
    private static SeedKey? _seedKey;
    private static bool _recheckSeed;

    // Store target identity by id because LookupCollection.Deregister can move lookup indices.
    private static string? _selectedTargetId;
    private static readonly List<TransferObject> _targetListBuffer = new();
    private static string? _lastTargetParentId;
    private static bool _setTarget;

    #endregion

    // The number of planned burns is part of the key because a burn that was added or flown changes the orbit the tool plans from.
    private readonly record struct SeedKey(SeedPhase Phase, string Body, bool Chained, int Burns);

    private readonly record struct DetailRow(string Group, string Label, string Value, string Tooltip);

    public const string BurnAtLabel = "BURN AT";

    private const string GroupNow = "ORBIT NOW";
    private const string GroupBurn = "BURN";
    private const string GroupResult = "RESULTING ORBIT";
    private static readonly string[] DetailGroups = { GroupNow, GroupBurn, GroupResult };

    private static readonly List<DetailRow> _detail = new();

    private static void Detail(string group, string label, string value, string tooltip = "")
        => _detail.Add(new DetailRow(group, label, value, tooltip));

    public static FoldPart[] DetailParts()
    {
        var parts = new List<FoldPart>(DetailGroups.Length);
        foreach (string group in DetailGroups)
        {
            int rows = 0;
            foreach (DetailRow row in _detail)
                if (row.Group == group)
                    rows++;
            if (rows > 0)
                parts.Add(new FoldPart(group, true, () => DrawDetailGroup(group, rows)));
        }
        return parts.ToArray();
    }

    private static void DrawDetailGroup(string group, int rows)
    {
        if (rows > 1)
            ConsoleUi.SectionHead(group);
        foreach (DetailRow row in _detail)
        {
            if (row.Group != group)
                continue;
            ConsoleUi.Readout(row.Label.AsSpan(), row.Value.AsSpan());
            if (row.Tooltip.Length > 0 && ImGui.IsItemHovered())
                ConsoleUi.Tooltip(row.Tooltip.AsSpan());
        }
    }

    /// <summary>Why the tool has no maneuver this frame, in the words of the footer, or empty. The form sets it while it draws.</summary>
    public static string Problem { get; private set; } = "";

    public static double CurrentInclinationDeg { get; private set; }

    public static void DrawInline(string typeKey, Vehicle source, PlanningBasis basis, OrbitManeuvers.ManeuverResult? maneuver)
    {
        if (_lastSourceId != source.Id)
        {
            _lastSourceId = source.Id;
            ResetForContextChange();
        }
        Problem = "";
        _detail.Clear();

        if (maneuver is OrbitManeuvers.ManeuverResult m)
        {
            Detail(GroupBurn, "DELTA-V", FormatSpeed(m.DvCci.Length()));
            Detail(GroupBurn, "IN", FormatHelper.FormatDuration(Math.Max(0.0, (m.BurnTime - Universe.GetElapsedTime()).Seconds())));
        }

        if (basis.IsChained)
            ConsoleUi.MutedWrapped("Plans from after your last burn.");

        if (typeKey == ManeuverTools.KeySetPeriapsis)
            DrawSetApse(source, isSetPeriapsis: true, basis);
        else if (typeKey == ManeuverTools.KeySetApoapsis)
            DrawSetApse(source, isSetPeriapsis: false, basis);
        else if (typeKey == ManeuverTools.KeyMatchInclination)
            DrawMatchInclination(source, basis);
        else if (typeKey == ManeuverTools.KeySetInclination)
            DrawSetInclination(source, basis);
        else if (typeKey == ManeuverTools.KeyCircularizeApoapsis)
            DrawCircularize(source, useApoapsis: true, basis);
        else if (typeKey == ManeuverTools.KeyCircularizePeriapsis)
            DrawCircularize(source, useApoapsis: false, basis);
    }

    /// <summary>
    /// Seeds the defaults of a new tool, craft or situation and writes the shared input state, before the maneuver
    /// is computed, so the picture, the burn bars and the footer of the first frame use the new inputs
    /// and not the previous tool's. The form draw after it edits the same values.
    /// </summary>
    public static void ApplyInputs(string typeKey, Vehicle source, PlanningBasis basis, CraftSituation where)
    {
        if (_lastSourceId != source.Id)
        {
            _lastSourceId = source.Id;
            ResetForContextChange();
        }

        SeedPhase phase = ManeuverSeed.PhaseOf(where);
        if (phase == SeedPhase.Unknown && _seedKey is SeedKey last)
            phase = last.Phase;
        var seedKey = new SeedKey(phase, source.Parent?.Id ?? "", basis.IsChained, source.FlightComputer.BurnPlan.BurnCount);
        if (seedKey != _seedKey)
        {
            _seedKey = seedKey;
            _defaultsInitialized = false;
            _nodeDefaultInitialized = false;
        }
        bool recheck = _recheckSeed;
        _recheckSeed = false;

        Orbit orbit = basis.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        bool setPe = typeKey == ManeuverTools.KeySetPeriapsis;
        if (setPe || typeKey == ManeuverTools.KeySetApoapsis)
        {
            double peKm = AltitudeKm(orbit.Periapsis, parentRadius);
            double apKm = AltitudeKm(orbit.Apoapsis, parentRadius);
            if (recheck && (!ManeuverSeed.ApseFits(setPe, _inputAltitudeKm, peKm, apKm, MinApseSeparationKm) || ManeuverSeed.AtApse(setPe, _inputAltitudeKm, peKm, apKm)))
                _defaultsInitialized = false;
            if (!_defaultsInitialized)
            {
                _inputAltitudeKm = setPe
                    ? ManeuverSeed.PeriapsisKm(peKm, apKm, MinApseSeparationKm)
                    : ManeuverSeed.ApoapsisKm(peKm, apKm, MinApseSeparationKm);
                _defaultsInitialized = true;
            }
            if (_inputAltitudeKm < 0.0)
                _inputAltitudeKm = 0.0;
            TargetAltitude = _inputAltitudeKm * 1000.0;
        }
        else if (typeKey == ManeuverTools.KeySetInclination && orbit.Eccentricity < 1.0)
        {
            double currentDeg = OrbitManeuvers.GetInclinationAgainst(orbit, InclinationRef) * (180.0 / Math.PI);
            if (recheck && ManeuverSeed.AtInclination(_inputInclinationDeg, currentDeg))
                _defaultsInitialized = false;
            if (!_defaultsInitialized)
            {
                _inputInclinationDeg = ManeuverSeed.InclinationDeg(currentDeg);
                _defaultsInitialized = true;
            }
            _inputInclinationDeg = Math.Clamp(_inputInclinationDeg, 0.0, 180.0);
            TargetInclinationRad = _inputInclinationDeg * (Math.PI / 180.0);
        }
    }

    // An open path has no apoapsis, so its altitude reads as 0 and the form refuses it by the eccentricity.
    private static double AltitudeKm(double radius, double parentRadius)
        => double.IsFinite(radius) ? Math.Max(0.0, radius - parentRadius) / 1000.0 : 0.0;

    public static void RecheckSeed() => _recheckSeed = true;

    public static void ClearFrame()
    {
        Problem = "";
        _detail.Clear();
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
        _seedKey = null;
        _recheckSeed = false;
        _selectedTargetId = null;
        _targetListBuffer.Clear();
        _lastTargetParentId = null;
        _setTarget = false;
        Problem = "";
    }

    #region Set Periapsis / Set Apoapsis

    private static void DrawSetApse(Vehicle source, bool isSetPeriapsis, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        double currentPeAlt = Math.Max(0.0, orbit.Periapsis - parentRadius);
        double currentApAlt = Math.Max(0.0, orbit.Apoapsis - parentRadius);

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        ConsoleUi.InputDoubleRow(
            isSetPeriapsis ? "NEW PERIAPSIS".AsSpan() : "NEW APOAPSIS".AsSpan(),
            "##AfcAltInput"u8, ref _inputAltitudeKm, 10.0, 100.0, "%.1f"u8, unit: "km");
        if (_inputAltitudeKm < 0.0)
            _inputAltitudeKm = 0.0;
        TargetAltitude = _inputAltitudeKm * 1000.0;

        DetailOrbitNow(orbit, currentPeAlt, currentApAlt);
        Detail(GroupBurn, "BURN LOCATION", isSetPeriapsis ? "Apoapsis" : "Periapsis", isSetPeriapsis
            ? "Burns at apoapsis change the periapsis on the opposite side of the orbit. This is the most fuel-efficient point to lower or raise your periapsis."
            : "Burns at periapsis change the apoapsis on the opposite side of the orbit. This is the most fuel-efficient point to lower or raise your apoapsis.");

        if (!ManeuverSeed.ApseFits(isSetPeriapsis, _inputAltitudeKm, currentPeAlt / 1000.0, currentApAlt / 1000.0, MinApseSeparationKm))
        {
            Problem = Ui.Logic.PageText.ApseMargin(isSetPeriapsis, MinApseSeparationKm);
            return;
        }
        if (ManeuverSeed.AtApse(isSetPeriapsis, _inputAltitudeKm, currentPeAlt / 1000.0, currentApAlt / 1000.0))
        {
            Problem = Ui.Logic.PageText.AtTargetApse(isSetPeriapsis);
            return;
        }

        double newRadius = TargetAltitude + parentRadius;

        if (!isSetPeriapsis)
        {
            double parentSoi = source.Parent?.SphereOfInfluence ?? 0.0;
            if (parentSoi > 0.0 && newRadius > parentSoi)
                ConsoleUi.WarningWrapped(
                    "This apoapsis is past the reach of this body, so the craft escapes after the burn.");
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

        DetailOrbitNow(orbit, currentPeAlt, currentApAlt);

        if (orbit.Eccentricity < OrbitToolCatalog.CircularEccentricity)
        {
            Problem = Ui.Logic.PageText.AlreadyCircular;
            return;
        }

        DrawPostBurnOrbitInfo(targetRadius, targetRadius, orbit.Mu, parentRadius);
    }

    #endregion

    #region Match Inclination

    private static void DrawMatchInclination(Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        UniverseTime now = basis.Earliest;

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        DrawTargetSelector(source);

        Orbit? targetOrbit = GetSelectedTargetOrbit();
        if (targetOrbit == null)
        {
            ConsoleUi.Muted("Pick a craft or a body to match.".AsSpan());
            if (Problem.Length == 0)
                Problem = "Pick what to match";
            return;
        }

        DetailOrbitNow(orbit, Math.Max(0.0, orbit.Periapsis - (source.Parent?.MeanRadius ?? 0.0)), Math.Max(0.0, orbit.Apoapsis - (source.Parent?.MeanRadius ?? 0.0)));

        double relIncDeg = orbit.GetRelativeInclination(targetOrbit).Value() * (180.0 / Math.PI);
        ConsoleUi.Readout("RELATIVE INCLINATION".AsSpan(),
            Ui.Logic.Fmt.Deg(relIncDeg).AsSpan());

        if (relIncDeg < ManeuverSeed.AtTargetDeg)
        {
            Problem = Ui.Logic.PageText.AlreadyCoplanar;
            return;
        }

        TrueAnomaly anTa = orbit.GetAscendingNode(targetOrbit);
        TrueAnomaly dnTa = orbit.GetDescendingNode(targetOrbit);
        UniverseTime? anTime = orbit.TimeOfTrueAnomaly(anTa, now);
        UniverseTime? dnTime = orbit.TimeOfTrueAnomaly(dnTa, now);

        var anResult = OrbitManeuvers.ComputeMatchInclination(orbit, targetOrbit, false, now);
        var dnResult = OrbitManeuvers.ComputeMatchInclination(orbit, targetOrbit, true, now);

        DrawNodeSelection(orbit, anTime, dnTime, anResult, dnResult);
    }

    #endregion

    #region Set Inclination

    private static void DrawSetInclination(Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        UniverseTime now = basis.Earliest;

        if (orbit.Eccentricity >= 1.0)
        {
            ConsoleUi.Muted("Requires a bound (elliptical) orbit.".AsSpan());
            return;
        }

        int picked = ConsoleUi.ComboRow((Ui.Console.AfcConsole.HasPanes ? "PLANE" : "REFERENCE PLANE").AsSpan(), "AfcMtIncRef".AsSpan(),
            (int)InclinationRef, InclinationRefLabels);
        if (picked >= 0 && (OrbitManeuvers.InclinationReference)picked != InclinationRef)
        {
            InclinationRef = (OrbitManeuvers.InclinationReference)picked;
            _defaultsInitialized = false;
        }

        double currentIncDeg = OrbitManeuvers.GetInclinationAgainst(orbit, InclinationRef)
            * (180.0 / Math.PI);
        CurrentInclinationDeg = currentIncDeg;

        if (!_defaultsInitialized)
        {
            _inputInclinationDeg = ManeuverSeed.InclinationDeg(currentIncDeg);
            _defaultsInitialized = true;
        }

        ConsoleUi.InputDoubleRow("NEW INCLINATION".AsSpan(), "##AfcIncInput"u8,
            ref _inputInclinationDeg, 1.0, 10.0, "%.1f"u8, unit: "deg");
        _inputInclinationDeg = Math.Clamp(_inputInclinationDeg, 0.0, 180.0);
        TargetInclinationRad = _inputInclinationDeg * (Math.PI / 180.0);

        DetailOrbitNow(orbit, Math.Max(0.0, orbit.Periapsis - (source.Parent?.MeanRadius ?? 0.0)), Math.Max(0.0, orbit.Apoapsis - (source.Parent?.MeanRadius ?? 0.0)));
        Detail(GroupNow, "INCLINATION", Ui.Logic.Fmt.Deg(currentIncDeg));

        if (ManeuverSeed.AtInclination(_inputInclinationDeg, currentIncDeg))
        {
            Problem = Ui.Logic.PageText.AtTargetInclination;
            return;
        }

        var (_, _, anTime, dnTime) = OrbitManeuvers.GetReferenceNodes(orbit, now, InclinationRef);

        var anResult = OrbitManeuvers.ComputeSetInclination(
            orbit, TargetInclinationRad, false, now, InclinationRef);
        var dnResult = OrbitManeuvers.ComputeSetInclination(
            orbit, TargetInclinationRad, true, now, InclinationRef);

        DrawNodeSelection(orbit, anTime, dnTime, anResult, dnResult);
    }

    private static void DrawNodeSelection(Orbit orbit,
        UniverseTime? anTime, UniverseTime? dnTime,
        OrbitManeuvers.ManeuverResult? anResult, OrbitManeuvers.ManeuverResult? dnResult)
    {
        // Unreachable nodes use NaN in readouts and cannot win a cost comparison.
        UniverseTime now = Universe.GetElapsedTime();
        double timeToAn = anTime.HasValue ? (anTime.Value - now).Seconds() : double.NaN;
        double timeToDn = dnTime.HasValue ? (dnTime.Value - now).Seconds() : double.NaN;
        double speedAtAn = anTime.HasValue
            ? orbit.GetStateVectorsAt(anTime.Value).VelocityCci.Length() : double.NaN;
        double speedAtDn = dnTime.HasValue
            ? orbit.GetStateVectorsAt(dnTime.Value).VelocityCci.Length() : double.NaN;
        double dvAn = anResult?.DvCci.Length() ?? double.NaN;
        double dvDn = dnResult?.DvCci.Length() ?? double.NaN;

        if (!_nodeDefaultInitialized)
        {
            UseDescendingNode = dvDn < dvAn || double.IsNaN(dvAn);
            _nodeDefaultInitialized = true;
        }

        string[] priced = { Ui.Logic.PageText.NodeChoice(false, dvAn, timeToAn), Ui.Logic.PageText.NodeChoice(true, dvDn, timeToDn) };
        bool fits = ConsoleUi.ChoiceRowFits(BurnAtLabel, priced);
        int picked = ConsoleUi.ChoiceRow(BurnAtLabel, "AfcMtNode", fits ? priced : NodeLabels, UseDescendingNode ? 1 : 0,
            description: fits ? "" : Ui.Logic.PageText.NodeLine(dvAn, timeToAn, dvDn, timeToDn),
            labelTip: "Where the orbit crosses the reference plane: ascending upward, descending downward. Same plane change either way; the cheaper one is the node with the lower orbital speed.", feedsPicture: true);
        if (picked >= 0)
            UseDescendingNode = picked == 1;

        Detail(GroupBurn, "SPEED AT NODE", FormatSpeed(UseDescendingNode ? speedAtDn : speedAtAn));
    }

    private static void DetailOrbitNow(Orbit orbit, double peAltM, double apAltM)
    {
        Detail(GroupNow, "PERIAPSIS", FormatDistance(peAltM));
        Detail(GroupNow, "APOAPSIS", FormatDistance(apAltM));
        Detail(GroupNow, "PERIOD", FormatHelper.FormatDuration(orbit.Period));
    }

    private static string FormatSpeed(double metersPerSecond)
        => double.IsNaN(metersPerSecond)
            ? "N/A"
            : Ui.Logic.Fmt.Dv(metersPerSecond);

    private static void DrawTargetSelector(Vehicle source)
    {
        string? parentId = source.Parent?.Id;
        if (parentId != _lastTargetParentId)
        {
            _lastTargetParentId = parentId;
            _selectedTargetId = null;
        }
        // The pick starts on the craft's own stock target when that target is in the list.
        _selectedTargetId ??= (source.Target as Astronomical)?.Id;

        TargetSelection.BuildList(source, _targetListBuffer);
        if (TargetSelection.Reconcile(_targetListBuffer, ref _selectedTargetId)
            is not TransferObject reconciled)
        {
            ConsoleUi.Muted("Nothing to match around this body.".AsSpan());
            Problem = "Nothing to match here";
            return;
        }

        TransferObject selected = reconciled;
        if (ConsoleUi.ComboRow("MATCH WITH".AsSpan(), "AfcMtTarget".AsSpan(), ref selected, _targetListBuffer)
            && selected.GetKey() != reconciled.GetKey())
        {
            _defaultsInitialized = false;
            if (_setTarget && selected.Body is IOrbiter newOrbiter)
                QueueTargetChange(source, newOrbiter);
        }
        _selectedTargetId = selected.GetKey();
    }

    /// <summary>
    /// SET AS CRAFT TARGET of the MATCH tool, drawn in the GNC fold. On, it makes the picked
    /// target the craft's stock target and keeps it so when the pick changes. Off clears the stock target.
    /// </summary>
    public static void DrawSetTargetRow(Vehicle source)
    {
        IOrbiter? targetOrbiter = GetSelectedTargetOrbiter();
        using (new ImGuiDisabledScope(targetOrbiter == null))
        {
            bool prevSetTarget = _setTarget;
            if (ConsoleUi.CheckboxRow("SET AS CRAFT TARGET".AsSpan(), "AfcMtSetTarget".AsSpan(), ref _setTarget,
                    "Makes the target picked here the craft's own stock target, as the stock target picker does.".AsSpan(), stockChip: true)
                && _setTarget != prevSetTarget)
            {
                QueueTargetChange(source, _setTarget ? targetOrbiter : null);
            }
        }
    }

    public static bool SetsCraftTarget => _setTarget;

    private static void QueueTargetChange(Vehicle source, IOrbiter? target)
    {
        // Queue target changes as TransferPlanner.DrawPlanWindow does because workers remain active while the UI runs.
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

        Detail(GroupResult, "PERIAPSIS", FormatDistance(peRadius - parentRadius));
        Detail(GroupResult, "APOAPSIS", FormatDistance(apRadius - parentRadius));
        Detail(GroupResult, "ECCENTRICITY", string.Format(Inv, "{0:F4}", ecc));
        Detail(GroupResult, "PERIOD", FormatHelper.FormatDuration(period));
    }

    #endregion

    #region Formatting

    internal static string FormatDistance(double meters)
    {
        if (double.IsNaN(meters) || double.IsInfinity(meters))
            return "N/A";
        return Ui.Logic.Fmt.Distance(meters);
    }

    #endregion
}
