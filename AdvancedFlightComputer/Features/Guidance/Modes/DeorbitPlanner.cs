using System;
using System.Collections.Generic;
using System.Diagnostics;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

internal readonly record struct DeorbitSettings(double Latitude, double Longitude, double BrakingAltitude,
    double GateAltitude, double GateUprange, double SinkRate, double DownrangeFactor, double? ArrivalDescentDeg = null)
{
    internal bool IsValid => double.IsFinite(Latitude) && Math.Abs(Latitude) <= 90
        && double.IsFinite(Longitude) && Math.Abs(Longitude) <= 180
        && double.IsFinite(BrakingAltitude) && BrakingAltitude > GateAltitude
        && double.IsFinite(GateAltitude) && GateAltitude > 0
        && double.IsFinite(GateUprange) && GateUprange >= 0
        && double.IsFinite(SinkRate) && SinkRate > 0
        && double.IsFinite(DownrangeFactor) && DownrangeFactor >= 1
        && (!ArrivalDescentDeg.HasValue || (double.IsFinite(ArrivalDescentDeg.Value)
            && ArrivalDescentDeg.Value >= 0 && ArrivalDescentDeg.Value <= 30));
}

internal readonly record struct DeorbitEngine(double Throttle, double Thrust, double MassFlow);

internal sealed record DeorbitPlan(double DepartureTime, double ArrivalTime, double3 DeparturePosition,
    double3 DepartureVelocity, double3 DeltaV, double3 ArrivalPosition, double3 ArrivalVelocity,
    double BrakingDistance, double Clearance, double CutoffMass);

internal sealed record DirectBrakingPlan(double IgnitionTime, double3 Position, double3 Velocity,
    double Downrange, string Refusal);

// All epochs and vectors belong to the captured parent body, even when another vehicle is stepped.
internal sealed class DeorbitRequest
{
    internal readonly Orbit Source;
    internal readonly IParentBody Parent;
    internal readonly double Epoch;
    internal readonly double Mass;
    internal readonly double VehicleRadius;
    internal readonly double MinimumPulse;
    internal readonly double ControlStep;
    internal readonly double GLimit;
    // Stock Auto needs this long before ignition to turn the craft onto the node.
    internal readonly double StockPreparation;
    internal readonly DeorbitSettings Settings;
    internal readonly UpfgVehicle Model;
    // Sorted by throttle. The planner uses the full-throttle entry, and the search log reports both ends.
    internal readonly DeorbitEngine[] Engines;
    internal readonly double SiteHeight;
    internal readonly double3 SiteCcf;
    internal double GateRadius => Parent.MeanRadius + SiteHeight + Settings.GateAltitude;
    internal double BrakingRadius => Parent.MeanRadius + SiteHeight + Settings.BrakingAltitude;
    internal double ExhaustVelocity => Engines.Length == 0 ? double.NaN : Engines[^1].Thrust / Engines[^1].MassFlow;

    internal DeorbitRequest(Orbit source, double epoch, double mass, double vehicleRadius,
        DeorbitSettings settings, UpfgVehicle model, DeorbitEngine[] engines, double minimumPulse, double controlStep, double gLimit = 0,
        double stockPreparation = DeorbitPlanner.PrepSeconds)
    {
        Parent = source.Parent;
        StateVectors state = source.GetStateVectorsAt(new UniverseTime(epoch));
        Source = Orbit.CreateFromStateCci(Parent, new UniverseTime(epoch), state.PositionCci,
            state.VelocityCci, source.OrbitLineColor);
        Epoch = epoch;
        Mass = mass;
        VehicleRadius = vehicleRadius;
        Settings = settings;
        Model = CopyModel(model);
        Engines = engines;
        MinimumPulse = minimumPulse;
        ControlStep = controlStep;
        GLimit = gLimit;
        StockPreparation = double.IsFinite(stockPreparation) ? Math.Max(stockPreparation, DeorbitPlanner.PrepSeconds) : DeorbitPlanner.PrepSeconds;
        SiteCcf = GuidanceWindow.SiteDirCcf(settings.Latitude, settings.Longitude);
        SiteHeight = Terrain(SiteCcf);
    }

    internal double3 SiteAt(double time) => SiteCcf.Transform(Parent.GetCcf2Cci(new UniverseTime(time)));

    internal double3 GateAt(double time, double3 normal) =>
        GuidanceWindow.RotateAbout(SiteAt(time), normal, -Settings.GateUprange / Parent.MeanRadius) * GateRadius;

    internal double Terrain(double3 directionCcf) => Parent is Celestial celestial
        ? celestial.GetTerrainHeightFromDirCcf(directionCcf) : double.NaN;

    internal double Clearance(double3 position, double time)
    {
        double radius = position.Length();
        if (!(radius > 0) || !DeorbitPlanner.Finite(position)) return double.NaN;
        double terrain = Terrain((position / radius).Transform(Parent.GetCci2Ccf(new UniverseTime(time))));
        double ocean = Parent.GetOceanReference() == null ? double.NegativeInfinity : 0;
        return radius - Parent.MeanRadius - Math.Max(terrain, ocean) - VehicleRadius;
    }

    internal UpfgVehicle ModelAt(double mass)
    {
        UpfgVehicle result = CopyModel(Model);
        if (result.Stages.Count > 0) result.Stages[0].MassTotal = mass;
        if (GLimit > 0) GuidanceWindow.ApplyGLimit(result, GLimit);
        return result;
    }

    private static UpfgVehicle CopyModel(UpfgVehicle source)
    {
        var copy = new UpfgVehicle();
        foreach (UpfgStage stage in source.Stages)
            copy.Stages.Add(new UpfgStage { Mode = stage.Mode, Thrust = stage.Thrust, Isp = stage.Isp,
                MassTotal = stage.MassTotal, MassDry = stage.MassDry, GLim = stage.GLim,
                Seq = stage.Seq, Engines = stage.Engines });
        return copy;
    }
}

// The search is an iterator that yields after each Lambert solve, UPFG step or terrain sample, so Step can stop it on the simulation thread's time budget.
// A nested search returns its value through a Result holder, because an iterator cannot have an out parameter.
internal sealed class DeorbitPlanner : IDisposable
{
    internal const double ClearanceMargin = 1000;
    internal const double PrepSeconds = 30;
    internal const int CandidateLimit = 8;
    internal const double WorkLimitMilliseconds = 500;
    internal const double ArrivalAngleToleranceDeg = 1;
    // Two propagations of one state are treated as the same state within these bounds.
    internal const double PositionToleranceM = 100;
    internal const double VelocityToleranceMs = 0.1;
    private const double BadCost = 1e12;
    private const int DepartureSamples = 16;
    private const int FlightSamples = 24;
    private const int PredictionIterations = 400;
    private const int BrakingFitIterations = 12;
    private const double BrakingFitToleranceM = 100;
    private const double RefineToleranceSeconds = 1;
    // Planning runs for a few simulation seconds before stock can load the node.
    private const double PlanningAllowanceSeconds = 10;
    private const int ApproachSamplesPerOrbit = 720;
    private const int ApproachOrbits = 5;
    // The transfer must arrive between this far below and this far above the local horizon.
    private const double MaximumArrivalDescentDeg = 10;
    private const double MaximumArrivalClimbDeg = 0.5;
    private const int ArrivalAngleFitIterations = 8;
    // Direct braking is refused when the craft sits above this slope over the gate, or when UPFG steers further below the horizon.
    private const double DirectApproachSlopeDeg = 30;
    private const double LowestBrakingPitchDeg = -30;

    private readonly DeorbitRequest _request;
    private readonly IEnumerator<bool> _search;
    private readonly bool _brakingOnly;
    private readonly DeorbitPlan? _arrivalHint;
    private double _firstDeparture;
    private double _lastDeparture;
    private double _departureStep;
    private double _shortestFlight;
    private double _longestFlight;
    private double _flightStep;
    private double _seedDistance;

    internal DeorbitPlan? Plan { get; private set; }
    internal DirectBrakingPlan? Direct { get; private set; }
    internal bool Complete { get; private set; }
    internal string Status { get; private set; } = "Planning the approach.";
    internal string DirectRefusal { get; private set; } = "";
    internal int Evaluations { get; private set; }
    internal double WorkMilliseconds { get; private set; }
    internal int CandidatesChecked { get; private set; }
    internal int RefinementSamples { get; private set; }

    internal DeorbitPlanner(DeorbitRequest request, bool brakingOnly = false, DeorbitPlan? arrivalHint = null)
    {
        _request = request;
        _brakingOnly = brakingOnly;
        _arrivalHint = arrivalHint;
        _search = Search().GetEnumerator();
    }

    internal void Step(double budgetMilliseconds = 3)
    {
        if (Complete) return;
        long start = Stopwatch.GetTimestamp();
        try
        {
            do
            {
                if (WorkMilliseconds + Stopwatch.GetElapsedTime(start).TotalMilliseconds >= WorkLimitMilliseconds)
                {
                    Status = Plan != null ? "Stock deorbit node ready. Using the checked plan."
                        : "The planning work limit ended before a safe approach was found. Change the site or braking altitude.";
                    // An accepted direct approach that was still being checked becomes a refusal. An earlier refusal keeps its reason.
                    if (Direct is { Refusal.Length: 0 }) Direct = Direct with { Refusal = Status };
                    Complete = true;
                    _search.Dispose();
                    return;
                }
                if (!_search.MoveNext()) { Complete = true; return; }
            } while (Stopwatch.GetElapsedTime(start).TotalMilliseconds < budgetMilliseconds);
        }
        // A failed iterator cannot continue, so the refusal names the failure instead of the last progress status.
        catch (Exception error)
        {
            Status = "Deorbit planning failed: " + error.Message;
            if (Direct is { Refusal.Length: 0 }) Direct = Direct with { Refusal = Status };
            Plan = null;
            Complete = true;
            _search.Dispose();
        }
        finally { WorkMilliseconds += Stopwatch.GetElapsedTime(start).TotalMilliseconds; }
    }

    public void Dispose() => _search.Dispose();

    private IEnumerable<bool> Search()
    {
        DeorbitRequest request = _request;
        if (!request.Settings.IsValid || !double.IsFinite(request.SiteHeight)
            || !(request.Source.Period > 0) || !double.IsFinite(request.Source.Period)
            || request.Source.Eccentricity >= 1 || !(request.Mass > 0) || request.Model.Stages.Count == 0)
        {
            Status = "The approach needs a finite bound orbit, terrain, and an engine model.";
            yield break;
        }
        foreach (bool step in DirectSteps()) yield return step;
        if (Direct is { Refusal.Length: 0 })
        {
            Status = "Direct braking approach ready.";
            yield break;
        }
        yield return true;
        if (_brakingOnly)
        {
            Status = "The actual orbit has no safe braking approach. " + DirectRefusal;
            yield break;
        }
        if (request.Engines.Length == 0)
        {
            Status = DirectRefusal + " A deorbit burn needs active, supplied liquid engines.";
            yield break;
        }
        foreach (bool step in TransferSteps()) yield return step;
    }

    // Sets Direct and DirectRefusal. An accepted direct approach has also passed the flown-arc check.
    private IEnumerable<bool> DirectSteps()
    {
        var found = new Result<DirectBrakingPlan?>();
        foreach (bool step in FindDirectSteps(_request, found, _arrivalHint)) yield return step;
        Direct = found.Value;
        DirectRefusal = Direct?.Refusal ?? $"No approaching braking point was found within {ApproachOrbits} orbits.";
        if (Direct is not { Refusal.Length: 0 }) yield break;
        // After the node, the measured orbit decides braking. Execution error must not demand another node.
        if (!_brakingOnly && _request.Settings.ArrivalDescentDeg.HasValue
            && !MatchesArrivalAngle(_request.Settings, Direct.Position, Direct.Velocity))
        {
            RefuseDirect("The current orbit misses the selected arrival angle. A deorbit transfer is required.");
            yield break;
        }
        var arc = new DeorbitArcCheck(_request, _request.Source, _request.Epoch, Direct.IgnitionTime);
        while (arc.Advance()) yield return true;
        if (arc.Refusal.Length > 0) RefuseDirect(arc.Refusal);
    }

    private void RefuseDirect(string reason)
    {
        DirectRefusal = reason;
        Direct = Direct! with { Refusal = reason };
    }

    // Scans departures and flight times within the next source orbit, then fully checks the cheapest candidates.
    private IEnumerable<bool> TransferSteps()
    {
        double period = _request.Source.Period;
        _firstDeparture = _request.Epoch + 2 * PrepSeconds;
        _lastDeparture = _request.Epoch + period;
        _shortestFlight = 2 * PrepSeconds + 1;
        _longestFlight = 0.9 * period;
        if (_lastDeparture <= _firstDeparture || _longestFlight <= _shortestFlight)
        {
            Status = "The next orbit has no transfer window with enough preparation time.";
            yield break;
        }
        _departureStep = (_lastDeparture - _firstDeparture) / DepartureSamples;
        _flightStep = (_longestFlight - _shortestFlight) / (FlightSamples - 1);

        // A circular orbit at the braking altitude over the site gives the first braking distance for every endpoint.
        double3 seedPosition = _request.SiteAt(_firstDeparture) * _request.BrakingRadius;
        double3 seedVelocity = double3.Normalize(double3.Cross(_request.Source.GetOrbitNormalCci(), seedPosition))
            * Math.Sqrt(_request.Parent.Mu / _request.BrakingRadius);
        var seed = new Result<double>();
        foreach (bool step in PredictBrakingDistanceSteps(_request, seedPosition, seedVelocity, _request.Mass, seed)) yield return step;
        _seedDistance = seed.Value;
        if (!(_seedDistance > 0))
        {
            Status = "The braking prediction did not converge at the selected altitude.";
            yield break;
        }

        var candidates = new List<DeorbitPlan>();
        foreach (bool step in ScanSteps(candidates)) yield return step;
        candidates.Sort((a, b) => Cost(a).CompareTo(Cost(b)));
        foreach (DeorbitPlan candidate in candidates.Take(CandidateLimit))
        {
            CandidatesChecked++;
            var checkedPlan = new Result<DeorbitPlan?>();
            foreach (bool step in CheckTransferSteps(candidate.DepartureTime, candidate.ArrivalTime - candidate.DepartureTime, checkedPlan))
                yield return step;
            if (checkedPlan.Value is not DeorbitPlan plan) continue;
            Plan = plan;
            foreach (bool step in RefineSteps(plan)) yield return step;
            Status = "Stock deorbit node ready.";
            yield break;
        }
        Status = candidates.Count > CandidateLimit
            ? $"The {CandidateLimit}-candidate limit ended before a safe approach was found. Change the site or braking altitude."
            : "Deorbit refused. No safe transfer was found in the next orbit. Change the site, braking altitude or arrival angle.";
    }

    // A narrow selected angle band can fall between coarse flight times, so every interval that brackets it is fitted as well.
    private IEnumerable<bool> ScanSteps(List<DeorbitPlan> candidates)
    {
        double? selectedAngle = _request.Settings.ArrivalDescentDeg;
        int total = (DepartureSamples + 1) * FlightSamples;
        for (int i = 0; i <= DepartureSamples; i++)
        {
            double departure = _firstDeparture + i * _departureStep;
            DeorbitPlan? previous = null;
            for (int j = 1; j <= FlightSamples; j++)
            {
                double flight = _shortestFlight + (j - 1) * _flightStep;
                DeorbitPlan? candidate = Transfer(departure, flight, _seedDistance, checkAngle: !selectedAngle.HasValue);
                if (candidate != null)
                {
                    if (MatchesArrivalAngle(_request.Settings, candidate.ArrivalPosition, candidate.ArrivalVelocity)
                        && FitsStockBurn(candidate))
                        candidates.Add(candidate);
                    if (previous != null && selectedAngle is double angle
                        && AngleError(previous, angle) * AngleError(candidate, angle) < 0)
                    {
                        var fitted = new Result<DeorbitPlan?>();
                        foreach (bool step in FitArrivalAngleSteps(previous, candidate, angle, fitted)) yield return step;
                        if (fitted.Value != null && FitsStockBurn(fitted.Value)) candidates.Add(fitted.Value);
                    }
                }
                previous = candidate;
                Status = $"Searching deorbit transfers ({i * FlightSamples + j}/{total}).";
                yield return true;
            }
        }
    }

    private IEnumerable<bool> FitArrivalAngleSteps(DeorbitPlan lower, DeorbitPlan upper, double descentAngle,
        Result<DeorbitPlan?> result)
    {
        result.Value = null;
        double departure = lower.DepartureTime;
        double lo = lower.ArrivalTime - departure, hi = upper.ArrivalTime - departure;
        double loError = AngleError(lower, descentAngle);
        for (int i = 0; i < ArrivalAngleFitIterations; i++)
        {
            double flight = (lo + hi) / 2;
            DeorbitPlan? sample = Transfer(departure, flight, _seedDistance, checkAngle: false);
            if (sample == null) yield break;
            yield return true;
            double error = AngleError(sample, descentAngle);
            if (Math.Abs(error) <= ArrivalAngleToleranceDeg
                && (result.Value == null || Math.Abs(error) < Math.Abs(AngleError(result.Value, descentAngle))))
                result.Value = sample;
            if (Math.Abs(error) <= ArrivalAngleToleranceDeg * 0.25) yield break;
            if (loError * error < 0) hi = flight;
            else { lo = flight; loError = error; }
        }
    }

    private static double AngleError(DeorbitPlan plan, double descentAngle) => ArrivalAngleDegrees(plan) + descentAngle;

    // The full check fits the braking point, then checks braking, stock propagation, and terrain along both flown arcs.
    private IEnumerable<bool> CheckTransferSteps(double departure, double flight, Result<DeorbitPlan?> result)
    {
        result.Value = null;
        var fitted = new Result<DeorbitPlan?>();
        foreach (bool step in FitBrakingPointSteps(departure, flight, fitted)) yield return step;
        if (fitted.Value is not DeorbitPlan candidate) yield break;
        Status = $"Checking departure clearance ({CandidatesChecked}/{CandidateLimit}).";
        var sourceArc = new DeorbitArcCheck(_request, _request.Source, _request.Epoch, candidate.DepartureTime);
        while (sourceArc.Advance()) yield return true;
        if (sourceArc.Refusal.Length > 0) yield break;
        Orbit transfer = Orbit.CreateFromStateCci(_request.Parent, new UniverseTime(candidate.DepartureTime),
            candidate.DeparturePosition, candidate.DepartureVelocity, _request.Source.OrbitLineColor);
        Status = $"Checking transfer clearance ({CandidatesChecked}/{CandidateLimit}).";
        var transferArc = new DeorbitArcCheck(_request, transfer, candidate.DepartureTime, candidate.ArrivalTime);
        while (transferArc.Advance()) yield return true;
        if (transferArc.Refusal.Length == 0 && FitsStockBurn(candidate)) result.Value = candidate;
    }

    // The endpoint moves until the UPFG braking distance from its arrival state matches the distance the endpoint assumed.
    private IEnumerable<bool> FitBrakingPointSteps(double departure, double flight, Result<DeorbitPlan?> result)
    {
        result.Value = null;
        DeorbitRequest r = _request;
        double distance = _seedDistance;
        for (int iteration = 1; iteration <= BrakingFitIterations; iteration++)
        {
            DeorbitPlan? transfer = Transfer(departure, flight, distance, checkAngle: true);
            if (transfer == null) yield break;
            yield return true;
            Status = $"Fitting the braking point ({CandidatesChecked}/{CandidateLimit}, correction {iteration}/{BrakingFitIterations}).";
            var prediction = new Result<double>();
            foreach (bool step in PredictBrakingDistanceSteps(r, transfer.ArrivalPosition, transfer.ArrivalVelocity,
                transfer.CutoffMass, prediction)) yield return step;
            double predicted = prediction.Value;
            if (!(predicted > 0)) yield break;
            if (Math.Abs(predicted - distance) * r.Settings.DownrangeFactor > BrakingFitToleranceM)
            {
                distance = 0.5 * (distance + predicted);
                continue;
            }
            Status = $"Checking the braking burn ({CandidatesChecked}/{CandidateLimit}).";
            var brake = new Result<string>();
            foreach (bool step in CanBrakeSteps(r, transfer.ArrivalPosition, transfer.ArrivalVelocity,
                transfer.ArrivalTime, transfer.CutoffMass, brake)) yield return step;
            if (brake.Value.Length > 0) yield break;
            Orbit orbit = Orbit.CreateFromStateCci(r.Parent, new UniverseTime(departure), transfer.DeparturePosition,
                transfer.DepartureVelocity, r.Source.OrbitLineColor);
            if (!SameState(orbit.GetStateVectorsAt(new UniverseTime(transfer.ArrivalTime)),
                transfer.ArrivalPosition, transfer.ArrivalVelocity)) yield break;
            double clearance = r.Clearance(transfer.ArrivalPosition, transfer.ArrivalTime);
            if (!(clearance >= ClearanceMargin)) yield break;
            result.Value = transfer with { BrakingDistance = predicted, Clearance = clearance };
            yield break;
        }
    }

    // A checked plan is kept while one departure and one flight-time pass try to lower its delta-v.
    private IEnumerable<bool> RefineSteps(DeorbitPlan plan)
    {
        double flight = plan.ArrivalTime - plan.DepartureTime;
        var departure = new Result<double>();
        Status = "A safe node is ready. Refining its departure time.";
        foreach (bool step in MinimizeSteps(t => TransferCost(t, flight),
            Math.Max(_firstDeparture, plan.DepartureTime - _departureStep),
            Math.Min(_lastDeparture, plan.DepartureTime + _departureStep), departure)) yield return step;
        var refinedFlight = new Result<double>();
        Status = "A safe node is ready. Refining its flight time.";
        foreach (bool step in MinimizeSteps(t => TransferCost(departure.Value, t),
            Math.Max(_shortestFlight, flight - _flightStep), Math.Min(_longestFlight, flight + _flightStep),
            refinedFlight)) yield return step;
        var improvement = new Result<DeorbitPlan?>();
        foreach (bool step in CheckTransferSteps(departure.Value, refinedFlight.Value, improvement)) yield return step;
        if (improvement.Value is DeorbitPlan better && Cost(better) < Cost(Plan)) Plan = better;
    }

    // Golden-section search with one Lambert solve per yield, so the planner budget can stop it between samples.
    private IEnumerable<bool> MinimizeSteps(Func<double, double> cost, double lower, double upper, Result<double> result)
    {
        double ratio = (Math.Sqrt(5) - 1) / 2;
        double a = lower, b = upper;
        double c = b - ratio * (b - a), d = a + ratio * (b - a);
        double costC = Sample(c);
        yield return true;
        double costD = Sample(d);
        yield return true;
        while (b - a > RefineToleranceSeconds)
        {
            if (costC <= costD)
            {
                (b, d, costD) = (d, c, costC);
                c = b - ratio * (b - a);
                costC = Sample(c);
            }
            else
            {
                (a, c, costC) = (c, d, costD);
                d = a + ratio * (b - a);
                costD = Sample(d);
            }
            yield return true;
        }
        result.Value = costC <= costD ? c : d;

        double Sample(double x)
        {
            RefinementSamples++;
            return cost(x);
        }
    }

    // FlightComputer.UpdateBurnTarget ignites half the burn duration before the node, timed at the minimum-throttle preview.
    // A candidate is kept only if that ignition leaves time for the stock turn and the burn ends before braking preparation.
    private bool FitsStockBurn(DeorbitPlan plan)
    {
        DeorbitEngine minimum = _request.Engines[0];
        double exhaustVelocity = minimum.Thrust / minimum.MassFlow;
        double duration = _request.Mass * (1 - Math.Exp(-plan.DeltaV.Length() / exhaustVelocity)) / minimum.MassFlow;
        return double.IsFinite(duration)
            && plan.DepartureTime - duration / 2 >= _request.Epoch + _request.StockPreparation + PlanningAllowanceSeconds
            && plan.DepartureTime + duration / 2 + _request.MinimumPulse < plan.ArrivalTime - PrepSeconds;
    }

    private double TransferCost(double departure, double flight) =>
        Cost(Transfer(departure, flight, _seedDistance, checkAngle: true));

    private static double Cost(DeorbitPlan? plan) => plan?.DeltaV.Length() ?? BadCost;

    // One impulsive transfer to the braking point `distance` uprange of the site at arrival, without clearance or braking checks.
    private DeorbitPlan? Transfer(double departure, double flight, double distance, bool checkAngle)
    {
        DeorbitRequest r = _request;
        if (!(flight > 2 * PrepSeconds) || !double.IsFinite(departure + flight)) return null;
        StateVectors source = r.Source.GetStateVectorsAt(new UniverseTime(departure));
        double arrival = departure + flight;
        double3 site = r.SiteAt(arrival);
        double3 normal = double3.Cross(source.PositionCci, site);
        if (normal.Length() < 1) return null;
        if (double3.Dot(normal, r.Source.GetOrbitNormalCci()) < 0) normal = -normal;
        normal = double3.Normalize(normal);
        double3 endpoint = GuidanceWindow.RotateAbout(site, normal,
            -(r.Settings.GateUprange + distance * r.Settings.DownrangeFactor) / r.Parent.MeanRadius) * r.BrakingRadius;
        Evaluations++;
        if (!TryLambert(r.Parent.Mu, source.PositionCci, endpoint, flight, normal, out double3 v1, out double3 v2)) return null;
        if (checkAngle && !MatchesArrivalAngle(r.Settings, endpoint, v2)) return null;
        double3 dv = v1 - source.VelocityCci;
        double mass = r.Mass * Math.Exp(-dv.Length() / r.ExhaustVelocity);
        if (!double.IsFinite(mass) || mass <= r.Model.Stages[0].MassDry) return null;
        return new(departure, arrival, source.PositionCci, v1, dv, endpoint, v2, distance, 0, mass);
    }

    internal static bool TryLambert(double mu, double3 start, double3 end, double duration,
        double3 normal, out double3 departure, out double3 arrival)
    {
        departure = arrival = default;
        if (!(mu > 0) || !(duration > 0) || !double.IsFinite(duration) || !Finite(start) || !Finite(end)
            || !Finite(normal) || start.Length() < 1 || end.Length() < 1)
            return false;
        double3 cross = double3.Cross(start / start.Length(), end / end.Length());
        if (cross.Length() < 1e-6) return false;

        // The stock solver chooses its transfer direction from global Z, so make the requested orbit normal local Z.
        double3 x = start / start.Length();
        double3 z = normal - x * double3.Dot(normal, x);
        if (z.Length() < 1e-6) return false;
        z = z.Normalized();
        double3 y = double3.Cross(z, x);
        OrbitalTransfers.SuperiorLambert(mu, ToLocal(start), ToLocal(end), new UniverseTime(duration),
            out double3 localDeparture, out double3 localArrival);
        departure = ToWorld(localDeparture);
        arrival = ToWorld(localArrival);
        return Finite(departure) && Finite(arrival)
            && double3.Dot(double3.Cross(start, departure), z) > 0;

        double3 ToLocal(double3 value) => new(double3.Dot(value, x), double3.Dot(value, y), double3.Dot(value, z));
        double3 ToWorld(double3 value) => x * value.X + y * value.Y + z * value.Z;
    }

    internal static DirectBrakingPlan? FindDirect(DeorbitRequest request) =>
        RunToEnd<DirectBrakingPlan?>(result => FindDirectSteps(request, result));

    private static IEnumerable<bool> FindDirectSteps(DeorbitRequest request, Result<DirectBrakingPlan?> result, DeorbitPlan? arrivalHint = null)
    {
        StateVectors initial = request.Source.GetStateVectorsAt(new UniverseTime(arrivalHint?.ArrivalTime ?? request.Epoch));
        var distance = new Result<double>();
        foreach (bool step in PredictBrakingDistanceSteps(request, initial.PositionCci, initial.VelocityCci, request.Mass, distance)) yield return step;
        double downrange = distance.Value;
        if (!(downrange > 0)) yield break;
        // The braking distance depends on the ignition state, so the ignition time is searched twice.
        double time = double.NaN;
        for (int pass = 0; pass < 2; pass++)
        {
            var approach = new Result<double>();
            foreach (bool step in FindApproachSteps(request, downrange * request.Settings.DownrangeFactor, approach)) yield return step;
            time = approach.Value;
            if (!double.IsFinite(time)) yield break;
            StateVectors state = request.Source.GetStateVectorsAt(new UniverseTime(time));
            foreach (bool step in PredictBrakingDistanceSteps(request, state.PositionCci, state.VelocityCci, request.Mass, distance)) yield return step;
            double prediction = distance.Value;
            if (!(prediction > 0)) yield break;
            if (pass == 0) downrange = prediction;
        }
        StateVectors final = request.Source.GetStateVectorsAt(new UniverseTime(time));
        var brake = new Result<string>();
        foreach (bool step in CanBrakeSteps(request, final.PositionCci, final.VelocityCci, time, request.Mass, brake)) yield return step;
        result.Value = new(time, final.PositionCci, final.VelocityCci, downrange, brake.Value);
    }

    private static IEnumerable<bool> FindApproachSteps(DeorbitRequest request, double distance, Result<double> result)
    {
        result.Value = double.NaN;
        double period = request.Source.Period;
        if (!(distance > 0) || distance >= Math.PI * request.Parent.MeanRadius) yield break;
        double step = period / ApproachSamplesPerOrbit;
        double previous = Distance(request.Epoch);
        for (int i = 1; i <= ApproachOrbits * ApproachSamplesPerOrbit; i++)
        {
            double t = request.Epoch + i * step;
            double current = Distance(t);
            yield return true;
            if (previous > distance && current <= distance && previous - current < Math.PI * request.Parent.MeanRadius)
            {
                double lo = t - step, hi = t;
                for (int j = 0; j < 30; j++)
                {
                    double mid = (lo + hi) / 2;
                    if (Distance(mid) > distance) lo = mid; else hi = mid;
                    yield return true;
                }
                result.Value = hi;
                yield break;
            }
            previous = current;
        }

        double Distance(double time)
        {
            StateVectors state = request.Source.GetStateVectorsAt(new UniverseTime(time));
            double3 normal = double3.Normalize(double3.Cross(state.PositionCci, state.VelocityCci));
            return SignedDistance(state.PositionCci, request.GateAt(time, normal), normal, request.Parent.MeanRadius);
        }
    }

    private static IEnumerable<bool> PredictBrakingDistanceSteps(DeorbitRequest request, double3 position, double3 velocity, double mass, Result<double> result)
    {
        result.Value = double.NaN;
        if (!Finite(position) || !Finite(velocity) || !(mass > 0)) yield break;
        var target = new UpfgTarget { Radius = request.GateRadius, Velocity = request.Settings.SinkRate,
            Fpa = -Math.PI / 2, Normal = double3.Normalize(double3.Cross(position, velocity)) };
        var solver = new UpfgGuidance();
        UpfgVehicle model = request.ModelAt(mass);
        for (int i = 0; i < PredictionIterations; i++)
        {
            solver.Step(position, velocity, mass, request.Parent.Mu, target, model, 1);
            yield return true;
            if (solver.Converged)
            {
                result.Value = SignedDistance(position, solver.Rd, target.Normal, request.Parent.MeanRadius);
                yield break;
            }
        }
    }

    internal static bool CanBrake(DeorbitRequest request, double3 position, double3 velocity,
        double time, double mass, out string refusal)
    {
        refusal = RunToEnd<string>(result => CanBrakeSteps(request, position, velocity, time, mass, result));
        return refusal.Length == 0;
    }

    private static IEnumerable<bool> CanBrakeSteps(DeorbitRequest request, double3 position, double3 velocity,
        double time, double mass, Result<string> result)
    {
        result.Value = "The braking state is not finite.";
        if (!Finite(position) || !Finite(velocity) || !double.IsFinite(time) || !(mass > 0)) yield break;
        double3 normal = double3.Normalize(double3.Cross(position, velocity));
        double3 gate = request.GateAt(time, normal);
        double distance = SignedDistance(position, gate, normal, request.Parent.MeanRadius);
        double height = position.Length() - request.GateRadius;
        if (!(distance > 0) || height > distance * Math.Tan(DirectApproachSlopeDeg * Math.PI / 180))
        {
            result.Value = $"Direct braking refused at {(position.Length() - request.Parent.MeanRadius) / 1000:F1} km altitude and {distance / 1000:F1} km uprange. Lower the approach first.";
            yield break;
        }
        var solver = new UpfgGuidance();
        var target = new UpfgTarget { Radius = request.GateRadius, Velocity = 0,
            DescentRate = request.Settings.SinkRate, Normal = normal, Rdes = gate };
        UpfgVehicle model = request.ModelAt(mass);
        for (int i = 0; i < PredictionIterations; i++)
        {
            solver.Step(position, velocity, mass, request.Parent.Mu, target, model, 3);
            yield return true;
            if (!solver.Converged) continue;
            double pitch = Math.Asin(Math.Clamp(double3.Dot(solver.Steering, position / position.Length()), -1, 1));
            if (!double.IsFinite(pitch) || pitch < LowestBrakingPitchDeg * Math.PI / 180 || !(solver.Tgo > 0)
                || !double.IsFinite(solver.Tgo) || !double.IsFinite(solver.VgoMag)) break;
            double capacity = 0;
            foreach (UpfgStage stage in model.Stages)
            {
                if (!(stage.MassTotal > stage.MassDry) || !(stage.MassDry > 0) || !(stage.Isp > 0)) continue;
                capacity += stage.Isp * 9.80665 * Math.Log(stage.MassTotal / stage.MassDry);
            }
            if (solver.VgoMag > capacity) { result.Value = "The braking burn exceeds the remaining propellant."; yield break; }
            result.Value = "";
            yield break;
        }
        result.Value = "The braking solve does not give a converged shallow approach. Lower the orbit or change the braking altitude.";
    }

    // Without a selected angle, the automatic range applies.
    internal static bool MatchesArrivalAngle(DeorbitSettings settings, double3 position, double3 velocity)
    {
        const double rounding = 1e-9;
        double angle = ArrivalAngleDegrees(position, velocity);
        if (!double.IsFinite(angle)) return false;
        return settings.ArrivalDescentDeg is double requested
            ? Math.Abs(angle + requested) <= ArrivalAngleToleranceDeg + rounding
            : angle >= -MaximumArrivalDescentDeg - rounding && angle <= MaximumArrivalClimbDeg + rounding;
    }

    internal static double ArrivalAngleDegrees(DeorbitPlan plan) => ArrivalAngleDegrees(plan.ArrivalPosition, plan.ArrivalVelocity);

    private static double ArrivalAngleDegrees(double3 position, double3 velocity) => Math.Asin(Math.Clamp(
        double3.Dot(position, velocity) / (position.Length() * velocity.Length()), -1, 1)) * 180 / Math.PI;

    internal static bool SameState(StateVectors state, double3 position, double3 velocity) =>
        (state.PositionCci - position).Length() <= PositionToleranceM
        && (state.VelocityCci - velocity).Length() <= VelocityToleranceMs;

    internal static double SignedDistance(double3 from, double3 to, double3 normal, double radius) =>
        Math.Atan2(double3.Dot(double3.Cross(from, to), normal), double3.Dot(from, to)) * radius;

    internal static bool Finite(double3 value) => double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);

    private static T RunToEnd<T>(Func<Result<T>, IEnumerable<bool>> steps)
    {
        var result = new Result<T>();
        foreach (bool _ in steps(result)) { }
        return result.Value;
    }

    private sealed class Result<T> { internal T Value = default!; }
}

internal sealed class DeorbitArcCheck
{
    private const int SampleLimit = 100000;
    // A sampled local minimum is resampled by dividing its two neighboring intervals into this many parts.
    private const int RefineSamples = 8;
    private const double LowArcHeightM = 50000;
    private const double LowArcSpacingM = 250;
    private const double HighArcSpacingM = 5000;
    private const double LongestStepSeconds = 5;
    private readonly DeorbitRequest _request;
    private readonly Orbit _orbit;
    private readonly double _end;
    private readonly double _safeRadius;
    private double _time;
    private int _samples;
    private bool _last;
    private double _beforeTime = double.NaN;
    private double _previousTime = double.NaN;
    private double _beforeClearance = double.NaN;
    private double _previousClearance = double.NaN;
    private double _refineStart;
    private double _refineStep;
    private int _refineIndex;
    internal string Refusal { get; private set; } = "";

    internal DeorbitArcCheck(DeorbitRequest request, Orbit orbit, double start, double end)
    {
        _request = request;
        _orbit = orbit;
        _time = start;
        _end = end;
        _safeRadius = Math.Max(Math.Max(request.Parent.MaxTerrainRadius, request.Parent.MeanRadius)
            + request.VehicleRadius + DeorbitPlanner.ClearanceMargin + 50, request.Parent.GetAtmosphereRadius());
        // PatchedConic.MarchImpactSearch also uses the maximum terrain radius to exclude clear orbits.
        if (orbit.Periapsis > _safeRadius) { _last = true; return; }
        // The next periapsis counts only if it belongs to the flown interval.
        double pe = orbit.TimeAtPeriapsis.Seconds();
        if (orbit.Eccentricity < 1 && orbit.Period > 0)
            pe += Math.Ceiling((start - pe) / orbit.Period) * orbit.Period;
        if (pe >= start && pe <= end) Check(pe);
    }

    internal bool Advance()
    {
        if (_last || Refusal.Length > 0) return false;
        if (++_samples > SampleLimit) { Refusal = "The terrain check exceeded its sample budget."; return false; }
        if (_refineIndex > 0)
        {
            Check(_refineStart + _refineStep * _refineIndex);
            _refineIndex = _refineIndex < RefineSamples - 1 ? _refineIndex + 1 : 0;
            return Refusal.Length == 0;
        }
        StateVectors state = _orbit.GetStateVectorsAt(new UniverseTime(_time));
        if (state.PositionCci.Length() > _safeRadius + 1)
        {
            // Orbit.GetNextTimeOfRadius locates the first possible entry into the terrain or atmosphere band.
            var entry = _orbit.GetNextTimeOfRadius(new UniverseTime(_time), new Radius(_safeRadius));
            if (!entry.HasValue || entry.Value.Item1.Seconds() > _end) { _last = true; return false; }
            double entryTime = entry.Value.Item1.Seconds();
            if (!double.IsFinite(entryTime) || entryTime <= _time)
            { Refusal = "The terrain band entry could not be resolved."; return false; }
            _time = entryTime;
            _beforeTime = _previousTime = double.NaN;
            return true;
        }
        double clearance = Check(_time);
        // Refine a sampled local minimum without blocking the next planner budget check.
        if (double.IsFinite(_beforeTime) && _previousClearance < _beforeClearance && _previousClearance < clearance)
        {
            _refineStart = _beforeTime;
            _refineStep = (_time - _beforeTime) / RefineSamples;
            _refineIndex = 1;
        }
        (_beforeTime, _beforeClearance) = (_previousTime, _previousClearance);
        (_previousTime, _previousClearance) = (_time, clearance);
        if (Refusal.Length > 0 || _time >= _end)
        {
            if (_refineIndex > 0 && Refusal.Length == 0) return true;
            _last = true;
            return false;
        }
        double height = state.PositionCci.Length() - _request.Parent.MaxTerrainRadius;
        double spacing = height < LowArcHeightM ? LowArcSpacingM : HighArcSpacingM;
        double speedBound = state.VelocityCci.Length() + Math.Abs(_request.Parent.GetAngularVelocity()) * state.PositionCci.Length();
        _time = Math.Min(_end, _time + Math.Min(LongestStepSeconds, spacing / Math.Max(speedBound, 1)));
        return true;
    }

    private double Check(double time)
    {
        StateVectors state = _orbit.GetStateVectorsAt(new UniverseTime(time));
        double clearance = _request.Clearance(state.PositionCci, time);
        if (!(clearance >= DeorbitPlanner.ClearanceMargin))
            Refusal = "The flown arc does not clear terrain by the planning margin.";
        else if (state.PositionCci.Length() < _request.Parent.GetAtmosphereRadius())
            Refusal = "The coast enters the atmosphere before braking.";
        return clearance;
    }
}
