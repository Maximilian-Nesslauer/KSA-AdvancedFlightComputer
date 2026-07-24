using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>
/// Retargets a stock Hohmann/Lambert transfer so the arrival is a flyby at a
/// chosen periapsis instead of a center-aimed impact. The stock planner aims
/// the Lambert solve at the target body's center
/// (<see cref="OrbitalTransfers.SolveLambert"/> uses
/// <c>Target.Orbit.GetStateVectorsAt(arrival).PositionCci</c>), which is why a
/// well-timed transfer intersects the body. We instead aim at
/// <c>center + b * n_hat</c>, where b is the hyperbolic impact parameter that
/// produces the requested flyby periapsis and n_hat is perpendicular to the
/// approach relative velocity, then re-solve the departure so the burn itself
/// yields the flyby (no separate mid-course correction).
///
/// Two gravitational parameters are involved and must not be conflated:
///   * The Lambert departure/arrival solve is around the SHARED PARENT of the
///     transfer (Earth for LEO -&gt; Luna, the Sun for LEO -&gt; Mars). All
///     <see cref="OrbitalTransfers.SuperiorLambert"/> calls use that parent mu.
///   * The impact parameter and periapsis speed use the TARGET BODY's OWN mu
///     (Luna's / Mars's), read via <see cref="IParentBody.Mu"/>. Note the stock
///     course-correction (<c>CorrectionBurnTask.CourseCorrectCci</c>) reads
///     <c>target.Orbit.Mu</c>, which resolves to the target's PARENT mu, so this
///     re-derivation deliberately does not follow it there.
///
/// Impact-parameter relations (standard patched-conic B-plane targeting):
///   v_p = sqrt(v_inf^2 + 2 mu_target / r_p),  b = r_p * v_p / v_inf.
/// v_inf is taken at the SOI boundary (energy corrected by -mu_target/SOI) so the
/// achieved patched-conic periapsis matches r_p, matching the game's own model.
/// </summary>
internal static class FlybyTargeting
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // Coarse samples for the transfer-vs-target closest-approach bracket before
    // Brent refinement. 48 over the arrival window resolves the minimum well
    // below the Brent tolerance for Earth-Luna and Earth-Mars geometries.
    private const int ClosestApproachCoarseSteps = 48;

    // Fixed-point iterations for v_inf convergence. The offset moves the aim
    // point by ~b (a few thousand km for a moon, tiny against the transfer
    // scale), so v_inf shifts by well under 1% and 3 passes over-converge.
    private const int DefaultIterations = 3;

    /// <summary>Smallest usable axis alignment (sine of the angle between a side's
    /// axis and the approach relative velocity). Below roughly 9 degrees the
    /// projected offset direction is dominated by numerical noise rather than the
    /// requested side, so the UI disables that side instead of aiming blindly.</summary>
    public const double MinAxisAlignment = 0.15;

    /// <summary>Result of a flyby retarget: the departure burn (VLF and CCI) plus
    /// the geometry used, so the UI can show v_inf / impact parameter and the
    /// caller can build a preview. <see cref="BurnTime"/> equals the transfer
    /// Start for same-parent; for cross-parent it carries the hyperbolic-escape
    /// true-anomaly nudge the stock planner applies.
    ///
    /// <see cref="PlannerVInfMs"/> and <see cref="PlannerApoTargetMeters"/> are the
    /// parking-frame energy descriptors the multi-pass planner locks (the
    /// hyperbolic ejection excess for cross-parent, the post-burn apoapsis for
    /// same-parent) so a flyby departure can be split into perigee kicks. These
    /// are distinct from <see cref="VInfMs"/>, which is the target-relative speed at
    /// the SOI boundary (v_soi) that the impact parameter b = r_p * v_p / v_soi
    /// divides by. That is NOT the asymptotic excess sqrt(2E) (a few percent lower);
    /// the divisor must be the SOI-boundary speed because the aim point is the
    /// parent-frame closest approach, which is where the game builds the flyby
    /// conic from the relative state.</summary>
    public readonly record struct FlybyResult(
        double3 DvVlf,
        double3 DvCci,
        SimTime BurnTime,
        double ImpactParameterMeters,
        double VInfMs,
        double TargetPeRadiusMeters,
        bool IsCrossParent,
        double PlannerVInfMs,
        double PlannerApoTargetMeters);

    #region Reference-radius resolution

    /// <summary>Turns a user-entered value + reference into a periapsis radius
    /// measured from the target center.</summary>
    public static double ResolvePeriapsisRadius(
        IParentBody target, double value, FlybyReference reference)
    {
        return reference switch
        {
            FlybyReference.Center => value,
            FlybyReference.Atmosphere => target.GetAtmosphereRadius() + value,
            _ => target.MeanRadius + value,
        };
    }

    /// <summary>Lowest periapsis radius the game treats as clear of the body: the
    /// terrain ceiling for an airless body, but the top of the atmosphere for one
    /// with an atmosphere (<see cref="Astronomical.GetNearSurfaceRadius"/> and the
    /// <c>AtmosphericBody</c> override). Below it a flyby would impact or enter the
    /// atmosphere, so the UI blocks Create.</summary>
    public static double MinFlybyRadius(IParentBody target) => target.GetNearSurfaceRadius();

    /// <summary><see cref="IParentBody.GetAtmosphereRadius"/> returns 0 for airless
    /// bodies, so a positive value means the Atmosphere reference is meaningful.</summary>
    public static bool HasAtmosphere(IParentBody target) => target.GetAtmosphereRadius() > 0.0;

    #endregion

    #region Impact-parameter closed forms (also the unit-test oracle base)

    /// <summary>Impact parameter b that yields flyby periapsis
    /// <paramref name="rpRadius"/> for hyperbolic excess speed
    /// <paramref name="vInf"/> about a body of gravitational parameter
    /// <paramref name="muTarget"/>. NaN on non-physical inputs.</summary>
    public static double ImpactParameterForPeriapsis(double vInf, double rpRadius, double muTarget)
    {
        if (!(vInf > 0.0) || !(rpRadius > 0.0) || !(muTarget > 0.0))
            return double.NaN;
        double vP = Math.Sqrt(vInf * vInf + 2.0 * muTarget / rpRadius);
        return rpRadius * vP / vInf;
    }

    /// <summary>Inverse of <see cref="ImpactParameterForPeriapsis"/>: the flyby
    /// periapsis produced by impact parameter <paramref name="b"/>. Used by the
    /// tests to close the loop against the game's own orbit propagation.</summary>
    public static double PeriapsisForImpactParameter(double vInf, double b, double muTarget)
    {
        if (!(vInf > 0.0) || !(b > 0.0) || !(muTarget > 0.0))
            return double.NaN;
        // From b = r_p * sqrt(v_inf^2 + 2 mu / r_p) / v_inf, solving for r_p:
        //   r_p = -mu/v_inf^2 + sqrt((mu/v_inf^2)^2 + b^2).
        double k = muTarget / (vInf * vInf);
        return -k + Math.Sqrt(k * k + b * b);
    }

    #endregion

    #region Cross-parent detection

    /// <summary>True when the departure is a hyperbolic escape (target orbits a
    /// different parent than the vehicle, e.g. LEO -&gt; Mars). Uses a direct
    /// parent-id compare rather than <see cref="OrbitalTransfers.SameSoiTransfer"/>
    /// because stock rewrites the transfer source to the vehicle for same-SOI
    /// transfers, which makes a re-check unreliable.</summary>
    public static bool IsCrossParentTransfer(Vehicle source, IOrbiter target)
    {
        string? sp = source.Orbit.Parent?.Id;
        string? tp = target.Orbit?.Parent?.Id;
        return sp == null || tp == null || sp != tp;
    }

    #endregion

    #region Retarget

    /// <summary>Retargets the departure of a stock transfer (identified by its
    /// Start and Transit, e.g. from a selected porkchop entry) so it flies by
    /// <paramref name="target"/> at <paramref name="targetPeRadius"/> instead of
    /// impacting, placing the periapsis on the requested <paramref name="side"/>.
    ///
    /// <see cref="FlybyOutcome.Result"/> is null on a degenerate geometry (the
    /// requested side's axis is near-parallel to the approach velocity, or the
    /// solve is non-physical); the caller should then fall back to the stock
    /// center-aimed burn and warn. The axis alignments are reported either way so
    /// the UI can show which sides this approach can actually reach.</summary>
    public static FlybyOutcome ComputeFlybyDeparture(
        Vehicle source, IOrbiter target, SimTime start, SimTime transit,
        double targetPeRadius, FlybySide side, int iterations = DefaultIterations)
    {
        // Vehicle.Orbit is non-nullable (it throws on an empty plan rather than
        // returning null), so only the target orbit / parent needs a null guard.
        if (target.Orbit?.Parent == null) return FlybyOutcome.Unavailable;
        if (target is not IParentBody targetBody) return FlybyOutcome.Unavailable;
        if (!(targetPeRadius > 0.0)) return FlybyOutcome.Unavailable;

        IParentBody lambertParent = target.Orbit.Parent;
        double muParent = lambertParent.Mu;
        double muTarget = targetBody.Mu;
        double soiTarget = targetBody.SphereOfInfluence;
        if (!(muParent > 0.0) || !(muTarget > 0.0) || !(soiTarget > 0.0))
            return FlybyOutcome.Unavailable;

        bool isCross = IsCrossParentTransfer(source, target);

        // OrbitalTransfers.SingleImpulseHyperbolicEscape throws outright on a
        // hyperbolic parking orbit, and the offset solve below runs on the parking
        // celestial's orbit, so nothing downstream would catch it. Stock guards the
        // same way at its own call sites.
        if (isCross && source.Orbit.Eccentricity >= 1.0) return FlybyOutcome.Unavailable;

        // The Lambert is solved in the target-parent CCI frame. For same-parent
        // the departure body is the vehicle itself; for cross-parent it is the
        // vehicle's parking-parent celestial, whose (heliocentric) orbit shares
        // the target's parent, matching the frame stock's SolveLambert uses.
        Orbit sourceInFrame;
        if (isCross)
        {
            if (source.Orbit.Parent is not Celestial parkingCelestial
                || parkingCelestial.Orbit?.Parent?.Id != lambertParent.Id)
                return FlybyOutcome.Unavailable;
            sourceInFrame = parkingCelestial.Orbit;
        }
        else
        {
            sourceInFrame = source.Orbit;
        }

        OffsetSolve? solve = SolveOffsetTransfer(
            muParent, sourceInFrame, target.Orbit, start, transit,
            targetPeRadius, muTarget, soiTarget, side, iterations,
            out double radialAlign, out double normalAlign);
        if (solve == null)
            return new FlybyOutcome(null, radialAlign, normalAlign);
        OffsetSolve s = solve.Value;

        FlybyResult result = isCross
            ? BuildCrossParentDeparture(source, lambertParent, start, s, targetPeRadius)
            : BuildSameParentDeparture(source, start, s, targetPeRadius);

        if (!IsFinite(result.DvVlf) || !(result.DvVlf.Length() > 0.0))
            return new FlybyOutcome(null, radialAlign, normalAlign);

        if (DebugConfig.Flyby)
        {
            // Offset direction in the target's own orbital frame, so the flyby side
            // is readable as physics instead of a raw CCI vector: radial is
            // toward/away from the parent, along-track is leading/trailing on the
            // target's path, normal is out of its orbital plane. The offset is
            // always perpendicular to the approach relative velocity, so a purely
            // along-track offset is generally not reachable.
            StateVectors tsv = target.Orbit.GetStateVectorsAt(s.CaTime);
            double3 radial = tsv.PositionCci.NormalizeOrZero();
            double3 alongTrack = tsv.VelocityCci.NormalizeOrZero();
            double3 normal = double3.Cross(tsv.PositionCci, tsv.VelocityCci).NormalizeOrZero();

            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] FlybyTargeting.ComputeFlybyDeparture: vehicle='{0}' target='{1}' " +
                "cross={2} rp={3:F0}m b={4:F0}m vInf={5:F1}m/s side={6} " +
                "|dvVlf|={7:F1}m/s burnT={8:F0}s offsetDir[radial={9:F2} alongTrack={10:F2} " +
                "normal={11:F2}] axisAlign[radial={12:F2} normal={13:F2}]",
                source.Id, (target as Astronomical)?.Id ?? "?", isCross, targetPeRadius,
                s.ImpactParameter, s.VInf, side, result.DvVlf.Length(),
                result.BurnTime.Seconds(),
                double3.Dot(s.OffsetDir, radial),
                double3.Dot(s.OffsetDir, alongTrack),
                double3.Dot(s.OffsetDir, normal),
                radialAlign, normalAlign));
        }

        return new FlybyOutcome(result, radialAlign, normalAlign);
    }

    /// <summary>Result of a retarget attempt plus how well each named side axis can
    /// be reached for this approach. An alignment is the sine of the angle between
    /// the axis and the approach relative velocity: at 1 the axis is fully usable,
    /// near 0 the offset would have to point along the approach, which cannot move
    /// the periapsis. <see cref="MinAxisAlignment"/> is the cutoff the UI uses to
    /// disable a side.</summary>
    public readonly record struct FlybyOutcome(
        FlybyResult? Result,
        double RadialAxisAlignment,
        double NormalAxisAlignment)
    {
        public static FlybyOutcome Unavailable => new(null, 0.0, 0.0);

        /// <summary>Whether the solve got far enough to measure the axes. It bails
        /// out before that for reasons unrelated to the side (bad target, no SOI,
        /// an approach energy too low for the requested periapsis), and reporting
        /// those as "no side is reachable" would disable the whole picker.</summary>
        public bool HasAxisData => RadialAxisAlignment > 0.0 || NormalAxisAlignment > 0.0;

        public bool CanReach(FlybySide side) =>
            !HasAxisData || AxisAlignmentFor(side) >= MinAxisAlignment;

        public double AxisAlignmentFor(FlybySide side) =>
            side is FlybySide.Inner or FlybySide.Outer
                ? RadialAxisAlignment
                : NormalAxisAlignment;
    }

    /// <summary>Departure state from the offset-aim Lambert solve, in the
    /// target-parent CCI frame. <see cref="EjectDeltaCci"/> is
    /// <c>vEject - sourceInFrame.velocity(start)</c>: the injection dV for
    /// same-parent, or the hyperbolic excess relative to the parking parent for
    /// cross-parent (the quantity stock names EjectionVelocityCci).</summary>
    private readonly record struct OffsetSolve(
        double3 EjectDeltaCci,
        SimTime Transit,
        double ImpactParameter,
        double VInf,
        double3 OffsetDir,
        SimTime CaTime);

    private static OffsetSolve? SolveOffsetTransfer(
        double muParent, Orbit sourceInFrame, Orbit targetOrbit,
        SimTime start, SimTime transit, double rpRadius, double muTarget,
        double soiTarget, FlybySide side, int iterations,
        out double radialAlign, out double normalAlign)
    {
        radialAlign = 0.0;
        normalAlign = 0.0;
        double3 departurePos = sourceInFrame.GetStateVectorsAt(start).PositionCci;
        double3 departureVel = sourceInFrame.GetStateVectorsAt(start).VelocityCci;
        byte4 lineColor = sourceInFrame.OrbitLineColor;

        // Seed with the stock center-aimed solve.
        double3 arrivalCenter = targetOrbit.GetStateVectorsAt(start + transit).PositionCci;
        OrbitalTransfers.SuperiorLambert(
            muParent, departurePos, arrivalCenter, transit, out double3 vEject, out _);

        SimTime lastTransit = transit;
        double lastB = double.NaN;
        double lastVInf = double.NaN;
        double3 lastOffsetDir = double3.Zero;
        SimTime lastCaTime = default;

        for (int iter = 0; iter < iterations; iter++)
        {
            Orbit transfer = Orbit.CreateFromStateCci(
                targetOrbit.Parent, start, departurePos, vEject, lineColor);

            SimTime caTime = FindClosestApproach(
                transfer, targetOrbit, start, start + lastTransit);

            // Relative velocity at closest approach (offset direction) and, a
            // little earlier, at the SOI boundary (energy for the flyby speed).
            double3 vRelCa = transfer.GetStateVectorsAt(caTime).VelocityCci
                             - targetOrbit.GetStateVectorsAt(caTime).VelocityCci;
            double vRelCaLen = vRelCa.Length();
            if (!(vRelCaLen > 0.0)) return null;

            SimTime soiTime = new SimTime(caTime.Seconds() - soiTarget / vRelCaLen);
            double3 vRelSoi = transfer.GetStateVectorsAt(soiTime).VelocityCci
                              - targetOrbit.GetStateVectorsAt(soiTime).VelocityCci;
            double vInf = vRelSoi.Length();
            if (!(vInf > 0.0)) return null;

            // Patched-conic flyby speed at r_p, matching CorrectionBurnTask's form
            // but with the target body's own mu. b is the impact parameter.
            double energy = 0.5 * vInf * vInf - muTarget / soiTarget;
            double vpArg = 2.0 * (energy + muTarget / rpRadius);
            if (!(vpArg > 0.0)) return null;
            double vP = Math.Sqrt(vpArg);
            double b = rpRadius * vP / vInf;
            if (!(b > 0.0) || double.IsNaN(b)) return null;

            // Named sides live in the TARGET's orbital frame: radial is the target's
            // own radius from its parent, normal its orbit normal. The offset has to
            // be perpendicular to the approach relative velocity, so the requested
            // axis is projected into that plane; how much survives the projection is
            // the alignment the UI gates on.
            StateVectors tsvCa = targetOrbit.GetStateVectorsAt(caTime);
            double3 radialAxis = tsvCa.PositionCci.NormalizeOrZero();
            double3 normalAxis = double3.Cross(
                tsvCa.PositionCci, tsvCa.VelocityCci).NormalizeOrZero();
            double3 vRelHat = vRelCa / vRelCaLen;

            radialAlign = PerpendicularComponent(radialAxis, vRelHat).Length();
            normalAlign = PerpendicularComponent(normalAxis, vRelHat).Length();

            double3 wantedAxis = side switch
            {
                FlybySide.Inner => -radialAxis,
                FlybySide.Outer => radialAxis,
                FlybySide.North => normalAxis,
                _ => -normalAxis,
            };

            // Axis too close to the approach direction: the surviving perpendicular
            // component is noise rather than the requested side, so refuse instead of
            // aiming at a direction the user did not ask for. Same cutoff the picker
            // greys the side out with, so UI and solver agree.
            double3 wantedPerp = PerpendicularComponent(wantedAxis, vRelHat);
            if (wantedPerp.Length() < MinAxisAlignment) return null;
            double3 offsetDir = wantedPerp.NormalizeOrZero();
            if (offsetDir.LengthSquared() < 1e-12) return null;

            double3 aim = tsvCa.PositionCci + b * offsetDir;

            SimTime newTransit = new SimTime(caTime.Seconds() - start.Seconds());
            if (!(newTransit.Seconds() > 0.0)) return null;

            OrbitalTransfers.SuperiorLambert(
                muParent, departurePos, aim, newTransit, out vEject, out _);

            lastTransit = newTransit;
            lastB = b;
            lastVInf = vInf;
            lastOffsetDir = offsetDir;
            lastCaTime = caTime;
        }

        double3 ejectDelta = vEject - departureVel;
        if (!IsFinite(ejectDelta)) return null;
        return new OffsetSolve(
            ejectDelta, lastTransit, lastB, lastVInf, lastOffsetDir, lastCaTime);
    }

    private static FlybyResult BuildSameParentDeparture(
        Vehicle source, SimTime start, OffsetSolve s, double rpRadius)
    {
        // Same-parent: EjectDeltaCci is the geocentric injection dV directly
        // (matches OrbitalTransfers.FinalizeLambert's Source == Vehicle branch, magnitude
        // preserved through the VLF rotation). Departs at the transfer Start,
        // no true-anomaly nudge.
        double3 dvCci = s.EjectDeltaCci;
        StateVectors sv = source.Orbit.GetStateVectorsAt(start);
        double3 dvVlf = ToVlf(sv, dvCci);

        // Post-burn apoapsis is the energy descriptor the multi-pass planner
        // targets for same-parent transfers.
        Orbit postBurn = Orbit.CreateFromStateCci(
            source.Orbit.Parent, start, sv.PositionCci, sv.VelocityCci + dvCci,
            source.Orbit.OrbitLineColor);
        double apoTarget = postBurn.IsBound() ? postBurn.Apoapsis : double.NaN;

        return new FlybyResult(dvVlf, dvCci, start,
            s.ImpactParameter, s.VInf, rpRadius, IsCrossParent: false,
            PlannerVInfMs: double.NaN, PlannerApoTargetMeters: apoTarget);
    }

    private static FlybyResult BuildCrossParentDeparture(
        Vehicle source, IParentBody lambertParent, SimTime start,
        OffsetSolve s, double rpRadius)
    {
        // Cross-parent: EjectDeltaCci is the heliocentric excess relative to the
        // parking parent. Convert it to the parking-parent frame and solve the
        // single-impulse hyperbolic escape, mirroring OrbitalTransfers.FinalizeLambert's
        // Source != Vehicle branch (including its true-anomaly start nudge).
        IParentBody parkingParent = source.Orbit.Parent!;
        double muParking = parkingParent.Mu;

        doubleQuat cci2Cce = lambertParent.GetCci2Cce();
        doubleQuat cce2Cci = parkingParent.GetCce2Cci();
        doubleQuat toParking = doubleQuat.Concatenate(cci2Cce, cce2Cci);
        double3 velHypInfinity = s.EjectDeltaCci.Transform(toParking);

        StateVectors sv = source.Orbit.GetStateVectorsAt(start);
        OrbitalTransfers.SingleImpulseTransfer impulse =
            OrbitalTransfers.SingleImpulseHyperbolicEscape(
                muParking, source.Orbit, sv.PositionCci, sv.VelocityCci, velHypInfinity);

        double3 dvCci = impulse.VelocityEject - impulse.VelParking;

        // Shift the burn to the escape true anomaly the impulse solver chose.
        SimTime toBurnTa = source.Orbit.GetTimeFromPeTo(impulse.BurnTrueAnomaly);
        SimTime toCurrentTa = source.Orbit.GetTimeFromPeTo(sv.TrueAnomaly);
        SimTime burnTime = start + (toBurnTa - toCurrentTa);

        double3 dvVlf = ToVlf(source.Orbit.GetStateVectorsAt(burnTime), dvCci);
        // The hyperbolic ejection excess relative to the parking parent is the
        // energy descriptor the multi-pass planner targets for cross-parent.
        double plannerVInf = s.EjectDeltaCci.Length();
        return new FlybyResult(dvVlf, dvCci, burnTime,
            s.ImpactParameter, s.VInf, rpRadius, IsCrossParent: true,
            PlannerVInfMs: plannerVInf, PlannerApoTargetMeters: double.NaN);
    }

    #endregion

    #region Helpers

    private static double3 ToVlf(StateVectors sv, double3 dvCci)
    {
        doubleQuat parentCci2Vlf = sv.GetVlf2ParentCci().OrIdentity().Inverse();
        return dvCci.Transform(parentCci2Vlf);
    }

    /// <summary>Time of closest approach of <paramref name="transfer"/> to
    /// <paramref name="target"/> within [<paramref name="tStart"/>,
    /// <paramref name="tEnd"/>]: a coarse scan to bracket the minimum, then
    /// Brent refinement (same shape as the stock closest-approach search).</summary>
    private static SimTime FindClosestApproach(
        Orbit transfer, Orbit target, SimTime tStart, SimTime tEnd)
    {
        double a = tStart.Seconds();
        double bEnd = tEnd.Seconds();
        // Widen the window slightly past the seed arrival so an offset transfer's
        // true closest approach (a touch later or earlier) stays bracketed.
        double span = bEnd - a;
        // Skip the first second so the search never latches onto the departure
        // point itself (distance is small there for a low parking orbit).
        a += 1.0;
        bEnd += span * 0.25;

        double Dist(double t)
        {
            double3 p = transfer.GetStateVectorsAt(new SimTime(t)).PositionCci
                        - target.GetStateVectorsAt(new SimTime(t)).PositionCci;
            return p.Length();
        }

        double bestT = a;
        double bestD = double.MaxValue;
        double step = (bEnd - a) / ClosestApproachCoarseSteps;
        if (!(step > 0.0)) return new SimTime(a);
        for (int i = 0; i <= ClosestApproachCoarseSteps; i++)
        {
            double t = a + i * step;
            double d = Dist(t);
            if (d < bestD) { bestD = d; bestT = t; }
        }

        double lo = Math.Max(a, bestT - step);
        double hi = Math.Min(bEnd, bestT + step);
        double refined = MathEx.BrentMin(Dist, lo, hi, 1e-06);
        return new SimTime(refined);
    }

    /// <summary>Component of <paramref name="v"/> perpendicular to the unit vector
    /// <paramref name="unitAxis"/>. Its length is the sine of the angle between
    /// them, which is what makes it double as the reachability measure.</summary>
    private static double3 PerpendicularComponent(double3 v, double3 unitAxis) =>
        v - unitAxis * double3.Dot(v, unitAxis);

    private static bool IsFinite(double3 v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);

    #endregion
}
