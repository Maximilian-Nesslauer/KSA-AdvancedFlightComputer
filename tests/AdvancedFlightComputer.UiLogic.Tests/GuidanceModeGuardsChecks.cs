using AdvancedFlightComputer.Features.Ui.Logic;

internal static partial class Program
{
    private static void GuidanceModeGuardsRules()
    {
        // #245: Hover-aware climb-cut guard in 6-DOF touchdown guidance.
        Check("hoverable craft with live 6-DOF plan does not cut on brief +0.18 m/s gust",
            !GuidanceModeGuards.ShouldCutSixDofOnClimb(verticalSpeedMs: 0.18, minTwr: 0.70, hasLivePlan: true,
                climbCutMs: 0.1, hoverMaxMinTwr: 0.9));
        Check("hoverable craft with live 6-DOF plan does not cut on +0.50 m/s gust",
            !GuidanceModeGuards.ShouldCutSixDofOnClimb(verticalSpeedMs: 0.50, minTwr: 0.445, hasLivePlan: true,
                climbCutMs: 0.1, hoverMaxMinTwr: 0.9));
        Check("hoverable craft without live plan still cuts on climb above threshold",
            GuidanceModeGuards.ShouldCutSixDofOnClimb(verticalSpeedMs: 0.18, minTwr: 0.70, hasLivePlan: false,
                climbCutMs: 0.1, hoverMaxMinTwr: 0.9));
        Check("hoverable craft without live plan does not cut below climb threshold",
            !GuidanceModeGuards.ShouldCutSixDofOnClimb(verticalSpeedMs: 0.05, minTwr: 0.70, hasLivePlan: false,
                climbCutMs: 0.1, hoverMaxMinTwr: 0.9));
        Check("non-hoverable craft cuts immediately at zero vertical speed even with live plan",
            GuidanceModeGuards.ShouldCutSixDofOnClimb(verticalSpeedMs: 0.0, minTwr: 1.15, hasLivePlan: true,
                climbCutMs: 0.1, hoverMaxMinTwr: 0.9));
        Check("non-hoverable craft does not cut while descending",
            !GuidanceModeGuards.ShouldCutSixDofOnClimb(verticalSpeedMs: -0.5, minTwr: 1.15, hasLivePlan: true,
                climbCutMs: 0.1, hoverMaxMinTwr: 0.9));

        // #239: Vacuum climb attitude hold prevents engines-up flip over apogee and meets retrograde smoothly at vUp = 0.
        Check("vacuum climb hold activates when q < 10 Pa and climbing",
            GuidanceModeGuards.ShouldHoldVacuumClimbAttitude(dynamicPressurePa: 0.0, verticalSpeedMs: 120.0));
        Check("vacuum climb hold deactivates once descending",
            !GuidanceModeGuards.ShouldHoldVacuumClimbAttitude(dynamicPressurePa: 0.0, verticalSpeedMs: -1.0));
        Check("vacuum climb hold deactivates in sensible air",
            !GuidanceModeGuards.ShouldHoldVacuumClimbAttitude(dynamicPressurePa: 25.0, verticalSpeedMs: 120.0));

        // Climbing west (-x) and up (+z) over local vertical (+z): mirrored retrograde points east (+x) and UP (+z).
        bool dirOk = GuidanceModeGuards.VacuumClimbHoldDirection(-300.0, 0.0, 400.0, 0.0, 0.0, 1.0,
            out double ux, out double uy, out double uz);
        Check("vacuum climb hold points engines downward (positive radial component) while climbing",
            dirOk && Math.Abs(ux - 0.6) < 1e-12 && Math.Abs(uy) < 1e-12 && Math.Abs(uz - 0.8) < 1e-12);

        // Continuity across apogee (vUp -> 0+ matches surface retrograde -vHat at vUp = 0).
        bool apogeeOk = GuidanceModeGuards.VacuumClimbHoldDirection(-300.0, 0.0, 1e-9, 0.0, 0.0, 1.0,
            out double ax, out double ay, out double az);
        Check("vacuum climb hold joins retrograde continuously at apogee",
            apogeeOk && Math.Abs(ax - 1.0) < 1e-9 && Math.Abs(ay) < 1e-9 && Math.Abs(az) < 1e-9);

        // #240: Non-sticky AeroError retry guard.
        Check("non-stale aero table with prior transient error retries once retry cooldown elapses",
            GuidanceModeGuards.ShouldStartAeroSweep(force: false, now: 12.0, retryAt: 11.0,
                hasError: true, meshActivelyRebuilding: false, wrongKind: false, isStale: false));
        Check("non-stale aero table with prior error waits during retry cooldown",
            !GuidanceModeGuards.ShouldStartAeroSweep(force: false, now: 10.5, retryAt: 11.0,
                hasError: true, meshActivelyRebuilding: false, wrongKind: false, isStale: false));
        Check("active mesh rebuild error does not spam sweeps until rebuild finishes",
            !GuidanceModeGuards.ShouldStartAeroSweep(force: false, now: 12.0, retryAt: 11.0,
                hasError: true, meshActivelyRebuilding: true, wrongKind: false, isStale: false));
        Check("clean non-stale aero table does not trigger a sweep",
            !GuidanceModeGuards.ShouldStartAeroSweep(force: false, now: 12.0, retryAt: 11.0,
                hasError: false, meshActivelyRebuilding: false, wrongKind: false, isStale: false));
        Check("forced resample bypasses cooldown and staleness",
            GuidanceModeGuards.ShouldStartAeroSweep(force: true, now: 10.0, retryAt: 15.0,
                hasError: false, meshActivelyRebuilding: true, wrongKind: false, isStale: false));

        // #234: Unpowered-impact emergency braking backstop.
        // 55 t booster, 1.55 MN thrust, g = 9.81 m/s^2 -> aNet = 18.37 m/s^2.
        // At 170 m/s sink, hStop = 786.5 m -> hTrigger = 1.35 * 786.5 + 85 = 1146.8 m.
        Check("emergency brake does not trigger high in the glide at 5 km",
            !GuidanceModeGuards.ShouldTriggerEmergencyBrake(legsAltM: 5000.0, verticalSpeedMs: -170.0,
                maxThrustN: 1.55e6, massKg: 55_000.0, gravityMs2: 9.81));
        Check("emergency brake triggers when descending inside stopping margin",
            GuidanceModeGuards.ShouldTriggerEmergencyBrake(legsAltM: 1000.0, verticalSpeedMs: -170.0,
                maxThrustN: 1.55e6, massKg: 55_000.0, gravityMs2: 9.81));
        Check("emergency brake does not trigger while climbing or hovering",
            !GuidanceModeGuards.ShouldTriggerEmergencyBrake(legsAltM: 50.0, verticalSpeedMs: -2.0,
                maxThrustN: 1.55e6, massKg: 55_000.0, gravityMs2: 9.81));
        Check("emergency brake does not trigger without thrust",
            !GuidanceModeGuards.ShouldTriggerEmergencyBrake(legsAltM: 200.0, verticalSpeedMs: -150.0,
                maxThrustN: 0.0, massKg: 55_000.0, gravityMs2: 9.81));

        double demandHigh = GuidanceModeGuards.ComputeEmergencyBrakeDemandN(
            legsAltM: 800.0, verticalSpeedMs: -160.0, massKg: 55_000.0, gravityMs2: 9.81,
            verticalCos: 1.0, minTwr: 1.2, hoverMaxMinTwr: 0.9);
        double expectedHigh = 55_000.0 * (160.0 * 160.0 / 1600.0 + 9.81);
        Check("emergency brake demand matches exact vertical stopping curve",
            Math.Abs(demandHigh - expectedHigh) < 1e-6);

        double demandCut = GuidanceModeGuards.ComputeEmergencyBrakeDemandN(
            legsAltM: 10.0, verticalSpeedMs: 0.2, massKg: 55_000.0, gravityMs2: 9.81,
            verticalCos: 1.0, minTwr: 1.2, hoverMaxMinTwr: 0.9);
        Check("emergency brake cuts non-hoverable engine once descent stops", demandCut == 0.0);

        double demandHover = GuidanceModeGuards.ComputeEmergencyBrakeDemandN(
            legsAltM: 10.0, verticalSpeedMs: -0.3, massKg: 55_000.0, gravityMs2: 9.81,
            verticalCos: 1.0, minTwr: 0.4, hoverMaxMinTwr: 0.9);
        Check("emergency brake holds hoverable craft gently near zero sink",
            Math.Abs(demandHover - 55_000.0 * 9.81) < 1e-6);
    }
}
