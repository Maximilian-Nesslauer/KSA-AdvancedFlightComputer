#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using KSA;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Features.Ui.Logic;

// The countdown to the next separation while guidance flies an ascent: SEP T- in the console head.
//
// Each cue that fires a staging row is a propellant margin running down, and the countdown is that margin over the rate it falls at, measured live (DrainRate):
//   - THE BOOSTER RESERVE: the propellant still to burn before the reserve cue, as ShouldStageForReserve counts it - in the side boosters' own tanks for side boosters, else the stack's mass down to the stage model's stage end.
//   - BURNOUT, with no reserve: the propellant to the end of the stage model's first stage. That model is the game's own drain simulation (SequencePerformance.Phases), split wherever the number of burning engines changes, so its first stage ends where the side boosters or the core burn out and AutoStage drops them. The last stage ends in the ascent's cutoff, not a separation, so it gets no countdown.
//   - THE DRONE SHIP: how far the ship lies beyond the live impact over how fast the impact closes on it (DroneShipSeparation).
// The soonest wins, and the row's decoupler delay goes on top: the count is to the parts coming off, not to the next engine's ignition.
public static partial class GuidanceWindow
{
    /// <summary>How often the countdown is taken, s of sim time. The head reads it down in between (StagingCountdown.Now).</summary>
    private const double SeparationCountdownIntervalS = 0.25;

    /// <summary>The countdown now, s, NaN when no separation is coming or it cannot be known.</summary>
    internal static double SeparationSecondsNow()
        => StagingCountdown.Now(_s.SeparationInS, _s.SeparationTakenAt, SimNow());

    /// <summary>The countdown of <paramref name="vehicle"/>'s own guidance, for a reader that is not bound to it, such as the separation band.</summary>
    internal static double SeparationSecondsOf(Vehicle vehicle)
        => vehicle != null && VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState s) && s.Running
            ? StagingCountdown.Now(s.SeparationInS, s.SeparationTakenAt, SimNow())
            : double.NaN;

    private static void UpdateSeparationCountdown(Vehicle vehicle, bool thrustOn)
    {
        double now = SimNow();
        if (!_s.Running || !thrustOn || !StagingDetector.IsArmed(vehicle) || StagingDetector.IsHeldForControl(vehicle))
        {
            _s.SeparationInS = double.NaN;
            _s.SeparationDrain.Reset();
            return;
        }
        if (now - _s.SeparationTakenAt < SeparationCountdownIntervalS && now >= _s.SeparationTakenAt)
            return;

        double seconds = double.NaN;
        UpfgVehicle model = _s.StageModel;
        int stages = model?.Stages.Count ?? 0;
        bool reserve = _s.ReserveArmed && !_s.ReserveStaged && _s.ReserveKg > 0.0;
        double mass = double.NaN, end = double.NaN;
        int cue;
        if (reserve && _s.ReserveFlowShare < 1.0)
        {
            // Side boosters: their own tanks, down to the reserve.
            cue = 1;
            mass = ReserveTankMass(vehicle);
            end = _s.ReserveKg;
        }
        else if (stages >= 2)
        {
            // The stack down to the end of its first stage, short of the reserve when there is one.
            cue = reserve ? 2 : 3;
            mass = vehicle.TotalMass;
            end = model.Stages[0].MassDry + (reserve ? _s.ReserveKg : 0.0);
        }
        else
        {
            cue = 0;
        }

        double rate = _s.SeparationDrain.Update(now, mass, HashCode.Combine(cue, stages, _s.ReserveRootId));
        if (cue != 0)
            seconds = StagingCountdown.Seconds(mass - end, rate);

        // The drone ship, while it is the next to separate and its impact closes on the ship.
        if (reserve && double.IsFinite(_s.DroneShipLastD) && double.IsFinite(_s.DroneShipRate) && now - _s.DroneShipLastT < 1.0)
        {
            double ship = _s.DroneShipLastD <= 0.0 ? 0.0 : _s.DroneShipRate < 0.0 ? _s.DroneShipLastD / -_s.DroneShipRate : double.NaN;
            seconds = StagingCountdown.Soonest(seconds, StagingCountdown.Now(ship, _s.DroneShipLastT, now));
        }

        if (double.IsFinite(seconds) && vehicle.Parts?.SequenceList is SequenceList list && StagingHooks.NextRowToFire(list) is Sequence next)
            seconds += Math.Max(0.0, StagingConfig.GetSequenceDecouplerDelay(vehicle, next.Number));

        _s.SeparationInS = seconds;
        _s.SeparationTakenAt = now;
    }
}
