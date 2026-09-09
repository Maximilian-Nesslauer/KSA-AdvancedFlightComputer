#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using KSA;

public static partial class GuidanceWindow
{
    private const double LandingEngineNoProgressLimit = 3.0 * SequenceCooldown;
    private const double LandingEngineWaitLimit = 10.0 * SequenceCooldown;
    private const double LandingLastSequenceGrace = 2.0 * SequenceCooldown;

    private static bool PrepareLandingEngines(Vehicle vehicle, IParentBody parent, double now, bool requireAirless)
    {
        if (requireAirless && parent?.GetAtmosphereReference()?.Physical != null)
            return RefuseLandingEngines("G-FOLD is limited to airless bodies. Use terminal hover in atmosphere.");
        if (KsaEnginePerf.GetThrottleControlStatus(vehicle) == KsaEnginePerf.ThrustStatus.UnsupportedEngine)
            return RefuseLandingEngines("Landing control requires liquid engines with shutdown control.");

        // A wait that is already over must not buy one more staging action on its way out,
        // because staging cannot be taken back once the sequence fires.
        if (double.IsFinite(_s.LandingEngineWaitStart)
            && (!double.IsFinite(now) || LandingEngineWaitExpired(now)))
            return RefuseLandingEngines("Landing engine wait timed out without usable thrust.");

        if (_s.Engage && _s.AutoStage)
            AutoSequence(vehicle);

        KsaEnginePerf.ThrustStatus status = KsaEnginePerf.GetThrottleControlStatus(vehicle);
        if (status == KsaEnginePerf.ThrustStatus.UnsupportedEngine)
            return RefuseLandingEngines("Landing control requires liquid engines with shutdown control.");
        if (status == KsaEnginePerf.ThrustStatus.Available)
        {
            if (double.IsFinite(_s.LandingEngineWaitStart))
                _s.LandingStatus = "";
            ResetLandingEngineWait();
            return true;
        }

        ClearLandingEngineCommand();
        Sequence next = null;
        if (vehicle.Parts?.SequenceList != null)
            foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
                if (!sequence.Activated)
                {
                    next = sequence;
                    break;
                }

        // Engine activation is buffered, so allow time for the last sequence to take effect.
        bool settlingLastSequence = next == null && now >= _s.LastSequenceTime
            && now - _s.LastSequenceTime < LandingLastSequenceGrace;
        if (!_s.Engage || !_s.AutoStage || !(_s.StagingActive || settlingLastSequence) || !double.IsFinite(now))
            return RefuseLandingEngines("Landing control has no active, supplied liquid engine.");

        if (!double.IsFinite(_s.LandingEngineWaitStart))
        {
            _s.LandingEngineWaitStart = now;
            _s.LandingEngineWaitProgressTime = now;
            _s.LandingEngineWaitNextSequence = next;
        }
        else if (!ReferenceEquals(next, _s.LandingEngineWaitNextSequence))
        {
            _s.LandingEngineWaitNextSequence = next;
            _s.LandingEngineWaitProgressTime = now;
        }

        // Only a change in the next sequence counts as progress.
        // Retries must not extend the total wait.
        if (LandingEngineWaitExpired(now))
            return RefuseLandingEngines("Landing engine wait timed out without usable thrust.");
        if (next == null && !settlingLastSequence)
            return RefuseLandingEngines("The last sequence did not provide a usable landing engine.");

        _s.LandingStatus = "Waiting for staging to supply a landing engine.";
        return false;
    }

    private static void ClearLandingEngineCommand()
    {
        _s.GfoldPlan = null;
        _s.GfoldThrottle = 0.0;
        _s.GfoldEngineOn = false;
        _s.GfoldTrackInit = false;
        _s.GfoldLastSolveTime = double.NegativeInfinity;
        _s.GfoldThrustStatus = "";
        _s.TermInit = false;
        _s.TermPidUp = _s.TermPidE = _s.TermPidN = default;
        _s.HasCommand = false;
        _s.LandingCutPending = true;
    }

    private static bool RefuseLandingEngines(string reason)
    {
        ClearLandingEngineCommand();
        ResetLandingEngineWait();
        _s.LandingPhase = LandingPhase.Done;
        _s.LandingStatus = reason;
        return false;
    }

    private static bool LandingEngineWaitExpired(double now)
        => now < _s.LandingEngineWaitStart
        || now - _s.LandingEngineWaitStart >= LandingEngineWaitLimit
        || now - _s.LandingEngineWaitProgressTime >= LandingEngineNoProgressLimit;

    private static void ResetLandingEngineWait()
    {
        _s.LandingEngineWaitStart = double.NaN;
        _s.LandingEngineWaitProgressTime = 0.0;
        _s.LandingEngineWaitNextSequence = null;
    }
}
