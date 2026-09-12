using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Everything the detector knows about one vehicle. Created on first contact through the gauge,
// the settings page, a guidance request or a dispose, and dropped when the vehicle is disposed.
internal sealed class StagingState
{
    internal enum Phase { Monitoring, AwaitingIgnition, AwaitingPropagation }

    internal enum SpentDropBlocker
    {
        None,
        Unreported,
        NoJettison,
        NothingSpent,
        InactiveInside,
        FueledInside,
        BrokenInside,
        NothingThrusting,
        BurnComplete,
        CarriesPropellant,
    }

    // The player's or guidance's switch for this vehicle. Not persisted, like the stock RCS toggle.
    public bool Active;

    // A one-row staging asked for by a caller that has its own cue, consumed by the next evaluation.
    public bool Requested;

    public Phase State = Phase.Monitoring;
    public int PropagationFrames;
    public double SpentJettisonSince = double.NaN;
    // Unreported rather than None, because None is the armed state and would suppress the first arm line.
    public SpentDropBlocker SpentJettisonBlockerReported = SpentDropBlocker.Unreported;
    public FlightComputerBurnMode TriggeredMode;
    public PendingStaging? Pending;

    // Rows this detector activated on the vehicle, so a caller can see its own request land.
    public int Activations;

    // The control-module guard refused the next row. Latched, so the refusal is reported once
    // per row and a caller can show why nothing happens.
    public bool HeldForControl;

    // Taken before the solver results reach the universe and consumed afterwards; the flag makes
    // sure one sample is never evaluated twice.
    public bool Sampled;
    public FlightComputerBurnMode SampledBurnMode;
    public bool SampledHadPropellant;

    // Whether an unactivated row still lights an engine, refreshed on sequence activation.
    public int NextEngineGeneration = -1;
    public bool NextEngineSequence;

    // The parts the next row would shed, refreshed when the tree or the sequence list changes.
    public readonly HashSet<Part> JettisonSet = new();
    public int JettisonGeneration = -1;
    public int JettisonPartCount = -1;
    public int JettisonSequenceNumber = -1;
    public bool JettisonValid;

    public void ResetDwell()
    {
        SpentJettisonSince = double.NaN;
        SpentJettisonBlockerReported = SpentDropBlocker.Unreported;
    }
}
