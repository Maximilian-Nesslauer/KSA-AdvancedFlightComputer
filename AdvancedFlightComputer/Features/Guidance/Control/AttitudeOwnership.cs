#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using Brutal.Numerics;
using KSA;

/// <summary>
/// Restores attitude settings that still match guidance's last command.
/// </summary>
internal sealed class AttitudeOwnership
{
    private FlightComputer computer;
    private Snapshot original;
    private Snapshot last;
    private bool rollWritten;

    private readonly record struct Snapshot(
        FlightComputerAttitudeMode Mode,
        VehicleReferenceFrame Frame,
        FlightComputerAttitudeTrackTarget Target,
        double3 Custom,
        FlightComputerRollMode Roll)
    {
        public static Snapshot Read(FlightComputer value) => new(
            value.AttitudeMode, value.AttitudeFrame, value.AttitudeTrackTarget,
            value.CustomAttitudeTarget, value.RollMode);
    }

    public void BeginWrite(FlightComputer value)
    {
        if (computer != null)
            return;
        computer = value;
        original = Snapshot.Read(value);
        last = original;
    }

    public bool IsCurrent(FlightComputer value) => computer == null
        || (ReferenceEquals(computer, value) && last.Equals(Snapshot.Read(value)));

    /// <summary>
    /// What differs between the last command guidance wrote and the flight computer now, for the
    /// log line that reports a takeover.
    /// </summary>
    public string DescribeChange(FlightComputer value)
    {
        if (computer == null)
            return "nothing written yet";
        if (!ReferenceEquals(computer, value))
            return "the flight computer instance was replaced";

        Snapshot now = Snapshot.Read(value);
        string change = "";
        if (now.Mode != last.Mode)
            change += $" mode {last.Mode}->{now.Mode}";
        if (now.Frame != last.Frame)
            change += $" frame {last.Frame}->{now.Frame}";
        if (now.Target != last.Target)
            change += $" target {last.Target}->{now.Target}";
        if (!now.Custom.Equals(last.Custom))
            change += $" custom {last.Custom}->{now.Custom}";
        if (now.Roll != last.Roll)
            change += $" roll {last.Roll}->{now.Roll}";
        return change.Length == 0 ? "no difference" : change.Substring(1);
    }

    public void EndWrite(FlightComputer value, bool writesRoll)
    {
        rollWritten |= writesRoll;
        last = Snapshot.Read(value);
    }

    public void Release(FlightComputer value)
    {
        if (computer == null)
            return;
        if (!ReferenceEquals(computer, value))
        {
            computer = null;
            rollWritten = false;
            return;
        }

        // Restore frame, target, and coordinates together because they define one attitude command.
        bool commandUnchanged = value.AttitudeFrame == last.Frame
            && value.AttitudeTrackTarget == last.Target && value.CustomAttitudeTarget.Equals(last.Custom);
        if (commandUnchanged)
        {
            value.AttitudeFrame = original.Frame;
            value.AttitudeTrackTarget = original.Target;
            value.CustomAttitudeTarget = original.Custom;
        }
        else if (value.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.None
            && value.CustomAttitudeTarget.Equals(last.Custom))
        {
            // FlightComputer.UpdateAttitudeTarget treats these values as rotation rates when tracking is None.
            // Clear leftover guidance angles so they cannot command rotation.
            value.CustomAttitudeTarget = default;
        }

        if (commandUnchanged && value.AttitudeMode == last.Mode)
            value.AttitudeMode = original.Mode;
        if (rollWritten && value.RollMode == last.Roll)
            value.RollMode = original.Roll;
        computer = null;
        rollWritten = false;
    }
}
