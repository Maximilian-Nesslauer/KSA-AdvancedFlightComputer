using AdvancedFlightComputer.Features.AutoStage;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

// Main-thread snapshot. Stock's attached set is taken after its sequence jettison and
// before draining the row. The shrouds are inert, so only that row's mass changes.
internal sealed record FairingDrainSnapshot(int Row, (Part Part, Part Mount, double Mass)[] Parts)
{
    internal static FairingDrainSnapshot? Capture(Vehicle vehicle, bool automatic, out string reason)
    {
        reason = "";
        if (!automatic || vehicle.Parts == null)
            return null;
        var rows = FairingJettison.Rows(vehicle, StagingDetector.StateOf(vehicle)).Where(r => !r.Mixed).ToArray();
        if (rows.Length == 0)
            return null;
        if (rows.Length != 1)
        {
            reason = "Multiple independent fairing rows use the stock mass model.";
            return null;
        }
        var leaves = rows[0].Leaves.Select(leaf => (leaf.Shroud, leaf.Mount.Parent.FullPart,
            (double)leaf.Shroud.ComputeSubtreeInertMass())).ToArray();
        if (leaves.Length == 0 || leaves.Any(leaf => !double.IsFinite(leaf.Item3) || leaf.Item3 <= 0.0))
        {
            reason = "The attached shroud mass could not be measured.";
            return null;
        }
        return new FairingDrainSnapshot(rows[0].Number, leaves);
    }

    internal double MassExcludedByStock(IReadOnlySet<Part> attached)
        => Parts.Where(leaf => attached.Contains(leaf.Mount) && !attached.Contains(leaf.Part)).Sum(leaf => leaf.Mass);

    // If that mount later stages with the stack, its shroud cannot be retained past that row.
    internal bool MountsStay(IReadOnlySet<Part> attached)
        => Parts.All(leaf => attached.Contains(leaf.Mount));
}
