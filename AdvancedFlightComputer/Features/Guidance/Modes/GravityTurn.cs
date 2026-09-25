#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using KSA;

// The gravity-turn ascent: an ascent of its own, separate from the CAT-S / UPFG one in Ascent.cs and ConvexAscent.cs and sharing none of its state. A placeholder until it is written, so EXECUTE flies nothing while it is selected. Its page is Ui/Gauges/GravityTurnGauge.cs.
public static partial class GuidanceWindow
{
    /// <summary>The two ascents the Ascent tab switches between.</summary>
    public enum AscentMethod { CatsUpfg, GravityTurn }

    private static void ExecuteGravityTurn(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        // Not implemented yet.
    }
}
