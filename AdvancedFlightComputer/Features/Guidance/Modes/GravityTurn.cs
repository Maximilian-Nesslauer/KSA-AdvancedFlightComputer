#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

// The gravity-turn ascent: an ascent of its own, separate from the CAT-S / UPFG one in Ascent.cs and ConvexAscent.cs and sharing none of its state. A slot until it is written: its entry in GuidanceModes is not available, so the LAUNCH page does not select it. The mode commits, steps and stops through the hooks it registers with GuidanceWindow.RegisterMode.
public static partial class GuidanceWindow
{
    /// <summary>The two ascents the ASCENT row of the LAUNCH page switches between.</summary>
    public enum AscentMethod { CatsUpfg, GravityTurn }
}
