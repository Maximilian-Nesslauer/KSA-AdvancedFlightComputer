#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using Brutal.Numerics;

/// <summary>
/// Everything the sim thread measures each cycle that the SOLVER needs to read - gathered into one immutable object, handed over whole, and never edited in place.
///  WHY THIS EXISTS. These values previously be written straight into the guidance's dynamics parameters whenever the sim thread happened to measure them:
/// SetAccelBias wrote _dyn.Gx/Gy/Gz, SetInertia wrote _dyn.Ixx/Iyy/Izz. That is harmless while one thread does everything, because "between solves" is a real moment. It stops being harmless the instant the solve runs somewhere else, since those exact fields are read inside the linearisation loop - every node, every iteration. A bias update landing halfway through would linearise the first half of a trajectory against one gravity and the second half against another, and then the ratio test would compare a merit computed under one model against a step taken under the other. Nothing would crash. The plans would just be quietly wrong, occasionally, and never the same way twice.
///  So the rule is: a solve reads nothing that is not in its inputs, and its inputs cannot change once it has started. This type is a record precisely so there is no way to edit one - a new measurement produces a new object, and publishing it is a single reference assignment, which is atomic on every runtime we care about. The solve takes its copy at entry and works from that to the end.
///  It costs nothing synchronously; the point is that the synchronous code is then already correct for the threaded case, and the refactor can be verified while everything is still deterministic.
/// </summary>
/// <param name="AccelBias">
/// Unmodelled acceleration in the site frame, m/s^2, added to the model's gravity so the optimiser plans around it. See Ksa6DofGuidance.AccelBias for why this is estimated as a residual rather than attributed to any particular force.
/// </param>
/// <param name="Ixx">Body-x inertia, kg m^2. Live: it tracks propellant drain.</param>
/// <param name="Iyy">Body-y inertia, kg m^2.</param>
/// <param name="Izz">Body-z (roll) inertia, kg m^2.</param>
/// <param name="MomentLimiterPerPa">The game's air moment limiter as measured now (BodyAero6Dof.MomentLimiterPerPa), per Pa; NaN leaves the model's.</param>
/// <param name="MomentBiasPerPa">The air moment's measured error per pascal of dynamic pressure, model body axes, N m/Pa (BodyAero6Dof.MomentBias*PerPa; SixDof's moment-error integrator); zero for none.</param>
public sealed record Ksa6DofInputs(double3 AccelBias, double Ixx, double Iyy, double Izz,
                                   double MomentLimiterPerPa = double.NaN, double3 MomentBiasPerPa = default,
                                   double MomentLimiterStiffnessPerPa = double.NaN, double MomentLimiterDampingPerPaMs = double.NaN,
                                   double LArm = double.NaN, double TipRadiusM = double.NaN, double TipHeightM = double.NaN)
{
    /// <summary>What the craft stands on for the tip-over line (Scvx6DofConfig.TipLimit): the edge's distance from the thrust axis and the centre of mass's height over the contact plane, m; NaN leaves the line loose.</summary>
    public Ksa6DofInputs WithTipGeometry(double radiusM, double heightM) => this with { TipRadiusM = radiusM, TipHeightM = heightM };

    /// <summary>
    /// The engine's lever arm now, m: centre of mass to the dominant gimbal's pivot along the thrust axis (Ksa6DofSetup.EngineArm); NaN or 0 keeps the model's. Live, as the inertia is: the centre of mass moves as the propellant drains.
    ///  WHY (2026-11-27). It was measured when a plan was built and kept. The Starship's nose header tank drains over the landing burn and its centre of mass comes 1.1-1.4 m nearer the engine, so by touchdown the plan's arm was about 6 % long, and the torque the allocator delivered came with that much more side force than the plan had.
    /// </summary>
    public Ksa6DofInputs WithArm(double armM) => this with { LArm = armM };

    public Ksa6DofInputs WithMomentLimiter(double perPa) => this with { MomentLimiterPerPa = perPa };

    public Ksa6DofInputs WithMomentLimiterExact(double perPa, double stiffPerPa, double dampPerPaMs)
        => this with { MomentLimiterPerPa = perPa, MomentLimiterStiffnessPerPa = stiffPerPa, MomentLimiterDampingPerPaMs = dampPerPaMs };

    public Ksa6DofInputs WithMomentBias(double3 perPa) => this with { MomentBiasPerPa = perPa };

    /// <summary>Nothing measured yet: no bias, and whatever inertia the config was built with.</summary>
    public static readonly Ksa6DofInputs None = new(default, 0.0, 0.0, 0.0);

    /// <summary>
    /// True if every field is finite. A single NaN here would propagate through the scale chain into A and P, and SCS is native and does not validate its input - it can take the process down rather than return an error.
    /// </summary>
    public bool IsUsable =>
        double.IsFinite(AccelBias.X) && double.IsFinite(AccelBias.Y) && double.IsFinite(AccelBias.Z) &&
        double.IsFinite(Ixx) && double.IsFinite(Iyy) && double.IsFinite(Izz);

    public Ksa6DofInputs WithBias(double3 bias) => this with { AccelBias = bias };

    public Ksa6DofInputs WithInertia(double ixx, double iyy, double izz) =>
        this with { Ixx = ixx, Iyy = iyy, Izz = izz };
}
