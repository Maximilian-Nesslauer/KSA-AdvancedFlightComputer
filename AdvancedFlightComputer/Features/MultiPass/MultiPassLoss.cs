using System;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Robbins/sinc finite-burn loss model shared by the Apse and Hohmann
/// multi-pass UIs. Constant-inertial-attitude steering loss as a
/// fraction of command dV:
///   loss = 1 - sin(phi) / phi
/// with phi = pi * burnRatio half the orbital angle swept during the
/// burn (so theta_total / 2 collapses to pi * burnTime / period for an
/// apse-centred burn). Saturates at 1.0 when burnRatio &gt;= 1 (the burn
/// covers a full orbit and constant-inertial thrust averages to zero);
/// clamped past that point where the sinc approximation starts to
/// oscillate. Reduces to phi^2 / 6 for small phi.
///
/// Honest for apse-centred / Hohmann periapsis kicks (the burn vector
/// is along velocity, steering loss == sinc loss). Plane-change burns
/// at AN / DN are mostly velocity-orthogonal and not strictly tangential,
/// so the steering-loss derivation does not directly apply; the formula
/// is still used as a first-order proxy there but the savings figure is
/// looser than the apse case.
/// </summary>
internal static class MultiPassLoss
{
    public static double FiniteBurnLossFraction(double burnRatio)
    {
        if (burnRatio <= 0.0) return 0.0;
        if (burnRatio >= 1.0) return 1.0;
        double phi = Math.PI * burnRatio;
        return 1.0 - Math.Sin(phi) / phi;
    }
}
