using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// Where an ascent hands over to UPFG. The solved climb hands over at UPFG FROM and the gravity turn by its failsafe
/// altitude at the latest, but neither may wait above three quarters of the target periapsis, on any body. A target
/// orbit lower than the hand-over altitude would otherwise be reached without ever handing over, as on the Moon, where
/// a 30 km orbit sits far under the 80 km of UPFG FROM.
/// </summary>
internal static class AscentHandover
{
    /// <summary>The share of the target periapsis height above which no ascent waits to hand over.</summary>
    public const double PeShare = 0.75;

    /// <summary>
    /// The hand-over altitude in km: <paramref name="settingKm"/>, or <see cref="PeShare"/> of the lower of the two target
    /// apsides when that is lower. The two are taken as they stand, because a periapsis above the apoapsis swaps them.
    /// A target with no height above the surface leaves the setting alone.
    /// </summary>
    public static double AltitudeKm(double settingKm, double peKm, double apKm)
    {
        double cap = PeShare * Math.Min(peKm, apKm);
        if (!double.IsFinite(cap) || cap <= 0.0)
            return settingKm;
        return double.IsFinite(settingKm) ? Math.Min(settingKm, cap) : cap;
    }

    /// <summary>Whether <see cref="AltitudeKm"/> is lower than the setting, so a readout can say why.</summary>
    public static bool CappedByPe(double settingKm, double peKm, double apKm) => AltitudeKm(settingKm, peKm, apKm) < settingKm;
}
