#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using KSA;

/// <summary>
/// Work the guidance step does only because a console picture or page looks at it: the pass strip
/// of the deorbit, the aero model of a booster before its boostback, a fresh resample of that model,
/// and the impact prediction of an idle booster. The draw marks the demand, and the step on the main
/// thread does the work while the mark is younger than a second, so the draw never writes guidance
/// state and nothing runs for a craft nobody looks at.
/// </summary>
internal static class ViewDemand
{
    internal enum Kind { Passes, Aero, Impact }

    private const double LifetimeSeconds = 1.0;
    private static readonly Dictionary<(string Id, Kind Kind), DateTime> _marks = new();
    private static readonly HashSet<string> _resample = new(StringComparer.Ordinal);

    public static void Mark(Vehicle vehicle, Kind kind)
    {
        if (vehicle == null)
            return;
        _marks[(vehicle.Id, kind)] = DateTime.UtcNow.AddSeconds(LifetimeSeconds);
    }

    public static bool Wants(Vehicle vehicle, Kind kind)
        => vehicle != null && _marks.TryGetValue((vehicle.Id, kind), out DateTime until) && DateTime.UtcNow < until;

    /// <summary>A one-time request to sample the aero model of <paramref name="vehicle"/> again, from the RESAMPLE button.</summary>
    public static void RequestResample(Vehicle vehicle)
    {
        if (vehicle != null)
            _resample.Add(vehicle.Id);
    }

    public static bool ConsumeResample(Vehicle vehicle) => vehicle != null && _resample.Remove(vehicle.Id);

    public static void Reset()
    {
        _marks.Clear();
        _resample.Clear();
    }
}
