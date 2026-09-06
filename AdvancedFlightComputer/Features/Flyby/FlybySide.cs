namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>Which side of the target body the flyby periapsis is placed on,
/// named in the target's own orbital frame. <see cref="Inner"/> and
/// <see cref="Outer"/> lie along the target's radius from its parent, toward it
/// and away from it. <see cref="North"/> and <see cref="South"/> lie along the
/// target's orbit normal.
///
/// The aim offset must stay perpendicular to the approach relative velocity, so a
/// requested side is only reachable when its axis is not nearly parallel to that
/// velocity. For a Hohmann style arrival the relative velocity runs roughly along
/// the target's track, which is why there is no leading or trailing option.</summary>
internal enum FlybySide
{
    Inner,
    Outer,
    North,
    South,
}
