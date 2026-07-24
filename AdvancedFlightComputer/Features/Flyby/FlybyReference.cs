namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>Reference datum the user enters a flyby periapsis against.
/// <see cref="Surface"/>: altitude above the body's mean radius.
/// <see cref="Center"/>: radius measured straight from the body center.
/// <see cref="Atmosphere"/>: altitude above the atmosphere boundary (only
/// offered for bodies that have an atmosphere).</summary>
internal enum FlybyReference
{
    Surface,
    Center,
    Atmosphere,
}
