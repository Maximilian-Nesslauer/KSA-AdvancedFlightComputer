namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>Reference datum the user enters a flyby periapsis against.
/// <see cref="Surface"/> is the altitude above the body's mean radius.
/// <see cref="Center"/> is the radius measured straight from the body center.
/// <see cref="Atmosphere"/> is the altitude above the atmosphere boundary and is
/// only offered for bodies that have one.</summary>
internal enum FlybyReference
{
    Surface,
    Center,
    Atmosphere,
}
