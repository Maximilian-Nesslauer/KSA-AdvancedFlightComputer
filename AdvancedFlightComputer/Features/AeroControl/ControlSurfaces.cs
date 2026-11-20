namespace AdvancedFlightComputer.Features.AeroControl;

using KSA;
using KSA.Aero;

/// <summary>
/// One movable aerodynamic control surface on a vehicle, as its provider reports it: the part that carries it, a name, its deflection range and its rate. Deflections are in degrees, 0 at rest.
/// </summary>
public sealed class ControlSurface
{
    internal ControlSurface(ControlSurfaces.Provider provider, Part part, string name, double minDeg, double maxDeg, double rateDegS)
    {
        Provider = provider;
        Part = part;
        Name = name;
        MinDeg = minDeg;
        MaxDeg = maxDeg;
        RateDegS = rateDegS;
    }

    internal ControlSurfaces.Provider Provider { get; }

    /// <summary>The part that carries the surface; its identity is the surface's.</summary>
    public Part Part { get; }

    public string Name { get; }
    public double MinDeg { get; }
    public double MaxDeg { get; }
    public double RateDegS { get; }

    /// <summary>The deflection the surface has reached, degrees. Main thread.</summary>
    public double DeflectionDeg => Provider.Deflection(Part);

    /// <summary>Asks for a deflection, degrees, clamped to the range; the provider moves the surface at its rate. Main thread.</summary>
    public void Command(double deg) => Provider.Command(Part, Math.Clamp(deg, MinDeg, MaxDeg));
}

/// <summary>
/// Where AFC learns of a vehicle's aerodynamic control surfaces, whoever provides them. A parts mod registers its surfaces (KSAero's all-moving fins today); when the game has control surfaces of its own, AFC registers a provider for them here too, and nothing that steers with them changes.
///
/// THE CONTRACT, in KSA and .NET types only, so a provider registers by reflection with no reference to AFC's assembly, whatever order the mods load in (<see cref="Register"/>):
///  - collect: the surfaces on a vehicle, as (part, name, min deg, max deg, rate deg/s). Main thread.
///  - deflection: the deflection a surface's part has reached, degrees. Main thread.
///  - command: ask a surface's part for a deflection, degrees. Main thread.
///  - poser: for the aero sweep, which flies a PRIVATE copy of the vehicle's aero mesh off the main thread (KsaAeroSweep). Called on the main thread with the vehicle and the surface's part, it returns a binder, or null where the surface cannot be swept. The binder takes the private mesh - built from AeroMeshSource.Gather of the vehicle's parts, with every surface as Gather saw it - and returns a pose action, or null where the surface is not in that mesh. The action turns the surface in that mesh to a deflection, degrees from where Gather saw it, and brings the mesh's surface flow up to date. Binder and action run on the sweep's thread, on that mesh only.
/// </summary>
public static class ControlSurfaces
{
    internal sealed record Provider(
        string Name,
        Func<Vehicle, IReadOnlyList<(Part Part, string Name, double MinDeg, double MaxDeg, double RateDegS)>> Collect,
        Func<Part, double> Deflection,
        Action<Part, double> Command,
        Func<Vehicle, Part, Func<Aerodynamics, Action<double>?>?> Poser);

    private static readonly object Gate = new();
    private static Provider[] _providers = [];

    /// <summary>
    /// Registers a provider of control surfaces, by its name; a second registration under the same name replaces the first. See the class for the contract. Public, and in KSA and .NET types only, so a mod can call it by reflection.
    /// </summary>
    public static void Register(
        string name,
        Func<Vehicle, IReadOnlyList<(Part Part, string Name, double MinDeg, double MaxDeg, double RateDegS)>> collect,
        Func<Part, double> deflection,
        Action<Part, double> command,
        Func<Vehicle, Part, Func<Aerodynamics, Action<double>?>?> poser)
    {
        ArgumentNullException.ThrowIfNull(name);
        ArgumentNullException.ThrowIfNull(collect);
        ArgumentNullException.ThrowIfNull(deflection);
        ArgumentNullException.ThrowIfNull(command);
        ArgumentNullException.ThrowIfNull(poser);
        var provider = new Provider(name, collect, deflection, command, poser);
        lock (Gate)
        {
            var list = new List<Provider>(_providers.Length + 1);
            foreach (Provider p in _providers)
                if (!string.Equals(p.Name, name, StringComparison.Ordinal))
                    list.Add(p);
            list.Add(provider);
            _providers = list.ToArray();
        }
    }

    /// <summary>Whether any provider is registered.</summary>
    public static bool Any => _providers.Length > 0;

    /// <summary>
    /// The surfaces on a vehicle from every provider, in provider then provider order. A provider that throws is skipped, once logged. Main thread.
    /// </summary>
    internal static void Collect(Vehicle vehicle, List<ControlSurface> into)
    {
        into.Clear();
        if (vehicle == null)
            return;
        foreach (Provider provider in _providers)
        {
            try
            {
                foreach ((Part part, string name, double min, double max, double rate) in provider.Collect(vehicle))
                {
                    if (part == null || !(max > min) || !double.IsFinite(min) || !double.IsFinite(max))
                        continue;
                    into.Add(new ControlSurface(provider, part, name ?? "", min, max, double.IsFinite(rate) && rate > 0.0 ? rate : 0.0));
                }
            }
            catch (Exception e)
            {
                AdvancedFlightComputer.Core.LogHelper.WarnOnce("afc-surfaces-collect:" + provider.Name,
                    $"[AFC] Control surface provider '{provider.Name}' failed to list the surfaces of '{vehicle.Id}': {e.Message}");
            }
        }
    }

    /// <summary>The surface's binder for the aero sweep (see the class), or null. Main thread.</summary>
    internal static Func<Aerodynamics, Action<double>?>? Poser(Vehicle vehicle, ControlSurface surface)
    {
        try
        {
            return surface.Provider.Poser(vehicle, surface.Part);
        }
        catch (Exception e)
        {
            AdvancedFlightComputer.Core.LogHelper.WarnOnce("afc-surfaces-poser:" + surface.Provider.Name,
                $"[AFC] Control surface provider '{surface.Provider.Name}' failed to prepare '{surface.Name}' for the aero sweep: {e.Message}");
            return null;
        }
    }
}
