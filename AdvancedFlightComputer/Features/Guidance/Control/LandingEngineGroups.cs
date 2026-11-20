using System.Text;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

internal sealed record LandingEngineGroup(string Name, string[] Engines);

internal sealed record ManualEngineChange(string[] Engines, bool? EnableSingle);

internal sealed record LandingEngineGroups(LandingEngineGroup[] Groups, string Selected)
{
    internal static readonly LandingEngineGroups Empty = new([], "");

    internal LandingEngineGroup? Chosen => Array.Find(Groups, group => group.Name == Selected);

    internal const string CraftScope = "";
    internal const int NameLimit = 64;

    internal bool IsValid()
    {
        if (Groups == null || Selected == null)
            return false;
        var names = new HashSet<string>(StringComparer.Ordinal);
        foreach (LandingEngineGroup group in Groups)
        {
            if (group == null || string.IsNullOrWhiteSpace(group.Name) || !names.Add(group.Name)
                || Encoding.UTF8.GetByteCount(group.Name) > NameLimit || group.Name.Any(char.IsControl)
                || group.Engines == null || group.Engines.Length == 0
                || group.Engines.Any(string.IsNullOrWhiteSpace)
                || group.Engines.Distinct(StringComparer.Ordinal).Count() != group.Engines.Length)
                return false;
        }
        return Selected.Length == 0 || names.Contains(Selected);
    }

    internal static string? PartAddress(Part part, Part root)
    {
        var path = new Stack<string>();
        for (Part? current = part; current != null; current = current.TreeParent)
        {
            if (ReferenceEquals(current, root))
            {
                path.Push(Uri.EscapeDataString(root.Template.Id));
                return string.Join("/", path);
            }
            path.Push(AddressStep(current));
        }
        return null;
    }

    // Instance IDs restart on load. The two connector keys normally identify the child.
    // A surface mount uses its position in the vehicle frame (Part.PositionParentAsmb of a full part) because the parent has no socket. Only the editor moves parts, and a split moves the whole child tree, which Rebase reads before the split.
    private static string AddressStep(Part part)
    {
        string type = Uri.EscapeDataString(part.Template.Id);
        foreach (Part.Connector connector in part.Connectors)
        {
            if (connector.Connection is { } connection
                && connection.OtherConnector(connector) is Part.Connector parent
                && ReferenceEquals(parent.Parent, part.TreeParent))
                return $"{type}@{Uri.EscapeDataString(parent.Id)}:{Uri.EscapeDataString(connector.Id)}";
        }
        var p = part.PositionParentAsmb;
        return FormattableString.Invariant($"{type}@{Math.Round(1000 * p.X)}:{Math.Round(1000 * p.Y)}:{Math.Round(1000 * p.Z)}");
    }

    internal static string? EngineAddress(EngineController engine, Part root)
    {
        Part part = engine.Parent;
        string? path = PartAddress(part.FullPart, root);
        if (path == null)
            return null;
        string subpart = part.IsSubPart ? Uri.EscapeDataString(part.Id) : "";
        return $"{path}|{subpart}|{Uri.EscapeDataString(engine.TemplateId)}";
    }

    internal static Dictionary<string, EngineController?> Controllers(Vehicle vehicle, Part root, HashSet<Part>? allowed = null)
    {
        var result = new Dictionary<string, EngineController?>(StringComparer.Ordinal);
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            if (allowed != null && !allowed.Contains(engine.Parent.FullPart))
                continue;
            string? key = EngineAddress(engine, root);
            if (key != null && !result.TryAdd(key, engine))
                result[key] = null;
        }
        return result;
    }

    internal static bool Resolve(LandingEngineGroup group, Dictionary<string, EngineController?> candidates, out EngineController[] engines)
    {
        engines = [];
        var found = new List<EngineController>(group.Engines.Length);
        foreach (string key in group.Engines)
        {
            if (!candidates.TryGetValue(key, out EngineController? engine) || engine == null)
                return false;
            found.Add(engine);
        }
        engines = found.ToArray();
        return engines.Length > 0;
    }

    internal static void Activate(Vehicle vehicle, EngineController[] engines, bool? enableSingle = null)
    {
        if (enableSingle is bool enable)
        {
            engines[0].SetIsActive(vehicle, enable);
            return;
        }

        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            bool active = Array.IndexOf(engines, engine) >= 0;
            if (active != engine.IsActive)
                engine.SetIsActive(vehicle, active);
        }
    }

    internal LandingEngineGroups Rebase(Dictionary<string, EngineController?> beforeSplit, PartTree newTree)
    {
        return this with
        {
            Groups = Groups.Select(group => group with { Engines = group.Engines.Select(RebaseEngine).ToArray() }).ToArray(),
        };

        string RebaseEngine(string key)
        {
            if (beforeSplit.TryGetValue(key, out EngineController? previous)
                && previous != null && ReferenceEquals(previous.Parent.Tree, newTree))
            {
                foreach (EngineController engine in previous.Parent.Modules.Get<EngineController>())
                    if (engine.TemplateId == previous.TemplateId && EngineAddress(engine, newTree.Root) is { } address)
                        return address;
            }
            // Preserve the requested group for a visible refusal if a part did not reach the child.
            return "missing|" + key;
        }
    }

    internal static bool HasRetainedFeeds(Vehicle vehicle, EngineController[] engines, HashSet<Part> parts)
    {
        ReadOnlySpan<MoleState> moles = vehicle.Parts.Moles.States;
        var reserveTanks = new HashSet<Tank>();
        foreach (Part part in parts)
            foreach (Tank tank in part.SubtreeModules.Get<Tank>())
                if (tank.RoleAffinity == ConsumerRole.Engine && tank.PropellantUseEnabled && tank.ComputeSubstanceMass(moles) > 0)
                    reserveTanks.Add(tank);
        if (reserveTanks.Count == 0)
            return false;

        foreach (EngineController engine in engines)
        foreach (Rocket rocket in engine.Rockets)
        foreach (RocketCore core in rocket.Cores)
        {
            if (core is not PlumbedCore plumbed || plumbed.ResourceManager is not { } manager)
                return false;
            FlowOrder<Tank> order = manager.ConsumptionOrder;
            var reachable = new HashSet<Tank>();
            for (int i = 0; i < order.LevelCount; i++)
                foreach (Tank tank in order[i])
                    if (tank != null && reserveTanks.Contains(tank))
                        reachable.Add(tank);
            // The reserve is a mass on the whole stage, so each selected engine must be able to draw from that pool.
            if (!reachable.SetEquals(reserveTanks))
                return false;
            foreach (Reactant reactant in plumbed.Mix.Reactants)
            {
                bool fed = false;
                foreach (Tank tank in reachable)
                    if (tank.TryGetMoleAndState(reactant.SubstancePhase, moles, out var mole) && mole.State.Mass > 0)
                    {
                        fed = true;
                        break;
                    }
                if (!fed)
                    return false;
            }
        }
        return true;
    }
}
