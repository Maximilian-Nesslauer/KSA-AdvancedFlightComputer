using KSA;

namespace AutoStage;

/// <summary>
/// A sequence is a set of ISequenced modules, not of parts. One part can put
/// each of its modules in a different sequence, so Sequence.Parts lists a part
/// once per sequence any of its modules sits in and a part alone never answers
/// what a sequence does. Scope is the part plus its direct sub-parts, matching
/// Part.ActivateSubtreeInStage.
/// </summary>
static class SequencedModules
{
    /// <summary>Yields in the order the game activates them.</summary>
    public static SequencedModuleEnumerator InSequence(this Part part, int sequence)
        => new SequencedModuleEnumerator(part, sequence);

    // Each kind names its type, so a third kind the game may add later matches
    // neither and fires without a delay.
    public static bool Matches(ISequenced module, DelayKind kind)
        => kind == DelayKind.Engine ? module is EngineController : module is Decoupler;

    public static bool HasEngineIn(Part part, int sequence)
    {
        foreach (ISequenced module in part.InSequence(sequence))
        {
            if (module is EngineController) return true;
        }
        return false;
    }

    public static bool LightsEngine(Sequence sequence)
    {
        ReadOnlySpan<Part> parts = sequence.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            if (HasEngineIn(parts[i], sequence.Number)) return true;
        }
        return false;
    }

    // The tree part, not the module's own: the settings table lists placeable
    // parts. Identical on stock content, where no sub-part is sequenced.
    public static string DelayKey(ISequenced module) => module.Parent.FullPart.Template.Id;

    // Same wording as the stock staging window's chip tooltip.
    public static string Describe(ISequenced module)
    {
        string kind = module.GetType().Name;
        return module is ModuleBase { TemplateId.Length: > 0 } moduleBase
            ? $"{kind} '{moduleBase.TemplateId}'"
            : kind;
    }
}

enum DelayKind
{
    Engine,
    Decoupler,
}

// A ref struct because the game's own enumerator is one, so this cannot be
// stored in a field or produced by an iterator method.
ref struct SequencedModuleEnumerator
{
    private Part.SubtreeSequencedModuleEnumerator _inner;
    private readonly int _sequence;

    public SequencedModuleEnumerator(Part part, int sequence)
    {
        _inner = part.GetSubtreeSequencedModules();
        _sequence = sequence;
    }

    public readonly ISequenced Current => _inner.Current;

    public bool MoveNext()
    {
        while (_inner.MoveNext())
        {
            if (_inner.Current.Sequence == _sequence) return true;
        }
        return false;
    }

    // A copy of the unstarted enumerator, the same way the game's does it.
    public readonly SequencedModuleEnumerator GetEnumerator() => this;
}
