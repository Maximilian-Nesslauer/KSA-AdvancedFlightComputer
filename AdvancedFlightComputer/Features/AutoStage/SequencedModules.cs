using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// A sequence is a set of ISequenced modules, not of parts: one part can put each of its modules in
// a different row, so Sequence.Parts lists a part once per row any of its modules sits in. Scope is
// the part plus its direct sub-parts, matching Part.ActivateSubtreeInStage.
internal static class SequencedModules
{
    // Yields in the order the game activates them.
    public static SequencedModuleEnumerator InSequence(this Part part, int sequence)
        => new(part, sequence);

    // Each kind names its type, so a third kind the game may add matches neither and fires without a delay.
    public static bool Matches(ISequenced module, DelayKind kind)
        => kind == DelayKind.Engine ? module is EngineController : module is Decoupler;

    public static bool HasEngineIn(Part part, int sequence)
    {
        foreach (ISequenced module in part.InSequence(sequence))
        {
            if (module is EngineController)
                return true;
        }
        return false;
    }

    public static bool LightsEngine(Sequence sequence)
    {
        ReadOnlySpan<Part> parts = sequence.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            if (HasEngineIn(parts[i], sequence.Number))
                return true;
        }
        return false;
    }

    // The tree part, because the settings table lists placeable parts.
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

internal enum DelayKind
{
    Engine,
    Decoupler,
}

// A ref struct like the game's own enumerator, so it cannot be stored or produced by an iterator.
internal ref struct SequencedModuleEnumerator
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
            if (_inner.Current.Sequence == _sequence)
                return true;
        }
        return false;
    }

    public readonly SequencedModuleEnumerator GetEnumerator() => this;
}
