namespace AdvancedFlightComputer.Core;

/// <summary>Copy of the ModMenu attribute. ModMenu matches the type and property names, so they must not change.</summary>
[AttributeUsage(AttributeTargets.Method, AllowMultiple = false)]
internal sealed class ModMenuEntryAttribute : Attribute
{
    public ModMenuEntryAttribute(string menuName, string? isModMenuActivePropertyName = null)
    {
        MenuName = menuName;
        IsModMenuActivePropertyName = isModMenuActivePropertyName;
    }

    public string MenuName { get; }

    public string? IsModMenuActivePropertyName { get; }
}
