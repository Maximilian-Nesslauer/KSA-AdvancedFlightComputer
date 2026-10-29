using System.Reflection;
using System.Runtime.Loader;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.HarnessTests.Framework;

namespace AdvancedFlightComputer.HarnessTests;

// Replays the ModMenu lookup of the AFC entry, because the harness has no menu bar for ModMenu to scan.
public sealed class ModMenuContractTest : AfcTest
{
    private const BindingFlags AllMethods = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static | BindingFlags.Instance;
    private const BindingFlags AnyStatic = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static;

    public override string Name => "afc-modmenu-contract";

    protected override void Execute(TestContext t)
    {
        Assembly afc = typeof(ModMenuBridge).Assembly;

        // ModMenu skips the whole assembly when GetTypes throws.
        Type[] types;
        try
        {
            types = afc.GetTypes();
            t.Check("AFC assembly types load", true);
        }
        catch (ReflectionTypeLoadException ex)
        {
            t.Fail("AFC assembly types load", ex.LoaderExceptions.FirstOrDefault()?.Message ?? ex.Message);
            return;
        }

        // ModMenu skips a type whose attributes throw.
        List<(MethodInfo Method, object Attribute)> entries = [];
        List<string> unreadable = [];
        foreach (Type type in types)
        {
            try
            {
                foreach (MethodInfo method in type.GetMethods(AllMethods))
                {
                    object? attribute = method.GetCustomAttributes(false)
                        .FirstOrDefault(a => a.GetType().Name == "ModMenuEntryAttribute");
                    if (attribute != null)
                        entries.Add((method, attribute));
                }
            }
            catch (Exception ex)
            {
                unreadable.Add($"{type.FullName}: {ex.Message}");
            }
        }
        t.Check("every method's attributes read", unreadable.Count == 0, string.Join("; ", unreadable));

        MethodInfo? expected = typeof(ModMenuBridge).GetMethod(nameof(ModMenuBridge.DrawEntry), AnyStatic);
        if (!t.Check("exactly one ModMenu entry, on ModMenuBridge.DrawEntry",
                entries.Count == 1 && entries[0].Method == expected,
                string.Join(", ", entries.Select(e => $"{e.Method.DeclaringType?.FullName}.{e.Method.Name}"))))
            return;

        (MethodInfo entry, object attr) = entries[0];
        string? menuName = attr.GetType().GetProperty("MenuName")?.GetValue(attr) as string;
        t.Check("MenuName reads by name", menuName == ModMenuBridge.MenuName, menuName ?? "null");

        string? flagName = attr.GetType().GetProperty("IsModMenuActivePropertyName")?.GetValue(attr) as string;
        if (!t.Check("flag name reads by name", !string.IsNullOrEmpty(flagName)))
            return;

        PropertyInfo? flag = entry.DeclaringType!.GetProperty(flagName!, AnyStatic);
        if (!t.Check("flag is a writable static bool property",
                flag != null && flag.PropertyType == typeof(bool) && flag.CanWrite))
            return;

        CheckLatch(t, flag!);

        bool bound;
        try
        {
            bound = Delegate.CreateDelegate(typeof(Action), entry) is Action;
        }
        catch (Exception)
        {
            bound = false;
        }
        t.Check("entry binds as a static Action", bound);

        CheckReservedNames(t, afc, types);
    }

    // The flag must only turn on. The session value is put back.
    private static void CheckLatch(TestContext t, PropertyInfo flag)
    {
        FieldInfo? field = typeof(ModMenuBridge).GetField("_active", AnyStatic);
        if (!t.Check("latch field exists", field != null))
            return;

        object? saved = field!.GetValue(null);
        try
        {
            flag.SetValue(null, true);
            t.Check("flag write true sets Active", ModMenuBridge.Active);
            flag.SetValue(null, false);
            t.Check("flag write false keeps Active", ModMenuBridge.Active);
        }
        finally
        {
            field.SetValue(null, saved);
        }
    }

    // ModMenu also calls these names in every assembly it scans, including the guidance assemblies.
    private static void CheckReservedNames(TestContext t, Assembly afc, Type[] afcTypes)
    {
        AssemblyLoadContext? context = AssemblyLoadContext.GetLoadContext(afc);
        List<AssemblyName> shipped = afc.GetReferencedAssemblies()
            .Where(n => n.Name != null && n.Name.StartsWith("AdvancedFlightComputer.", StringComparison.Ordinal))
            .ToList();
        if (!t.Check("guidance assemblies are referenced", context != null && shipped.Count > 0))
            return;

        List<string> offenders = ReservedNameMethods(afcTypes);
        foreach (AssemblyName name in shipped)
        {
            Type?[] types;
            try
            {
                types = context!.LoadFromAssemblyName(name).GetTypes();
            }
            catch (ReflectionTypeLoadException ex)
            {
                types = ex.Types;
            }
            offenders.AddRange(ReservedNameMethods(types));
        }
        t.Check("no public static CreateModMenu or DrawModMenu", offenders.Count == 0, string.Join(", ", offenders));
    }

    private static List<string> ReservedNameMethods(IEnumerable<Type?> types)
    {
        List<string> found = [];
        foreach (Type? type in types)
        {
            if (type == null)
                continue;
            foreach (MethodInfo method in type.GetMethods(AllMethods))
            {
                if (method.IsStatic && method.IsPublic && (method.Name == "CreateModMenu" || method.Name == "DrawModMenu"))
                    found.Add($"{type.FullName}.{method.Name}");
            }
        }
        return found;
    }
}
