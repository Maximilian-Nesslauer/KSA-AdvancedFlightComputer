using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Loader;

// P/Invoke layout errors fail silently. This check compiles a probe against the
// solver headers, compares the native layout with the managed declarations, and
// verifies that all bound entry points are exported.
//
// The probe must run on the runtime it describes. Numerical behaviour is checked by
// the console projects under AdvancedFlightComputer.Guidance.Tests.

const string usage = "Usage: NativeChecks <clarabel|scs> <assembly-path> "
    + "[--rid <win-x64|linux-x64>] [--zig <path>] [--scs-include <dir>] [--clarabel-include <dir>]";

if (args.Length < 2 || args[0] is not ("scs" or "clarabel"))
{
    Console.Error.WriteLine(usage);
    return 2;
}

string repoRoot = Assembly.GetExecutingAssembly()
    .GetCustomAttributes<AssemblyMetadataAttribute>()
    .Single(attribute => attribute.Key == "RepoRoot").Value!;

string library = args[0];
bool scs = library == "scs";
string assemblyPath = Path.GetFullPath(args[1]);
string rid = HostRid();
string zig = "zig";
string scsInclude = Path.Combine(repoRoot, "third_party", "scs", "include");
string clarabelInclude = Path.Combine(repoRoot, "third_party", "clarabel", "include", "c");

for (int i = 2; i < args.Length; i += 2)
{
    if (i + 1 >= args.Length)
    {
        Console.Error.WriteLine(usage);
        return 2;
    }
    switch (args[i])
    {
        case "--rid": rid = args[i + 1]; break;
        case "--zig": zig = args[i + 1]; break;
        case "--scs-include": scsInclude = args[i + 1]; break;
        case "--clarabel-include": clarabelInclude = args[i + 1]; break;
        default:
            Console.Error.WriteLine(usage);
            return 2;
    }
}

try
{
    (string zigTarget, string clarabelName, string scsName) = Runtime(rid);
    if (rid != HostRid())
        throw new InvalidOperationException(
            $"The layout probe has to run as a {rid} process, so run this check on {rid}.");

    string workDir = Path.Combine(AppContext.BaseDirectory, "probe");
    Directory.CreateDirectory(workDir);
    string probe = Path.Combine(workDir, OperatingSystem.IsWindows() ? "native-layout.exe" : "native-layout");
    string probeSource = Path.Combine(repoRoot, "build", "NativeChecks", "native-layout.c");

    Run(zig, ["cc", "-target", zigTarget, "-std=c11", "-I" + scsInclude, "-I" + clarabelInclude, probeSource, "-o", probe], workDir,
        "The check compiles its probe with Zig. Put zig on the PATH or name it with --zig, see docs/guidance/native-build.md.");
    Dictionary<string, int> layout = Run(probe, [], workDir)
        .Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
        .Where(line => line.Contains('='))
        .Select(line => line.Split('='))
        .ToDictionary(parts => parts[0], parts => int.Parse(parts[1]));

    string directory = Path.GetDirectoryName(assemblyPath)!;
    AssemblyLoadContext.Default.Resolving += (_, name) =>
    {
        string path = Path.Combine(directory, name.Name + ".dll");
        return File.Exists(path) ? AssemblyLoadContext.Default.LoadFromAssemblyPath(path) : null;
    };
    Assembly core = AssemblyLoadContext.Default.LoadFromAssemblyPath(assemblyPath);
    Type binding = core.GetTypes().Single(type => type.Name == (scs ? "ScsNative" : "ClarabelNative"));

    int checks = 0;
    void Equal(string key, int actual)
    {
        if (!layout.TryGetValue(key, out int expected))
            throw new InvalidOperationException($"The probe printed no value for {key}, so the binding has a type the headers do not.");
        if (actual != expected)
            throw new InvalidOperationException($"ABI mismatch for {key}, managed {actual}, native {expected}");
        checks++;
    }

    foreach (Type type in binding.GetNestedTypes(BindingFlags.NonPublic)
        .Where(type => type.IsValueType && !type.IsEnum && !type.IsDefined(typeof(CompilerGeneratedAttribute))))
    {
        Equal(type.Name + ".size", Marshal.SizeOf(type));
        foreach (FieldInfo field in type.GetFields(BindingFlags.Public | BindingFlags.Instance))
            Equal(type.Name + "." + field.Name, checked((int)Marshal.OffsetOf(type, field.Name)));
    }
    Console.WriteLine($"{library} ABI passed on {rid}, {checks} size and field-offset checks");

    string nativePath = Path.Combine(directory, scs ? scsName : clarabelName);
    if (!File.Exists(nativePath))
        throw new FileNotFoundException($"{nativePath} is missing, so build it first and put it beside the managed assembly.");

    nint handle = NativeLibrary.Load(nativePath);
    try
    {
        int exports = 0;
        foreach (MethodInfo method in binding.GetMethods(BindingFlags.Static | BindingFlags.NonPublic))
        {
            if (method.GetCustomAttribute<DllImportAttribute>() is not { } import) continue;
            string entry = import.EntryPoint ?? method.Name;
            if (!NativeLibrary.TryGetExport(handle, entry, out _))
                throw new EntryPointNotFoundException($"{Path.GetFileName(nativePath)} does not export {entry}");
            exports++;
        }
        Console.WriteLine($"{library} exports passed, {exports} bound entry points");
    }
    finally
    {
        NativeLibrary.Free(handle);
    }
    return 0;
}
catch (Exception error)
{
    Console.Error.WriteLine(error.Message);
    return 1;
}

static string HostRid() => OperatingSystem.IsWindows() ? "win-x64" : "linux-x64";

static (string ZigTarget, string Clarabel, string Scs) Runtime(string rid) => rid switch
{
    "win-x64" => ("x86_64-windows-gnu", "clarabel_c.dll", "scs.dll"),
    "linux-x64" => ("x86_64-linux-gnu", "libclarabel_c.so", "libscs.so"),
    _ => throw new ArgumentException($"Unknown runtime identifier {rid}"),
};

static string Run(string file, string[] arguments, string workDir, string? missingHint = null)
{
    ProcessStartInfo start = new(file)
    {
        RedirectStandardOutput = true,
        RedirectStandardError = true,
    };
    foreach (string argument in arguments)
        start.ArgumentList.Add(argument);
    // Keep compiler caches in the build output instead of the user's shared cache.
    start.Environment["ZIG_GLOBAL_CACHE_DIR"] = Path.Combine(workDir, "zig-cache");
    start.Environment["ZIG_LOCAL_CACHE_DIR"] = Path.Combine(workDir, "zig-local-cache");

    Process process;
    try
    {
        process = Process.Start(start) ?? throw new InvalidOperationException($"Could not start {file}.");
    }
    catch (System.ComponentModel.Win32Exception)
    {
        throw new InvalidOperationException(missingHint ?? $"Could not start {file}.");
    }
    using Process owned = process;
    Task<string> output = process.StandardOutput.ReadToEndAsync();
    Task<string> error = process.StandardError.ReadToEndAsync();
    process.WaitForExit();
    if (process.ExitCode != 0)
        throw new InvalidOperationException(
            $"{Path.GetFileName(file)} failed with exit code {process.ExitCode}. {error.GetAwaiter().GetResult()}");
    return output.GetAwaiter().GetResult();
}
