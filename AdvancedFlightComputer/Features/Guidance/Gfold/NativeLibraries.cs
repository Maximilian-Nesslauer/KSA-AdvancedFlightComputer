using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AdvancedFlightComputer.Guidance.Gfold;

/// <summary>
/// Single DllImportResolver registration for the whole assembly.
///
/// NativeLibrary.SetDllImportResolver throws if called twice for the same assembly, so every native library called through P/Invoke from this assembly must use this resolver. A second [ModuleInitializer] would cause an InvalidOperationException during assembly load. Add new native libraries to the switch, never as a new initializer.
///
/// Resolving from next to this assembly rather than the process working directory lets the same assembly work in the console runner, a test host, and the game.
///
/// KSA ships for Windows and Linux, so one mod folder can hold both builds of the same library. Their file names differ, and the running platform loads only its own.
/// </summary>
internal static class NativeLibraries
{
#pragma warning disable CA2255
    [ModuleInitializer]
#pragma warning restore CA2255
    internal static void Init()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeLibraries).Assembly, (name, asm, _) =>
        {
            string? fileName = name switch
            {
                "clarabel_c" => OperatingSystem.IsWindows() ? "clarabel_c.dll" : "libclarabel_c.so",
                _ => null,
            };
            if (fileName == null)
                return IntPtr.Zero;
            string dir = Path.GetDirectoryName(asm.Location) ?? ".";
            string candidate = Path.Combine(dir, fileName);
            return NativeLibrary.TryLoad(candidate, out IntPtr handle) ? handle : IntPtr.Zero;
        });
    }
}
