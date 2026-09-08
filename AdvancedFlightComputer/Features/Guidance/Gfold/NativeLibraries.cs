using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AdvancedFlightComputer.Guidance.Gfold;

/// <summary>
/// Single DllImportResolver registration for the whole assembly.
///
/// NativeLibrary.SetDllImportResolver THROWS if called twice for the same assembly, so
/// every native library P/Invoke'd from this assembly must be dispatched from this one
/// resolver. A second [ModuleInitializer] registering its own is an
/// InvalidOperationException at assembly load - before Main, with a
/// TypeInitializationException for &lt;Module&gt; as the only clue - which is exactly
/// what adding a second native binding once did, and how this file came to exist.
/// (The Scvx project has the same file for the same reason.) Add new natives to the
/// switch, never as a new initializer.
///
/// Resolving from next to this assembly rather than the process working directory is
/// what lets the same assembly work in the console runner, a test host, and the game
/// with the mod's DLLs in the mod folder.
///
/// KSA ships for Windows and for Linux, so one mod folder can hold the Windows build
/// and the Linux build of the same library. Their file names differ, so both can sit
/// beside each other and the running platform loads only its own.
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
