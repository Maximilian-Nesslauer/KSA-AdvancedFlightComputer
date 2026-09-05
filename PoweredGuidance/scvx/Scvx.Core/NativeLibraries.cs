using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Scvx;

/// <summary>
/// Single DllImportResolver registration for the whole assembly.
///
/// NativeLibrary.SetDllImportResolver THROWS if called twice for the same
/// assembly, so every native library P/Invoke'd from Scvx.Core must be
/// dispatched from this one resolver — a second [ModuleInitializer] registering
/// its own is an InvalidOperationException at load, which is how this file came
/// to exist. Add new natives to the switch below, never as a new initializer.
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
                "scs" => "scs.dll",
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
