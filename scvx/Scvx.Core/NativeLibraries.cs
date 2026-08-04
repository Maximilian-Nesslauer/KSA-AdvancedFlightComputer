using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Scvx;

/// <summary>
/// Single DllImportResolver registration for the whole assembly.
///
/// NativeLibrary.SetDllImportResolver throws if called twice for the same
/// assembly — so ECOS and SCS, both P/Invoke'd from Scvx.Core, cannot each
/// register their own resolver in their own ModuleInitializer the way each did
/// independently at first. One resolver here dispatches by requested library
/// name and loads whichever native DLL sits beside this assembly.
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
                "ecos" => "ecos.dll",
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
