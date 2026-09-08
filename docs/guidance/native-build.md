# Native solver build

Guidance uses two native libraries.
Clarabel is a Rust crate for the G-FOLD planner. SCS is a C library for the six degree of freedom solver.
Neither library is committed as a binary. Both are built from the vendored sources under `third_party/`.

KSA runs on Windows and Linux, so each library has one build per runtime identifier.

| Runtime | Clarabel | SCS |
| --- | --- | --- |
| `win-x64` | `build/native/win-x64/clarabel_c.dll` | `build/native/win-x64/scs.dll` |
| `linux-x64` | `build/native/linux-x64/libclarabel_c.so` | `build/native/linux-x64/libscs.so` |

The file names differ by platform. One mod folder can carry both runtimes, and each platform loads its own files.
`build/GuidanceNative.props` is the single place that defines this layout.

## Toolchains

Both libraries have been built and checked with Rust and Cargo 1.90.0 and Zig 0.14.1, and again with Rust 1.98.1 and Zig 0.16.0.
Install Rust from https://rustup.rs and put Zig on the PATH, or pass `-CargoExe` and `-ZigExe` to the build scripts.
The MSVC Rust target also needs the Visual Studio C++ build tools and the Windows SDK.
CI pins one of those pairs, so a pipeline build uses compilers that a local build can reproduce.

SCS uses Zig instead of the platform C compiler, so its flags stay the same on every host.
`DLONG` and `SFLOAT` stay undefined, and only the Anderson acceleration unit and its BLAS shim define `USE_LAPACK`.
The reasons are in the header of `build/build-scs.ps1`.

## Building

```powershell
pwsh build/build-clarabel.ps1
pwsh build/build-scs.ps1
```

Both scripts build for the host runtime by default. They accept `-Rid win-x64` or `-Rid linux-x64`, plus `-OutDir` for a different output folder.

Zig is a cross compiler, so SCS can target either runtime from either host.
Clarabel names its Rust target explicitly, so a target other than the host one needs `rustup target add <triple>` and a linker for that platform.
The release pipeline builds the Linux library on Linux.

## ABI check

```powershell
dotnet run --project build/NativeChecks -- clarabel AdvancedFlightComputer/Features/Guidance/Gfold/bin/Debug/net10.0/AdvancedFlightComputer.Guidance.Gfold.dll
dotnet run --project build/NativeChecks -- scs AdvancedFlightComputer/Features/Guidance/Scvx/bin/Debug/net10.0/AdvancedFlightComputer.Guidance.Scvx.dll
```

The check compiles `build/NativeChecks/native-layout.c` against the vendored headers. It uses Zig from the PATH unless `--zig <path>` names another executable.
The probe checks 64-bit pointers, 32-bit SCS integers, double precision, and one-byte Clarabel booleans. It then prints every bound structure size and field offset.
The check compares these values with the managed declarations and confirms that the native library exports every bound entry point.
The native library must sit beside the managed assembly. The solver projects do this for the host runtime.

The probe runs as a process of the runtime it describes, so each runtime must be checked on its own platform.
On Windows this checks `win-x64`, and the Linux CI job checks `linux-x64`.

The check does not test numerical behaviour.
The console projects cover that, for example with `--sub-scs` and `--loop` in `AdvancedFlightComputer.Guidance.Tests.Scvx`.
The default `AdvancedFlightComputer.Guidance.Tests.Gfold 81 120` also compares fuel used against dry mass, which a passing ABI check does not cover.

## Packaging

A Release build verifies the archive against the payload list that produced it. A missing or undeclared file fails the build.
A local Release build carries the host runtime only.
A release that carries both runtimes passes them explicitly.

```powershell
dotnet build AdvancedFlightComputer/AdvancedFlightComputer.csproj -c Release -p:GuidanceNativeRids="win-x64;linux-x64"
```

## Current status

The `win-x64` path is built and checked, including the ABI check, the native numerical checks, and a verified release package.
The `linux-x64` path is wired through the scripts, loaders, and payload, but no Linux library has been built or checked yet.
