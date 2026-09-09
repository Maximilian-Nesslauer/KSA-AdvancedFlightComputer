# Native solver build

Guidance builds Clarabel and SCS from the sources in `third_party/`.
Clarabel serves the G-FOLD planner, and SCS serves the six degree of freedom solver.

| Runtime | Clarabel | SCS |
| --- | --- | --- |
| `win-x64` | `build/native/win-x64/clarabel_c.dll` | `build/native/win-x64/scs.dll` |
| `linux-x64` | `build/native/linux-x64/libclarabel_c.so` | `build/native/linux-x64/libscs.so` |

`build/GuidanceNative.props` defines the output layout.
A mod package can carry both runtimes, with each platform loading its own libraries.

## Build and test locally

Run these commands from the repository root in PowerShell 7.
The installer downloads Rust 1.90.0 and Zig 0.14.1, verifies their hashes, and returns their executable paths.

```powershell
$toolsRoot = Join-Path ([IO.Path]::GetTempPath()) "afc-guidance-tools"
$nativeTools = ./build/ci/install-native-tools.ps1 -InstallRoot $toolsRoot -RuntimeIdentifier win-x64
./build/ci/test-guidance.ps1 -RepositoryRoot $PWD -RuntimeIdentifier win-x64 -CargoExe $nativeTools.CargoExe -ZigExe $nativeTools.ZigExe
```

On Linux, use `linux-x64` in both commands.
Windows also needs the Visual Studio C++ build tools and Windows SDK for Rust's MSVC target.
Linux needs a GNU linker.

The check builds both native libraries and seven game-free managed projects, then checks native layouts and exports against the C# bindings.
It runs each Scvx check (`--fd`, `--aero`, `--impact`, `--sub-scs`, `--loop`) and `Worker.Tests`, stopping on failure.
The default `Gfold 81 120` example is not a CI requirement because its fuel comparison is an example policy.

To build a single library with tools already on PATH, run:

```powershell
./build/build-clarabel.ps1 -Rid win-x64
./build/build-scs.ps1 -Rid win-x64
```

Both scripts default to the host runtime and accept `-OutDir` for a different output directory.
Zig can cross-compile SCS.
Clarabel cross builds need the selected Rust target and a suitable linker.
ABI checks must run on the target platform.

## Public CI

`.github/workflows/guidance-native.yml` runs on pull requests, pushes to `main` and `powered-guidance`, and manual dispatch.
Its Windows and Linux jobs run the same checks and upload exactly two native libraries per runtime.
Missing, empty, or extra artifact files fail the job.
The jobs use read-only repository access and need no game files or private tokens.

Actions are pinned to commits.
`install-native-tools.ps1` holds the Rust and Zig versions and hashes, including the Rust manifest used to verify compiler components.
`global.json` selects .NET SDK 10.0.400 and allows later patches in the same feature band.
`NuGet.config` selects nuget.org without inherited feeds.

CI uses locked Cargo and NuGet restores to reject unexpected dependency changes.
Each `packages.lock.json` covers both runtimes declared in `Directory.Build.props`.
Restore must run without `--runtime`, then build with `--no-restore --runtime <rid>`.
Passing `--runtime` to restore narrows the runtime set and causes NU1004 in locked mode.
After an intended package change, run a plain restore and review the updated lock file with the change.

The ABI check also requires fixed counts to catch removed bindings that would otherwise go unchecked.
Review binding changes before updating these values in `test-guidance.ps1`.

| Library | Layout checks | Exports |
| --- | --- | --- |
| Clarabel | 62 | 5 |
| SCS | 73 | 6 |

## Packaging

A local Release build packages the host runtime and checks the ZIP against its payload list.
To package both runtimes, build both native library pairs first, then pass both runtime identifiers to the mod build.
Use `DeployToMods=false` and `GenerateLaunchProfile=false` for a compile-only check.

Windows native, ABI, and numerical checks have passed locally.
Linux execution and GitHub artifact upload remain unverified until the workflow runs on GitHub.
