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

The check builds both native libraries and eight game-free managed projects, then checks native layouts and exports against the C# bindings in the Conic assembly.
It runs each Scvx check (`--fd`, `--aero`, `--impact`, `--sub-scs`, `--loop`, `--ascent`) and `Worker.Tests`, stopping on failure.
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

`.github/workflows/ci.yml` runs on pull requests, pushes to `main`, and manual dispatch.
Its Windows and Linux native jobs run the same checks and upload exactly two native libraries per runtime.
Missing, empty, or extra artifact files fail the job.
The native jobs use read-only repository access and need no game files or private tokens.

The `mod` job then builds and packages the whole mod in Release, with the native libraries from both runtimes.
It compiles against KSA reference assemblies, which keep the game's API and strip every method body.
They live in a private Backblaze B2 bucket, not in this repository.
`build/ci/fetch-ksa-refs.ps1` downloads the zip for `TestedGameVersion` in `Mod.cs`, using the `KSAREFS_ID` and `KSAREFS_KEY` secrets.
Pull requests from forks get no secrets, so the job is skipped for them.
The reference assemblies stay in the runner's temp folder and are never uploaded or cached.
The package check in `GuidancePayload.targets` keeps game assemblies out of the uploaded mod zip.

After installing a new KSA build, run `build/publish-ksa-refs.ps1` on a machine with the game.
It reads the installed `KSA.dll` version and does nothing if that zip is already in the bucket.
Otherwise it runs Refasmer on the game's managed DLLs, skipping the bundled .NET runtime and native libraries, and uploads the result.
It needs `B2_APPLICATION_KEY_ID`, `B2_APPLICATION_KEY` and `B2_BUCKET`, set to a key that can write to the bucket.
Until the zip for a new `TestedGameVersion` is uploaded, the `mod` job fails.

`build/hooks/pre-push` runs this check on every push, once enabled with `git config core.hooksPath build/hooks`.
It reads `TestedGameVersion` from each pushed commit and uploads the zip if it is missing and that build is installed.
Otherwise it warns, and without the B2 variables it does nothing.
It never blocks a push.

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


