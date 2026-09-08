# AFC guidance

Guidance is part of AFC under `AdvancedFlightComputer/Features/Guidance/`.
The current production bootstrap registers only an AFC Guidance menu with an unavailable message.
Control ownership must be integrated before guidance execution can be enabled.
The bootstrap does not call `GuidanceWindow`, install actuator patches, start solver workers, or write to vehicles.

| Path | Purpose |
| --- | --- |
| `AdvancedFlightComputer/Features/Guidance/Modes` | Guidance modes and vehicle state. |
| `AdvancedFlightComputer/Features/Guidance/Control` | Attitude, engine, and gimbal adapters. |
| `AdvancedFlightComputer/Features/Guidance/Adapters` | Game data conversion for the solvers. |
| `AdvancedFlightComputer/Features/Guidance/Upfg` | Ascent guidance. |
| `AdvancedFlightComputer/Features/Guidance/Ui` | Guidance panels and overlays, with no production draw callback yet. |
| `AdvancedFlightComputer/Features/Guidance/Numerics` | Game-independent numerical primitives and flight models. |
| `AdvancedFlightComputer/Features/Guidance/Gfold` | Game-independent G-FOLD solver. |
| `AdvancedFlightComputer/Features/Guidance/Scvx` | Game-independent six degree of freedom solver. |
| `AdvancedFlightComputer.Guidance.Tests` | Console checks and Python reference fixtures. |
| `third_party` | Vendored native solver sources and their licenses. |
| `build` | Native build scripts and the shared payload contract. |

The root solution builds the AFC mod, the separate `AdvancedFlightComputer.HarnessTests` mod, three numerical libraries, and two console check projects.
The three numerical projects have no game references.
The enclosing AFC project excludes their source trees from its compile items and uses explicit project references.

Build the solution in Debug with `dotnet build AdvancedFlightComputer.slnx -p:DeployToMods=false -p:GenerateLaunchProfile=false -p:KsaDir="<game install>" --disable-build-servers` for a managed compile check.
This command does not need native solver DLLs.
Run `build/build-clarabel.ps1` with Rust and `build/build-scs.ps1` with Zig to produce the native libraries in `build/native/<rid>/`.
Deployment and Release builds need both native libraries for their target runtime.
See [native-build.md](native-build.md) for the toolchains, runtime identifiers, ABI check, and package check.

The production payload has one `AdvancedFlightComputer` folder and one production `mod.toml`.
It includes the AFC assembly, the three numerical assemblies, both native solvers, XML patches, the root license, third-party notices, and the license texts in `licenses/`.
The same explicit `ModPayloadFile` list supplies deployment and Release packaging.
Game assemblies, loader assemblies, test assemblies, and native source files are not payload files.

The console checks can run without a game process.
For example, after a Debug build, run `dotnet AdvancedFlightComputer.Guidance.Tests/Scvx/bin/Debug/net10.0/AdvancedFlightComputer.Guidance.Tests.Scvx.dll --fd` from the repository root.
The `--fd`, `--aero`, and `--impact` checks need no native solver.
The `python_ref` folders contain the retained numerical references and fixtures.

The other documents in this folder retain the imported guidance design and release history.
Their historical standalone build and installation instructions are superseded by this page.
Authorship remains in the Git history and root `LICENSE`.
See the root `THIRD-PARTY-NOTICES.md` for the solver attribution.

Before a future combined flight test, disable an installed standalone PoweredGuidance mod through the normal mod controls.
The AFC build does not change the user's manifest.
