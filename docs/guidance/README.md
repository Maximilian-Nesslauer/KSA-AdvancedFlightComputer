# AFC guidance

Guidance is part of AFC under `AdvancedFlightComputer/Features/Guidance/`.
`GuidanceFeature` registers two patch blocks. The diagnostics block owns the AFC Guidance menu, and the driver block owns the per-vehicle step, the worker hooks and the panel.
Every guidance write to a craft goes through the ownership described in `../control-ownership.md`.
Single-craft ascent has been flown in the game from this tree. The other modes have not.

| Path | Purpose |
| --- | --- |
| `AdvancedFlightComputer/Features/Guidance/Modes` | Guidance modes and vehicle state. |
| `AdvancedFlightComputer/Features/Guidance/Control` | Attitude, engine, and gimbal adapters. |
| `AdvancedFlightComputer/Features/Guidance/Adapters` | Game data conversion for the solvers. |
| `AdvancedFlightComputer/Features/Guidance/Upfg` | Ascent guidance. |
| `AdvancedFlightComputer/Features/Guidance/Ui` | Guidance panel and overlays, drawn from the loader's after-GUI hook. |
| `AdvancedFlightComputer/Features/Guidance/Numerics` | Game-independent numerical primitives and flight models. |
| `AdvancedFlightComputer/Features/Guidance/Gfold` | Game-independent G-FOLD solver. |
| `AdvancedFlightComputer/Features/Guidance/Scvx` | Game-independent six degree of freedom solver. |
| `tests/AdvancedFlightComputer.Guidance.Tests` | Console checks and Python reference fixtures, no game needed. |
| `tests/AdvancedFlightComputer.HarnessTests` | The HeadlessHarness test mod, runs inside the real game. |
| `third_party` | Vendored native solver sources and their licenses. |
| `build` | Native build scripts and the shared payload contract. |

The root solution builds the AFC mod, the separate harness test mod, three numerical libraries, and two console check projects.
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
For example, after a Debug build, run `dotnet tests/AdvancedFlightComputer.Guidance.Tests/Scvx/bin/Debug/net10.0/AdvancedFlightComputer.Guidance.Tests.Scvx.dll --fd` from the repository root.
The `--fd`, `--aero`, and `--impact` checks need no native solver.
The `python_ref` folders contain the retained numerical references and fixtures.

## Documents in this folder

| Document | What it covers |
| --- | --- |
| [runtime.md](runtime.md) | How guidance hangs on AFC: entry, patch blocks, ownership, staging, layout, panel. |
| [native-build.md](native-build.md) | Building the two native solvers, the ABI check, and the package check. |
| [gfold.md](gfold.md) | The G-FOLD planner. |
| [scvx.md](scvx.md) | The 6-DOF successive-convexification solver, the algorithm as a specification. |
| [scvx-bridge.md](scvx-bridge.md) | From the 6-DOF solver to vehicle commands: frames, measurement, the MPC loop, actuators. |
| [scvx-bridge-devnotes.md](scvx-bridge-devnotes.md) | What had to be true besides the maths, learned by flying the 6-DOF path. |
| [CHANGELOG.md](CHANGELOG.md) | The release history of the guidance feature before it joined AFC. |

The solver documents keep the imported design and its notation.
Authorship remains in the Git history and root `LICENSE`.
See the root `THIRD-PARTY-NOTICES.md` for the solver attribution.

Before a combined flight test, remove an installed standalone PoweredGuidance mod folder, because the game loads the XML patches of every manifest entry whether it is enabled or not.
The AFC build does not change the user's manifest.
