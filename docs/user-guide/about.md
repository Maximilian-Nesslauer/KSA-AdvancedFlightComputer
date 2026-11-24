# Compatibility and credits

## Mod compatibility

- [AutoStage](https://github.com/Maximilian-Nesslauer/KSA-AutoStage), [AutoRemoveFinishedBurns](https://github.com/Maximilian-Nesslauer/KSA-AutoRemoveFinishedBurns) and [PoweredGuidance](https://github.com/cairn5/PoweredGuidance) are built into AFC. Delete their standalone folders under `mods\`, because KittenExtensions Continued applies XML patches from disabled mods as well.
- The **AFC** menu sits in the top bar.
- Known conflicts: the standalone AutoStage and AutoRemoveFinishedBurns mods.

## Credits

- **Oliver Cairns ([cairn5](https://github.com/cairn5))** wrote [PoweredGuidance](https://github.com/cairn5/PoweredGuidance), which forms the powered guidance inside AFC: UPFG ascent, convex ascent, G-FOLD, successive-convexification 3-DOF entry and 6-DOF touchdown guidance, boostback, terminal hover, returnable stages, impact prediction and the guidance console pages. Its full history is part of this repository, and he is co-author of the project.
- **[PEGAS](https://github.com/Noiredd/PEGAS)** by Noiredd is the reference and foundation for the UPFG implementation; its MIT license is kept at [`third_party/pegas/LICENSE`](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/blob/main/third_party/pegas/LICENSE).
- **G-FOLD** follows Acikmese and Ploen, *Convex Programming Approach to Powered Descent Guidance for Mars Landing* (2007), and Acikmese, Carson and Blackmore, *Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem* (2013).
- **[Clarabel](https://github.com/oxfordcontrol/Clarabel.rs)** by Goulart and Chen and **[SCS](https://github.com/cvxgrp/scs)** by O'Donoghue are the conic solvers behind G-FOLD, convex ascent, entry guidance and touchdown guidance.

## License

MIT - see [`LICENSE`](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/blob/main/LICENSE). Vendored code under `third_party/` keeps its own upstream license.

PoweredGuidance releases up to and including **v0.3.1** linked [ECOS](https://github.com/embotech/ecos) (GPLv3) and were distributed under GPLv3 as a whole. Those releases remain under GPLv3, but ECOS was replaced by Clarabel (Apache-2.0) before the merge into AFC, so **nothing in the current tree is GPL**.

Attribution for the vendored solvers and their transitive dependencies is in [`THIRD-PARTY-NOTICES.md`](https://github.com/Maximilian-Nesslauer/KSA-AdvancedFlightComputer/blob/main/THIRD-PARTY-NOTICES.md). All of them are permissive and impose no copyleft on this work.

## Community

- KSA forum thread: https://forums.ahwoo.com/threads/advanced-flight-computer.783/

## Other mods

- [DeltaVMap](https://github.com/Maximilian-Nesslauer/KSA-DeltaVMap) - interactive delta-v subway map and transfer-window planner, auto-generated from the loaded system ([forum thread](https://forums.ahwoo.com/threads/deltavmap.978/))
- [MeasureTools](https://github.com/Maximilian-Nesslauer/KSA-MeasureTools) - click-to-measure ruler, protractor and surface measuring in the map view ([forum thread](https://forums.ahwoo.com/threads/measuretools.992/))
