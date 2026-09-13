# Third-party notices

AdvancedFlightComputer is MIT licensed, see [`LICENSE`](LICENSE) at the repository root.
The guidance feature vendors and redistributes the third-party components below, each under its own permissive licence.
Their terms require the copyright and permission notices to travel with any distribution.
None of them imposes copyleft on this work, and none of them makes this work Apache or BSD, because a permissive dependency does not relicense its dependent.

## Shipped files

A binary release carries the full licence texts in `licenses/` next to the DLLs they apply to.
This table says which text belongs to which shipped file.

| Shipped file | Component | Licence text in `licenses/` |
| --- | --- | --- |
| `clarabel_c.dll` | Clarabel, with the vendored AMD and QDLDL | `Clarabel-Apache-2.0.txt`, `AMD-BSD-3-Clause.txt`, `QDLDL-Apache-2.0.txt` |
| `scs.dll` | SCS, with the vendored AMD and QDLDL, plus the project's own BLAS shim | `SCS-MIT.txt`, `AMD-BSD-3-Clause.txt`, `QDLDL-Apache-2.0.txt` |
| `AdvancedFlightComputer.Guidance.Gfold.dll` | The G-FOLD planner, original work, calls Clarabel | `../LICENSE` |
| `AdvancedFlightComputer.Guidance.Scvx.dll` | The 6-DOF solver, original work, calls SCS | `../LICENSE` |
| `AdvancedFlightComputer.Guidance.Numerics.dll` | Original work | `../LICENSE` |
| `AdvancedFlightComputer.dll` | Original work | `../LICENSE` |

In the source tree the licence texts live at the paths given below.

## Clarabel, Apache-2.0

Interior-point conic solver, used by the G-FOLD powered-descent planner.

- Source: `third_party/clarabel/`, from [oxfordcontrol/Clarabel.cpp](https://github.com/oxfordcontrol/Clarabel.cpp) and its `Clarabel.rs` submodule.
- Licence: `third_party/clarabel/LICENSE.md`, shipped as `licenses/Clarabel-Apache-2.0.txt`.
- Copyright (c) the Clarabel authors, Paul Goulart and Yuwen Chen.

Clarabel in turn vendors:

- AMD, BSD 3-clause. Copyright (c) 1996-2015 Timothy A. Davis, Patrick R. Amestoy and Iain S. Duff. Used through the `amd` crate; the licence is at `third_party/clarabel/Clarabel.rs/linsys/external/amd/LICENSE.txt` in the upstream tree.
- QDLDL, Apache-2.0. Copyright (c) the QDLDL authors.

## SCS, MIT

First-order conic solver, used by the 6-DOF successive-convexification guidance.

- Source: `third_party/scs/`, from [cvxgrp/scs](https://github.com/cvxgrp/scs), 3.2.11.
- Licence: `third_party/scs/LICENSE.txt`, shipped as `licenses/SCS-MIT.txt`.
- Copyright (c) 2012 Brendan O'Donoghue.

SCS vendors:

- AMD, BSD 3-clause, as above, `third_party/scs/linsys/external/amd/LICENSE.txt`.
- QDLDL, Apache-2.0, `third_party/scs/linsys/external/qdldl/LICENSE`.

## Not third-party

`build/native_src/blas_shim.c` is original work under this project's MIT licence, not a vendored BLAS.
It implements the six double-precision routines SCS's Anderson acceleration needs, so that no external BLAS or LAPACK is linked.

## Previously

PoweredGuidance releases up to and including v0.3.1 linked [ECOS](https://github.com/embotech/ecos), which is GPLv3, and were therefore distributed under GPLv3 as a whole.
ECOS has since been removed and replaced by Clarabel.
Those older versions remain available under GPLv3, because a licence already granted cannot be withdrawn, but they are the only versions to which that applies.
See the [guidance changelog](docs/guidance/CHANGELOG.md).
