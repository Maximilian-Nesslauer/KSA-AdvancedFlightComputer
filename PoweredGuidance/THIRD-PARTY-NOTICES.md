# Third-party notices

navbox is MIT licensed (see [`LICENSE`](LICENSE)). It vendors and redistributes the
following third-party components, each under its own permissive licence. Their terms
require the copyright and permission notices to travel with any distribution; none of
them impose copyleft on this work, and none of them make this work Apache or BSD — a
permissive dependency does not relicense its dependent.

**Binary releases** carry the full licence texts in `licenses/` alongside the DLLs they
apply to. In the source tree they live at the paths given below.

## Clarabel — Apache-2.0

Interior-point conic solver, used by the G-FOLD powered-descent planner.

- Source: `gfold/clarabel/` (from [oxfordcontrol/Clarabel.cpp](https://github.com/oxfordcontrol/Clarabel.cpp)
  and its `Clarabel.rs` submodule)
- Licence: `gfold/clarabel/LICENSE.md`, shipped as `licenses/Clarabel-Apache-2.0.txt` (Apache License 2.0)
- Copyright (c) the Clarabel authors, Paul Goulart and Yuwen Chen

Clarabel in turn vendors:

- **AMD** — BSD 3-clause. Copyright (c) 1996-2015 Timothy A. Davis,
  Patrick R. Amestoy and Iain S. Duff. `gfold/clarabel/Clarabel.rs/...` via the `amd`
  crate; licence at `gfold/clarabel/Clarabel.rs/linsys/external/amd/LICENSE.txt` in the
  upstream tree.
- **QDLDL** — Apache-2.0. Copyright (c) the QDLDL authors.

## SCS — MIT

First-order conic solver, used by the 6-DOF successive-convexification guidance and
kept as a cross-check backend for G-FOLD.

- Source: `scvx/scs/` (from [cvxgrp/scs](https://github.com/cvxgrp/scs), 3.2.11)
- Licence: `scvx/scs/LICENSE.txt`, shipped as `licenses/SCS-MIT.txt` (MIT)
- Copyright (c) 2012 Brendan O'Donoghue

SCS vendors:

- **AMD** — BSD 3-clause, as above. `scvx/scs/linsys/external/amd/LICENSE.txt`
- **QDLDL** — Apache-2.0. `scvx/scs/linsys/external/qdldl/LICENSE`

## Not third-party

`scvx/native_src/blas_shim.c` is original work under this project's MIT licence, not a
vendored BLAS. It implements the six double-precision routines SCS's Anderson
acceleration needs, so that no external BLAS/LAPACK is linked.

## Previously

Releases up to and including **v0.3.1** linked [ECOS](https://github.com/embotech/ecos),
which is GPLv3, and were therefore distributed under GPLv3 as a whole. ECOS has since
been removed and replaced by Clarabel. Those older versions remain available under
GPLv3 — a licence already granted cannot be withdrawn — but they are the only versions
to which that applies. See [`CHANGELOG.md`](CHANGELOG.md).
