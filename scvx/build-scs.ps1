# Builds scs.dll from the vendored SCS sources (scvx/scs, cvxgrp/scs 3.2.11)
# using a portable Zig as the C compiler — no MSVC required. Mirrors
# gfold/build-ecos.ps1's approach and flag choices.
#
# Output: scvx/native/scs.dll  (x86_64, MinGW-style: all public symbols exported)
#
# Build choices:
#  - DLONG left UNDEFINED (not "=0"): same #ifdef-tests-definedness trap as
#    USE_LAPACK below — scs_types.h has `#ifdef DLONG`, so `-DDLONG=0` still
#    DEFINES it and switches scs_int to a 64-bit `long long`, silently breaking
#    every C# struct that assumes the 32-bit `int` DLONG=0 looks like it should
#    mean. (Found the hard way: struct offsets computed fine, data validated
#    fine, and scs_init still failed — because the native side was reading
#    Z/L/Bsize/Qsize/M/N and every SOC-dims array element at the wrong width.)
#    Leaving DLONG undefined gives scs_int = int32, matching the C# side.
#  - CTRLC=0: no console signal handler — this DLL ends up inside the game
#    process, which must own its own signal handling.
#  - USE_LAPACK is defined for src/aa.c ONLY, which is why that file is compiled
#    to its own object first and linked in separately below.
#
#    This USED to be undefined everywhere, with the note "aa.c has a complete
#    no-LAPACK fallback (acceleration becomes a no-op), so leaving the macro
#    undefined costs nothing we use." The fact was right and the conclusion was
#    wrong. Anderson acceleration is not an optional extra — SCS turns it on by
#    default (ACCELERATION_LOOKBACK = 10) and it is the mechanism that keeps the
#    ADMM iteration count down on ill-conditioned problems, which every SCvx
#    subproblem is. Measured in closed loop, scs_init is 2-6% of solve time and
#    the ADMM sweeps are the other 94-98%, so iteration count is the ONLY thing
#    worth attacking and we had disabled the tool for attacking it.
#
#    Scoping it to aa.c alone is what makes this cheap. `struct ACCEL_WORK` is
#    defined inside aa.c and everyone else sees an opaque AaWork*, so no other
#    translation unit's ABI depends on the macro. That keeps cones.c on its
#    non-LAPACK path (so the SDP-only dsyevr_/dgesvd_/dsyrk_ never enter the
#    link, and we use no SDP cones) and leaves linalg.c on its hand-written
#    loops. Only six routines are then needed — dnrm2_, daxpy_, dscal_, dgemv_,
#    dgemm_, dgesv_ — supplied by native_src/blas_shim.c rather than by linking
#    a multi-megabyte OpenBLAS into the game process. The sizes involved are
#    tiny (Anderson memory is 10, so a 10x10 LU and skinny dim-by-10 products),
#    nowhere near the KKT solve that dominates each iteration.
#  - NDEBUG: disables AMD's internal debug dumps, matching the ECOS build.

param(
    [string]$ZigExe = "zig",
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$scs = Join-Path $root "scs"
$out = Join-Path $root "native"

if (-not (Get-Command $ZigExe -ErrorAction SilentlyContinue)) {
    throw "zig not found ('$ZigExe'). Install Zig and put it on PATH, or pass -ZigExe <path>."
}
if (-not (Test-Path $scs)) { throw "SCS sources not found at $scs" }
New-Item -ItemType Directory -Force $out | Out-Null

$sources = @(
    "src/scs.c", "src/scs_version.c", "src/cones.c", "src/ctrlc.c",
    "src/exp_cone.c", "src/linalg.c", "src/normalize.c", "src/rw.c",
    "src/util.c",
    "linsys/scs_matrix.c", "linsys/csparse.c",
    "linsys/cpu/direct/private.c",
    "linsys/external/qdldl/qdldl.c"
) + (Get-ChildItem (Join-Path $scs "linsys/external/amd") -Filter *.c |
        ForEach-Object { "linsys/external/amd/" + $_.Name })

$includes = @(
    "-Iinclude", "-Ilinsys",
    "-Ilinsys/cpu/direct",
    "-Ilinsys/external/amd", "-Ilinsys/external/qdldl"
)

$opt = if ($Configuration -eq "Release") { "-O2" } else { "-O0 -g" }

$shim = Join-Path $root "native_src/blas_shim.c"
if (-not (Test-Path $shim)) { throw "BLAS shim not found at $shim" }
$aaObj = Join-Path $out "aa_lapack.o"
$shimObj = Join-Path $out "blas_shim.o"

Push-Location $scs
try {
    # aa.c alone gets -DUSE_LAPACK, so Anderson acceleration is compiled in
    # rather than stubbed out to a no-op. See the header comment.
    $aaArgs = @("cc", "-target", "x86_64-windows-gnu", "-c") +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG", "-DUSE_LAPACK") +
        $includes + @("src/aa.c", "-o", $aaObj)
    & $ZigExe @aaArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed compiling aa.c with exit code $LASTEXITCODE" }

    # The shim needs the macro too: scs_blas.h puts the blas_int typedef and the
    # BLAS() name-mangling macros behind #ifdef USE_LAPACK, so without it the
    # shim cannot even name the types it is implementing.
    $shimArgs = @("cc", "-target", "x86_64-windows-gnu", "-c") +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG", "-DUSE_LAPACK") +
        $includes + @($shim, "-o", $shimObj)
    & $ZigExe @shimArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed compiling blas_shim.c with exit code $LASTEXITCODE" }

    $zigArgs = @("cc", "-target", "x86_64-windows-gnu", "-shared") +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG") +
        $includes + $sources + @($shimObj, $aaObj) +
        @("-o", (Join-Path $out "scs.dll"))
    & $ZigExe @zigArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
    Remove-Item $aaObj, $shimObj -ErrorAction SilentlyContinue
}

Write-Host "Built: $(Join-Path $out 'scs.dll')" -ForegroundColor Green
