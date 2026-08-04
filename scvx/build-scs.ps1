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
#  - USE_LAPACK left UNDEFINED (not "=0"): every LAPACK-gated block in the
#    source is `#ifdef USE_LAPACK`, which tests DEFINEDNESS, not the value —
#    so `-DUSE_LAPACK=0` still compiles the LAPACK path and the link fails on
#    an undefined dsyevr_. We only use the zero/positive-orthant/second-order
#    cones (no SDP), and the direct linear-system backend (AMD + qdldl, both
#    vendored under linsys/external, no BLAS/LAPACK) solves the KKT system.
#    LAPACK is needed only for SDP cone projection and Anderson acceleration;
#    aa.c has a complete no-LAPACK fallback (acceleration becomes a no-op), so
#    leaving the macro undefined costs nothing we use.
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
    "src/util.c", "src/aa.c",
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

Push-Location $scs
try {
    $zigArgs = @("cc", "-target", "x86_64-windows-gnu", "-shared") +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG") +
        $includes + $sources +
        @("-o", (Join-Path $out "scs.dll"))
    & $ZigExe @zigArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
}

Write-Host "Built: $(Join-Path $out 'scs.dll')" -ForegroundColor Green
