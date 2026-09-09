# Builds SCS from third_party/scs with Zig for Windows or Linux.
# Leave DLONG and SFLOAT undefined to keep 32-bit integers and 64-bit floats in the C ABI.
# Defining either macro as 0 still changes the types because SCS tests whether it is defined.
# CTRLC=0 leaves signal handling to the host process, and NDEBUG disables debug output.
# Anderson acceleration uses aa.c with the small BLAS shim in native_src/blas_shim.c.
# Compile only those two files with USE_LAPACK to avoid a full BLAS/LAPACK dependency.

[CmdletBinding()]
param(
    [string]$ZigExe = "zig",
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [ValidateSet("win-x64", "linux-x64")]
    [string]$Rid = $(if ($env:OS -eq "Windows_NT") { "win-x64" } else { "linux-x64" }),
    [string]$OutDir
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$scs = Join-Path $root "../third_party/scs"

$runtimes = @{
    "win-x64"   = @{ Target = "x86_64-windows-gnu"; Library = "scs.dll";   Pic = @() }
    "linux-x64" = @{ Target = "x86_64-linux-gnu";   Library = "libscs.so"; Pic = @("-fPIC") }
}
$target = $runtimes[$Rid].Target
$library = $runtimes[$Rid].Library
$pic = $runtimes[$Rid].Pic
$out = if ($OutDir) { $OutDir } else { Join-Path $root "native/$Rid" }

if (-not (Get-Command $ZigExe -ErrorAction SilentlyContinue)) {
    throw "zig not found ('$ZigExe'). Install Zig and put it on PATH, or pass -ZigExe <path>."
}
if (-not (Test-Path -LiteralPath $scs -PathType Container)) { throw "SCS sources not found at $scs" }
New-Item -ItemType Directory -Force -Path $out | Out-Null

$sources = @(
    "src/scs.c", "src/scs_version.c", "src/cones.c", "src/ctrlc.c",
    "src/exp_cone.c", "src/linalg.c", "src/normalize.c", "src/rw.c",
    "src/util.c",
    "linsys/scs_matrix.c", "linsys/csparse.c",
    "linsys/cpu/direct/private.c",
    "linsys/external/qdldl/qdldl.c"
) + (Get-ChildItem -LiteralPath (Join-Path $scs "linsys/external/amd") -Filter *.c |
        ForEach-Object { "linsys/external/amd/" + $_.Name })

$includes = @(
    "-Iinclude", "-Ilinsys",
    "-Ilinsys/cpu/direct",
    "-Ilinsys/external/amd", "-Ilinsys/external/qdldl"
)

$opt = if ($Configuration -eq "Release") { "-O2" } else { "-O0 -g" }

$shim = Join-Path $root "native_src/blas_shim.c"
if (-not (Test-Path -LiteralPath $shim -PathType Leaf)) { throw "BLAS shim not found at $shim" }
$aaObj = Join-Path $out "aa_lapack.o"
$shimObj = Join-Path $out "blas_shim.o"

Push-Location -LiteralPath $scs
try {
    $aaArgs = @("cc", "-target", $target, "-c") + $pic +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG", "-DUSE_LAPACK") +
        $includes + @("src/aa.c", "-o", $aaObj)
    & $ZigExe @aaArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed compiling aa.c with exit code $LASTEXITCODE" }

    # USE_LAPACK also exposes the BLAS types and names that the shim implements.
    $shimArgs = @("cc", "-target", $target, "-c") + $pic +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG", "-DUSE_LAPACK") +
        $includes + @($shim, "-o", $shimObj)
    & $ZigExe @shimArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed compiling blas_shim.c with exit code $LASTEXITCODE" }

    $zigArgs = @("cc", "-target", $target, "-shared") + $pic +
        $opt.Split(" ") +
        @("-DCTRLC=0", "-DNDEBUG") +
        $includes + $sources + @($shimObj, $aaObj) +
        @("-o", (Join-Path $out $library))
    & $ZigExe @zigArgs
    if ($LASTEXITCODE -ne 0) { throw "zig cc failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
    Remove-Item -LiteralPath $aaObj, $shimObj -ErrorAction SilentlyContinue
}

Write-Host "Built: $(Join-Path $out $library)" -ForegroundColor Green
