# Builds Clarabel's C library from third_party/clarabel/rust_wrapper.
# Cargo produces the C library directly, so this build does not need CMake.
# Cross builds need the selected Rust target and a linker for that platform.

[CmdletBinding()]
param(
    [string]$CargoExe = "cargo",
    [ValidateSet("win-x64", "linux-x64")]
    [string]$Rid = $(if ($env:OS -eq "Windows_NT") { "win-x64" } else { "linux-x64" }),
    [string]$OutDir
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$runtimes = @{
    "win-x64"   = @{ Triple = "x86_64-pc-windows-msvc";   Library = "clarabel_c.dll" }
    "linux-x64" = @{ Triple = "x86_64-unknown-linux-gnu"; Library = "libclarabel_c.so" }
}
$triple = $runtimes[$Rid].Triple
$library = $runtimes[$Rid].Library
if (-not $OutDir) { $OutDir = Join-Path $PSScriptRoot "native/$Rid" }

$crate = Join-Path $PSScriptRoot "../third_party/clarabel/rust_wrapper"
if (-not (Test-Path -LiteralPath (Join-Path $crate "Cargo.toml") -PathType Leaf)) {
    throw "clarabel rust_wrapper crate not found at $crate"
}

if (-not (Get-Command $CargoExe -ErrorAction SilentlyContinue)) {
    throw "cargo not found ('$CargoExe'). Install Rust from https://rustup.rs, or pass -CargoExe <path>."
}

Write-Host "building $library (release, $triple) with $CargoExe ..."
Push-Location $crate
try {
    & $CargoExe build --locked --release --target $triple
    if ($LASTEXITCODE -ne 0) { throw "cargo build failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
}

$built = Join-Path $crate "target/$triple/release/$library"
if (-not (Test-Path -LiteralPath $built -PathType Leaf)) {
    throw "cargo reported success but $built does not exist"
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item -LiteralPath $built -Destination (Join-Path $OutDir $library) -Force
Write-Host "wrote $(Join-Path $OutDir $library)"

# Keep the optional MSVC import library and debug symbols beside the DLL.
if ($Rid -eq "win-x64") {
    foreach ($extra in @("clarabel_c.dll.lib", "clarabel_c.pdb")) {
        $p = Join-Path $crate "target/$triple/release/$extra"
        if (Test-Path -LiteralPath $p -PathType Leaf) { Copy-Item -LiteralPath $p -Destination $OutDir -Force }
    }
}
