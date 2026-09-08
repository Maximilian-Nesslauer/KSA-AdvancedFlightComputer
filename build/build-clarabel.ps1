# Builds the Clarabel C library from the vendored sources (third_party/clarabel,
# oxfordcontrol/Clarabel.cpp + its Clarabel.rs submodule).
#
# Output: build/native/<rid>/clarabel_c.dll on Windows, libclarabel_c.so on Linux.
#
# WHY CARGO AND NOT CMAKE. Clarabel.cpp's README asks for Rust *and* CMake, but the
# CMake layer exists to build the optional C++/Eigen interface and the test binaries.
# The C ABI we bind to comes out of the rust_wrapper crate, whose Cargo.toml already
# declares `crate-type = ["cdylib", "staticlib"]` - so cargo alone emits the library
# and CMake is not in our path at all. That matters: it drops the toolchain
# requirement from "Rust + CMake + a C++ compiler" to "Rust".
#
# WHY NOT ZIG, unlike build-scs.ps1. That compiles C. This is a Rust crate, so it
# needs rustc/cargo, and no amount of zig substitutes. This is the one native
# dependency in the tree that cannot be built with the portable C compiler.
#
# Install Rust from https://rustup.rs. The build always names its target explicitly,
# so a runtime other than the host one needs that target installed with
# `rustup target add` and a linker for it. Building the Linux library on Linux is the
# short path, which is also what the release pipeline does.

[CmdletBinding()]
param(
    [string]$CargoExe = "cargo",
    [ValidateSet("win-x64", "linux-x64")]
    [string]$Rid = $(if ($env:OS -eq "Windows_NT") { "win-x64" } else { "linux-x64" }),
    [string]$OutDir
)

$ErrorActionPreference = "Stop"

$runtimes = @{
    "win-x64"   = @{ Triple = "x86_64-pc-windows-msvc";   Library = "clarabel_c.dll" }
    "linux-x64" = @{ Triple = "x86_64-unknown-linux-gnu"; Library = "libclarabel_c.so" }
}
$triple = $runtimes[$Rid].Triple
$library = $runtimes[$Rid].Library
if (-not $OutDir) { $OutDir = Join-Path $PSScriptRoot "native/$Rid" }

$crate = Join-Path $PSScriptRoot "../third_party/clarabel/rust_wrapper"
if (-not (Test-Path (Join-Path $crate "Cargo.toml"))) {
    throw "clarabel rust_wrapper crate not found at $crate"
}

if (-not (Get-Command $CargoExe -ErrorAction SilentlyContinue)) {
    throw "cargo not found ('$CargoExe'). Install Rust from https://rustup.rs, or pass -CargoExe <path>."
}

Write-Host "building $library (release, $triple) with $CargoExe ..."
Push-Location $crate
try {
    & $CargoExe build --release --target $triple
    if ($LASTEXITCODE -ne 0) { throw "cargo build failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
}

# cargo puts the artefacts under the crate's own target/ unless told otherwise, and
# under a per-target subfolder once --target is named.
$built = Join-Path $crate "target/$triple/release/$library"
if (-not (Test-Path $built)) {
    throw "cargo reported success but $built does not exist"
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item $built (Join-Path $OutDir $library) -Force
Write-Host "wrote $(Join-Path $OutDir $library)"

# The import library and symbols are not needed by the P/Invoke binding (it loads by
# name through NativeLibraries.cs), but copy them when present so the output folder
# matches what build-scs.ps1 leaves behind. Only the MSVC build produces them.
if ($Rid -eq "win-x64") {
    foreach ($extra in @("clarabel_c.dll.lib", "clarabel_c.pdb")) {
        $p = Join-Path $crate "target/$triple/release/$extra"
        if (Test-Path $p) { Copy-Item $p $OutDir -Force }
    }
}
