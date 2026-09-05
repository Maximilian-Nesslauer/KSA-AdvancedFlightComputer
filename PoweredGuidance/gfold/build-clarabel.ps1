# Builds clarabel_c.dll from the vendored Clarabel sources (gfold/clarabel,
# oxfordcontrol/Clarabel.cpp + its Clarabel.rs submodule).
#
# Output: gfold/native/clarabel_c.dll  (x86_64)
#
# WHY CARGO AND NOT CMAKE. Clarabel.cpp's README asks for Rust *and* CMake, but the
# CMake layer exists to build the optional C++/Eigen interface and the test binaries.
# The C ABI we bind to comes out of the rust_wrapper crate, whose Cargo.toml already
# declares `crate-type = ["cdylib", "staticlib"]` — so cargo alone emits the DLL and
# CMake is not in our path at all. That matters: it drops the toolchain requirement
# from "Rust + CMake + a C++ compiler" to "Rust".
#
# WHY NOT ZIG, unlike build-scs.ps1 and build-ecos.ps1. Those compile C. This is a
# Rust crate, so it needs rustc/cargo, and no amount of zig substitutes. This is the
# one native dependency in the tree that cannot be built with the portable C compiler.
#
# Install Rust from https://rustup.rs (the default stable x86_64-pc-windows-msvc
# toolchain is fine; so is the -gnu one).

[CmdletBinding()]
param(
    [string]$CargoExe = "cargo",
    [string]$OutDir = (Join-Path $PSScriptRoot "native")
)

$ErrorActionPreference = "Stop"

$crate = Join-Path $PSScriptRoot "clarabel/rust_wrapper"
if (-not (Test-Path (Join-Path $crate "Cargo.toml"))) {
    throw "clarabel rust_wrapper crate not found at $crate"
}

if (-not (Get-Command $CargoExe -ErrorAction SilentlyContinue)) {
    throw "cargo not found ('$CargoExe'). Install Rust from https://rustup.rs, or pass -CargoExe <path>."
}

Write-Host "building clarabel_c (release) with $CargoExe ..."
Push-Location $crate
try {
    & $CargoExe build --release
    if ($LASTEXITCODE -ne 0) { throw "cargo build failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
}

# cargo puts the artefacts under the crate's own target/ unless told otherwise.
$built = Join-Path $crate "target/release/clarabel_c.dll"
if (-not (Test-Path $built)) {
    throw "cargo reported success but $built does not exist"
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item $built (Join-Path $OutDir "clarabel_c.dll") -Force
Write-Host "wrote $(Join-Path $OutDir 'clarabel_c.dll')"

# The import library and symbols are not needed by the P/Invoke binding (it loads by
# name through NativeLibraries.cs), but copy them when present so the output folder
# matches what build-scs.ps1 leaves behind.
foreach ($extra in @("clarabel_c.dll.lib", "clarabel_c.pdb")) {
    $p = Join-Path $crate "target/release/$extra"
    if (Test-Path $p) { Copy-Item $p $OutDir -Force }
}
