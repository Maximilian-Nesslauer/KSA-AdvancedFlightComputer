#requires -Version 7.0

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$InstallRoot,

    [ValidateSet("win-x64", "linux-x64")]
    [string]$RuntimeIdentifier = $(if ($IsWindows) { "win-x64" } else { "linux-x64" }),

    [string]$RustVersion = "1.90.0",
    [string]$ZigVersion = "0.14.1"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if ($RustVersion -ne "1.90.0") {
    throw "RustVersion must be 1.90.0 because this script contains the official hash for that release."
}
if ($ZigVersion -ne "0.14.1") {
    throw "ZigVersion must be 0.14.1 because this script contains the official hashes for that release."
}

$hostRid = if ($IsWindows) { "win-x64" } else { "linux-x64" }
if ($RuntimeIdentifier -ne $hostRid) {
    throw "RuntimeIdentifier is $RuntimeIdentifier, but this host is $hostRid. Install tools on the runner that will execute the native checks."
}

$platforms = @{
    "win-x64" = @{
        RustTarget = "x86_64-pc-windows-msvc"
        RustupFile = "rustup-init.exe"
        RustupSha256 = "88d8258dcf6ae4f7a80c7d1088e1f36fa7025a1cfd1343731b4ee6f385121fc0"
        ZigArchive = "zig-x86_64-windows-0.14.1.zip"
        ZigSha256 = "554f5378228923ffd558eac35e21af020c73789d87afeabf4bfd16f2e6feed2c"
        ZigExecutable = "zig.exe"
    }
    "linux-x64" = @{
        RustTarget = "x86_64-unknown-linux-gnu"
        RustupFile = "rustup-init"
        RustupSha256 = "20a06e644b0d9bd2fbdbfd52d42540bdde820ea7df86e92e533c073da0cdd43c"
        ZigArchive = "zig-x86_64-linux-0.14.1.tar.xz"
        ZigSha256 = "24aeeec8af16c381934a6cd7d95c807a8cb2cf7df9fa40d359aa884195c4716c"
        ZigExecutable = "zig"
    }
}

$rustupVersion = "1.28.2"
$rustManifestSha256 = "489c19f20d331765ab2835661eb546de90f6446a107a8db83045e7371e45cae2"
$platform = $platforms[$RuntimeIdentifier]
$InstallRoot = [IO.Path]::GetFullPath($InstallRoot)
$downloads = Join-Path $InstallRoot "downloads"
$cargoHome = Join-Path $InstallRoot "cargo"
$rustupHome = Join-Path $InstallRoot "rustup"
$zigParent = Join-Path $InstallRoot "zig"
$zigFolderName = [IO.Path]::GetFileNameWithoutExtension($platform.ZigArchive)
if ($platform.ZigArchive.EndsWith(".tar.xz", [StringComparison]::Ordinal)) {
    $zigFolderName = [IO.Path]::GetFileNameWithoutExtension($zigFolderName)
}
$zigRoot = Join-Path $zigParent $zigFolderName
$zigExe = Join-Path $zigRoot $platform.ZigExecutable
$rustupInstaller = Join-Path $downloads $platform.RustupFile
$rustManifest = Join-Path $downloads "channel-rust-$RustVersion.toml"
$zigArchive = Join-Path $downloads $platform.ZigArchive
$rustDistRoot = Join-Path $InstallRoot "rust-dist"
$rustDist = Join-Path $rustDistRoot "dist"

New-Item -ItemType Directory -Force -Path $downloads, $cargoHome, $rustupHome, $zigParent, $rustDist | Out-Null

function Get-VerifiedDownload {
    param(
        [Parameter(Mandatory = $true)][uri]$Uri,
        [Parameter(Mandatory = $true)][string]$Destination,
        [Parameter(Mandatory = $true)][string]$Sha256
    )

    if (-not (Test-Path -LiteralPath $Destination)) {
        $partial = "$Destination.download"
        Remove-Item -LiteralPath $partial -Force -ErrorAction SilentlyContinue
        Invoke-WebRequest -UseBasicParsing -Uri $Uri -OutFile $partial
        Move-Item -LiteralPath $partial -Destination $Destination
    }

    $actual = (Get-FileHash -LiteralPath $Destination -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $Sha256) {
        throw "The SHA-256 hash for $Destination is $actual, but the pinned official hash is $Sha256."
    }
}

$rustupUrl = "https://static.rust-lang.org/rustup/archive/$rustupVersion/$($platform.RustTarget)/$($platform.RustupFile)"
$rustManifestUrl = "https://static.rust-lang.org/dist/channel-rust-$RustVersion.toml"
$zigUrl = "https://ziglang.org/download/$ZigVersion/$($platform.ZigArchive)"

Get-VerifiedDownload -Uri $rustupUrl -Destination $rustupInstaller -Sha256 $platform.RustupSha256
Get-VerifiedDownload -Uri $rustManifestUrl -Destination $rustManifest -Sha256 $rustManifestSha256
Get-VerifiedDownload -Uri $zigUrl -Destination $zigArchive -Sha256 $platform.ZigSha256

$localManifest = Join-Path $rustDist "channel-rust-$RustVersion.toml"
Copy-Item -LiteralPath $rustManifest -Destination $localManifest -Force
"$rustManifestSha256  channel-rust-$RustVersion.toml" | Set-Content -LiteralPath "$localManifest.sha256" -Encoding ascii
$rustDistServer = ([uri]$rustDistRoot).AbsoluteUri.TrimEnd('/')

$manifestLines = @(Get-Content -LiteralPath $rustManifest)
$rustComponents = [ordered]@{}
foreach ($component in @("cargo", "rust-std", "rustc")) {
    $sectionName = "[pkg.$component.target.$($platform.RustTarget)]"
    $sectionStart = [Array]::IndexOf($manifestLines, $sectionName)
    if ($sectionStart -lt 0) {
        throw "The verified Rust manifest does not contain $sectionName."
    }
    $sectionEnd = $manifestLines.Count
    for ($index = $sectionStart + 1; $index -lt $manifestLines.Count; $index++) {
        if ($manifestLines[$index].StartsWith("[", [StringComparison]::Ordinal)) {
            $sectionEnd = $index
            break
        }
    }
    $section = $manifestLines[($sectionStart + 1)..($sectionEnd - 1)]
    $urlMatch = [regex]::Match(($section -join "`n"), '(?m)^xz_url = "([^"]+)"$')
    $hashMatch = [regex]::Match(($section -join "`n"), '(?m)^xz_hash = "([0-9a-f]{64})"$')
    if (-not $urlMatch.Success -or -not $hashMatch.Success) {
        throw "The verified Rust manifest does not give one xz URL and hash for $component on $($platform.RustTarget)."
    }
    $componentUrl = [uri]$urlMatch.Groups[1].Value
    if (-not $componentUrl.AbsoluteUri.StartsWith("https://static.rust-lang.org/dist/", [StringComparison]::Ordinal)) {
        throw "The verified Rust manifest gives an unexpected URL for $component, $componentUrl."
    }
    $relativePath = $componentUrl.AbsolutePath.TrimStart('/').Replace('/', [IO.Path]::DirectorySeparatorChar)
    $componentPath = Join-Path $rustDistRoot $relativePath
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $componentPath) | Out-Null
    Get-VerifiedDownload -Uri $componentUrl -Destination $componentPath -Sha256 $hashMatch.Groups[1].Value
    $rustComponents[$component] = [ordered]@{ Url = $componentUrl.AbsoluteUri; Sha256 = $hashMatch.Groups[1].Value }
}

if (-not $IsWindows) {
    & chmod +x $rustupInstaller
    if ($LASTEXITCODE -ne 0) { throw "chmod failed for $rustupInstaller with exit code $LASTEXITCODE." }
}

$env:RUSTUP_HOME = $rustupHome
$env:CARGO_HOME = $cargoHome
$env:RUSTUP_DIST_SERVER = $rustDistServer
$env:RUSTUP_UPDATE_ROOT = "https://static.rust-lang.org/rustup"

$rustupExeName = if ($IsWindows) { "rustup.exe" } else { "rustup" }
$rustupExe = Join-Path $cargoHome "bin/$rustupExeName"
if (-not (Test-Path -LiteralPath $rustupExe)) {
    & $rustupInstaller -y --no-modify-path --profile minimal --default-toolchain none
    if ($LASTEXITCODE -ne 0) { throw "rustup-init failed with exit code $LASTEXITCODE." }
}

$toolchain = "$RustVersion-$($platform.RustTarget)"
& $rustupExe toolchain install $toolchain --profile minimal --no-self-update
if ($LASTEXITCODE -ne 0) { throw "rustup failed to install $toolchain with exit code $LASTEXITCODE." }
& $rustupExe default $toolchain
if ($LASTEXITCODE -ne 0) { throw "rustup failed to select $toolchain with exit code $LASTEXITCODE." }
& $rustupExe set auto-self-update disable
if ($LASTEXITCODE -ne 0) { throw "rustup failed to disable automatic updates with exit code $LASTEXITCODE." }

$cargoExeName = if ($IsWindows) { "cargo.exe" } else { "cargo" }
$rustcExeName = if ($IsWindows) { "rustc.exe" } else { "rustc" }
$cargoExe = Join-Path $cargoHome "bin/$cargoExeName"
$rustcExe = Join-Path $cargoHome "bin/$rustcExeName"
$rustcOutput = & $rustcExe --version
if ($LASTEXITCODE -ne 0) { throw "rustc failed to report its version with exit code $LASTEXITCODE." }
$rustcVersion = "$rustcOutput".Trim()
if (-not $rustcVersion.StartsWith("rustc $RustVersion ", [StringComparison]::Ordinal)) {
    throw "The installed Rust compiler is '$rustcVersion', but rustc $RustVersion is required."
}
$cargoOutput = & $cargoExe --version
if ($LASTEXITCODE -ne 0) { throw "cargo failed to report its version with exit code $LASTEXITCODE." }
$cargoVersion = "$cargoOutput".Trim()
if (-not $cargoVersion.StartsWith("cargo $RustVersion ", [StringComparison]::Ordinal)) {
    throw "The installed Cargo version is '$cargoVersion', but cargo $RustVersion is required."
}

if (-not (Test-Path -LiteralPath $zigExe)) {
    if ($IsWindows) {
        Expand-Archive -LiteralPath $zigArchive -DestinationPath $zigParent
    }
    else {
        & tar -xJf $zigArchive -C $zigParent
        if ($LASTEXITCODE -ne 0) { throw "tar failed to extract Zig with exit code $LASTEXITCODE." }
    }
}

if (-not (Test-Path -LiteralPath $zigExe)) {
    throw "The Zig archive did not contain $zigExe."
}
if (-not $IsWindows) {
    & chmod +x $zigExe
    if ($LASTEXITCODE -ne 0) { throw "chmod failed for $zigExe with exit code $LASTEXITCODE." }
}
$zigOutput = & $zigExe version
if ($LASTEXITCODE -ne 0) { throw "zig failed to report its version with exit code $LASTEXITCODE." }
$installedZigVersion = "$zigOutput".Trim()
if ($installedZigVersion -cne $ZigVersion) {
    throw "The installed Zig version is '$installedZigVersion', but Zig $ZigVersion is required."
}

if ($env:GITHUB_ENV) {
    "RUSTUP_HOME=$rustupHome" | Add-Content -LiteralPath $env:GITHUB_ENV -Encoding utf8
    "CARGO_HOME=$cargoHome" | Add-Content -LiteralPath $env:GITHUB_ENV -Encoding utf8
    "RUSTUP_TOOLCHAIN=$toolchain" | Add-Content -LiteralPath $env:GITHUB_ENV -Encoding utf8
}
if ($env:GITHUB_PATH) {
    (Join-Path $cargoHome "bin") | Add-Content -LiteralPath $env:GITHUB_PATH -Encoding utf8
    $zigRoot | Add-Content -LiteralPath $env:GITHUB_PATH -Encoding utf8
}

$provenance = [ordered]@{
    RuntimeIdentifier = $RuntimeIdentifier
    RustVersion = $rustcVersion
    CargoVersion = $cargoVersion
    RustToolchain = $toolchain
    RustupVersion = $rustupVersion
    RustupUrl = $rustupUrl
    RustupSha256 = $platform.RustupSha256
    RustManifestUrl = $rustManifestUrl
    RustManifestSha256 = $rustManifestSha256
    RustDistServer = $rustDistServer
    RustComponents = $rustComponents
    ZigVersion = $installedZigVersion
    ZigUrl = $zigUrl
    ZigSha256 = $platform.ZigSha256
}
$provenance | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $InstallRoot "toolchain-provenance.json") -Encoding ascii

[pscustomobject]@{
    CargoExe = $cargoExe
    RustcExe = $rustcExe
    RustupExe = $rustupExe
    ZigExe = $zigExe
    Toolchain = $toolchain
    ProvenanceFile = Join-Path $InstallRoot "toolchain-provenance.json"
}
