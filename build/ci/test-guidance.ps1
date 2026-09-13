#requires -Version 7.0

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$RepositoryRoot,

    [Parameter(Mandatory = $true)]
    [ValidateSet("win-x64", "linux-x64")]
    [string]$RuntimeIdentifier,

    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",

    [string]$ArtifactsRoot,
    [string]$CargoExe = "cargo",
    [string]$ZigExe = "zig",
    [switch]$SkipNativeBuild
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepositoryRoot = [IO.Path]::GetFullPath($RepositoryRoot)
if (-not $ArtifactsRoot) {
    $ArtifactsRoot = Join-Path ([IO.Path]::GetTempPath()) "afc-guidance-native"
}
$ArtifactsRoot = [IO.Path]::GetFullPath($ArtifactsRoot)

$hostRid = if ($IsWindows) { "win-x64" } else { "linux-x64" }
if ($RuntimeIdentifier -ne $hostRid) {
    throw "RuntimeIdentifier is $RuntimeIdentifier, but this host is $hostRid. Native ABI checks must run on the target platform."
}

$projects = [ordered]@{
    Numerics = "AdvancedFlightComputer/Features/Guidance/Numerics/AdvancedFlightComputer.Guidance.Numerics.csproj"
    Gfold = "AdvancedFlightComputer/Features/Guidance/Gfold/AdvancedFlightComputer.Guidance.Gfold.csproj"
    Scvx = "AdvancedFlightComputer/Features/Guidance/Scvx/AdvancedFlightComputer.Guidance.Scvx.csproj"
    GfoldConsole = "tests/AdvancedFlightComputer.Guidance.Tests/Gfold/AdvancedFlightComputer.Guidance.Tests.Gfold.csproj"
    ScvxConsole = "tests/AdvancedFlightComputer.Guidance.Tests/Scvx/AdvancedFlightComputer.Guidance.Tests.Scvx.csproj"
    WorkerTests = "tests/AdvancedFlightComputer.Guidance.Tests/Worker.Tests/Worker.Tests.csproj"
    NativeChecks = "build/NativeChecks/NativeChecks.csproj"
}
foreach ($relativePath in $projects.Values) {
    if (-not (Test-Path -LiteralPath (Join-Path $RepositoryRoot $relativePath) -PathType Leaf)) {
        throw "Required project not found at $relativePath."
    }
}

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [Parameter(Mandatory = $true)][string[]]$ArgumentList,
        [Parameter(Mandatory = $true)][string]$FailureMessage
    )

    & $FilePath @ArgumentList
    if ($LASTEXITCODE -ne 0) {
        throw "$FailureMessage Exit code $LASTEXITCODE."
    }
}

$nativeNames = if ($RuntimeIdentifier -eq "win-x64") {
    @("clarabel_c.dll", "scs.dll")
}
else {
    @("libclarabel_c.so", "libscs.so")
}
$nativeDir = Join-Path $RepositoryRoot "build/native/$RuntimeIdentifier"

if (-not $SkipNativeBuild) {
    $clarabelScript = Join-Path $RepositoryRoot "build/build-clarabel.ps1"
    $scsScript = Join-Path $RepositoryRoot "build/build-scs.ps1"
    & $clarabelScript -CargoExe $CargoExe -Rid $RuntimeIdentifier -OutDir $nativeDir
    & $scsScript -ZigExe $ZigExe -Configuration $Configuration -Rid $RuntimeIdentifier -OutDir $nativeDir
}

foreach ($fileName in $nativeNames) {
    $nativePath = Join-Path $nativeDir $fileName
    if (-not (Test-Path -LiteralPath $nativePath -PathType Leaf)) {
        throw "The native build did not produce $nativePath."
    }
    if ((Get-Item -LiteralPath $nativePath).Length -eq 0) {
        throw "The native build produced an empty file at $nativePath."
    }
}

# Restore both runtimes to match the lock files, then build for one.
# Passing --runtime to restore narrows that set and causes NU1004 in locked mode.
$restoreArguments = @(
    "-p:Configuration=$Configuration",
    "-p:RestoreLockedMode=true",
    "--disable-build-servers"
)
foreach ($project in $projects.GetEnumerator()) {
    $projectPath = Join-Path $RepositoryRoot $project.Value
    Write-Host "Restoring $($project.Key) against its lock file."
    Invoke-Checked -FilePath "dotnet" -ArgumentList (@("restore", $projectPath) + $restoreArguments) -FailureMessage "The $($project.Key) restore failed."
}

$commonBuildArguments = @(
    "--no-restore",
    "--configuration", $Configuration,
    "--runtime", $RuntimeIdentifier,
    "-p:DeployToMods=false",
    "-p:GenerateLaunchProfile=false",
    "--disable-build-servers"
)
foreach ($project in $projects.GetEnumerator()) {
    $projectPath = Join-Path $RepositoryRoot $project.Value
    Write-Host "Building $($project.Key) for $RuntimeIdentifier."
    Invoke-Checked -FilePath "dotnet" -ArgumentList (@("build", $projectPath) + $commonBuildArguments) -FailureMessage "The $($project.Key) build failed."
}

$targetFramework = "net10.0"
$gfoldAssembly = Join-Path $RepositoryRoot "AdvancedFlightComputer/Features/Guidance/Gfold/bin/$Configuration/$targetFramework/$RuntimeIdentifier/AdvancedFlightComputer.Guidance.Gfold.dll"
$scvxAssembly = Join-Path $RepositoryRoot "AdvancedFlightComputer/Features/Guidance/Scvx/bin/$Configuration/$targetFramework/$RuntimeIdentifier/AdvancedFlightComputer.Guidance.Scvx.dll"
$scvxConsoleDir = Join-Path $RepositoryRoot "tests/AdvancedFlightComputer.Guidance.Tests/Scvx/bin/$Configuration/$targetFramework/$RuntimeIdentifier"

Copy-Item -LiteralPath (Join-Path $nativeDir $nativeNames[0]) -Destination (Split-Path -Parent $gfoldAssembly) -Force
Copy-Item -LiteralPath (Join-Path $nativeDir $nativeNames[1]) -Destination (Split-Path -Parent $scvxAssembly) -Force
foreach ($fileName in $nativeNames) {
    Copy-Item -LiteralPath (Join-Path $nativeDir $fileName) -Destination $scvxConsoleDir -Force
}

$nativeCheckProject = Join-Path $RepositoryRoot $projects.NativeChecks
$nativeCheckBase = @(
    "run", "--no-build",
    "--project", $nativeCheckProject,
    "--configuration", $Configuration,
    "--runtime", $RuntimeIdentifier,
    "--"
)
# Fixed counts catch removed bindings that the layout and export checks would no longer see.
# Review binding changes before updating these values.
$expectedClarabelChecks = 62
$expectedClarabelExports = 5
$expectedScsChecks = 73
$expectedScsExports = 6
Invoke-Checked -FilePath "dotnet" -ArgumentList ($nativeCheckBase + @("clarabel", $gfoldAssembly, "--rid", $RuntimeIdentifier, "--zig", $ZigExe, "--expect-checks", $expectedClarabelChecks, "--expect-exports", $expectedClarabelExports)) -FailureMessage "The Clarabel ABI check failed."
Invoke-Checked -FilePath "dotnet" -ArgumentList ($nativeCheckBase + @("scs", $scvxAssembly, "--rid", $RuntimeIdentifier, "--zig", $ZigExe, "--expect-checks", $expectedScsChecks, "--expect-exports", $expectedScsExports)) -FailureMessage "The SCS ABI check failed."

$scvxProject = Join-Path $RepositoryRoot $projects.ScvxConsole
$scvxBase = @(
    "run", "--no-build",
    "--project", $scvxProject,
    "--configuration", $Configuration,
    "--runtime", $RuntimeIdentifier,
    "--"
)
foreach ($command in @("--fd", "--aero", "--impact", "--sub-scs", "--loop")) {
    Invoke-Checked -FilePath "dotnet" -ArgumentList ($scvxBase + $command) -FailureMessage "The Scvx $command check failed."
}

$workerProject = Join-Path $RepositoryRoot $projects.WorkerTests
$workerArguments = @(
    "run", "--no-build",
    "--project", $workerProject,
    "--configuration", $Configuration,
    "--runtime", $RuntimeIdentifier
)
Invoke-Checked -FilePath "dotnet" -ArgumentList $workerArguments -FailureMessage "The Worker.Tests check failed."

$artifactDir = Join-Path $ArtifactsRoot $RuntimeIdentifier
New-Item -ItemType Directory -Force -Path $artifactDir | Out-Null
foreach ($fileName in $nativeNames) {
    Copy-Item -LiteralPath (Join-Path $nativeDir $fileName) -Destination (Join-Path $artifactDir $fileName) -Force
}

$expected = [Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
foreach ($fileName in $nativeNames) { [void]$expected.Add($fileName) }
$actualFiles = @(Get-ChildItem -LiteralPath $artifactDir -File -Recurse)
$actual = [Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
foreach ($file in $actualFiles) {
    $relative = [IO.Path]::GetRelativePath($artifactDir, $file.FullName).Replace('\', '/')
    [void]$actual.Add($relative)
}
if (-not $actual.SetEquals($expected)) {
    $missing = @($expected | Where-Object { -not $actual.Contains($_) })
    $unexpected = @($actual | Where-Object { -not $expected.Contains($_) })
    throw "The artifact allowlist failed. Missing files are '$($missing -join ', ')'. Unexpected files are '$($unexpected -join ', ')'."
}
if ($actualFiles.Count -ne 2) {
    throw "The artifact directory contains $($actualFiles.Count) files, but it must contain exactly two native libraries."
}

Write-Host "Guidance checks passed for $RuntimeIdentifier. The artifact contains only these files."
Get-FileHash -LiteralPath $actualFiles.FullName -Algorithm SHA256
