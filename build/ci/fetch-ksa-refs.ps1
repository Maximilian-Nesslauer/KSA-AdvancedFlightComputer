<#
.SYNOPSIS
Downloads the KSA reference assemblies for the tested game build from the private B2 bucket.

.DESCRIPTION
build/publish-ksa-refs.ps1 uploads <Prefix><version>.zip from a machine with the game installed.
This fetches the zip for TestedGameVersion in AdvancedFlightComputer/Mod.cs, checks it against the
SHA-256 recorded at upload, and unpacks it into -Destination, which the mod build then gets as
-p:KsaDir. Keep -Destination outside anything uploaded as an artifact or cached.

Credentials come from B2_APPLICATION_KEY_ID and B2_APPLICATION_KEY. CI uses a read-only key.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Destination,

    [string]$RepositoryRoot,

    [string]$Bucket = $env:B2_BUCKET,

    [string]$Prefix = "ksa-refs/",

    # Defaults to TestedGameVersion in Mod.cs, without its leading 'v'.
    [string]$Version
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

if (-not $RepositoryRoot) {
    # Windows PowerShell leaves $PSScriptRoot empty in parameter defaults, so this is set here.
    $RepositoryRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
}
if (-not $Bucket) {
    throw "No bucket given. Pass -Bucket or set B2_BUCKET."
}
if (-not $env:B2_APPLICATION_KEY_ID -or -not $env:B2_APPLICATION_KEY) {
    throw "B2_APPLICATION_KEY_ID and B2_APPLICATION_KEY are not set. Fork pull requests get no secrets, so they cannot build the mod."
}

if (-not $Version) {
    $modCs = Join-Path $RepositoryRoot "AdvancedFlightComputer/Mod.cs"
    $match = [regex]::Match((Get-Content -LiteralPath $modCs -Raw), 'TestedGameVersion\s*=\s*"v?([^"]+)"')
    if (-not $match.Success) {
        throw "Could not read TestedGameVersion from $modCs."
    }
    $Version = $match.Groups[1].Value
}
$objectName = "$Prefix$Version.zip"

# PowerShell 7 rejects B2's authorization token as a header value unless validation is skipped.
# Windows PowerShell 5.1 has no such check, and no such parameter.
$webArgs = @{}
if ($PSVersionTable.PSVersion.Major -ge 6) {
    $webArgs.SkipHeaderValidation = $true
}

# GitHub masks the secrets, but not the credentials derived from them. Errors can quote headers,
# and the log of a public repository is public.
function Hide-FromLog([string]$Value) {
    if ($env:GITHUB_ACTIONS -eq "true" -and $Value) {
        Write-Host "::add-mask::$Value"
    }
}

$basic = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("$($env:B2_APPLICATION_KEY_ID):$($env:B2_APPLICATION_KEY)"))
Hide-FromLog $basic
try {
    $auth = Invoke-RestMethod -Uri "https://api.backblazeb2.com/b2api/v3/b2_authorize_account" -Headers @{ Authorization = "Basic $basic" } @webArgs
}
catch {
    throw "B2 rejected the application key: $($_.Exception.Message)"
}
Hide-FromLog $auth.authorizationToken

$zipPath = Join-Path ([IO.Path]::GetTempPath()) "ksa-refs-$Version.zip"
$encodedName = [Uri]::EscapeDataString($objectName).Replace("%2F", "/")
$uri = "$($auth.apiInfo.storageApi.downloadUrl)/file/$Bucket/$encodedName"
try {
    $response = Invoke-WebRequest -Uri $uri -Headers @{ Authorization = $auth.authorizationToken } -OutFile $zipPath -PassThru -UseBasicParsing @webArgs
}
catch {
    $status = $null
    if ($_.Exception.PSObject.Properties["Response"] -and $_.Exception.Response) {
        $status = [int]$_.Exception.Response.StatusCode
    }
    if ($status -eq 404) {
        throw "b2://$Bucket/$objectName does not exist. Install KSA $Version and run build/publish-ksa-refs.ps1 to upload it."
    }
    throw "Downloading b2://$Bucket/$objectName failed: $($_.Exception.Message)"
}

try {
    # publish-ksa-refs.ps1 records the zip's SHA-256 as file info, which B2 returns as a header.
    $expected = $null
    if ($response.Headers.ContainsKey("x-bz-info-sha256")) {
        $expected = @($response.Headers["x-bz-info-sha256"])[0]
    }
    $actual = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if (-not $expected) {
        Write-Warning "b2://$Bucket/$objectName has no recorded SHA-256, so it could not be checked."
    }
    elseif ($expected -ne $actual) {
        throw "b2://$Bucket/$objectName has SHA-256 $actual, but $expected was recorded when it was uploaded."
    }

    if (Test-Path -LiteralPath $Destination) {
        Remove-Item -LiteralPath $Destination -Recurse -Force
    }
    New-Item -ItemType Directory -Path $Destination | Out-Null
    Expand-Archive -LiteralPath $zipPath -DestinationPath $Destination

    if (-not (Test-Path -LiteralPath (Join-Path $Destination "KSA.dll"))) {
        throw "b2://$Bucket/$objectName has no KSA.dll."
    }
    $count = @(Get-ChildItem -LiteralPath $Destination -Filter "*.dll" -File).Count
    Write-Host "Fetched KSA $Version reference assemblies ($count DLLs, SHA-256 $actual)."
}
finally {
    Remove-Item -LiteralPath $zipPath -Force -ErrorAction SilentlyContinue
}
