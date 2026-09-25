<#
.SYNOPSIS
Publishes reference assemblies of the installed KSA build to a private Backblaze B2 bucket.

.DESCRIPTION
Reads the version of KSA.dll in the game folder and checks whether <Prefix><version>.zip is
already in the bucket. If it is not, every managed .dll in the game folder's root that belongs to
the game or its packages is run through Refasmer, which keeps the API surface and strips all
method bodies, and the result is zipped and uploaded. CI downloads that zip and builds the mod
with -p:KsaDir pointing at it.

The bundled .NET runtime (the runtimepack in KSA.deps.json) is skipped because CI's SDK provides
it, and native DLLs because they have no metadata. A managed DLL Refasmer cannot process is
reported and left out, unless a project in this repo references it through $(KsaDir), in which
case nothing is uploaded.

Credentials come from B2_APPLICATION_KEY_ID and B2_APPLICATION_KEY (the same variables the b2 CLI
reads). Use an application key restricted to the refs bucket.

.EXAMPLE
./build/publish-ksa-refs.ps1 -Bucket afc-ksa-refs

.EXAMPLE
./build/publish-ksa-refs.ps1 -NoUpload -OutputDir C:\temp\ksa-refs
Builds the zip without contacting B2.

.EXAMPLE
./build/publish-ksa-refs.ps1 -Version 2026.9.22.5482
Makes sure that build is in the bucket, as the pre-push hook in build/hooks does. If a different
build is installed, it can only check, and warns when the zip is missing.
#>
[CmdletBinding()]
param(
    [string]$GameDir = $(if ($env:KsaDir) { $env:KsaDir } else { "C:\Program Files\Kitten Space Agency" }),

    [string]$Bucket = $env:B2_BUCKET,

    [string]$Prefix = "ksa-refs/",

    # Upload even when the bucket already has this version. B2 keeps the old copy as a previous version.
    [switch]$Force,

    # Build the zip only, without contacting B2.
    [switch]$NoUpload,

    # Where to keep the zip. Without it the zip is deleted after uploading, or with -NoUpload
    # written to the current folder.
    [string]$OutputDir,

    # The build that has to be in the bucket, with or without a leading 'v'. Defaults to the
    # installed one; any other build can only be checked, because its DLLs are not on this machine.
    [string]$Version
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

# Windows PowerShell 5.1 may still default to TLS 1.0, which B2 refuses.
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

$repoRoot = Split-Path -Parent $PSScriptRoot

# --- Game version ------------------------------------------------------------------------------

$ksaDll = Join-Path $GameDir "KSA.dll"
$installed = $null
if (Test-Path -LiteralPath $ksaDll) {
    $versionInfo = (Get-Item -LiteralPath $ksaDll).VersionInfo
    $installed = $versionInfo.FileVersion
    if (-not $installed) {
        throw "KSA.dll in '$GameDir' has no file version."
    }
    Write-Host "Installed KSA: $installed ($($versionInfo.ProductVersion))"
}
elseif (-not $Version) {
    throw "KSA.dll was not found in '$GameDir'. Pass -GameDir or set the KsaDir environment variable."
}

$version = if ($Version) { $Version.TrimStart('v') } else { $installed }
$canBuild = $version -eq $installed
if ($NoUpload -and -not $canBuild) {
    throw "KSA $version is not installed in '$GameDir' (found '$installed'), so its reference assemblies cannot be built."
}
$objectName = "$Prefix$version.zip"

# --- B2 ----------------------------------------------------------------------------------------

function Invoke-B2 {
    param(
        [string]$Uri,
        [hashtable]$Headers,
        [object]$Body,
        [string]$ContentType = "application/json",
        [string]$Method = "Post"
    )

    if ($Body -is [hashtable]) {
        $Body = $Body | ConvertTo-Json -Compress
    }
    # PowerShell 7 rejects B2's authorization token as a header value unless validation is
    # skipped. Windows PowerShell 5.1 has no such check, and no such parameter.
    $webArgs = @{}
    if ($PSVersionTable.PSVersion.Major -ge 6) {
        $webArgs.SkipHeaderValidation = $true
    }
    try {
        if ($null -eq $Body) {
            return Invoke-RestMethod -Method $Method -Uri $Uri -Headers $Headers @webArgs
        }
        return Invoke-RestMethod -Method $Method -Uri $Uri -Headers $Headers -ContentType $ContentType -Body $Body @webArgs
    }
    catch {
        $detail = if ($_.ErrorDetails -and $_.ErrorDetails.Message) { $_.ErrorDetails.Message } else { $_.Exception.Message }
        throw "B2 request to $Uri failed: $detail"
    }
}

function Get-OptionalProperty {
    param([object]$Object, [string]$Name)

    if ($null -ne $Object -and $Object.PSObject.Properties[$Name]) {
        return $Object.$Name
    }
    return $null
}

$b2 = $null
if (-not $NoUpload) {
    if (-not $Bucket) {
        throw "No bucket given. Pass -Bucket or set B2_BUCKET."
    }
    if (-not $env:B2_APPLICATION_KEY_ID -or -not $env:B2_APPLICATION_KEY) {
        throw "Set B2_APPLICATION_KEY_ID and B2_APPLICATION_KEY to an application key for bucket '$Bucket'."
    }

    $basic = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("$($env:B2_APPLICATION_KEY_ID):$($env:B2_APPLICATION_KEY)"))
    $auth = Invoke-B2 -Method Get -Uri "https://api.backblazeb2.com/b2api/v3/b2_authorize_account" -Headers @{ Authorization = "Basic $basic" }
    $storage = $auth.apiInfo.storageApi
    $headers = @{ Authorization = $auth.authorizationToken }

    # A key restricted to a bucket names it in the authorize response, so a key without
    # listBuckets can still upload; otherwise the bucket is looked up. v3, which this calls, puts
    # bucketId and bucketName directly in apiInfo.storageApi. v2 put them in a top-level allowed,
    # and v4 has an allowed.buckets array of id and name, so those are read too.
    $bucketId = $null
    foreach ($allowed in @($storage, (Get-OptionalProperty $storage "allowed"), (Get-OptionalProperty $auth "allowed"))) {
        if ($null -eq $allowed) {
            continue
        }
        if ((Get-OptionalProperty $allowed "bucketName") -eq $Bucket) {
            $bucketId = $allowed.bucketId
        }
        foreach ($entry in @(Get-OptionalProperty $allowed "buckets")) {
            if ($null -ne $entry -and (Get-OptionalProperty $entry "name") -eq $Bucket) {
                $bucketId = $entry.id
            }
        }
    }
    if (-not $bucketId) {
        $found = Invoke-B2 -Uri "$($storage.apiUrl)/b2api/v3/b2_list_buckets" -Headers $headers -Body @{
            accountId = $auth.accountId
            bucketName = $Bucket
        }
        if (@($found.buckets).Count -eq 0) {
            throw "Bucket '$Bucket' was not found, or this key cannot see it."
        }
        $bucketId = @($found.buckets)[0].bucketId
    }

    $b2 = @{ ApiUrl = $storage.apiUrl; Headers = $headers; BucketId = $bucketId }

    $existing = Invoke-B2 -Uri "$($b2.ApiUrl)/b2api/v3/b2_list_file_names" -Headers $b2.Headers -Body @{
        bucketId = $b2.BucketId
        startFileName = $objectName
        prefix = $objectName
        maxFileCount = 1
    }
    $match = @($existing.files) | Where-Object { $_.fileName -eq $objectName }
    if ($match -and -not $Force) {
        Write-Host "b2://$Bucket/$objectName already exists, nothing to do. Pass -Force to upload it again."
        return
    }
    if (-not $canBuild) {
        $have = if ($installed) { "KSA $installed is installed" } else { "KSA is not installed in '$GameDir'" }
        if ($match) {
            Write-Warning "Cannot upload b2://$Bucket/$objectName again: $have."
        }
        else {
            Write-Warning "b2://$Bucket/$objectName is missing and $have, so it cannot be built here. CI's mod job fails until KSA $version is installed and this script is run."
        }
        return
    }
    if ($match) {
        Write-Host "b2://$Bucket/$objectName exists, uploading again because of -Force."
    }
    else {
        Write-Host "b2://$Bucket/$objectName is missing, building it."
    }
}

# --- Refasmer ----------------------------------------------------------------------------------

$refasmer = Get-Command refasmer -ErrorAction SilentlyContinue
if ($refasmer) {
    $refasmer = $refasmer.Source
}
else {
    $refasmer = Join-Path $HOME ".dotnet\tools\refasmer.exe"
    if (-not (Test-Path -LiteralPath $refasmer)) {
        throw "Refasmer was not found. Install it with: dotnet tool install -g JetBrains.Refasmer.CliTool"
    }
}

$workRoot = Join-Path ([IO.Path]::GetTempPath()) "ksa-refs-$version"
$stageDir = Join-Path $workRoot "stage"
$refsDir = Join-Path $workRoot "refs"
if (Test-Path -LiteralPath $workRoot) {
    Remove-Item -LiteralPath $workRoot -Recurse -Force
}
New-Item -ItemType Directory -Path $stageDir, $refsDir | Out-Null

try {
    # The game ships self-contained, so the folder also holds the whole .NET runtime. KSA.deps.json
    # lists those files under the runtimepack library; CI gets them from its own SDK instead.
    $depsPath = Join-Path $GameDir "KSA.deps.json"
    if (-not (Test-Path -LiteralPath $depsPath)) {
        throw "KSA.deps.json was not found in '$GameDir', so the .NET runtime files cannot be told apart from the game's."
    }
    $deps = Get-Content -LiteralPath $depsPath -Raw | ConvertFrom-Json
    $runtimePacks = @($deps.libraries.PSObject.Properties | Where-Object { $_.Value.type -eq "runtimepack" } | ForEach-Object { $_.Name })
    $runtimeFiles = @{}
    foreach ($target in $deps.targets.PSObject.Properties) {
        foreach ($library in $target.Value.PSObject.Properties | Where-Object { $runtimePacks -contains $_.Name }) {
            $runtime = Get-OptionalProperty $library.Value "runtime"
            if ($null -ne $runtime) {
                foreach ($file in $runtime.PSObject.Properties) {
                    $runtimeFiles[(Split-Path -Leaf $file.Name)] = $true
                }
            }
        }
    }
    if ($runtimeFiles.Count -eq 0) {
        throw "KSA.deps.json lists no runtime pack files. Its layout may have changed; check it before uploading the whole .NET runtime."
    }

    # Refasmer opens its inputs for writing, which Program Files does not allow, so it works on copies.
    $managed = @()
    $native = @()
    $runtimeSkipped = 0
    foreach ($dll in Get-ChildItem -LiteralPath $GameDir -Filter "*.dll" -File) {
        if ($runtimeFiles.ContainsKey($dll.Name)) {
            $runtimeSkipped++
            continue
        }
        try {
            [void][Reflection.AssemblyName]::GetAssemblyName($dll.FullName)
            Copy-Item -LiteralPath $dll.FullName -Destination $stageDir
            $managed += $dll.Name
        }
        catch [BadImageFormatException] {
            $native += $dll.Name
        }
    }
    Write-Host "Found $($managed.Count) managed game DLLs in $GameDir, skipping $runtimeSkipped .NET runtime and $($native.Count) native ones."

    # --omit-non-api-members=true drops private members that are not part of the API, which is all a
    # compile needs. File names rather than full paths keep the command line well under its limit.
    $refasmerArgs = @("--continue", "--omit-non-api-members=true", "--outputdir=$refsDir") + $managed
    # Refasmer reports each skipped file on stderr, which Windows PowerShell turns into a
    # terminating error under Stop; the exit code and the output folder are the real verdict.
    Push-Location $stageDir
    $ErrorActionPreference = "Continue"
    try {
        $refasmerLog = & $refasmer @refasmerArgs 2>&1 | ForEach-Object { "$_" }
    }
    finally {
        $ErrorActionPreference = "Stop"
        Pop-Location
    }
    if ($LASTEXITCODE -ne 0) {
        $refasmerLog | Write-Host
        throw "Refasmer exited with code $LASTEXITCODE."
    }

    $failed = @($managed | Where-Object { -not (Test-Path -LiteralPath (Join-Path $refsDir $_)) })

    # Anything a project here references through $(KsaDir) must have made it.
    $required = @(
        Get-ChildItem -LiteralPath $repoRoot -Filter "*.csproj" -File -Recurse |
            Where-Object { $_.FullName -notmatch '[\\/](bin|obj)[\\/]' } |
            ForEach-Object { [regex]::Matches((Get-Content -LiteralPath $_.FullName -Raw), '\$\(KsaDir\)[\\/]([^<\\/]+\.dll)') } |
            ForEach-Object { $_.Groups[1].Value } |
            Sort-Object -Unique
    )
    $missingRequired = @($required | Where-Object { -not (Test-Path -LiteralPath (Join-Path $refsDir $_)) })
    if ($missingRequired.Count -gt 0) {
        $refasmerLog | Where-Object { $_ -match ($missingRequired -join '|').Replace('.', '\.') } | Write-Host
        throw "Refasmer produced no reference assembly for $($missingRequired -join ', '), which the mod references. Nothing was uploaded."
    }
    if ($failed.Count -gt 0) {
        Write-Warning "Refasmer could not process $($failed.Count) managed DLL(s), left out of the zip: $($failed -join ', ')"
    }
    Write-Host "Made $($managed.Count - $failed.Count) reference assemblies, including all $($required.Count) the mod references."

    $manifest = @(
        "KSA reference assemblies, made by build/publish-ksa-refs.ps1. Method bodies are stripped; this is not a runnable game."
        "version = $version"
        "productVersion = $($versionInfo.ProductVersion)"
        "created = $([DateTime]::UtcNow.ToString('u'))"
        "refasmer = --omit-non-api-members=true"
        "skipped = $($failed -join ', ')"
    )
    Set-Content -LiteralPath (Join-Path $refsDir "manifest.txt") -Value $manifest -Encoding ASCII

    # --- Zip and upload ------------------------------------------------------------------------

    $zipDir = if ($OutputDir) { $OutputDir } elseif ($NoUpload) { (Get-Location).Path } else { $workRoot }
    New-Item -ItemType Directory -Path $zipDir -Force | Out-Null
    $zipPath = Join-Path $zipDir "$version.zip"
    if (Test-Path -LiteralPath $zipPath) {
        Remove-Item -LiteralPath $zipPath -Force
    }
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    [IO.Compression.ZipFile]::CreateFromDirectory($refsDir, $zipPath, [IO.Compression.CompressionLevel]::Optimal, $false)

    $zipSize = (Get-Item -LiteralPath $zipPath).Length
    $sha256 = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
    Write-Host ("Zipped {0:N1} MB, SHA-256 {1}" -f ($zipSize / 1MB), $sha256)

    if ($NoUpload) {
        Write-Host "Kept $zipPath (-NoUpload)."
        return
    }

    $upload = Invoke-B2 -Uri "$($b2.ApiUrl)/b2api/v3/b2_get_upload_url" -Headers $b2.Headers -Body @{ bucketId = $b2.BucketId }
    $sha1 = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA1).Hash.ToLowerInvariant()
    $result = Invoke-B2 -Uri $upload.uploadUrl -ContentType "application/zip" -Body ([IO.File]::ReadAllBytes($zipPath)) -Headers @{
        Authorization = $upload.authorizationToken
        "X-Bz-File-Name" = [Uri]::EscapeDataString($objectName).Replace("%2F", "/")
        "X-Bz-Content-Sha1" = $sha1
        "X-Bz-Info-ksa-version" = $version
        "X-Bz-Info-sha256" = $sha256
    }
    Write-Host "Uploaded b2://$Bucket/$($result.fileName) ($($result.fileId))."
    if ($OutputDir) {
        Write-Host "Kept $zipPath."
    }
}
finally {
    Remove-Item -LiteralPath $workRoot -Recurse -Force -ErrorAction SilentlyContinue
}
