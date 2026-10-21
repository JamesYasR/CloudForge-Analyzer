param(
    [Parameter(Mandatory = $true)][ValidateSet('Restore', 'Backup')][string]$Mode,
    [Parameter(Mandatory = $true)][string]$CacheKey,
    [switch]$Required
)
$ErrorActionPreference = 'Stop'
$tag = 'ci-dependency-cache'
$repo = $env:GITHUB_REPOSITORY
$root = $env:GITHUB_WORKSPACE
$cache = Join-Path $root 'vcpkg-bincache'
$work = Join-Path $env:RUNNER_TEMP 'dependency-release-cache'
New-Item -ItemType Directory -Force -Path $work, $cache | Out-Null
$name = "$CacheKey.tar"
$archive = Join-Path $work $name
$checksum = "$archive.sha256"

function Get-CacheRelease {
    $errorFile = Join-Path $work 'release-query-error.txt'
    $result = & gh release view $tag --repo $repo --json assets,url 2> $errorFile
    if ($LASTEXITCODE -eq 0) { return ($result | ConvertFrom-Json) }
    $message = Get-Content $errorFile -Raw
    if ($message -match 'release not found|HTTP 404|Not Found') { return $null }
    throw "Cannot query dependency backup release: $message"
}

$release = Get-CacheRelease
$present = $release -and ($release.assets.name -contains $name) -and ($release.assets.name -contains "$name.sha256")
if ($Mode -eq 'Restore') {
    if (-not $present) {
        "[RELEASE_CACHE] No backup for $CacheKey" | Tee-Object -FilePath "$root/ci-diagnostics/release-cache.txt"
        if ($Required) { throw 'Required dependency backup is missing' }
        return
    }
    foreach ($asset in @($name, "$name.sha256")) {
        & gh release download $tag --repo $repo --pattern $asset --dir $work --clobber
        if ($LASTEXITCODE -ne 0) { throw "Cannot download dependency backup: $asset" }
    }
    $expected = (Get-Content $checksum -Raw).Trim().Split(' ')[0]
    $actual = (Get-FileHash $archive -Algorithm SHA256).Hash
    if ($expected -notmatch '^[a-fA-F0-9]{64}$' -or $expected -ne $actual) {
        throw 'Dependency backup SHA256 mismatch'
    }
    & tar -xf $archive -C $cache
    if ($LASTEXITCODE -ne 0) { throw 'Cannot extract dependency backup' }
    $count = @(Get-ChildItem $cache -Recurse -File -Filter '*.zip').Count
    "[RELEASE_CACHE] Restored $count binary package archives; SHA256 verified; release=$($release.url)" |
        Tee-Object -FilePath "$root/ci-diagnostics/release-cache.txt"
    "source=release" >> $env:GITHUB_OUTPUT
    return
}

if ($present) {
    Write-Host "[RELEASE_CACHE] Durable backup already exists: $name"
    return
}
$packages = @(Get-ChildItem $cache -Recurse -File -Filter '*.zip')
if ($packages.Count -eq 0) { throw 'No binary packages to back up' }
if (@(Get-ChildItem $cache -Recurse -File | Where-Object { $_.Extension -ne '.zip' }).Count -ne 0) {
    throw 'Unexpected non-package file in the dependency cache'
}
& tar -cf $archive -C $cache .
if ($LASTEXITCODE -ne 0) { throw 'Cannot create dependency backup' }
if ((Get-Item $archive).Length -ge 2GB) { throw 'Dependency archive exceeds the GitHub Release asset limit; split it before uploading' }
$hash = (Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant()
"$hash  $name" | Set-Content $checksum -Encoding ascii
$manifest = Join-Path $work "$CacheKey.json"
[ordered]@{
    key = $CacheKey
    sha256 = $hash
    packageCount = $packages.Count
    sourceCommit = $env:GITHUB_SHA
    sourceRun = "https://github.com/$repo/actions/runs/$env:GITHUB_RUN_ID"
    toolchain = Get-Content "$root/ci-diagnostics/toolchain.json" -Raw | ConvertFrom-Json
} | ConvertTo-Json -Depth 8 | Set-Content $manifest -Encoding utf8
if (-not $release) {
    & gh release create $tag --repo $repo --target $env:GITHUB_SHA --prerelease --latest=false `
        --title 'CI dependency binary backups' `
        --notes 'Long-term vcpkg binary packages keyed by compatible toolchain and dependency inputs. SHA256 sidecars accompany each archive. These are CI dependencies, not application installers. Retain these assets for future builds.'
    if ($LASTEXITCODE -ne 0 -and -not (Get-CacheRelease)) { throw 'Cannot create dependency backup release' }
}
& gh release upload $tag $archive $checksum $manifest --repo $repo
if ($LASTEXITCODE -ne 0) {
    $release = Get-CacheRelease
    if (-not ($release -and ($release.assets.name -contains $name) -and ($release.assets.name -contains "$name.sha256"))) {
        throw 'Cannot upload dependency backup'
    }
}
Write-Host "[RELEASE_CACHE] Durable backup saved: $name; packages=$($packages.Count); SHA256=$hash"
