param([switch]$CacheOnly)
$ErrorActionPreference = 'Stop'
Set-Location $env:GITHUB_WORKSPACE
$log = Join-Path $env:GITHUB_WORKSPACE 'vcpkg-install.log'
New-Item -ItemType Directory -Force -Path "$env:GITHUB_WORKSPACE/vcpkg-bincache" | Out-Null
$arguments = @('install', '--triplet', $env:VCPKG_DEFAULT_TRIPLET,
    '--x-install-root', $env:VCPKG_INSTALLED_DIR, '--debug')
if ($CacheOnly) { $arguments += '--only-binarycaching' }
$timer = [Diagnostics.Stopwatch]::StartNew()
& "$env:VCPKG_ROOT/vcpkg.exe" @arguments 2>&1 | Tee-Object -FilePath $log
$code = $LASTEXITCODE
$timer.Stop()
$content = Get-Content $log -Raw
$restored = 0
foreach ($match in [regex]::Matches($content, 'Restored (\d+) package\(s\)')) { $restored += [int]$match.Groups[1].Value }
$built = [regex]::Matches($content, '(?m)^Building [^\r\n]+').Count
$seconds = [math]::Round($timer.Elapsed.TotalSeconds, 1)
$summary = "restored=$restored built=$built seconds=$seconds cache_only=$CacheOnly exit=$code"
Write-Host "[DEPENDENCY_CACHE] $summary"
@("restored_packages=$restored", "built_packages=$built", "install_seconds=$seconds") >> $env:GITHUB_OUTPUT
@('### Dependency cache', '', "- $summary") >> $env:GITHUB_STEP_SUMMARY
$summary | Set-Content "$env:GITHUB_WORKSPACE/ci-diagnostics/cache-result.txt"
if ($code -ne 0) {
    Get-Content $log -Tail 60 | ForEach-Object { Write-Host $_ }
    exit $code
}
if ($CacheOnly -and ($restored -eq 0 -or $built -ne 0)) {
    throw "Cache-only verification did not restore packages without building: $summary"
}
