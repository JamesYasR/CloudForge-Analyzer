param([Parameter(Mandatory = $true)][string]$VcpkgRoot)
$ErrorActionPreference = 'Stop'

# Only replace retired MSYS2 build-tool archives. The vcpkg baseline and
# application dependencies (Qt/PCL/VTK) remain pinned to their existing versions.
$path = Join-Path $VcpkgRoot 'scripts/cmake/vcpkg_acquire_msys.cmake'
$content = Get-Content $path -Raw
$packages = Get-Content (Join-Path $PSScriptRoot 'msys-packages.json') -Raw | ConvertFrom-Json
foreach ($package in $packages) {
    $oldUrl = "https://mirror.msys2.org/msys/x86_64/$($package.old)"
    $newUrl = "https://mirror.msys2.org/msys/x86_64/$($package.new)"
    if (-not $content.Contains($oldUrl) -or -not $content.Contains($package.old_sha512)) {
        throw "Pinned vcpkg MSYS2 definition changed: $($package.old)"
    }
    $content = $content.Replace($oldUrl, $newUrl).Replace($package.old_sha512, $package.sha512)
    Write-Host "MSYS2: $($package.old) -> $($package.new)"
}
Set-Content $path -Value $content -NoNewline -Encoding utf8
