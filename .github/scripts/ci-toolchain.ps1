$ErrorActionPreference = 'Stop'
$diagnostics = Join-Path $env:GITHUB_WORKSPACE 'ci-diagnostics'
New-Item -ItemType Directory -Force -Path $diagnostics | Out-Null

# These tools participate in vcpkg's ABI. Use the baseline's downloaded tools,
# rather than newer versions that vary across hosted runner image revisions.
Remove-Item Env:VCPKG_FORCE_SYSTEM_BINARIES -ErrorAction SilentlyContinue
$vswhere = "${env:ProgramFiles(x86)}/Microsoft Visual Studio/Installer/vswhere.exe"
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'Visual Studio C++ tools were not found' }
$compiler = Join-Path $vs 'VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe'
$sdk = "${env:ProgramFiles(x86)}/Windows Kits/10/Lib/10.0.26100.0"
if (-not (Test-Path $compiler)) { throw "Pinned MSVC compiler missing: $compiler" }
if (-not (Test-Path $sdk)) { throw "Pinned Windows SDK missing: $sdk" }
"VCPKG_VISUAL_STUDIO_PATH=$vs" >> $env:GITHUB_ENV

$identity = [ordered]@{
    msvc = '14.44.35207'
    compilerSha256 = (Get-FileHash $compiler -Algorithm SHA256).Hash
    sdk = '10.0.26100.0'
    tools = [ordered]@{}
}
foreach ($tool in @('cmake', 'ninja', 'powershell-core')) {
    & "$env:VCPKG_ROOT/vcpkg.exe" fetch $tool
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    [xml]$manifest = Get-Content "$env:VCPKG_ROOT/scripts/vcpkgTools.xml" -Raw
    $definition = $manifest.tools.tool | Where-Object { $_.name -eq $tool -and $_.os -eq 'windows' }
    $identity.tools[$tool] = [ordered]@{ version = $definition.version; sha512 = $definition.sha512 }
}
$json = $identity | ConvertTo-Json -Depth 5 -Compress
$bytes = [Text.Encoding]::UTF8.GetBytes($json)
$fingerprint = [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($bytes)).ToLowerInvariant()
"fingerprint=$fingerprint" >> $env:GITHUB_OUTPUT
$json | Set-Content "$diagnostics/toolchain.json" -Encoding utf8
@("ImageOS=$env:ImageOS", "ImageVersion=$env:ImageVersion", "VisualStudio=$vs", "ShellPowerShell=$($PSVersionTable.PSVersion)") |
    Set-Content "$diagnostics/runner.txt" -Encoding utf8
Write-Host "Cache toolchain fingerprint: $fingerprint"
Write-Host $json
