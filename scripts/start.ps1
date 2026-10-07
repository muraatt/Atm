param([switch]$SetupOnly,[switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Set-Location -LiteralPath $projectRoot

function Find-PackageManager {
    $npmCommand = Get-Command npm.cmd -ErrorAction SilentlyContinue
    if ($npmCommand) { return @{ Executable=$npmCommand.Source; Prefix=@() } }
    $pnpmCommand = Get-Command pnpm.cmd -ErrorAction SilentlyContinue
    if ($pnpmCommand) { return @{ Executable=$pnpmCommand.Source; Prefix=@() } }
    $bundled = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\node\node_modules\pnpm\bin\pnpm.cjs'
    if (Test-Path -LiteralPath $bundled) { return @{ Executable='node'; Prefix=@($bundled) } }
    throw 'Node.js and npm (or pnpm) are required. Install Node.js LTS, then run start.cmd again.'
}

if (-not (Test-Path -LiteralPath '.venv\Scripts\python.exe')) {
    Write-Host 'Creating the local Python environment...'
    python -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw 'Python 3.12 or newer is required.' }
}
$pythonExecutable = Join-Path $projectRoot '.venv\Scripts\python.exe'
$dependencyCheck = & $pythonExecutable -c "import fastapi,uvicorn,numpy,scipy,astropy,pymsis,rasterio,httpx,shapely,netCDF4,casadi" 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host 'Installing calculation dependencies into the local environment...'
    & $pythonExecutable -m pip install -e '.[test]'
    if ($LASTEXITCODE -ne 0) { throw 'Python dependency installation failed.' }
}
$env:PYTHONPATH = Join-Path $projectRoot 'backend'
if (-not (Test-Path -LiteralPath 'frontend\dist\index.html')) {
    $manager = Find-PackageManager
    Push-Location -LiteralPath (Join-Path $projectRoot 'frontend')
    try {
        $prefixArguments = $manager.Prefix
        if (-not (Test-Path -LiteralPath 'node_modules\react')) {
            Write-Host 'Installing interface dependencies...'
            & $manager.Executable @prefixArguments install
            if ($LASTEXITCODE -ne 0) { throw 'Interface dependency installation failed.' }
        }
        Write-Host 'Building the interface...'
        & $manager.Executable @prefixArguments run build
        if ($LASTEXITCODE -ne 0) { throw 'Interface build failed.' }
    } finally { Pop-Location }
}
if ($SetupOnly) { Write-Host 'Atmosphere is ready.'; exit 0 }
$healthUrl = 'http://127.0.0.1:8000/api/health'
try {
    $existing = Invoke-RestMethod -Uri $healthUrl -TimeoutSec 2
    if ($existing.status -eq 'ok' -and $existing.version -eq '0.1.0') {
        Write-Host 'Atmosphere is already running at http://127.0.0.1:8000'
        if (-not $NoBrowser) { Start-Process 'http://127.0.0.1:8000' }
        exit 0
    }
} catch { }
if (-not $NoBrowser) {
    $openScript = Join-Path $PSScriptRoot 'open-browser.ps1'
    Start-Process powershell.exe -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-File',('"'+$openScript+'"')) -WindowStyle Hidden
}
Write-Host 'Atmosphere: http://127.0.0.1:8000'
Write-Host 'Keep this window open. Press Ctrl+C to stop the calculation service.'
& $pythonExecutable -m uvicorn atmosphere.main:app --host 127.0.0.1 --port 8000
