# TCM HMI — delega en iniciar_hmi.py (venv opcional, Chrome, build UI)
param(
    [int]$Port = 5050,
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
$HmiRoot = Split-Path $PSScriptRoot -Parent
$Launcher = Join-Path $HmiRoot 'iniciar_hmi.py'

$py = Join-Path (Split-Path $HmiRoot -Parent) '.venv\Scripts\python.exe'
if (-not (Test-Path $py)) {
    $py = 'python'
}

$args = @('-u', $Launcher, '--restart', '--port', $Port)
if ($NoBrowser) { $args += '--no-browser' }

Set-Location $HmiRoot
& $py @args
exit $LASTEXITCODE
