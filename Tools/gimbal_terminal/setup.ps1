$ErrorActionPreference = "Stop"
$TerminalRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvPath = Join-Path $TerminalRoot ".venv"

if (-not (Test-Path $VenvPath)) {
    py -3 -m venv $VenvPath
}

$PythonExe = Join-Path $VenvPath "Scripts\python.exe"
& $PythonExe -m pip install --upgrade pip
& $PythonExe -m pip install -r (Join-Path $TerminalRoot "requirements.txt")

Write-Host "Python environment is ready." -ForegroundColor Green
Write-Host "HIKROBOT MVS and its USB3 Vision driver must be installed separately." -ForegroundColor Yellow
