@echo off
setlocal
set "TERMINAL_ROOT=%~dp0"
set "PYTHON_EXE=%TERMINAL_ROOT%.venv\Scripts\python.exe"

if not exist "%PYTHON_EXE%" (
    echo Python environment is missing. Running setup...
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%TERMINAL_ROOT%setup.ps1"
    if errorlevel 1 exit /b 1
)

"%PYTHON_EXE%" "%TERMINAL_ROOT%app.py"
endlocal
