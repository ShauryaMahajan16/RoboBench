@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo RoboBench's Python environment was not found.
    echo Create it with: python -m venv .venv
    pause
    exit /b 1
)

".venv\Scripts\python.exe" "noise_gui.py"

if errorlevel 1 (
    echo.
    echo RoboBench stopped with an error. See the message above.
    pause
)

endlocal