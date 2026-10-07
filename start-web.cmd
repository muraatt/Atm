@echo off
cd /d "%~dp0"
if "%~1"=="" (
  echo Usage: start-web.cmd https://your-project.vercel.app
  echo Use the exact production website address.
  exit /b 1
)
if not exist ".venv\Scripts\python.exe" call start.cmd -SetupOnly
if not exist ".venv\Scripts\python.exe" exit /b 1
".venv\Scripts\python.exe" "scripts\allow_web_origin.py" "%~1"
if errorlevel 1 exit /b 1
call start.cmd -NoBrowser
