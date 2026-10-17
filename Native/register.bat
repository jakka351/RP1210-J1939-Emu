@echo off
REM Self-elevating installer for the Tester Present RP1210 emulator.
REM Re-launches register.ps1 as administrator (UAC prompt).
cd /d "%~dp0"
net session >nul 2>&1
if %errorlevel% neq 0 (
  echo Requesting administrator privileges...
  powershell -Command "Start-Process -Verb RunAs -FilePath powershell -ArgumentList '-ExecutionPolicy Bypass -NoProfile -File \"%~dp0register.ps1\"'"
  exit /b
)
powershell -ExecutionPolicy Bypass -NoProfile -File "%~dp0register.ps1"
pause
