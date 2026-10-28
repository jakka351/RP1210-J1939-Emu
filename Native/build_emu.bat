@echo off
REM =============================================================================================
REM  build_emu.bat — build the Tester Present RP1210 emulator (TPRP32.dll, x86) + its tools.
REM  Robust across VS editions: finds the C++ toolchain with vswhere and sets up the compiler
REM  + Windows SDK via vcvarsall.bat.
REM
REM  "Cannot open include file: 'ctype.h' / 'math.h' / 'crtdbg.h'"  => the Windows SDK (Universal
REM  CRT) is missing. Visual Studio Installer -> Modify -> "Desktop development with C++", re-run.
REM =============================================================================================
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" set "VSWHERE=%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" ( echo ERROR: vswhere.exe not found. Install VS with "Desktop development with C++". & exit /b 1 )

set "VSINSTALL="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * ^
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 ^
    -property installationPath`) do set "VSINSTALL=%%i"
if not defined VSINSTALL (
  echo ERROR: No Visual Studio install with the C++ tools was found.
  echo        Visual Studio Installer -^> Modify -^> "Desktop development with C++", then re-run.
  exit /b 1
)
set "VCVARS=%VSINSTALL%\VC\Auxiliary\Build\vcvarsall.bat"
if not exist "%VCVARS%" ( echo ERROR: vcvarsall.bat not found under "%VSINSTALL%". & exit /b 1 )

echo Using Visual Studio at: %VSINSTALL%
call "%VCVARS%" x86
if errorlevel 1 exit /b 1

if not exist build mkdir build

echo === TPRP32.dll ===
cl /nologo /LD /EHsc /std:c++17 /W3 /MT /DWIN32 /D_WINDOWS /D_CRT_SECURE_NO_WARNINGS ^
   rp1210_emu.cpp ecm_sim.cpp ^
   /Fobuild\ ^
   /link /DEF:TPRP32.def /OUT:build\TPRP32.dll
if errorlevel 1 ( echo DLL BUILD FAILED & exit /b 1 )

echo === emu_selftest.exe ===
cl /nologo /EHsc /std:c++17 /W3 /MT /D_CRT_SECURE_NO_WARNINGS ^
   emu_selftest.cpp /Fobuild\ /Febuild\emu_selftest.exe
if errorlevel 1 ( echo SELFTEST BUILD FAILED & exit /b 1 )

echo === emu_monitor.exe ===
cl /nologo /EHsc /std:c++17 /W3 /MT /D_CRT_SECURE_NO_WARNINGS ^
   emu_monitor.cpp /Fobuild\ /Febuild\emu_monitor.exe
if errorlevel 1 ( echo MONITOR BUILD FAILED & exit /b 1 )

echo BUILD OK: build\TPRP32.dll  build\emu_selftest.exe  build\emu_monitor.exe
endlocal
