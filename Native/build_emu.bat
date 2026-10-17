@echo off
REM Build the Tester Present RP1210 emulator (TPRP32.dll, x86) and its standalone self-test.
setlocal
cd /d "%~dp0"
set VCVARS="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
if not exist %VCVARS% ( echo ERROR: vcvarsall.bat not found & exit /b 1 )
call %VCVARS% x86
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
