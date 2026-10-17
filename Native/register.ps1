# =============================================================================================
#  register.ps1  —  install the Tester Present RP1210 emulator as a standard RP1210 device.
#  Requires elevation (writes to %WINDIR% and %WINDIR%\SysWOW64).
#
#  Installs:
#    - TPRP32.dll  -> %WINDIR%\SysWOW64\TPRP32.dll   (32-bit, loadable by the 32-bit tool)
#    - TPRP32.INI  -> %WINDIR%\TPRP32.INI            (vendor descriptor)
#    - adds "TPRP32" to [RP1210Support] APIImplementations in %WINDIR%\RP121032.INI
#
#  Run from this folder after build_emu.bat:  (elevated PowerShell)
#    powershell -ExecutionPolicy Bypass -File register.ps1
#  Or double-click register.bat (it self-elevates).
# =============================================================================================
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$win  = $env:WINDIR
$sys  = Join-Path $win 'SysWOW64'           # 32-bit DLL home on 64-bit Windows

$dll = Join-Path $here 'build\TPRP32.dll'
$ini = Join-Path $here 'TPRP32.INI'
if (-not (Test-Path $dll)) { throw "Build first: $dll not found (run build_emu.bat)." }
if (-not (Test-Path $ini)) { throw "TPRP32.INI not found next to this script." }

Copy-Item $dll (Join-Path $sys 'TPRP32.dll') -Force
Copy-Item $ini (Join-Path $win 'TPRP32.INI') -Force
Write-Host "Copied TPRP32.dll -> $sys"
Write-Host "Copied TPRP32.INI -> $win"

$rp = Join-Path $win 'RP121032.INI'
if (-not (Test-Path $rp)) {
    # Create a minimal RP121032.INI if none exists.
    "[RP1210Support]`r`nAPIImplementations=TPRP32`r`n" | Set-Content -Path $rp -Encoding ASCII
    Write-Host "Created $rp with APIImplementations=TPRP32"
} else {
    $content = Get-Content $rp
    $line = $content | Where-Object { $_ -match '^\s*APIImplementations\s*=' }
    if (-not $line) {
        Add-Content -Path $rp -Value "APIImplementations=TPRP32"
        Write-Host "Added APIImplementations=TPRP32 to $rp"
    } elseif ($line -notmatch '\bTPRP32\b') {
        $new = $line -replace '(APIImplementations\s*=\s*)(.*)', ('$1$2,TPRP32')
        $content = $content -replace [regex]::Escape($line), $new
        Set-Content -Path $rp -Value $content -Encoding ASCII
        Write-Host "Appended TPRP32 to APIImplementations in $rp"
    } else {
        Write-Host "TPRP32 already present in $rp"
    }
}

Write-Host "`nDone. In the ADEM Flash Programming Tool click Device -> Refresh interfaces;"
Write-Host "'Tester Present RP1210' should appear with protocols [J1939, Cdl, Ata]."
