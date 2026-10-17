# =============================================================================================
#  register-vci.ps1 — register the Tester Present RP1210 emulator as a system RP1210 device, using
#  the TPRP32.dll / TPRP32.INI shipped beside this script (the TPSAS-RP1210 app folder).
#  Launched elevated by the GUI's "Register VCI" button (UAC). Requires administrator.
# =============================================================================================
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$win  = $env:WINDIR
$sys  = Join-Path $win 'SysWOW64'

$dll = Join-Path $here 'TPRP32.dll'
$ini = Join-Path $here 'TPRP32.INI'
if (-not (Test-Path $dll)) { throw "TPRP32.dll not found beside this script ($dll)." }
if (-not (Test-Path $ini)) { throw "TPRP32.INI not found beside this script ($ini)." }

Copy-Item $dll (Join-Path $sys 'TPRP32.dll') -Force
Copy-Item $ini (Join-Path $win 'TPRP32.INI') -Force
Write-Host "Copied TPRP32.dll -> $sys"
Write-Host "Copied TPRP32.INI -> $win"

$rp = Join-Path $win 'RP121032.INI'
if (-not (Test-Path $rp)) {
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

Write-Host "`nDone. 'Tester Present RP1210' will now enumerate in RP1210 clients (protocols [J1939, Cdl, Ata])."
Read-Host "`nPress Enter to close"
