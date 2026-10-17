# =============================================================================================
#  unregister.ps1 — remove the Tester Present RP1210 emulator. Requires elevation.
# =============================================================================================
$ErrorActionPreference = 'SilentlyContinue'
$win = $env:WINDIR
$sys = Join-Path $win 'SysWOW64'

Remove-Item (Join-Path $sys 'TPRP32.dll') -Force
Remove-Item (Join-Path $win 'TPRP32.INI') -Force
Write-Host "Removed TPRP32.dll and TPRP32.INI"

$rp = Join-Path $win 'RP121032.INI'
if (Test-Path $rp) {
    $content = Get-Content $rp
    $line = $content | Where-Object { $_ -match '^\s*APIImplementations\s*=' }
    if ($line -and $line -match '\bTPRP32\b') {
        $vals = ($line -replace '.*=\s*', '') -split ',' | ForEach-Object { $_.Trim() } | Where-Object { $_ -and $_ -ne 'TPRP32' }
        $new = 'APIImplementations=' + ($vals -join ',')
        $content = $content -replace [regex]::Escape($line), $new
        Set-Content -Path $rp -Value $content -Encoding ASCII
        Write-Host "Removed TPRP32 from APIImplementations in $rp"
    }
}
Write-Host "Done."
