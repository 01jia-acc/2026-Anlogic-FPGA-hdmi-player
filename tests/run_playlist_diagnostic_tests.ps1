param([string]$ModelSimBin = 'E:\digitalsystem\win64')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_playlist_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
$sources = @(Get-ChildItem "$rtl/SD/*.v" | Where-Object Name -ne 'sdram.v' | ForEach-Object FullName)
$sources += @(Get-ChildItem "$rtl/audio/*.v" | ForEach-Object FullName)
$sources += @(Get-ChildItem "$rtl/uart/*.v" | ForEach-Object FullName)
$sources += "$rtl/IP/afifo_32_16_256.v"
$sources += "$rtl/top_tf_hdmi_audio.v"
$benches = @('tb_uart_diagnostics', 'tb_multi_bmp_scanner', 'tb_multi_bmp_cache', 'tb_thumbnail_control', 'tb_mode_switch_control')
$sources += @($benches | Where-Object { $_ -ne 'tb_mode_switch_control' } | ForEach-Object { Join-Path $PSScriptRoot "$_.v" })
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create simulation library' }
    $compileLog = @(& (Join-Path $ModelSimBin 'vlog.exe') -work work "+incdir+$rtl/SD" @sources 2>&1)
    $compileLog | Set-Content -LiteralPath (Join-Path $simDir 'compile.log')
    if ($LASTEXITCODE -ne 0) { $compileLog | Write-Output; throw 'Compilation failed' }
    foreach ($bench in $benches) {
        $variants = @('')
        if ($bench -eq 'tb_uart_diagnostics') { $variants += '-GCOMPACT_LOG=1' }
        foreach ($variant in $variants) {
        $extra = @()
        if ($variant) { $extra += $variant }
        $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -suppress 2685,2718 -lib work "work.$bench" @extra -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -f' 2>&1)
        $exitCode = $LASTEXITCODE
        $log | Set-Content -LiteralPath (Join-Path $simDir "$bench$variant.log")
        if ($exitCode -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
            $log | Write-Output
            throw "Simulation failed: $bench"
        }
        $log | Where-Object { $_ -match 'PASS:' } | Write-Output
        }
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
