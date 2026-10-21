param([string]$ModelSimBin = 'E:\digitalsystem\win64')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_waveform_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
$benches = @('tb_audio_waveform', 'tb_waveform_axis', 'tb_subtitle_axis', 'tb_scrolling_subtitle')
$sources = @(Get-ChildItem "$rtl/SD/*.v", "$rtl/audio/*.v", "$rtl/uart/*.v" | Where-Object Name -ne 'sdram.v' | ForEach-Object FullName)
$sources += "$rtl/video_rgb_to_axis_640x480.v"
$sources += @($benches | ForEach-Object { Join-Path $PSScriptRoot "$_.v" })
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create simulation library' }
    $log = @(& (Join-Path $ModelSimBin 'vlog.exe') -work work "+incdir+$rtl/SD" @sources 2>&1)
    $log | Set-Content (Join-Path $simDir 'compile.log')
    if ($LASTEXITCODE -ne 0) { $log | Write-Output; throw 'Compilation failed' }
    foreach ($bench in $benches) {
        $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -lib work "work.$bench" -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -f' 2>&1)
        $simExit = $LASTEXITCODE
        $log | Set-Content (Join-Path $simDir "$bench.log")
        if ($simExit -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
            $log | Write-Output; throw "Simulation failed: $bench"
        }
        $log | Where-Object { $_ -match 'PASS:' } | Write-Output
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
