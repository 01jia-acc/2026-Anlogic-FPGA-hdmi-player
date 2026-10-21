param(
    [string]$ModelSimBin = 'E:\digitalsystem\win64',
    [switch]$FullSize
)
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_transition_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
$sources = @(Get-ChildItem "$rtl/SD/*.v" | Where-Object Name -ne 'sdram.v' | ForEach-Object FullName)
$sources += @(Get-ChildItem "$rtl/audio/*.v" | ForEach-Object FullName)
$sources += "$rtl/IP/afifo_32_16_256.v"
$sources += "$rtl/top_tf_hdmi_audio.v"
$benches = @('tb_thumbnail_control', 'tb_thumbnail_osd', 'tb_image_zoom', 'tb_video_wipe', 'tb_video_fade_in', 'tb_four_bank_memory', 'tb_four_buffer_audio', 'tb_multi_bmp_cache')
$sources += @($benches | ForEach-Object { Join-Path $PSScriptRoot "$_.v" })
function Invoke-CheckedSimulation([string]$bench, [string[]]$overrides = @()) {
    $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -suppress 2685,2718 -lib work "work.$bench" @overrides -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -f' 2>&1)
    $exitCode = $LASTEXITCODE
    $log | Set-Content -LiteralPath (Join-Path $simDir "$bench$($overrides -join '_').log")
    if ($exitCode -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
        $log | Write-Output
        throw "Simulation failed: $bench $overrides"
    }
    $log | Where-Object { $_ -match 'PASS:' } | Write-Output
}
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create simulation library' }
    & (Join-Path $ModelSimBin 'vlog.exe') -work work "+incdir+$rtl/SD" @sources
    if ($LASTEXITCODE -ne 0) { throw 'RTL/test compilation failed' }
    foreach ($bench in $benches) { Invoke-CheckedSimulation $bench }
    Invoke-CheckedSimulation 'tb_multi_bmp_cache' @('-GWIPE_FRAMES=4')
    if ($FullSize) {
        Invoke-CheckedSimulation 'tb_image_zoom' @('-GWIDTH=640', '-GHEIGHT=480', '-GBURST=128', '-GFIFO_DEPTH=512')
        Invoke-CheckedSimulation 'tb_image_zoom' @('-GWIDTH=640', '-GHEIGHT=480', '-GBURST=128', '-GFIFO_DEPTH=512', '-GSTALLS=1')
        Invoke-CheckedSimulation 'tb_video_wipe' @('-GWIDTH=640', '-GHEIGHT=480', '-GFRAMES=16', '-GBURST=128', '-GFIFO_DEPTH=512')
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
