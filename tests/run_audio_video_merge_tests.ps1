param([string]$ModelSimBin = 'E:\digitalsystem\win64')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_audio_video_merge_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
$benches = @('tb_image_pcm', 'tb_pcm_file', 'tb_hdmi_audio', 'tb_pcm_media_tone',
    'tb_four_buffer_audio', 'tb_four_bank_memory', 'tb_image_zoom', 'tb_video_wipe',
    'tb_video_fade_in', 'tb_thumbnail_osd', 'tb_bilinear_image', 'tb_fat32_file_geometry')
$sources = @(Get-ChildItem "$rtl/SD/*.v" | Where-Object Name -ne 'sdram.v' | ForEach-Object FullName)
$sources += @(Get-ChildItem "$rtl/audio/*.v" | ForEach-Object FullName)
$sources += "$rtl/IP/afifo_32_16_256.v"
$sources += @($benches | ForEach-Object { Join-Path $PSScriptRoot "$_.v" })
function Invoke-Run([string]$bench, [string[]]$overrides = @()) {
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
    $compileLog = @(& (Join-Path $ModelSimBin 'vlog.exe') -work work "+incdir+$rtl/SD" @sources 2>&1)
    $compileLog | Set-Content -LiteralPath (Join-Path $simDir 'compile.log')
    if ($LASTEXITCODE -ne 0) { $compileLog | Write-Output; throw 'Compilation failed' }
    foreach ($bench in $benches) { Invoke-Run $bench }
    Invoke-Run 'tb_pcm_file' @('-GBMP_WIDTH=319', '-GBMP_HEIGHT=239')
    Invoke-Run 'tb_four_bank_memory' @('-GMEM_HALF_NS=10')
    # New 50 MHz SDRAM domain: sustained full-resolution zoom with FIFO stalls.
    Invoke-Run 'tb_image_zoom' @('-GMEM_HALF_NS=10', '-GWIDTH=640', '-GHEIGHT=480', '-GBURST=128', '-GFIFO_DEPTH=512', '-GSTALLS=1')
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
