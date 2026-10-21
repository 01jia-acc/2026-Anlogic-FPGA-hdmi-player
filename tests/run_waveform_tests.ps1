param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
foreach ($bench in @('tb_audio_waveform', 'tb_waveform_axis', 'tb_subtitle_axis', 'tb_scrolling_subtitle')) {
    $output = Join-Path $buildDir "$bench.vvp"
    & (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s $bench -o $output `
        -y "$rtl/SD" -y "$rtl/audio" -y "$rtl/uart" "$rtl/video_rgb_to_axis_640x480.v" `
        "$rtl/SD/subtitle_font.v" (Join-Path $PSScriptRoot "$bench.v")
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $bench" }
    & (Join-Path $IverilogBin 'vvp.exe') $output
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $bench" }
}
