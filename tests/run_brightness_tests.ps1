param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
foreach ($bench in @('tb_brightness_control', 'tb_video_brightness', 'tb_brightness_osd', 'tb_brightness_axis', 'tb_video_fade_in')) {
    $output = Join-Path $buildDir "$bench.vvp"
    & (Join-Path $IverilogBin 'iverilog.exe') -g2005 -s $bench -o $output `
        -y "$rtl/SD" -y "$rtl/audio" -y "$rtl/uart" "$rtl/SD/media_frame_reader.v" "$rtl/SD/osd_overlay_24.v" `
        "$rtl/SD/subtitle_font.v" "$rtl/video_rgb_to_axis_640x480.v" `
        "$PSScriptRoot/tb_thumbnail_control.v" "$PSScriptRoot/tb_brightness_control.v" `
        "$PSScriptRoot/tb_video_brightness.v" "$PSScriptRoot/tb_volume_osd.v" `
        "$PSScriptRoot/tb_brightness_axis.v" "$PSScriptRoot/tb_waveform_axis.v" "$PSScriptRoot/tb_video_fade_in.v"
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $bench" }
    & (Join-Path $IverilogBin 'vvp.exe') $output
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $bench" }
}
