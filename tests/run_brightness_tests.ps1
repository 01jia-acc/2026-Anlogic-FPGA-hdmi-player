param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
foreach ($bench in @('tb_brightness_control', 'tb_video_brightness', 'tb_brightness_osd', 'tb_contrast_osd', 'tb_brightness_axis', 'tb_video_fade_in')) {
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
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot '../src/td_project')).Path
$projectText = Get-Content -LiteralPath (Join-Path $projectDir 'lab_ex5_i2s_v1.0.al') -Raw
$projectSources = @([regex]::Matches($projectText, '<File Path="([^"]+\.v)"') |
    ForEach-Object { $_.Groups[1].Value } |
    Where-Object { $_ -notmatch '\.enc\.v$' } |
    ForEach-Object { [System.IO.Path]::GetFullPath((Join-Path $projectDir $_)) })
$resetOutput = Join-Path $buildDir 'tb_sw1_key1_reset.vvp'
& (Join-Path $IverilogBin 'iverilog.exe') -g2005 -i -I "$rtl/SD" -I $projectDir -s tb_sw1_key1_reset -o $resetOutput `
    "$PSScriptRoot/tb_sw1_key1_reset.v" @projectSources
if ($LASTEXITCODE -ne 0) { throw 'Compile failed: tb_sw1_key1_reset' }
& (Join-Path $IverilogBin 'vvp.exe') $resetOutput
if ($LASTEXITCODE -ne 0) { throw 'Simulation failed: tb_sw1_key1_reset' }
