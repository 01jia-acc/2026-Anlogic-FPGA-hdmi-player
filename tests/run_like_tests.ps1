param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
Push-Location (Join-Path $PSScriptRoot '..')
try {
    foreach ($bench in @('tb_image_like_control', 'tb_image_likes_osd')) {
        $output = Join-Path $buildDir "$bench.vvp"
        & (Join-Path $IverilogBin 'iverilog.exe') -g2005 -s $bench -o $output `
            -y "$rtl/SD" -y "$rtl/audio" "$rtl/SD/media_frame_reader.v" "$rtl/SD/osd_overlay_24.v" `
            "$PSScriptRoot/tb_thumbnail_control.v" "$PSScriptRoot/tb_image_likes_osd.v"
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $bench" }
        & (Join-Path $IverilogBin 'vvp.exe') $output
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $bench" }
    }
} finally { Pop-Location }
