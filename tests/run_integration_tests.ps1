param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$rtl = Join-Path $repoRoot 'src\user_source\hdl_source'
$audio = @('pcm_test_tone.v','pcm_media_tone.v','media_hdmi_audio_bridge.v') |
    ForEach-Object { Join-Path $rtl "audio\$_" }
$output = Join-Path $buildDir 'tb_four_buffer_audio.vvp'
& (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -Wall -s tb_four_buffer_audio -o $output -y (Join-Path $rtl 'SD') (Join-Path $PSScriptRoot 'tb_four_buffer_audio.v') @audio
if ($LASTEXITCODE -ne 0) { throw 'Compile failed: tb_four_buffer_audio' }
& (Join-Path $IverilogBin 'vvp.exe') $output
if ($LASTEXITCODE -ne 0) { throw 'Simulation failed: tb_four_buffer_audio' }

$output = Join-Path $buildDir 'tb_four_bank_memory.vvp'
& (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -Wall -s tb_four_bank_memory -o $output -y (Join-Path $rtl 'SD') (Join-Path $PSScriptRoot 'tb_four_bank_memory.v') (Join-Path $rtl 'IP\afifo_32_16_256.v')
if ($LASTEXITCODE -ne 0) { throw 'Compile failed: tb_four_bank_memory' }
& (Join-Path $IverilogBin 'vvp.exe') $output
if ($LASTEXITCODE -ne 0) { throw 'Simulation failed: tb_four_bank_memory' }
