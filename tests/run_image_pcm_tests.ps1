param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = Join-Path $PSScriptRoot '..\src\user_source\hdl_source'
$output = Join-Path $PSScriptRoot 'build\tb_image_pcm.vvp'
New-Item -ItemType Directory -Path (Split-Path $output) -Force | Out-Null
& (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s tb_image_pcm -o $output -y (Join-Path $rtl 'SD') -y (Join-Path $rtl 'audio') (Join-Path $PSScriptRoot 'tb_image_pcm.v')
if ($LASTEXITCODE -ne 0) { throw 'Per-image PCM compile failed' }
& (Join-Path $IverilogBin 'vvp.exe') $output
if ($LASTEXITCODE -ne 0) { throw 'Per-image PCM simulation failed' }
