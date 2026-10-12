param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = Join-Path $PSScriptRoot '..\src\user_source\hdl_source'
$output = Join-Path $PSScriptRoot 'build\tb_pcm_file.vvp'
New-Item -ItemType Directory -Path (Split-Path $output) -Force | Out-Null
& (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s tb_pcm_file -o $output -y (Join-Path $rtl 'SD') -y (Join-Path $rtl 'audio') (Join-Path $PSScriptRoot 'tb_pcm_file.v')
if ($LASTEXITCODE -ne 0) { throw 'PCM compile failed' }
& (Join-Path $IverilogBin 'vvp.exe') $output
if ($LASTEXITCODE -ne 0) { throw 'PCM simulation failed' }
