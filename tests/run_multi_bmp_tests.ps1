param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = Join-Path $PSScriptRoot '..\src\user_source\hdl_source'
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
foreach ($bench in @('tb_multi_bmp_cache', 'tb_multi_bmp_scanner', 'tb_multi_bmp_osd')) {
    $output = Join-Path $buildDir "$bench.vvp"
    & (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s $bench -o $output -y (Join-Path $rtl 'SD') -y (Join-Path $rtl 'audio') (Join-Path $PSScriptRoot "$bench.v")
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $bench" }
    & (Join-Path $IverilogBin 'vvp.exe') $output
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $bench" }
}
