param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$rtl = Join-Path $repoRoot 'src/user_source/hdl_source'
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
# Functional verification at the new 50 MHz memory clock. This cannot model
# analog SDRAM/PLL behavior: check the routed timing report and the real board.
foreach ($bench in @('tb_four_bank_memory', 'tb_image_zoom')) {
    $output = Join-Path $buildDir "$($bench)_50m.vvp"
    $overrides = @("-P$bench.MEM_HALF_NS=10")
    if ($bench -eq 'tb_image_zoom') {
        $overrides += @('-Ptb_image_zoom.WIDTH=640', '-Ptb_image_zoom.HEIGHT=480',
            '-Ptb_image_zoom.BURST=128', '-Ptb_image_zoom.FIFO_DEPTH=512')
    }
    & (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s $bench @overrides -o $output `
        -y (Join-Path $rtl 'SD') (Join-Path $PSScriptRoot "$bench.v") `
        (Join-Path $rtl 'IP/afifo_32_16_256.v')
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $bench" }
    & (Join-Path $IverilogBin 'vvp.exe') $output
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $bench" }
}
