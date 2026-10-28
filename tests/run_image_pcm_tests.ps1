param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$rtl = Join-Path $PSScriptRoot '..\src\user_source\hdl_source'
$output = Join-Path $PSScriptRoot 'build\tb_image_pcm.vvp'
New-Item -ItemType Directory -Path (Split-Path $output) -Force | Out-Null
& (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s tb_image_pcm -o $output -y (Join-Path $rtl 'SD') -y (Join-Path $rtl 'audio') (Join-Path $PSScriptRoot 'tb_image_pcm.v')
if ($LASTEXITCODE -ne 0) { throw 'Per-image PCM compile failed' }
& (Join-Path $IverilogBin 'vvp.exe') $output
if ($LASTEXITCODE -ne 0) { throw 'Per-image PCM simulation failed' }
foreach ($case in @(
    @{Name='auto_pcm'; First=1; Missing=0},
    @{Name='auto_pcm_sparse'; First=3; Missing=0},
    @{Name='auto_pcm_missing'; First=1; Missing=1}
)) {
    $output = Join-Path $PSScriptRoot ('build\tb_' + $case.Name + '.vvp')
    & (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -s tb_image_pcm '-Ptb_image_pcm.AUTO_TEST=1' `
        "-Ptb_image_pcm.FIRST_PIC=$($case.First)" "-Ptb_image_pcm.FIRST_AUDIO_MISSING=$($case.Missing)" `
        -o $output -y (Join-Path $rtl 'SD') -y (Join-Path $rtl 'audio') (Join-Path $PSScriptRoot 'tb_image_pcm.v')
    if ($LASTEXITCODE -ne 0) { throw "Slideshow PCM compile failed: $($case.Name)" }
    & (Join-Path $IverilogBin 'vvp.exe') $output
    if ($LASTEXITCODE -ne 0) { throw "Slideshow PCM simulation failed: $($case.Name)" }
}
