param([string]$IverilogBin = 'F:\FPGA_anlu\iverilog\bin')
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$rtlDir = Join-Path $repoRoot 'src\user_source\hdl_source\audio'
$sources = @(
    (Join-Path $rtlDir 'pcm_test_tone.v'),
    (Join-Path $rtlDir 'pcm_media_tone.v'),
    (Join-Path $rtlDir 'media_hdmi_audio_bridge.v')
)
foreach ($test in @('tb_hdmi_audio', 'tb_pcm_media_tone')) {
    $output = Join-Path $buildDir ($test + '.vvp')
    & (Join-Path $IverilogBin 'iverilog.exe') -g2005-sv -Wall -s $test -o $output (Join-Path $PSScriptRoot ($test + '.v')) @sources
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $test" }
    & (Join-Path $IverilogBin 'vvp.exe') $output
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $test" }
}
