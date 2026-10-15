param(
    [string]$ModelSimBin = 'E:\digitalsystem\win64',
    [switch]$FullSize
)
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_scaling_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
$sources = @(Get-ChildItem "$rtl/SD/*.v" | Where-Object Name -ne 'sdram.v' | ForEach-Object FullName)
$sources += @(Get-ChildItem "$rtl/audio/*.v" | ForEach-Object FullName)
$sources += @(Get-ChildItem "$rtl/uart/*.v" | ForEach-Object FullName)
$sources += @('tb_bilinear_image', 'tb_pcm_file', 'tb_fat32_file_geometry', 'tb_uart_diagnostics' | ForEach-Object { Join-Path $PSScriptRoot "$_.v" })
function Invoke-CheckedSimulation([string]$bench, [string[]]$overrides = @()) {
    $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -suppress 2685,2718 -lib work "work.$bench" @overrides -do 'run -all; quit -f' 2>&1)
    $exitCode = $LASTEXITCODE
    $log | Set-Content -LiteralPath (Join-Path $simDir "$bench$($overrides -join '_').log")
    if ($exitCode -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
        $log | Write-Output
        throw "Simulation failed: $bench $overrides"
    }
    $log | Where-Object { $_ -match 'PASS:' } | Write-Output
}
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create simulation library' }
    & (Join-Path $ModelSimBin 'vlog.exe') -work work "+incdir+$rtl/SD" @sources
    if ($LASTEXITCODE -ne 0) { throw 'RTL/test compilation failed' }
    Invoke-CheckedSimulation 'tb_bilinear_image'
    Invoke-CheckedSimulation 'tb_fat32_file_geometry'
    Invoke-CheckedSimulation 'tb_uart_diagnostics'
    Invoke-CheckedSimulation 'tb_pcm_file'
    Invoke-CheckedSimulation 'tb_pcm_file' @('-GBMP_WIDTH=319', '-GBMP_HEIGHT=239')
    if ($FullSize) {
        Invoke-CheckedSimulation 'tb_bilinear_image' @('-GWIDTH=640', '-GHEIGHT=480', '-GFULL_SIZE=1')
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
