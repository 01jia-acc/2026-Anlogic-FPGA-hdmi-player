param(
    [string]$ModelSimBin = 'E:\digitalsystem\win64',
    [switch]$FullSize
)
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_subtitle_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create simulation library' }
    $sources = @("$rtl/SD/scrolling_subtitle.v", "$rtl/SD/subtitle_font.v", "$rtl/uart/uart_rx.v", "$PSScriptRoot/tb_scrolling_subtitle.v")
    & (Join-Path $ModelSimBin 'vlog.exe') -work work @sources
    if ($LASTEXITCODE -ne 0) { throw 'Subtitle compilation failed' }
    $runs = [System.Collections.Generic.List[object]]::new()
    $runs.Add([string[]]@())
    if ($FullSize) { $runs.Add([string[]]@('-GWIDTH=640', '-GHEIGHT=480', '-GTEXT_Y=442')) }
    foreach ($overrides in $runs) {
        $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -suppress 2685,2718 -lib work work.tb_scrolling_subtitle @overrides -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -f' 2>&1)
        $exitCode = $LASTEXITCODE
        $log | Set-Content -LiteralPath (Join-Path $simDir "subtitle$($overrides -join '_').log")
        if ($exitCode -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
            $log | Write-Output
            throw 'Subtitle simulation failed'
        }
        $log | Where-Object { $_ -match 'PASS:' } | Write-Output
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
