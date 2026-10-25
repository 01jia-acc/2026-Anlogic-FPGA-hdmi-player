param([string]$ModelSimBin = 'E:\digitalsystem\win64')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source/SD')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_opening_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create library' }
    $sources = @('scrolling_subtitle.v','subtitle_font.v','audio4_subtitle_data.v','audio4_subtitle_timeline.v') | ForEach-Object { Join-Path $rtl $_ }
    $sources += "$PSScriptRoot/tb_opening_animation.v", "$PSScriptRoot/tb_audio4_subtitle.v"
    & (Join-Path $ModelSimBin 'vlog.exe') -work work @sources
    if ($LASTEXITCODE -ne 0) { throw 'Opening compilation failed' }
    foreach ($bench in @('tb_opening_animation','tb_audio4_subtitle')) {
        $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -suppress 2685,2718 -lib work "work.$bench" -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -f' 2>&1)
        $code = $LASTEXITCODE
        $log | Set-Content -LiteralPath (Join-Path $simDir "$bench.log")
        if ($code -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
            $log | Write-Output
            throw "Simulation failed: $bench"
        }
        $log | Where-Object { $_ -match 'PASS:' } | Write-Output
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
