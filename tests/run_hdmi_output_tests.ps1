param([string]$ModelSimBin = 'E:\digitalsystem\win64')
$ErrorActionPreference = 'Stop'
$rtl = (Resolve-Path (Join-Path $PSScriptRoot '../src/user_source/hdl_source')).Path
$simDir = Join-Path ([System.IO.Path]::GetTempPath()) ('codex_hdmi_output_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $simDir | Out-Null
Push-Location $simDir
try {
    & (Join-Path $ModelSimBin 'vlib.exe') work
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create simulation library' }
    $sources = @("$rtl/hdmi_phy_warpper.v", "$rtl/lane_lvds_10_1.v",
        "$rtl/video_rgb_to_axis_640x480.v", "$rtl/SD/scrolling_subtitle.v", "$rtl/SD/subtitle_font.v",
        "$PSScriptRoot/tb_hdmi_phy_alignment.v", "$PSScriptRoot/tb_subtitle_axis.v")
    & (Join-Path $ModelSimBin 'vlog.exe') -work work @sources
    if ($LASTEXITCODE -ne 0) { throw 'HDMI output compilation failed' }
    $runs = @(0..7 | ForEach-Object { @{ Bench='tb_hdmi_phy_alignment'; Override="-GSERIAL_PHASE=$_" } })
    $runs += @{ Bench='tb_subtitle_axis'; Override=$null }
    foreach ($run in $runs) {
        $extra = @()
        if ($run.Override) { $extra += $run.Override }
        $log = @(& (Join-Path $ModelSimBin 'vsim.exe') -c -quiet -suppress 2685,2718 -lib work "work.$($run.Bench)" @extra -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -f' 2>&1)
        $code = $LASTEXITCODE
        $log | Set-Content -LiteralPath (Join-Path $simDir "$($run.Bench)$($run.Override).log")
        if ($code -ne 0 -or !($log -match 'PASS:') -or ($log -match '\*\* (Fatal|Error):')) {
            $log | Write-Output
            throw "HDMI output simulation failed: $($run.Bench) $extra"
        }
        $log | Where-Object { $_ -match 'PASS:' } | Write-Output
    }
} finally {
    Pop-Location
    Write-Output "Simulation logs: $simDir"
}
