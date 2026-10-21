param([ValidatePattern('^[A-Za-z]$')][string]$DriveLetter = 'G')
$ErrorActionPreference = 'Stop'
# Read-only volume access; only local simulation inputs are written.
$outputDir = Join-Path $PSScriptRoot 'build/card_scan'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$volume = [IO.File]::Open(('\\.\' + $DriveLetter + ':'), [IO.FileMode]::Open,
    [IO.FileAccess]::Read, [IO.FileShare]::ReadWrite)
try {
    function Read-CardSector([long]$Lba) {
        $bytes = [byte[]]::new(512)
        $null = $volume.Seek($Lba * 512, [IO.SeekOrigin]::Begin)
        if ($volume.Read($bytes, 0, 512) -ne 512) { throw "Short sector read: $Lba" }
        return ,$bytes
    }
    $boot = Read-CardSector 0
    if ([BitConverter]::ToUInt16($boot,11) -ne 512 -or $boot[13] -eq 0) {
        throw 'Expected a FAT32 volume boot sector'
    }
    $fat = [BitConverter]::ToUInt16($boot,14)
    $spc = $boot[13]
    $data = $fat + $boot[16] * [BitConverter]::ToUInt32($boot,36)
    $cluster = [BitConverter]::ToUInt32($boot,44)
    $sectors = [ordered]@{}
    $sectors[0L] = $boot
    $ended = $false
    $visited = 0
    while (!$ended) {
        if (++$visited -gt 1024 -or $cluster -lt 2) { throw 'Invalid root chain' }
        $rootLba = [long]$data + ([long]$cluster - 2) * $spc
        for ($s=0; $s -lt $spc; $s++) {
            $bytes = Read-CardSector ($rootLba+$s)
            $sectors[[long]($rootLba+$s)] = $bytes
            for ($i=0; $i -lt 512; $i+=32) {
                if ($bytes[$i] -eq 0) { $ended=$true; break }
            }
            if ($ended) { break }
        }
        if (!$ended) {
            $fatLba = [long]$fat + [long][Math]::Floor($cluster/128)
            $bytes = Read-CardSector $fatLba
            $sectors[$fatLba] = $bytes
            $cluster = [BitConverter]::ToUInt32($bytes,($cluster%128)*4) -band 0x0fffffff
            if ($cluster -ge 0x0ffffff8) { $ended=$true }
        }
    }
    $hex = [Collections.Generic.List[string]]::new()
    $addresses = [Collections.Generic.List[string]]::new()
    foreach ($lba in $sectors.Keys) {
        $addresses.Add(('{0:x8}' -f $lba))
        foreach ($byte in $sectors[$lba]) { $hex.Add(('{0:x2}' -f $byte)) }
    }
    [IO.File]::WriteAllLines((Join-Path $outputDir 'bytes.hex'),$hex)
    [IO.File]::WriteAllLines((Join-Path $outputDir 'lbas.hex'),$addresses)
    Write-Output "Captured $($sectors.Count) sectors from $DriveLetter`: read-only into $outputDir"
    Write-Output "Run simulator with +SECTORS=$($sectors.Count)"
} finally { $volume.Dispose() }
