param(
    [Parameter(Mandatory = $true)]
    [string] $LinkerPath
)

$content = Get-Content -Path $LinkerPath -Raw -Encoding ASCII
$reserved = @"
    FLASH           (RX)  : ORIGIN = 0x00000000, LENGTH = 0x0001FC00
    IMU_CAL_FLASH   (R)   : ORIGIN = 0x0001FC00, LENGTH = 0x00000400
"@

if ($content -match 'IMU_CAL_FLASH') {
    $content = $content -replace `
        '    FLASH\s+\(RX\)\s+:\s+ORIGIN\s+=\s+0x00000000,\s+LENGTH\s+=\s+0x[0-9A-Fa-f]+', `
        '    FLASH           (RX)  : ORIGIN = 0x00000000, LENGTH = 0x0001FC00'
    $content = $content -replace `
        '    IMU_CAL_FLASH\s+\(R\)\s+:\s+ORIGIN\s+=\s+0x[0-9A-Fa-f]+,\s+LENGTH\s+=\s+0x[0-9A-Fa-f]+', `
        '    IMU_CAL_FLASH   (R)   : ORIGIN = 0x0001FC00, LENGTH = 0x00000400'
} else {
    $content = $content -replace `
        '    FLASH\s+\(RX\)\s+:\s+ORIGIN\s+=\s+0x00000000,\s+LENGTH\s+=\s+0x00020000', `
        $reserved
}

Set-Content -Path $LinkerPath -Value $content -Encoding ASCII
