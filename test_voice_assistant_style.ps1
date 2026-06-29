param(
    [string]$PortName = "COM6",
    [int]$BaudRate = 115200,
    [int]$RecordSeconds = 5,
    [string]$LogFile = "voice_assistant_style.log"
)

# Voice-Assistant style record-then-playback test for the RE1.0 + ES8388 board.
# Uses the PlatformIO firmware in this repo (no ESPHome / HASS required).
#
# Verified hardware mapping:
#   I2C : SDA=GPIO14, SCL=GPIO47, ES8388 @ 0x10
#   I2S : MCLK=GPIO8, BCLK=GPIO3, WS=GPIO9, DOUT=GPIO46, DIN=GPIO10
#
# Flow: STATUS -> REC_START (RecordSeconds) -> REC_STOP -> PLAY -> STATUS

$ErrorActionPreference = "Stop"
if (Test-Path $LogFile) { Remove-Item $LogFile -Force }

$port = New-Object System.IO.Ports.SerialPort `
    $PortName, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.NewLine = "`r`n"
$port.ReadTimeout = 50
$port.Handshake = [System.IO.Ports.Handshake]::None
$port.DtrEnable = $true
$port.RtsEnable = $false
$port.ReadBufferSize = 65536

function Pump([int]$ms) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = ""
    while ($sw.ElapsedMilliseconds -lt $ms) {
        try { $raw = $port.ReadExisting() } catch { $raw = "" }
        if (-not [string]::IsNullOrEmpty($raw)) {
            $buf += $raw
            while ($buf.Contains("`n")) {
                $idx = $buf.IndexOf("`n")
                $line = $buf.Substring(0, $idx).TrimEnd("`r")
                $buf = $buf.Substring($idx + 1)
                if (-not [string]::IsNullOrEmpty($line)) {
                    $stamp = Get-Date -Format "HH:mm:ss.fff"
                    $out = "[$stamp] $line"
                    Write-Host $out
                    Add-Content -Path $LogFile -Value $out -Encoding UTF8
                }
            }
        } else {
            Start-Sleep -Milliseconds 20
        }
    }
}

function Send-Cmd([string]$Cmd) {
    $stamp = Get-Date -Format "HH:mm:ss.fff"
    $line = "[$stamp] >>> $Cmd"
    Write-Host $line -ForegroundColor Yellow
    Add-Content -Path $LogFile -Value $line -Encoding UTF8
    $port.WriteLine($Cmd)
}

try {
    $port.Open()
    Write-Host "=== Boot wait ===" -ForegroundColor Magenta
    Pump 5000

    Write-Host ""
    Write-Host "=== STATUS (sanity) ===" -ForegroundColor Magenta
    Send-Cmd "STATUS"
    Pump 700

    Write-Host ""
    Write-Host "=== REC_START : SPEAK NOW for $RecordSeconds seconds ===" -ForegroundColor Cyan
    Send-Cmd "REC_START"
    $recMs = ($RecordSeconds * 1000) + 500
    Pump $recMs

    Write-Host ""
    Write-Host "=== REC_STOP ===" -ForegroundColor Cyan
    Send-Cmd "REC_STOP"
    Pump 1500

    Write-Host ""
    Write-Host "=== PLAY : speaker should now play back what you just said ===" -ForegroundColor Cyan
    Send-Cmd "PLAY"
    $playMs = ($RecordSeconds * 1000) + 2000
    Pump $playMs

    Write-Host ""
    Write-Host "=== STATUS (final) ===" -ForegroundColor Magenta
    Send-Cmd "STATUS"
    Pump 700

    Write-Host ""
    Write-Host "----------------------------------------------------------------" -ForegroundColor Green
    Write-Host " Done. The speaker should have just played back your voice."     -ForegroundColor Green
    Write-Host " Full log: $LogFile"                                              -ForegroundColor Green
    Write-Host "----------------------------------------------------------------" -ForegroundColor Green
}
finally {
    if ($port -and $port.IsOpen) { $port.Close() }
}
