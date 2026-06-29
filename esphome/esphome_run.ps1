param(
    [string]$Port = "COM6",
    [switch]$LogsOnly,
    [switch]$CompileOnly
)

$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    Write-Host "[1/3] Checking esphome installation..."
    $hasEsphome = Get-Command esphome -ErrorAction SilentlyContinue
    if (-not $hasEsphome) {
        Write-Host "esphome not found. Installing via pip..."
        python -m pip install --upgrade pip
        python -m pip install --upgrade "esphome>=2024.2.0"
    } else {
        Write-Host "    found: $($hasEsphome.Source)"
        esphome version
    }

    if ($LogsOnly) {
        Write-Host "[2/3] Logs only..."
        esphome logs re1_audio.yaml --device $Port
        return
    }

    if ($CompileOnly) {
        Write-Host "[2/3] Compile only..."
        esphome compile re1_audio.yaml
        Write-Host "[3/3] Compile finished. Firmware in .esphome/build/re1-audio/."
        return
    }

    Write-Host "[2/3] Compiling + flashing to $Port ..."
    esphome run re1_audio.yaml --device $Port --no-logs
    Write-Host ""
    Write-Host "[3/3] Flash done. Tailing logs (Ctrl+C to exit)..."
    esphome logs re1_audio.yaml --device $Port
}
finally {
    Pop-Location
}
