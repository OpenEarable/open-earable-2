[CmdletBinding()]
param(
  [Parameter(Mandatory = $true)]
  [ValidatePattern('^\d+$')]
  [string]$Snr,                # Device serial number

  [switch]$Left,               # Set left configuration
  [switch]$Right,              # Set right configuration
  [switch]$Standalone,         # Only valid together with -Left or -Right

  [string]$Hw,                 # Hardware version x.y.z (e.g. 2.0.0)

  [string]$Chip = 'NRF53',     # nrfjprog --family
  [ValidateRange(1, 50000)]
  [int]$Clockspeed = 8000,     # nrfjprog --clockspeed (kHz)
  [string]$BuildDir = 'build_fota',
  [string]$Python = 'python'   # Python from the SDK environment
)

$ErrorActionPreference = 'Stop'

# --- Simple argument validation ---
if ($Left -and $Right) {
  Write-Error "Choose either -Left or -Right, not both."
  exit 1
}
if ($Standalone -and -not ($Left -or $Right)) {
  Write-Error "-Standalone can only be used with -Left or -Right."
  exit 1
}

# --Hw can only be used with -Left or -Right
if ($Hw -and -not ($Left -or $Right)) {
  Write-Error "-Hw can only be used together with -Left or -Right."
  exit 1
}

# If Left/Right are set but no Hw is given, use default 2.0.0
if (($Left -or $Right) -and -not $Hw) {
  $Hw = "2.0.0"
  Write-Host "No hardware version specified, using default: $Hw"
}

# --- Parse and validate hardware version if provided ---
$hwValue = $null
if ($Hw) {
  # Expect format x.y.z
  $parts = $Hw.Split('.')
  if ($parts.Count -ne 3) {
    Write-Error "Hardware version must be in format x.y.z (e.g., 2.0.0)."
    exit 1
  }

  [int]$hwMajor = 0
  [int]$hwMinor = 0
  [int]$hwPatch = 0

  if (-not ([int]::TryParse($parts[0], [ref]$hwMajor)) -or
      -not ([int]::TryParse($parts[1], [ref]$hwMinor)) -or
      -not ([int]::TryParse($parts[2], [ref]$hwPatch))) {
    Write-Error "Hardware version components must be numeric."
    exit 1
  }

  foreach ($c in @($hwMajor, $hwMinor, $hwPatch)) {
    if ($c -lt 0 -or $c -gt 255) {
      Write-Error "Each hardware version component must be between 0 and 255."
      exit 1
    }
  }

  # Match Bash behavior: "0x%02X%02X%02X00"
  $hwValue = ("0x{0:X2}{1:X2}{2:X2}00" -f $hwMajor, $hwMinor, $hwPatch)
}

# Resolve and merge the signed sysbuild images before accessing the device.
$flashDir = Join-Path ([System.IO.Path]::GetTempPath()) ("openearable-flash-$Snr-" + [guid]::NewGuid())
New-Item -ItemType Directory -Path $flashDir | Out-Null
$flashSucceeded = $false
try {
& $Python (Join-Path $PSScriptRoot 'prepare_images.py') --build-dir $BuildDir --output-dir $flashDir --fota
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$netHex = Join-Path $flashDir 'merged_CPUNET.hex'
$appHex = Join-Path $flashDir 'merged.hex'
$uicrBackup = Join-Path $flashDir 'uicr_backup.hex'

Write-Host "nrfjprog starting..."
Write-Host "  SNR: $Snr"
Write-Host "  CHIP: $Chip"
Write-Host "  CLOCKSPEED: $Clockspeed"
Write-Host "  Left: $Left  Right: $Right  Standalone: $Standalone"
if ($Hw) { Write-Host "  HW: $Hw (value $hwValue)" } else { Write-Host "  HW: (not set)" }
Write-Host "  APP HEX: $appHex"
Write-Host "  NET HEX: $netHex"
Write-Host ""

# --- Backup UICR if neither side flag is set (matches Bash behavior) ---
if (-not $Left -and -not $Right) {
  # Ensure target folder exists
  $uicrDir = Split-Path -Parent $uicrBackup
  if (-not (Test-Path $uicrDir)) { New-Item -ItemType Directory -Path $uicrDir | Out-Null }
  if (Test-Path $uicrBackup) { Remove-Item $uicrBackup -Force -ErrorAction SilentlyContinue }

  Write-Host "Backing up UICR -> $uicrBackup"
  & nrfjprog --coprocessor CP_APPLICATION --readuicr "$uicrBackup" --family $Chip --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# --- Flash each complete core image with one erase per core ---
Write-Host "Flashing CPUNET..."
& nrfjprog --program "$netHex" --chiperase --verify --family $Chip --coprocessor CP_NETWORK --snr $Snr --clockspeed $Clockspeed
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# --- Flash CPUAPP ---
Write-Host "Flashing CPUAPP..."
& nrfjprog --program "$appHex" --chiperase --verify --family $Chip --coprocessor CP_APPLICATION --snr $Snr --clockspeed $Clockspeed
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# --- Restore UICR if we backed it up (i.e., no left/right flags) ---
if (-not $Left -and -not $Right) {
  if (Test-Path $uicrBackup) {
    Write-Host "Restoring UICR from $uicrBackup"
    & nrfjprog --coprocessor CP_APPLICATION --program "$uicrBackup" --family $Chip --snr $Snr --clockspeed $Clockspeed --verify
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  } else {
    throw "UICR backup not found; cannot restore device identity."
  }
}

# --- Left/Right configuration ---
if ($Left) {
  Write-Host "Setting LEFT config (0x00FF80F4 = 0)"
  & nrfjprog --memwr 0x00FF80F4 --val 0 --family $Chip --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
elseif ($Right) {
  Write-Host "Setting RIGHT config (0x00FF80F4 = 1)"
  & nrfjprog --memwr 0x00FF80F4 --val 1 --family $Chip --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# --- Standalone mode ---
if ($Standalone) {
  Write-Host "Enabling standalone mode (0x00FF80FC = 0)"
  & nrfjprog --memwr 0x00FF80FC --val 0 --family $Chip --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# --- Hardware version (if provided / defaulted) ---
if ($Hw) {
  Write-Host "Setting hardware version $Hw (0x00FF8100 = $hwValue)"
  & nrfjprog --memwr 0x00FF8100 --val $hwValue --family $Chip --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# --- Reset ---
# Reset the complete SoC first, then issue an application soft reset. The
# application uses the resulting SREQ reset reason as a power-on request.
Write-Host "Resetting device..."
& nrfjprog --pinreset --family $Chip --snr $Snr --clockspeed $Clockspeed
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# Allow the bootloader and application to finish handling the pin reset before
# setting the SREQ reset reason used by PowerManager.
Start-Sleep -Seconds 5

& nrfjprog --reset --family $Chip --coprocessor CP_APPLICATION --snr $Snr --clockspeed $Clockspeed
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`nDone. Device reset; application is starting." -ForegroundColor Green

$flashSucceeded = $true
} finally {
  if ($flashSucceeded) {
    Remove-Item -Recurse -Force $flashDir
  } else {
    Write-Warning "Flash failed; images and any UICR backup retained at: $flashDir"
  }
}
