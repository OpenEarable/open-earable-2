powershell
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
  [int]$Clockspeed = 8000      # nrfjprog --clockspeed (kHz)
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

# --- Fixed paths relative to CURRENT WORKING DIRECTORY (repo root) ---
$netHex = Join-Path (Get-Location) 'build_fota\merged_CPUNET.hex'
$appHex = Join-Path (Get-Location) 'build_fota\merged.hex'
$uicrBackup = Join-Path (Get-Location) 'tools\flash\uicr_backup.hex'

# --- Require application hex; CPUNET is optional ---
if (-not (Test-Path $appHex)) {
  Write-Error "Missing file: $appHex  (run from the repo root where build_fota\merged.hex exists)"
  exit 1
}
$haveNet = Test-Path $netHex

Write-Host "nrfjprog starting..."
Write-Host "  SNR: $Snr"
Write-Host "  CHIP: $Chip"
Write-Host "  CLOCKSPEED: $Clockspeed"
Write-Host "  Left: $Left  Right: $Right  Standalone: $Standalone"
if ($Hw) { Write-Host "  HW: $Hw (value $hwValue)" } else { Write-Host "  HW: (not set)" }
Write-Host "  APP HEX: $appHex"
if ($haveNet) { Write-Host "  NET HEX: $netHex" } else { Write-Host "  NET HEX: (not found, will skip)" }
Write-Host ""

# --- Backup UICR if neither side flag is set (matches Bash behavior) ---
if (-not $Left -and -not $Right) {
  # Ensure target folder exists
  $uicrDir = Split-Path -Parent $uicrBackup
  if (-not (Test-Path $uicrDir)) { New-Item -ItemType Directory -Path $uicrDir | Out-Null }
  if (Test-Path $uicrBackup) { Remove-Item $uicrBackup -Force -ErrorAction SilentlyContinue }

  Write-Host "Backing up UICR -> $uicrBackup"
  & nrfjprog --readuicr "$uicrBackup" --family $Chip --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# --- Flash CPUNET if file exists ---
if ($haveNet) {
  Write-Host "Flashing CPUNET..."
  & nrfjprog --program "$netHex" --chiperase --verify --family $Chip --coprocessor CP_NETWORK --snr $Snr --clockspeed $Clockspeed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} else {
  Write-Host "Skipping CPUNET (merged_CPUNET.hex not found)."
}

# --- Flash CPUAPP ---
Write-Host "Flashing CPUAPP..."
& nrfjprog --program "$appHex" --chiperase --verify --family $Chip --coprocessor CP_APPLICATION --snr $Snr --clockspeed $Clockspeed
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# --- Restore UICR if we backed it up (i.e., no left/right flags) ---
if (-not $Left -and -not $Right) {
  if (Test-Path $uicrBackup) {
    Write-Host "Restoring UICR from $uicrBackup"
    & nrfjprog --program "$uicrBackup" --family $Chip --snr $Snr --clockspeed $Clockspeed --verify
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  } else {
    Write-Warning "UICR backup not found; skipping restore."
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
Write-Host "Resetting device..."
& nrfjprog --reset --family $Chip --snr $Snr --clockspeed $Clockspeed
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`nDone." -ForegroundColor Green
