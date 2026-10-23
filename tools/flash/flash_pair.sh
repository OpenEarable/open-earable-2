#!/bin/bash
# Provision a left/right nRF5340 pair with two J-Links (Bash 3.2+, macOS/Linux).
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BUILD_DIR="$SCRIPT_DIR/../../build_fota"
LEFT_SNR=
RIGHT_SNR=
HW_VERSION=2.0.1
DRY_RUN=false
LOCKS=()
WORKERS=()
RUN_DIR=

usage() {
    cat <<'EOF'
Usage: flash_pair.sh --left-snr SERIAL --right-snr SERIAL [options]
  --left-snr SERIAL  J-Link attached to the left earable (required)
  --right-snr SERIAL J-Link attached to the right earable (required)
  --hw x.y.z        Hardware revision for both devices (default: 2.0.1)
  --build-dir DIR   Merged FOTA images (default: repository's build_fota)
  --dry-run         Validate inputs and show the plan without accessing hardware
  -h, --help        Show this help

Requires nrfjprog on PATH, or NRFJPROG=/path/to/nrfjprog.
Both devices are fully erased, including phone bonds. Their pair identity is
derived from their chip IDs, and both are configured for paired operation.
EOF
}

die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --left-snr|--right-snr|--hw|--build-dir)
            [[ $# -ge 2 && -n "$2" ]] || die "Missing value for $1"
            case "$1" in
                --left-snr) LEFT_SNR=$2 ;;
                --right-snr) RIGHT_SNR=$2 ;;
                --hw) HW_VERSION=$2 ;;
                --build-dir) BUILD_DIR=$2 ;;
            esac
            shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown option: $1 (use --help)" ;;
    esac
done

# Normalize decimal serials without arithmetic overflow or octal interpretation.
normalize_serial() {
    local value=$1
    [[ "$value" =~ ^[0-9]+$ ]] || die "J-Link serial must be a positive number"
    while [[ "$value" == 0* ]]; do value=${value#0}; done
    [[ -n "$value" ]] || die "J-Link serial must be a positive number"
    printf '%s\n' "$value"
}
LEFT_SNR=$(normalize_serial "$LEFT_SNR")
RIGHT_SNR=$(normalize_serial "$RIGHT_SNR")
[[ "$LEFT_SNR" != "$RIGHT_SNR" ]] || die "Left and right must use different J-Links"
[[ "$HW_VERSION" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]] || die "Hardware version must be x.y.z (components 0..255)"
IFS=. read -r HW_MAJOR HW_MINOR HW_PATCH <<< "$HW_VERSION"
HW_MAJOR=$((10#$HW_MAJOR)); HW_MINOR=$((10#$HW_MINOR)); HW_PATCH=$((10#$HW_PATCH))
(( HW_MAJOR <= 255 && HW_MINOR <= 255 && HW_PATCH <= 255 )) || die "Hardware version components must be 0..255"
printf -v HW_WORD '%02X%02X%02X00' "$HW_MAJOR" "$HW_MINOR" "$HW_PATCH"

[[ -d "$BUILD_DIR" ]] || die "Build directory does not exist: $BUILD_DIR"
BUILD_DIR=$(cd -- "$BUILD_DIR" && pwd -P)
NET_HEX="$BUILD_DIR/merged_openearable_v2_nrf5340_cpunet.hex"
APP_HEX="$BUILD_DIR/merged_openearable_v2_nrf5340_cpuapp.hex"
if [[ ! -f "$NET_HEX" || ! -f "$APP_HEX" ]]; then
    NET_HEX="$BUILD_DIR/merged_CPUNET.hex"
    APP_HEX="$BUILD_DIR/merged.hex"
fi
[[ -s "$NET_HEX" && -s "$APP_HEX" ]] || die "Missing merged application/network images in $BUILD_DIR"
printf 'Left: %s; right: %s; hardware: %s\nImages: %s\n' "$LEFT_SNR" "$RIGHT_SNR" "$HW_VERSION" "$BUILD_DIR"
printf '%s\n' 'Both devices will be erased, including phone bonds, and provisioned as a new pair.'
if "$DRY_RUN"; then
    printf '%s\n' 'Plan: read chip IDs -> back up UICR -> flash both cores in parallel -> write/read back pair settings -> reset both. No hardware accessed.'
    exit 0
fi
NRFJPROG=$(command -v "${NRFJPROG:-nrfjprog}") || die "nrfjprog not found; add it to PATH or set NRFJPROG"

# Keep locks until both workers have stopped, including on interruption. A
# SIGKILL can leave a lock behind; never remove another process's lock here.
cleanup() {
    local status=$? pid lock
    trap - EXIT
    trap '' INT TERM
    # The guarded expansions also support empty arrays with Bash 3.2 nounset.
    for pid in ${WORKERS[@]+"${WORKERS[@]}"}; do wait "$pid" || :; done
    for lock in ${LOCKS[@]+"${LOCKS[@]}"}; do
        rm -f -- "$lock/pid"
        rmdir -- "$lock" || :
    done
    if [[ $status -ne 0 && -n "$RUN_DIR" ]]; then
        printf 'Pair operation incomplete. Inspect logs before retrying: %s\n' "$RUN_DIR" >&2
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Per-probe locks also prevent two versions/build directories racing each other.
for snr in "$LEFT_SNR" "$RIGHT_SNR"; do
    lock="${TMPDIR:-/tmp}/openearable-jlink-$UID-$snr.lock"
    mkdir -m 700 -- "$lock" 2>/dev/null || die "Probe is locked: $lock (check its pid file before removing a stale lock)"
    LOCKS+=("$lock")
    printf '%s\n' "$$" > "$lock/pid"
done
AVAILABLE=$("$NRFJPROG" --ids) || die "Could not enumerate connected J-Links"
for snr in "$LEFT_SNR" "$RIGHT_SNR"; do
    printf '%s\n' "$AVAILABLE" | awk -v snr="$snr" '$1 == snr && NF == 1 { found=1 } END { exit !found }' || die "J-Link $snr is not connected"
done
mkdir -p -- "$BUILD_DIR/pair-logs"
RUN_DIR=$(mktemp -d "$BUILD_DIR/pair-logs/$(date +%Y%m%d-%H%M%S).XXXXXX")
printf 'Logs and original UICR backups: %s\n' "$RUN_DIR"

# Run one probe command, retaining its output for memory parsing and diagnosis.
# Only the known Nordic worker-process crash is retried, once with identical args.
run() {
    local side=$1 snr=$2 attempt status
    shift 2
    for attempt in 1 2; do
        {
            printf '\n%s $ ' "$(date +%Y-%m-%dT%H:%M:%S%z)"
            printf '%q ' "$NRFJPROG" "$@" -f NRF53 --snr "$snr" --clockspeed 8000
            printf '\n'
        } >> "$RUN_DIR/$side.log"
        status=0
        "$NRFJPROG" "$@" -f NRF53 --snr "$snr" --clockspeed 8000 > "$RUN_DIR/$side.last-output" 2>&1 || status=$?
        cat "$RUN_DIR/$side.last-output" >> "$RUN_DIR/$side.log"
        if [[ $status -eq 0 ]]; then return 0; fi
        if [[ $attempt -eq 1 && $status -eq 63 ]] && grep -q 'Worker process is dead' "$RUN_DIR/$side.last-output"; then
            printf '%s: Nordic worker exited; retrying this command once.\n' "$side"
            sleep 1
        else
            die "$side: command failed ($status); see $RUN_DIR/$side.log"
        fi
    done
}

# Extract exactly count hexadecimal words from the requested nrfjprog row.
read_words() {
    local side=$1 address=$2 count=$3
    awk -v address="$address:" -v count="$count" '
        toupper($1) == toupper(address) {
            for (i=2; i<=count+1; i++) {
                if (length($i) != 8 || $i !~ /^[0-9a-fA-F]+$/) exit 1
                printf "%s%s", toupper($i), (i==count+1 ? "\n" : " ")
            }
            found=1; exit
        }
        END { if (!found) exit 1 }
    ' "$RUN_DIR/$side.last-output"
}

# Start one worker per probe and wait for BOTH before advancing the phase.
# Do not call this function in a conditional: workers rely on Bash errexit.
parallel() {
    local step=$1 left_pid right_pid left_status=0 right_status=0
    (trap - EXIT INT TERM; "$step" left "$LEFT_SNR" 0) &
    left_pid=$!
    WORKERS=("$left_pid")
    (trap - EXIT INT TERM; "$step" right "$RIGHT_SNR" 1) &
    right_pid=$!
    WORKERS+=("$right_pid")
    wait "$left_pid" || left_status=$?
    wait "$right_pid" || right_status=$?
    WORKERS=()
    [[ $left_status -eq 0 && $right_status -eq 0 ]] || die "$step failed (left=$left_status, right=$right_status)"
}

identify() {
    local side=$1 snr=$2
    run "$side" "$snr" --memrd 0x00FF0204 --n 8 --coprocessor CP_APPLICATION
    read_words "$side" 0x00FF0204 2 > "$RUN_DIR/$side.chip-id"
    run "$side" "$snr" --readuicr "$RUN_DIR/$side-original-uicr.hex" --coprocessor CP_APPLICATION
    [[ -s "$RUN_DIR/$side-original-uicr.hex" ]] || die "$side: UICR backup is missing or empty"
}
parallel identify
read -r LEFT_LOW LEFT_HIGH < "$RUN_DIR/left.chip-id"
read -r RIGHT_LOW RIGHT_HIGH < "$RUN_DIR/right.chip-id"
[[ "$LEFT_LOW $LEFT_HIGH" != "$RIGHT_LOW $RIGHT_HIGH" ]] || die "Both probes report the same chip identity"
PAIR_ID=$((16#$LEFT_LOW ^ 16#$RIGHT_LOW))
(( PAIR_ID != 0 && PAIR_ID != 0xFFFFFFFF )) || die "Derived pair identity is reserved"
printf -v PAIR_WORD '%08X' "$PAIR_ID"
printf 'LEFT_SNR=%s\nRIGHT_SNR=%s\nPAIR_ID=%s\nHARDWARE_WORD=%s\n' "$LEFT_SNR" "$RIGHT_SNR" "$PAIR_WORD" "$HW_WORD" > "$RUN_DIR/pair.conf"
printf 'Pair identity: %s. Programming both devices...\n' "$PAIR_WORD"

program() {
    local side=$1 snr=$2 channel=$3 actual expected
    run "$side" "$snr" --program "$NET_HEX" --chiperase --verify --coprocessor CP_NETWORK
    run "$side" "$snr" --program "$APP_HEX" --chiperase --verify --coprocessor CP_APPLICATION
    run "$side" "$snr" --memwr 0x00FF80F4 --val "$channel" --coprocessor CP_APPLICATION
    run "$side" "$snr" --memwr 0x00FF80F8 --val "0x$PAIR_WORD" --coprocessor CP_APPLICATION
    run "$side" "$snr" --memwr 0x00FF8100 --val "0x$HW_WORD" --coprocessor CP_APPLICATION
    run "$side" "$snr" --memrd 0x00FF80F4 --n 16 --coprocessor CP_APPLICATION
    actual=$(read_words "$side" 0x00FF80F4 4)
    printf -v expected '%08X %s FFFFFFFF %s' "$channel" "$PAIR_WORD" "$HW_WORD"
    [[ "$actual" == "$expected" ]] || die "$side: configuration mismatch ($actual != $expected)"
    printf '%s: both images and pair configuration verified.\n' "$side"
}
parallel program
printf 'IMAGES_VERIFIED=true\nPAIR_CONFIGURATION_VERIFIED=true\n' > "$RUN_DIR/verification.conf"

pin_reset() { run "$1" "$2" --pinreset; }
app_reset() { run "$1" "$2" --reset --coprocessor CP_APPLICATION; }
parallel pin_reset
# Let the bootloader finish before setting the application's SREQ reset reason.
sleep 5
parallel app_reset
printf 'RESET_COMMANDS_SUCCEEDED=true\nBLUETOOTH_RUNTIME_VERIFIED=false\n' >> "$RUN_DIR/verification.conf"
printf '%s\n' 'Both devices flashed and provisioned as left/right; resets succeeded. Verify Bluetooth startup on your phone.'
