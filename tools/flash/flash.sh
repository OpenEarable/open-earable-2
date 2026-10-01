#!/bin/bash

# Stop immediately if preparation, flashing, or reset fails.
set -e

# Default parameters
CLOCKSPEED=8000
CHIP=NRF53
BUILD_DIR=build
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Function to show usage
show_usage() {
    echo "Usage: $0 --snr <serial_number> [--left|--right] [--standalone] [--hw x.y.z] [--build-dir path] [--clockspeed kHz]"
    echo "  --snr: Device serial number (required)"
    echo "  --left: Flash left earable configuration"
    echo "  --right: Flash right earable configuration"
    echo "  --standalone: Configure device for standalone mode"
    echo "  --hw: Set hardware version (format: x.y.z, e.g., 2.0.0)"
    echo "  --build-dir: Sysbuild output directory (default: $BUILD_DIR)"
    echo "  --clockspeed: J-Link clock in kHz (default: 8000)"
    exit 1
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --snr) SNR="$2"; shift ;;
        --left) LEFT=true ;;
        --right) RIGHT=true ;;
        --standalone) STANDALONE=true ;;
        --hw) HW_VERSION="$2"; shift ;;
        --build-dir) BUILD_DIR="$2"; shift ;;
        --clockspeed) CLOCKSPEED="$2"; shift ;;
        *) show_usage ;;
    esac
    shift
done

# Check if SNR is provided
if [ -z "$SNR" ]; then
    echo "Error: Serial number (--snr) is required"
    show_usage
fi

# Validate serial number is numeric
if ! [[ "$SNR" =~ ^[0-9]+$ ]]; then
    echo "Error: Serial number must be numeric"
    exit 1
fi

if [ "$LEFT" == true ] && [ "$RIGHT" == true ]; then
    echo "Error: Choose either --left or --right, not both"
    exit 1
fi
if ! [[ "$CLOCKSPEED" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: Clock speed must be a positive integer"
    exit 1
fi

# Check if --hw is used without --left or --right
if [ -n "$HW_VERSION" ] && [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    echo "Error: --hw can only be used with --left or --right"
    show_usage
fi

# Set default hardware version if --left or --right is specified without --hw
if [ -n "$LEFT" ] || [ -n "$RIGHT" ]; then
    if [ -z "$HW_VERSION" ]; then
        HW_VERSION="2.0.0"
        echo "No hardware version specified, using default: $HW_VERSION"
    fi
fi

# Validate hardware version format if provided
if [ -n "$HW_VERSION" ]; then
    if ! [[ "$HW_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "Error: Hardware version must be in format x.y.z (e.g., 2.0.0)"
        exit 1
    fi
    
    # Extract version components
    IFS='.' read -r HW_MAJOR HW_MINOR HW_PATCH <<< "$HW_VERSION"
    
    # Validate each component is within uint8_t range (0-255)
    if [ "$HW_MAJOR" -gt 255 ] || [ "$HW_MINOR" -gt 255 ] || [ "$HW_PATCH" -gt 255 ]; then
        echo "Error: Each version component must be between 0 and 255"
        exit 1
    fi
    
    # Calculate uint32_t value: (major << 16) | (minor << 8) | patch
    HW_VALUE=$(printf "0x%02X%02X%02X00" $HW_MAJOR $HW_MINOR $HW_PATCH)
fi

if [ "$STANDALONE" == true ] && [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    echo "Error: --standalone can only be used with --left or --right"
    show_usage
fi

# Validate and merge every required image before accessing the device.
FLASH_DIR=$(mktemp -d "${TMPDIR:-/tmp}/openearable-flash-${SNR}.XXXXXX")
cleanup() {
    status=$?
    if [ "$status" -eq 0 ]; then
        rm -rf -- "$FLASH_DIR"
    else
        echo "Flash failed; images and any UICR backup retained at: $FLASH_DIR" >&2
    fi
}
trap cleanup EXIT
"${PYTHON:-python3}" "$SCRIPT_DIR/prepare_images.py" --build-dir "$BUILD_DIR" --output-dir "$FLASH_DIR"
UICR_BACKUP="$FLASH_DIR/uicr_backup.hex"

if [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    nrfjprog --coprocessor CP_APPLICATION --readuicr "$UICR_BACKUP" -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
fi

nrfjprog --program "$FLASH_DIR/merged_CPUNET.hex" --chiperase --verify -f $CHIP --coprocessor CP_NETWORK --snr $SNR --clockspeed $CLOCKSPEED

nrfjprog --program "$FLASH_DIR/merged.hex" --chiperase --verify -f $CHIP --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED

if [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    nrfjprog --coprocessor CP_APPLICATION --program "$UICR_BACKUP" -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED --verify
fi

if [ "$LEFT" == true ]; then
    nrfjprog --memwr 0x00FF80F4 --val 0 -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
elif [ "$RIGHT" == true ]; then
    nrfjprog --memwr 0x00FF80F4 --val 1 -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
fi

# Set standalone mode configuration if requested
if [ "$STANDALONE" == true ]; then
    nrfjprog --memwr 0x00FF80FC --val 0 -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
    echo "Device configured for standalone mode"
fi

# Set hardware version if provided
if [ -n "$HW_VERSION" ]; then
    nrfjprog --memwr 0x00FF8100 --val $HW_VALUE -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
    echo "Hardware version set to $HW_VERSION"
fi

# Start both cores cleanly, then request application power-on through SREQ.
nrfjprog --pinreset -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
sleep 5
nrfjprog --reset -f $CHIP --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED
echo "Device reset; application is starting."
