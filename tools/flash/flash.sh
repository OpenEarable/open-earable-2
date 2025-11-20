#!/bin/bash

# Default parameters
CLOCKSPEED=8000
CHIP=NRF53

# Function to show usage
show_usage() {
    echo "Usage: $0 --snr <serial_number> [--left|--right] [--standalone] [--hw x.y.z]"
    echo "  --snr: Device serial number (required)"
    echo "  --left: Flash left earable configuration"
    echo "  --right: Flash right earable configuration"
    echo "  --standalone: Configure device for standalone mode"
    echo "  --hw: Set hardware version (format: x.y.z, e.g., 2.0.0)"
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

if [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    nrfjprog --readuicr ./tools/flash/uicr_backup.hex -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
fi

nrfjprog --program ./build/merged_CPUNET.hex --chiperase --verify -f $CHIP --coprocessor CP_NETWORK --snr $SNR --clockspeed $CLOCKSPEED

nrfjprog --program ./build/merged.hex --chiperase --verify -f $CHIP --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED

if [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    nrfjprog --program ./tools/flash/uicr_backup.hex -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED --verify
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

nrfjprog --reset -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
