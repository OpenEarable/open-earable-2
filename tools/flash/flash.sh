#!/bin/bash

# Default parameters
CLOCKSPEED=4000
FAMILY=nrf53

# Common nrfutil options
NRFUTIL_OPTS="--family $FAMILY"

# UICR address range for nRF5340
UICR_ADDR=0x00FF8000
UICR_SIZE=4096

# Function to show usage
show_usage() {
    echo "Usage: $0 [--snr <serial_number>] [--left|--right] [--standalone] [--hw x.y.z]"
    echo "  --snr: Device serial number"
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
if [ -n "$SNR" ]; then
    # Validate serial number is numeric
    if ! [[ "$SNR" =~ ^[0-9]+$ ]]; then
        echo "Error: Serial number must be numeric"
        exit 1
    fi
    NRFUTIL_OPTS="--serial-number $SNR $NRFUTIL_OPTS"
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

# Check if standalone was specified without left/right
if [ "$STANDALONE" == true ] && [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    echo "Error: --standalone can only be used with --left or --right"
    show_usage
fi


# First, explicitly recover both cores separately to ensure they are unlocked
nrfutil device recover --core network $NRFUTIL_OPTS
nrfutil device recover --core application $NRFUTIL_OPTS
sleep 1

# Flash network core (CPUNET)
nrfutil device program --firmware ./build/merged_CPUNET.hex --core network --options chip_erase_mode=ERASE_ALL,verify=VERIFY_READ $NRFUTIL_OPTS
nrfutil device reset --core network $NRFUTIL_OPTS
sleep 1

# Flash application core (CPUAPP)
nrfutil device program --firmware ./build/merged.hex --core application --options verify=VERIFY_READ $NRFUTIL_OPTS

# Set left/right configuration
if [ "$LEFT" == true ]; then
    nrfutil device write --address 0x00FF80F4 --value 0 $NRFUTIL_OPTS
elif [ "$RIGHT" == true ]; then
    nrfutil device write --address 0x00FF80F4 --value 1 $NRFUTIL_OPTS
fi

# Set standalone mode configuration if requested
if [ "$STANDALONE" == true ]; then
    nrfutil device write --address 0x00FF80FC --value 0 $NRFUTIL_OPTS
    echo "Device configured for standalone mode"
fi

# Set hardware version if provided
if [ -n "$HW_VERSION" ]; then
    nrfutil device write --address 0x00FF8100 --value $HW_VALUE $NRFUTIL_OPTS
    echo "Hardware version set to $HW_VERSION"
fi

# Reset device
nrfutil device reset --core application $NRFUTIL_OPTS
