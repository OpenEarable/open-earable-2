#!/bin/bash

# Default parameters
CLOCKSPEED=8000
CHIP=NRF53

# Function to show usage
show_usage() {
    echo "Usage: $0 --snr <serial_number> [--left|--right] [--standalone]"
    echo "  --snr: Device serial number (required)"
    echo "  --left: Flash left earable configuration"
    echo "  --right: Flash right earable configuration"
    echo "  --standalone: Configure device for standalone mode"
    exit 1
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --snr) SNR="$2"; shift ;;
        --left) LEFT=true ;;
        --right) RIGHT=true ;;
        --standalone) STANDALONE=true ;;
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

# Rest of your script remains the same
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

nrfjprog --reset -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
