#!/bin/bash

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --snr)
            SNR="$2"
            shift 2
            ;;
        *)
            echo "Error: Unknown parameter $1"
            echo "Usage: $0 --snr <serial_number>"
            exit 1
            ;;
    esac
done

# Check if SNR is provided
if [ -z "$SNR" ]; then
    echo "Error: Serial number (SNR) is required"
    echo "Usage: $0 --snr <serial_number>"
    exit 1
fi

# Validate serial number is numeric
if ! [[ "$SNR" =~ ^[0-9]+$ ]]; then
    echo "Error: Serial number must be numeric"
    exit 1
fi

# Recover both network and application processors
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --snr $SNR --clockspeed 8000
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --snr $SNR --clockspeed 8000
