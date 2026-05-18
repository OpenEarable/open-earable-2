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

CLOCK_SPEED=4000

# Recover both network and application processors
nrfutil device recover --serial-number $SNR --family nrf53 --core network --swd-clock-frequency $CLOCK_SPEED
nrfutil device recover --serial-number $SNR --family nrf53 --core application --swd-clock-frequency $CLOCK_SPEED
