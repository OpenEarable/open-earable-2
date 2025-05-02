#!/bin/bash

# Parse serial number argument
if [ -z "$1" ]; then
    echo "Error: Serial number (SNR) is required"
    echo "Usage: $0 <serial_number>"
    exit 1
fi

# Validate serial number is numeric
if ! [[ "$1" =~ ^[0-9]+$ ]]; then
    echo "Error: Serial number must be numeric"
    exit 1
fi

SNR=$1

# Recover both network and application processors
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --snr $SNR --clockspeed 8000
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --snr $SNR --clockspeed 8000
