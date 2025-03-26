#!/bin/bash

# Standard-Parameter
SNR=261010806
CLOCKSPEED=8000
CHIP=NRF53

# Prüfen, ob --left oder --right als Argument übergeben wurde
if [[ "$1" == "--left" ]]; then
    LEFT=true
elif [[ "$1" == "--right" ]]; then
    RIGHT=true
fi

# Falls weder --left noch --right angegeben wurde, Backup UICR durchführen
if [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    nrfjprog --readuicr ./tools/flash/uicr_backup.hex -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
fi

# Flash CPUNET
nrfjprog --program ./build/merged_CPUNET.hex --chiperase --verify -f $CHIP --coprocessor CP_NETWORK --snr $SNR --clockspeed $CLOCKSPEED

# Flash CPUAPP
nrfjprog --program ./build/merged.hex --chiperase --verify -f $CHIP --coprocessor CP_APPLICATION --snr $SNR --clockspeed $CLOCKSPEED

# Falls weder --left noch --right angegeben wurde, UICR wiederherstellen
if [ -z "$LEFT" ] && [ -z "$RIGHT" ]; then
    nrfjprog --program ./tools/flash/uicr_backup.hex -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED --verify
fi

# Wenn --left oder --right angegeben wurde, den entsprechenden Wert setzen
if [ "$LEFT" == true ]; then
    nrfjprog --memwr 0x00FF80F4 --val 0 -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
elif [ "$RIGHT" == true ]; then
    nrfjprog --memwr 0x00FF80F4 --val 1 -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED
fi

# Reset
nrfjprog --reset -f $CHIP --snr $SNR --clockspeed $CLOCKSPEED