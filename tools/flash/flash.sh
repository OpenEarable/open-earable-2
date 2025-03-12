nrfjprog --readuicr ./tools/flash/uicr_backup.hex -f NRF53 --snr 261010806 --clockspeed 8000
#nrfjprog --memrd 0x000FF000 --n 4096 --outfile ./tools/flash/settings_backup.hex -f NRF53 --snr 261010806 --clockspeed 8000
nrfjprog --program ./build/merged_CPUNET.hex --chiperase --verify -f NRF53 --coprocessor CP_NETWORK --snr 261010806 --clockspeed 8000
nrfjprog --program ./build/merged.hex --chiperase --verify -f NRF53 --coprocessor CP_APPLICATION --snr 261010806 --clockspeed 8000
nrfjprog --program ./tools/flash/uicr_backup.hex -f NRF53 --snr 261010806 --clockspeed 8000 --verify
#nrfjprog --program ./tools/flash/settings_backup.hex -f NRF53 --snr 261010806 --clockspeed 8000 --verify
nrfjprog --reset