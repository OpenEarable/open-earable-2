#pragma once

/**
 * Initialize SD card presence monitoring for the USB mass-storage LUN.
 *
 * The SD power rail must already be enabled. A missing card is a supported
 * state and does not make this function fail.
 */
int sd_mass_storage_init();
