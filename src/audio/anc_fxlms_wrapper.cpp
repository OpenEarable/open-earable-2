/*
 * ANC FxLMS Wrapper - C++ to C bridge for DSP safe load operations
 */

#include "ADAU1860.h"

extern "C" {

// C-compatible wrapper function for fdsp_safe_load with MIXER address
int fdsp_safe_load_mixer_gain(uint32_t param) {
    extern ADAU1860 dac;  // Global DAC instance
    
    // Call ADAU1860 fdsp_safe_load with MIXER address and channel 2 (microphone)
    return dac.fdsp_safe_load(MIXER, 2, param, false);
}

} // extern "C"
