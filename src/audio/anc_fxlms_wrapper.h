/*
 * ANC FxLMS Wrapper Header - C declarations for C++ DSP functions
 */

#ifndef ANC_FXLMS_WRAPPER_H
#define ANC_FXLMS_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C wrapper for DSP safe load mixer gain operation
 * 
 * @param param     Q5.27 fixed-point gain parameter
 * @param update_inactive Whether to update inactive bank
 * @return int      0 on success, negative error code on failure
 */
int fdsp_safe_load_mixer_gain(uint32_t param);

#ifdef __cplusplus
}
#endif

#endif // ANC_FXLMS_WRAPPER_H
