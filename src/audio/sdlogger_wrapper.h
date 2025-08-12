#ifndef SDLOGGER_WRAPPER_H
#define SDLOGGER_WRAPPER_H

#include "openearable_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wrapper function to call sdlogger.write_data from C code
 * 
 * @param data Pointer to sensor data header
 * @param size Size of the audio data
 * @return int Return code (0 for success)
 */
int sdlogger_write_data(const void* const* data_blocks, const size_t* lengths, size_t block_count);

#ifdef __cplusplus
}
#endif

#endif /* SDLOGGER_WRAPPER_H */
