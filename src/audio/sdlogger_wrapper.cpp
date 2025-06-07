#include "sdlogger_wrapper.h"
#include "openearable_common.h"
#include "SDLogger.h"

// Assuming sdlogger is an accessible global object
extern SDLogger sdlogger;

extern "C" {
    int sdlogger_write_data(const void* const* data_blocks, const size_t* lengths, size_t block_count) {
        return sdlogger.write_sensor_data(data_blocks, lengths, block_count);
    }
}
