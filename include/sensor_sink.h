#ifndef SENSOR_SINK_H
#define SENSOR_SINK_H

#include "openearable_common.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int sensor_sink_put(const struct sensor_msg *msg);

int sensor_sink_write_sd(const void *const *data_blocks, const size_t *lengths,
                         size_t block_count);

#ifdef __cplusplus
}
#endif

#endif
