#include "sensor_sink.h"
#include "SDLogger.h"
#include "sensor_service.h"

extern "C" int sensor_sink_put(const struct sensor_msg *msg)
{
    int ret = 0;

    if (msg->sd) {
        int r = sdlogger.write_sensor_data(msg->data);
        if (r < 0) ret = r;
    }

    if (msg->stream) {
        int r = sensor_gatt_queue_put(&msg->data);
        if (r < 0 && ret == 0) ret = r;
    }

    return ret;
}

extern "C" int sensor_sink_write_sd(const void *const *data_blocks,
                                    const size_t *lengths, size_t block_count)
{
    return sdlogger.write_sensor_data(data_blocks, lengths, block_count);
}
