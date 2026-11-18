#ifndef OE_SENSOR_TRANSPORT_H
#define OE_SENSOR_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One 251-byte link-layer payload, less L2CAP and ATT headers. */
#define OE_SENSOR_PACKET_MAX 244

struct oe_sensor_batch {
    uint8_t data[OE_SENSOR_PACKET_MAX];
    uint16_t len;
    uint16_t count;
    uint16_t period;
    uint64_t last_time;
};

unsigned oe_sensor_sample_size(uint8_t id);
unsigned oe_sensor_sample_count(uint8_t id, unsigned size);
/* False means flush the previous batch and try this sample again. */
bool oe_sensor_batch_append(struct oe_sensor_batch *batch, uint8_t id,
                            const uint8_t *sample, uint64_t time, unsigned limit);

#ifdef __cplusplus
}
#endif
#endif
