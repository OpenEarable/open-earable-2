#ifndef MIC_BLE_STREAMER_H
#define MIC_BLE_STREAMER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

void mic_ble_streamer_set_sensor_queue(struct k_msgq *queue);
int mic_ble_streamer_enable(uint8_t sensor_id, bool enable);
bool mic_ble_streamer_is_active(void);
void mic_ble_streamer_process_i2s_block(const void *pcm_block, size_t size,
					uint64_t first_sample_time_us);

#ifdef __cplusplus
}
#endif

#endif
