#ifndef SENSOR_FAIR_QUEUE_H
#define SENSOR_FAIR_QUEUE_H

#include <zephyr/kernel.h>
#include "openearable_common.h"

#define SENSOR_FAIR_QUEUE_IDS (ID_BONE_CONDUCTION + 1)

struct sensor_fair_share {
	uint32_t offered;
	uint32_t served;
	uint32_t pending;
};

/* Buffer has capacity + 1 entries, with sensor_data first in each entry. */
struct sensor_fair_queue {
	struct k_spinlock lock;
	struct k_sem ready;
	void *buffer;
	size_t item_size;
	uint32_t capacity;
	uint32_t used_msgs;
	uint32_t last_decay;
	uint8_t widths[SENSOR_FAIR_QUEUE_IDS];
	struct sensor_fair_share shares[SENSOR_FAIR_QUEUE_IDS];
};

void sensor_fair_queue_init(struct sensor_fair_queue *q, void *buffer,
			   size_t item_size, uint32_t capacity, const uint8_t *widths);
void sensor_fair_queue_purge(struct sensor_fair_queue *q);
int sensor_fair_queue_put(struct sensor_fair_queue *q, const void *item);
int sensor_fair_queue_get(struct sensor_fair_queue *q, void *item, k_timeout_t timeout);

#endif
