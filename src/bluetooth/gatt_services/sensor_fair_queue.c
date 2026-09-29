#include "sensor_fair_queue.h"

#include <errno.h>
#include <string.h>

static struct sensor_data *entry(struct sensor_fair_queue *q, uint32_t i)
{
	return (struct sensor_data *)((uint8_t *)q->buffer + i * q->item_size);
}

static uint32_t samples(struct sensor_fair_queue *q, const struct sensor_data *data)
{
	uint8_t width = q->widths[data->id];
	if (width == 0 || data->size <= width) {
		return 1;
	}
	return MAX(1, (data->size - sizeof(uint16_t)) / width);
}

static void decay(struct sensor_fair_queue *q)
{
	uint32_t now = k_uptime_get_32();
	uint32_t periods = MIN((now - q->last_decay) / 1000U, 16U);
	if (periods == 0) {
		return;
	}
	for (unsigned i = 0; i < SENSOR_FAIR_QUEUE_IDS; i++) {
		q->shares[i].offered >>= periods;
		q->shares[i].served >>= periods;
	}
	q->last_decay = now;
}

/* Compare delivered fractions without division; queued samples count for admission. */
static bool less(struct sensor_fair_queue *q, uint8_t a, uint8_t b, bool admission)
{
	const struct sensor_fair_share *x = &q->shares[a], *y = &q->shares[b];
	uint64_t nx = x->served + (admission ? x->pending : 0);
	uint64_t ny = y->served + (admission ? y->pending : 0);
	return nx * MAX(y->offered, 1U) < ny * MAX(x->offered, 1U);
}

static void remove_entry(struct sensor_fair_queue *q, uint32_t i)
{
	struct sensor_data *data = entry(q, i);
	q->shares[data->id].pending -= samples(q, data);
	q->used_msgs--;
	memmove(data, entry(q, i + 1), (q->used_msgs - i) * q->item_size);
}

void sensor_fair_queue_init(struct sensor_fair_queue *q, void *buffer,
			   size_t item_size, uint32_t capacity, const uint8_t *widths)
{
	q->buffer = buffer;
	q->item_size = item_size;
	q->capacity = capacity;
	memcpy(q->widths, widths, sizeof(q->widths));
	k_sem_init(&q->ready, 0, 1);
	sensor_fair_queue_purge(q);
}

void sensor_fair_queue_purge(struct sensor_fair_queue *q)
{
	k_spinlock_key_t key = k_spin_lock(&q->lock);
	q->used_msgs = 0;
	memset(q->shares, 0, sizeof(q->shares));
	q->last_decay = k_uptime_get_32();
	k_spin_unlock(&q->lock, key);
	/* A stale ready token is harmless; resetting it could lose a concurrent put. */
}

int sensor_fair_queue_put(struct sensor_fair_queue *q, const void *item)
{
	const struct sensor_data *data = item;
	if (data->id >= SENSOR_FAIR_QUEUE_IDS || data->size > sizeof(data->data)) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&q->lock);
	decay(q);
	uint32_t n = samples(q, data);
	q->shares[data->id].offered += n;
	q->shares[data->id].pending += n;
	memcpy(entry(q, q->used_msgs++), item, q->item_size);
	bool full = q->used_msgs > q->capacity;
	if (full) {
		uint32_t discard = 0;
		for (uint32_t i = 1; i < q->used_msgs; i++) {
			if (less(q, entry(q, discard)->id, entry(q, i)->id, true)) {
				discard = i;
			}
		}
		/* First match is the oldest publication from the most represented sensor. */
		remove_entry(q, discard);
	}
	k_spin_unlock(&q->lock, key);
	k_sem_give(&q->ready);
	return full ? -ENOSPC : 0;
}

int sensor_fair_queue_get(struct sensor_fair_queue *q, void *item, k_timeout_t timeout)
{
	k_timepoint_t deadline = sys_timepoint_calc(timeout);
	while (true) {
		k_spinlock_key_t key = k_spin_lock(&q->lock);
		if (q->used_msgs) {
			decay(q);
			uint32_t next = 0;
			for (uint32_t i = 1; i < q->used_msgs; i++) {
				if (less(q, entry(q, i)->id, entry(q, next)->id, false)) {
					next = i;
				}
			}
			struct sensor_data *data = entry(q, next);
			q->shares[data->id].served += samples(q, data);
			memcpy(item, data, q->item_size);
			remove_entry(q, next);
			k_spin_unlock(&q->lock, key);
			return 0;
		}
		k_spin_unlock(&q->lock, key);
		if (k_sem_take(&q->ready, sys_timepoint_timeout(deadline))) {
			return -ENOMSG;
		}
	}
}
