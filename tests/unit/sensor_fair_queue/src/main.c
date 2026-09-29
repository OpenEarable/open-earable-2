#include <unity.h>
#include <errno.h>
#include <string.h>
#include "sensor_fair_queue.h"

static const uint8_t widths[SENSOR_FAIR_QUEUE_IDS] = {36, 8, 0, 0, 16, 0, 4, 6};
static struct sensor_fair_queue queue;
struct item { struct sensor_data data; uint32_t epoch; };
static struct item storage[65];

void setUp(void) {}

static void init(unsigned capacity)
{
	memset(&queue, 0xa5, sizeof(queue));
	sensor_fair_queue_init(&queue, storage, sizeof(storage[0]), capacity, widths);
}

static void put(unsigned id, unsigned n, uint64_t time)
{
	struct item x = {.data = {.id = id, .size = n == 1 ? widths[id] : n * widths[id] + 2,
		.time = time}, .epoch = 12345};
	int ret = sensor_fair_queue_put(&queue, &x);
	TEST_ASSERT_TRUE(ret == 0 || ret == -ENOSPC);
	TEST_ASSERT_LESS_OR_EQUAL_UINT32(queue.capacity, queue.used_msgs);
}

void test_overflow_keeps_latest_readings_and_preserves_item_metadata(void)
{
	init(4);
	for (unsigned i = 0; i < 8; i++) put(0, 1, i);
	for (unsigned i = 4; i < 8; i++) {
		struct item x;
		TEST_ASSERT_EQUAL_INT(0, sensor_fair_queue_get(&queue, &x, K_NO_WAIT));
		TEST_ASSERT_EQUAL_UINT64(i, x.data.time);
		TEST_ASSERT_EQUAL_UINT32(12345, x.epoch);
	}
	struct item x;
	TEST_ASSERT_EQUAL_INT(-ENOMSG, sensor_fair_queue_get(&queue, &x, K_NO_WAIT));
}

void test_purge_discards_backlog_and_resets_sensor_shares(void)
{
	init(8);
	put(0, 1, 1); put(4, 2, 2);
	sensor_fair_queue_purge(&queue);
	struct item x;
	TEST_ASSERT_EQUAL_INT(-ENOMSG, sensor_fair_queue_get(&queue, &x, K_NO_WAIT));
	for (unsigned i = 0; i < SENSOR_FAIR_QUEUE_IDS; i++) {
		TEST_ASSERT_EQUAL_UINT32(0, queue.shares[i].offered);
		TEST_ASSERT_EQUAL_UINT32(0, queue.shares[i].served);
		TEST_ASSERT_EQUAL_UINT32(0, queue.shares[i].pending);
	}
	put(6, 1, 3);
	TEST_ASSERT_EQUAL_INT(0, sensor_fair_queue_get(&queue, &x, K_NO_WAIT));
	TEST_ASSERT_EQUAL_UINT8(6, x.data.id);
}

static void simulate(unsigned capacity, unsigned service_rate, bool change_rates)
{
	const unsigned ids[] = {0, 4, 7, 6, 1};
	unsigned rates[] = {100, 512, 800, 54, 120};
	const unsigned batch[] = {1, 2, 5, 1, 1};
	const unsigned burst[] = {1, 8, 8, 1, 1};
	uint32_t input[8] = {0}, output[8] = {0}, source_credit[5] = {0};
	uint32_t service_credit = 0;
	uint64_t last[8] = {0};
	init(capacity);
	for (unsigned t = 0; t < 30000; t++) {
		if (t && t % 1000 == 0) queue.last_decay = k_uptime_get_32() - 1000U;
		if (change_rates && t == 5000) {rates[1] = 128; rates[4] = 60;}
		for (unsigned j = 0; j < 5; j++) {
			unsigned id = ids[j], n = batch[j];
			source_credit[j] += rates[j];
			if (source_credit[j] >= 1000 * n * burst[j]) {
				source_credit[j] -= 1000 * n * burst[j];
				for (unsigned b = 0; b < burst[j]; b++) {
					put(id, n, t);
					if (t >= 10000) input[id] += n;
				}
			}
		}
		service_credit += service_rate;
		while (service_credit >= 1000) {
			service_credit -= 1000;
			struct item x;
			if (sensor_fair_queue_get(&queue, &x, K_NO_WAIT) == 0) {
				unsigned id = x.data.id;
				TEST_ASSERT_TRUE(x.data.time >= last[id]); last[id] = x.data.time;
				TEST_ASSERT_EQUAL_UINT32(12345, x.epoch);
				if (t >= 10000) output[id] += x.data.size == widths[id] ? 1 : (x.data.size - 2) / widths[id];
			}
		}
	}
	unsigned low = 10000, high = 0;
	for (unsigned j = 0; j < 5; j++) {
		unsigned fraction = output[ids[j]] * 10000U / input[ids[j]];
		low = MIN(low, fraction); high = MAX(high, fraction);
	}
	TEST_ASSERT_GREATER_THAN_UINT32(1000, low);
	TEST_ASSERT_LESS_THAN_UINT32(500, high - low);
}

void test_bursty_sensors_retain_similar_sample_fractions_under_load(void)
{
	for (unsigned c = 8; c <= 32; c *= 2) {
		for (unsigned rate = 150; rate <= 450; rate += 150) simulate(c, rate, false);
	}
}

void test_fairness_adapts_to_changed_sensor_rates(void)
{
	simulate(16, 150, true);
	simulate(32, 300, true);
}

void test_invalid_sensor_or_oversized_payload_cannot_modify_queue(void)
{
	init(4);
	struct item x = {.data = {.id = 255, .size = 1}};
	TEST_ASSERT_EQUAL_INT(-EINVAL, sensor_fair_queue_put(&queue, &x));
	x.data.id = 0; x.data.size = 255;
	TEST_ASSERT_EQUAL_INT(-EINVAL, sensor_fair_queue_put(&queue, &x));
	TEST_ASSERT_EQUAL_UINT32(0, queue.used_msgs);
}

#ifdef __ZEPHYR__
extern int unity_main(void);

int main(void)
{
	(void)unity_main();
	return 0;
}
#endif
