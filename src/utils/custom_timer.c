#include <custom_timer.h>
#include <zephyr/kernel.h>

#define US_PER_TICK (1000000ULL / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_INTERVAL_US (UINT32_MAX * US_PER_TICK)

static struct k_timer ticks_timer;
static volatile uint64_t micros_counter = 0;

// Timer expiry function to increment the counter
void timer_expiry_function(struct k_timer *timer_id)
{
    micros_counter += MAX_INTERVAL_US;
}

void custom_timer_start(void)
{
    k_timer_init(&ticks_timer, timer_expiry_function, NULL);
    k_timer_start(&ticks_timer, K_TICKS(UINT32_MAX), K_TICKS(UINT32_MAX)); // Max interval
}

uint64_t custom_micros(void)
{
    // Read remaining ticks and convert to microseconds
    k_ticks_t elapsed_ticks = UINT32_MAX - k_timer_remaining_ticks(&ticks_timer);
    return micros_counter + (elapsed_ticks * US_PER_TICK);
}

uint64_t custom_millis(void)
{
    return custom_micros() / 1000;
}
