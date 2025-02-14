#include <custom_timer.h>
#include <zephyr/kernel.h>

static struct k_timer my_timer;

// 64-bit counter for microseconds
static volatile uint64_t micros_counter = 0;

// Timer expiry function to increment the counter
void timer_expiry_function(struct k_timer *timer_id)
{
    micros_counter += UINT32_MAX * 1000000ULL / CONFIG_SYS_CLOCK_TICKS_PER_SEC; 
}

void custom_timer_start(void)
{
    k_timer_init(&my_timer, timer_expiry_function, NULL);
    k_timer_start(&my_timer, K_TICKS(UINT32_MAX), K_TICKS(UINT32_MAX)); // Max interval
}

uint64_t custom_micros(void)
{
    // Read remaining ticks and convert to microseconds
    k_ticks_t elapsed_ticks = UINT32_MAX - k_timer_remaining_ticks(&my_timer);
    uint64_t elapsed_us = elapsed_ticks * 1000000ULL / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

    return micros_counter + elapsed_us;
}

uint64_t custom_millis(void)
{
    return custom_micros() / 1000;
}
