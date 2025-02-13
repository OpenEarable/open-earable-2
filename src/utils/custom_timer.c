#include <custom_timer.h>

#include <hal/nrf_timer.h>
#include <nrfx_timer.h>
#include <zephyr/logging/log.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

LOG_MODULE_REGISTER(custom_timer, CONFIG_LOG_DEFAULT_LEVEL);


#define TIMER_INSTANCE NRF_TIMER0  // Use TIMER0 for microsecond tracking

static volatile uint32_t upper_32_bits = 0;  // Tracks the high 32-bit overflow

// Timer overflow interrupt handler
void TIMER0_IRQHandler(void) {
    /** if (nrf_timer_event_check(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0)) {
        nrf_timer_event_clear(TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
        upper_32_bits++;  // Increment the upper 32-bit counter on overflow
    }

    //k_work_submit for logging

    LOG_INF("TIMER OVERFLOWED");*/
}

// Function to start the 64-bit timer
void custom_timer_start(void) {
    // Configure TIMER0 as a 32-bit timer
   /*** / nrf_timer_mode_set(TIMER_INSTANCE, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(TIMER_INSTANCE, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(TIMER_INSTANCE, 7); // 1 tick == 1µs

    // Set the overflow event at the max 32-bit value (0xFFFFFFFF)
    nrf_timer_cc_set(TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0, 0xFFFFFFFF);

    // Enable interrupt on overflow
    nrf_timer_int_enable(TIMER_INSTANCE, NRF_TIMER_INT_COMPARE0_MASK);

    // Configure and enable interrupt
    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Start the timer
    nrf_timer_task_trigger(TIMER_INSTANCE, NRF_TIMER_TASK_START);*/
}

// Function to get microseconds since boot (64-bit)
uint64_t custom_micros(void) {
    return k_cyc_to_us_floor64(k_cycle_get_32());
    return 0;
    uint32_t high, high_check, low;
    
    do {
        high = upper_32_bits;  
        nrf_timer_task_trigger(TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE0); // captures value in cc register
        low = nrf_timer_cc_get(TIMER_INSTANCE, NRF_TIMER_CC_CHANNEL0); // reads it from cc register after capture, avoids changes during read
        high_check = upper_32_bits;
    } while (high != high_check);  // Retry if an overflow happened while reading

    return ((uint64_t)high << 32) | low;
}

// Function to get milliseconds since boot (64-bit)
uint64_t custom_millis(void) {
    return k_cyc_to_ms_floor32(k_cycle_get_32());
    return custom_micros() / 1000;  // Convert microseconds to milliseconds
}
