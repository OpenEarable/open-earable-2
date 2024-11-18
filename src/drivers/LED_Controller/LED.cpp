#include "LED.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define TOP 255u

#define DT_LABEL_AND_COMMA(node_id)	    DT_PROP(node_id, label),
#define GPIO_DT_SPEC_GET_AND_COMMA(node_id) GPIO_DT_SPEC_GET(node_id, gpios),

static const char *const led_labels[] = {DT_FOREACH_CHILD(DT_PATH(leds), DT_LABEL_AND_COMMA)};

static const struct gpio_dt_spec leds[] = {
DT_FOREACH_CHILD(DT_PATH(leds), GPIO_DT_SPEC_GET_AND_COMMA)};

const uint8_t black[3] = {0,0,0};

LED::LED(uint32_t pin_r, uint32_t pin_g, uint32_t pin_b) {
    _r = pin_r;
    _g = pin_g;
    _b = pin_b;
}

bool LED::init() {
    nrfx_pwm_config_t const config0 = { 
      .output_pins =
      {
          _r, // channel 0
          _g, // channel 1
          _b, // channel 2
          NRF_PWM_PIN_NOT_CONNECTED  // channel 3
      },
      //.pin_inverted  = {true, true, true, true}, // not working
      .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
      .base_clock   = NRF_PWM_CLK_500kHz,
      .count_mode   = NRF_PWM_MODE_UP,
      .top_value    = TOP,
      .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
      .step_mode    = NRF_PWM_STEP_AUTO
    };
   
    if (nrfx_pwm_init(&m_pwm, &config0, NULL, NULL) != NRFX_SUCCESS)  {
        return false;
    }

    seq0.values.p_individual = &seq0_values;
    seq0.length          = NRF_PWM_VALUES_LENGTH(seq0_values);
    seq0.repeats         = 0;
    seq0.end_delay       = 0;

    set_color(black);
    
    (void)nrfx_pwm_simple_playback(&m_pwm, &seq0, 1, NRFX_PWM_FLAG_LOOP);
    
    return true;
}

void LED::set_color(const RGBColor col) {
    //seq0_values = {col[0],col[1],col[2],0};
    seq0_values = {(uint8_t)(TOP-col[0]),(uint8_t)(TOP-col[1]),(uint8_t)(TOP-col[2])};
}

LED earable_led(28u, 29u, 30u);