#ifndef OPEN_EARABLE_LED_H
#define OPEN_EARABLE_LED_H

#include <nrfx_pwm.h>
#include <zephyr/devicetree.h>
//#include <hal/nrf_pwm.h>

/*#define EPIN_RGB_R 13
#define EPIN_RGB_G 14
#define EPIN_RGB_B 15*/

typedef uint8_t RGBColor[3];

class LED {
public:
    LED(uint32_t pin_r, uint32_t pin_g, uint32_t pin_b);

    bool init();

    void set_color(const RGBColor col);

private:
    uint32_t _r,_g,_b;

    nrfx_pwm_t m_pwm = NRFX_PWM_INSTANCE(1);
    nrf_pwm_sequence_t seq0;
    nrf_pwm_values_individual_t seq0_values = {0,0,0,0};
};

extern LED earable_led;

#endif //OPEN_EARABLE_LED_H
