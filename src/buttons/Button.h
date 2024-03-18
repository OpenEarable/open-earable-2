#ifndef OPEN_EARABLE_BUTTON_H
#define OPEN_EARABLE_BUTTON_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "button_assignments.h"
#include "nrf5340_audio_common.h"

class Button {
public:
    Button(gpio_dt_spec spec, bool inverted = false);

    void begin();
    void end();

    void inverted(bool _inverted = true);

    button_action getState() const;

    void setDebounceTime(unsigned long debounceTime);

    //static void task();
private:
    //int _pin;
    bool _inverted = false;
    unsigned long _lastDebounceTime;
    //unsigned long _pressStartTime;
    unsigned long _debounceDelay = 25;

    //static bool running = false;

    //static const k_tid_t button_publish;

    const static struct gpio_dt_spec buttons[];

    const struct gpio_dt_spec button;
    static struct gpio_callback button_cb_data;

    button_action _buttonState = BUTTON_RELEASED;

    void _read_state();
    static void _earable_btn_read_state();
    static void button_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins);

    int update_state();
};

extern Button earable_btn;
extern Button volume_up_btn;
extern Button volume_down_btn;
extern Button four_btn;
//extern Button five_btn;

#endif //OPEN_EARABLE_BUTTON_H
