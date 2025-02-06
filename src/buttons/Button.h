#ifndef OPEN_EARABLE_BUTTON_H
#define OPEN_EARABLE_BUTTON_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "button_assignments.h"
#include "openearable_common.h"
#include "zbus_common.h"

#define BUTTON_DEBOUNCE K_MSEC(10)

class Button {
public:
    Button(gpio_dt_spec spec);

    void begin();
    void end();

    button_action getState() const;
private:
    k_work_delayable button_work;

    const struct gpio_dt_spec button;
    static struct gpio_callback button_cb_data;

    button_action _buttonState = BUTTON_RELEASED;
    button_action _temp_buttonState = BUTTON_RELEASED;

    static void button_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins);

    int update_state();

    static void button_work_handler(struct k_work * work);
};

extern Button earable_btn;
// extern Button volume_up_btn;
// extern Button volume_down_btn;
// extern Button four_btn;
// extern Button five_btn;

#endif //OPEN_EARABLE_BUTTON_H
