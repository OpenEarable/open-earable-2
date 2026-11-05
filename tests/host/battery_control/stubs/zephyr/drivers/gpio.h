#pragma once
#include <zephyr/drivers/i2c.h>
struct gpio_dt_spec {const device *port;unsigned pin;};
struct gpio_callback {uint32_t pin_mask=0;};
using gpio_callback_handler_t=void(*)(const device *,gpio_callback *,uint32_t);
enum {pg_gpios=1,cd_gpios=2,int_gpios=3};
#define GPIO_DT_SPEC_GET(n,p) gpio_dt_spec{&fake_device,p}
#define GPIO_INPUT 1
#define GPIO_OUTPUT_INACTIVE 2
#define GPIO_OUTPUT_ACTIVE 4
#define GPIO_INT_EDGE_BOTH 8
#define GPIO_INT_EDGE_TO_ACTIVE 16
#define GPIO_INT_LEVEL_ACTIVE 32
extern bool cd_high,usb_power;
inline int gpio_pin_configure_dt(const gpio_dt_spec *s,int flags) {if(s->pin==2)cd_high=flags&GPIO_OUTPUT_ACTIVE;return 0;}
inline int gpio_pin_interrupt_configure_dt(const gpio_dt_spec *,int) {return 0;}
inline int gpio_pin_set_dt(const gpio_dt_spec *s,int value) {if(s->pin==2)cd_high=value;return 0;}
inline int gpio_pin_get_dt(const gpio_dt_spec *s) {return s->pin==1?usb_power:cd_high;}
inline void gpio_init_callback(gpio_callback *,gpio_callback_handler_t,uint32_t) {}
inline int gpio_add_callback(const device *,gpio_callback *) {return 0;}
