#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>

#include "button_assignments.h"
//#include "Button.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct k_msgq button_queue;

bool button_pressed(enum button_pin_names pin);

#ifdef __cplusplus
}
#endif

#endif