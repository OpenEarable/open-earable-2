#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include "button_assignments.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct k_msgq button_queue;

int button_pressed(enum button_pin_names pin, bool * pressed);

#ifdef __cplusplus
}
#endif

#endif