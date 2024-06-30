#ifndef _LOAD_SWITCH_PM_H
#define _LOAD_SWITCH_PM_H

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#ifdef __cplusplus
extern "C" {
#endif

int dev_pm_init(const struct device *dev);
int dev_pm_control(const struct device *dev, enum pm_device_action action);

#ifdef __cplusplus
}
#endif

#endif