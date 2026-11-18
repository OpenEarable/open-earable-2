#pragma once
#include "battery_settings.h"
#include <zephyr/kernel.h>
inline uint64_t micros() {return clock_us.load();}
