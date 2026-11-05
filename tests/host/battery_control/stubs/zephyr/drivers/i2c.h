#pragma once
#include <zephyr/kernel.h>
#include <exception>
struct device {};
extern const device fake_device;
#define DT_NODELABEL(x) x
#define DT_REG_ADDR(x) 0x6a
#define DEVICE_DT_GET(x) (&fake_device)
#define I2C_SPEED_FAST 2
#define I2C_SPEED_MASK 7
#define I2C_SPEED_SET(x) (x)
inline int device_is_ready(const device *) { return 1; }
inline int i2c_get_config(const device *, uint32_t *v) { *v=2;return 0; }
inline int i2c_configure(const device *, uint32_t) { return 0; }
#define __ASSERT(x,...) do { if (!(x)) std::terminate(); } while(0)
int i2c_burst_read(const device *, uint16_t, uint8_t, uint8_t *, uint32_t);
int i2c_burst_write(const device *, uint16_t, uint8_t, const uint8_t *, uint32_t);
