#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

struct device {};
struct gpio_dt_spec {};
struct k_work {};
struct k_work_delayable {};
static device fake_bus;
static const device *charger_i2c = &fake_bus;
static gpio_dt_spec ls_3_3_pin;
static k_work_delayable charger_watchdog_work;
static bool rail_enabled, bus_ready = true, drop_writes, fail_reads, fail_gpio;
static int scheduled;
static uint8_t ldo;
static constexpr int CHARGER_ADDR = 0x6a, CHARGER_LDO_CTRL = 7;
static constexpr uint8_t CHARGER_LDO_MASK = 0x7c, CHARGER_LDO_3V3 = 25 << 2;
#define ARG_UNUSED(x) (void)(x)
#define K_SECONDS(x) (x)
static bool device_is_ready(const device *) { return bus_ready; }
static void k_busy_wait(int) {}
static int gpio_pin_set_dt(const gpio_dt_spec *, int value) {
    if (fail_gpio) return -EIO;
    rail_enabled = value;
    return 0;
}
static int i2c_reg_read_byte(const device *, int, int, uint8_t *value) {
    if (fail_reads) return -EIO;
    *value = ldo;
    return 0;
}
static int i2c_reg_write_byte(const device *, int, int, uint8_t value) {
    if (!drop_writes) {
        if (rail_enabled || (ldo & 0x80)) value = (value & ~0x7c) | (ldo & 0x7c);
        ldo = value;
    }
    return 0;
}
static void k_work_init_delayable(k_work_delayable *, void (*)(k_work *)) {}
static void k_work_schedule(k_work_delayable *, int) { scheduled++; }
#define printk(...) ((void)0)
#include "bootloader_functions.inc"

int main() {
    for (uint8_t initial : {uint8_t(0x7c), uint8_t(0xfd), uint8_t(0x64)}) {
        rail_enabled = false; ldo = initial; scheduled = 0;
        assert(init_3v3_regulator() == 0);
        assert(rail_enabled && ldo == (0x64 | (initial & 1)) && scheduled == 1);
        charger_watchdog_handler(nullptr);
        assert(rail_enabled && scheduled == 2);
        ldo = 0x7c; charger_watchdog_handler(nullptr);
        assert(!rail_enabled && scheduled == 2);
    }
    rail_enabled = false; ldo = 0x7c; drop_writes = true;
    assert(init_3v3_regulator() == -EIO && !rail_enabled);
    drop_writes = false; fail_reads = true;
    assert(init_3v3_regulator() == -EIO && !rail_enabled);
    fail_reads = false; bus_ready = false;
    assert(init_3v3_regulator() == -ENODEV && !rail_enabled);
    std::puts("PASS: bootloader verifies 3.3 V before enabling flash and disables the rail on lost configuration");
}
