#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include "boot_charger_status.h"

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
static uint8_t ldo, button = 0x68, control;
struct FakePower { volatile uint32_t GPREGRET[2]; } fake_power;
#define NRF_POWER (&fake_power)
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
static int i2c_reg_read_byte(const device *, int, int reg, uint8_t *value) {
    if (fail_reads) return -EIO;
    assert(reg == 0);
    *value = control; control &= ~0x10;
    return 0;
}
static int i2c_burst_read(const device *, int, int reg, uint8_t *value, uint32_t len) {
    if (fail_reads) return -EIO;
    assert(reg == 7 && len == 2);
    value[0] = ldo; value[1] = button;
    button &= ~3; control &= ~0x10;
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
    bus_ready = true;
    for (uint8_t flags = 0; flags < 8; flags++) {
        rail_enabled = false; ldo = 0x7c;
        button = 0x68 | (flags & 3); control = (flags & BOOT_CHARGER_RESET) << 2;
        fake_power.GPREGRET[1] = BOOT_CHARGER_MARKER | 7; // stale previous boot
        assert(init_3v3_regulator() == 0);
        assert(boot_charger_take(&fake_power.GPREGRET[1]) == flags);
        assert(boot_charger_take(&fake_power.GPREGRET[1]) == 0);
        assert(fake_power.GPREGRET[1] == 0);
    }
    // Events arriving during a long image swap are accumulated by the watchdog.
    rail_enabled = false; ldo = 0x64; button = 0x6a;
    assert(init_3v3_regulator() == 0);
    button = 0x69; charger_watchdog_handler(nullptr);
    assert(boot_charger_take(&fake_power.GPREGRET[1]) == 3);
    fake_power.GPREGRET[1] = CLEANUP_CHARGING_OFF_MARKER;
    button = 0x6b; control = 0x10; rail_enabled = false;
    assert(init_3v3_regulator() == 0);
    assert(boot_charger_take(&fake_power.GPREGRET[1]) == 0);
    assert(fake_power.GPREGRET[1] == CLEANUP_CHARGING_OFF_MARKER);
    std::puts("PASS: boot wake/reset flags survive LDO reads and are consumed once; stale and charging-only markers handled");
    std::puts("PASS: bootloader verifies 3.3 V before enabling flash and disables the rail on lost configuration");
}
