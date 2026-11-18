#ifndef OPENEARABLE_BOOT_CHARGER_STATUS_H
#define OPENEARABLE_BOOT_CHARGER_STATUS_H

#include <stdint.h>

/* GPREGRET[1] is shared by MCUboot and PowerManager. The application consumes
 * this snapshot once. Preserve the existing explicit charging-only reboot. */
#define CLEANUP_CHARGING_OFF_MARKER 0xA6u
#define BOOT_CHARGER_MARKER 0xB0u
#define BOOT_CHARGER_EVENTS 0x07u
#define BOOT_CHARGER_WAKE2 0x01u
#define BOOT_CHARGER_RESET 0x04u

static inline uint8_t boot_charger_take(volatile uint32_t *reg)
{
    uint32_t value = *reg;
    if ((value & ~BOOT_CHARGER_EVENTS) != BOOT_CHARGER_MARKER) return 0;
    *reg = 0;
    return value & BOOT_CHARGER_EVENTS;
}

static inline void boot_charger_save(volatile uint32_t *reg, uint8_t events)
{
    if (*reg == CLEANUP_CHARGING_OFF_MARKER) return;
    events |= boot_charger_take(reg);
    *reg = BOOT_CHARGER_MARKER | (events & BOOT_CHARGER_EVENTS);
}

#endif
