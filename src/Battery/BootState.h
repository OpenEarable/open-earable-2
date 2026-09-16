#ifndef _BOOT_STATE_H
#define _BOOT_STATE_H

#include "openearable_common.h"

/* GPREGRET[1] belongs to battery management; GPREGRET[0] remains for DFU.
 * Consume only the charge-only request at boot. Safety bits survive resets
 * until USB absence is observed, so reboot cannot renew the charger timer.
 */
#define OE_BOOT_TAG 0xC8
#define OE_BOOT_TAG_MASK 0xF8
#define OE_BOOT_FLAG_CHARGE_ONLY 0x01
#define OE_BOOT_FLAG_TIMER_USED 0x02
#define OE_BOOT_FLAG_CHARGER_FAULT 0x04
#define OE_BOOT_SAFETY_FLAGS (OE_BOOT_FLAG_TIMER_USED | OE_BOOT_FLAG_CHARGER_FAULT)
#define OE_BOOT_CHARGE_ONLY (OE_BOOT_TAG | OE_BOOT_FLAG_CHARGE_ONLY)

static inline uint8_t oe_boot_flags(uint32_t retained)
{
    return (retained & OE_BOOT_TAG_MASK) == OE_BOOT_TAG ? retained & 0x07 : 0;
}

static inline uint8_t oe_boot_encode(uint8_t flags)
{
    return (flags & 0x07) ? OE_BOOT_TAG | (flags & 0x07) : 0;
}

static inline uint8_t oe_boot_request_charge_only(uint32_t retained)
{
    return oe_boot_encode(oe_boot_flags(retained) | OE_BOOT_FLAG_CHARGE_ONLY);
}

#ifdef __cplusplus
extern "C" {
#endif

extern struct boot_state oe_boot_state;

#ifdef __cplusplus
}
#endif

#endif
