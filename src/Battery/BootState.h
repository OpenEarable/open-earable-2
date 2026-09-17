#ifndef _BOOT_STATE_H
#define _BOOT_STATE_H

#include "openearable_common.h"

// GPREGRET[0] is reserved for MCUboot; this marker survives a recovery reboot.
#define OE_SHUTDOWN_MARKER 0xA5u

#ifdef __cplusplus
extern "C" {
#endif

extern struct boot_state oe_boot_state;

#ifdef __cplusplus
}
#endif

#endif
