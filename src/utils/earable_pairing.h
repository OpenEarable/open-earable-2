/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENEARABLE_PAIRING_H_
#define OPENEARABLE_PAIRING_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum earable_pair_mode {
	EARABLE_PROVISIONED = 0,
	EARABLE_SEEKING = 1,
	EARABLE_STANDALONE = 2,
	EARABLE_PAIRED = 3,
};

/* Read the boot configuration once, before starting Bluetooth or audio. */
int earable_pairing_load(void);
bool earable_pairing_is_seeking(void);
uint32_t earable_pairing_identity(void);
int earable_pairing_side_override(void); /* -1 uses the provisioned side */
uint32_t earable_pairing_previous_identity(void);

struct bt_data;
int earable_pairing_start_discovery(const struct bt_data *adv, size_t count);

/* Serialize confirmed control commands and discovery through the next reboot. */
bool earable_pairing_claim(void);
void earable_pairing_release(void);
int earable_pairing_set_mode(enum earable_pair_mode mode, uint8_t side);
int earable_pairing_bond(uint32_t id, uint8_t side);
int earable_pairing_allow_peer(uint32_t id);
/* Application reboot that drains active recording and sensors. */
void earable_pairing_reboot(void);

#ifdef __cplusplus
}
#endif
#endif
