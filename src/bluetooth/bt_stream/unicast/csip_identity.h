/* SPDX-License-Identifier: LicenseRef-Nordic-5-Clause */
#ifndef OPENEARABLE_CSIP_IDENTITY_H_
#define OPENEARABLE_CSIP_IDENTITY_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define OPENEARABLE_CSIP_SIRK_SIZE 16U

/**
 * Encode the set identity used by OpenEarable 2.2.9 and earlier.
 *
 * A programmed UICR pair ID takes precedence, including zero. Only erased
 * UICR (UINT32_MAX) falls back to the low 32 bits of the chip ID, allowing
 * unpaired devices to advertise their own identity until peer pairing.
 * The wire format is eight uppercase ASCII hex digits followed by eight
 * zero bytes, not the raw integer or a newly generated key. No storage is
 * modified, so FOTA upgrades retain the existing pair identity.
 *
 * @param pair_id Stored UICR pair identifier.
 * @param device_id Factory chip identifier.
 * @param sirk Output buffer of OPENEARABLE_CSIP_SIRK_SIZE bytes.
 */
static inline void openearable_csip_identity_encode(
	uint32_t pair_id, uint64_t device_id, uint8_t sirk[OPENEARABLE_CSIP_SIRK_SIZE])
{
	uint32_t identity = pair_id == UINT32_MAX ? (uint32_t)device_id : pair_id;

	memset(sirk, 0, OPENEARABLE_CSIP_SIRK_SIZE);
	(void)snprintf((char *)sirk, OPENEARABLE_CSIP_SIRK_SIZE, "%08X",
		       (unsigned int)identity);
}

#endif /* OPENEARABLE_CSIP_IDENTITY_H_ */
