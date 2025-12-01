/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "uicr.h"

#include <stdint.h>
#include <errno.h>
#include <nrfx_nvmc.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uicr, CONFIG_LOG_DEFAULT_LEVEL);

/* Memory address to store segger number of the board */
#define MEM_ADDR_UICR_SNR UICR_APP_BASE_ADDR
/* Memory address to store the channel intended used for this board */
#define MEM_ADDR_UICR_CH (MEM_ADDR_UICR_SNR + sizeof(uint32_t))
#define MEM_ADDR_UICR_SIRK (MEM_ADDR_UICR_CH + sizeof(uint32_t))
#define MEM_ADDR_UICR_STAL (MEM_ADDR_UICR_SIRK + sizeof(uint32_t))
#define MEM_ADDR_UICR_HW (MEM_ADDR_UICR_STAL + sizeof(uint32_t))

uint8_t uicr_channel_get(void)
{
	return *(uint8_t *)MEM_ADDR_UICR_CH;
}

int uicr_channel_set(uint8_t channel)
{
	if (channel == *(uint8_t *)MEM_ADDR_UICR_CH) {
		return 0;
	} else if (*(uint32_t *)MEM_ADDR_UICR_CH != 0xFFFFFFFF) {
		return -EROFS;
	}

	nrfx_nvmc_byte_write(MEM_ADDR_UICR_CH, channel);

	if (channel == *(uint8_t *)MEM_ADDR_UICR_CH) {
		return 0;
	} else {
		return -EIO;
	}
}

uint32_t uicr_sirk_get(void)
{
	return *(uint32_t *)MEM_ADDR_UICR_SIRK;
}

int uicr_sirk_set(uint32_t sirk)
{
	if (sirk == *(uint32_t *)MEM_ADDR_UICR_SIRK) {
		return 0;
	} else if (*(uint32_t *)MEM_ADDR_UICR_SIRK != 0xFFFFFFFF) {
		return -EROFS;
	}

	bool write_check = nrfx_nvmc_word_writable_check(MEM_ADDR_UICR_SIRK, sirk);

	if (!write_check) return -EROFS;

	nrfx_nvmc_word_write(MEM_ADDR_UICR_SIRK, sirk);

	while(!nrfx_nvmc_write_done_check()) {}

	if (sirk == *(uint32_t *)MEM_ADDR_UICR_SIRK) {
		return 0;
	} else {
		return -EIO;
	}
}

uint8_t uicr_standalone_get(void)
{
	return *(uint8_t *)MEM_ADDR_UICR_STAL;
}

// Michael: Collision with MEM_ADDR_UICR_CH
uint64_t uicr_snr_get(void)
{
	return *(uint64_t *)MEM_ADDR_UICR_SNR;
}

void uicr_hw_revision_get(char *hw_version)
{
	uint32_t hw_data = *(uint32_t *)MEM_ADDR_UICR_HW;
	
	uint8_t major = (hw_data >> 24) & 0xFF;
	uint8_t minor = (hw_data >> 16) & 0xFF;
	uint8_t patch = (hw_data >> 8) & 0xFF;

	if (major == 0xFF && minor == 0xFF && patch == 0xFF) {
		LOG_WRN("UICR HW revision not set, returning default 2.0.0");
		snprintf(hw_version, 16, "2.0.0");
		return;
	}

	snprintf(hw_version, 16, "%u.%u.%u", major, minor, patch);
}
