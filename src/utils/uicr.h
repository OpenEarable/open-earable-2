/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _UICR_H_
#define _UICR_H_

#include <stdint.h>

// TODO: Discuss better alternative for UICR storage. This memory range is not documented
#define UICR_APP_BASE_ADDR (NRF_UICR_S_BASE + 0xF0)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get raw channel value from UICR
 */
uint8_t uicr_channel_get(void);

/**
 * @brief Write raw channel value to UICR
 *
 * @param channel Channel value
 *
 * @return 0 if successful
 * @return -EROFS if different channel is already written
 * @return -EIO if channel failed to be written
 */
int uicr_channel_set(uint8_t channel);

/**
 * @brief Get raw channel value from UICR
 */
uint32_t uicr_sirk_get(void);

/**
 * @brief Write raw channel value to UICR
 *
 * @param channel Channel value
 *
 * @return 0 if successful
 * @return -EROFS if different channel is already written
 * @return -EIO if channel failed to be written
 */
int uicr_sirk_set(uint32_t sirk);

/**
 * @brief Get standalone value from UICR
 */
uint8_t uicr_standalone_get(void);

/**
 * @brief Get Segger serial number value from UICR
 */
uint64_t uicr_snr_get(void);

/**
 * @brief Get hardware revision string from UICR
 */
void uicr_hw_revision_get(char *hw_version);

/**
 * @brief Promote a 2.0.x hardware revision to 2.1.x.
 *
 * A blank UICR value is persisted as 2.1.0. An already-programmed 2.0.x
 * value cannot be changed without erasing the full UICR page, so the
 * promoted value is applied at runtime and reported through Device Info.
 * The hardware probe repeats on every boot.
 *
 * @return 0 when no promotion is needed or the promoted value is active
 * @return -EIO if a blank UICR value could not be programmed
 */
int uicr_hw_revision_promote_2_0_to_2_1(void);

#ifdef __cplusplus
}
#endif

#endif /* _UICR_H_ */
