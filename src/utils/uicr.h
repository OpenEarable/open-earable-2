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

#ifdef __cplusplus
}
#endif

#endif /* _UICR_H_ */
