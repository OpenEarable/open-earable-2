/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BANK_CONTROLLER_H_
#define _BANK_CONTROLLER_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the bank controller
 * @return 0 for success, error otherwise
 */
int bank_controller_init(void);

/**
 * @brief Get current filter bank state
 * @return true if bank A is active, false if bank B is active
 */
bool bank_controller_is_bank_a_active(void);

#ifdef __cplusplus
}
#endif

#endif /* _BANK_CONTROLLER_H_ */
