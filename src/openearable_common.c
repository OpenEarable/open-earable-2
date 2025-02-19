/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "openearable_common.h"

const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
const struct device *const ls_1_8 = DEVICE_DT_GET(load_switch_1_8_id);
const struct device *const ls_3_3 = DEVICE_DT_GET(load_switch_3_3_id);
const struct device *const ls_sd = DEVICE_DT_GET(load_switch_sd_id);
