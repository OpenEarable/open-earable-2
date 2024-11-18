/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _PDM_MIC_H_
#define _PDM_MIC_H_

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

#include "data_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Start the mic.
 */
void pdm_mic_start(void);

/**
 * @brief	Stop the mic.
 */
void pdm_mic_stop(void);

/**
 * @brief	Initialize the pdm mic.
 *
 * @return	0 on success, error otherwise.
 */
int pdm_mic_init(void);

int pdm_datapath_start(struct data_fifo *fifo_rx);

#ifdef __cplusplus
}
#endif

#endif /* _PDM_MIC_H_ */
