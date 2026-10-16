/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @defgroup audio_app_streamctrl Audio Stream Control
 * @{
 * @brief Stream control API for Audio applications.
 *
 * This module provides stream state management and control functions for audio streaming
 * operations.
 */

#ifndef _STREAMCTRL_H_
#define _STREAMCTRL_H_

#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>

/**
 * @brief Stream state enumeration for audio streaming operations.
 */
enum stream_state {
	STATE_STREAMING,
	STATE_PAUSED,
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Get the current streaming state.
 *
 * @return	strm_state enum value.
 */
uint8_t stream_state_get(void);

/**
 * @brief	Send audio data over the stream.
 *
 * @param	audio_frame	Pointer to the audio buffer.
 */
void streamctrl_send(struct net_buf const *const audio_frame);

/**
 * @}
 */

int streamctrl_start();

#ifdef __cplusplus
}
#endif

#endif /* _STREAMCTRL_H_ */
