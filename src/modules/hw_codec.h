/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _HW_CODEC_H_
#define _HW_CODEC_H_

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

// Audio Mode Options
enum audio_mode {
    AUDIO_MODE_NORMAL = 0,
    AUDIO_MODE_TRANSPARENCY = 1,
    AUDIO_MODE_ANC = 2
};

#ifdef __cplusplus
extern "C" {
#endif

ZBUS_OBS_DECLARE(volume_evt_sub);

/**
 * @brief  Set volume on HW_CODEC
 *
 * @details Also unmutes the volume on HW_CODEC
 *
 * @param  set_val  Set the volume to a specific value.
 *                  This range of the value is between 0 to 128.
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_set(uint8_t set_val);

/**
 * @brief  Adjust volume on HW_CODEC
 *
 * @details Also unmute the volume on HW_CODEC
 *
 * @param  adjustment  The adjustment in dB, can be negative or positive.
 *			If the value 0 is used, the previous known value will be
 *			written, default value will be used if no previous value
 *			exists
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_adjust(int8_t adjustment);

/**
 * @brief Decrease output volume on HW_CODEC by 3 dB
 *
 * @details Also unmute the volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_decrease(void);

/**
 * @brief Increase output volume on HW_CODEC by 3 dB
 *
 * @details Also unmute the volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_increase(void);

/**
 * @brief  Mute volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_mute(void);

/**
 * @brief  Unmute volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_unmute(void);

/**
 * @brief Enable relevant settings in HW_CODEC to
 *        send and receive PCM data over I2S
 *
 * @note  FLL1 must be toggled after I2S has started to enable HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_default_conf_enable(void);

/**
 * @brief Reset HW_CODEC
 *
 * @note  This will first disable output, then do a soft reset
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_soft_reset(void);

/**
 * @brief Initialize HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_init(void);

int hw_codec_stop_audio(void);

/**
 * @brief Set the audio processing mode
 *
 * @param mode The audio mode to set (normal, transparency, ANC)
 * @return 0 if successful, error otherwise
 */
int hw_codec_set_audio_mode(enum audio_mode mode);

enum audio_mode hw_codec_get_audio_mode();

#ifdef __cplusplus
}
#endif

#endif /* _HW_CODEC_H_ */
