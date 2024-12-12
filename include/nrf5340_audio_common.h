/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _NRF5340_AUDIO_COMMON_H_
#define _NRF5340_AUDIO_COMMON_H_

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "LoadSwitch.h"

#define ZBUS_READ_TIMEOUT_MS	K_MSEC(100)
#define ZBUS_ADD_OBS_TIMEOUT_MS K_MSEC(200)

#define SENSOR_DATA_FIXED_LENGTH 9

#define millis() k_cyc_to_ms_floor32(k_cycle_get_64())
#define micros() k_cyc_to_us_floor64(k_cycle_get_64())

#define load_switch_sd_id DT_NODELABEL(load_switch_sd)
#define load_switch_1_8_id DT_NODELABEL(load_switch)
#define load_switch_3_3_id DT_NODELABEL(bq25120a)

extern const struct device *const cons;
extern const struct device *const ls_1_8;
extern const struct device *const ls_3_3;
extern const struct device *const ls_sd;

struct boot_state {
	bool timer_reset;
	uint64_t device_id;
};

enum pairing_state {
	SET_PAIRING, //BINDING,
	BONDING,
	PAIRED, //DISONNECTED
	CONNECTED,
};

enum charging_state {
	DISCHARGING,
	CHARGING,
	FULLY_CHARGED,
	FAULT,
};

enum led_state {
	STATE_INDICATION,
	CUSTOM,
};

struct earable_state {
	enum pairing_state pairing_state;
	enum charging_state charging_state;
	enum led_state led_state;
};	 

enum sensor_id {
	ID_IMU=0,
	ID_TEMP_BARO=1,
	ID_PPG=4,
	ID_PULSOX=5,
	ID_OPTTEMP=6,
	ID_BONE_CONDUCTION=7,
};

struct battery_data {
    uint8_t battery_level;
    uint16_t charging_state;
};

struct sensor_data {
    uint8_t id;
    uint8_t size;
    uint32_t time;
    float data[SENSOR_DATA_FIXED_LENGTH];
    //uint8_t * data;
} __attribute__((packed));

struct sensor_config {
    uint8_t sensorId;
    float sampleRate;
    uint32_t latency;
} __attribute__((packed));

/***** Messages for zbus ******/

enum button_action {
	BUTTON_RELEASED = 0,
	BUTTON_PRESS = 1,
};

struct button_msg {
	uint32_t button_pin;
	enum button_action button_action;
};

enum le_audio_evt_type {
	LE_AUDIO_EVT_CONFIG_RECEIVED = 1,
	LE_AUDIO_EVT_PRES_DELAY_SET,
	LE_AUDIO_EVT_STREAMING,
	LE_AUDIO_EVT_NOT_STREAMING,
	LE_AUDIO_EVT_SYNC_LOST,
	LE_AUDIO_EVT_NO_VALID_CFG,
};

struct le_audio_msg {
	enum le_audio_evt_type event;
	struct bt_conn *conn;
	struct bt_le_per_adv_sync *pa_sync;
	enum bt_audio_dir dir;
};

/**
 * tx_sync_ts_us	The timestamp from get_tx_sync.
 * curr_ts_us		The current time. This must be in the controller frame of reference.
 */
struct sdu_ref_msg {
	uint32_t tx_sync_ts_us;
	uint32_t curr_ts_us;
	bool adjust;
};

enum bt_mgmt_evt_type {
	BT_MGMT_EXT_ADV_WITH_PA_READY = 1,
	BT_MGMT_CONNECTED,
	BT_MGMT_SECURITY_CHANGED,
	BT_MGMT_PA_SYNCED,
	BT_MGMT_PA_SYNC_LOST,
	BT_MGMT_DISCONNECTED,
};

struct bt_mgmt_msg {
	enum bt_mgmt_evt_type event;
	struct bt_conn *conn;
	struct bt_le_ext_adv *ext_adv;
	struct bt_le_per_adv_sync *pa_sync;
	uint32_t broadcast_id;
	uint8_t pa_sync_term_reason;
};

enum volume_evt_type {
	VOLUME_UP = 1,
	VOLUME_DOWN,
	VOLUME_SET,
	VOLUME_MUTE,
	VOLUME_UNMUTE,
};

struct volume_msg {
	enum volume_evt_type event;
	uint8_t volume;
};

enum content_control_evt_type {
	MEDIA_START = 1,
	MEDIA_STOP,
};

struct content_control_msg {
	enum content_control_evt_type event;
};

/**
 * @brief	Initialize the software modules that are common for all the audio samples.
 *
 * @return	0 if successful, error otherwise.
 */
int nrf5340_audio_common_init(void);

#endif /* _NRF5340_AUDIO_COMMON_H_ */
