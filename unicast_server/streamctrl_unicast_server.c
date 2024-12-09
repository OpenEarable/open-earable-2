/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/sys/reboot.h>

#include "nrf5340_audio_common.h"
#include "nrf5340_audio_dk.h"
//#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
//#include "button_handler.h"
#include "../src/buttons/button_manager.h"
#include "bt_le_audio_tx.h"
#include "bt_mgmt.h"
#include "bt_rendering_and_capture.h"
#include "audio_datapath.h"
#include "bt_content_ctrl.h"
#include "unicast_server.h"
#include "le_audio.h"
#include "le_audio_rx.h"

#include "common/bt_str.h"

#include "../src/Battery/BootState.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(streamctrl_unicast_server, CONFIG_STREAMCTRL_LOG_LEVEL);

ZBUS_SUBSCRIBER_DEFINE(button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_MSG_SUBSCRIBER_DEFINE(le_audio_evt_sub);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(volume_chan);

ZBUS_OBS_DECLARE(volume_evt_sub);

static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;

static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;

K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);

static enum stream_state strm_state = STATE_PAUSED;

/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

/**
 * @brief	Handle button activity.
 */
static void button_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&button_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct button_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		LOG_DBG("Got btn evt from queue - id = %d, action = %d", msg.button_pin,
			msg.button_action);

		if (msg.button_action == BUTTON_RELEASED) continue;

		if (msg.button_action != BUTTON_PRESS) {
			LOG_WRN("Unhandled button action");
			return;
		}

		switch (msg.button_pin) {
		case BUTTON_PLAY_PAUSE:
			if (IS_ENABLED(CONFIG_WALKIE_TALKIE_DEMO)) {
				LOG_WRN("Play/pause not supported in walkie-talkie mode");
				break;
			}

			if (bt_content_ctlr_media_state_playing()) {
				ret = bt_content_ctrl_stop(NULL);
				if (ret) {
					LOG_WRN("Could not stop: %d", ret);
				}

			} else if (!bt_content_ctlr_media_state_playing()) {
				ret = bt_content_ctrl_start(NULL);
				if (ret) {
					LOG_WRN("Could not start: %d", ret);
				}

			} else {
				LOG_WRN("In invalid state: %d", strm_state);
			}

			break;

		case BUTTON_VOLUME_UP:
			ret = bt_r_and_c_volume_up();
			if (ret) {
				LOG_WRN("Failed to increase volume: %d", ret);
			}

			break;

		case BUTTON_VOLUME_DOWN:
			ret = bt_r_and_c_volume_down();
			if (ret) {
				LOG_WRN("Failed to decrease volume: %d", ret);
			}

			break;

		case BUTTON_4:
			if (IS_ENABLED(CONFIG_AUDIO_TEST_TONE)) {
				if (IS_ENABLED(CONFIG_WALKIE_TALKIE_DEMO)) {
					LOG_DBG("Test tone is disabled in walkie-talkie mode");
					break;
				}

				if (strm_state != STATE_STREAMING) {
					LOG_WRN("Not in streaming state");
					break;
				}

				ret = audio_system_encode_test_tone_step();
				if (ret) {
					LOG_WRN("Failed to play test tone, ret: %d", ret);
				}

				break;
			}

			break;

		case BUTTON_5:
			if (IS_ENABLED(CONFIG_AUDIO_MUTE)) {
				ret = bt_r_and_c_volume_mute(false);
				if (ret) {
					LOG_WRN("Failed to mute, ret: %d", ret);
				}

				break;
			}

			break;

		default:
			LOG_WRN("Unexpected/unhandled button id: %d", msg.button_pin);
		}

		STACK_USAGE_PRINT("button_msg_thread", &button_msg_sub_thread_data);
	}
}

/**
 * @brief	Handle Bluetooth LE audio events.
 */
static void le_audio_msg_sub_thread(void)
{
	int ret;
	uint32_t pres_delay_us;
	uint32_t bitrate_bps;
	uint32_t sampling_rate_hz;
	const struct zbus_channel *chan;

	while (1) {
		struct le_audio_msg msg;

		ret = zbus_sub_wait_msg(&le_audio_evt_sub, &chan, &msg, K_FOREVER);
		ERR_CHK(ret);

		LOG_DBG("Received event = %d, current state = %d", msg.event, strm_state);

		switch (msg.event) {
		case LE_AUDIO_EVT_STREAMING:
			LOG_DBG("LE audio evt streaming");

			if (msg.dir == BT_AUDIO_DIR_SOURCE) {
				audio_system_encoder_start();
			}

			if (strm_state == STATE_STREAMING) {
				LOG_DBG("Got streaming event in streaming state");
				break;
			}

			audio_system_start();
			stream_state_set(STATE_STREAMING);
			//ret = led_blink(LED_APP_1_BLUE);
			ERR_CHK(ret);

			break;

		case LE_AUDIO_EVT_NOT_STREAMING:
			LOG_DBG("LE audio evt not streaming");

			if (msg.dir == BT_AUDIO_DIR_SOURCE) {
				audio_system_encoder_stop();
			}

			if (strm_state == STATE_PAUSED) {
				LOG_DBG("Got not_streaming event in paused state");
				break;
			}

			stream_state_set(STATE_PAUSED);
			audio_system_stop();
			//ret = led_on(LED_APP_1_BLUE);
			ERR_CHK(ret);

			break;

		case LE_AUDIO_EVT_CONFIG_RECEIVED:
			LOG_DBG("LE audio config received");

			ret = unicast_server_config_get(msg.conn, msg.dir, &bitrate_bps,
							&sampling_rate_hz, NULL);
			if (ret) {
				LOG_WRN("Failed to get config: %d", ret);
				break;
			}

			LOG_DBG("\tSampling rate: %d Hz", sampling_rate_hz);
			LOG_DBG("\tBitrate (compressed): %d bps", bitrate_bps);

			if (msg.dir == BT_AUDIO_DIR_SINK) {
				ret = audio_system_config_set(VALUE_NOT_SET, VALUE_NOT_SET,
							      sampling_rate_hz);
				ERR_CHK(ret);
			} else if (msg.dir == BT_AUDIO_DIR_SOURCE) {
				ret = audio_system_config_set(sampling_rate_hz, bitrate_bps,
							      VALUE_NOT_SET);
				ERR_CHK(ret);
			}

			break;

		case LE_AUDIO_EVT_PRES_DELAY_SET:
			LOG_DBG("Set presentation delay");

			ret = unicast_server_config_get(msg.conn, BT_AUDIO_DIR_SINK, NULL, NULL,
							&pres_delay_us);
			if (ret) {
				LOG_ERR("Failed to get config: %d", ret);
				break;
			}

			ret = audio_datapath_pres_delay_us_set(pres_delay_us);
			if (ret) {
				LOG_ERR("Failed to set presentation delay to %d", pres_delay_us);
				break;
			}

			LOG_INF("Presentation delay %d us is set by initiator", pres_delay_us);

			break;

		case LE_AUDIO_EVT_NO_VALID_CFG:
			LOG_WRN("No valid configurations found, will disconnect");

			ret = bt_mgmt_conn_disconnect(msg.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			if (ret) {
				LOG_ERR("Failed to disconnect: %d", ret);
			}

			break;

		default:
			LOG_WRN("Unexpected/unhandled le_audio event: %d", msg.event);

			break;
		}

		STACK_USAGE_PRINT("le_audio_msg_thread", &le_audio_msg_sub_thread_data);
	}
}

/**
 * @brief	Create zbus subscriber threads.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_subscribers_create(void)
{
	int ret;

	button_msg_sub_thread_id = k_thread_create(
		&button_msg_sub_thread_data, button_msg_sub_thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)button_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(button_msg_sub_thread_id, "BUTTON_MSG_SUB");
	if (ret) {
		LOG_ERR("Failed to create button_msg thread");
		return ret;
	}

	le_audio_msg_sub_thread_id = k_thread_create(
		&le_audio_msg_sub_thread_data, le_audio_msg_sub_thread_stack,
		CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE, (k_thread_entry_t)le_audio_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_LE_AUDIO_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(le_audio_msg_sub_thread_id, "LE_AUDIO_MSG_SUB");
	if (ret) {
		LOG_ERR("Failed to create le_audio_msg thread");
		return ret;
	}

	return 0;
}

/**
 * @brief	Zbus listener to receive events from bt_mgmt.
 *
 * @param[in]	chan	Zbus channel.
 *
 * @note	Will in most cases be called from BT_RX context,
 *		so there should not be too much processing done here.
 */
static void bt_mgmt_evt_handler(const struct zbus_channel *chan)
{
	int ret;
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
		LOG_INF("Connected");

		break;

	case BT_MGMT_DISCONNECTED:
		LOG_INF("Disconnected");

		ret = bt_content_ctrl_conn_disconnected(msg->conn);
		if (ret) {
			LOG_ERR("Failed to handle disconnection in content control: %d", ret);
		}

		break;

	case BT_MGMT_SECURITY_CHANGED:
		LOG_INF("Security changed");

		ret = bt_r_and_c_discover(msg->conn);
		if (ret) {
			LOG_WRN("Failed to discover rendering services");
		}

		ret = bt_content_ctrl_discover(msg->conn);
		if (ret == -EALREADY) {
			LOG_DBG("Discovery in progress or already done");
		} else if (ret) {
			LOG_ERR("Failed to start discovery of content control: %d", ret);
		}

		break;

	default:
		LOG_WRN("Unexpected/unhandled bt_mgmt event: %d", msg->event);

		break;
	}
}

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen, bt_mgmt_evt_handler);

/**
 * @brief	Link zbus producers and observers.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_link_producers_observers(void)
{
	int ret;

	if (!IS_ENABLED(CONFIG_ZBUS)) {
		return -ENOTSUP;
	}

	ret = zbus_chan_add_obs(&button_chan, &button_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add button sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&le_audio_chan, &le_audio_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add le_audio sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add bt_mgmt sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&volume_chan, &volume_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add volume sub");
		return ret;
	}

	return 0;
}

static int ext_adv_populate(struct bt_data *ext_adv_buf, size_t ext_adv_buf_size,
			    size_t *ext_adv_count)
{
	int ret;
	size_t ext_adv_buf_cnt = 0;

	NET_BUF_SIMPLE_DEFINE_STATIC(uuid_buf, CONFIG_EXT_ADV_UUID_BUF_MAX);

	ext_adv_buf[ext_adv_buf_cnt].type = BT_DATA_UUID16_SOME;
	ext_adv_buf[ext_adv_buf_cnt].data_len = 0;
	ext_adv_buf[ext_adv_buf_cnt].data = uuid_buf.data;
	ext_adv_buf_cnt++;

	ret = bt_r_and_c_uuid_populate(&uuid_buf);

	if (ret) {
		LOG_ERR("Failed to add adv data from renderer: %d", ret);
		return ret;
	}

	ret = bt_content_ctrl_uuid_populate(&uuid_buf);

	if (ret) {
		LOG_ERR("Failed to add adv data from content ctrl: %d", ret);
		return ret;
	}

	ret = bt_mgmt_manufacturer_uuid_populate(&uuid_buf, CONFIG_BT_DEVICE_MANUFACTURER_ID);
	if (ret) {
		LOG_ERR("Failed to add adv data with manufacturer ID: %d", ret);
		return ret;
	}

	ret = unicast_server_adv_populate(&ext_adv_buf[ext_adv_buf_cnt],
					  ext_adv_buf_size - ext_adv_buf_cnt);

	if (ret < 0) {
		LOG_ERR("Failed to add adv data from unicast server: %d", ret);
		return ret;
	}

	ext_adv_buf_cnt += ret;

	/* Add the number of UUIDs */
	ext_adv_buf[0].data_len = uuid_buf.len;

	LOG_DBG("Size of adv data: %d, num_elements: %d", sizeof(struct bt_data) * ext_adv_buf_cnt,
		ext_adv_buf_cnt);

	*ext_adv_count = ext_adv_buf_cnt;

	return 0;
}

uint8_t stream_state_get(void)
{
	return strm_state;
}

void streamctrl_send(void const *const data, size_t size, uint8_t num_ch)
{
	int ret;
	static int prev_ret;

	struct le_audio_encoded_audio enc_audio = {.data = data, .size = size, .num_ch = num_ch};

	if (strm_state == STATE_STREAMING) {
		ret = unicast_server_send(enc_audio);

		if (ret != 0 && ret != prev_ret) {
			if (ret == -ECANCELED) {
				LOG_WRN("Sending operation cancelled");
			} else {
				LOG_WRN("Problem with sending LE audio data, ret: %d", ret);
			}
		}

		prev_ret = ret;
	}
}

/*
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	uint8_t base_addr[6] = { 0xb2, 0xfc, 0x69, 0x22, 0x91, 0xb0 };
	char addr_str[BT_ADDR_LE_STR_LEN] = {0};
	//int err;

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	if (!memcmp(&addr->a, base_addr, 6)) {
		if (type == 0) {
			LOG_INF("Device found: %s (RSSI %d), %s\n", addr_str, rssi, "ADV_INT");
		} if (type == 4) {
			LOG_INF("Device found: %s (RSSI %d), %s\n", addr_str, rssi, "SCAN_RSP");
		}
	}
}*/

bool device_paired = false;

// #include "csip_crypto.h"

#include <zephyr/bluetooth/audio/csip.h>

#define BT_CSIP_CRYPTO_KEY_SIZE   16
#define BT_CSIP_CRYPTO_SALT_SIZE  16
#define BT_CSIP_CRYPTO_PRAND_SIZE 3
#define BT_CSIP_CRYPTO_HASH_SIZE  3

#define BT_CSIP_CRYPTO_PADDING_SIZE 13
#define BT_CSIP_PADDED_RAND_SIZE    (BT_CSIP_CRYPTO_PADDING_SIZE + BT_CSIP_CRYPTO_PRAND_SIZE)
#define BT_CSIP_R_MASK              BIT_MASK(24) /* r is 24 bit / 3 octet */

// Callback-Funktion für gefundene Geräte
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    int ret;
	bool is_le_audio_device = false;
	uint8_t csis_rsi[6];
	uint8_t chip_id[8];

    while (ad->len > 0) {
        uint8_t len = net_buf_simple_pull_u8(ad);
        if (len == 0 || len > ad->len) {
            break; // Ungültige Länge
        }

        uint8_t type = net_buf_simple_pull_u8(ad);
        const uint8_t *data = ad->data;
        ad->data += len - 1;
        ad->len -= len - 1;

		//printk("test\n");

        // Suchen nach 16-bit Service UUIDs (LE Audio Services)
        if (type == BT_DATA_SVC_DATA16) { //BT_DATA_UUID16_SOME || type == BT_DATA_UUID16_ALL) {
			//printk("LE Audio-Gerät gefunden! UUID\n");
			//printk("type 0x%04X ", type);
            for (size_t i = 0; i < len - 1; i += 2) {
                uint16_t uuid = (data[i + 1] << 8) | data[i];
				if (uuid == BT_UUID_CAS_VAL) {
					is_le_audio_device = true;
                    //printk("LE Audio-Gerät gefunden! UUID: 0x%04X\n", uuid);
                }
            }
			//printk("\n");
        }

		if (is_le_audio_device && type == BT_DATA_MANUFACTURER_DATA) {
			memset(chip_id, 0, sizeof(chip_id));
			memcpy(chip_id, data, sizeof(chip_id));
        }

		if (type == BT_DATA_CSIS_RSI) {
			memcpy(csis_rsi, data, sizeof(csis_rsi));
		}

		// channel
		// device_id = 0
		// if (channel == 0x1 - this.channel)
		// setSIRK(device_id - 0)
    }

	if (is_le_audio_device) {
		LOG_INF("LE Audio-Gerät gefunden! UUID\n");

		// printk("MANUFACTURER DATA 0x%s\n", bt_hex(chip_id, sizeof(chip_id)));

		// printk("CSIS DATA 0x%s\n", bt_hex(csis_rsi, sizeof(csis_rsi)));

		uint32_t hash_ref = (csis_rsi[2] << 16) | (csis_rsi[1] << 8) | csis_rsi[0];
		uint32_t hash;

		if(!device_paired) {
			uint32_t new_sirk = *((uint32_t *) chip_id) ^ oe_boot_state.device_id;

			enum audio_channel channel;

			//backup channel
			channel_assignment_get(&channel);

			if (channel == AUDIO_CH_L) {

				LOG_INF("Device ID 1: %016X", oe_boot_state.device_id);
				LOG_INF("Device ID 2: %016X", *((uint32_t *) chip_id));
				LOG_INF("New Sirk: %016X", new_sirk);

				//TODO: check if the device wants to pair (sirk == device_id)
				//TODO: check channel

				//ret = uicr_sirk_set(new_sirk);

				bt_le_scan_stop();

				ret = uicr_sirk_set(new_sirk);

				device_paired = true;

				if (ret == 0) sys_reboot(SYS_REBOOT_COLD);
				else LOG_ERR("UICR writing error: %i", ret);
			} else if (channel == AUDIO_CH_R) {
				uint8_t res[BT_CSIP_PADDED_RAND_SIZE];

				uint8_t sirk[BT_CSIP_SET_SIRK_SIZE + 1];
				uint8_t r[BT_CSIP_CRYPTO_PRAND_SIZE];
				uint8_t out[BT_CSIP_CRYPTO_HASH_SIZE];

				for (size_t i = 0; i < 3; i++) {
					//hash[i] = csis_rsi[i];
					r[i] = csis_rsi[i+3];
				}

				// memcpy(new_sirk, )

				if ((r[BT_CSIP_CRYPTO_PRAND_SIZE - 1] & BIT(7)) ||
				((r[BT_CSIP_CRYPTO_PRAND_SIZE - 1] & BIT(6)) == 0)) {
					LOG_WRN("Invalid r %s", bt_hex(r, BT_CSIP_CRYPTO_PRAND_SIZE));
				}

				// r' = padding || r
				(void)memset(res + BT_CSIP_CRYPTO_PRAND_SIZE, 0, BT_CSIP_CRYPTO_PADDING_SIZE);
				memcpy(res, r, BT_CSIP_CRYPTO_PRAND_SIZE);

				memset(sirk, 0, BT_CSIP_CRYPTO_KEY_SIZE + 1);
				snprintf(sirk, BT_CSIP_SET_SIRK_SIZE, "%08X", new_sirk);

				//LOG_INF("New Sirk: %016X", new_sirk);
				LOG_INF("SIRK as String: %s", sirk);

				//LOG_INF("sirk %s", bt_hex(sirk, BT_CSIP_SET_SIRK_SIZE));
				//LOG_INF("r' %s", bt_hex(res, sizeof(res)));

				int err = bt_encrypt_le(sirk, res, res);

				memcpy(out, res, BT_CSIP_CRYPTO_HASH_SIZE);

				// LOG_INF("prand: 0x%s", bt_hex(r, BT_CSIP_CRYPTO_PRAND_SIZE));
				// LOG_INF("hash to match: 0x%s", bt_hex(&hash, BT_CSIP_CRYPTO_HASH_SIZE)); //[0] << 16 | hash[1] << 8 | hash[2]);
				// LOG_INF("hash result: 0x%s", bt_hex(out, BT_CSIP_CRYPTO_HASH_SIZE));

				hash = out[2] << 16 | out[1] << 8 | out[0];

				// LOG_INF("h1 0x%08X h2 0x%08X", hash, hash2);

				if (hash_ref == hash) {
					// LOG_INF("Device ID 1: %016X", oe_boot_state.device_id);
					// LOG_INF("Device ID 2: %016X", *((uint32_t *) chip_id));
					// LOG_INF("New Sirk: %016X", new_sirk);

					//uicr_sirk_set(new_sirk);

					bt_le_scan_stop();

					ret = uicr_sirk_set(new_sirk);

					device_paired = true;

					if (ret == 0) sys_reboot(SYS_REBOOT_COLD);
					else LOG_ERR("UICR writing error: %i", ret);
				}
			}
		}
	}
}

int streamctrl_start(void)
{
	int ret;
	static struct bt_data ext_adv_buf[CONFIG_EXT_ADV_BUF_MAX];

	LOG_DBG("nRF5340 APP core started");

	size_t ext_adv_buf_cnt = 0;

	ret = nrf5340_audio_dk_init();
	ERR_CHK(ret);

	ret = nrf5340_audio_common_init();
	ERR_CHK(ret);

	ret = zbus_subscribers_create();
	ERR_CHK_MSG(ret, "Failed to create zbus subscriber threads");

	ret = zbus_link_producers_observers();
	ERR_CHK_MSG(ret, "Failed to link zbus producers and observers");

	ret = le_audio_rx_init();
	ERR_CHK_MSG(ret, "Failed to initialize rx path");

	ret = unicast_server_enable(le_audio_rx_data_handler);
	ERR_CHK_MSG(ret, "Failed to enable LE Audio");

	ret = bt_r_and_c_init();
	ERR_CHK(ret);

	ret = bt_content_ctrl_init();
	ERR_CHK(ret);

	ret = ext_adv_populate(ext_adv_buf, ARRAY_SIZE(ext_adv_buf), &ext_adv_buf_cnt);
	ERR_CHK(ret);

	ret = bt_mgmt_adv_start(ext_adv_buf, ext_adv_buf_cnt, NULL, 0, true);
	ERR_CHK(ret);

	uint32_t sirk = uicr_sirk_get();

	if (sirk == 0xFFFFFFFF) {
		struct bt_le_scan_param  * scan_param = BT_LE_SCAN_PARAM(NRF5340_AUDIO_GATEWAY_SCAN_TYPE, BT_LE_SCAN_OPT_FILTER_DUPLICATE, 32, 32);
		
		int err = bt_le_scan_start(scan_param, device_found);
		if (err) LOG_ERR("Scanning failed to start (err %d)", err);
		else LOG_INF("Scanning successfully started");
	}

	return 0;
}
