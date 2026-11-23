/* SPDX-License-Identifier: Apache-2.0 */
#include "earable_pairing.h"
#include "BootState.h"
#include "channel_assignment.h"
#include "csip_identity.h"
#include "audio/csip_crypto.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/audio/csip.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(earable_discovery, CONFIG_MAIN_LOG_LEVEL);

struct peer {
	uint64_t chip;
	uint8_t side;
	uint8_t rsi[BT_CSIP_RSI_SIZE];
	bool cas;
	bool has_side;
	bool has_rsi;
	bool seeking;
};

/* Byte 9 is zero in normal and legacy advertisements. Only the discovery
 * set marks itself as actively seeking. The legacy ID and side stay in place. */
#define SEEKING_MARKER 0x50
#define OFFER_WINDOW_MS 5000

/* The phone can reconnect and stop connectable advertising during discovery.
 * Keep this small, non-connectable set on air until the pairing reboot. */
static enum audio_channel side;
static struct bt_le_ext_adv *beacon;
static uint8_t beacon_identifier[11];
static uint8_t beacon_rsi[BT_CSIP_RSI_SIZE];
static struct bt_data beacon_data[] = {
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_CAS_VAL), 0),
	BT_DATA(BT_DATA_CSIS_RSI, beacon_rsi, sizeof(beacon_rsi)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, beacon_identifier, sizeof(beacon_identifier)),
};

static int identity_hash(uint32_t id, const uint8_t rsi[BT_CSIP_RSI_SIZE], uint8_t hash[3])
{
	uint8_t key[BT_CSIP_SIRK_SIZE], random[3];
	openearable_csip_identity_encode(id, oe_boot_state.device_id, key);
	memcpy(random, &rsi[3], sizeof(random));
	return bt_csip_sih(key, random, hash);
}

static int advertise_identity(uint32_t id)
{
	int err = identity_hash(id, beacon_rsi, beacon_rsi);
	if (err) {
		return err;
	}
	return bt_le_ext_adv_set_data(beacon, beacon_data, ARRAY_SIZE(beacon_data), NULL, 0);
}

static bool rsi_matches(uint32_t id, const uint8_t rsi[BT_CSIP_RSI_SIZE])
{
	uint8_t hash[3];
	return identity_hash(id, rsi, hash) == 0 && memcmp(hash, rsi, sizeof(hash)) == 0;
}

static void reboot_paired(struct k_work *work)
{
	earable_pairing_reboot();
}

static K_WORK_DELAYABLE_DEFINE(pair_reboot, reboot_paired);

static void consider_peer(const struct peer *peer)
{
	static uint64_t offered_chip;
	static int64_t offer_until;
	uint32_t chip = (uint32_t)peer->chip;
	uint32_t id = chip ^ (uint32_t)oe_boot_state.device_id;

	/* These values cannot distinguish a shared identity from an erased word
	 * or one member's individual identity on legacy firmware. */
	if (id == UINT32_MAX || id == chip || id == (uint32_t)oe_boot_state.device_id) {
		return;
	}

	/* A reset must not immediately rejoin the previous, idle partner. An
	 * older left can show intent by switching from its own RSI to ours. */
	bool participating = peer->seeking || peer->chip == offered_chip;
	if ((participating || id != earable_pairing_previous_identity()) &&
	    rsi_matches(id, peer->rsi)) {
		if (!earable_pairing_claim()) {
			return;
		}
		int err = earable_pairing_bond(id, side);
		if (err) {
			LOG_ERR("Could not save earable pair: %d", err);
			earable_pairing_release();
			return;
		}
		LOG_INF("Saved earable pair %08x", (unsigned int)id);
		if (side == AUDIO_CH_R) {
			/* Acknowledge on air before a bonded phone can reconnect on reboot. */
			err = advertise_identity(id);
			if (err) {
				LOG_WRN("Could not advertise pair confirmation: %d", err);
			}
			k_work_schedule(&pair_reboot, K_SECONDS(2));
		} else {
			earable_pairing_reboot();
		}
		return;
	}

	/* A legacy standalone right and a fresh right advertise the same type of
	 * individual RSI. Offer the derived identity temporarily. Only a seeking
	 * right answers with its shared RSI. An unanswered offer stays in RAM. */
	if (!rsi_matches(chip, peer->rsi)) {
		return;
	}
	if (side == AUDIO_CH_L && (offered_chip == peer->chip || k_uptime_get() < offer_until)) {
		return;
	}
	int err = earable_pairing_allow_peer(id);
	if (err || side == AUDIO_CH_R) {
		return;
	}
	err = advertise_identity(id);
	if (err) {
		LOG_WRN("Could not advertise pair offer: %d", err);
		return;
	}
	offered_chip = peer->chip;
	offer_until = k_uptime_get() + OFFER_WINDOW_MS;
}

K_MSGQ_DEFINE(peer_reports, sizeof(struct peer), 4, 1);

static void process_reports(struct k_work *work)
{
	struct peer peer;
	while (k_msgq_get(&peer_reports, &peer, K_NO_WAIT) == 0) {
		consider_peer(&peer);
	}
}

static K_WORK_DEFINE(process_peers, process_reports);

static bool parse_peer(struct bt_data *field, void *user_data)
{
	struct peer *peer = user_data;
	switch (field->type) {
	case BT_DATA_SVC_DATA16:
		if (field->data_len >= 2 && sys_get_le16(field->data) == BT_UUID_CAS_VAL) {
			peer->cas = true;
		}
		break;
	case BT_DATA_MANUFACTURER_DATA:
		if (field->data_len >= 9) {
			peer->chip = sys_get_le64(field->data);
			peer->side = field->data[8];
			peer->has_side = true;
			peer->seeking = field->data_len >= 10 && field->data[9] == SEEKING_MARKER;
		}
		break;
	case BT_DATA_CSIS_RSI:
		if (field->data_len == sizeof(peer->rsi) && (field->data[5] & 0xc0) == 0x40) {
			memcpy(peer->rsi, field->data, sizeof(peer->rsi));
			peer->has_rsi = true;
		}
		break;
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct peer peer = {0};
	bt_data_parse(ad, parse_peer, &peer);
	if (peer.cas && peer.has_side && peer.has_rsi && peer.chip != oe_boot_state.device_id &&
	    peer.side == (side == AUDIO_CH_L ? AUDIO_CH_R : AUDIO_CH_L) &&
	    k_msgq_put(&peer_reports, &peer, K_NO_WAIT) == 0) {
		k_work_submit(&process_peers);
	}
}

int earable_pairing_start_discovery(const struct bt_data *adv, size_t count)
{
	channel_assignment_get(&side);
	bool have_rsi = false;
	for (size_t i = 0; i < count; ++i) {
		if (adv[i].type == BT_DATA_CSIS_RSI && adv[i].data_len == sizeof(beacon_rsi)) {
			memcpy(beacon_rsi, adv[i].data, sizeof(beacon_rsi));
			have_rsi = true;
		}
		if (adv[i].type == BT_DATA_MANUFACTURER_DATA &&
		    adv[i].data_len == sizeof(beacon_identifier)) {
			memcpy(beacon_identifier, adv[i].data, sizeof(beacon_identifier));
			beacon_identifier[9] = SEEKING_MARKER;
		}
	}
	if (!have_rsi || beacon_identifier[9] != SEEKING_MARKER) {
		return -EINVAL;
	}

	struct bt_le_adv_param param = *BT_LE_EXT_ADV_NCONN;
	param.sid = CONFIG_BLE_ACL_ADV_SID ^ 1;
	int err = bt_le_ext_adv_create(&param, NULL, &beacon);
	if (err) {
		return err;
	}
	err = bt_le_ext_adv_set_data(beacon, beacon_data, ARRAY_SIZE(beacon_data), NULL, 0);
	if (!err) {
		err = bt_le_ext_adv_start(beacon, BT_LE_EXT_ADV_START_DEFAULT);
	}
	if (!err) {
		/* Repeated reports carry offers and confirmations as their RSI changes. */
		err = bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE,
				       BT_LE_SCAN_OPT_NONE, 32, 32), device_found);
	}
	if (err) {
		bt_le_ext_adv_stop(beacon);
		bt_le_ext_adv_delete(beacon);
	}
	return err;
}
