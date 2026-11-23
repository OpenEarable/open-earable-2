/* SPDX-License-Identifier: Apache-2.0 */
#include "earable_pairing.h"
#include "BootState.h"
#include "channel_assignment.h"
#include "uicr.h"

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(earable_pairing, CONFIG_MAIN_LOG_LEVEL);

#define PAIR_SETTING "earable/pair"

/* One NVS record keeps mode, side and identity consistent across power loss. */
struct pair_setting {
	uint8_t mode;
	uint8_t side;
	uint8_t id[4];
};

static struct pair_setting active;
static atomic_t busy;

int earable_pairing_load(void)
{
	int err = settings_subsys_init();
	if (err) {
		return err;
	}
	struct pair_setting stored;
	ssize_t len = settings_load_one(PAIR_SETTING, &stored, sizeof(stored));
	if (len == 0 || len == -ENOENT) {
		active.mode = EARABLE_PROVISIONED;
		/* Preserve first-boot provisioning for a freshly flashed standalone. */
		if (uicr_standalone_get() == 0 && uicr_sirk_get() == UINT32_MAX) {
			err = uicr_sirk_set((uint32_t)oe_boot_state.device_id);
			if (!err) {
				sys_reboot(SYS_REBOOT_COLD);
			}
			return err;
		}
		return 0;
	}
	if (len < 0) {
		return (int)len;
	}
	if (len != sizeof(stored) || stored.mode < EARABLE_SEEKING ||
	    stored.mode > EARABLE_PAIRED || stored.side > AUDIO_CH_R ||
	    (stored.mode == EARABLE_PAIRED && sys_get_le32(stored.id) == UINT32_MAX)) {
		LOG_WRN("Ignoring invalid earable pairing setting");
		active.mode = EARABLE_PROVISIONED;
		return 0;
	}
	active = stored;
	return 0;
}

int earable_pairing_side_override(void)
{
	return active.mode == EARABLE_PROVISIONED ? -1 : active.side;
}

bool earable_pairing_is_seeking(void)
{
	return active.mode == EARABLE_SEEKING ||
	       (active.mode == EARABLE_PROVISIONED && uicr_sirk_get() == UINT32_MAX &&
		uicr_standalone_get() != 0);
}

uint32_t earable_pairing_identity(void)
{
	if (active.mode == EARABLE_PAIRED) {
		return sys_get_le32(active.id);
	}
	if (active.mode == EARABLE_PROVISIONED && uicr_sirk_get() != UINT32_MAX) {
		return uicr_sirk_get();
	}
	return (uint32_t)oe_boot_state.device_id;
}

bool earable_pairing_claim(void)
{
	return atomic_cas(&busy, 0, 1);
}

void earable_pairing_release(void)
{
	atomic_clear(&busy);
}

static int store(enum earable_pair_mode mode, uint8_t side, uint32_t pair_id)
{
	struct pair_setting setting = { .mode = mode, .side = side };
	sys_put_le32(pair_id, setting.id);
	return settings_save_one(PAIR_SETTING, &setting, sizeof(setting));
}

uint32_t earable_pairing_previous_identity(void)
{
	return active.mode == EARABLE_PROVISIONED ? uicr_sirk_get() : sys_get_le32(active.id);
}

/* The previous partner has shown up with its individual identity again.
 * Allow its eventual confirmation to survive a power cut during the offer. */
int earable_pairing_allow_peer(uint32_t id)
{
	if (active.mode != EARABLE_SEEKING || id != sys_get_le32(active.id)) {
		return 0;
	}
	if (!earable_pairing_claim()) {
		return -EBUSY;
	}
	int err = store(EARABLE_SEEKING, active.side, UINT32_MAX);
	if (!err) {
		sys_put_le32(UINT32_MAX, active.id);
	}
	earable_pairing_release();
	return err;
}

int earable_pairing_set_mode(enum earable_pair_mode mode, uint8_t side)
{
	if (mode == EARABLE_PROVISIONED) {
		return settings_delete(PAIR_SETTING);
	}
	return store(mode, side, earable_pairing_previous_identity());
}

int earable_pairing_bond(uint32_t id, uint8_t side)
{
	if (active.mode == EARABLE_PROVISIONED) {
		return uicr_sirk_set(id);
	}
	return store(EARABLE_PAIRED, side, id);
}
