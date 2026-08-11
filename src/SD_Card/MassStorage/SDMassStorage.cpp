#include "SDMassStorage.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/disk_access.h>

#include <cerrno>

#include "openearable_common.h"
#include "SD_Card_Manager.h"

LOG_MODULE_REGISTER(sd_mass_storage, LOG_LEVEL_INF);

namespace {

constexpr char disk_name[] = "SD";

const gpio_dt_spec sd_detect =
	GPIO_DT_SPEC_GET(DT_NODELABEL(sd_state), gpios);

/*
 * Whether this module holds a disk_access reference. disk_access counts
 * references, so init and deinit have to be paired from here as well.
 */
bool media_initialized;

int init_media_disk()
{
	if (media_initialized) {
		return 0;
	}

	const int ret = disk_access_init(disk_name);
	if (!ret) {
		media_initialized = true;
	}
	return ret;
}

/*
 * Drops the reference and powers the card down in one step. Forced because the
 * mass-storage LUN must let go of media that is physically gone, whatever the
 * remaining reference count says.
 */
int force_disk_deinit()
{
	if (!media_initialized) {
		return 0;
	}

	bool force = true;
	const int ret =
		disk_access_ioctl(disk_name, DISK_IOCTL_CTRL_DEINIT, &force);
	media_initialized = false;
	return ret;
}

int status_command(const struct shell *shell, size_t, char **)
{
	shell_print(shell, "inserted=%d disk_status=%d mounted=%d",
		    gpio_pin_get_dt(&sd_detect) > 0,
		    disk_access_status(disk_name),
		    sdcard_manager.is_mounted());
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sd_msc_commands,
	SHELL_CMD(status, NULL, "Show card-detect and disk status",
		  status_command),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sd_msc, &sd_msc_commands,
		   "USB SD mass-storage control", NULL);

} // namespace

void sd_mass_storage_handle_card_change(bool inserted)
{
	if (inserted) {
		const int ret = init_media_disk();
		if (ret) {
			LOG_WRN("SD card detected but initialization failed: %d", ret);
			return;
		}

		LOG_INF("SD card is available to USB mass storage");
		return;
	}

	const int ret = force_disk_deinit();
	if (ret && ret != -ENOTSUP && ret != -EINVAL) {
		LOG_WRN("SD card deinitialization failed: %d", ret);
	}
	LOG_INF("No SD card; USB remains available without media");
}

int sd_mass_storage_init()
{
	if (!gpio_is_ready_dt(&sd_detect)) {
		LOG_ERR("SD card-detect GPIO is not ready");
		return -ENODEV;
	}

	const int ret = gpio_pin_configure_dt(&sd_detect, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	sd_mass_storage_handle_card_change(gpio_pin_get_dt(&sd_detect) > 0);
	return 0;
}
