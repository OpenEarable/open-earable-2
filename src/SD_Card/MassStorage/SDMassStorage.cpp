#include "SDMassStorage.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/disk_access.h>

#include <cerrno>
#include <cstring>

#include "openearable_common.h"

LOG_MODULE_REGISTER(sd_mass_storage, LOG_LEVEL_INF);

namespace {

constexpr char disk_name[] = "SD";
const k_timeout_t debounce_time = K_MSEC(100);

const gpio_dt_spec sd_detect =
	GPIO_DT_SPEC_GET(DT_NODELABEL(sd_state), gpios);
gpio_callback sd_detect_callback;

void refresh_media(k_work *)
{
	const bool inserted = gpio_pin_get_dt(&sd_detect) > 0;

	if (inserted) {
		const int ret = disk_access_init(disk_name);
		if (ret) {
			LOG_WRN("SD card detected but initialization failed: %d", ret);
		} else {
			LOG_INF("SD card is available to USB mass storage");
		}
	} else {
		const int ret =
			disk_access_ioctl(disk_name, DISK_IOCTL_CTRL_DEINIT, nullptr);
		if (ret && ret != -ENOTSUP && ret != -EINVAL) {
			LOG_WRN("SD card deinitialization failed: %d", ret);
		}
		LOG_INF("No SD card; USB remains available without media");
	}
}

K_WORK_DELAYABLE_DEFINE(media_work, refresh_media);

void sd_detect_changed(const struct device *, struct gpio_callback *, uint32_t)
{
	k_work_reschedule(&media_work, debounce_time);
}

int set_sd_power(bool enabled)
{
	int ret;

	if (!enabled) {
		ret = disk_access_ioctl(disk_name, DISK_IOCTL_CTRL_DEINIT, nullptr);
		if (ret && ret != -ENOTSUP && ret != -EINVAL) {
			LOG_WRN("SD card deinitialization before power-off failed: %d",
				ret);
		}
		ret = pm_device_action_run(ls_sd, PM_DEVICE_ACTION_SUSPEND);
		LOG_INF("SD load switch disabled: %d", ret);
		return ret;
	}

	ret = pm_device_action_run(ls_sd, PM_DEVICE_ACTION_RESUME);
	if (ret) {
		LOG_ERR("Failed to enable SD load switch: %d", ret);
		return ret;
	}

	k_msleep(10);
	ret = disk_access_init(disk_name);
	LOG_INF("SD load switch enabled, disk initialization: %d", ret);
	return ret;
}

int power_command(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "Usage: sd_msc power <on|off>");
		return -EINVAL;
	}

	if (!strcmp(argv[1], "on")) {
		return set_sd_power(true);
	}
	if (!strcmp(argv[1], "off")) {
		return set_sd_power(false);
	}

	shell_error(shell, "Expected 'on' or 'off'");
	return -EINVAL;
}

int status_command(const struct shell *shell, size_t, char **)
{
	shell_print(shell, "inserted=%d disk_status=%d",
		    gpio_pin_get_dt(&sd_detect) > 0,
		    disk_access_status(disk_name));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sd_msc_commands,
	SHELL_CMD_ARG(power, NULL, "Set SD load switch: power <on|off>",
		      power_command, 2, 0),
	SHELL_CMD(status, NULL, "Show card-detect and disk status",
		  status_command),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sd_msc, &sd_msc_commands,
		   "USB SD mass-storage control", NULL);

} // namespace

int sd_mass_storage_init()
{
	if (!gpio_is_ready_dt(&sd_detect)) {
		LOG_ERR("SD card-detect GPIO is not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&sd_detect, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&sd_detect, GPIO_INT_EDGE_BOTH);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&sd_detect_callback, sd_detect_changed,
			   BIT(sd_detect.pin));
	ret = gpio_add_callback(sd_detect.port, &sd_detect_callback);
	if (ret) {
		return ret;
	}

	refresh_media(nullptr);
	return 0;
}
