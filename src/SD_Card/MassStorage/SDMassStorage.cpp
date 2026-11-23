#include "SDMassStorage.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbd_msg.h>

#include <cerrno>

#include "openearable_common.h"
#include "SD_Card_Manager.h"
#include "SDLogger.h"

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
atomic_t host_active;
atomic_t usb_blocked_by_recording;
struct usbd_context *usb_context;

void remember_usb_context(struct usbd_context *ctx)
{
	if (ctx != nullptr) {
		usb_context = ctx;
	}
}

bool usb_host_configured()
{
	return (atomic_get(&host_active) != 0) ||
	       (usb_context != nullptr &&
		usb_context->ch9_data.state == USBD_STATE_CONFIGURED);
}

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
	shell_print(shell, "inserted=%d disk_status=%d mounted=%d host_active=%d usb_blocked=%d",
		    gpio_pin_get_dt(&sd_detect) > 0,
		    disk_access_status(disk_name),
		    sdcard_manager.is_mounted(),
		    usb_host_configured(),
		    atomic_get(&usb_blocked_by_recording) != 0);
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

void sd_mass_storage_usb_msg_cb(struct usbd_context *const ctx,
				const struct usbd_msg *const msg)
{
	remember_usb_context(ctx);

	switch (msg->type) {
	case USBD_MSG_CONFIGURATION:
		if (msg->status == 0) {
			atomic_clear(&host_active);
			break;
		}

		if (sdlogger.is_active()) {
			atomic_clear(&host_active);
			atomic_set(&usb_blocked_by_recording, 1);
			LOG_WRN("USB mass storage disabled while SD recording is active");

			if (ctx != nullptr) {
				const int ret = usbd_disable(ctx);
				if (ret && ret != -EALREADY) {
					LOG_WRN("Failed to disable USB mass storage: %d", ret);
				}
			}
			break;
		}

		if (sdcard_manager.is_mounted()) {
			const int ret = sdcard_manager.unmount();
			if (ret) {
				LOG_ERR("Failed to unmount SD filesystem for USB mass storage: %d", ret);
				atomic_clear(&host_active);
				break;
			}
		}

		if (gpio_pin_get_dt(&sd_detect) > 0) {
			const int ret = init_media_disk();
			if (ret) {
				LOG_ERR("Failed to initialize SD disk for USB mass storage: %d", ret);
				atomic_clear(&host_active);
				break;
			}
		}

		atomic_set(&host_active, 1);
		break;
	case USBD_MSG_VBUS_REMOVED:
	case USBD_MSG_RESET:
		atomic_clear(&host_active);
		break;
	default:
		break;
	}
}

void sd_mass_storage_set_usb_context(struct usbd_context *ctx)
{
	remember_usb_context(ctx);
}

bool sd_mass_storage_host_active()
{
	return usb_host_configured();
}

void sd_mass_storage_recording_stopped()
{
	if (!atomic_cas(&usb_blocked_by_recording, 1, 0)) {
		return;
	}

	if (sdcard_manager.is_mounted()) {
		const int ret = sdcard_manager.unmount();
		if (ret) {
			LOG_ERR("Failed to unmount SD filesystem after recording: %d", ret);
			atomic_set(&usb_blocked_by_recording, 1);
			return;
		}
	}

	if (gpio_pin_get_dt(&sd_detect) > 0) {
		const int ret = init_media_disk();
		if (ret) {
			LOG_ERR("Failed to initialize SD disk after recording: %d", ret);
			atomic_set(&usb_blocked_by_recording, 1);
			return;
		}
	}

	if (usb_context == nullptr) {
		LOG_WRN("USB mass storage was blocked, but no USB context is available");
		return;
	}

	const int ret = usbd_enable(usb_context);
	if (ret && ret != -EALREADY) {
		LOG_WRN("Failed to re-enable USB mass storage after recording: %d", ret);
		atomic_set(&usb_blocked_by_recording, 1);
	}
}

void sd_mass_storage_recording_aborted()
{
	if (sdcard_manager.is_mounted() || gpio_pin_get_dt(&sd_detect) <= 0) {
		return;
	}

	const int ret = init_media_disk();
	if (ret) {
		LOG_WRN("Failed to restore USB mass-storage disk after aborted recording: %d",
			ret);
	}
}

int sd_mass_storage_recording_starting()
{
	if (sdcard_manager.is_mounted()) {
		return 0;
	}

	const int ret = force_disk_deinit();
	if (ret && ret != -ENOTSUP && ret != -EINVAL) {
		LOG_WRN("Failed to release USB mass-storage disk before recording: %d", ret);
		return ret;
	}

	return 0;
}

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
