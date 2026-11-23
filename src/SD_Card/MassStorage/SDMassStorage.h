#pragma once

#include <zephyr/kernel.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT) && defined(CONFIG_USBD_MSC_CLASS)

#include <zephyr/usb/usbd.h>

/**
 * Attach the raw disk backing the USB mass-storage LUN to the card that is
 * currently present, if any.
 *
 * The SD power rail must already be enabled. A missing card is a supported
 * state and does not make this function fail.
 *
 * Card-detect monitoring itself belongs to SDCardManager; this module only
 * reacts to the transitions it reports via sd_mass_storage_handle_card_change().
 */
int sd_mass_storage_init();

/**
 * Bring the raw disk backing the USB mass-storage LUN in sync with the card
 * presence reported by SDCardManager.
 *
 * @param inserted true when a card is present.
 *
 * Ordering is the caller's responsibility: on removal this must run after the
 * filesystem has been unmounted, because it force-releases the disk regardless
 * of any remaining reference. On insertion it must run before the filesystem is
 * mounted again.
 */
void sd_mass_storage_handle_card_change(bool inserted);

/**
 * Track USB device state changes so SD logging can avoid sharing the FAT
 * filesystem with an enumerated mass-storage host.
 */
void sd_mass_storage_usb_msg_cb(struct usbd_context *const ctx,
				const struct usbd_msg *const msg);

/**
 * Keep the USB context so mass storage can be re-enabled after a recording that
 * temporarily owned the SD card.
 */
void sd_mass_storage_set_usb_context(struct usbd_context *ctx);

/**
 * Returns true once the USB host configured the MSC device.
 */
bool sd_mass_storage_host_active();

/**
 * Re-enable USB mass storage if it was held off while a recording was active.
 */
void sd_mass_storage_recording_stopped();

/**
 * Restore the mass-storage disk reference after a recording start failed before
 * the firmware filesystem could take ownership.
 */
void sd_mass_storage_recording_aborted();

/**
 * Release the mass-storage disk reference before the firmware mounts the
 * filesystem for recording.
 */
int sd_mass_storage_recording_starting();

#else

static inline int sd_mass_storage_init()
{
	return 0;
}

static inline void sd_mass_storage_handle_card_change(bool inserted)
{
	ARG_UNUSED(inserted);
}

static inline bool sd_mass_storage_host_active()
{
	return false;
}

static inline void sd_mass_storage_recording_stopped()
{
}

static inline void sd_mass_storage_recording_aborted()
{
}

static inline int sd_mass_storage_recording_starting()
{
	return 0;
}

#endif /* CONFIG_USB_DEVICE_STACK_NEXT && CONFIG_USBD_MSC_CLASS */
