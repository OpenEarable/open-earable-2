#pragma once

#include <zephyr/kernel.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT) && defined(CONFIG_USBD_MSC_CLASS)

/**
 * Initialize the raw disk backing the USB mass-storage LUN.
 *
 * The SD power rail must already be enabled. A missing card is a supported
 * state and does not make this function fail.
 *
 * Card-detect monitoring itself is owned by SDCardManager; this module only
 * reacts to the transitions it reports via sd_mass_storage_handle_card_change().
 */
int sd_mass_storage_init();

/**
 * Bring the raw disk backing the USB mass-storage LUN in sync with the card
 * presence reported by SDCardManager.
 *
 * @param inserted true when a card is present.
 *
 * Ordering is the caller's responsibility and matters: on removal this must run
 * *after* the filesystem has been unmounted, because it force-releases the disk
 * out from under any remaining reference. On insertion it must run before the
 * filesystem is mounted again.
 */
void sd_mass_storage_handle_card_change(bool inserted);

#else

static inline int sd_mass_storage_init()
{
	return 0;
}

static inline void sd_mass_storage_handle_card_change(bool inserted)
{
	ARG_UNUSED(inserted);
}

#endif /* CONFIG_USB_DEVICE_STACK_NEXT && CONFIG_USBD_MSC_CLASS */
