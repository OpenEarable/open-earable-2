#!/bin/bash

MGR_CMD="nrfutil mcu-manager serial --serial-port port=/dev/ttyACM0,baud_rate=57600"

HASH_APP="$(sha256sum build/open-earable-2/zephyr/zephyr.signed.bin | cut -d' ' -f1)"
HASH_NET="$(sha256sum build/signed_by_mcuboot_and_b0_ipc_radio.bin | cut -d' ' -f1)"
echo "Hash APP: $HASH_APP"
echo "Hash NET: $HASH_NET"

echo "Current images:"
$MGR_CMD image-list

# BEST OPTION!
#$MGR_CMD image-upload --firmware build/dfu_application.zip --mtu 256

# I think I can only upload the app image?
$MGR_CMD image-erase
$MGR_CMD image-upload --firmware build/open-earable-2/zephyr/zephyr.signed.bin --mtu 128
$MGR_CMD image-test --hash $HASH_APP && $MGR_CMD image-confirm 

#$MGR_CMD image-upload --firmware build/signed_by_mcuboot_and_b0_ipc_radio.bin --image-number 1
#$MGR_CMD image-test --hash $HASH_NET && $MGR_CMD image-confirm 

echo "Resetting device..."

$MGR_CMD reset