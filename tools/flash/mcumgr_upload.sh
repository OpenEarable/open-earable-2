mcumgr conn add acm0 type="serial" connstring="dev=/dev/tty.usbmodem11101,baud=115200,mtu=512"

# mcumgr -c acm0 image list

mcumgr -c acm0 image upload -e -n 0  build_fota/open-earable-v2/zephyr/zephyr.signed.bin
mcumgr -c acm0 image upload -e -n 1  build_fota/signed_by_mcuboot_and_b0_ipc_radio.bin

# Get hashes from image list
HASH_APP=$(mcumgr -c acm0 image list | grep "hash: " | sed -n 2p | cut -d" " -f6)
HASH_NET=$(mcumgr -c acm0 image list | grep "hash: " | sed -n 3p | cut -d" " -f6)

# Test both images with their hashes
mcumgr -c acm0 image test $HASH_APP > /dev/null 2>&1
mcumgr -c acm0 image test $HASH_NET

echo "Resetting device..."

mcumgr -c acm0 reset