#ifndef FILE_TRANSFER_SERVICE_H
#define FILE_TRANSFER_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>

#define BT_UUID_FILE_TRANSFER_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6cae1b90, 0x8ce4, 0x41ec, 0x8538, 0x8c4befed5a4d)
#define BT_UUID_COMMAND_CHARAC_VAL \
    BT_UUID_128_ENCODE(0x6cae1b91, 0x8ce4, 0x41ec, 0x8538, 0x8c4befed5a4d)
#define BT_UUID_STATUS_CHARAC_VAL \
    BT_UUID_128_ENCODE(0x6cae1b92, 0x8ce4, 0x41ec, 0x8538, 0x8c4befed5a4d)

#define BT_UUID_FILE_TRANSFER_SERVICE   BT_UUID_DECLARE_128(BT_UUID_FILE_TRANSFER_SERVICE_VAL)
#define BT_UUID_COMMAND_CHARAC          BT_UUID_DECLARE_128(BT_UUID_COMMAND_CHARAC_VAL)
#define BT_UUID_STATUS_CHARAC           BT_UUID_DECLARE_128(BT_UUID_STATUS_CHARAC_VAL)


void file_transfer_service_init(void);
void file_transfer_service_on_command_received(const uint8_t *data, uint16_t length, struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif // FILE_TRANSFER_SERVICE_H