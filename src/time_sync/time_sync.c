#include "time_sync.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/devicetree.h>

#include <zephyr/kernel.h>

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(time_sync, LOG_LEVEL_DBG);

#define BT_UUID_TIME_SYNC_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x2e04cbf7, 0x939d, 0x4be5, 0x823e, 0x271838b75259)
#define BT_UUID_TIME_SYNC_OFFSET_CHARAC_VAL \
    BT_UUID_128_ENCODE(0x2e04cbf8, 0x939d, 0x4be5, 0x823e, 0x271838b75259)
#define BT_UUID_TIME_SYNC_RTT_CHARAC_VAL \
    BT_UUID_128_ENCODE(0x2e04cbf9, 0x939d, 0x4be5, 0x823e, 0x271838b75259)

#define BT_UUID_TIME_SYNC_SERVICE           BT_UUID_DECLARE_128(BT_UUID_TIME_SYNC_SERVICE_VAL)
#define BT_UUID_TIME_SYNC_OFFSET_CHARAC     BT_UUID_DECLARE_128(BT_UUID_TIME_SYNC_OFFSET_CHARAC_VAL)
#define BT_UUID_TIME_SYNC_RTT_CHARAC        BT_UUID_DECLARE_128(BT_UUID_TIME_SYNC_RTT_CHARAC_VAL)

enum time_sync_op {
    TIME_SYNC_OP_REQUEST = 0,
    TIME_SYNC_OP_RESPONSE = 1,
};

struct __packed time_sync_packet {
    uint8_t  version;       // Version of the time sync packet
    uint8_t  op;            // 0 = request, 1 = response
    uint16_t seq;           // Sequence number, phone chooses
    uint64_t t1_phone;      // phone send time
    uint64_t t2_dev_rx;     // device receive time
    uint64_t t3_dev_tx;     // device transmit time
};

struct time_sync_packet time_sync_packet = {};
int64_t time_offset_us = 0;

bool notify_rtt_enabled = false;

inline uint64_t oe_micros() {
    return get_current_time_us();
}

static ssize_t write_rtt_request(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags
) {
    uint64_t rx_time = get_current_time_us();

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(struct time_sync_packet)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memcpy(&time_sync_packet, buf, sizeof(struct time_sync_packet));

    LOG_DBG("Received time sync RTT request, len: %u, handle: %u, conn: %p", len, attr->handle, (void *)conn);
    LOG_DBG("Request data: version: %d, op: %d, seq: %d, t1_phone: %llu, t2_dev_rx: %llu, t3_dev_tx: %llu",
        time_sync_packet.version,
        time_sync_packet.op,
        time_sync_packet.seq,
        time_sync_packet.t1_phone,
        time_sync_packet.t2_dev_rx,
        time_sync_packet.t3_dev_tx
    );

    if (time_sync_packet.version != 1) {
        LOG_ERR("Unsupported time sync packet version: %d", time_sync_packet.version);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (time_sync_packet.op != TIME_SYNC_OP_REQUEST) {
        LOG_ERR("Unsupported time sync packet operation: %d", time_sync_packet.op);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    time_sync_packet.op = TIME_SYNC_OP_RESPONSE;
    time_sync_packet.t2_dev_rx = rx_time;
    time_sync_packet.t3_dev_tx = get_current_time_us();

    if (notify_rtt_enabled) {
        bt_gatt_notify(conn, attr, &time_sync_packet, sizeof(struct time_sync_packet));
    }

    return len;
}

static ssize_t write_time_offset(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,uint8_t flags
) {
    if (len != sizeof(int64_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    time_offset_us += *(int64_t *)buf;
    LOG_DBG("Received time offset update: %lld us, new time offset: %lld us", *(int64_t *)buf, time_offset_us);

    return len;
}

bool can_sync_time() {
    //TODO: implement check if sensors are running that prevent time sync   
    return true;
}

int init_time_sync(void) {
	
	return 0;
}

inline uint64_t get_current_time_us() {
    return get_time_since_boot_us() + time_offset_us;
}

inline uint64_t get_time_since_boot_us() {
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

void rtt_cfg_changed(const struct bt_gatt_attr *attr,
                  uint16_t value) {
    LOG_DBG("RTT characteristic CCCD changed: %u", value);
    notify_rtt_enabled = (value == BT_GATT_CCC_NOTIFY);
}


BT_GATT_SERVICE_DEFINE(time_sync_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_TIME_SYNC_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_TIME_SYNC_OFFSET_CHARAC,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_time_offset, &time_offset_us),
    BT_GATT_CHARACTERISTIC(BT_UUID_TIME_SYNC_RTT_CHARAC,
                BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_WRITE,
                NULL, write_rtt_request, &time_sync_packet),
    BT_GATT_CCC(rtt_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
