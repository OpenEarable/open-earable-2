#include "time_sync.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/devicetree.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <limits.h>

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

int64_t time_offset_us = 0;

bool notify_rtt_enabled = false;

uint64_t oe_micros() {
    return get_current_time_us();
}

static int parse_time_sync_packet_le(const void *buf, uint16_t len, struct time_sync_packet *out)
{
    if (len != sizeof(struct time_sync_packet) || out == NULL || buf == NULL) {
        return -EINVAL;
    }

    const uint8_t *p = (const uint8_t *)buf;
    out->version  = p[0];
    out->op       = p[1];
    out->seq      = sys_get_le16(&p[2]);
    out->t1_phone = sys_get_le64(&p[4]);
    out->t2_dev_rx = sys_get_le64(&p[12]);
    out->t3_dev_tx = sys_get_le64(&p[20]);
    return 0;
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

    struct time_sync_packet pkt = {0};
    int pret = parse_time_sync_packet_le(buf, len, &pkt);
    if (pret != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    LOG_DBG("Received time sync RTT request, len: %u, handle: %u, conn: %p", len, attr->handle, (void *)conn);
    LOG_DBG("Request data: version: %u, op: %u, seq: %u, t1_phone: %llu, t2_dev_rx: %llu, t3_dev_tx: %llu",
        pkt.version,
        pkt.op,
        pkt.seq,
        pkt.t1_phone,
        pkt.t2_dev_rx,
        pkt.t3_dev_tx
    );

    if (pkt.version != 1) {
        LOG_ERR("Unsupported time sync packet version: %u", pkt.version);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (pkt.op != TIME_SYNC_OP_REQUEST) {
        LOG_ERR("Unsupported time sync packet operation: %u", pkt.op);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    pkt.op = TIME_SYNC_OP_RESPONSE;
    pkt.t2_dev_rx = rx_time;
    pkt.t3_dev_tx = get_current_time_us();

    if (notify_rtt_enabled) {
        (void)bt_gatt_notify(conn, attr, &pkt, sizeof(pkt));
    }

    return len;
}

static ssize_t write_time_offset(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags
) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(int64_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    int64_t delta;
    memcpy(&delta, buf, sizeof(delta));
    time_offset_us += delta;
    LOG_DBG("Received time offset update: %lld us, new time offset: %lld us", delta, time_offset_us);

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
   uint64_t base_u = get_time_since_boot_us();
   int64_t base_s = (base_u > (uint64_t)INT64_MAX) ? INT64_MAX : (int64_t)base_u;
   int64_t now_s = base_s + time_offset_us;
   if (now_s < 0) {
       LOG_WRN("Current time underflow, returning 0");
       return 0;
    }
    if (now_s != base_u + time_offset_us) {
        LOG_WRN("Current time overflow, returning UINT64_MAX");
        return UINT64_MAX;
    }
    return (uint64_t)now_s;
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
                NULL, write_rtt_request, NULL),
    BT_GATT_CCC(rtt_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
