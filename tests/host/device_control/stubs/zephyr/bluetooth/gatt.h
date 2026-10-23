#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include "conn.h"
struct bt_gatt_attr {};
struct bt_gatt_indicate_params {
 const bt_gatt_attr *attr;
 void (*func)(bt_conn *, bt_gatt_indicate_params *, uint8_t);
 void (*destroy)(bt_gatt_indicate_params *);
 const void *data;
 uint16_t len;
};
extern bt_gatt_attr status_attr;
extern bool subscribed;
extern int indicate_error;
extern bt_gatt_indicate_params *sent;
#define BT_UUID_128_ENCODE(...) 0
#define BT_UUID_DECLARE_128(...) 0
#define BT_GATT_SERVICE_DEFINE(...)
#define BT_GATT_ERR(n) (-(n))
#define BT_ATT_ERR_INVALID_OFFSET 7
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN 13
#define BT_ATT_ERR_WRITE_NOT_PERMITTED 3
#define BT_ATT_ERR_VALUE_NOT_ALLOWED 19
#define BT_ATT_ERR_CCC_IMPROPER_CONF 253
#define BT_ATT_ERR_PROCEDURE_IN_PROGRESS 254
#define BT_ATT_ERR_UNLIKELY 14
#define BT_GATT_WRITE_FLAG_PREPARE 1
#define BT_GATT_WRITE_FLAG_CMD 2
#define BT_GATT_CCC_INDICATE 2
inline bt_gatt_attr *bt_gatt_find_by_uuid(const bt_gatt_attr *, uint16_t, int) { return &status_attr; }
inline bool bt_gatt_is_subscribed(bt_conn *, const bt_gatt_attr *, int) { return subscribed; }
inline int bt_gatt_indicate(bt_conn *, bt_gatt_indicate_params *p) { sent = p; return indicate_error; }
inline ssize_t bt_gatt_attr_read(bt_conn *, const bt_gatt_attr *, void *out,
 uint16_t len, uint16_t offset, const void *data, uint16_t size) {
 if (offset > size) return -BT_ATT_ERR_INVALID_OFFSET;
 auto count = std::min<int>(len, size - offset);
 std::memcpy(out, static_cast<const uint8_t *>(data) + offset, count);
 return count;
}
