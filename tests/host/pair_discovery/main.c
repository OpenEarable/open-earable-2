#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARG_UNUSED(x) ((void)(x))
#define IS_ENABLED(x) (x)
#define CONFIG_BT_CSIP_SET_MEMBER 1
#define CONFIG_BT_DEVICE_APPEARANCE 0
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define BIT(x) (1U << (x))
#define BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED 0
#define BT_UUID_CAS_VAL 0x1853
#define BT_DATA_SVC_DATA16 0x16
#define BT_DATA_MANUFACTURER_DATA 0xff
#define BT_DATA_CSIS_RSI 0x2e
#define BT_DATA_GAP_APPEARANCE 0x19
#define BT_DATA_FLAGS 0x01
#define BT_LE_AD_GENERAL 0x02
#define BT_LE_AD_NO_BREDR 0x04
#define BT_CSIP_SIRK_SIZE 16
#define BT_CSIP_RSI_SIZE 6
#define BT_CSIP_CRYPTO_KEY_SIZE 16
#define BT_CSIP_CRYPTO_PRAND_SIZE 3
#define BT_CSIP_CRYPTO_HASH_SIZE 3
#define BT_CSIP_CRYPTO_PADDING_SIZE 13
#define BT_CSIP_PADDED_RAND_SIZE 16
#define BT_ADDR_LE_STR_LEN 32
#define LOG_DBG(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)

typedef struct { uint8_t dummy; } bt_addr_le_t;
struct net_buf_simple { uint8_t *data; size_t len; };
struct bt_data { uint8_t type; uint8_t data_len; const uint8_t *data; };
enum audio_channel { AUDIO_CH_L = 0, AUDIO_CH_R = 1 };
static struct { uint64_t device_id; } oe_boot_state;
static enum audio_channel channel;
static unsigned int resets;
static uint32_t written_sirk;
static uint8_t unicast_server_adv_data[] = {0x4e, 0x18, 0};
static uint8_t csip_rsi_adv_data[BT_CSIP_RSI_SIZE];
static uint8_t gap_appear_adv_data[2], flags_adv_data, device_identifier[11];
static const uint8_t cap_adv_data[] = {0x53, 0x18, 0};

static void sys_put_le16(uint16_t value, uint8_t *out) {
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8);
}
static void sys_put_le64(uint64_t value, uint8_t *out) {
    for (int i = 0; i < 8; i++) out[i] = (uint8_t)(value >> (8 * i));
}
static void channel_assignment_get(enum audio_channel *out) { *out = channel; }
static int bt_mgmt_adv_buffer_put(struct bt_data *buffer, uint32_t *count,
                                  uint8_t size, size_t len, uint8_t type, void *data) {
    if (*count >= size) return -ENOMEM;
    buffer[*count] = (struct bt_data){type, (uint8_t)len, data};
    ++*count;
    return 0;
}
static uint8_t net_buf_simple_pull_u8(struct net_buf_simple *buf) {
    uint8_t value = *buf->data++; --buf->len; return value;
}
static void bt_addr_le_to_str(const bt_addr_le_t *addr, char *out, size_t len) {
    (void)addr; if (len) out[0] = 0;
}
/* Stand-in for AES: deterministic and key-sensitive so both production paths
 * must construct the same key and random input. Real AES is provided by Zephyr. */
static int bt_encrypt_le(const uint8_t *key, const uint8_t *in, uint8_t *out) {
    for (int i = 0; i < 3; i++) out[i] = key[i] ^ in[i];
    return 0;
}
static void write_sirk(uint32_t value) { written_sirk = value; resets++; }

#include "production.inc"

static size_t serialize(struct bt_data *ad, int count, uint8_t *out,
                        bool drop_manufacturer, bool shorten_manufacturer) {
    size_t n = 0;
    for (int i = 0; i < count; ++i) {
        if (ad[i].type == BT_DATA_MANUFACTURER_DATA && drop_manufacturer) continue;
        size_t len = ad[i].data_len;
        if (ad[i].type == BT_DATA_MANUFACTURER_DATA && shorten_manufacturer) len = 7;
        out[n++] = (uint8_t)(len + 1);
        out[n++] = ad[i].type;
        memcpy(out + n, ad[i].data, len); n += len;
    }
    return n;
}

static void scan(uint8_t *packet, size_t size) {
    const bt_addr_le_t addr = {0};
    struct net_buf_simple ad = {packet, size};
    device_found(&addr, -40, 0, &ad);
}

int main(void) {
    const uint64_t left_id = UINT64_C(0x0123456710203040);
    const uint64_t right_id = UINT64_C(0x76543210b0e0f00d);
    const uint32_t pair_id = 0xa0c0c04d;
    struct bt_data ad[8];
    uint8_t packet[128];

    channel = AUDIO_CH_R; oe_boot_state.device_id = right_id;
    int count = unicast_server_adv_populate(ad, ARRAY_SIZE(ad));
    assert(count == 6 && ad[4].type == BT_DATA_SVC_DATA16);
    assert(ad[5].type == BT_DATA_MANUFACTURER_DATA && ad[5].data_len == 11);
    const uint8_t expected[] = {0x0d, 0xf0, 0xe0, 0xb0, 0x10, 0x32, 0x54, 0x76, 1, 0, 0};
    assert(memcmp(ad[5].data, expected, sizeof(expected)) == 0);
    assert(unicast_server_adv_populate(ad, 5) == -ENOMEM);

    channel = AUDIO_CH_L; oe_boot_state.device_id = left_id;
    size_t n = serialize(ad, count, packet, true, false);
    scan(packet, n); assert(resets == 0);
    n = serialize(ad, count, packet, false, true);
    scan(packet, n); assert(resets == 0);
    n = serialize(ad, count, packet, false, false);
    scan(packet, n); assert(resets == 1 && written_sirk == pair_id);

    // After the left resets, it advertises an RSI for the newly written pair ID.
    char key[17] = {0}; snprintf(key, sizeof(key), "%08X", pair_id);
    csip_rsi_adv_data[3] = 0x12; csip_rsi_adv_data[4] = 0x34; csip_rsi_adv_data[5] = 0x40;
    for (int i = 0; i < 3; ++i) csip_rsi_adv_data[i] = (uint8_t)key[i] ^ csip_rsi_adv_data[i + 3];
    channel = AUDIO_CH_L; oe_boot_state.device_id = left_id;
    count = unicast_server_adv_populate(ad, ARRAY_SIZE(ad));
    n = serialize(ad, count, packet, false, false);
    channel = AUDIO_CH_R; oe_boot_state.device_id = right_id;
    scan(packet, n); assert(resets == 2 && written_sirk == pair_id);
    packet[7] ^= 1; // Corrupt the RSI hash and ensure no reset.
    scan(packet, n); assert(resets == 2);
    puts("PASS: legacy manufacturer advertisement reaches left/right discovery and both reset with the same pair ID");
    return 0;
}
