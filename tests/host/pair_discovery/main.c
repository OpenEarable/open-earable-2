#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include "earable_pairing.h"
#include "csip_identity.h"

#define IS_ENABLED(x) (x)
#define CONFIG_BT_CSIP_SET_MEMBER 1
#define CONFIG_BT_DEVICE_APPEARANCE 0
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
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
/* Host substitutes for settings, UICR and Zephyr scheduling. */
#define SYS_REBOOT_COLD 0
#define K_SECONDS(n) (n)
#define K_NO_WAIT 0
#define BT_UUID_16_ENCODE(id) ((id) & 255), ((id) >> 8)
#define BT_DATA(t, data, size) {t, size, data}
#define BT_DATA_BYTES(t, ...) BT_DATA(t, ((const uint8_t[]){__VA_ARGS__}), sizeof((uint8_t[]){__VA_ARGS__}))
#define K_WORK_DELAYABLE_DEFINE(name, fn) struct k_work name = {fn}
#define k_work_schedule(work, delay) (work)->handler(work)
#define k_msgq_put(q, peer, timeout) (consider_peer(peer), 0)
#define k_work_submit(work) ((void)0)
struct k_work { void (*handler)(struct k_work *); };
struct bt_le_ext_adv;
typedef int atomic_t;
static bool atomic_cas(atomic_t *v, int old, int next) {
    if (*v != old) {
        return false;
    }
    *v = next;
    return true;
}
static void atomic_clear(atomic_t *v) { *v = 0; }
static uint64_t sys_get_le64(const uint8_t *p) {
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) value = (value << 8) | p[i];
    return value;
}
static uint16_t sys_get_le16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static uint32_t sys_get_le32(const uint8_t *p) { return sys_get_le16(p) | ((uint32_t)sys_get_le16(p+2) << 16); }
static void sys_put_le32(uint32_t v, uint8_t *p) { sys_put_le16(v, p); sys_put_le16(v >> 16, p + 2); }
static uint8_t flash[6], standalone = 255;
static size_t flash_len;
static int write_error;
static uint32_t uicr_sirk_get(void) { return written_sirk; }
static uint8_t uicr_standalone_get(void) { return standalone; }
static int uicr_sirk_set(uint32_t id) {
    assert(written_sirk == UINT32_MAX);
    written_sirk = id;
    return 0;
}
static int settings_subsys_init(void) { return 0; }
static ssize_t settings_load_one(const char *key, void *out, size_t len) {
    assert(strcmp(key, "earable/pair") == 0 && len >= flash_len);
    memcpy(out, flash, flash_len);
    return flash_len;
}
static int settings_save_one(const char *key, const void *data, size_t len) {
    assert(strcmp(key, "earable/pair") == 0 && len == sizeof(flash));
    if (write_error) return write_error;
    memcpy(flash, data, len);
    flash_len = len;
    return 0;
}
static int settings_delete(const char *key) {
    assert(strcmp(key, "earable/pair") == 0);
    flash_len = 0;
    return 0;
}
static void sys_reboot(int kind) { resets++; }
void earable_pairing_reboot(void) { resets++; }
static int64_t now;
static int64_t k_uptime_get(void) { return now; }
static int beacon_updates;
static int bt_le_ext_adv_set_data(struct bt_le_ext_adv *adv, const struct bt_data *ad,
                                 size_t count, const struct bt_data *sd, size_t sd_len) {
    ++beacon_updates;
    return 0;
}
static void bt_data_parse(struct net_buf_simple *ad, bool (*visit)(struct bt_data *, void *), void *ctx) {
    while (ad->len > 1) {
        uint8_t len = net_buf_simple_pull_u8(ad);
        if (!len || len > ad->len) return;
        struct bt_data field = {ad->data[0], len - 1, ad->data + 1};
        if (!visit(&field, ctx)) return;
        ad->len -= len;
        ad->data += len;
    }
}
/* Deterministic stand-in for Zephyr's CSIP hash. */
static int bt_csip_sih(const uint8_t *key, uint8_t *random, uint8_t *out) {
    for (int i = 0; i < 3; i++) out[i] = key[i] ^ key[i + 3] ^ key[i + 5] ^ random[i];
    return 0;
}

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

static void boot(void) {
    earable_pairing_release();
    assert(earable_pairing_load() == 0);
}

int main(void) {
    const uint64_t left_id = UINT64_C(0x0123456710203040);
    const uint64_t right_id = UINT64_C(0x76543210b0e0f00d);
    const uint32_t pair_id = 0xa0c0c04d;
    const uint32_t factory_pair = 0x12345678;
    struct bt_data ad[8];
    uint8_t packet[128];

    written_sirk = factory_pair;
    boot();
    assert(!earable_pairing_is_seeking() && earable_pairing_side_override() == -1);
    assert(earable_pairing_set_mode(EARABLE_SEEKING, AUDIO_CH_L) == 0);
    boot();
    assert(earable_pairing_is_seeking() && earable_pairing_side_override() == AUDIO_CH_L);
    assert(earable_pairing_previous_identity() == factory_pair);

    channel = AUDIO_CH_R; oe_boot_state.device_id = right_id;
    int count = unicast_server_adv_populate(ad, ARRAY_SIZE(ad));
    assert(count == 6 && ad[4].type == BT_DATA_SVC_DATA16);
    assert(ad[5].type == BT_DATA_MANUFACTURER_DATA && ad[5].data_len == 11);
    const uint8_t expected[] = {0x0d, 0xf0, 0xe0, 0xb0, 0x10, 0x32, 0x54, 0x76, 1, 0, 0};
    assert(memcmp(ad[5].data, expected, sizeof(expected)) == 0);
    assert(unicast_server_adv_populate(ad, 5) == -ENOMEM);

    side = AUDIO_CH_L; oe_boot_state.device_id = left_id;
    csip_rsi_adv_data[5] = beacon_rsi[5] = 0x40;
    identity_hash((uint32_t)right_id, csip_rsi_adv_data, csip_rsi_adv_data);
    size_t n = serialize(ad, count, packet, true, false);
    scan(packet, n);
    n = serialize(ad, count, packet, false, true);
    scan(packet, n);
    device_identifier[8] = AUDIO_CH_L;
    n = serialize(ad, count, packet, false, false);
    scan(packet, n);
    assert(beacon_updates == 0 && resets == 0);
    device_identifier[8] = AUDIO_CH_R;

    /* A legacy standalone right can elicit an offer, but cannot confirm. */
    n = serialize(ad, count, packet, false, false);
    scan(packet, n);
    assert(beacon_updates == 1 && resets == 0 && rsi_matches(pair_id, beacon_rsi));
    assert(earable_pairing_is_seeking() && written_sirk == factory_pair);

    /* A compatible right advertises the shared identity in reply. */
    memcpy(csip_rsi_adv_data, beacon_rsi, sizeof(beacon_rsi));
    n = serialize(ad, count, packet, false, false);
    packet[7] ^= 1;
    scan(packet, n); assert(resets == 0);
    packet[7] ^= 1;
    write_error = -EIO; scan(packet, n); assert(resets == 0);
    write_error = 0; scan(packet, n); assert(resets == 1);
    boot();
    assert(!earable_pairing_is_seeking() && earable_pairing_identity() == pair_id);
    assert(written_sirk == factory_pair && earable_pairing_side_override() == AUDIO_CH_L);

    /* GATT can switch to standalone or right even with a programmed UICR. */
    assert(earable_pairing_set_mode(EARABLE_STANDALONE, AUDIO_CH_L) == 0);
    boot();
    assert(!earable_pairing_is_seeking() && earable_pairing_identity() == (uint32_t)left_id);
    assert(earable_pairing_set_mode(EARABLE_SEEKING, AUDIO_CH_R) == 0);
    boot();
    side = AUDIO_CH_R;
    channel = AUDIO_CH_L; oe_boot_state.device_id = left_id;
    count = unicast_server_adv_populate(ad, ARRAY_SIZE(ad));
    n = serialize(ad, count, packet, false, false);
    oe_boot_state.device_id = right_id;
    scan(packet, n); assert(resets == 1); /* Its old left is still idle. */
    device_identifier[9] = SEEKING_MARKER;
    n = serialize(ad, count, packet, false, false);
    scan(packet, n); assert(resets == 2);
    boot();
    assert(earable_pairing_side_override() == AUDIO_CH_R && earable_pairing_identity() == pair_id);
    /* A legacy left can signal intent without a marker. If power is lost
     * after its individual RSI, its shared confirmation still completes. */
    assert(earable_pairing_set_mode(EARABLE_SEEKING, AUDIO_CH_R) == 0);
    boot();
    device_identifier[9] = 0;
    identity_hash((uint32_t)left_id, csip_rsi_adv_data, csip_rsi_adv_data);
    n = serialize(ad, count, packet, false, false);
    scan(packet, n); assert(resets == 2);
    boot();
    identity_hash(pair_id, csip_rsi_adv_data, csip_rsi_adv_data);
    n = serialize(ad, count, packet, false, false);
    scan(packet, n); assert(resets == 3);
    assert(earable_pairing_set_mode(EARABLE_PROVISIONED, AUDIO_CH_R) == 0);
    boot();
    assert(!flash_len && earable_pairing_side_override() == -1 && earable_pairing_identity() == factory_pair);

    /* Fresh provisioning keeps first-pair UICR semantics. Explicit GATT
     * pairing from a blank default only uses flash and is fully removable. */
    written_sirk = UINT32_MAX;
    boot(); assert(earable_pairing_is_seeking());
    assert(earable_pairing_set_mode(EARABLE_SEEKING, AUDIO_CH_R) == 0);
    boot();
    assert(earable_pairing_bond(pair_id, AUDIO_CH_R) == 0 && written_sirk == UINT32_MAX);
    assert(earable_pairing_set_mode(EARABLE_PROVISIONED, AUDIO_CH_R) == 0);
    boot(); assert(earable_pairing_is_seeking());
    assert(earable_pairing_bond(pair_id, AUDIO_CH_R) == 0);
    boot(); assert(!flash_len && written_sirk == pair_id && !earable_pairing_is_seeking());
    written_sirk = UINT32_MAX; standalone = 0;
    boot(); assert(written_sirk == (uint32_t)right_id && !earable_pairing_is_seeking());
    puts("PASS: opposite-side discovery, factory and override pairing, persistence and restore");
    return 0;
}
