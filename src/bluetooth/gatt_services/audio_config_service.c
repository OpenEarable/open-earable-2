#include "audio_config_service.h"
#include "../modules/hw_codec.h"

#include "zbus_common.h"

#include "audio_system.h"
#include "channel_assignment.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_config_service, CONFIG_BLE_LOG_LEVEL);

static ssize_t write_audio_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t mode = *((uint8_t*)buf);
    if (mode > AUDIO_MODE_ANC) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    hw_codec_set_audio_mode((enum audio_mode)mode);
    return len;
}

static ssize_t write_mic_select(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    LOG_INF("Mic select: %d", *((uint8_t*)buf));

    uint8_t mic_select = *((uint8_t*)buf);
    if (mic_select > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = audio_system_set_encoder_channel(mic_select == 0 ? AUDIO_CH_L : AUDIO_CH_R);
    if (ret) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return len;
}

static ssize_t read_audio_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t mode = hw_codec_get_audio_mode();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode, sizeof(mode));
}

static ssize_t read_mic_select(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t mic = audio_system_get_encoder_channel() == AUDIO_CH_L ? 0 : 1;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &mic, sizeof(mic));
}

static ssize_t read_audio_channel(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    enum audio_channel channel;

    //backup channel
    channel_assignment_get(&channel);
    uint8_t channel_u8 = channel;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &channel_u8, sizeof(channel));
}

BT_GATT_SERVICE_DEFINE(audio_config_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_AUDIO_CONFIG_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_MODE,
                       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                       BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                       read_audio_mode, write_audio_mode, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_MIC_SELECT,
                       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                       BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                       read_mic_select, write_mic_select, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_CHANNEL,
                       BT_GATT_CHRC_READ,
                       BT_GATT_PERM_READ,
                       read_audio_channel, NULL, NULL),
);

int init_audio_config_service(void)
{
    // Standardmäßig Normal-Modus aktivieren
    hw_codec_set_audio_mode(AUDIO_MODE_NORMAL);
    return 0;
}
