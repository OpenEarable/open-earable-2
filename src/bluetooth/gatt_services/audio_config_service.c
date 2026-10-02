#include "audio_config_service.h"
#include "../modules/hw_codec.h"

#include "zbus_common.h"

#include "audio_system.h"
#include "channel_assignment.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_config_service, CONFIG_BLE_LOG_LEVEL);

/** @brief Decode and apply a validated codec audio mode. */
static ssize_t write_audio_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    audio_configuration_audio_mode_t message;
    if (audio_configuration_audio_mode_decode(&message, buf, len, NULL) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t mode = message.mode;
    if (mode > AUDIO_MODE_ANC) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = hw_codec_set_audio_mode((enum audio_mode)mode);
    if (ret) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

/** @brief Decode and select the encoder microphone. */
static ssize_t write_mic_select(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    audio_configuration_microphone_selection_t message;
    if (audio_configuration_microphone_selection_decode(&message, buf, len, NULL) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t mic_select = message.microphone;
    LOG_INF("Mic select: %d", mic_select);
    if (mic_select > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = audio_system_set_encoder_channel(mic_select == 0 ? AUDIO_CH_L : AUDIO_CH_R);
    if (ret) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return len;
}

/** @brief Encode the current codec audio mode for a GATT read. */
static ssize_t read_audio_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t mode = hw_codec_get_audio_mode();
    audio_configuration_audio_mode_t message = { .mode = mode };
    uint8_t payload[1];
    size_t size;
    if (audio_configuration_audio_mode_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

/** @brief Encode the selected encoder microphone for a GATT read. */
static ssize_t read_mic_select(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t mic = audio_system_get_encoder_channel() == AUDIO_CH_L ? 0 : 1;
    audio_configuration_microphone_selection_t message = { .microphone = mic };
    uint8_t payload[1];
    size_t size;
    if (audio_configuration_microphone_selection_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

/** @brief Encode the assigned audio channel for a GATT read. */
static ssize_t read_audio_channel(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    enum audio_channel channel;

    //backup channel
    channel_assignment_get(&channel);
    uint8_t channel_u8 = channel;

    audio_configuration_audio_channel_t message = { .channel = channel_u8 };
    uint8_t payload[1];
    size_t size;
    if (audio_configuration_audio_channel_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

/** @brief Decode and apply the outer and inner microphone gain registers. */
static ssize_t write_dmic_gain(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    // Mic gain is 2 bytes: [outer_reg, inner_reg].
    // Outer mic maps to DMIC_VOL0; inner mic maps to DMIC_VOL1.
    // Per ADAU186x DMIC_VOL register (0x4000C045):
    //   0x00      = +24 dB
    //   0x01-0x3F = +23.625 to +0.375 dB (0.375 dB steps)
    //   0x40      = 0 dB
    //   0x41-0xFD = -0.375 to -70.875 dB (0.375 dB steps)
    //   0xFE      = -71.25 dB
    //   0xFF      = Mute
    if (len != 2) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    audio_configuration_microphone_gain_t message;
    if (audio_configuration_microphone_gain_decode(&message, buf, len, NULL) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t gain_outer = message.outer;
    uint8_t gain_inner = message.inner;

    int ret = hw_codec_mic_gain_set(gain_outer, gain_inner);
    if (ret) {
        LOG_ERR("Failed to set mic gain: %d", ret);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    LOG_INF("DMIC gain via BLE: outer=0x%02x inner=0x%02x",
            gain_outer, gain_inner);
    return len;
}

/** @brief Encode the current microphone gain registers for a GATT read. */
static ssize_t read_dmic_gain(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    audio_configuration_microphone_gain_t message = {
        .outer = hw_codec_mic_gain_get_outer(),
        .inner = hw_codec_mic_gain_get_inner()
    };
    uint8_t payload[2];
    size_t size;
    if (audio_configuration_microphone_gain_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

BT_GATT_SERVICE_DEFINE(audio_config_svc,
    BT_GATT_PRIMARY_SERVICE(AUDIO_CONFIGURATION_ZEPHYR_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(AUDIO_CONFIGURATION_ZEPHYR_AUDIO_MODE_CHARACTERISTIC_UUID,
                AUDIO_CONFIGURATION_ZEPHYR_AUDIO_MODE_CHARACTERISTIC_PROPERTIES,
                AUDIO_CONFIGURATION_ZEPHYR_AUDIO_MODE_CHARACTERISTIC_PERMISSIONS,
                       read_audio_mode, write_audio_mode, NULL),
    BT_GATT_CHARACTERISTIC(AUDIO_CONFIGURATION_ZEPHYR_MICROPHONE_SELECTION_CHARACTERISTIC_UUID,
                AUDIO_CONFIGURATION_ZEPHYR_MICROPHONE_SELECTION_CHARACTERISTIC_PROPERTIES,
                AUDIO_CONFIGURATION_ZEPHYR_MICROPHONE_SELECTION_CHARACTERISTIC_PERMISSIONS,
                       read_mic_select, write_mic_select, NULL),
    BT_GATT_CHARACTERISTIC(AUDIO_CONFIGURATION_ZEPHYR_AUDIO_CHANNEL_CHARACTERISTIC_UUID,
                AUDIO_CONFIGURATION_ZEPHYR_AUDIO_CHANNEL_CHARACTERISTIC_PROPERTIES,
                AUDIO_CONFIGURATION_ZEPHYR_AUDIO_CHANNEL_CHARACTERISTIC_PERMISSIONS,
                       read_audio_channel, NULL, NULL),
    BT_GATT_CHARACTERISTIC(AUDIO_CONFIGURATION_ZEPHYR_MICROPHONE_GAIN_CHARACTERISTIC_UUID,
                AUDIO_CONFIGURATION_ZEPHYR_MICROPHONE_GAIN_CHARACTERISTIC_PROPERTIES,
                AUDIO_CONFIGURATION_ZEPHYR_MICROPHONE_GAIN_CHARACTERISTIC_PERMISSIONS,
                       read_dmic_gain, write_dmic_gain, NULL),
);

/** @brief Initialize the codec to normal audio mode. */
int init_audio_config_service(void)
{
    // Standardmäßig Normal-Modus aktivieren
    hw_codec_set_audio_mode(AUDIO_MODE_NORMAL);
    return 0;
}
