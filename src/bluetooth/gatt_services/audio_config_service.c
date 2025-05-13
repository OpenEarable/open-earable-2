#include "audio_config_service.h"
#include "../modules/hw_codec.h"

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

BT_GATT_SERVICE_DEFINE(audio_config_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_AUDIO_CONFIG_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_MODE,
                       BT_GATT_CHRC_WRITE,
                       BT_GATT_PERM_WRITE,
                       NULL, write_audio_mode, NULL),
);

int init_audio_config_service(void)
{
    // Standardmäßig Normal-Modus aktivieren
    hw_codec_set_audio_mode(AUDIO_MODE_NORMAL);
    return 0;
}
