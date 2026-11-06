#ifndef OPENEARABLE_WIRELESS_AUDIO_CONFIGURATION_SERVICE_H_
#define OPENEARABLE_WIRELESS_AUDIO_CONFIGURATION_SERVICE_H_

#include <stdint.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/conn.h>

#include "wireless_audio_configuration_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Stable ASE lifecycle values exposed by the wireless-audio runtime protocol. */
enum wireless_audio_lifecycle_state {
	WIRELESS_AUDIO_LIFECYCLE_DISCONNECTED = 0,
	WIRELESS_AUDIO_LIFECYCLE_CONNECTED = 1,
	WIRELESS_AUDIO_LIFECYCLE_CODEC_CONFIGURED = 2,
	WIRELESS_AUDIO_LIFECYCLE_QOS_CONFIGURED = 3,
	WIRELESS_AUDIO_LIFECYCLE_ENABLED = 4,
	WIRELESS_AUDIO_LIFECYCLE_STREAMING = 5,
	WIRELESS_AUDIO_LIFECYCLE_RELEASING = 6,
};

/** Mark an ACL as carrying an OpenEarable LE Audio stream and apply local policies. */
void wireless_audio_configuration_audio_connection_set(struct bt_conn *conn);

/** Release runtime state associated with a disconnected ACL. */
void wireless_audio_configuration_disconnected(struct bt_conn *conn);

/** Record confirmed ACL connection parameters. */
void wireless_audio_configuration_conn_params_updated(struct bt_conn *conn, uint16_t interval,
					       uint16_t latency, uint16_t timeout);

/** Record confirmed ACL PHY parameters. */
void wireless_audio_configuration_phy_updated(struct bt_conn *conn,
				      const struct bt_conn_le_phy_info *info);

/** Record confirmed ACL Data Length Extension parameters. */
void wireless_audio_configuration_data_length_updated(
	struct bt_conn *conn, const struct bt_conn_le_data_len_info *info);

/** Copy the configured ASCS QoS preference tuple for an audio direction. */
void wireless_audio_configuration_qos_preferences_get(enum bt_audio_dir direction,
					       struct bt_bap_qos_cfg_pref *preferences);

/** Record a codec configuration accepted through standard ASCS. */
void wireless_audio_configuration_codec_configured(struct bt_conn *conn, uint8_t stream_id,
					    enum bt_audio_dir direction,
					    const struct bt_audio_codec_cfg *codec);

/** Record QoS selected by the Unicast Client through standard ASCS. */
void wireless_audio_configuration_qos_configured(struct bt_conn *conn, uint8_t stream_id,
					  enum bt_audio_dir direction,
					  const struct bt_bap_qos_cfg *qos);

/** Record an ASE lifecycle state using the protocol's stable numeric values. */
void wireless_audio_configuration_stream_state_set(struct bt_conn *conn, uint8_t stream_id,
					     enum bt_audio_dir direction,
					     enum wireless_audio_lifecycle_state lifecycle_state);

#ifdef __cplusplus
}
#endif

#endif /* OPENEARABLE_WIRELESS_AUDIO_CONFIGURATION_SERVICE_H_ */
