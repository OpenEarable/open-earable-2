#ifndef OPENEARABLE_BT_MGMT_CONN_INTERVAL_H_
#define OPENEARABLE_BT_MGMT_CONN_INTERVAL_H_

#include <stdint.h>

#include <zephyr/bluetooth/conn.h>

#include "wireless_audio_configuration_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Callback invoked after the controller accepts a local ACL update request. */
typedef void (*bt_mgmt_ci_adjustment_cb_t)(void);

/** Initialize the configurable ACL connection policy engine. */
int bt_mgmt_conn_interval_init(void);

/** Select the active ACL connection policy, copying the supplied value. */
int bt_mgmt_ci_policy_set(
	const wireless_audio_configuration_acl_connection_policy_t *policy);

/** Register a callback used to count locally accepted ACL update requests. */
void bt_mgmt_ci_adjustment_callback_set(bt_mgmt_ci_adjustment_cb_t callback);

/** Associate the policy engine with an ACL carrying an LE Audio stream. */
void bt_mgmt_ci_on_connected(struct bt_conn *conn);

/** Remove an ACL association when it disconnects. */
void bt_mgmt_ci_on_disconnected(struct bt_conn *conn, uint8_t reason);

/** Feed confirmed connection parameters back into the active policy. */
void bt_mgmt_ci_on_conn_param_updated(struct bt_conn *conn, uint16_t interval,
				      uint16_t latency, uint16_t timeout);

/** Feed an audio-underrun episode into policies that use runtime feedback. */
void bt_mgmt_report_audio_underrun(uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* OPENEARABLE_BT_MGMT_CONN_INTERVAL_H_ */
