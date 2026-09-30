#include "wireless_audio_configuration_service.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include "bt_mgmt_conn_interval.h"
#include "wireless_audio_configuration_protocol.h"
#include "zephyr/wireless_audio_configuration_ble.h"

LOG_MODULE_REGISTER(wireless_audio_configuration, LOG_LEVEL_DBG);

#define ACL_SECTION_BIT BIT(0)
#define RADIO_SECTION_BIT BIT(1)
#define QOS_SECTION_BIT BIT(2)
#define ALL_SECTION_BITS (ACL_SECTION_BIT | RADIO_SECTION_BIT | QOS_SECTION_BIT)

#define ACL_POLICY_ENCODED_MAX 31U
#define RADIO_POLICY_ENCODED_MAX 7U
#define QOS_PREFERENCES_ENCODED_SIZE 22U
#define COMMAND_ENCODED_MAX 35U
#define RESPONSE_ENCODED_MAX 35U
#define RUNTIME_STATE_ENCODED_SIZE 65U
#define CAPABILITIES_ENCODED_SIZE 61U

#define RUNTIME_VALID_CONNECTION BIT(0)
#define RUNTIME_VALID_PHY BIT(1)
#define RUNTIME_VALID_DATA_LENGTH BIT(2)
#define RUNTIME_VALID_CODEC BIT(3)
#define RUNTIME_VALID_QOS BIT(4)
#define RUNTIME_VALID_PRESENTATION_DELAY BIT(5)
#define RUNTIME_VALID_COUNTERS BIT(6)

#define PHY_MASK_SUPPORTED                                                                           \
	(BT_GAP_LE_PHY_1M | BT_GAP_LE_PHY_2M |                                                   \
	 (IS_ENABLED(CONFIG_BT_CTLR_PHY_CODED) ? BT_GAP_LE_PHY_CODED : 0U))

#define DIRECTION_MASK_SUPPORTED                                                                     \
	((IS_ENABLED(CONFIG_BT_AUDIO_RX) ? BT_AUDIO_DIR_SINK : 0U) |                                \
	 (IS_ENABLED(CONFIG_BT_AUDIO_TX) ? BT_AUDIO_DIR_SOURCE : 0U))

#define ACL_INTERVAL_MIN_US 7500U
#define ACL_INTERVAL_MAX_US 4000000U
#define ACL_INTERVAL_RESOLUTION_US 1250U
#define ACL_LATENCY_MAX 499U
#define ACL_TIMEOUT_MIN_MS 100U
#define ACL_TIMEOUT_MAX_MS 32000U

#define SETTINGS_ROOT "wireless_audio"
#define SETTINGS_ACL_KEY SETTINGS_ROOT "/acl"
#define SETTINGS_RADIO_KEY SETTINGS_ROOT "/radio"
#define SETTINGS_QOS_KEY SETTINGS_ROOT "/qos"

enum command_status {
	COMMAND_STATUS_APPLIED = 0,
	COMMAND_STATUS_PENDING = 1,
	COMMAND_STATUS_RECONNECT_REQUIRED = 2,
	COMMAND_STATUS_STREAM_RECONFIGURATION_REQUIRED = 3,
	COMMAND_STATUS_UNSUPPORTED = 4,
	COMMAND_STATUS_INVALID = 5,
	COMMAND_STATUS_BUSY = 6,
	COMMAND_STATUS_FAILED = 7,
};

enum error_domain {
	ERROR_DOMAIN_NONE = 0,
	ERROR_DOMAIN_PROTOCOL = 1,
	ERROR_DOMAIN_PLATFORM = 2,
	ERROR_DOMAIN_HCI = 3,
};

/** Value-attribute indexes within wireless_audio_configuration_svc. */
enum service_attribute_index {
	RESPONSE_ATTRIBUTE_INDEX = 4,
	RUNTIME_STATE_ATTRIBUTE_INDEX = 7,
};

struct policy_state {
	wireless_audio_configuration_acl_connection_policy_t acl;
	wireless_audio_configuration_acl_radio_policy_t radio;
	wireless_audio_configuration_unicast_server_qos_preferences_t qos;
	bool acl_persisted;
	bool radio_persisted;
	bool qos_persisted;
};

struct command_context {
	struct k_work work;
	struct bt_conn *conn;
	wireless_audio_configuration_configuration_command_t command;
	wireless_audio_configuration_configuration_response_t response;
	uint8_t encoded_response[RESPONSE_ENCODED_MAX];
	struct bt_gatt_indicate_params indication;
	bool busy;
};

static struct policy_state policies;
static wireless_audio_configuration_runtime_state_t runtime_state;
static struct bt_conn *audio_conn;
static struct command_context command_context;
static struct k_work runtime_notify_work;
static bool initialized;

K_MUTEX_DEFINE(policy_mutex);

extern const struct bt_gatt_service_static wireless_audio_configuration_svc;

static void runtime_state_changed_locked(void);
static int apply_radio_policy(struct bt_conn *conn);
static void command_work_handler(struct k_work *work);

/** Clear negotiated CIS QoS fields while the policy mutex is held. */
static void runtime_qos_clear_locked(void)
{
	runtime_state.validity_flags &=
		~(RUNTIME_VALID_QOS | RUNTIME_VALID_PRESENTATION_DELAY);
	runtime_state.iso_sdu_interval_us = 0U;
	runtime_state.iso_framing = 0U;
	runtime_state.iso_phy = 0U;
	runtime_state.iso_retransmission_number = 0U;
	runtime_state.iso_maximum_sdu_octets = 0U;
	runtime_state.iso_maximum_transport_latency_ms = 0U;
	runtime_state.presentation_delay_us = 0U;
}

/** Clear negotiated codec and dependent QoS fields while the policy mutex is held. */
static void runtime_codec_clear_locked(void)
{
	runtime_state.validity_flags &= ~RUNTIME_VALID_CODEC;
	runtime_state.lc3_sampling_frequency_hz = 0U;
	runtime_state.lc3_frame_duration_us = 0U;
	runtime_state.lc3_octets_per_frame = 0U;
	runtime_state.lc3_frame_blocks_per_sdu = 0U;
	runtime_state.lc3_channel_allocation = 0U;
	runtime_qos_clear_locked();
}

/** Count a locally accepted ACL parameter update request. */
static void acl_adjustment_observed(void)
{
	k_mutex_lock(&policy_mutex, K_FOREVER);
	runtime_state.acl_adjustment_count++;
	runtime_state.validity_flags |= RUNTIME_VALID_COUNTERS;
	runtime_state_changed_locked();
	k_mutex_unlock(&policy_mutex);
}

/** Populate the compiled policy defaults that preserve the firmware's previous behavior. */
static void policy_defaults_set(struct policy_state *state)
{
	memset(state, 0, sizeof(*state));
	state->acl.type =
		WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_ADAPTIVE_LINEAR_ACL_POLICY;
	state->acl.policy.adaptive_linear_acl_policy =
		(wireless_audio_configuration_adaptive_linear_acl_policy_t){
			.minimum_interval_us = CONFIG_BLE_ACL_CONN_INTERVAL * ACL_INTERVAL_RESOLUTION_US,
			.maximum_interval_us =
				CONFIG_BLE_ACL_CONN_INTERVAL_SLOW * ACL_INTERVAL_RESOLUTION_US,
			.underrun_increase_step_us = 4U * ACL_INTERVAL_RESOLUTION_US,
			.recovery_decrease_step_us = 4U * ACL_INTERVAL_RESOLUTION_US,
			.recovery_period_ms = 1000U,
			.minimum_update_period_ms = 1000U,
			.peripheral_latency = CONFIG_BLE_ACL_SLAVE_LATENCY,
			.supervision_timeout_ms = CONFIG_BLE_ACL_SUP_TIMEOUT * 10U,
		};

	state->radio.type = WIRELESS_AUDIO_CONFIGURATION_ACL_RADIO_POLICY_PREFERRED_ACL_RADIO_POLICY;
	state->radio.policy.preferred_acl_radio_policy =
		(wireless_audio_configuration_preferred_acl_radio_policy_t){
			.transmit_phy_mask = BT_GAP_LE_PHY_2M,
			.receive_phy_mask = BT_GAP_LE_PHY_2M,
			.transmit_max_data_octets = BT_GAP_DATA_LEN_MAX,
			.transmit_max_time_us = BT_GAP_DATA_TIME_MAX,
		};

	state->qos = (wireless_audio_configuration_unicast_server_qos_preferences_t){
		.direction_mask = DIRECTION_MASK_SUPPORTED,
		.unframed_supported = 1U,
		.preferred_phy_mask = BT_GAP_LE_PHY_2M,
		.preferred_retransmission_number = CONFIG_BT_AUDIO_RETRANSMITS,
		.maximum_transport_latency_ms = 10U,
		.minimum_presentation_delay_us = CONFIG_AUDIO_MIN_PRES_DLY_US,
		.maximum_presentation_delay_us = CONFIG_AUDIO_MAX_PRES_DLY_US,
		.preferred_minimum_presentation_delay_us =
			CONFIG_BT_AUDIO_PREFERRED_MIN_PRES_DLY_US,
		.preferred_maximum_presentation_delay_us =
			CONFIG_BT_AUDIO_PREFERRED_MAX_PRES_DLY_US,
	};
}

/** Return whether a connection-parameter tuple satisfies Core timing constraints. */
static bool acl_parameters_valid(uint32_t minimum_us, uint32_t maximum_us, uint16_t latency,
				 uint32_t timeout_ms)
{
	if (minimum_us < ACL_INTERVAL_MIN_US || maximum_us > ACL_INTERVAL_MAX_US ||
	    minimum_us > maximum_us || minimum_us % ACL_INTERVAL_RESOLUTION_US != 0U ||
	    maximum_us % ACL_INTERVAL_RESOLUTION_US != 0U || latency > ACL_LATENCY_MAX ||
	    timeout_ms < ACL_TIMEOUT_MIN_MS || timeout_ms > ACL_TIMEOUT_MAX_MS ||
	    timeout_ms % 10U != 0U) {
		return false;
	}

	const uint64_t minimum_timeout_us =
		2ULL * (1ULL + latency) * (uint64_t)maximum_us;
	return (uint64_t)timeout_ms * 1000ULL > minimum_timeout_us;
}

/** Validate a decoded ACL policy before it reaches the policy engine. */
static bool acl_policy_valid(const wireless_audio_configuration_acl_connection_policy_t *policy)
{
	switch (policy->type) {
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_CONTROLLER_DEFAULT_ACL_POLICY:
		return policy->policy.controller_default_acl_policy.reserved == 0U;
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_FIXED_ACL_POLICY: {
		const wireless_audio_configuration_fixed_acl_policy_t *fixed =
			&policy->policy.fixed_acl_policy;
		return acl_parameters_valid(fixed->interval_us, fixed->interval_us,
					    fixed->peripheral_latency,
					    fixed->supervision_timeout_ms);
	}
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_PREFERRED_RANGE_ACL_POLICY: {
		const wireless_audio_configuration_preferred_range_acl_policy_t *range =
			&policy->policy.preferred_range_acl_policy;
		return acl_parameters_valid(range->minimum_interval_us, range->maximum_interval_us,
					    range->peripheral_latency,
					    range->supervision_timeout_ms);
	}
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_ADAPTIVE_LINEAR_ACL_POLICY: {
		const wireless_audio_configuration_adaptive_linear_acl_policy_t *adaptive =
			&policy->policy.adaptive_linear_acl_policy;
		return acl_parameters_valid(adaptive->minimum_interval_us,
					    adaptive->maximum_interval_us,
					    adaptive->peripheral_latency,
					    adaptive->supervision_timeout_ms) &&
		       adaptive->underrun_increase_step_us != 0U &&
		       adaptive->recovery_decrease_step_us != 0U &&
		       adaptive->underrun_increase_step_us % ACL_INTERVAL_RESOLUTION_US == 0U &&
		       adaptive->recovery_decrease_step_us % ACL_INTERVAL_RESOLUTION_US == 0U &&
		       adaptive->recovery_period_ms != 0U &&
		       adaptive->minimum_update_period_ms != 0U;
	}
	default:
		return false;
	}
}

/** Validate an ACL PHY and Data Length Extension policy. */
static bool radio_policy_valid(const wireless_audio_configuration_acl_radio_policy_t *policy)
{
	if (policy->type ==
	    WIRELESS_AUDIO_CONFIGURATION_ACL_RADIO_POLICY_AUTOMATIC_ACL_RADIO_POLICY) {
		return policy->policy.automatic_acl_radio_policy.reserved == 0U;
	}
	if (policy->type !=
	    WIRELESS_AUDIO_CONFIGURATION_ACL_RADIO_POLICY_PREFERRED_ACL_RADIO_POLICY) {
		return false;
	}

	const wireless_audio_configuration_preferred_acl_radio_policy_t *preferred =
		&policy->policy.preferred_acl_radio_policy;
	if ((preferred->transmit_phy_mask & ~PHY_MASK_SUPPORTED) != 0U ||
	    (preferred->receive_phy_mask & ~PHY_MASK_SUPPORTED) != 0U) {
		return false;
	}
	if (preferred->transmit_max_data_octets != 0U &&
	    (preferred->transmit_max_data_octets < BT_GAP_DATA_LEN_DEFAULT ||
	     preferred->transmit_max_data_octets > BT_GAP_DATA_LEN_MAX)) {
		return false;
	}
	return preferred->transmit_max_time_us == 0U ||
	       (preferred->transmit_max_time_us >= BT_GAP_DATA_TIME_DEFAULT &&
		preferred->transmit_max_time_us <= BT_GAP_DATA_TIME_MAX);
}

/** Validate preferences represented by the standard ASCS QoS preference tuple. */
static bool qos_preferences_valid(
	const wireless_audio_configuration_unicast_server_qos_preferences_t *preferences)
{
	if (preferences->direction_mask == 0U ||
	    (preferences->direction_mask & ~DIRECTION_MASK_SUPPORTED) != 0U ||
	    preferences->unframed_supported > 1U || preferences->preferred_phy_mask == 0U ||
	    (preferences->preferred_phy_mask & ~PHY_MASK_SUPPORTED) != 0U ||
	    preferences->maximum_transport_latency_ms < BT_ISO_LATENCY_MIN ||
	    preferences->maximum_transport_latency_ms > BT_ISO_LATENCY_MAX ||
	    preferences->minimum_presentation_delay_us > preferences->maximum_presentation_delay_us ||
	    preferences->maximum_presentation_delay_us > BT_AUDIO_PD_MAX) {
		return false;
	}

	const uint32_t preferred_min = preferences->preferred_minimum_presentation_delay_us;
	const uint32_t preferred_max = preferences->preferred_maximum_presentation_delay_us;
	if (preferred_min == BT_AUDIO_PD_PREF_NONE && preferred_max == BT_AUDIO_PD_PREF_NONE) {
		return true;
	}
	return preferred_min >= preferences->minimum_presentation_delay_us &&
	       preferred_min <= preferred_max &&
	       preferred_max <= preferences->maximum_presentation_delay_us;
}

/** Encode and persist one policy value under a Zephyr settings key. */
static int policy_save(const char *key, const void *policy, uint8_t section)
{
	uint8_t encoded[ACL_POLICY_ENCODED_MAX];
	size_t written = 0U;
	protocol_status_t status;

	if (section == 0U) {
		status = wireless_audio_configuration_acl_connection_policy_encode(policy, encoded,
									 sizeof(encoded), &written);
	} else if (section == 1U) {
		status = wireless_audio_configuration_acl_radio_policy_encode(policy, encoded,
								      sizeof(encoded), &written);
	} else {
		status = wireless_audio_configuration_unicast_server_qos_preferences_encode(
			policy, encoded, sizeof(encoded), &written);
	}
	if (status != PROTOCOL_OK) {
		return -EINVAL;
	}
	return settings_save_one(key, encoded, written);
}

/** Apply the active ACL radio policy to a referenced audio connection. */
static int apply_radio_policy(struct bt_conn *conn)
{
	wireless_audio_configuration_acl_radio_policy_t policy;

	k_mutex_lock(&policy_mutex, K_FOREVER);
	policy = policies.radio;
	k_mutex_unlock(&policy_mutex);

	if (policy.type ==
	    WIRELESS_AUDIO_CONFIGURATION_ACL_RADIO_POLICY_AUTOMATIC_ACL_RADIO_POLICY) {
		return 0;
	}

	const wireless_audio_configuration_preferred_acl_radio_policy_t *preferred =
		&policy.policy.preferred_acl_radio_policy;
	int first_error = 0;
	if (preferred->transmit_phy_mask != 0U || preferred->receive_phy_mask != 0U) {
		const struct bt_conn_le_phy_param phy = {
			.options = BT_CONN_LE_PHY_OPT_NONE,
			.pref_tx_phy = preferred->transmit_phy_mask,
			.pref_rx_phy = preferred->receive_phy_mask,
		};
		first_error = bt_conn_le_phy_update(conn, &phy);
		if (first_error != 0) {
			LOG_WRN("ACL PHY preference request failed: %d", first_error);
		}
	}

	if (preferred->transmit_max_data_octets != 0U || preferred->transmit_max_time_us != 0U) {
		const struct bt_conn_le_data_len_param data_length = {
			.tx_max_len = preferred->transmit_max_data_octets != 0U
					      ? preferred->transmit_max_data_octets
					      : BT_GAP_DATA_LEN_DEFAULT,
			.tx_max_time = preferred->transmit_max_time_us != 0U
					       ? preferred->transmit_max_time_us
					       : BT_GAP_DATA_TIME_DEFAULT,
		};
		const int err = bt_conn_le_data_len_update(conn, &data_length);
		if (err != 0) {
			LOG_WRN("ACL data length preference request failed: %d", err);
			if (first_error == 0) {
				first_error = err;
			}
		}
	}
	return first_error;
}

/** Increment the runtime sequence and coalesce a notification on the system workqueue. */
static void runtime_state_changed_locked(void)
{
	runtime_state.sequence++;
	(void)k_work_submit(&runtime_notify_work);
}

/** Encode and notify the most recent runtime snapshot. */
static void runtime_notify_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t encoded[RUNTIME_STATE_ENCODED_SIZE];
	size_t written = 0U;
	wireless_audio_configuration_runtime_state_t snapshot;

	k_mutex_lock(&policy_mutex, K_FOREVER);
	snapshot = runtime_state;
	k_mutex_unlock(&policy_mutex);

	if (wireless_audio_configuration_runtime_state_encode(&snapshot, encoded, sizeof(encoded),
								 &written) != PROTOCOL_OK) {
		LOG_ERR("Failed to encode wireless audio runtime state");
		return;
	}

	const int err = bt_gatt_notify(
		NULL, &wireless_audio_configuration_svc.attrs[RUNTIME_STATE_ATTRIBUTE_INDEX], encoded,
		written);
	if (err != 0 && err != -ENOTCONN) {
		LOG_DBG("Runtime state notification skipped: %d", err);
	}
}

/** Fill the custom capability value without duplicating PACS codec capabilities. */
static wireless_audio_configuration_capabilities_t capabilities_get(void)
{
	return (wireless_audio_configuration_capabilities_t){
		.protocol_version = 1U,
		.supported_section_mask = ALL_SECTION_BITS,
		.supported_command_mask = BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4),
		.supported_acl_policy_mask = BIT(0) | BIT(1) | BIT(2) | BIT(3),
		.supported_phy_mask = PHY_MASK_SUPPORTED,
		.minimum_acl_interval_us = ACL_INTERVAL_MIN_US,
		.maximum_acl_interval_us = ACL_INTERVAL_MAX_US,
		.acl_interval_resolution_us = ACL_INTERVAL_RESOLUTION_US,
		.maximum_acl_peripheral_latency = ACL_LATENCY_MAX,
		.minimum_acl_supervision_timeout_ms = ACL_TIMEOUT_MIN_MS,
		.maximum_acl_supervision_timeout_ms = ACL_TIMEOUT_MAX_MS,
		.minimum_acl_data_octets = BT_GAP_DATA_LEN_DEFAULT,
		.maximum_acl_data_octets = BT_GAP_DATA_LEN_MAX,
		.minimum_acl_data_time_us = BT_GAP_DATA_TIME_DEFAULT,
		.maximum_acl_data_time_us = BT_GAP_DATA_TIME_MAX,
		.supported_audio_direction_mask = DIRECTION_MASK_SUPPORTED,
		.maximum_preferred_retransmission_number = UINT8_MAX,
		.maximum_transport_latency_ms = BT_ISO_LATENCY_MAX,
		.minimum_presentation_delay_us = 0U,
		.maximum_presentation_delay_us = BT_AUDIO_PD_MAX,
		.feature_flags = BIT(0) | BIT(1) | BIT(2),
	};
}

/** Build a command-result response. */
static void command_result_set(wireless_audio_configuration_configuration_response_t *response,
			       uint16_t request_id, enum command_status status,
			       enum error_domain domain, int error, uint32_t restart_mask)
{
	response->request_id = request_id;
	wireless_audio_configuration_configuration_response_set_payload_command_result(
		response, (wireless_audio_configuration_command_result_t){
			  .status = status,
			  .error_domain = domain,
			  .error_code = error,
			  .restart_required_mask = restart_mask,
		  });
}

/** Process an ACL policy mutation and return its correlated response. */
static void set_acl_policy_process(
	const wireless_audio_configuration_configuration_command_t *command,
	wireless_audio_configuration_configuration_response_t *response)
{
	const wireless_audio_configuration_set_acl_connection_policy_t *request =
		&command->operation.set_acl_connection_policy;
	if (request->persist > 1U || !acl_policy_valid(&request->policy)) {
		command_result_set(response, command->request_id, COMMAND_STATUS_INVALID,
				   ERROR_DOMAIN_PROTOCOL, -EINVAL, 0U);
		return;
	}

	int err = 0;
	if (request->persist != 0U) {
		err = policy_save(SETTINGS_ACL_KEY, &request->policy, 0U);
		if (err != 0) {
			command_result_set(response, command->request_id, COMMAND_STATUS_FAILED,
					   ERROR_DOMAIN_PLATFORM, err, 0U);
			return;
		}
	}

	k_mutex_lock(&policy_mutex, K_FOREVER);
	policies.acl = request->policy;
	policies.acl_persisted = request->persist != 0U;
	const bool connected = audio_conn != NULL;
	k_mutex_unlock(&policy_mutex);

	err = bt_mgmt_ci_policy_set(&request->policy);
	if (err != 0) {
		command_result_set(response, command->request_id, COMMAND_STATUS_FAILED,
				   ERROR_DOMAIN_PLATFORM, err, 0U);
		return;
	}

	if (connected && request->policy.type ==
				 WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_CONTROLLER_DEFAULT_ACL_POLICY) {
		command_result_set(response, command->request_id,
				   COMMAND_STATUS_RECONNECT_REQUIRED, ERROR_DOMAIN_NONE, 0,
				   ACL_SECTION_BIT);
	} else {
		command_result_set(response, command->request_id,
				   connected ? COMMAND_STATUS_PENDING : COMMAND_STATUS_APPLIED,
				   ERROR_DOMAIN_NONE, 0, 0U);
	}
}

/** Process an ACL radio policy mutation. */
static void set_radio_policy_process(
	const wireless_audio_configuration_configuration_command_t *command,
	wireless_audio_configuration_configuration_response_t *response)
{
	const wireless_audio_configuration_set_acl_radio_policy_t *request =
		&command->operation.set_acl_radio_policy;
	if (request->persist > 1U || !radio_policy_valid(&request->policy)) {
		command_result_set(response, command->request_id, COMMAND_STATUS_INVALID,
				   ERROR_DOMAIN_PROTOCOL, -EINVAL, 0U);
		return;
	}

	int err = 0;
	if (request->persist != 0U) {
		err = policy_save(SETTINGS_RADIO_KEY, &request->policy, 1U);
		if (err != 0) {
			command_result_set(response, command->request_id, COMMAND_STATUS_FAILED,
					   ERROR_DOMAIN_PLATFORM, err, 0U);
			return;
		}
	}

	struct bt_conn *conn = NULL;
	k_mutex_lock(&policy_mutex, K_FOREVER);
	policies.radio = request->policy;
	policies.radio_persisted = request->persist != 0U;
	if (audio_conn != NULL) {
		conn = bt_conn_ref(audio_conn);
	}
	k_mutex_unlock(&policy_mutex);

	if (conn == NULL) {
		command_result_set(response, command->request_id, COMMAND_STATUS_APPLIED,
				   ERROR_DOMAIN_NONE, 0, 0U);
		return;
	}
	if (request->policy.type ==
	    WIRELESS_AUDIO_CONFIGURATION_ACL_RADIO_POLICY_AUTOMATIC_ACL_RADIO_POLICY) {
		bt_conn_unref(conn);
		command_result_set(response, command->request_id,
				   COMMAND_STATUS_RECONNECT_REQUIRED, ERROR_DOMAIN_NONE, 0,
				   RADIO_SECTION_BIT);
		return;
	}

	err = apply_radio_policy(conn);
	bt_conn_unref(conn);
	command_result_set(response, command->request_id,
			   err == 0 ? COMMAND_STATUS_PENDING : COMMAND_STATUS_FAILED,
			   err == 0 ? ERROR_DOMAIN_NONE : ERROR_DOMAIN_PLATFORM, err, 0U);
}

/** Process a Unicast Server QoS preference mutation. */
static void set_qos_preferences_process(
	const wireless_audio_configuration_configuration_command_t *command,
	wireless_audio_configuration_configuration_response_t *response)
{
	const wireless_audio_configuration_set_unicast_server_qos_preferences_t *request =
		&command->operation.set_unicast_server_qos_preferences;
	if (request->persist > 1U || !qos_preferences_valid(&request->preferences)) {
		command_result_set(response, command->request_id, COMMAND_STATUS_INVALID,
				   ERROR_DOMAIN_PROTOCOL, -EINVAL, 0U);
		return;
	}

	int err = 0;
	if (request->persist != 0U) {
		err = policy_save(SETTINGS_QOS_KEY, &request->preferences, 2U);
		if (err != 0) {
			command_result_set(response, command->request_id, COMMAND_STATUS_FAILED,
					   ERROR_DOMAIN_PLATFORM, err, 0U);
			return;
		}
	}

	k_mutex_lock(&policy_mutex, K_FOREVER);
	policies.qos = request->preferences;
	policies.qos_persisted = request->persist != 0U;
	const bool configured_stream =
		runtime_state.lifecycle_state >= WIRELESS_AUDIO_LIFECYCLE_CODEC_CONFIGURED;
	k_mutex_unlock(&policy_mutex);

	command_result_set(response, command->request_id,
			   configured_stream ? COMMAND_STATUS_STREAM_RECONFIGURATION_REQUIRED
					     : COMMAND_STATUS_APPLIED,
			   ERROR_DOMAIN_NONE, 0,
			   configured_stream ? QOS_SECTION_BIT : 0U);
}

/** Process a policy query. */
static void get_configuration_process(
	const wireless_audio_configuration_configuration_command_t *command,
	wireless_audio_configuration_configuration_response_t *response)
{
	const uint8_t section = command->operation.get_configuration.section;
	response->request_id = command->request_id;

	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (section == 0U) {
		wireless_audio_configuration_configuration_response_set_payload_configured_acl_connection_policy(
			response,
			(wireless_audio_configuration_configured_acl_connection_policy_t){
				.persisted = policies.acl_persisted,
				.policy = policies.acl,
			});
	} else if (section == 1U) {
		wireless_audio_configuration_configuration_response_set_payload_configured_acl_radio_policy(
			response,
			(wireless_audio_configuration_configured_acl_radio_policy_t){
				.persisted = policies.radio_persisted,
				.policy = policies.radio,
			});
	} else if (section == 2U) {
		wireless_audio_configuration_configuration_response_set_payload_configured_unicast_server_qos_preferences(
			response,
			(wireless_audio_configuration_configured_unicast_server_qos_preferences_t){
				.persisted = policies.qos_persisted,
				.preferences = policies.qos,
			});
	} else {
		k_mutex_unlock(&policy_mutex);
		command_result_set(response, command->request_id, COMMAND_STATUS_UNSUPPORTED,
				   ERROR_DOMAIN_PROTOCOL, -ENOTSUP, 0U);
		return;
	}
	k_mutex_unlock(&policy_mutex);
}

/** Restore selected sections to compiled defaults and erase persisted overrides. */
static void restore_defaults_process(
	const wireless_audio_configuration_configuration_command_t *command,
	wireless_audio_configuration_configuration_response_t *response)
{
	uint32_t mask = command->operation.restore_defaults.section_mask;
	if (mask == 0U) {
		mask = ALL_SECTION_BITS;
	}
	if ((mask & ~ALL_SECTION_BITS) != 0U) {
		command_result_set(response, command->request_id, COMMAND_STATUS_UNSUPPORTED,
				   ERROR_DOMAIN_PROTOCOL, -ENOTSUP, 0U);
		return;
	}

	struct policy_state defaults;
	policy_defaults_set(&defaults);
	int err = 0;
	if ((mask & ACL_SECTION_BIT) != 0U) {
		err = settings_delete(SETTINGS_ACL_KEY);
		if (err == -ENOENT) {
			err = 0;
		}
	}
	if (err == 0 && (mask & RADIO_SECTION_BIT) != 0U) {
		err = settings_delete(SETTINGS_RADIO_KEY);
		if (err == -ENOENT) {
			err = 0;
		}
	}
	if (err == 0 && (mask & QOS_SECTION_BIT) != 0U) {
		err = settings_delete(SETTINGS_QOS_KEY);
		if (err == -ENOENT) {
			err = 0;
		}
	}
	if (err != 0) {
		command_result_set(response, command->request_id, COMMAND_STATUS_FAILED,
				   ERROR_DOMAIN_PLATFORM, err, 0U);
		return;
	}

	struct bt_conn *conn = NULL;
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if ((mask & ACL_SECTION_BIT) != 0U) {
		policies.acl = defaults.acl;
		policies.acl_persisted = false;
	}
	if ((mask & RADIO_SECTION_BIT) != 0U) {
		policies.radio = defaults.radio;
		policies.radio_persisted = false;
	}
	if ((mask & QOS_SECTION_BIT) != 0U) {
		policies.qos = defaults.qos;
		policies.qos_persisted = false;
	}
	if (audio_conn != NULL) {
		conn = bt_conn_ref(audio_conn);
	}
	const bool configured_stream =
		runtime_state.lifecycle_state >= WIRELESS_AUDIO_LIFECYCLE_CODEC_CONFIGURED;
	k_mutex_unlock(&policy_mutex);

	if ((mask & ACL_SECTION_BIT) != 0U) {
		(void)bt_mgmt_ci_policy_set(&defaults.acl);
	}
	if (conn != NULL && (mask & RADIO_SECTION_BIT) != 0U) {
		(void)apply_radio_policy(conn);
	}
	if (conn != NULL) {
		bt_conn_unref(conn);
	}

	const uint32_t restart_mask = configured_stream && (mask & QOS_SECTION_BIT) != 0U
					      ? QOS_SECTION_BIT
					      : 0U;
	command_result_set(response, command->request_id,
			   restart_mask != 0U ? COMMAND_STATUS_STREAM_RECONFIGURATION_REQUIRED
					      : COMMAND_STATUS_APPLIED,
			   ERROR_DOMAIN_NONE, 0, restart_mask);
}

/** Dispatch one decoded command without exposing generator callbacks to service logic. */
static void command_process(const wireless_audio_configuration_configuration_command_t *command,
			    wireless_audio_configuration_configuration_response_t *response)
{
	switch (command->type) {
	case WIRELESS_AUDIO_CONFIGURATION_CONFIGURATION_COMMAND_SET_ACL_CONNECTION_POLICY:
		set_acl_policy_process(command, response);
		break;
	case WIRELESS_AUDIO_CONFIGURATION_CONFIGURATION_COMMAND_SET_ACL_RADIO_POLICY:
		set_radio_policy_process(command, response);
		break;
	case WIRELESS_AUDIO_CONFIGURATION_CONFIGURATION_COMMAND_SET_UNICAST_SERVER_QOS_PREFERENCES:
		set_qos_preferences_process(command, response);
		break;
	case WIRELESS_AUDIO_CONFIGURATION_CONFIGURATION_COMMAND_GET_CONFIGURATION:
		get_configuration_process(command, response);
		break;
	case WIRELESS_AUDIO_CONFIGURATION_CONFIGURATION_COMMAND_RESTORE_DEFAULTS:
		restore_defaults_process(command, response);
		break;
	default:
		command_result_set(response, command->request_id, COMMAND_STATUS_UNSUPPORTED,
				   ERROR_DOMAIN_PROTOCOL, -ENOTSUP, 0U);
		break;
	}
}

/** Release the single command/indication slot after completion or failure. */
static void command_context_release(void)
{
	struct bt_conn *conn;

	k_mutex_lock(&policy_mutex, K_FOREVER);
	conn = command_context.conn;
	command_context.conn = NULL;
	command_context.busy = false;
	k_mutex_unlock(&policy_mutex);
	if (conn != NULL) {
		bt_conn_unref(conn);
	}
}

/** Complete a correlated response indication. */
static void response_indication_complete(struct bt_conn *conn,
					 struct bt_gatt_indicate_params *params, uint8_t err)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	if (err != 0U) {
		LOG_WRN("Wireless audio response indication failed: 0x%02x", err);
	}
}

/** Destroy callback used as the authoritative end of an indication lifetime. */
static void response_indication_destroy(struct bt_gatt_indicate_params *params)
{
	ARG_UNUSED(params);
	command_context_release();
}

/** Execute a command outside the Bluetooth receive callback and indicate its response. */
static void command_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	command_process(&command_context.command, &command_context.response);

	size_t written = 0U;
	if (wireless_audio_configuration_configuration_response_encode(
		    &command_context.response, command_context.encoded_response,
		    sizeof(command_context.encoded_response), &written) != PROTOCOL_OK) {
		LOG_ERR("Failed to encode wireless audio configuration response");
		command_context_release();
		return;
	}

	command_context.indication = (struct bt_gatt_indicate_params){
		.attr = &wireless_audio_configuration_svc.attrs[RESPONSE_ATTRIBUTE_INDEX],
		.func = response_indication_complete,
		.destroy = response_indication_destroy,
		.data = command_context.encoded_response,
		.len = written,
	};
	const int err = bt_gatt_indicate(command_context.conn, &command_context.indication);
	if (err != 0) {
		LOG_WRN("Failed to queue wireless audio response indication: %d", err);
		command_context_release();
	}
}

/** Decode and enqueue a command after verifying the indication channel is available. */
static ssize_t write_command(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buffer, uint16_t length, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);
	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (length > COMMAND_ENCODED_MAX) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (!bt_gatt_is_subscribed(conn,
				   &wireless_audio_configuration_svc.attrs[RESPONSE_ATTRIBUTE_INDEX],
				   BT_GATT_CCC_INDICATE)) {
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}

	wireless_audio_configuration_configuration_command_t decoded;
	size_t consumed = 0U;
	if (wireless_audio_configuration_configuration_command_decode(
		    &decoded, buffer, length, &consumed) != PROTOCOL_OK || consumed != length) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (command_context.busy) {
		k_mutex_unlock(&policy_mutex);
		return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
	}
	command_context.busy = true;
	command_context.conn = bt_conn_ref(conn);
	command_context.command = decoded;
	k_mutex_unlock(&policy_mutex);

	if (k_work_submit(&command_context.work) < 0) {
		command_context_release();
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return length;
}

/** Encode the current runtime state for a GATT read or long read. */
static ssize_t read_runtime_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buffer, uint16_t length, uint16_t offset)
{
	uint8_t encoded[RUNTIME_STATE_ENCODED_SIZE];
	size_t written = 0U;
	wireless_audio_configuration_runtime_state_t snapshot;

	k_mutex_lock(&policy_mutex, K_FOREVER);
	snapshot = runtime_state;
	k_mutex_unlock(&policy_mutex);
	if (wireless_audio_configuration_runtime_state_encode(&snapshot, encoded, sizeof(encoded),
								 &written) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return bt_gatt_attr_read(conn, attr, buffer, length, offset, encoded, written);
}

/** Encode custom policy capabilities for a GATT read or long read. */
static ssize_t read_capabilities(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buffer, uint16_t length, uint16_t offset)
{
	uint8_t encoded[CAPABILITIES_ENCODED_SIZE];
	size_t written = 0U;
	const wireless_audio_configuration_capabilities_t capabilities = capabilities_get();

	if (wireless_audio_configuration_capabilities_encode(&capabilities, encoded, sizeof(encoded),
								&written) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return bt_gatt_attr_read(conn, attr, buffer, length, offset, encoded, written);
}

BT_GATT_SERVICE_DEFINE(
	wireless_audio_configuration_svc,
	BT_GATT_PRIMARY_SERVICE(WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_SERVICE_UUID),
	BT_GATT_CHARACTERISTIC(WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_COMMAND_CHARACTERISTIC_UUID,
			       WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_COMMAND_CHARACTERISTIC_PROPERTIES,
			       BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_command, NULL),
	BT_GATT_CHARACTERISTIC(WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_RESPONSE_CHARACTERISTIC_UUID,
			       WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_RESPONSE_CHARACTERISTIC_PROPERTIES,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(
		WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_RUNTIME_STATE_CHARACTERISTIC_UUID,
		WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_RUNTIME_STATE_CHARACTERISTIC_PROPERTIES,
		BT_GATT_PERM_READ_ENCRYPT, read_runtime_state, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_CAPABILITIES_CHARACTERISTIC_UUID,
			       WIRELESS_AUDIO_CONFIGURATION_ZEPHYR_CAPABILITIES_CHARACTERISTIC_PROPERTIES,
			       BT_GATT_PERM_READ_ENCRYPT, read_capabilities, NULL, NULL));

/** Load a persisted policy value using the generated wire representation. */
static int settings_set_callback(const char *name, size_t length, settings_read_cb read_callback,
				 void *callback_argument)
{
	uint8_t encoded[ACL_POLICY_ENCODED_MAX];
	if (length > sizeof(encoded)) {
		return -EINVAL;
	}
	const ssize_t read = read_callback(callback_argument, encoded, length);
	if (read < 0 || (size_t)read != length) {
		return read < 0 ? (int)read : -EIO;
	}

	size_t consumed = 0U;
	protocol_status_t status;
	if (strcmp(name, "acl") == 0) {
		status = wireless_audio_configuration_acl_connection_policy_decode(
			&policies.acl, encoded, length, &consumed);
		if (status != PROTOCOL_OK || consumed != length || !acl_policy_valid(&policies.acl)) {
			return -EINVAL;
		}
		policies.acl_persisted = true;
	} else if (strcmp(name, "radio") == 0) {
		status = wireless_audio_configuration_acl_radio_policy_decode(
			&policies.radio, encoded, length, &consumed);
		if (status != PROTOCOL_OK || consumed != length || !radio_policy_valid(&policies.radio)) {
			return -EINVAL;
		}
		policies.radio_persisted = true;
	} else if (strcmp(name, "qos") == 0) {
		status = wireless_audio_configuration_unicast_server_qos_preferences_decode(
			&policies.qos, encoded, length, &consumed);
		if (status != PROTOCOL_OK || consumed != length || !qos_preferences_valid(&policies.qos)) {
			return -EINVAL;
		}
		policies.qos_persisted = true;
	} else {
		return -ENOENT;
	}
	return 0;
}

/** Apply persisted ACL policy after the settings subsystem finishes loading. */
static int settings_commit_callback(void)
{
	return initialized ? bt_mgmt_ci_policy_set(&policies.acl) : 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(wireless_audio, SETTINGS_ROOT, NULL, settings_set_callback,
			       settings_commit_callback, NULL);

int init_wireless_audio_configuration_service(void)
{
	if (initialized) {
		return -EALREADY;
	}

	policy_defaults_set(&policies);
	memset(&runtime_state, 0, sizeof(runtime_state));
	runtime_state.validity_flags = RUNTIME_VALID_COUNTERS;
	k_work_init(&command_context.work, command_work_handler);
	k_work_init(&runtime_notify_work, runtime_notify_handler);

	int err = bt_mgmt_conn_interval_init();
	if (err != 0) {
		return err;
	}
	bt_mgmt_ci_adjustment_callback_set(acl_adjustment_observed);
	err = bt_mgmt_ci_policy_set(&policies.acl);
	if (err != 0) {
		return err;
	}
	initialized = true;
	return 0;
}

void wireless_audio_configuration_audio_connection_set(struct bt_conn *conn)
{
	if (conn == NULL) {
		return;
	}

	struct bt_conn_info info;
	const int info_err = bt_conn_get_info(conn, &info);
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (audio_conn == conn) {
		k_mutex_unlock(&policy_mutex);
		return;
	}
	if (audio_conn != NULL) {
		bt_conn_unref(audio_conn);
	}
	audio_conn = bt_conn_ref(conn);
	runtime_state.connection_id = bt_conn_index(conn);
	runtime_state.lifecycle_state = WIRELESS_AUDIO_LIFECYCLE_CONNECTED;
	if (info_err == 0) {
		runtime_state.acl_interval_us = BT_CONN_INTERVAL_TO_US(info.le.interval);
		runtime_state.acl_peripheral_latency = info.le.latency;
		runtime_state.acl_supervision_timeout_ms = info.le.timeout * 10U;
		runtime_state.validity_flags |= RUNTIME_VALID_CONNECTION;
#if defined(CONFIG_BT_USER_PHY_UPDATE)
		if (info.le.phy != NULL) {
			runtime_state.transmit_phy = info.le.phy->tx_phy;
			runtime_state.receive_phy = info.le.phy->rx_phy;
			runtime_state.validity_flags |= RUNTIME_VALID_PHY;
		}
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
		if (info.le.data_len != NULL) {
			runtime_state.transmit_data_octets = info.le.data_len->tx_max_len;
			runtime_state.receive_data_octets = info.le.data_len->rx_max_len;
			runtime_state.validity_flags |= RUNTIME_VALID_DATA_LENGTH;
		}
#endif
	}
	runtime_state_changed_locked();
	k_mutex_unlock(&policy_mutex);

	bt_mgmt_ci_on_connected(conn);
	(void)apply_radio_policy(conn);
}

void wireless_audio_configuration_disconnected(struct bt_conn *conn)
{
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (audio_conn == conn) {
		bt_conn_unref(audio_conn);
		audio_conn = NULL;
		const uint32_t sequence = runtime_state.sequence;
		const uint32_t underruns = runtime_state.audio_underrun_count;
		const uint32_t adjustments = runtime_state.acl_adjustment_count;
		memset(&runtime_state, 0, sizeof(runtime_state));
		runtime_state.sequence = sequence;
		runtime_state.audio_underrun_count = underruns;
		runtime_state.acl_adjustment_count = adjustments;
		runtime_state.validity_flags = RUNTIME_VALID_COUNTERS;
		runtime_state_changed_locked();
	}
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_conn_params_updated(struct bt_conn *conn, uint16_t interval,
					       uint16_t latency, uint16_t timeout)
{
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (audio_conn == conn) {
		runtime_state.acl_interval_us = BT_CONN_INTERVAL_TO_US(interval);
		runtime_state.acl_peripheral_latency = latency;
		runtime_state.acl_supervision_timeout_ms = timeout * 10U;
		runtime_state.validity_flags |= RUNTIME_VALID_CONNECTION;
		runtime_state_changed_locked();
	}
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_phy_updated(struct bt_conn *conn,
				      const struct bt_conn_le_phy_info *info)
{
	if (info == NULL) {
		return;
	}
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (audio_conn == conn) {
		runtime_state.transmit_phy = info->tx_phy;
		runtime_state.receive_phy = info->rx_phy;
		runtime_state.validity_flags |= RUNTIME_VALID_PHY;
		runtime_state_changed_locked();
	}
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_data_length_updated(
	struct bt_conn *conn, const struct bt_conn_le_data_len_info *info)
{
	if (info == NULL) {
		return;
	}
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (audio_conn == conn) {
		runtime_state.transmit_data_octets = info->tx_max_len;
		runtime_state.receive_data_octets = info->rx_max_len;
		runtime_state.validity_flags |= RUNTIME_VALID_DATA_LENGTH;
		runtime_state_changed_locked();
	}
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_underrun_observed(uint32_t count)
{
	k_mutex_lock(&policy_mutex, K_FOREVER);
	runtime_state.audio_underrun_count = count;
	runtime_state.validity_flags |= RUNTIME_VALID_COUNTERS;
	runtime_state_changed_locked();
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_qos_preferences_get(enum bt_audio_dir direction,
					       struct bt_bap_qos_cfg_pref *preferences)
{
	if (preferences == NULL) {
		return;
	}
	k_mutex_lock(&policy_mutex, K_FOREVER);
	const wireless_audio_configuration_unicast_server_qos_preferences_t configured =
		policies.qos;
	k_mutex_unlock(&policy_mutex);

	if ((configured.direction_mask & direction) == 0U) {
		LOG_WRN("No QoS preference configured for direction 0x%02x", direction);
	}
	*preferences = (struct bt_bap_qos_cfg_pref){
		.unframed_supported = configured.unframed_supported != 0U,
		.phy = configured.preferred_phy_mask,
		.rtn = configured.preferred_retransmission_number,
		.latency = configured.maximum_transport_latency_ms,
		.pd_min = configured.minimum_presentation_delay_us,
		.pd_max = configured.maximum_presentation_delay_us,
		.pref_pd_min = configured.preferred_minimum_presentation_delay_us,
		.pref_pd_max = configured.preferred_maximum_presentation_delay_us,
	};
}

void wireless_audio_configuration_codec_configured(struct bt_conn *conn, uint8_t stream_id,
					    enum bt_audio_dir direction,
					    const struct bt_audio_codec_cfg *codec)
{
	if (codec == NULL) {
		return;
	}
	wireless_audio_configuration_audio_connection_set(conn);

	const int frequency = bt_audio_codec_cfg_get_freq(codec);
	const int frame_duration = bt_audio_codec_cfg_get_frame_dur(codec);
	const int octets = bt_audio_codec_cfg_get_octets_per_frame(codec);
	const int blocks = bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec, true);
	enum bt_audio_location allocation = BT_AUDIO_LOCATION_MONO_AUDIO;
	const int allocation_status = bt_audio_codec_cfg_get_chan_allocation(codec, &allocation, true);

	k_mutex_lock(&policy_mutex, K_FOREVER);
	runtime_state.stream_id = stream_id;
	runtime_state.direction = direction == BT_AUDIO_DIR_SOURCE ? 1U : 0U;
	runtime_state.lifecycle_state = WIRELESS_AUDIO_LIFECYCLE_CODEC_CONFIGURED;
	runtime_codec_clear_locked();
	if (frequency >= 0 && frame_duration >= 0 && octets >= 0 && blocks >= 0 &&
	    allocation_status == 0) {
		runtime_state.lc3_sampling_frequency_hz =
			bt_audio_codec_cfg_freq_to_freq_hz(frequency);
		runtime_state.lc3_frame_duration_us =
			bt_audio_codec_cfg_frame_dur_to_frame_dur_us(frame_duration);
		runtime_state.lc3_octets_per_frame = octets;
		runtime_state.lc3_frame_blocks_per_sdu = blocks;
		runtime_state.lc3_channel_allocation = allocation;
		runtime_state.validity_flags |= RUNTIME_VALID_CODEC;
	}
	runtime_state_changed_locked();
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_qos_configured(struct bt_conn *conn, uint8_t stream_id,
					  enum bt_audio_dir direction,
					  const struct bt_bap_qos_cfg *qos)
{
	if (qos == NULL) {
		return;
	}
	wireless_audio_configuration_audio_connection_set(conn);
	k_mutex_lock(&policy_mutex, K_FOREVER);
	runtime_state.stream_id = stream_id;
	runtime_state.direction = direction == BT_AUDIO_DIR_SOURCE ? 1U : 0U;
	runtime_state.lifecycle_state = WIRELESS_AUDIO_LIFECYCLE_QOS_CONFIGURED;
	runtime_state.iso_sdu_interval_us = qos->interval;
	runtime_state.iso_framing = qos->framing;
	runtime_state.iso_phy = qos->phy;
	runtime_state.iso_retransmission_number = qos->rtn;
	runtime_state.iso_maximum_sdu_octets = qos->sdu;
	runtime_state.iso_maximum_transport_latency_ms = qos->latency;
	runtime_state.presentation_delay_us = qos->pd;
	runtime_state.validity_flags |= RUNTIME_VALID_QOS | RUNTIME_VALID_PRESENTATION_DELAY;
	runtime_state_changed_locked();
	k_mutex_unlock(&policy_mutex);
}

void wireless_audio_configuration_stream_state_set(struct bt_conn *conn, uint8_t stream_id,
					     enum bt_audio_dir direction,
					     enum wireless_audio_lifecycle_state lifecycle_state)
{
	k_mutex_lock(&policy_mutex, K_FOREVER);
	if (audio_conn == conn) {
		runtime_state.stream_id = stream_id;
		runtime_state.direction = direction == BT_AUDIO_DIR_SOURCE ? 1U : 0U;
		runtime_state.lifecycle_state = lifecycle_state;
		if (lifecycle_state <= WIRELESS_AUDIO_LIFECYCLE_CONNECTED) {
			runtime_codec_clear_locked();
		}
		runtime_state_changed_locked();
	}
	k_mutex_unlock(&policy_mutex);
}
