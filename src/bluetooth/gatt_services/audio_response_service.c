#include "audio_response_service.h"

#include <data_fifo.h>
#include <math.h>
#include <string.h>
#include <zephyr/audio_response_ble.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>

#include "arm_math.h"
#include "audio_datapath.h"
#include "hw_codec.h"

LOG_MODULE_REGISTER(audio_response_service, CONFIG_LOG_DEFAULT_LEVEL);

#define AUDIO_RESPONSE_CAPTURE_SAMPLES 2048
#define AUDIO_RESPONSE_CAPTURE_SAMPLE_RATE 4000U
#define AUDIO_RESPONSE_INITIAL_DROP 128
#define AUDIO_RESPONSE_RESULT_POINTS 9
#define AUDIO_RESPONSE_TRANSFER_CREDITS 1
#define AUDIO_RESPONSE_TRANSFER_TIMEOUT K_SECONDS(30)
#define AUDIO_RESPONSE_STATUS_ENCODED_SIZE 9
#define AUDIO_RESPONSE_RESULT_ENCODED_SIZE (2 + (AUDIO_RESPONSE_RESULT_POINTS * 4))

enum audio_response_transfer_status {
	AUDIO_RESPONSE_TRANSFER_READY = 0,
	AUDIO_RESPONSE_TRANSFER_COMMITTED = 1,
	AUDIO_RESPONSE_TRANSFER_ABORTED = 2,
	AUDIO_RESPONSE_TRANSFER_INVALID_STATE = 3,
	AUDIO_RESPONSE_TRANSFER_INVALID_CHUNK = 4,
	AUDIO_RESPONSE_TRANSFER_INSUFFICIENT_STORAGE = 5,
	AUDIO_RESPONSE_TRANSFER_CHECKSUM_MISMATCH = 6,
	AUDIO_RESPONSE_TRANSFER_TIMED_OUT = 7,
};

struct audio_response_transfer {
	uint16_t id;
	uint32_t total_samples;
	uint32_t sampling_rate;
	uint32_t expected_checksum;
	uint32_t received_samples;
	int16_t *samples;
	bool active;
	bool committed;
};

/**
 * Carries the GATT write metadata and result through the generated union dispatcher.
 */
struct transfer_control_dispatch_context {
	uint16_t write_len;
	ssize_t result;
};

static const uint16_t target_frequencies[AUDIO_RESPONSE_RESULT_POINTS] = {
	40, 60, 90, 135, 203, 304, 456, 683, 1025,
};

static int16_t captured_samples[AUDIO_RESPONSE_CAPTURE_SAMPLES];
static int16_t uploaded_samples[CONFIG_AUDIO_RESPONSE_MAX_SAMPLES];
static q15_t fft_output[AUDIO_RESPONSE_CAPTURE_SAMPLES * 2];
static q15_t magnitudes[AUDIO_RESPONSE_CAPTURE_SAMPLES / 2];
static uint16_t result_frequencies[AUDIO_RESPONSE_RESULT_POINTS];
static uint16_t result_response[AUDIO_RESPONSE_RESULT_POINTS];
static uint8_t result_payload[AUDIO_RESPONSE_RESULT_ENCODED_SIZE];

static struct audio_response_transfer transfer;
static audio_response_config_t pending_config;
static bool measurement_active;
static bool transfer_status_notifications_enabled;
static bool result_notifications_enabled;

static struct k_work measurement_work;
static struct k_work measurement_complete_work;
static struct k_work_delayable transfer_timeout_work;
K_MUTEX_DEFINE(service_mutex);

extern struct data_fifo fifo_rx;
extern const struct bt_gatt_service_static audio_response_svc;

static int notify_transfer_status(enum audio_response_transfer_status status, uint16_t credits);
static void reset_transfer(void);

/**
 * Return a readable transfer status name for logs.
 */
static const char *transfer_status_name(enum audio_response_transfer_status status)
{
	switch (status) {
	case AUDIO_RESPONSE_TRANSFER_READY:
		return "ready";
	case AUDIO_RESPONSE_TRANSFER_COMMITTED:
		return "committed";
	case AUDIO_RESPONSE_TRANSFER_ABORTED:
		return "aborted";
	case AUDIO_RESPONSE_TRANSFER_INVALID_STATE:
		return "invalid_state";
	case AUDIO_RESPONSE_TRANSFER_INVALID_CHUNK:
		return "invalid_chunk";
	case AUDIO_RESPONSE_TRANSFER_INSUFFICIENT_STORAGE:
		return "insufficient_storage";
	case AUDIO_RESPONSE_TRANSFER_CHECKSUM_MISMATCH:
		return "checksum_mismatch";
	case AUDIO_RESPONSE_TRANSFER_TIMED_OUT:
		return "timed_out";
	default:
		return "unknown";
	}
}

/**
 * Return a readable transfer-control command name for logs.
 */
static const char *transfer_control_type_name(audio_response_transfer_control_type_t type)
{
	switch (type) {
	case AUDIO_RESPONSE_TRANSFER_CONTROL_START:
		return "start";
	case AUDIO_RESPONSE_TRANSFER_CONTROL_COMMIT:
		return "commit";
	case AUDIO_RESPONSE_TRANSFER_CONTROL_ABORT:
		return "abort";
	default:
		return "unknown";
	}
}

/**
 * Clear all transfer metadata and make the upload buffer reusable.
 */
static void reset_transfer(void)
{
	LOG_DBG("Reset transfer state: id=%u active=%d committed=%d received=%u/%u", transfer.id,
		transfer.active, transfer.committed, transfer.received_samples, transfer.total_samples);
	memset(&transfer, 0, sizeof(transfer));
}

/**
 * Notify connected clients about the current upload state.
 */
static int notify_transfer_status(enum audio_response_transfer_status status, uint16_t credits)
{
	audio_response_transfer_status_t message = {
		.transfer_id = transfer.id,
		.status = status,
		.next_sample_offset = transfer.received_samples,
		.credits = credits,
	};
	uint8_t payload[AUDIO_RESPONSE_STATUS_ENCODED_SIZE];
	size_t payload_size;

	if (!transfer_status_notifications_enabled) {
		LOG_DBG("Skipping transfer status notification: status=%s id=%u notifications=disabled",
			transfer_status_name(status), transfer.id);
		return -EACCES;
	}

	if (audio_response_transfer_status_encode(&message, payload, sizeof(payload), &payload_size) !=
	    PROTOCOL_OK) {
		LOG_ERR("Failed to encode transfer status: status=%s id=%u offset=%u credits=%u",
			transfer_status_name(status), transfer.id, transfer.received_samples, credits);
		return -EMSGSIZE;
	}

	int ret = bt_gatt_notify(NULL, &audio_response_svc.attrs[6], payload, payload_size);
	if (ret != 0) {
		LOG_WRN("Failed to notify transfer status: status=%s id=%u err=%d",
			transfer_status_name(status), transfer.id, ret);
		return ret;
	}

	LOG_DBG("Notified transfer status: status=%s id=%u offset=%u credits=%u bytes=%u",
		transfer_status_name(status), transfer.id, transfer.received_samples, credits,
		payload_size);
	return 0;
}

/**
 * Reject a transfer operation and report the supplied protocol status.
 */
static ssize_t reject_transfer(enum audio_response_transfer_status status, ssize_t att_error)
{
	LOG_WRN("Rejecting transfer operation: status=%s id=%u active=%d committed=%d received=%u/%u att_error=%zd",
		transfer_status_name(status), transfer.id, transfer.active, transfer.committed,
		transfer.received_samples, transfer.total_samples, att_error);
	(void)notify_transfer_status(status, 0);
	return att_error;
}

/**
 * Start a new audio-buffer upload.
 */
static ssize_t start_transfer(const audio_response_transfer_start_t *start, uint16_t write_len)
{
	LOG_INF("Transfer start requested: id=%u samples=%u rate=%u checksum=0x%08x",
		start->transfer_id, start->total_samples, start->sampling_rate, start->checksum);

	if (measurement_active || transfer.active || start->total_samples == 0 ||
	    start->sampling_rate != CONFIG_AUDIO_SAMPLE_RATE_HZ) {
		LOG_WRN("Transfer start rejected: measurement_active=%d active=%d samples=%u rate=%u expected_rate=%u",
			measurement_active, transfer.active, start->total_samples, start->sampling_rate,
			CONFIG_AUDIO_SAMPLE_RATE_HZ);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_STATE,
				       BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED));
	}

	if (start->total_samples > CONFIG_AUDIO_RESPONSE_MAX_SAMPLES) {
		transfer.id = start->transfer_id;
		LOG_WRN("Transfer start rejected: samples=%u max_samples=%u", start->total_samples,
			CONFIG_AUDIO_RESPONSE_MAX_SAMPLES);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INSUFFICIENT_STORAGE,
				       BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES));
	}

	reset_transfer();
	transfer.id = start->transfer_id;
	transfer.total_samples = start->total_samples;
	transfer.sampling_rate = start->sampling_rate;
	transfer.expected_checksum = start->checksum;
	transfer.samples = uploaded_samples;
	transfer.active = true;
	k_work_reschedule(&transfer_timeout_work, AUDIO_RESPONSE_TRANSFER_TIMEOUT);
	(void)notify_transfer_status(AUDIO_RESPONSE_TRANSFER_READY, AUDIO_RESPONSE_TRANSFER_CREDITS);

	LOG_INF("Transfer started: id=%u samples=%u timeout_ms=%lld", transfer.id,
		transfer.total_samples, k_ticks_to_ms_floor64(AUDIO_RESPONSE_TRANSFER_TIMEOUT.ticks));
	return write_len;
}

/**
 * Validate and retain a completely uploaded audio buffer.
 */
static ssize_t commit_transfer(const audio_response_transfer_commit_t *commit, uint16_t write_len)
{
	LOG_INF("Transfer commit requested: id=%u received=%u/%u", commit->transfer_id,
		transfer.received_samples, transfer.total_samples);

	if (!transfer.active || transfer.committed || commit->transfer_id != transfer.id ||
	    transfer.received_samples != transfer.total_samples) {
		LOG_WRN("Transfer commit rejected: requested_id=%u current_id=%u active=%d committed=%d received=%u/%u",
			commit->transfer_id, transfer.id, transfer.active, transfer.committed,
			transfer.received_samples, transfer.total_samples);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_STATE,
				       BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED));
	}

	uint32_t checksum =
		crc32_ieee((const uint8_t *)transfer.samples, transfer.total_samples * sizeof(int16_t));
	if (checksum != transfer.expected_checksum) {
		LOG_WRN("Transfer checksum mismatch: id=%u expected=0x%08x actual=0x%08x",
			transfer.id, transfer.expected_checksum, checksum);
		(void)notify_transfer_status(AUDIO_RESPONSE_TRANSFER_CHECKSUM_MISMATCH, 0);
		reset_transfer();
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	transfer.committed = true;
	k_work_cancel_delayable(&transfer_timeout_work);
	(void)notify_transfer_status(AUDIO_RESPONSE_TRANSFER_COMMITTED, 0);
	LOG_INF("Transfer committed: id=%u samples=%u checksum=0x%08x", transfer.id,
		transfer.total_samples, checksum);
	return write_len;
}

/**
 * Abort the active or committed upload.
 */
static ssize_t abort_transfer(const audio_response_transfer_abort_t *abort, uint16_t write_len)
{
	LOG_INF("Transfer abort requested: id=%u", abort->transfer_id);

	if ((!transfer.active && !transfer.committed) || abort->transfer_id != transfer.id ||
	    measurement_active) {
		LOG_WRN("Transfer abort rejected: requested_id=%u current_id=%u active=%d committed=%d measurement_active=%d",
			abort->transfer_id, transfer.id, transfer.active, transfer.committed,
			measurement_active);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_STATE,
				       BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED));
	}

	(void)notify_transfer_status(AUDIO_RESPONSE_TRANSFER_ABORTED, 0);
	k_work_cancel_delayable(&transfer_timeout_work);
	LOG_INF("Transfer aborted: id=%u received=%u/%u", transfer.id, transfer.received_samples,
		transfer.total_samples);
	reset_transfer();
	return write_len;
}

/**
 * Handle a typed transfer-start command dispatched by the generated protocol API.
 */
static protocol_status_t dispatch_transfer_start(void *context,
						 const audio_response_transfer_start_t *start)
{
	struct transfer_control_dispatch_context *dispatch_context = context;

	dispatch_context->result = start_transfer(start, dispatch_context->write_len);
	return PROTOCOL_OK;
}

/**
 * Handle a typed transfer-commit command dispatched by the generated protocol API.
 */
static protocol_status_t dispatch_transfer_commit(void *context,
						  const audio_response_transfer_commit_t *commit)
{
	struct transfer_control_dispatch_context *dispatch_context = context;

	dispatch_context->result = commit_transfer(commit, dispatch_context->write_len);
	return PROTOCOL_OK;
}

/**
 * Handle a typed transfer-abort command dispatched by the generated protocol API.
 */
static protocol_status_t dispatch_transfer_abort(void *context,
						 const audio_response_transfer_abort_t *abort)
{
	struct transfer_control_dispatch_context *dispatch_context = context;

	dispatch_context->result = abort_transfer(abort, dispatch_context->write_len);
	return PROTOCOL_OK;
}

/**
 * Decode and execute a transfer-control command.
 */
static ssize_t write_transfer_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	static const audio_response_transfer_control_handler_t handlers = {
		.start = dispatch_transfer_start,
		.commit = dispatch_transfer_commit,
		.abort = dispatch_transfer_abort,
	};
	audio_response_transfer_control_t control;
	struct transfer_control_dispatch_context dispatch_context = {
		.write_len = len,
		.result = BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED),
	};
	size_t bytes_read = 0;
	protocol_status_t status;

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	status = audio_response_transfer_control_decode(&control, buf, len, &bytes_read);
	if (status != PROTOCOL_OK || bytes_read != len) {
		LOG_ERR("Failed to decode transfer control command: status=%d bytes_read=%u len=%u",
			status, bytes_read, len);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_STATE,
				       BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN));
	}

	LOG_DBG("Decoded transfer control command: type=%s len=%u",
		transfer_control_type_name(control.type), len);

	k_mutex_lock(&service_mutex, K_FOREVER);
	status = audio_response_transfer_control_dispatch(&control, &handlers, &dispatch_context);
	k_mutex_unlock(&service_mutex);

	if (status != PROTOCOL_OK) {
		LOG_ERR("Failed to dispatch transfer control command: type=%s status=%d",
			transfer_control_type_name(control.type), status);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_STATE,
				       BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED));
	}
	LOG_DBG("Handled transfer control command: type=%s result=%zd",
		transfer_control_type_name(control.type), dispatch_context.result);
	return dispatch_context.result;
}

/**
 * Decode and append one contiguous chunk to the active transfer.
 */
static ssize_t write_transfer_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	protocol_reader_t reader = { .buffer = buf, .size = len, .offset = 0 };
	uint16_t transfer_id = 0;
	uint32_t sample_offset = 0;
	uint16_t sample_count = 0;

	if (offset != 0) {
		LOG_WRN("Transfer chunk rejected: invalid offset=%u len=%u", offset, len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (protocol_read_uint16(&reader, &transfer_id) != PROTOCOL_OK ||
	    protocol_read_uint32(&reader, &sample_offset) != PROTOCOL_OK ||
	    protocol_read_uint16(&reader, &sample_count) != PROTOCOL_OK ||
	    reader.size - reader.offset != sample_count * sizeof(int16_t)) {
		LOG_WRN("Transfer chunk decode failed: len=%u bytes_remaining=%u sample_count=%u",
			len, reader.size - reader.offset, sample_count);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_CHUNK,
				       BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN));
	}

	LOG_DBG("Transfer chunk received: id=%u offset=%u samples=%u len=%u", transfer_id,
		sample_offset, sample_count, len);

	k_mutex_lock(&service_mutex, K_FOREVER);
	if (!transfer.active || transfer.committed || transfer_id != transfer.id ||
	    sample_offset != transfer.received_samples || sample_count == 0 ||
	    sample_count > transfer.total_samples - transfer.received_samples) {
		LOG_WRN("Transfer chunk rejected: requested_id=%u current_id=%u offset=%u expected_offset=%u samples=%u remaining=%u active=%d committed=%d",
			transfer_id, transfer.id, sample_offset, transfer.received_samples,
			sample_count, transfer.total_samples - transfer.received_samples, transfer.active,
			transfer.committed);
		k_mutex_unlock(&service_mutex);
		return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_CHUNK,
				       BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED));
	}

	for (size_t index = 0; index < sample_count; ++index) {
		if (protocol_read_int16(&reader, &transfer.samples[transfer.received_samples + index]) !=
		    PROTOCOL_OK) {
			LOG_WRN("Transfer chunk sample decode failed: id=%u offset=%u sample_index=%u",
				transfer_id, sample_offset, index);
			k_mutex_unlock(&service_mutex);
			return reject_transfer(AUDIO_RESPONSE_TRANSFER_INVALID_CHUNK,
					       BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN));
		}
	}
	transfer.received_samples += sample_count;
	k_work_reschedule(&transfer_timeout_work, AUDIO_RESPONSE_TRANSFER_TIMEOUT);
	(void)notify_transfer_status(AUDIO_RESPONSE_TRANSFER_READY, AUDIO_RESPONSE_TRANSFER_CREDITS);
	if (transfer.received_samples == transfer.total_samples) {
		LOG_INF("Transfer upload complete: id=%u samples=%u", transfer.id,
			transfer.total_samples);
	} else {
		LOG_DBG("Transfer chunk accepted: id=%u received=%u/%u", transfer.id,
			transfer.received_samples, transfer.total_samples);
	}
	k_mutex_unlock(&service_mutex);
	return len;
}

/**
 * Decode a measurement request and queue it outside the Bluetooth callback.
 */
static ssize_t write_audio_response_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					   const void *buf, uint16_t len, uint16_t offset,
					   uint8_t flags)
{
	audio_response_config_t config;
	size_t bytes_read = 0;

	if (offset != 0) {
		LOG_WRN("Audio response config rejected: invalid offset=%u len=%u", offset, len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (audio_response_config_decode(&config, buf, len, &bytes_read) != PROTOCOL_OK ||
	    bytes_read != len) {
		LOG_WRN("Audio response config decode failed: bytes_read=%u len=%u", bytes_read, len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	LOG_INF("Audio response config requested: id=%u transfer_id=%u volume=%.2f", config.id,
		config.transfer_id, (double)config.volume);
	if (!isfinite(config.volume) || config.volume < 0.0f || config.volume > 1.0f) {
		LOG_WRN("Audio response config rejected: invalid volume=%.3f",
			(double)config.volume);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	k_mutex_lock(&service_mutex, K_FOREVER);
	if (!transfer.committed || config.transfer_id != transfer.id || measurement_active) {
		LOG_WRN("Audio response config rejected: requested_transfer_id=%u current_id=%u committed=%d measurement_active=%d",
			config.transfer_id, transfer.id, transfer.committed, measurement_active);
		k_mutex_unlock(&service_mutex);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	pending_config = config;
	measurement_active = true;
	k_mutex_unlock(&service_mutex);

	k_work_submit(&measurement_work);
	LOG_INF("Audio response measurement queued: id=%u transfer_id=%u", pending_config.id,
		pending_config.transfer_id);
	return len;
}

/**
 * Track whether transfer-status notifications are enabled.
 */
static void transfer_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	transfer_status_notifications_enabled = value == BT_GATT_CCC_NOTIFY;
	LOG_DBG("Audio response transfer status notifications %s",
		transfer_status_notifications_enabled ? "enabled" : "disabled");
}

/**
 * Track whether audio-response result notifications are enabled.
 */
static void result_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	result_notifications_enabled = value == BT_GATT_CCC_NOTIFY;
	LOG_DBG("Audio response result notifications %s",
		result_notifications_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(audio_response_svc,
	BT_GATT_PRIMARY_SERVICE(AUDIO_RESPONSE_ZEPHYR_SERVICE_UUID),
	BT_GATT_CHARACTERISTIC(AUDIO_RESPONSE_ZEPHYR_TRANSFER_CONTROL_CHARACTERISTIC_UUID,
			       AUDIO_RESPONSE_ZEPHYR_TRANSFER_CONTROL_CHARACTERISTIC_PROPERTIES,
			       AUDIO_RESPONSE_ZEPHYR_TRANSFER_CONTROL_CHARACTERISTIC_PERMISSIONS, NULL,
			       write_transfer_control, NULL),
	BT_GATT_CHARACTERISTIC(AUDIO_RESPONSE_ZEPHYR_TRANSFER_DATA_CHARACTERISTIC_UUID,
			       AUDIO_RESPONSE_ZEPHYR_TRANSFER_DATA_CHARACTERISTIC_PROPERTIES,
			       AUDIO_RESPONSE_ZEPHYR_TRANSFER_DATA_CHARACTERISTIC_PERMISSIONS, NULL,
			       write_transfer_data, NULL),
	BT_GATT_CHARACTERISTIC(AUDIO_RESPONSE_ZEPHYR_TRANSFER_STATUS_CHARACTERISTIC_UUID,
			       AUDIO_RESPONSE_ZEPHYR_TRANSFER_STATUS_CHARACTERISTIC_PROPERTIES,
			       AUDIO_RESPONSE_ZEPHYR_TRANSFER_STATUS_CHARACTERISTIC_PERMISSIONS, NULL,
			       NULL, NULL),
	BT_GATT_CCC(transfer_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(AUDIO_RESPONSE_ZEPHYR_CONFIG_CHARACTERISTIC_UUID,
			       AUDIO_RESPONSE_ZEPHYR_CONFIG_CHARACTERISTIC_PROPERTIES,
			       AUDIO_RESPONSE_ZEPHYR_CONFIG_CHARACTERISTIC_PERMISSIONS, NULL,
			       write_audio_response_config, NULL),
	BT_GATT_CHARACTERISTIC(AUDIO_RESPONSE_ZEPHYR_RESULT_CHARACTERISTIC_UUID,
			       AUDIO_RESPONSE_ZEPHYR_RESULT_CHARACTERISTIC_PROPERTIES,
			       AUDIO_RESPONSE_ZEPHYR_RESULT_CHARACTERISTIC_PERMISSIONS, NULL, NULL,
			       NULL),
	BT_GATT_CCC(result_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/**
 * Run the audio playback and capture setup from the system work queue.
 */
static void measurement_work_handler(struct k_work *work)
{
	int ret;

	LOG_INF("Starting audio response measurement: id=%u transfer_id=%u samples=%u volume=%.2f",
		pending_config.id, pending_config.transfer_id, transfer.total_samples,
		(double)pending_config.volume);

	if (!fifo_rx.initialized) {
		LOG_DBG("Initializing RX FIFO for audio response measurement");
		ret = data_fifo_init(&fifo_rx);
		if (ret != 0) {
			LOG_ERR("Failed to initialize RX FIFO: %d", ret);
			goto fail;
		}
	}

	ret = hw_codec_volume_unmute();
	if (ret != 0) {
		LOG_ERR("Failed to unmute codec volume: %d", ret);
		goto fail;
	}
	ret = audio_datapath_decimator_init(CONFIG_AUDIO_SAMPLE_RATE_HZ /
					    AUDIO_RESPONSE_CAPTURE_SAMPLE_RATE);
	if (ret != 0) {
		LOG_ERR("Failed to initialize audio response decimator: %d", ret);
		goto fail;
	}
	ret = audio_datapath_aquire(&fifo_rx);
	if (ret != 0) {
		LOG_ERR("Failed to acquire audio datapath: %d", ret);
		goto fail;
	}
	ret = audio_datapath_buffer_play(transfer.samples, transfer.total_samples, false,
					 pending_config.volume, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to start audio response playback: %d", ret);
		audio_datapath_release();
		goto fail;
	}

	LOG_INF("Audio response playback started: id=%u capture_samples=%u capture_rate=%u initial_drop=%u",
		pending_config.id, AUDIO_RESPONSE_CAPTURE_SAMPLES,
		AUDIO_RESPONSE_CAPTURE_SAMPLE_RATE, AUDIO_RESPONSE_INITIAL_DROP);
	record_to_buffer(captured_samples, AUDIO_RESPONSE_CAPTURE_SAMPLES,
			 AUDIO_RESPONSE_INITIAL_DROP, false, true, audio_response_capture_complete);
	return;

fail:
	LOG_ERR("Failed to start audio response measurement: %d", ret);
	k_mutex_lock(&service_mutex, K_FOREVER);
	measurement_active = false;
	k_mutex_unlock(&service_mutex);
}

/**
 * Encode and notify the FFT response at the configured target frequencies.
 */
static void notify_result(void)
{
	audio_response_result_t result = {
		.id = pending_config.id,
		.points = AUDIO_RESPONSE_RESULT_POINTS,
		.frequencies = result_frequencies,
		.response = result_response,
	};
	size_t payload_size;

	if (!result_notifications_enabled) {
		LOG_WRN("Audio response result notifications are not enabled: id=%u",
			pending_config.id);
		return;
	}
	if (audio_response_result_encode(&result, result_payload, sizeof(result_payload), &payload_size) !=
	    PROTOCOL_OK) {
		LOG_ERR("Failed to encode audio response result: id=%u points=%u", result.id,
			result.points);
		return;
	}

	int ret = bt_gatt_notify(NULL, &audio_response_svc.attrs[11], result_payload, payload_size);
	if (ret != 0) {
		LOG_ERR("Failed to notify audio response result: id=%u err=%d", result.id, ret);
		return;
	}
	LOG_INF("Audio response result notified: id=%u points=%u bytes=%u", result.id,
		result.points, payload_size);
}

/**
 * Compute the captured signal's FFT response and notify the requesting client.
 */
static void measurement_complete_work_handler(struct k_work *work)
{
	static arm_rfft_instance_q15 rfft;
	static bool rfft_initialized;

	LOG_INF("Audio response capture complete: id=%u", pending_config.id);
	audio_datapath_buffer_stop();
	audio_datapath_release();

	if (!rfft_initialized) {
		LOG_DBG("Initializing audio response FFT: samples=%u", AUDIO_RESPONSE_CAPTURE_SAMPLES);
		if (arm_rfft_init_q15(&rfft, AUDIO_RESPONSE_CAPTURE_SAMPLES, 0, 1) != ARM_MATH_SUCCESS) {
			LOG_ERR("Failed to initialize audio response FFT");
			goto complete;
		}
		rfft_initialized = true;
	}

	arm_rfft_q15(&rfft, captured_samples, fft_output);
	arm_cmplx_mag_q15(fft_output, magnitudes, AUDIO_RESPONSE_CAPTURE_SAMPLES / 2);

	for (size_t index = 0; index < AUDIO_RESPONSE_RESULT_POINTS; ++index) {
		uint32_t bin = ((uint32_t)target_frequencies[index] * AUDIO_RESPONSE_CAPTURE_SAMPLES +
				AUDIO_RESPONSE_CAPTURE_SAMPLE_RATE / 2) /
			       AUDIO_RESPONSE_CAPTURE_SAMPLE_RATE;
		result_frequencies[index] = target_frequencies[index];
		result_response[index] = magnitudes[bin];
	}
	LOG_DBG("Audio response FFT complete: id=%u bins=%u result_points=%u", pending_config.id,
		AUDIO_RESPONSE_CAPTURE_SAMPLES / 2, AUDIO_RESPONSE_RESULT_POINTS);
	notify_result();

complete:
	k_mutex_lock(&service_mutex, K_FOREVER);
	measurement_active = false;
	k_mutex_unlock(&service_mutex);
	LOG_DBG("Audio response measurement complete: id=%u", pending_config.id);
}

/**
 * Release an incomplete transfer after the protocol timeout.
 */
static void transfer_timeout_work_handler(struct k_work *work)
{
	k_mutex_lock(&service_mutex, K_FOREVER);
	if (transfer.active && !transfer.committed) {
		LOG_WRN("Transfer timed out: id=%u received=%u/%u", transfer.id,
			transfer.received_samples, transfer.total_samples);
		(void)notify_transfer_status(AUDIO_RESPONSE_TRANSFER_TIMED_OUT, 0);
		reset_transfer();
	}
	k_mutex_unlock(&service_mutex);
}

void audio_response_capture_complete(void)
{
	LOG_DBG("Audio response capture completion callback");
	k_work_submit(&measurement_complete_work);
}

int init_audio_response_service(void)
{
	LOG_INF("Initializing audio response service");
	k_work_init(&measurement_work, measurement_work_handler);
	k_work_init(&measurement_complete_work, measurement_complete_work_handler);
	k_work_init_delayable(&transfer_timeout_work, transfer_timeout_work_handler);
	return 0;
}
