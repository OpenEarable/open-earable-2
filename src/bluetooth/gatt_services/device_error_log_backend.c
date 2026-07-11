#include "device_error_service.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

/* LOG_LEVEL_WRN forwards LOG_ERR and LOG_WRN. Use LOG_LEVEL_INF to include info logs. */
#define DEVICE_ERROR_LOG_BACKEND_LEVEL LOG_LEVEL_WRN
#define DEVICE_ERROR_LOG_QUEUE_SIZE 8
#define DEVICE_ERROR_LOG_FORMAT_BUFFER_SIZE 128

struct device_error_log_event {
	enum device_error_level level;
	uint16_t error_code;
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
};

struct log_capture_context {
	char *buffer;
	size_t size;
	size_t offset;
};

K_MSGQ_DEFINE(device_error_log_queue, sizeof(struct device_error_log_event),
	      DEVICE_ERROR_LOG_QUEUE_SIZE, 4);

		  static atomic_t device_error_log_backend_enabled;

static void device_error_log_work_handler(struct k_work *work);
K_WORK_DEFINE(device_error_log_work, device_error_log_work_handler);

static uint8_t log_output_buffer[DEVICE_ERROR_LOG_FORMAT_BUFFER_SIZE];
static int capture_log_output(uint8_t *data, size_t length, void *ctx);
LOG_OUTPUT_DEFINE(device_error_log_output, capture_log_output, log_output_buffer,
		  sizeof(log_output_buffer));

static bool log_level_should_forward(uint8_t level)
{
	return (level != LOG_LEVEL_NONE) && (level <= DEVICE_ERROR_LOG_BACKEND_LEVEL);
}

static enum device_error_level device_error_level_from_log(uint8_t level)
{
	switch (level) {
	case LOG_LEVEL_ERR:
		return DEVICE_ERROR_LEVEL_ERROR;
	case LOG_LEVEL_WRN:
		return DEVICE_ERROR_LEVEL_WARNING;
	case LOG_LEVEL_INF:
	default:
		return DEVICE_ERROR_LEVEL_INFO;
	}
}

static uint16_t device_error_code_from_log(uint8_t level)
{
	switch (level) {
	case LOG_LEVEL_ERR:
		return DEVICE_ERROR_CODE_FIRMWARE_LOG_ERROR;
	case LOG_LEVEL_WRN:
		return DEVICE_ERROR_CODE_FIRMWARE_LOG_WARNING;
	case LOG_LEVEL_INF:
		return DEVICE_ERROR_CODE_FIRMWARE_LOG_INFO;
	case LOG_LEVEL_DBG:
	default:
		return DEVICE_ERROR_CODE_FIRMWARE_LOG_DEBUG;
	}
}

static bool is_device_error_service_log(struct log_msg *log_msg)
{
	int16_t source_id = log_msg_get_source_id(log_msg);

	if (source_id < 0) {
		return false;
	}

	const char *source_name = log_source_name_get(log_msg_get_domain(log_msg), source_id);

	return (source_name != NULL) && (strcmp(source_name, "device_error_service") == 0);
}

static int capture_log_output(uint8_t *data, size_t length, void *ctx)
{
	struct log_capture_context *capture = ctx;

	if ((capture->size > 0) && (capture->offset < (capture->size - 1))) {
		size_t remaining = capture->size - 1 - capture->offset;
		size_t copy_len = MIN(length, remaining);

		memcpy(&capture->buffer[capture->offset], data, copy_len);
		capture->offset += copy_len;
		capture->buffer[capture->offset] = '\0';
	}

	return length;
}

static void trim_trailing_line_endings(char *message)
{
	size_t len = strlen(message);

	while ((len > 0) &&
	       ((message[len - 1] == '\r') || (message[len - 1] == '\n') ||
		(message[len - 1] == ' '))) {
		message[len - 1] = '\0';
		len--;
	}
}

static void strip_default_log_tag(char *message)
{
#if defined(CONFIG_LOG_TAG_MAX_LEN) && (CONFIG_LOG_TAG_MAX_LEN > 0) && defined(CONFIG_LOG_TAG_DEFAULT)
	const char *tag = CONFIG_LOG_TAG_DEFAULT;
	size_t tag_len = strlen(tag);

	if ((tag_len > 0) && (strncmp(message, tag, tag_len) == 0) &&
	    (message[tag_len] == ' ')) {
		memmove(message, &message[tag_len + 1], strlen(&message[tag_len + 1]) + 1);
	}
#else
	ARG_UNUSED(message);
#endif
}

static void format_log_message(struct log_msg *log_msg, char *message, size_t message_len)
{
	struct log_capture_context capture = {
		.buffer = message,
		.size = message_len,
		.offset = 0,
	};

	if (message_len == 0) {
		return;
	}

	message[0] = '\0';
	log_output_ctx_set(&device_error_log_output, &capture);
	log_output_msg_process(&device_error_log_output, log_msg, LOG_OUTPUT_FLAG_CRLF_NONE);
	strip_default_log_tag(message);
	trim_trailing_line_endings(message);
}

static void process(const struct log_backend *const backend, union log_msg_generic *msg)
{
	ARG_UNUSED(backend);
	
	if (!atomic_get(&device_error_log_backend_enabled)) {
		return;
	}

	if (!z_log_item_is_msg(msg)) {
		return;
	}

	uint8_t level = log_msg_get_level(&msg->log);

	if (!log_level_should_forward(level) || is_device_error_service_log(&msg->log)) {
		return;
	}

	struct device_error_log_event event = {
		.level = device_error_level_from_log(level),
		.error_code = device_error_code_from_log(level),
	};

	format_log_message(&msg->log, event.message, sizeof(event.message));

	if (event.message[0] == '\0') {
		strcpy(event.message, "Firmware log event");
	}

	if (k_msgq_put(&device_error_log_queue, &event, K_NO_WAIT) == 0) {
		k_work_submit(&device_error_log_work);
	}
}

static void device_error_log_work_handler(struct k_work *work)
{
	struct device_error_log_event event;

	ARG_UNUSED(work);
	
	if (!atomic_get(&device_error_log_backend_enabled)) {
		k_msgq_purge(&device_error_log_queue);
		return;
	}

	while (k_msgq_get(&device_error_log_queue, &event, K_NO_WAIT) == 0) {
		(void)send_device_error(event.level, event.error_code, DEVICE_ERROR_SOURCE_SYSTEM,
					event.message);
	}
}

static void panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	atomic_clear(&device_error_log_backend_enabled);
	k_msgq_purge(&device_error_log_queue);
}

static const struct log_backend_api device_error_log_backend_api = {
	.process = process,
	.panic = panic,
};

LOG_BACKEND_DEFINE(device_error_log_backend, device_error_log_backend_api, false);

void device_error_log_backend_enable(void)
{
	k_msgq_purge(&device_error_log_queue);
	atomic_set(&device_error_log_backend_enabled, 1);
	log_backend_enable(&device_error_log_backend, NULL, CONFIG_LOG_MAX_LEVEL);
}

void device_error_log_backend_disable(void)
{
	atomic_clear(&device_error_log_backend_enabled);
	log_backend_disable(&device_error_log_backend);
	(void)k_work_cancel(&device_error_log_work);
	k_msgq_purge(&device_error_log_queue);
}
