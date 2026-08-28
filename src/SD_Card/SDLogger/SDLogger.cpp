#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "../SensorManager/SensorManager.h"
#include "macros_common.h"
#include "SDLogger.h"
#include "BootState.h"
#include "PowerManager.h"
#include "SensorScheme.h"
#include "channel_assignment.h"
#include <errno.h>
#include <stdint.h>
#include "audio_datapath.h"
#include "SDMassStorage.h"

#include "StateIndicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sd_logger, CONFIG_LOG_DEFAULT_LEVEL);

ZBUS_CHAN_DECLARE(sd_card_chan);

void sensor_listener_cb(const struct zbus_channel *chan);

ZBUS_LISTENER_DEFINE(sensor_data_listener, sensor_listener_cb);

// Define thread stack
K_THREAD_STACK_DEFINE(thread_stack, CONFIG_SENSOR_SD_STACK_SIZE);

ZBUS_CHAN_DECLARE(sensor_chan);

void sd_listener_callback(const struct zbus_channel *chan);

ZBUS_LISTENER_DEFINE(sd_card_event_listener, sd_listener_callback);

static struct k_thread thread_data;
static k_tid_t thread_id;

struct ring_buf ring_buffer;
struct k_mutex ring_mutex;   // Protects ring_buffer operations
uint8_t buffer[BUFFER_SIZE];  // Ring Buffer Speicher

/*
 * Set while begin()/end()/abort_recording() own the ring buffer and the log
 * file. Producers drop their samples and the writer thread parks while it is
 * set. Atomic because all three run on other threads than the writer.
 */
static atomic_t g_stop_writing;

uint32_t count_max_buffer_fill = 0;

struct k_poll_signal logger_sig;
static struct k_poll_event logger_evt =
		 K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &logger_sig);

/*
 * Given by the writer thread once it has observed g_stop_writing at the top of
 * its loop, which means it holds no ring buffer claim and is not inside
 * SDCardManager. A teardown waits for it before flushing or closing.
 */
static K_SEM_DEFINE(writer_parked, 0, 1);

/*
 * Upper bound for the park handshake.
 *
 * One 4 KiB block write on a healthy card takes low tens of milliseconds, so
 * this is a generous margin. It is deliberately far shorter than the SD driver
 * timeout (CONFIG_SD_DATA_TIMEOUT, 10 s): if the writer is stuck on media that
 * no longer answers, the teardown gives up and skips the flush rather than
 * waiting the card out.
 */
static constexpr int32_t WRITER_PARK_TIMEOUT_MS = 250;

namespace {

constexpr uint8_t OE_HEADER_SIDE_LEFT = 0x00;
constexpr uint8_t OE_HEADER_SIDE_RIGHT = 0x01;
constexpr uint8_t OE_HEADER_SIDE_UNKNOWN = 0xFF;

uint8_t get_header_side() {
    enum audio_channel channel;
    channel_assignment_get(&channel);

    if (channel == AUDIO_CH_L) {
        return OE_HEADER_SIDE_LEFT;
    }
    if (channel == AUDIO_CH_R) {
        return OE_HEADER_SIDE_RIGHT;
    }

    return OE_HEADER_SIDE_UNKNOWN;
}

} // namespace

SDLogger::SDLogger() {
    sd_card = &sdcard_manager;
    k_mutex_init(&ring_mutex);
    atomic_clear(&g_stop_writing);
}

SDLogger::~SDLogger() {

}

//static bool _prio_boost = false;

void sensor_listener_cb(const struct zbus_channel *chan) {
    const sensor_msg* msg = (sensor_msg*)zbus_chan_const_msg(chan);

	if (msg->sd) {
        int ret = sdlogger.write_sensor_data(msg->data);
        if (ret < 0) {
            if (ret == -ENODEV) {
                LOG_DBG("Dropping SD sample because logger is not open");
            } else {
                LOG_WRN("Failed to enqueue sensor data for SD: %d", ret);
            }
        }
	}
}


void sd_listener_callback(const struct zbus_channel *chan)
{
    const struct sd_msg * sd_msg_event = (const sd_msg *)zbus_chan_const_msg(chan);

    if (sd_msg_event->removed) {
        sensor_manager_sd_card_removed();
        sdlogger.abort_recording();
    }
}

/**
 * @brief Clear the wake-up signal.
 *
 * @details Must only be called from the SD writer thread. While that thread is
 *      blocked in k_poll(), logger_evt is registered with the kernel and its
 *      state field shares a word with the event type, so it is not safe to
 *      write from any other context.
 */
inline void reset_logger_signal() {
    k_poll_signal_reset(&logger_sig);
    logger_evt.state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief Wait until the SD writer thread has released the ring buffer and file.
 *
 * @details Expects g_stop_writing to be set by the caller; the flag is what
 *      makes the writer park instead of picking up the next block.
 *
 * @return true if the writer parked, false on timeout.
 */
static bool park_writer() {
    k_sem_reset(&writer_parked);
    k_poll_signal_raise(&logger_sig, 0);

    if (k_sem_take(&writer_parked, K_MSEC(WRITER_PARK_TIMEOUT_MS)) != 0) {
        LOG_WRN("SD writer thread did not park within %d ms", WRITER_PARK_TIMEOUT_MS);
        return false;
    }

    return true;
}

/**
 * @brief SD writer thread: drains the ring buffer into the open log file.
 *
 * @details Sleeps on logger_sig and writes at most one aligned chunk per
 *      wake-up, so the thread stays responsive to teardown requests. It is the
 *      sole owner of logger_evt and the only consumer of the ring buffer while
 *      g_stop_writing is clear. Runs for the lifetime of the device: errors are
 *      reported and retried on the next wake-up, never by leaving the loop.
 */
void SDLogger::sensor_sd_task() {
    int ret;

    while (1) {
        ret = k_poll(&logger_evt, 1, K_FOREVER);

        if (ret < 0) {
            LOG_ERR("k_poll failed: %d", ret);
            continue;
        }

        unsigned int signaled;
        int result;
        k_poll_signal_check(&logger_sig, &signaled, &result);
        reset_logger_signal();

        if (signaled == 0) {
            LOG_DBG("Poll woke up without signal");
            continue;
        }

        /* A teardown owns the ring buffer and the file; acknowledge and wait. */
        if (atomic_get(&g_stop_writing)) {
            k_sem_give(&writer_parked);
            continue;
        }

        if (!sdlogger.is_open) {
            continue;
        }

        if (!sdcard_manager.is_mounted()) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("SD Card not mounted!");
            continue;
        }

        uint32_t fill = ring_buf_size_get(&ring_buffer);

        if (fill < SD_BLOCK_SIZE) {
            continue;
        }

        if (fill > count_max_buffer_fill) {
            count_max_buffer_fill = fill;
        }

        uint8_t *data = nullptr;

        // Claim a block-aligned chunk from the ring buffer under lock.
        k_mutex_lock(&ring_mutex, K_FOREVER);
        uint32_t claimed = ring_buf_get_claim(&ring_buffer, &data, fill - (fill % SD_BLOCK_SIZE));
        k_mutex_unlock(&ring_mutex);

        if (claimed == 0 || data == nullptr) {
            continue;
        }

        size_t write_size = claimed;
        int written = sdlogger.sd_card->write((char*)data, &write_size, false);

        if (written < 0) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("SD write failed: %d", written);
        }

        /*
         * A claim must always be closed, including after a failed write:
         * ring_buf_get_finish() is what returns get_head to get_tail. Finishing
         * with 0 releases the claim and leaves the data queued for a retry.
         */
        k_mutex_lock(&ring_mutex, K_FOREVER);
        ring_buf_get_finish(&ring_buffer, written > 0 ? (uint32_t)written : 0);
        k_mutex_unlock(&ring_mutex);

        if (written < 0) {
            /* Back off; the next producer wake-up retries. */
            continue;
        }

        /* The signal was cleared before the write, so re-arm if data is left. */
        if (ring_buf_size_get(&ring_buffer) >= SD_BLOCK_SIZE) {
            k_poll_signal_raise(&logger_sig, 0);
        }
    }
}

int SDLogger::init() {
    int ret;

    sd_card->init();

    ring_buf_init(&ring_buffer, BUFFER_SIZE, buffer);

    atomic_clear(&g_stop_writing);

    k_poll_signal_init(&logger_sig);

	thread_id = k_thread_create(
		&thread_data, thread_stack,
		CONFIG_SENSOR_SD_STACK_SIZE, (k_thread_entry_t)sensor_sd_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_SENSOR_SD_THREAD_PRIO), 0, K_NO_WAIT);
	
	ret = k_thread_name_set(thread_id, "SENSOR_SD_SUB");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_data_listener, ZBUS_ADD_OBS_TIMEOUT_MS);
    if (ret) {
        LOG_ERR("Failed to add sensor sub");
        return ret;
    }

    ret = zbus_chan_add_obs(&sd_card_chan, &sd_card_event_listener, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sd sub");
		return ret;
	}

    return 0;
}

/**
 * @brief Begin logging to a file
 * @param filename Base filename without extension
 * @return 0 on success, negative error code on failure
 * 
 * Opens a file for logging with .oe extension appended to the filename.
 * Returns -EBUSY if logger is already open or -ENODEV if SD card not initialized.
 */
int SDLogger::begin(const std::string& filename) {
    int ret;

    if (is_open) {
        LOG_ERR("Logger already open");
        return -EBUSY;
    }

    if (sd_mass_storage_host_active()) {
        LOG_WRN("Cannot start SD recording while USB mass storage is active");
        return -EBUSY;
    }

    ret = sd_mass_storage_recording_starting();
    if (ret < 0) {
        state_indicator.set_sd_state(SD_FAULT);
        return ret;
    }

    if (!sd_card->is_mounted()) {
        ret = sd_card->mount();
        if (ret < 0) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("Failed to mount sd card: %d", ret);
            sd_mass_storage_recording_aborted();
            return ret;
        }
    }

    LOG_INF("OPEN FILE: %s", filename.c_str());

    std::string full_filename = filename + ".oe";
    ret = sd_card->open_file(full_filename, true, false, true);
    if (ret < 0) {
        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    /* Take ownership of the buffer: no teardown can be in flight from here. */
    atomic_clear(&g_stop_writing);

    k_mutex_lock(&ring_mutex, K_FOREVER);
    ring_buf_reset(&ring_buffer);
    k_mutex_unlock(&ring_mutex);

    current_file = full_filename;

    /*
     * The header bypasses the ring buffer, so it must reach the file before
     * is_open lets producers enqueue and the writer thread start emitting
     * data blocks.
     */
    ret = write_header();
    if (ret < 0) {
        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("Failed to write header: %d", ret);
        sd_card->close_file();
        current_file.clear();
        return ret;
    }

    is_open = true;

    k_poll_signal_raise(&logger_sig, 0);

    return 0;
}

int SDLogger::write_header() {
    const size_t parse_info_size = getParseInfoStorageSize();
    if (parse_info_size == 0) {
        LOG_ERR("Parse info scheme is unavailable");
        return -ENODATA;
    }

    const size_t header_size = sizeof(FileHeader) + parse_info_size;
    if ((header_size > UINT32_MAX) || (parse_info_size > UINT32_MAX)) {
        LOG_ERR("Parse info header too large: header=%zu parse_info=%zu", header_size, parse_info_size);
        return -EOVERFLOW;
    }

    uint8_t* header_buffer = static_cast<uint8_t*>(k_malloc(header_size));
    if (header_buffer == nullptr) {
        LOG_ERR("Failed to allocate %zu bytes for OE header", header_size);
        return -ENOMEM;
    }

    FileHeader* header = reinterpret_cast<FileHeader*>(header_buffer);

    header->version = SENSOR_LOG_VERSION;
    header->timestamp = micros();
    header->header_size = header_size;
    header->parse_info_size = parse_info_size;
    header->device_id = oe_boot_state.device_id;
    header->side = get_header_side();

    ssize_t serialized_size = serializeParseInfoStorage(
        reinterpret_cast<char*>(header_buffer + sizeof(FileHeader)),
        parse_info_size
    );
    if (serialized_size < 0) {
        k_free(header_buffer);
        LOG_ERR("Failed to serialize parse info storage: %d", (int)serialized_size);
        return (int)serialized_size;
    }
    if ((size_t)serialized_size != parse_info_size) {
        k_free(header_buffer);
        LOG_ERR("Parse info size mismatch: %d != %zu", (int)serialized_size, parse_info_size);
        return -EIO;
    }

    size_t bytes_to_write = header_size;
    int ret = sd_card->write(reinterpret_cast<char*>(header_buffer), &bytes_to_write, false);
    k_free(header_buffer);

    if ((ret >= 0) && ((size_t)ret != header_size)) {
        LOG_ERR("Incomplete header write: %d != %zu", ret, header_size);
        return -EIO;
    }

    return ret;
}

int SDLogger::write_sensor_data(const void* const* data_blocks, const size_t* lengths, size_t block_count) {
    if (!is_open || data_blocks == nullptr || lengths == nullptr || block_count == 0) {
        return -ENODEV;
    }

    // Calculate total length needed
    size_t total_length = 0;
    for (size_t i = 0; i < block_count; i++) {
        total_length += lengths[i];
    }

    // Single message larger than buffer -> cannot ever fit
    if (total_length > BUFFER_SIZE) {
        LOG_WRN("Dropping oversize record: %zu > BUFFER_SIZE=%u", total_length, (unsigned)BUFFER_SIZE);
        return -EMSGSIZE;
    }

    // A teardown owns the buffer; drop rather than wait for it.
    if (atomic_get(&g_stop_writing)) {
        return -ENODEV;
    }

    // Do not block producers; if mutex is contended, drop quickly
    if (k_mutex_lock(&ring_mutex, K_NO_WAIT) != 0) {
        return -EAGAIN;
    }

    // Records are enqueued whole or not at all, so partial writes cannot
    // desynchronise the block framing in the log file.
    uint32_t space = ring_buf_space_get(&ring_buffer);
    if (space < total_length) {
        LOG_ERR("Ring buffer low on space: have %u, need %zu. Skipping data",
            space, total_length);
        k_mutex_unlock(&ring_mutex);
        return -ENOSPC;
    }

    // Try to write all blocks
    for (size_t i = 0; i < block_count; ++i) {
        const uint8_t* src = (const uint8_t*)data_blocks[i];
        size_t len = lengths[i];
        while (len > 0) {
            int wrote = ring_buf_put(&ring_buffer, src, len);
            if (wrote <= 0) {
                // Buffer still tight -> give up quickly; do not block the producer
                k_mutex_unlock(&ring_mutex);
                LOG_DBG("Ring buffer tight; partial enqueue. Dropping remainder=%zu", len);
                return -ENOSPC;
            }
            src += wrote;
            len -= wrote;
        }
    }

    k_mutex_unlock(&ring_mutex);

    if (ring_buf_size_get(&ring_buffer) >= SD_BLOCK_SIZE) {
        k_poll_signal_raise(&logger_sig, 0);
    }
    return 0;
}

int SDLogger::write_sensor_data(const sensor_data& msg) {
    const size_t data_size = sizeof(sensor_data) - sizeof(msg.data) + msg.size;
    const void* msg_ptr = &msg;
    return write_sensor_data(&msg_ptr, &data_size, 1);
}

/**
 * @brief Write out everything still buffered.
 *
 * @details The caller must have parked the SD writer thread first. The ring
 *      buffer supports a single claimer at a time; two concurrent claims hand
 *      out overlapping regions and the first get_finish() invalidates the other.
 *
 * @return Number of bytes written, or a negative error code.
 */
int SDLogger::flush() {
    uint32_t total_written = 0;
    int first_error = 0;

    for (;;) {
        uint8_t *data = nullptr;
        uint32_t fill;

        k_mutex_lock(&ring_mutex, K_FOREVER);
        fill = ring_buf_size_get(&ring_buffer);
        if (fill == 0) {
            k_mutex_unlock(&ring_mutex);
            break;
        }

        uint32_t claimed = ring_buf_get_claim(&ring_buffer, &data, fill);
        k_mutex_unlock(&ring_mutex);

        if (claimed == 0 || data == nullptr) {
            break;
        }

        size_t req = claimed;
        int written = sd_card->write((char*)data, &req, false);

        if (written < 0) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("Failed to flush SD buffer: %d", written);
            first_error = written;
        }

        /* A claim must always be closed again, see sensor_sd_task(). */
        k_mutex_lock(&ring_mutex, K_FOREVER);
        ring_buf_get_finish(&ring_buffer, written > 0 ? (uint32_t)written : 0);
        k_mutex_unlock(&ring_mutex);

        if (written <= 0) {
            /* No progress possible; stop instead of spinning on the same block. */
            break;
        }

        total_written += (uint32_t)written;
    }

    return first_error ? first_error : (int)total_written;
}

void SDLogger::abort_recording() {
    if (!is_open) {
        return;
    }

    LOG_ERR("SD card removed mid recording. Stop recording.");

    /*
     * Runs before SDCardManager unmounts, so the writer has to be off the file
     * by the time this returns. Anything still buffered is unwritable and is
     * discarded by the next begin(); the file handle is released by unmount().
     */
    atomic_set(&g_stop_writing, 1);
    park_writer();

    is_open = false;
    current_file.clear();

    state_indicator.set_sd_state(SD_FAULT);
}

/**
 * @brief Flush and close the current log file.
 *
 * @details Safe to call without an open recording; the coordination state is
 *      always left ready for the next begin().
 *
 * @return 0 on success, -ENODEV if no recording was open, or the first error
 *      encountered while flushing or closing.
 */
int SDLogger::end() {
    int first_error = 0;

    atomic_set(&g_stop_writing, 1);
    const bool parked = park_writer();

    const bool was_open = is_open;

    if (was_open && sd_card->is_mounted()) {
        if (parked) {
            int ret = flush();
            if (ret < 0) {
                LOG_ERR("Failed to flush file buffer: %d", ret);
                first_error = ret;
            }
        } else {
            /* Writer still owns the ring buffer; flushing would race with it. */
            LOG_WRN("Skipping flush: SD writer thread is still busy");
            first_error = -ETIMEDOUT;
        }
    }

    if (parked) {
        k_mutex_lock(&ring_mutex, K_FOREVER);
        ring_buf_reset(&ring_buffer);
        k_mutex_unlock(&ring_mutex);
    }

    if (was_open) {
        LOG_INF("Close File ....");
        LOG_DBG("Max buffer fill: %u bytes", count_max_buffer_fill);

        int ret = sd_card->close_file();
        if (ret < 0 && !first_error) {
            first_error = ret;
        }
    }

    is_open = false;
    current_file.clear();
    atomic_clear(&g_stop_writing);

    if (was_open) {
        sd_mass_storage_recording_stopped();
    }

    return was_open ? first_error : -ENODEV;
}

bool SDLogger::is_active() {
    return is_open;
}

SDLogger sdlogger;
