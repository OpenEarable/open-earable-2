#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "../SensorManager/SensorManager.h"
#include "macros_common.h"
#include "SDLogger.h"
#include "PowerManager.h"
#include <errno.h>
#include "audio_datapath.h"
#include "channel_assignment.h"

#include "StateIndicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sd_logger, CONFIG_LOG_DEFAULT_LEVEL);

ZBUS_CHAN_DECLARE(sd_card_chan);

K_THREAD_STACK_DEFINE(thread_stack, CONFIG_SENSOR_SD_STACK_SIZE);

void sd_listener_callback(const struct zbus_channel *chan);

ZBUS_LISTENER_DEFINE(sd_card_event_listener, sd_listener_callback);

static struct k_thread thread_data;
static k_tid_t thread_id;

struct ring_buf ring_buffer;
struct k_mutex ring_mutex;   // Protects ring_buffer operations
struct k_mutex file_mutex;   // Protects sd_card open/write/close
uint8_t buffer[BUFFER_SIZE];

// Coordination flags (atomic because they are accessed from multiple threads)
static atomic_t g_stop_writing;   // 1 while end()/flush/close is in progress
static atomic_t g_sd_removed;     // 1 if SD was removed while recording

uint32_t count_max_buffer_fill = 0;

static struct {
    uint32_t ring_full;
    uint32_t ring_mutex_fail;
    uint32_t sd_writes;
    uint32_t sd_write_max_us;
    uint64_t sd_write_total_us;
    uint32_t bytes_written;
    uint32_t bytes_dropped;
} sd_stats;

struct k_poll_signal logger_sig;
static struct k_poll_event logger_evt =
		 K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &logger_sig);

SDLogger::SDLogger() {
    sd_card = &sdcard_manager;
    k_mutex_init(&ring_mutex);
    k_mutex_init(&file_mutex);
    atomic_clear(&g_stop_writing);
    atomic_clear(&g_sd_removed);
}

SDLogger::~SDLogger() {

}

void sd_listener_callback(const struct zbus_channel *chan)
{
    const struct sd_msg * sd_msg_event = (sd_msg *)zbus_chan_const_msg(&sd_card_chan);

    if (sdlogger.is_open && sd_msg_event->removed) {
        // Signal SD thread to stop writing immediately.
        atomic_set(&g_sd_removed, 1);
        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("SD card removed mid recording. Stop recording.");

        // Wake the SD thread so it can react quickly.
        k_poll_signal_raise(&logger_sig, 0);
    }
}


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

        if (signaled == 0) {
            LOG_DBG("Poll woke up without signal");
            continue;
        }

        // Reset signal early so signals raised during the drain loop
        // are preserved for the next k_poll wakeup.
        k_poll_signal_reset(&logger_sig);
        logger_evt.state = K_POLL_STATE_NOT_READY;

        // If a close/flush is in progress, do not write concurrently.
        if (atomic_get(&g_stop_writing)) {
            continue;
        }

        // If SD was removed, stop writing and drop buffered data.
        if (atomic_get(&g_sd_removed)) {
            k_mutex_lock(&ring_mutex, K_FOREVER);
            ring_buf_reset(&ring_buffer);
            k_mutex_unlock(&ring_mutex);
            sdlogger.is_open = false;
            continue;
        }

        if (!sdcard_manager.is_mounted()) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("SD Card not mounted!");
            return;
        }

        // Drain all available full blocks from the ring buffer.
        while (ring_buf_size_get(&ring_buffer) >= SD_BLOCK_SIZE) {
            if (atomic_get(&g_stop_writing) || atomic_get(&g_sd_removed)) {
                break;
            }

            uint32_t fill = ring_buf_size_get(&ring_buffer);
            if (fill > count_max_buffer_fill) {
                count_max_buffer_fill = fill;
            }

            uint8_t *data = nullptr;

            // Claim up to 4 SD blocks from the ring buffer under lock.
            k_mutex_lock(&ring_mutex, K_FOREVER);
            uint32_t claimed = ring_buf_get_claim(&ring_buffer, &data, 4 * SD_BLOCK_SIZE);
            k_mutex_unlock(&ring_mutex);

            if (claimed == 0 || data == nullptr) {
                break;
            }

            // Write the claimed bytes under file lock.
            size_t write_size = claimed;
            int written;
            uint64_t t0 = micros();
            k_mutex_lock(&file_mutex, K_FOREVER);
            written = sdlogger.sd_card->write((char*)data, &write_size, false);
            k_mutex_unlock(&file_mutex);
            uint32_t elapsed = (uint32_t)(micros() - t0);

            sd_stats.sd_writes++;
            sd_stats.sd_write_total_us += elapsed;
            if (elapsed > sd_stats.sd_write_max_us) {
                sd_stats.sd_write_max_us = elapsed;
            }

            if (written < 0) {
                state_indicator.set_sd_state(SD_FAULT);
                LOG_ERR("SD write failed: %d", written);
                break;
            }

            sd_stats.bytes_written += (uint32_t)written;

            // Advance ring buffer by the number of bytes actually written.
            k_mutex_lock(&ring_mutex, K_FOREVER);
            ring_buf_get_finish(&ring_buffer, (uint32_t)written);
            k_mutex_unlock(&ring_mutex);

            // Let lower-priority threads (sensor work queue) run between writes
            k_yield();
        }

        STACK_USAGE_PRINT("sensor_msg_thread", &sdlogger.thread_data);
    }
}

int SDLogger::init() {
    int ret;

    ring_buf_init(&ring_buffer, BUFFER_SIZE, buffer);

    atomic_clear(&g_stop_writing);
    atomic_clear(&g_sd_removed);

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

    if (!sd_card->is_mounted()) {
        ret = sd_card->mount();
        if (ret < 0) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("Failed to mount sd card: %d", ret);
            return ret;
        }
    }

    LOG_INF("OPEN FILE: %s", filename.c_str());

    std::string full_filename = filename + ".oe";
    k_mutex_lock(&file_mutex, K_FOREVER);
    ret = sd_card->open_file(full_filename, true, false, true);
    k_mutex_unlock(&file_mutex);
    if (ret < 0) {
        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    // Ensure no concurrent end()/flush is running
    atomic_clear(&g_stop_writing);
    atomic_clear(&g_sd_removed);

    current_file = full_filename;
    is_open = true;

    k_mutex_lock(&ring_mutex, K_FOREVER);
    ring_buf_reset(&ring_buffer);
    k_mutex_unlock(&ring_mutex);

    memset(&sd_stats, 0, sizeof(sd_stats));
    count_max_buffer_fill = 0;

    ret = write_header();
    if (ret < 0) {
        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("Failed to write header: %d", ret);
        return ret;
    }

    k_poll_signal_raise(&logger_sig, 0);

    return 0;
}

int SDLogger::write_header() {
    size_t header_size = sizeof(FileHeader);
    uint8_t header_buffer[header_size];
    FileHeader* header = reinterpret_cast<FileHeader*>(header_buffer);

    header->version = SENSOR_LOG_VERSION;
    header->timestamp = micros();
    header->device_id = oe_boot_state.device_id;

    enum audio_channel ch;
    channel_assignment_get(&ch);
    header->channel = (uint8_t)ch;

    int ret;
    k_mutex_lock(&file_mutex, K_FOREVER);
    ret = sd_card->write((char *)header_buffer, &header_size, false);
    k_mutex_unlock(&file_mutex);
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

    // If a close/flush is in progress or SD was removed, drop quickly
    if (atomic_get(&g_stop_writing) || atomic_get(&g_sd_removed)) {
        return -ENODEV;
    }

    // Allow a short wait for mutex contention to clear.
    // The writer holds ring_mutex only during fast ring_buf operations.
    if (k_mutex_lock(&ring_mutex, K_USEC(200)) != 0) {
        sd_stats.ring_mutex_fail++;
        sd_stats.bytes_dropped += total_length;
        return -EAGAIN;
    }

    // Ensure there is enough space; if not, free up room by discarding oldest bytes
    // in SD_BLOCK_SIZE chunks to keep SD writer alignment and minimize partial writes.
    uint32_t space = ring_buf_space_get(&ring_buffer);
    if (space < total_length) {
        sd_stats.ring_full++;
        sd_stats.bytes_dropped += total_length;
        LOG_DBG("Ring buffer full: have %u, need %zu", space, total_length);
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

    k_poll_signal_raise(&logger_sig, 0);
    return 0;
}

int SDLogger::write_sensor_data(const sensor_data& msg) {
    const size_t data_size = sizeof(sensor_data) - sizeof(msg.data) + msg.size;
    const void* msg_ptr = &msg;
    return write_sensor_data(&msg_ptr, &data_size, 1);
}

int SDLogger::flush() {
    // Prevent SD thread from writing concurrently
    atomic_set(&g_stop_writing, 1);
    k_poll_signal_raise(&logger_sig, 0);

    uint32_t total_written = 0;
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
        int written;
        k_mutex_lock(&file_mutex, K_FOREVER);
        written = sd_card->write((char*)data, &req, false);
        k_mutex_unlock(&file_mutex);

        if (written < 0) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("Failed to flush SD buffer: %d", written);
            break;
        }

        k_mutex_lock(&ring_mutex, K_FOREVER);
        ring_buf_get_finish(&ring_buffer, (uint32_t)written);
        k_mutex_unlock(&ring_mutex);

        total_written += (uint32_t)written;

        if ((uint32_t)written < claimed) {
            k_yield();
        }
    }

    return (int)total_written;
}

int SDLogger::end() {
    int ret;
    
    if (!is_open) {
        return -ENODEV;
    }

    if (!sd_card->is_mounted()) {
        //k_poll_signal_reset(&logger_sig);
        is_open = false;
        return -ENODEV;
    }

    // Prevent SD thread/producers from writing while we flush/close
    atomic_set(&g_stop_writing, 1);
    k_poll_signal_raise(&logger_sig, 0);

    ret = flush();
    if (ret < 0) {
        LOG_ERR("Failed to flush file buffer.");
        return ret;
    }

    LOG_INF("Close File ....");

    uint32_t avg_write_us = sd_stats.sd_writes ? (uint32_t)(sd_stats.sd_write_total_us / sd_stats.sd_writes) : 0;
    LOG_INF("SD stats: ring_full=%u ring_mutex_fail=%u bytes_dropped=%u",
            sd_stats.ring_full, sd_stats.ring_mutex_fail, sd_stats.bytes_dropped);
    LOG_INF("SD stats: writes=%u bytes=%u max_fill=%u/%u",
            sd_stats.sd_writes, sd_stats.bytes_written, count_max_buffer_fill, (unsigned)BUFFER_SIZE);
    LOG_INF("SD stats: write_avg=%u us write_max=%u us",
            avg_write_us, sd_stats.sd_write_max_us);

    k_mutex_lock(&file_mutex, K_FOREVER);
    ret = sd_card->close_file();
    k_mutex_unlock(&file_mutex);
    if (ret < 0) {
        k_poll_signal_reset(&logger_sig);
        return ret;
    }

    is_open = false;

    k_poll_signal_reset(&logger_sig);
    atomic_clear(&g_stop_writing);
    atomic_clear(&g_sd_removed);

    /* Unmount the SD card so USB MSC can re-enable for host access. */
    sd_card->unmount();

    return 0;
}

bool SDLogger::is_active() {
    return is_open;
}

SDLogger sdlogger;
