#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"
#include "macros_common.h"
#include "SDLogger.h"
#include "PowerManager.h"
#include <errno.h>
#include "audio_datapath.h"

#include "StateIndicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sd_logger, CONFIG_LOG_DEFAULT_LEVEL);

ZBUS_CHAN_DECLARE(sd_card_chan);

void sensor_listener_cb(const struct zbus_channel *chan);

K_MSGQ_DEFINE(sd_sensor_queue, sizeof(sensor_data), CONFIG_SENSOR_SD_SUB_QUEUE_SIZE, 4);
ZBUS_LISTENER_DEFINE(sensor_data_listener, sensor_listener_cb);

// Define thread stack
K_THREAD_STACK_DEFINE(thread_stack, CONFIG_SENSOR_SD_STACK_SIZE);

ZBUS_CHAN_DECLARE(sensor_chan);

void sd_listener_callback(const struct zbus_channel *chan);

ZBUS_LISTENER_DEFINE(sd_card_event_listener, sd_listener_callback);

static struct k_thread thread_data;
static k_tid_t thread_id;

struct ring_buf ring_buffer;
struct k_mutex write_mutex;
uint8_t buffer[BUFFER_SIZE];  // Ring Buffer Speicher

int count_max_buffer_fill = 0;

struct k_poll_signal logger_sig;
static struct k_poll_event logger_evt =
		 K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &logger_sig);

SDLogger::SDLogger() {
    sd_card = &sdcard_manager;
    k_mutex_init(&write_mutex);
}

SDLogger::~SDLogger() {

}

//static bool _prio_boost = false;

void sensor_listener_cb(const struct zbus_channel *chan) {
    int ret;
    const sensor_msg* msg = (sensor_msg*)zbus_chan_const_msg(chan);

	if (msg->sd) {
        /*if (!_prio_boost) {
            if (k_msgq_num_free_get(&sd_sensor_queue) < CONFIG_SENSOR_SD_SUB_QUEUE_SIZE / 2) {
                k_thread_priority_set(thread_id, K_PRIO_PREEMPT(CONFIG_SENSOR_SD_THREAD_PRIO - 1));
                
                _prio_boost = true;

                LOG_DBG("SD thread priority boost boost");
            }
        } else if (_prio_boost) {
            if (k_msgq_num_used_get(&sd_sensor_queue) == 0) { // < CONFIG_SENSOR_SD_SUB_QUEUE_SIZE / 4
                k_thread_priority_set(thread_id, K_PRIO_PREEMPT(CONFIG_SENSOR_SD_THREAD_PRIO));
                _prio_boost = false;

                LOG_DBG("End SD thread priority boost boost");
            }
        }*/

        sdlogger.write_sensor_data(msg->data);

		if (ret) {
			LOG_WRN("sd msg queue full");
		}
	}
}


void sd_listener_callback(const struct zbus_channel *chan)
{
    const struct sd_msg * sd_msg_event = (sd_msg *)zbus_chan_const_msg(&sd_card_chan);

    if (sdlogger.is_open && sd_msg_event->removed) {
        k_poll_signal_reset(&logger_sig);

        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("SD card removed mid recording. Stop recording.");

        // sdlogger.end();
        sdlogger.is_open = false;
    }
}


void SDLogger::sensor_sd_task() {
    int ret;

    while (1) {
        ret = k_poll(&logger_evt, 1, K_FOREVER);

        if (!sdcard_manager.is_mounted()) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("SD Card not mounted!");
            return;
        }

        uint32_t fill = ring_buf_size_get(&ring_buffer);

        if (fill >= SD_BLOCK_SIZE) {
            if (fill > count_max_buffer_fill) {
                count_max_buffer_fill = fill;
                //LOG_INF("Max buffer fill: %d bytes", count_max_buffer_fill);
            }
            //k_mutex_lock(&write_mutex, K_FOREVER);
            uint32_t bytes_read;
            uint8_t * data;
            size_t write_size = SD_BLOCK_SIZE;
    
            ring_buf_get_claim(&ring_buffer, &data, SD_BLOCK_SIZE);
            bytes_read = sdlogger.sd_card->write((char*)data, &write_size, false);
            ring_buf_get_finish(&ring_buffer, bytes_read);

            //k_mutex_unlock(&write_mutex);
    
            //fill -= bytes_read;
        } else {
            k_yield();
        }

        if (ret < 0) {
            state_indicator.set_sd_state(SD_FAULT);
            LOG_ERR("Failed to write sensor data: %d", ret);
        }

        STACK_USAGE_PRINT("sensor_msg_thread", &sdlogger.thread_data);
    }
}

int SDLogger::init() {
    int ret;

    sd_card->init();

    ring_buf_init(&ring_buffer, BUFFER_SIZE, buffer);

    //set_ring_buffer(&ring_buffer);

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
    ret = sd_card->open_file(full_filename, true, false, true);
    if (ret < 0) {
        state_indicator.set_sd_state(SD_FAULT);
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    current_file = full_filename;
    is_open = true;
    //buffer_pos = 0;

    ring_buf_reset(&ring_buffer);

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

    return sd_card->write((char *) header_buffer, &header_size, false);
}

int SDLogger::write_sensor_data(const void* const* data_blocks, const size_t* lengths, size_t block_count) {
    k_mutex_lock(&write_mutex, K_FOREVER);

    // Calculate total length needed
    size_t total_length = 0;
    for (size_t i = 0; i < block_count; i++) {
        total_length += lengths[i];
    }

    uint32_t left_space = ring_buf_space_get(&ring_buffer);
    if (left_space < total_length) {
        k_mutex_unlock(&write_mutex);
        LOG_WRN("Not enough space in ring buffer: %d bytes needed, %d bytes available", total_length, left_space);
        return -ENOSPC;
    }
    
    for (size_t i = 0; i < block_count; i++) {
        int written = ring_buf_put(&ring_buffer, (const uint8_t*)data_blocks[i], lengths[i]);
        if (written < lengths[i]) {
            k_mutex_unlock(&write_mutex);
            LOG_ERR("Failed to write data block: %d bytes written, %d bytes requested", written, lengths[i]);
            return -ENOSPC;
        }
    }

    k_mutex_unlock(&write_mutex);
    
    return 0;
}

int SDLogger::write_sensor_data(const sensor_data& msg) {
    const size_t data_size = sizeof(msg.id) + sizeof(msg.size) + sizeof(msg.time) + msg.size;
    const void* msg_ptr = &msg;
    return write_sensor_data(&msg_ptr, &data_size, 1);
}

int SDLogger::flush() {
    uint32_t bytes_read;
    uint8_t * data;
    size_t write_size = SD_BLOCK_SIZE;

    uint32_t fill = ring_buf_size_get(&ring_buffer);

    ring_buf_get_claim(&ring_buffer, &data, fill);

    bytes_read = sd_card->write((char*)ring_buffer.buffer, &write_size, false);

    ring_buf_get_finish(&ring_buffer, bytes_read);

    return bytes_read;
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

    // wait till sensor work queue is empty
	while (k_msgq_num_used_get(&sd_sensor_queue) > 0) {
		k_sleep(K_MSEC(10));
	}

    k_msgq_purge(&sd_sensor_queue);

    ret = flush();
    if (ret < 0) {
        LOG_ERR("Failed to flush file buffer.");
        return ret;
    }

    LOG_INF("Close File ....");

    LOG_DBG("Max buffer fill: %d bytes", count_max_buffer_fill);

    ret = sd_card->close_file();
    if (ret < 0) {
        k_poll_signal_reset(&logger_sig);
        return ret;
    }

    is_open = false;

    k_poll_signal_reset(&logger_sig);

    return 0;
}

bool SDLogger::is_active() {
    return is_open;
}

SDLogger sdlogger;
