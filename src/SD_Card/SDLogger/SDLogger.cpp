#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"
#include "macros_common.h"
#include <zephyr/logging/log.h>
#include "SDLogger.h"
#include <errno.h>

LOG_MODULE_REGISTER(sd_logger, LOG_LEVEL_DBG);

// Initialize static member
//SDLogger* SDLogger::instance_ptr = nullptr;

// Define thread stack
K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE * 4);

ZBUS_SUBSCRIBER_DEFINE(sensor_sd_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
ZBUS_CHAN_DECLARE(sensor_chan);
K_WORK_DEFINE(sd_sensor_work, sd_work_handler);

SDLogger::SDLogger() {
    sd_card = &sdcard_manager;
}

SDLogger::~SDLogger() {
    if (is_open) {
        end();
    }
}

// Define the work handler implementation
void sd_work_handler(struct k_work* work) {
    if (sdlogger.msg.sd) {
        int ret = sdlogger.write_sensor_data();
        if (ret < 0) {
            LOG_ERR("Failed to write sensor data: %d", ret);
        }
    }
}

void SDLogger::sensor_sd_task() {
    int ret;
    const struct zbus_channel* chan;

    while (1) {
        ret = zbus_sub_wait(&sensor_sd_sub, &chan, K_FOREVER);
        ERR_CHK(ret);

        ret = zbus_chan_read(chan, &sdlogger.msg, ZBUS_READ_TIMEOUT_MS);
        ERR_CHK(ret);

        k_work_submit(&sd_sensor_work);

        STACK_USAGE_PRINT("sensor_msg_thread", &sdlogger.thread_data);
    }
}

int SDLogger::init() {
    int ret;

    sd_card->init();

	thread_id = k_thread_create(
		&thread_data, thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE * 4, (k_thread_entry_t)sensor_sd_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	
	ret = k_thread_name_set(thread_id, "SENSOR_SD_SUB");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_sd_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sensor sub");
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
            LOG_ERR("Failed to mount sd card: %d", ret);
            return ret;
        }
    }

    std::string full_filename = filename + ".oe";
    ret = sd_card->open_file(full_filename, true, false, true);
    if (ret < 0) {
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    current_file = full_filename;
    is_open = true;
    buffer_pos = 0;

    ret = write_header();
    if (ret < 0) {
        LOG_ERR("Failed to write header: %d", ret);
        return ret;
    }

    k_thread_resume(thread_id);

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

int SDLogger::write_sensor_data(const void* data, size_t length) {
    const uint8_t* src = static_cast<const uint8_t*>(data);
    
    while (length > 0) {
        size_t space = BUFFER_SIZE - buffer_pos;
        size_t to_copy = std::min(length, space);
        
        memcpy(&buffer[buffer_pos], src, to_copy);
        buffer_pos += to_copy;
        src += to_copy;
        length -= to_copy;

        if (buffer_pos == BUFFER_SIZE) {
            //LOG_INF("flushing .....");
            int ret = flush();
            if (ret < 0) {
                return ret;
            }
        }
    }
    
    return 0;
}

int SDLogger::write_sensor_data() {
    const uint16_t data_size = sizeof(data_buf->id) + sizeof(data_buf->size) + sizeof(data_buf->time) + data_buf->size;
    return write_sensor_data(data_buf, data_size);
}

int SDLogger::flush() {
    if (buffer_pos == 0) {
        return 0;
    }
    // During normal writes, buffer_pos will be exactly BUFFER_SIZE
    // During end(), buffer_pos may be any value and the filesystem handles any necessary block alignment
    
    size_t write_size = buffer_pos;
    int ret = sd_card->write((char *) buffer, &write_size, false);
    if (ret >= 0) {
        buffer_pos = 0;
    }
    return ret;
}

int SDLogger::end() {
    if (!is_open) {
        return -ENODEV;
    }

    int ret = flush();
    if (ret < 0) {
        return ret;
    }

    LOG_INF("Close File ....");

    ret = sd_card->close_file();
    if (ret < 0) {
        return ret;
    }

    is_open = false;

    k_thread_suspend(thread_id);

    return 0;
}

SDLogger sdlogger;
