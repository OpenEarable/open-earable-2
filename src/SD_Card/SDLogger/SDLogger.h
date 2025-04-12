#ifndef _SD_LOGGER_HPP_
#define _SD_LOGGER_HPP_

#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"
#include "zbus_common.h"
#include "SD_Card_Manager.h"
#include <string>


constexpr size_t SD_BLOCK_SIZE = 512;
constexpr size_t BUFFER_BLOCK_COUNT = 8;
constexpr size_t BUFFER_SIZE = SD_BLOCK_SIZE * BUFFER_BLOCK_COUNT;

// BUFFER_SIZE must always be a multiple of SD_BLOCK_SIZE to ensure proper block alignment
// without requiring padding. The SensorLogger implementation assumes this relationship
// and will not work correctly otherwise.
static_assert(BUFFER_SIZE % SD_BLOCK_SIZE == 0, "BUFFER_SIZE must be a multiple of SD_BLOCK_SIZE");

// Forward declare the work handler
static void sd_work_handler(struct k_work* work);

// Singleton pattern
class SDLogger {
protected:
        // Add static instance pointer for work handler
        //static SDLogger* instance_ptr;
        friend void sd_work_handler(struct k_work* work);
        
private:
        
        SDCardManager* sd_card = nullptr;
        //bool owns_sd_card = false;
        bool is_open = false;
        uint8_t buffer[BUFFER_SIZE]; // = nullptr;
        size_t buffer_pos = 0;
        std::string current_file;

        int write_header(); //Write file header with version and timestamp
        int write_sensor_data(); // Write sensor data to the log file from zbus channel
        int flush(); // Flush any buffered data to the SD card
        
        static constexpr uint16_t SENSOR_LOG_VERSION = 0x0001;

        struct __attribute__((packed)) FileHeader {
            uint16_t version;
            uint64_t timestamp;
        };

        struct k_thread thread_data;
        k_tid_t thread_id;
        struct sensor_msg msg;
        struct sensor_data* const data_buf = &(msg.data);
        static void sensor_sd_task();

        friend void sd_listener_callback(const struct zbus_channel *chan);

    public:
        SDLogger();
        ~SDLogger();
        /*static SDLogger * get_instance(SDCardManager* sd_card_manager = nullptr) {
            static SDLogger instance(sd_card_manager);
            return &instance;
        }*/

        /**
        * @brief Begin logging to a new file
        * @param filename Base filename without extension (.oe will be appended)
        * @return 0 on success, negative error code on failure
        */
       int init();

        /**
        * @brief Begin logging to a new file
        * @param filename Base filename without extension (.oe will be appended)
        * @return 0 on success, negative error code on failure
        */
        int begin(const std::string& filename);

        /**
        * @brief Write sensor data to the log file
        * @param data Pointer to data buffer to write
        * @param length Number of bytes to write
        * @return 0 on success, negative error code on failure
        */
        int write_sensor_data(const void* data, size_t length);

        /**
        * @brief End logging and close the current file
        * @return 0 on success, negative error code on failure
        */
        int end();

        SDLogger(SDLogger const&) = delete;
        SDLogger& operator=(SDLogger const&) = delete;
};

extern SDLogger sdlogger;

#endif