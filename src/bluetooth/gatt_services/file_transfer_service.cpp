#include "file_transfer_service.h"

#include "SD_Card_Manager.h"
#include "LittleFSManager.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(file_transfer, LOG_LEVEL_DBG);

#define MAX_CMD_LENGTH 512

static struct k_work command_work;
static struct bt_conn *pending_conn;
static char pending_command[MAX_CMD_LENGTH];
static size_t pending_command_length = 0;

static struct bt_conn *current_conn = NULL;

LittleFsManager littlefs_manager = LittleFsManager();

bool notify_enabled = false;

// --- Forward declarations ---
static ssize_t on_command_write(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len,
                                uint16_t offset, uint8_t flags);

void status_ccc_cfg(const struct bt_gatt_attr *attr,
                    uint16_t value) {
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

// --- BLE GATT service definition ---
BT_GATT_SERVICE_DEFINE(file_transfer_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_FILE_TRANSFER_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_COMMAND_CHARAC,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, on_command_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_STATUS_CHARAC,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(status_ccc_cfg,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
    ),
);

static void file_transfer_command_handler(struct k_work *work);

void file_transfer_service_init(void) {
    k_work_init(&command_work, file_transfer_command_handler);

    LOG_INF("FileTransferService initialized");
}

static void send_status(const char *msg, struct bt_conn *conn = NULL) {
    if (!conn) {
        LOG_WRN("No active connection, cannot send status");
        return;
    }
    if (!notify_enabled) {
        LOG_WRN("Notify not enabled, cannot send status");
        return;
    }

    bt_gatt_notify(conn, &file_transfer_svc.attrs[4], msg, strlen(msg));
    LOG_DBG("Sent status: %s", msg);
}

static ssize_t on_command_write(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len,
                                uint16_t offset, uint8_t flags) {
    current_conn = conn;
    file_transfer_service_on_command_received((const uint8_t *)buf, len, conn);
    return len;
}

// --- Command handler ---
void file_transfer_service_on_command_received(const uint8_t *data, uint16_t length, struct bt_conn *conn) {
    if (length >= MAX_CMD_LENGTH) {
        send_status("ERROR Command too long", conn);
        return;
    }

    LOG_HEXDUMP_DBG(data, length, "Command data");

    memcpy(pending_command, data, length);
    pending_command[length] = '\0';
    pending_command_length = length;
    pending_conn = conn;

    k_work_submit(&command_work);
}

void get_fs_manager(std::string path, FSManager *&fs);
std::string get_remaining_path(std::string path);

void command_list_files(std::string path);
void command_remove_file(std::string path);
void command_write_file(std::string path, char *data, size_t data_length);

static void file_transfer_command_handler(struct k_work *work) {
    std::string command(pending_command);
    char payload[MAX_CMD_LENGTH] = {0};

    size_t payload_length;
    if (command.length() == pending_command_length) {
        payload_length = 0;
    } else if (command.length() + 1 < pending_command_length) {
        payload_length = pending_command_length - command.length() - 1;
    } else {
        send_status("ERROR Invalid command format", current_conn);
        return;
    }
    memcpy(payload, pending_command + command.length() + 1, payload_length);

    LOG_INF("Processing command (deferred): %s", command.c_str());

    if (command.rfind("LIST ", 0) == 0) {
        std::string path = command.substr(5);
        command_list_files(path);
    } else if (command.rfind("REMOVE ", 0) == 0) {
        std::string path = command.substr(7);
        command_remove_file(path);
    } else if (command.rfind("WRITE ", 0) == 0) {
        std::string command_config = command.substr(6);
        command_write_file(command_config, payload, payload_length);
    } else {
        send_status("ERROR Unknown command", current_conn);
    }

}

// MARK: Write

void command_write_file(std::string command_config, char *data, size_t data_length) {
    LOG_DBG("Writing to file with config: %s", command_config.c_str());
    std::string path = command_config;

    FSManager *fs = nullptr;
    get_fs_manager(path, fs);
    if (!fs) {
        send_status("ERROR Invalid filesystem", current_conn);
        return;
    }

    std::string remaining_path = get_remaining_path(path);
    LOG_DBG("Remaining path for write: %s", remaining_path.c_str());

    if (!fs->is_mounted()) {
        LOG_DBG("Mounting filesystem for path: %s", path.c_str());
        int ret = fs->mount();
        if (ret < 0) {
            send_status("ERROR Mount failed", current_conn);
            return;
        }
    }

    LOG_DBG("Writing %d bytes to file: %s", data_length, path.c_str());
    // log the data being written
    LOG_HEXDUMP_DBG(data, data_length, "Data to write");

    ssize_t ret = fs->write(remaining_path, data, &data_length, true);
    if (ret < 0) {
        std::string error_msg = "ERROR Write failed, error code: " + std::to_string(ret);
        send_status(error_msg.c_str(), current_conn);
        return;
    } else {
        LOG_DBG("%d bytes written successfully: %s", ret, path.c_str());
        std::string success_msg = "ACK " + std::to_string(ret);
        send_status(success_msg.c_str(), current_conn);
    }
}

// MARK: Remove

void command_remove_file(std::string path) {
    FSManager *fs = nullptr;
    get_fs_manager(path, fs);
    if (!fs) return;

    std::string remaining_path = get_remaining_path(path);
    LOG_DBG("Removing file at path: %s", remaining_path.c_str());

    int ret = fs->rm(remaining_path);
    if (ret < 0) {
        std::string error_msg = "ERROR Remove failed, error code: " + std::to_string(ret);
        send_status(error_msg.c_str(), current_conn);
        return;
    } else {
        LOG_DBG("File removed successfully: %s", path.c_str());
        send_status("SUCCESS", current_conn);
    }
}

// MARK: List Files

void command_list_files(std::string path) {
    if (path == "/") {
        send_status("[DIR ]\tsd\n[DIR ]\tflash\n", current_conn);
        return;
    }

    FSManager *fs = nullptr;
    get_fs_manager(path, fs);
    if (!fs) return;

    if (!fs->is_mounted()) {
        LOG_DBG("Mounting filesystem for path: %s", path.c_str());
        int ret = fs->mount();
        if (ret < 0) {
            send_status("ERROR Mount failed");
            return;
        }
    }
    LOG_DBG("Filesystem mounted for path: %s", path.c_str());

    std::string remaining_path = get_remaining_path(path);
    
    if (!remaining_path.empty()) {
        int ret = fs->cd(remaining_path);
        if (ret < 0) {
            LOG_ERR("Failed to change directory to: %s", path.c_str());
            send_status(("ERROR Invalid path: " + std::to_string(ret)).c_str());
            return;
        }
    }

    char buf[512];
    size_t buf_size = sizeof(buf);
    int ret = fs->ls(buf, &buf_size);
    if (ret < 0) {
        send_status("ERROR Listing failed", current_conn);
        return;
    }

    buf[buf_size] = '\0';
    send_status(buf, current_conn);
}

// MARK: Utility Functions

void get_fs_manager(std::string path, FSManager *&fs) {
    if (path.rfind("/sd", 0) == 0) {
        fs = &sdcard_manager;
        LOG_DBG("Using SD Card Manager for path: %s", path.c_str());
    } else if (path.rfind("/flash", 0) == 0) {
        fs = &littlefs_manager;
        LOG_DBG("Using LittleFS Manager for path: %s", path.c_str());
    } else {
        LOG_WRN("Unknown filesystem: %s", path.c_str());
        fs = nullptr;
    }
}

std::string get_remaining_path(std::string path) {
    size_t remaining_path_idx = path.find("/", 1);
    std::string remaining_path = "/";
    if (remaining_path_idx != std::string::npos) {
        remaining_path += path.substr(remaining_path_idx);
    }
    return remaining_path;
}
