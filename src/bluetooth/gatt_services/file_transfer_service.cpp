#include "file_transfer_service.h"

#include "SD_Card_Manager.h"
#include "LittleFSManager.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(file_transfer, LOG_LEVEL_DBG);

#define MAX_CMD_LENGTH 128

static struct k_work command_work;
static struct bt_conn *pending_conn;
static char pending_command[MAX_CMD_LENGTH];

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

    memcpy(pending_command, data, length);
    pending_command[length] = '\0';
    pending_conn = conn;

    k_work_submit(&command_work);
}

static void file_transfer_command_handler(struct k_work *work) {
    std::string command(pending_command);
    LOG_INF("Processing command (deferred): %s", command.c_str());

    if (command.rfind("LIST ", 0) == 0) {
        std::string path = command.substr(5);

        FSManager *fs = nullptr;
        if (path.rfind("/sd", 0) == 0) {
            fs = &sdcard_manager;
            LOG_DBG("Using SD Card Manager for path: %s", path.c_str());
        } else if (path.rfind("/flash", 0) == 0) {
            fs = &littlefs_manager;
            LOG_DBG("Using LittleFS Manager for path: %s", path.c_str());
        } else if (path == "/") {
            send_status("[DIR ]\tsd\n[DIR ]\tflash\n", current_conn);
            return;
        } else {
            LOG_WRN("Unknown filesystem: %s", path.c_str());
            send_status("ERROR Unknown FS", current_conn);
            return;
        }

        if (!fs->is_mounted()) {
            LOG_DBG("Mounting filesystem for path: %s", path.c_str());
            int ret = fs->mount();
            if (ret < 0) {
                send_status("ERROR Mount failed");
                return;
            }
        }
        LOG_DBG("Filesystem mounted for path: %s", path.c_str());

        size_t remaining_path_idx = path.find("/", 1);
        if (remaining_path_idx == std::string::npos) {
            LOG_DBG("No subdirectory specified, listing root");
        } else {
            std::string remaining_path = path.substr(remaining_path_idx);
            LOG_DBG("Changing directory to: %s", remaining_path.c_str());

            if (!remaining_path.empty()) {
                if (fs->cd(remaining_path) < 0) {
                    LOG_ERR("Failed to change directory to: %s", path.c_str());
                    send_status("ERROR Invalid path");
                    return;
                }
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
        return;
    }

    send_status("ERROR Unknown command", current_conn);
}