#include "ModdedLogger.h"
#include <cstring>

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ModdedLogger, LOG_LEVEL_DBG);

#define LFS_ROOT_PATH "/flash"
#define DEFAULT_LOG_FILE "modded_log.txt"


static std::string create_lfs_path(const std::string &current_path, const std::string &new_path) {
    if (new_path.empty()) return LFS_ROOT_PATH;
    if (new_path[0] == '/') return LFS_ROOT_PATH + new_path;
    return current_path + "/" + new_path;
}

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(lfs_storage);
static struct fs_mount_t lfs_mnt = {
    .type = FS_LITTLEFS,
    .mnt_point = LFS_ROOT_PATH,
    .fs_data = &lfs_storage,
    .storage_dev = (void *)FIXED_PARTITION_ID(moddedlogger_partition),
    .flags = 0,
};

ModdedLogger::ModdedLogger() : path(LFS_ROOT_PATH) {
    fs_dir_t_init(&this->dirp);
    int ret = this->mount();
    if (ret != 0) {
        LOG_ERR("Failed to mount! Error: %d\n", ret);
    }
}

ModdedLogger::~ModdedLogger() {
    unmount();
}

int ModdedLogger::mount() {
    if (this->mounted) return 0;

    int ret = fs_mount(&lfs_mnt);
    if (ret < 0) {
        LOG_ERR("LittleFS mount failed: %d", ret);
        return ret;
    }

    fs_dir_t_init(&this->dirp);
    ret = fs_opendir(&this->dirp, LFS_ROOT_PATH);
    if (ret < 0) {
        LOG_ERR("Failed to open root dir: %d", ret);
        fs_unmount(&lfs_mnt);
        return ret;
    }

    this->mounted = true;
    return 0;
}

int ModdedLogger::unmount() {
    if (!this->mounted) return 0;

    if (this->tracked_file.is_open) {
        close_file();
    }

    fs_closedir(&this->dirp);
    fs_unmount(&lfs_mnt);

    this->mounted = false;
    return 0;
}


int ModdedLogger::log_flash_err(const char* input) {
    if (!this->mounted) {
        LOG_ERR("FS not mounted, cannot log to flash.");
        return -ENODEV;
    }

    uint64_t uptime_ticks = k_uptime_ticks();
    uint64_t total_us = k_ticks_to_us_floor64(uptime_ticks);
    uint32_t us = total_us % 1000;
    uint64_t total_ms = total_us / 1000;
    uint32_t ms = total_ms % 1000;
    uint64_t total_sec = total_ms / 1000;
    uint32_t sec = total_sec % 60;
    uint64_t total_min = total_sec / 60;
    uint32_t min = total_min % 60;
    uint32_t hours = total_min / 60; 
    
    char time_buf[64];
    snprintf(time_buf, sizeof(time_buf), "-- [%02u:%02u:%02u.%03u,%03u] ", 
             hours, min, sec, ms, us);

    std::string message = std::string(time_buf) + "<log_flash_err>" + input + "\n";

    size_t len = message.length(); 
    
    ssize_t ret = this->write(DEFAULT_LOG_FILE, message.c_str(), &len, true);

    if (ret < 0) {
        LOG_ERR("moddedlogger failed: %d", ret);
        return (int)ret;
    }
    this->sync();
    LOG_ERR("%s", input);
    
    return 0;
}

int ModdedLogger::log_flash_dbg(const char* input) {
    if (!this->mounted) {
        LOG_ERR("FS not mounted, cannot log to flash.");
        return -ENODEV;
    }
    uint64_t uptime_ticks = k_uptime_ticks();
    uint64_t total_us = k_ticks_to_us_floor64(uptime_ticks);
    uint32_t us = total_us % 1000;
    uint64_t total_ms = total_us / 1000;
    uint32_t ms = total_ms % 1000;
    uint64_t total_sec = total_ms / 1000;
    uint32_t sec = total_sec % 60;
    uint64_t total_min = total_sec / 60;
    uint32_t min = total_min % 60;
    uint32_t hours = total_min / 60; 
    
    char time_buf[64];
    snprintf(time_buf, sizeof(time_buf), "-- [%02u:%02u:%02u.%03u,%03u] ", 
             hours, min, sec, ms, us);

    std::string message = std::string(time_buf) + "<log_flash_dbg>" + input + "\n";

    size_t len = message.length(); 
    
    ssize_t ret = this->write(DEFAULT_LOG_FILE, message.c_str(), &len, true);

    if (ret < 0) {
        LOG_ERR("moddedlogger failed: %d", ret);
        return (int)ret;
    }
    
    this->sync();
    LOG_DBG("%s", input);
    return 0;
}

int ModdedLogger::log_flash_inf(const char* input) {
    if (!this->mounted) {
        LOG_ERR("FS not mounted, cannot log to flash.");
        return -ENODEV;
    }
    uint64_t uptime_ticks = k_uptime_ticks();
    uint64_t total_us = k_ticks_to_us_floor64(uptime_ticks);
    uint32_t us = total_us % 1000;
    uint64_t total_ms = total_us / 1000;
    uint32_t ms = total_ms % 1000;
    uint64_t total_sec = total_ms / 1000;
    uint32_t sec = total_sec % 60;
    uint64_t total_min = total_sec / 60;
    uint32_t min = total_min % 60;
    uint32_t hours = total_min / 60; 
    
    char time_buf[64];
    snprintf(time_buf, sizeof(time_buf), "-- [%02u:%02u:%02u.%03u,%03u] ", 
             hours, min, sec, ms, us);

    std::string message = std::string(time_buf) + "<log_flash_inf>" + input + "\n";

    size_t len = message.length(); 
    
    ssize_t ret = this->write(DEFAULT_LOG_FILE, message.c_str(), &len, true);

    if (ret < 0) {
        LOG_ERR("moddedlogger failed: %d", ret);
        return (int)ret;
    }
    
    this->sync();
    LOG_INF("%s", input);
    return 0;
}

int ModdedLogger::log_flash_wrn(const char* input) {
    if (!this->mounted) {
        LOG_ERR("FS not mounted, cannot log to flash.");
        return -ENODEV;
    }
    uint64_t uptime_ticks = k_uptime_ticks();
    uint64_t total_us = k_ticks_to_us_floor64(uptime_ticks);
    uint32_t us = total_us % 1000;
    uint64_t total_ms = total_us / 1000;
    uint32_t ms = total_ms % 1000;
    uint64_t total_sec = total_ms / 1000;
    uint32_t sec = total_sec % 60;
    uint64_t total_min = total_sec / 60;
    uint32_t min = total_min % 60;
    uint32_t hours = total_min / 60; 
    
    char time_buf[64];
    snprintf(time_buf, sizeof(time_buf), "-- [%02u:%02u:%02u.%03u,%03u] ", 
             hours, min, sec, ms, us);

    std::string message = std::string(time_buf) + "<log_flash_wrn>" + input + "\n";

    size_t len = message.length(); 
    
    ssize_t ret = this->write(DEFAULT_LOG_FILE, message.c_str(), &len, true);

    if (ret < 0) {
        LOG_ERR("moddedlogger failed: %d", ret);
        return (int)ret;
    }
    
    this->sync();
    LOG_WRN("%s", input);
    return 0;
}


int ModdedLogger::cd(const std::string &new_path) {
    if (!this->mounted) return -ENODEV;

    std::string abs_path = create_lfs_path(this->path, new_path);
    fs_closedir(&this->dirp);

    int ret = fs_opendir(&this->dirp, abs_path.c_str());
    if (ret) {
        LOG_ERR("fs_opendir failed: %d", ret);
        fs_opendir(&this->dirp, this->path.c_str());
        return ret;
    }

    this->path = abs_path;
    return 0;
}

int ModdedLogger::ls(char *buf, size_t *buf_size) {
    if (!this->mounted) return -ENODEV;

    struct fs_dir_t local_dirp;
    fs_dir_t_init(&local_dirp);

    int ret = fs_opendir(&local_dirp, this->path.c_str());
    if (ret < 0) return ret;

    size_t used_buf_size = 0;
    struct fs_dirent entry;

    while (true) {
        ret = fs_readdir(&local_dirp, &entry);
        if (ret < 0) break;            
        if (entry.name[0] == 0) break; 

        if (buf != nullptr) {
            size_t remaining = *buf_size - used_buf_size;
            if (remaining < 2) { 
                ret = -ENOMEM;
                break;
            }

            int len = snprintk(
                &buf[used_buf_size], remaining - 1, // Leave space for \0
                "[%s] %s\n",
                entry.type == FS_DIR_ENTRY_DIR ? "DIR " : "FILE", entry.name);

            if (len < 0) {
                 ret = len; 
                 break;
            }
            used_buf_size += len;
        }
    }

    fs_closedir(&local_dirp);
    
    if (buf != nullptr && *buf_size > 0) {
        if (used_buf_size >= *buf_size) used_buf_size = *buf_size - 1;
        buf[used_buf_size] = '\0';
    }

    *buf_size = used_buf_size;
    
    return (ret < 0) ? ret : 0;
}

int ModdedLogger::mkdir(const std::string &subdir) {
    if (!this->mounted) return -ENODEV;
    std::string abs_path = create_lfs_path(this->path, subdir);
    return fs_mkdir(abs_path.c_str());
}

int ModdedLogger::open_file(const std::string &filename, bool write, bool append, bool create) {
    if (!this->mounted || this->tracked_file.is_open) return -EBUSY;

    std::string abs_path = create_lfs_path(this->path, filename);
    fs_mode_t flags = FS_O_READ;

    if (write) flags |= FS_O_WRITE;
    if (append) flags |= FS_O_APPEND;
    if (create) flags |= FS_O_CREATE;

    fs_file_t_init(&this->tracked_file.filep);
    int ret = fs_open(&this->tracked_file.filep, abs_path.c_str(), flags);
    if (ret == 0) {
        this->tracked_file.is_open = true;
    }
    return ret;
}

int ModdedLogger::close_file() {
    if (!this->tracked_file.is_open) return 0;
    int ret = fs_close(&this->tracked_file.filep);
    this->tracked_file.is_open = false;
    return ret;
}

ssize_t ModdedLogger::write(const char *buf, size_t *buf_size, bool sync) {
    if (!this->tracked_file.is_open) return -EINVAL;
    ssize_t ret = fs_write(&this->tracked_file.filep, buf, *buf_size);
    if (ret < 0) return ret;
    if (sync) fs_sync(&this->tracked_file.filep);
    return ret;
}

ssize_t ModdedLogger::write(const std::string &path, const char *buf, size_t *buf_size, bool append) {
    int ret = this->open_file(path, true, append, true);
    if (ret) {
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }
    ssize_t written_size = this->write(buf, buf_size);
    if (written_size < 0) {
        LOG_ERR("Failed to write to file: %d", written_size);
    }

    ret = this->close_file();
    if (ret) {
        LOG_ERR("Failed to close file: %d", ret);
        return ret;
    }

    return written_size;
}

int ModdedLogger::sync() {
    if (!this->tracked_file.is_open) return -EINVAL;
    return fs_sync(&this->tracked_file.filep);
}

int ModdedLogger::read(char *buf, size_t *buf_size) {
    if (!this->tracked_file.is_open) return -EINVAL;
    ssize_t ret = fs_read(&this->tracked_file.filep, buf, *buf_size);
    if (ret < 0) return ret;
    *buf_size = ret;
    return 0;
}

int ModdedLogger::open_write_close(const std::string &path, char *buf, size_t *buf_size, bool append, bool create) {
    int ret = open_file(path, true, append, create);
    if (ret) return ret;

    ret = write((const char*)buf, buf_size, true);
    close_file();
    return ret;
}

int ModdedLogger::open_read_close(const std::string &path, char *buf, size_t *buf_size) {
    int ret = open_file(path, false, false, false);
    if (ret) return ret;

    ret = read(buf, buf_size);
    close_file();
    return ret;
}

int ModdedLogger::rm(const std::string &path) {
    if (!this->mounted) return -ENODEV;
    std::string abs_path = create_lfs_path(this->path, path);
    return fs_unlink(abs_path.c_str());
}

bool ModdedLogger::is_mounted() const {
    return this->mounted;
}