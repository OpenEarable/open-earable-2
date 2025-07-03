#include "LittleFSManager.h"

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LittleFsManager, LOG_LEVEL_DBG);

#define LFS_ROOT_PATH "/flash"
#define PATH_MAX_LEN 260

K_MUTEX_DEFINE(m_sem_lfs_mngr_oper_ongoing);

static std::string create_lfs_path(std::string current_path, std::string new_path) {
    if (new_path.empty()) return LFS_ROOT_PATH;
    if (new_path[0] == '/') return LFS_ROOT_PATH + new_path;
    return current_path + "/" + new_path;
}

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(lfs_storage);
static struct fs_mount_t lfs_mnt = {
    .type = FS_LITTLEFS,
    .mnt_point = LFS_ROOT_PATH,
    .fs_data = &lfs_storage,
    .storage_dev = (void *)FIXED_PARTITION_ID(micropython_lfs),
    .flags = 0,
};

LittleFsManager::LittleFsManager() : path(LFS_ROOT_PATH) {
    fs_dir_t_init(&this->dirp);
}

LittleFsManager::~LittleFsManager() {
    unmount();
}

int LittleFsManager::mount() {
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

int LittleFsManager::unmount() {
    if (!this->mounted) return 0;

    if (this->tracked_file.is_open) {
        close_file();
    }

    fs_closedir(&this->dirp);
    fs_unmount(&lfs_mnt);

    this->mounted = false;
    return 0;
}

int LittleFsManager::cd(std::string new_path) {
    if (!this->mounted) return -ENODEV;

    std::string abs_path = create_lfs_path(this->path, new_path);
    fs_closedir(&this->dirp);

    int ret = fs_opendir(&this->dirp, abs_path.c_str());
    if (ret) {
        LOG_ERR("fs_opendir failed: %d", ret);
        fs_opendir(&this->dirp, this->path.c_str()); // revert
        return ret;
    }

    this->path = abs_path;
    return 0;
}

int LittleFsManager::ls(char *buf, size_t *buf_size) {
    if (!this->mounted) return -ENODEV;

    size_t used_buf_size = 0;
    struct fs_dirent entry;

    while (true) {
        int ret = fs_readdir(&this->dirp, &entry);
        if (ret < 0) return ret;
        if (entry.name[0] == 0) break;

        if (buf != nullptr) {
            size_t remaining = *buf_size - used_buf_size;
            int len = snprintk(
                &buf[used_buf_size], remaining, "[%s]\t%s\n",
                entry.type == FS_DIR_ENTRY_DIR ? "DIR " : "FILE", entry.name);

            if (len < 0 || (size_t)len >= remaining) return -ENOMEM;
            used_buf_size += len;
        }
    }

    *buf_size = used_buf_size;
    return 0;
}

int LittleFsManager::mkdir(std::string subdir) {
    if (!this->mounted) return -ENODEV;
    std::string abs_path = create_lfs_path(this->path, subdir);
    return fs_mkdir(abs_path.c_str());
}

int LittleFsManager::open_file(std::string filename, bool write, bool append, bool create) {
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
        this->path = abs_path;
    }
    return ret;
}

int LittleFsManager::close_file() {
    if (!this->tracked_file.is_open) return 0;
    int ret = fs_close(&this->tracked_file.filep);
    this->tracked_file.is_open = false;
    return ret;
}

ssize_t LittleFsManager::write(char *buf, size_t *buf_size, bool sync) {
    if (!this->tracked_file.is_open) return -EINVAL;
    ssize_t ret = fs_write(&this->tracked_file.filep, buf, *buf_size);
    if (ret < 0) return ret;
    if (sync) fs_sync(&this->tracked_file.filep);
    return ret;
}

ssize_t LittleFsManager::write(std::string path, char *buf, size_t *buf_size, bool append) {
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

int LittleFsManager::sync() {
    if (!this->tracked_file.is_open) return -EINVAL;
    return fs_sync(&this->tracked_file.filep);
}

int LittleFsManager::read(char *buf, size_t *buf_size) {
    if (!this->tracked_file.is_open) return -EINVAL;
    ssize_t ret = fs_read(&this->tracked_file.filep, buf, *buf_size);
    if (ret < 0) return ret;
    *buf_size = ret;
    return 0;
}

int LittleFsManager::open_write_close(std::string path, char *buf, size_t *buf_size, bool append, bool create) {
    int ret = open_file(path, true, append, create);
    if (ret) return ret;

    ret = write(buf, buf_size, true);
    close_file();
    return ret;
}

int LittleFsManager::open_read_close(std::string path, char *buf, size_t *buf_size) {
    int ret = open_file(path, false, false, false);
    if (ret) return ret;

    ret = read(buf, buf_size);
    close_file();
    return ret;
}

int LittleFsManager::rm(std::string path) {
    if (!this->mounted) return -ENODEV;
    std::string abs_path = create_lfs_path(this->path, path);
    return fs_unlink(abs_path.c_str());
}

bool LittleFsManager::is_mounted() const {
    return this->mounted;
}