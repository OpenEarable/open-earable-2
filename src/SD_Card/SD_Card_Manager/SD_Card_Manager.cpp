#include "SD_Card_Manager.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SDCardManager, LOG_LEVEL_DBG);

#define SD_ROOT_PATH	      "/SD:"
#define PATH_MAX_LEN	      260
#define K_SEM_OPER_TIMEOUT_MS 100

//K_SEM_DEFINE(m_sem_sd_mngr_oper_ongoing, 1, 1);
K_MUTEX_DEFINE(m_sem_sd_mngr_oper_ongoing);

SDCardManager::SDCardManager(): path(SD_ROOT_PATH) {
	fs_dir_t_init(&this->dirp);
	//k_sem_reset(&m_sem_sd_mngr_oper_ongoing);
}

std::string create_path(std::string current_path, std::string new_path) {
	if (new_path.empty()) {
		return SD_ROOT_PATH;
	} else if (new_path[0] == '/') {
		return SD_ROOT_PATH + new_path;
	} else {
		return current_path + "/" + new_path;
	}
}

SDCardManager::~SDCardManager() {
	
}

int SDCardManager::unmount() {
	if (this->mounted) {
		fs_closedir(&this->dirp);
		if (this->tracked_file.is_open) {
			this->close_file();
		}
		fs_unmount(&this->mnt_pt);
	}

	pm_device_runtime_put(ls_sd);
	pm_device_runtime_put(ls_3_3);
	pm_device_runtime_put(ls_1_8);
}

int SDCardManager::mount() {
	int ret;
	static const char* sd_dev = "SD";

	uint64_t sd_card_size_bytes;
	uint32_t sector_count;
	size_t sector_size;

	ret = pm_device_runtime_get(ls_1_8);
	ret = pm_device_runtime_get(ls_3_3);
	ret = pm_device_runtime_get(ls_sd);

	ret = disk_access_init(sd_dev);
	if (ret) {
		LOG_DBG("SD card init failed, please check if SD card inserted");
		return -ENODEV;
	}

	ret = disk_access_ioctl(sd_dev, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	if (ret) {
		LOG_ERR("Unable to get sector count");
		return ret;
	}

	LOG_DBG("Sector count: %d", sector_count);

	ret = disk_access_ioctl(sd_dev, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	if (ret) {
		LOG_ERR("Unable to get sector size");
		return ret;
	}

	LOG_DBG("Sector size: %d bytes", sector_size);

	sd_card_size_bytes = (uint64_t)sector_count * sector_size;

	LOG_INF("SD card volume size: %d MB", (uint32_t)(sd_card_size_bytes >> 20));

	this->mnt_pt.mnt_point = SD_ROOT_PATH;

	ret = fs_mount(&this->mnt_pt);
	if (ret) {
		this->mounted = ret == -EBUSY;

		LOG_ERR("Mnt. disk failed, could be format issue. should be FAT/exFAT. Error: %d", ret);

		if (ret != -EBUSY) {
			return ret;
		}
	}

	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	LOG_DBG("Root dir: %s", this->path.c_str());
	ret = fs_opendir(&this->dirp, this->path.c_str());
	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	if (ret) {
		LOG_ERR("Open root dir failed. Error: %d", ret);
		return ret;
	}

	this->mounted = true;
	return 0;
}

int SDCardManager::cd(std::string path) {
	LOG_INF("Changing dir to %s", path.c_str());

	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (!this->mounted) {
		LOG_ERR("SD card not mounted! Call SDCardManager::mount() first!");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -ENODEV;
	}

	if (path.length() > CONFIG_FS_FATFS_MAX_LFN) {
		LOG_ERR("Path is too long");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -FR_INVALID_NAME;
	}

	ret = fs_closedir(&this->dirp);
	if (ret) {
		LOG_ERR("Close SD card dir failed");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	std::string abs_path_name = create_path(this->path, path);

	LOG_DBG("abs path name:\t%s", abs_path_name.c_str());

	fs_dir_t_init(&this->dirp);

	ret = fs_opendir(&this->dirp, abs_path_name.c_str());
	if (ret) {
		LOG_ERR("Open SD card dir failed: %d", ret);
		// Try to revert to the previous path if the new one fails
		if (this->path != path) {
			int rret = fs_opendir(&this->dirp, this->path.c_str());
			if (rret) {
				LOG_ERR("Failed to cd back to previous dir: %d", rret);
			}
		}
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	this->path = abs_path_name;

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

int SDCardManager::ls(char *buf, size_t *buf_size) {
	int ret;
	static struct fs_dirent entry;
	size_t used_buf_size = 0;

	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (!this->mounted) {
		LOG_ERR("SD card not mounted! Call SDCardManager::mount() first!");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -ENODEV;
	}

	if (this->path.length() > CONFIG_FS_FATFS_MAX_LFN) {
		LOG_ERR("Path is too long");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -FR_INVALID_NAME;
	}

	while (1) {
		ret = fs_readdir(&this->dirp, &entry);
		if (ret) {
			k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
			return ret;
		}

		if (entry.name[0] == 0) {
			break;
		}

		if (buf != NULL) {
			size_t remaining_buf_size = *buf_size - used_buf_size;
			ssize_t len = snprintk(
				&buf[used_buf_size], remaining_buf_size, "[%s]\t%s\n",
				entry.type == FS_DIR_ENTRY_DIR ? "DIR " : "FILE", entry.name);

			if (len >= remaining_buf_size) {
				LOG_ERR("Failed to append to buffer, error: %d", len);
				k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
				return -EINVAL;
			}

			used_buf_size += len;
		}

		LOG_INF("[%s] %s", entry.type == FS_DIR_ENTRY_DIR ? "DIR " : "FILE", entry.name);
	}

	*buf_size = used_buf_size;
	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

int SDCardManager::mkdir(std::string path) {
	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (!this->mounted) {
		LOG_ERR("SD card not mounted! Call SDCardManager::mount() first!");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -ENODEV;
	}

	if (path.length() > CONFIG_FS_FATFS_MAX_LFN) {
		LOG_ERR("Path is too long");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -FR_INVALID_NAME;
	}

	std::string abs_path_name = create_path(this->path, path);

	ret = fs_mkdir(abs_path_name.c_str());
	if (ret) {
		LOG_ERR("Failed to create dir: %d", ret);
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

int SDCardManager::open_file(std::string path, bool write, bool append, bool create) {
	if (!this->mounted) {
		LOG_ERR("SD card not mounted! Call SDCardManager::mount() first!");
		return -ENODEV;
	}

	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (path.length() > CONFIG_FS_FATFS_MAX_LFN) {
		LOG_ERR("Path is too long");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -ENAMETOOLONG;
	}

	std::string abs_path_name = create_path(this->path, path);

	if (this->tracked_file.is_open) {
		LOG_ERR("File is already open");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -EBUSY;
	}

	fs_mode_t flags = FS_O_READ;
	if (write) {
		flags |= FS_O_WRITE;
	}
	if (append) {
		flags |= FS_O_APPEND;
	}
	if (create) {
		flags |= FS_O_CREATE;
	}

	fs_file_t_init(&this->tracked_file.filep);

	ret = fs_open(&this->tracked_file.filep, abs_path_name.c_str(), flags);
	if (ret) {
		LOG_ERR("Failed to open file: %d", ret);
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	this->tracked_file.is_open = true;
	this->path = abs_path_name;

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

int SDCardManager::close_file() {
	if (!this->mounted) {
		LOG_ERR("SD card not mounted! Call SDCardManager::mount() first!");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -ENODEV;
	}

	int ret;

	if (!this->tracked_file.is_open) {
		LOG_INF("File is not open");
		return 0;
	}

	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	ret = fs_close(&this->tracked_file.filep);
	if (ret) {
		LOG_ERR("Failed to close file: %d", ret);
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	LOG_DBG("File %s closed", this->path.c_str());
	size_t last_slash_pos = this->path.find_last_of("/");
	if (last_slash_pos != std::string::npos) {
		this->path = this->path.substr(0, last_slash_pos);
	}
	this->tracked_file.is_open = false;

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

ssize_t SDCardManager::write(char *buf, size_t *buf_size, bool sync) {
	if (!this->tracked_file.is_open) {
		LOG_ERR("File is not open");
		return -EINVAL;
	}

	if (!(this->tracked_file.filep.flags & FS_O_WRITE)) {
		LOG_ERR("File is not open for writing");
		return -EINVAL;
	}

	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	ret = fs_write(&this->tracked_file.filep, buf, *buf_size);
	if (ret < 0) {
		LOG_ERR("Write file failed. Ret: %d", ret);
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	if (sync) {
		ret = fs_sync(&this->tracked_file.filep);
		if (ret) {
			LOG_ERR("Failed to sync file: %d", ret);
			k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
			return ret;
		}
	}

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);

	return ret;
}

ssize_t SDCardManager::write(std::string path, char *buf, size_t *buf_size, bool append) {
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

int SDCardManager::read(char *buffer, size_t *buf_size) {
	if (!this->tracked_file.is_open) {
		LOG_ERR("No file opened");
		return -1;
	}
	
	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);

	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	ret = fs_read(&(this->tracked_file.filep), buffer, *buf_size);
	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	if (ret < 0) {
		LOG_ERR("Read file failed. Ret: %d", ret);
		return ret;
	}

	*buf_size = ret;

	return 0;
}

int SDCardManager::open_write_close(std::string path, char *buf, size_t *buf_size, bool append, bool create) {
    int ret = this->open_file(path, true, append, create);
    if (ret) {
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    ret = this->write(buf, buf_size, true);
    if (ret < 0) {
        LOG_ERR("Write failed: %d", ret);
        this->close_file();
        return ret;
    }

    ret = this->close_file();
    if (ret) {
        LOG_ERR("Failed to close file: %d", ret);
        return ret;
    }

    return 0;
}

int SDCardManager::open_read_close(std::string path, char *buf, size_t *buf_size) {
    int ret = this->open_file(path, false, false, false);
    if (ret) {
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    ret = this->read(buf, buf_size);
    if (ret) {
        LOG_ERR("Read failed: %d", ret);
        this->close_file();
        return ret;
    }

    ret = this->close_file();
    if (ret) {
        LOG_ERR("Failed to close file: %d", ret);
        return ret;
    }

    return 0;
}

int SDCardManager::rm(std::string path) {
	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (!this->mounted) {
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -ENODEV;
	}

	if (path.length() > CONFIG_FS_FATFS_MAX_LFN) {
		LOG_ERR("Path is too long");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -FR_INVALID_NAME;
	}

	std::string abs_path_name = create_path(this->path, path);

	ret = fs_unlink(abs_path_name.c_str());
	if (ret) {
		LOG_ERR("Failed to remove file: %d", ret);
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

int SDCardManager::sync() {
	if (!this->mounted) {
		return -ENODEV;
	}

	int ret;
	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	ret = fs_sync(&this->tracked_file.filep);
	if (ret) {
		LOG_ERR("Failed to sync file: %d", ret);
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return ret;
	}

	k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	return 0;
}

SDCardManager sdcard_manager;