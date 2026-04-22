#include "SD_Card_Manager.h"

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/atomic.h>
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
#define SD_DEBOUNCE_MS K_MSEC(100)

namespace {
constexpr const char *SD_DEV = "SD";
constexpr uint32_t SD_POWER_SETTLE_MS = 20U;
constexpr uint32_t SD_POWER_CYCLE_OFF_MS = 40U;
constexpr int SD_MOUNT_MAX_ATTEMPTS = 2;
constexpr int SD_HOTPLUG_WORK_Q_PRIO = 4;
constexpr int SD_HOTPLUG_WORK_Q_STACK_SIZE = 2048;

atomic_t sd_removal_seen = ATOMIC_INIT(0);

K_THREAD_STACK_DEFINE(sd_hotplug_work_q_stack, SD_HOTPLUG_WORK_Q_STACK_SIZE);
struct k_work_q sd_hotplug_work_q;
struct k_work_queue_config sd_hotplug_work_q_config = {
	.name = "sd_hotplug",
	.no_yield = false,
};

int force_sd_deinit()
{
	bool force = true;
	int ret = disk_access_ioctl(SD_DEV, DISK_IOCTL_CTRL_DEINIT, &force);

	if (ret && ret != -EINVAL) {
		LOG_WRN("Forced SD deinit failed: %d", ret);
	}

	return ret;
}
} /* namespace */

K_MUTEX_DEFINE(m_sem_sd_mngr_oper_ongoing);

ZBUS_CHAN_DEFINE(sd_card_chan, struct sd_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
	ZBUS_MSG_INIT(0));


bool SDCardManager::sd_inserted() {
	int sd_inserted = gpio_pin_get_dt(&sdcard_manager.sd_state_pin);

	return sd_inserted == 1;
}

void SDCardManager::unmount_work_handler(struct k_work *work) {
	int ret;
	bool removed_seen = atomic_cas(&sd_removal_seen, 1, 0);
	bool inserted = sdcard_manager.sd_inserted();

	sd_msg msg = { .removed = true };

	if (removed_seen) {
		ret = sdcard_manager.unmount();
		if (ret == 0) {
			LOG_INF("SD card unmounted due to card removal/change.");
		} else {
			LOG_WRN("Failed to unmount SD card after removal/change: %d", ret);
		}

		ret = zbus_chan_pub(&sd_card_chan, &msg, K_FOREVER);
		if (ret != 0) {
			LOG_ERR("Failed to publish sd_card_chan: %d", ret);
		}
	}

	if (!inserted) {
		return;
	}

	if (removed_seen || !sdcard_manager.is_mounted()) {
		ret = sdcard_manager.mount();
		if (ret == 0) {
			LOG_INF("SD card mounted after insertion.");
		} else {
			LOG_WRN("Failed to mount SD card after insertion: %d", ret);
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SDCardManager::unmount_work, SDCardManager::unmount_work_handler);

void SDCardManager::sd_card_state_change_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	if (!sdcard_manager.sd_inserted()) {
		atomic_set(&sd_removal_seen, 1);
	}

	k_work_reschedule_for_queue(&sd_hotplug_work_q, &sdcard_manager.unmount_work, SD_DEBOUNCE_MS);
}

SDCardManager::SDCardManager(): path(SD_ROOT_PATH) {
	fs_dir_t_init(&this->dirp);
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

void SDCardManager::reset_state_locked()
{
	this->path = SD_ROOT_PATH;
	fs_dir_t_init(&this->dirp);
	fs_file_t_init(&this->tracked_file.filep);
	this->tracked_file.is_open = false;
	this->mounted = false;
}

int SDCardManager::aquire_ls() {
	int ret;

	if (ls_aquired) return -EALREADY;

	ret = pm_device_runtime_get(ls_sd);
	if (ret) {
		LOG_ERR("Failed to get ls_sd");
		return ret;
	}

	ls_aquired = true;

	return 0;
}

int SDCardManager::release_ls() {
	int ret;

	if (!ls_aquired) return -EALREADY;

	ret = pm_device_runtime_put(ls_sd);

	ls_aquired = false;

	return 0;
}

void SDCardManager::init() {
	int ret;

	k_work_queue_init(&sd_hotplug_work_q);
	k_work_queue_start(&sd_hotplug_work_q, sd_hotplug_work_q_stack,
		K_THREAD_STACK_SIZEOF(sd_hotplug_work_q_stack),
		K_PRIO_PREEMPT(SD_HOTPLUG_WORK_Q_PRIO), &sd_hotplug_work_q_config);

	if (!device_is_ready(sd_state_pin.port)) {
		ret = aquire_ls();
		LOG_ERR("SD state GPIO device not ready\n");
		return;
	}

	gpio_pin_configure_dt(&sd_state_pin, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&sd_state_pin, GPIO_INT_EDGE_BOTH);

	gpio_init_callback(&sd_state_cb, sd_card_state_change_isr, BIT(sd_state_pin.pin));
	ret = gpio_add_callback(sd_state_pin.port, &sd_state_cb);

	if (ret) LOG_ERR("Failed to add callback");
}

int SDCardManager::unmount() {
	int ret;
	int close_ret;
	int deinit_ret;
	int unlock_ret;

	if (!this->mounted && !ls_aquired) {
		return 0;
	}

	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (this->tracked_file.is_open) {
		close_ret = fs_close(&this->tracked_file.filep);
		if (close_ret) {
			LOG_WRN("Failed to close tracked file during unmount: %d", close_ret);
		}
		fs_file_t_init(&this->tracked_file.filep);
		this->tracked_file.is_open = false;
	}

	close_ret = fs_closedir(&this->dirp);
	if (close_ret) {
		LOG_DBG("Failed to close tracked dir during unmount: %d", close_ret);
	}

	this->path = SD_ROOT_PATH;
	fs_dir_t_init(&this->dirp);

	if (this->mounted) {
		ret = fs_unmount(&this->mnt_pt);
		if (ret) {
			LOG_WRN("Failed to unmount SD card cleanly: %d", ret);
			ret = 0;
		}
	}

	deinit_ret = force_sd_deinit();
	if (deinit_ret && deinit_ret != -EINVAL) {
		LOG_WRN("Failed to deinit SD backend during unmount: %d", deinit_ret);
	}

	reset_state_locked();

	unlock_ret = k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	if (unlock_ret) {
		LOG_ERR("Sem give failed. Ret: %d", unlock_ret);
		return unlock_ret;
	}

	close_ret = release_ls();
	if (close_ret && close_ret != -EALREADY) {
		LOG_ERR("Failed to release SD load switches: %d", close_ret);
		return close_ret;
	}

	return ret;
}

int SDCardManager::mount() {
	int ret;
	int unlock_ret;
	int release_ret;
	bool rails_were_off;

	uint64_t sd_card_size_bytes;
	uint32_t sector_count;
	size_t sector_size;

	if (this->mounted) {
		return 0;
	}

	rails_were_off = !ls_aquired;

	ret = aquire_ls();
	if (ret && ret != -EALREADY) {
		LOG_ERR("Failed to enable SD load switches: %d", ret);
		return ret;
	}

	bool _sd_inserted = sd_inserted();

	if (!_sd_inserted) {
		force_sd_deinit();
		release_ls();
		LOG_ERR("No SD card inserted.");
		return -ENODEV;
	}

	ret = k_mutex_lock(&m_sem_sd_mngr_oper_ongoing, K_FOREVER);
	if (ret) {
		release_ls();
		LOG_ERR("Sem take failed. Ret: %d", ret);
		return ret;
	}

	if (this->mounted) {
		unlock_ret = k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		if (unlock_ret) {
			LOG_ERR("Sem give failed. Ret: %d", unlock_ret);
			return unlock_ret;
		}
		return 0;
	}

	if (rails_were_off) {
		k_msleep(SD_POWER_SETTLE_MS);
	}

	for (int attempt = 1; attempt <= SD_MOUNT_MAX_ATTEMPTS; ++attempt) {
		ret = disk_access_init(SD_DEV);
		if (ret) {
			LOG_WRN("SD card init failed on attempt %d/%d: %d",
				attempt, SD_MOUNT_MAX_ATTEMPTS, ret);
			ret = -ENODEV;
			goto retry_or_fail;
		}

		ret = disk_access_ioctl(SD_DEV, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
		if (ret) {
			LOG_ERR("Unable to get sector count on attempt %d/%d: %d",
				attempt, SD_MOUNT_MAX_ATTEMPTS, ret);
			goto retry_or_fail;
		}

		LOG_DBG("Sector count: %d", sector_count);

		ret = disk_access_ioctl(SD_DEV, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
		if (ret) {
			LOG_ERR("Unable to get sector size on attempt %d/%d: %d",
				attempt, SD_MOUNT_MAX_ATTEMPTS, ret);
			goto retry_or_fail;
		}

		LOG_DBG("Sector size: %d bytes", sector_size);

		sd_card_size_bytes = (uint64_t)sector_count * sector_size;

		LOG_INF("SD card volume size: %d MB", (uint32_t)(sd_card_size_bytes >> 20));

		this->path = SD_ROOT_PATH;
		fs_dir_t_init(&this->dirp);
		this->mnt_pt.mnt_point = SD_ROOT_PATH;
		ret = fs_mount(&this->mnt_pt);
		if (ret && ret != -EBUSY) {
			LOG_ERR("Mnt. disk failed on attempt %d/%d, could be format issue. should be FAT/exFAT. Error: %d",
				attempt, SD_MOUNT_MAX_ATTEMPTS, ret);
			goto retry_or_fail;
		}

		LOG_DBG("Root dir: %s", SD_ROOT_PATH);
		ret = fs_opendir(&this->dirp, SD_ROOT_PATH);
		if (ret) {
			LOG_ERR("Open root dir failed on attempt %d/%d. Error: %d",
				attempt, SD_MOUNT_MAX_ATTEMPTS, ret);
			if (ret == -EBUSY) {
				LOG_WRN("Root dir busy after mount.");
			}
			fs_unmount(&this->mnt_pt);
			goto retry_or_fail;
		}

		this->mounted = true;
		break;

retry_or_fail:
		force_sd_deinit();
		reset_state_locked();

		if (attempt == SD_MOUNT_MAX_ATTEMPTS) {
			k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
			release_ls();
			return ret;
		}

		release_ret = release_ls();
		if (release_ret && release_ret != -EALREADY) {
			k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
			LOG_ERR("Failed to power-cycle SD load switches: %d", release_ret);
			return release_ret;
		}

		k_msleep(SD_POWER_CYCLE_OFF_MS);

		ret = aquire_ls();
		if (ret && ret != -EALREADY) {
			k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
			LOG_ERR("Failed to re-enable SD load switches: %d", ret);
			return ret;
		}

		k_msleep(SD_POWER_SETTLE_MS);

		if (!sd_inserted()) {
			k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
			release_ls();
			LOG_ERR("No SD card inserted during mount retry.");
			return -ENODEV;
		}
	}

	unlock_ret = k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
	if (unlock_ret) {
		LOG_ERR("Sem give failed. Ret: %d", unlock_ret);
		return unlock_ret;
	}

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

	if (!this->tracked_file.is_open) {
		LOG_INF("File is not open");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return 0;
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

	if (!this->tracked_file.is_open) {
		LOG_ERR("File is not open");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -EINVAL;
	}

	if (!(this->tracked_file.filep.flags & FS_O_WRITE)) {
		LOG_ERR("File is not open for writing");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -EINVAL;
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

	if (!this->tracked_file.is_open) {
		LOG_ERR("No file opened");
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -EINVAL;
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

	if (!this->tracked_file.is_open) {
		k_mutex_unlock(&m_sem_sd_mngr_oper_ongoing);
		return -EINVAL;
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
