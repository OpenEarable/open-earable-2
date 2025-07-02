#ifndef LITTLE_FS_MANAGER_H
#define LITTLE_FS_MANAGER_H

#include <string>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/fs/littlefs.h>

#include "FsManager.h"

class LittleFsManager : public FSManager {
public:
    LittleFsManager();
    ~LittleFsManager();

    int mount() override;
    int unmount() override;
    int cd(std::string path) override;
    int ls(char *buf, size_t *buf_size) override;
    int mkdir(std::string path) override;
    int open_file(std::string path, bool write, bool append, bool create) override;
    int close_file() override;
    ssize_t write(char *buf, size_t *buf_size, bool sync = false) override;
    int read(char *buf, size_t *buf_size) override;
    int rm(std::string path) override;

    bool is_mounted() const override;

    int open_write_close(std::string path, char *buf, size_t *buf_size,
                         bool append = false, bool create = false) override;
    int open_read_close(std::string path, char *buf, size_t *buf_size) override;

private:
    std::string path;
    bool mounted = false;
    struct fs_dir_t dirp;

    fs_mount_t mnt;
    struct tracked_fs_file_t tracked_file = {
        .is_open = false,
    };

    static k_work_delayable unmount_work;

    static void unmount_work_handler(struct k_work *work);

    int aquire_ls();
    int release_ls();

    bool ls_aquired = false;
};

#endif // LITTLE_FS_MANAGER_H