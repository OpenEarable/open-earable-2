#ifndef MODDED_LOGGER_H
#define MODDED_LOGGER_H

#include <string>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

struct tracked_fs_file_t {
    struct fs_file_t filep;
    bool is_open;
};

class ModdedLogger {
public:
    ModdedLogger();
    ~ModdedLogger();

    int mount();
    int unmount();
    
    int cd(const std::string &path);
    int ls(char *buf, size_t *buf_size);
    int mkdir(const std::string &path);
    int open_file(const std::string &path, bool write, bool append, bool create);
    int close_file();
    
    ssize_t write(const char *buf, size_t *buf_size, bool sync = false);
    ssize_t write(const std::string &path, const char *buf, size_t *buf_size, bool append = false);
    
    int sync();
    int read(char *buf, size_t *buf_size);
    int rm(const std::string &path);
    bool is_mounted() const;
    
    int open_write_close(const std::string &path, char *buf, size_t *buf_size,
                         bool append = false, bool create = false);
    int open_read_close(const std::string &path, char *buf, size_t *buf_size);

    /* --- Flash Logging Methods --- */
    int log_flash_err(const char* input);
    int log_flash_dbg(const char* input);
    int log_flash_wrn(const char* input);
    int log_flash_inf(const char* input);


private:
    std::string path;
    bool mounted = false;
    struct fs_dir_t dirp;

    fs_mount_t mnt;
    struct tracked_fs_file_t tracked_file = {
        .is_open = false,
    };
};

#endif // MODDED_LOGGER_H