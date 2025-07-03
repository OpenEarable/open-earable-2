#ifndef FS_MANAGER_H
#define FS_MANAGER_H

#include <string>

class FSManager {
public:
    virtual int mount() = 0;
    virtual int unmount() = 0;
    virtual int cd(std::string path) = 0;
    virtual int ls(char *buf, size_t *buf_size) = 0;
    virtual int mkdir(std::string path) = 0;
    virtual int open_file(std::string path, bool write, bool append, bool create) = 0;
    virtual int close_file() = 0;
    virtual ssize_t write(char *buf, size_t *buf_size, bool sync = false) = 0;
    virtual ssize_t write(std::string path, char *buf, size_t *buf_size, bool append = false) = 0;
    virtual int sync() = 0;
    virtual int read(char *buf, size_t *buf_size) = 0;
    virtual int rm(std::string path) = 0;

    virtual bool is_mounted() const = 0;

    // Optional convenience methods
    virtual int open_write_close(std::string path, char *buf, size_t *buf_size,
                                 bool append = false, bool create = false) = 0;
    virtual int open_read_close(std::string path, char *buf, size_t *buf_size) = 0;
};

struct tracked_fs_file_t {
    struct fs_file_t filep;
    bool is_open;
};

#endif // FS_MANAGER_H