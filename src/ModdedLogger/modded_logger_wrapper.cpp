#include "modded_logger_wrapper.h"
#include "ModdedLogger.h"

extern "C" {

modded_logger_t modded_logger_create(void) {
    return new ModdedLogger();
}

void modded_logger_destroy(modded_logger_t logger) {
    if (logger != nullptr) {
        delete static_cast<ModdedLogger*>(logger);
    }
}

int modded_logger_mount(modded_logger_t logger) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->mount();
}

int modded_logger_unmount(modded_logger_t logger) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->unmount();
}

bool modded_logger_is_mounted(modded_logger_t logger) {
    if (!logger) return false;
    return static_cast<ModdedLogger*>(logger)->is_mounted();
}

int modded_logger_cd(modded_logger_t logger, const char *path) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->cd(std::string(path));
}

int modded_logger_ls(modded_logger_t logger, char *buf, size_t *buf_size) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->ls(buf, buf_size);
}

int modded_logger_mkdir(modded_logger_t logger, const char *path) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->mkdir(std::string(path));
}

int modded_logger_rm(modded_logger_t logger, const char *path) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->rm(std::string(path));
}

int modded_logger_open_file(modded_logger_t logger, const char *path, bool write, bool append, bool create) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->open_file(std::string(path), write, append, create);
}

int modded_logger_close_file(modded_logger_t logger) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->close_file();
}

int modded_logger_sync(modded_logger_t logger) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->sync();
}

int modded_logger_read(modded_logger_t logger, char *buf, size_t *buf_size) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->read(buf, buf_size);
}

ssize_t modded_logger_write(modded_logger_t logger, const char *buf, size_t *buf_size, bool sync) {
    if (!logger) return -1;
    return static_cast<ModdedLogger*>(logger)->write(buf, buf_size, sync);
}

ssize_t modded_logger_write_path(modded_logger_t logger, const char *path, const char *buf, size_t *buf_size, bool append) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->write(std::string(path), buf, buf_size, append);
}

int modded_logger_open_write_close(modded_logger_t logger, const char *path, char *buf, size_t *buf_size, bool append, bool create) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->open_write_close(std::string(path), buf, buf_size, append, create);
}

int modded_logger_open_read_close(modded_logger_t logger, const char *path, char *buf, size_t *buf_size) {
    if (!logger || !path) return -1;
    return static_cast<ModdedLogger*>(logger)->open_read_close(std::string(path), buf, buf_size);
}


int modded_logger_log_err(modded_logger_t logger, const char* input) {
    if (!logger || !input) return -1;
    return static_cast<ModdedLogger*>(logger)->log_flash_err(input);
}

int modded_logger_log_dbg(modded_logger_t logger, const char* input) {
    if (!logger || !input) return -1;
    return static_cast<ModdedLogger*>(logger)->log_flash_dbg(input);
}

int modded_logger_log_wrn(modded_logger_t logger, const char* input) {
    if (!logger || !input) return -1;
    return static_cast<ModdedLogger*>(logger)->log_flash_wrn(input);
}

int modded_logger_log_inf(modded_logger_t logger, const char* input) {
    if (!logger || !input) return -1;
    return static_cast<ModdedLogger*>(logger)->log_flash_inf(input);
}

} // extern "C"