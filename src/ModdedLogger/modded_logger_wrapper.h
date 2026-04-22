#ifndef MODDED_LOGGER_WRAPPER_H
#define MODDED_LOGGER_WRAPPER_H

#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* modded_logger_t;

modded_logger_t modded_logger_create(void);
void modded_logger_destroy(modded_logger_t logger);

int modded_logger_mount(modded_logger_t logger);
int modded_logger_unmount(modded_logger_t logger);
bool modded_logger_is_mounted(modded_logger_t logger);

int modded_logger_cd(modded_logger_t logger, const char *path);
int modded_logger_ls(modded_logger_t logger, char *buf, size_t *buf_size);
int modded_logger_mkdir(modded_logger_t logger, const char *path);
int modded_logger_rm(modded_logger_t logger, const char *path);

int modded_logger_open_file(modded_logger_t logger, const char *path, bool write, bool append, bool create);
int modded_logger_close_file(modded_logger_t logger);
int modded_logger_sync(modded_logger_t logger);
int modded_logger_read(modded_logger_t logger, char *buf, size_t *buf_size);

ssize_t modded_logger_write(modded_logger_t logger, const char *buf, size_t *buf_size, bool sync);
ssize_t modded_logger_write_path(modded_logger_t logger, const char *path, const char *buf, size_t *buf_size, bool append);

int modded_logger_open_write_close(modded_logger_t logger, const char *path, char *buf, size_t *buf_size, bool append, bool create);
int modded_logger_open_read_close(modded_logger_t logger, const char *path, char *buf, size_t *buf_size);

/* --- Flash Logging Methods --- */
int modded_logger_log_err(modded_logger_t logger, const char* input);
int modded_logger_log_dbg(modded_logger_t logger, const char* input);
int modded_logger_log_wrn(modded_logger_t logger, const char* input);
int modded_logger_log_inf(modded_logger_t logger, const char* input);

#ifdef __cplusplus
}
#endif

#endif /* MODDED_LOGGER_WRAPPER_H */