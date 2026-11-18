#pragma once
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...)
#define LOG_WRN(...)
#define LOG_ERR(...)
template<typename... Args> inline void host_log_debug(const char *, const Args &...) {}
#define LOG_DBG(...) host_log_debug(__VA_ARGS__)
