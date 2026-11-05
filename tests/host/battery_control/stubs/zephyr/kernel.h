#pragma once
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <chrono>
#define ARG_UNUSED(x) (void)(x)
#define MIN(a,b) std::min((a),(b))
#define MAX(a,b) std::max((a),(b))
#define CLAMP(x,a,b) std::clamp((x),(a),(b))
#define BIT(n) (1U << (n))
#define K_FOREVER -1
extern std::atomic<uint64_t> clock_us;
inline void k_usleep(int n) { clock_us += n; }
inline void k_msleep(int n) { clock_us += n * 1000; }
struct k_mutex { std::recursive_mutex mutex; };
inline void k_mutex_init(k_mutex *) {}
inline int k_mutex_lock(k_mutex *m, int) { m->mutex.lock(); return 0; }
inline int k_mutex_unlock(k_mutex *m) { m->mutex.unlock(); return 0; }
