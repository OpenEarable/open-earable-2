#pragma once
#include <cstdint>
struct k_work {};
struct k_work_delayable { void (*handler)(k_work *); };
extern int scheduled;
extern int delay_ms;
#define K_MSEC(n) (n)
#define K_WORK_DELAYABLE_DEFINE(name, fn) k_work_delayable name = {fn}
inline int k_work_reschedule(k_work_delayable *, int delay) {
 scheduled++; delay_ms = delay; return 0;
}
