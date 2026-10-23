#pragma once
using atomic_t = int;
inline void atomic_clear(atomic_t *v) { *v = 0; }
inline bool atomic_cas(atomic_t *v, int old, int next) {
 if (*v != old) return false;
 *v = next; return true;
}
