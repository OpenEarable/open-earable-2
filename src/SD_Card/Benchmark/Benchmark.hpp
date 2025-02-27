#ifndef _BENCHMARK_H_
#define _BENCHMARK_H_

#include <zephyr/kernel.h>

class Benchmark {
public:
    /**
     * @brief Construct a new Benchmark object
     * @param function The function that should be benchmarked
     */
    Benchmark(void (*function)());

    void start();

    uint32_t get_time();
    uint32_t get_cycles();

private:
    uint32_t start_time;
    uint32_t start_cycles;
    uint32_t end_time;
    uint32_t end_cycles;

    // the pointer to the function that should be benchmarked
    void (*function)();
};

#endif