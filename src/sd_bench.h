#ifndef SD_BENCHMARK_H
#define SD_BENCHMARK_H

#include <zephyr/kernel.h>
#include <string>
#include "SD_Card_Manager.h"

#define BENCHMARK_BLOCK_SIZE 4096  // 4KB
#define BENCHMARK_TOTAL_SIZE (1024 * 1024)  // 1MB
#define BENCHMARK_ITERATIONS 10
#define BENCHMARK_THREAD_STACK_SIZE 8192  // 4KB
#define BENCHMARK_THREAD_PRIORITY 7

static uint8_t write_buffer[BENCHMARK_BLOCK_SIZE];
class SDBenchmark {
public:
    struct BenchmarkResult {
        uint32_t bytes_written;
        uint32_t duration_us;
        uint32_t speed_kbps;
        uint32_t speed_mbps;
    };

    SDBenchmark();
    ~SDBenchmark();

    int start();
    bool is_running();
    void wait_complete();

    struct k_thread thread_data;  // Make public like SDLogger

    int init() {
        if (!sd_card) {
            sd_card = &sdcard_manager;  // ✅ Erst hier zuweisen, nach main()
        }
        return 0;
    }
    
private:
    SDCardManager* sd_card = nullptr;
    
    
    k_tid_t thread_id;
    
    volatile bool running;
    volatile int result_code;
    
    BenchmarkResult results[BENCHMARK_ITERATIONS];
    
    static void benchmark_thread(void* p1, void* p2, void* p3);
    
    int run_benchmark();
    int init_benchmark();
    int run_write_test(BenchmarkResult* result);
    void cleanup();
    void print_results(const BenchmarkResult* results, int count);
    void fill_test_pattern();
};
extern SDBenchmark sd_benchmark;

#endif // SD_BENCHMARK_H
