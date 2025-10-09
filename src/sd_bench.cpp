#include "sd_bench.h"
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(sd_benchmark, CONFIG_LOG_DEFAULT_LEVEL);

K_THREAD_STACK_DEFINE(benchmark_thread_stack, BENCHMARK_THREAD_STACK_SIZE);

K_SEM_DEFINE(benchmark_completion_sem, 0, 1);

SDBenchmark::SDBenchmark() : running(false), result_code(0) {
}

SDBenchmark::~SDBenchmark() {
    if (running) {
        wait_complete();
    }
    cleanup();
}

void SDBenchmark::fill_test_pattern() {
    for (size_t i = 0; i < BENCHMARK_BLOCK_SIZE; i++) {
        write_buffer[i] = (uint8_t)(i & 0xFF);
    }
}

int SDBenchmark::init_benchmark() {
    int ret;
    
    LOG_INF("Initializing SD card for benchmark...");
    
    sd_card->init();
    
    if (!sd_card->is_mounted()) {
        ret = sd_card->mount();
        if (ret < 0) {
            LOG_ERR("Failed to mount SD card: %d", ret);
            return ret;
        }
    }
    
    std::string filename = "benchmark_test.dat";
    ret = sd_card->open_file(filename, true, false, true);
    if (ret < 0) {
        LOG_ERR("Failed to open benchmark file: %d", ret);
        return ret;
    }
    
    LOG_INF("SD Card benchmark initialized successfully");
    return 0;
}

int SDBenchmark::run_write_test(BenchmarkResult* result) {
    int ret;
    uint32_t total_bytes = 0;
    uint32_t start_time, end_time;
    
    fill_test_pattern();
    
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    
    uint32_t blocks_to_write = BENCHMARK_TOTAL_SIZE / BENCHMARK_BLOCK_SIZE;
    
    for (uint32_t i = 0; i < blocks_to_write; i++) {
        size_t write_size = BENCHMARK_BLOCK_SIZE;
        ret = sd_card->write((char*)write_buffer, &write_size, false);
        
        if (ret < 0) {
            LOG_ERR("Write failed at block %u: %d", i, ret);
            return ret;
        }
        
        total_bytes += write_size;
        
        if (write_size != BENCHMARK_BLOCK_SIZE) {
            LOG_WRN("Partial write: %zu bytes instead of %u", 
                    write_size, BENCHMARK_BLOCK_SIZE);
        }
        
        if (i % 64 == 0) {
            k_yield();
        }
    }
    
    LOG_DBG("Syncing data to SD card...");
    ret = sd_card->sync();
    if (ret < 0) {
        LOG_ERR("Sync failed: %d", ret);
        return ret;
    }
    
    end_time = k_cyc_to_us_floor32(k_cycle_get_32());
    
    result->bytes_written = total_bytes;
    result->duration_us = (uint32_t)(end_time - start_time);
    
    if (result->duration_us > 0) {
        result->speed_kbps = (total_bytes * 1000ULL) / result->duration_us;
        result->speed_mbps = result->speed_kbps / 1024;
    } else {
        result->speed_kbps = 0;
        result->speed_mbps = 0;
    }
    
    return 0;
}

void SDBenchmark::print_results(const BenchmarkResult* results, int count) {
    uint64_t total_bytes = 0;
    uint64_t total_duration = 0;
    uint32_t min_speed = UINT32_MAX;
    uint32_t max_speed = 0;
    
    LOG_INF("========================================");
    LOG_INF("SD Card Write Benchmark Results");
    LOG_INF("========================================");
    LOG_INF("Block Size: %u bytes", BENCHMARK_BLOCK_SIZE);
    LOG_INF("Total Size per Test: %u bytes (%u KB)", 
            BENCHMARK_TOTAL_SIZE, BENCHMARK_TOTAL_SIZE / 1024);
    LOG_INF("Number of Iterations: %d", count);
    LOG_INF("----------------------------------------");
    
    for (int i = 0; i < count; i++) {
        LOG_INF("Test %d: %u bytes in %u us = %u KB/s (%u.%02u MB/s)",
                i + 1,
                results[i].bytes_written,
                results[i].duration_us,
                results[i].speed_kbps,
                results[i].speed_mbps,
                (results[i].speed_kbps % 1024) * 100 / 1024);
        
        total_bytes += results[i].bytes_written;
        total_duration += results[i].duration_us;
        
        if (results[i].speed_kbps < min_speed) {
            min_speed = results[i].speed_kbps;
        }
        if (results[i].speed_kbps > max_speed) {
            max_speed = results[i].speed_kbps;
        }
    }
    
    uint32_t avg_speed_kbps = (total_bytes * 1000ULL) / total_duration;
    uint32_t avg_speed_mbps = avg_speed_kbps / 1024;
    
    LOG_INF("========================================");
    LOG_INF("Summary:");
    LOG_INF("  Total Bytes Written: %llu bytes (%llu KB)", 
            total_bytes, total_bytes / 1024);
    LOG_INF("  Total Duration: %llu us (%llu ms)", 
            total_duration, total_duration / 1000);
    LOG_INF("  Average Speed: %u KB/s (%u.%02u MB/s)",
            avg_speed_kbps,
            avg_speed_mbps,
            (avg_speed_kbps % 1024) * 100 / 1024);
    LOG_INF("  Min Speed: %u KB/s (%u.%02u MB/s)",
            min_speed,
            min_speed / 1024,
            (min_speed % 1024) * 100 / 1024);
    LOG_INF("  Max Speed: %u KB/s (%u.%02u MB/s)",
            max_speed,
            max_speed / 1024,
            (max_speed % 1024) * 100 / 1024);
    LOG_INF("========================================");
}

void SDBenchmark::cleanup() {
    if (sd_card && sd_card->is_mounted()) {
        sd_card->close_file();
    }
}

int SDBenchmark::run_benchmark() {
    int ret;
    
    LOG_INF("Starting SD Card Write Benchmark...");
    
    ret = init_benchmark();
    if (ret < 0) {
        LOG_ERR("Benchmark initialization failed: %d", ret);
        return ret;
    }
    
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        LOG_INF("Running test iteration %d/%d...", i + 1, BENCHMARK_ITERATIONS);
        
        ret = run_write_test(&results[i]);
        if (ret < 0) {
            LOG_ERR("Test iteration %d failed: %d", i + 1, ret);
            cleanup();
            return ret;
        }
        
        k_sleep(K_MSEC(200));
    }
    
    print_results(results, BENCHMARK_ITERATIONS);
    
    cleanup();
    
    LOG_INF("Benchmark completed successfully");
    return 0;
}

void SDBenchmark::benchmark_thread(void* p1, void* p2, void* p3) {
    SDBenchmark* benchmark = static_cast<SDBenchmark*>(p1);
    
    benchmark->result_code = benchmark->run_benchmark();
    benchmark->running = false;
    
    k_sem_give(&benchmark_completion_sem);
}

int SDBenchmark::start() {
    if (running) {
        LOG_ERR("Benchmark already running");
        return -EBUSY;
    }
    
    running = true;
    result_code = 0;
    
    thread_id = k_thread_create(
        &thread_data,
        benchmark_thread_stack,
        K_THREAD_STACK_SIZEOF(benchmark_thread_stack),
        benchmark_thread,
        this,
        NULL,
        NULL,
        K_PRIO_PREEMPT(CONFIG_SENSOR_SD_THREAD_PRIO),
        0,
        K_NO_WAIT
    );
    
    int ret = k_thread_name_set(thread_id, "SD_BENCH");
    if (ret) {
        LOG_WRN("Failed to set thread name: %d", ret);
    }
    
    LOG_INF("Benchmark thread started");
    return 0;
}

bool SDBenchmark::is_running() {
    return running;
}

void SDBenchmark::wait_complete() {
    if (running) {
        LOG_INF("Waiting for benchmark to complete...");
        k_sem_take(&benchmark_completion_sem, K_FOREVER);
        LOG_INF("Benchmark thread finished with result: %d", result_code);
    }
}


SDBenchmark sd_benchmark;