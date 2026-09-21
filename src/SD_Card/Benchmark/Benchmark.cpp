#include "Benchmark.h"

Benchmark::Benchmark(void (*function)()) {
    this->function = function;
}

void Benchmark::start() {
    this->start_time = k_uptime_get_32();
    this->start_cycles = k_cycle_get_32();
    this->function();
    this->end_time = k_uptime_get_32();
    this->end_cycles = k_cycle_get_32();
}

uint32_t Benchmark::get_time() {
    return this->end_time - this->start_time;
}

uint32_t Benchmark::get_cycles() {
    return this->end_cycles - this->start_cycles;
}