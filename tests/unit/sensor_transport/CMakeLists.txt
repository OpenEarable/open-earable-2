cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_transport_tests)
test_runner_generate(src/main.c)
target_include_directories(app PRIVATE
    ../../../src/bluetooth/gatt_services
    ../../../protocol/generated/c/include)
target_sources(app PRIVATE src/main.c
    ../../../src/bluetooth/gatt_services/sensor_transport.c
    ../../../protocol/generated/c/src/ppg_protocol.c
    ../../../protocol/generated/c/src/imu_protocol.c
    ../../../protocol/generated/c/src/protocol_runtime.c)
