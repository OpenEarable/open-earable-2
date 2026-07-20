# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_component_tests)

test_runner_generate(src/main.cpp)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/ParseInfo
)

target_sources(app PRIVATE
    src/main.cpp
    ../../../src/ParseInfo/SensorComponent.cpp
)
