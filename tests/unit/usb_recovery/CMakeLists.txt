# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_recovery_tests)

test_runner_generate(src/main.c)
target_include_directories(app PRIVATE ../../../src/Battery)
target_sources(app PRIVATE src/main.c ../../../src/Battery/usb_recovery.c)
