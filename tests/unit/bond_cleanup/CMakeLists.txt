# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bond_cleanup_tests)

test_runner_generate(src/main.c)
target_include_directories(app PRIVATE
  ../../../src/bluetooth/bt_management
  ${ZEPHYR_BASE}/subsys/bluetooth
  ${ZEPHYR_BASE}/subsys/bluetooth/host)
target_sources(app PRIVATE src/main.c
  ../../../src/bluetooth/bt_management/bt_mgmt_bond_cleanup.c)
