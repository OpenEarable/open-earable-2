#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

# Flag which defines whether application is compiled as gateway/dongle or headset
add_compile_definitions(HEADSET=1)
add_compile_definitions(GATEWAY=2)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(openearable_v2_1)

#set(OVERLAY_CONFIG 
#    ${CMAKE_CURRENT_SOURCE_DIR}/unicast_server/overlay-unicast_server.conf
#)

string(TIMESTAMP NRF5340_AUDIO_CORE_APP_COMP_DATE "%a %b %d %H:%M:%S %Y")

# Generate fw_info_app.c
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/utils/fw_info_app.c.in"
               "${CMAKE_BINARY_DIR}/fw_info_app.c"
               @ONLY)

# Target sources below are specific to the nRF5340 Audio DK HW
target_sources(app PRIVATE
#    src/openearable_common.c
    ${CMAKE_BINARY_DIR}/fw_info_app.c
    )

if (CONFIG_BT_BAP_BROADCAST_SINK)
	add_subdirectory(broadcast_sink)
endif()

if (CONFIG_BT_BAP_UNICAST_SERVER)
	add_subdirectory(unicast_server)
endif()


# Include application events and configuration headers
zephyr_library_include_directories(
    include
    src/audio
    src/Battery
    src/Wire
    src/bluetooth
    src/drivers
    src/modules
    src/utils
    src/utils/macros
    src/ParseInfo
    src/SD_Card/SDLogger
    src/SD_Card/Benchmark
    src/SD_Card/SD_Card_Manager
    src/time_sync
)

zephyr_library_include_directories(app PRIVATE
    ${ZEPHYR_NRF_MODULE_DIR}/boards/teco/openearable_v2_nrf5340)

# Application sources
add_subdirectory(src/audio)
add_subdirectory(src/Wire)
add_subdirectory(src/Battery)
add_subdirectory(src/bluetooth)
add_subdirectory(src/buttons)
add_subdirectory(src/drivers)
add_subdirectory(src/modules)
add_subdirectory(src/utils)
add_subdirectory(src/ParseInfo)
add_subdirectory(src/SensorManager)
add_subdirectory(src/SD_Card/SDLogger)
add_subdirectory(src/SD_Card/Benchmark)
add_subdirectory(src/SD_Card/SD_Card_Manager)
add_subdirectory(src/time_sync)

## Cirrus Logic
if (CONFIG_HW_CODEC_CIRRUS_LOGIC)
    if (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR)
        add_subdirectory(${ZEPHYR_CIRRUS_LOGIC_MODULE_DIR} cirrus_logic_bin_dir)
    else()
        message(FATAL_ERROR "Cirrus Logic/sdk-mcu-drivers repository not found\n")
    endif()
endif()

include(version.cmake)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
