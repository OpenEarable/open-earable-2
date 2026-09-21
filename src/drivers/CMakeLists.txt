#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources_ifdef(CONFIG_NRF5340_AUDIO_CS47L63_DRIVER
		     app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ADAU1860.cpp
)

add_subdirectory(LED_Controller)

zephyr_library_include_directories(
    LED_Controller
)