if ((CONFIG_BOARD_OPENEARABLE_V2_NRF5340_CPUAPP OR CONFIG_BOARD_OPENEARABLE_V2_NRF5340_CPUAPP_NS) AND NOT CONFIG_MCUBOOT)
    zephyr_library()
    zephyr_library_sources(board_init.c)
    zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../include) # For openearable_common.h
endif()

if((CONFIG_BOARD_OPENEARABLE_V2_NRF5340_CPUAPP OR CONFIG_BOARD_OPENEARABLE_V2_NRF5340_CPUAPP_NS) AND CONFIG_MCUBOOT)
    zephyr_library()
    zephyr_library_sources(mcuboot_hook.c)
endif()