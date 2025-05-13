target_sources(app PRIVATE
           ${CMAKE_CURRENT_SOURCE_DIR}/device_info.c
           ${CMAKE_CURRENT_SOURCE_DIR}/battery_service.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/button_service.c
           ${CMAKE_CURRENT_SOURCE_DIR}/sensor_service.c
           ${CMAKE_CURRENT_SOURCE_DIR}/audio_config_service.c
           ${CMAKE_CURRENT_SOURCE_DIR}/led_service.cpp
)
