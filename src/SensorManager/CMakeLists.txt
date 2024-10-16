
add_subdirectory(BMP388)
#add_subdirectory(Adafruit_BusIO)
#add_subdirectory(Adafruit_Unified_Sensor)
add_subdirectory(BMX160)
add_subdirectory(MAX30102)
add_subdirectory(MLX90632)
add_subdirectory(MAXM86161)

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/SensorManager.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/SensorManager.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/EdgeMLSensor.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Baro.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/IMU.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PPG.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Temp.cpp
)
