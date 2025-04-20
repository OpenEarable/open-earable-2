
add_subdirectory(BMP388)
add_subdirectory(BMX160)
add_subdirectory(MLX90632)
add_subdirectory(MAXM86161)
add_subdirectory(BMA580)

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/SensorManager.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/EdgeMLSensor.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Baro.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/IMU.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PPG.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Temp.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/BoneConduction.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Microphone.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Microphone.c
)
