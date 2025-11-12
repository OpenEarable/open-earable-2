
add_subdirectory(BMP388)
add_subdirectory(BMX160)
add_subdirectory(MLX90632)
add_subdirectory(MAXM86161)
add_subdirectory(BMA580)
add_subdirectory(AD7124)

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/SensorManager.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/EdgeMLSensor.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Baro.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/IMU.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PPG_right_I2C2.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PPG_left_I2C3.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Temp_right_I2C2.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Temp_left_I2C3.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/BoneConduction.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Microphone.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/Microphone.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/ExG.cpp
)
