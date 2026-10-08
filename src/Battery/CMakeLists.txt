
target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/BQ27220.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/BQ25120a.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/AutoOffManager.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PowerManager.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PowerManager.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/BootState.c
)

target_sources_ifdef(CONFIG_USB_BATTERY_RECOVERY app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/usb_recovery.c
	${CMAKE_CURRENT_SOURCE_DIR}/usb_recovery_io.c
)
