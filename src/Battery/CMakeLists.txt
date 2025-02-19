
target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/BQ27220.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/BQ25120a.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PowerManager.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PowerManager.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/BootState.c
)