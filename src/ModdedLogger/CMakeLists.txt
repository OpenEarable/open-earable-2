enable_language(CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/ModdedLogger.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/modded_logger_wrapper.cpp
)