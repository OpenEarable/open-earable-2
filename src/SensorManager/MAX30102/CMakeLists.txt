#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
		   ${CMAKE_CURRENT_SOURCE_DIR}/MAX30102.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/SOSFilter.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/PulseOximeter.cpp
)
