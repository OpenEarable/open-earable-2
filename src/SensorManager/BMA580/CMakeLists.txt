#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/bma5.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/bma580.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/bma580_features.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/BMA580_Sensor.cpp
)
