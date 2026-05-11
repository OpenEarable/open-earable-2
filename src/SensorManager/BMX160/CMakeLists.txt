#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
		   ${CMAKE_CURRENT_SOURCE_DIR}/bmi160.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/BMX160_Sensor.cpp
)
