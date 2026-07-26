#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
		   ${CMAKE_CURRENT_SOURCE_DIR}/BMX160.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/bosch/bmi160.c
		   ${CMAKE_CURRENT_SOURCE_DIR}/bosch/bmm150.c
)

target_compile_definitions(app PRIVATE BMM150_USE_FLOATING_POINT)
