#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# ${CMAKE_CURRENT_SOURCE_DIR}/streamctrl_unicast_server.c

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
		   ${CMAKE_CURRENT_SOURCE_DIR}/streamctrl_unicast_server.c)
