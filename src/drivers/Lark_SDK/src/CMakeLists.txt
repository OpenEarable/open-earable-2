#
# Copyright (c) 2024 TECO
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Finde alle .c Dateien im aktuellen Verzeichnis
file(GLOB C_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

target_sources(app PRIVATE
    ${C_SOURCES}
)
