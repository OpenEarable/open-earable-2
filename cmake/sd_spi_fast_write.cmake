# Keep the SDK installation unchanged. Build a patched copy of its SD driver.
if(NOT CONFIG_OPENEARABLE_SD_SPI_FAST_WRITE)
  return()
endif()

set(sd_spi_source "${ZEPHYR_BASE}/drivers/sdhc/sdhc_spi.c")
set(sd_spi_patch "${CMAKE_CURRENT_LIST_DIR}/../patches/sdhc-spi-fast-write.patch")
set(sd_spi_build "${CMAKE_CURRENT_BINARY_DIR}/sd-spi-fast-write")
file(MAKE_DIRECTORY "${sd_spi_build}/drivers/sdhc")
configure_file("${sd_spi_source}" "${sd_spi_build}/drivers/sdhc/sdhc_spi.c" COPYONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${sd_spi_patch}")

find_package(Git REQUIRED)
execute_process(
  COMMAND "${GIT_EXECUTABLE}" "--git-dir=${sd_spi_build}/.git" apply --unsafe-paths
          "--directory=${sd_spi_build}" "${sd_spi_patch}"
  WORKING_DIRECTORY "${sd_spi_build}"
  RESULT_VARIABLE sd_spi_patch_result
  ERROR_VARIABLE sd_spi_patch_error
)
if(NOT sd_spi_patch_result EQUAL 0)
  message(FATAL_ERROR "Cannot apply SD SPI patch to this SDK: ${sd_spi_patch_error}")
endif()
# An explicit Git directory prevents discovery of a parent checkout, where
# git apply can silently skip paths under an ignored build directory.
file(SHA256 "${sd_spi_source}" sd_spi_original_hash)
file(SHA256 "${sd_spi_build}/drivers/sdhc/sdhc_spi.c" sd_spi_patched_hash)
if(sd_spi_original_hash STREQUAL sd_spi_patched_hash)
  message(FATAL_ERROR "SD SPI patch was skipped; refusing an unpatched build")
endif()

get_target_property(sd_spi_sources drivers__sdhc SOURCES)
set(sd_spi_replaced FALSE)
foreach(source IN LISTS sd_spi_sources)
  if(source MATCHES "(^|/)sdhc_spi\\.c$")
    list(REMOVE_ITEM sd_spi_sources "${source}")
    list(APPEND sd_spi_sources "${sd_spi_build}/drivers/sdhc/sdhc_spi.c")
    set(sd_spi_replaced TRUE)
  endif()
endforeach()
if(NOT sd_spi_replaced)
  message(FATAL_ERROR "SDK SD SPI driver source was not found")
endif()
set_property(TARGET drivers__sdhc PROPERTY SOURCES "${sd_spi_sources}")
