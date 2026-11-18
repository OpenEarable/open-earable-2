# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(audio_encoder_tests)

test_runner_generate(src/main.c)
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/audio
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/utils/macros
    ${ZEPHYR_NRFXLIB_MODULE_DIR}/lc3/include)
# Exercise the production wrapper with a fake LC3 codec on the host.
target_compile_definitions(app PRIVATE
    CONFIG_SW_CODEC_LC3=1
    CONFIG_SW_CODEC_SELECT_LOG_LEVEL=0
    CONFIG_AUDIO_ENCODE_CHANNELS_MAX=2
    CONFIG_AUDIO_DECODE_CHANNELS_MAX=2
    CONFIG_AUDIO_OUTPUT_CHANNELS=2
    CONFIG_AUDIO_SAMPLE_RATE_HZ=48000
    CONFIG_AUDIO_BIT_DEPTH_BITS=16
    CONFIG_AUDIO_BIT_DEPTH_OCTETS=2
    CONFIG_AUDIO_FRAME_DURATION_US=10000
    CONFIG_LC3_BITRATE_MAX=96000)
target_sources(app PRIVATE src/main.c src/codec_stub.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/audio/sw_codec_select.c)
