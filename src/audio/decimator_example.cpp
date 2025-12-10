/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(decimator_example, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

/**
 * @brief Example showing how to use the new Decimator and CascadedDecimator classes
 */

void decimator_usage_example() {
    // Example 1: Single stage decimation by factor 2
    Decimator dec2(2);
    if (dec2.init() == 0) {
        LOG_INF("2x Decimator initialized successfully");
        
        int16_t input_2x[96];  // 48 stereo frames at high rate
        int16_t output_2x[48]; // 24 stereo frames at output rate
        
        // Fill input with test data
        for (int i = 0; i < 96; i++) {
            input_2x[i] = i * 100;
        }
        
        int result = dec2.process(input_2x, output_2x, 48);
        LOG_INF("2x Decimator processed %d frames -> %d frames", 48, result);
    }
    
    // Example 2: Single stage decimation by factor 3
    Decimator dec3(3);
    if (dec3.init() == 0) {
        LOG_INF("3x Decimator initialized successfully");
        
        int16_t input_3x[144]; // 72 stereo frames at high rate
        int16_t output_3x[48];  // 24 stereo frames at output rate
        
        int result = dec3.process(input_3x, output_3x, 72);
        LOG_INF("3x Decimator processed %d frames -> %d frames", 72, result);
    }
    
    // Example 3: Cascaded decimation by factor 4 (2x -> 2x)
    CascadedDecimator dec4(4);
    if (dec4.init() == 0) {
        LOG_INF("4x CascadedDecimator (2x->2x) initialized successfully");
        
        int16_t input_4x[192]; // 96 stereo frames at input rate (e.g., 192kHz)
        int16_t output_4x[48];  // 24 stereo frames at output rate (e.g., 48kHz)
        
        int result = dec4.process(input_4x, output_4x, 96);
        LOG_INF("4x CascadedDecimator processed %d frames -> %d frames", 96, result);
    }
    
    // Example 4: Cascaded decimation by factor 6 (3x -> 2x)
    CascadedDecimator dec6(6);
    if (dec6.init() == 0) {
        LOG_INF("6x CascadedDecimator (3x->2x) initialized successfully");
        
        int16_t input_6x[288]; // 144 stereo frames at input rate 
        int16_t output_6x[48];  // 24 stereo frames at output rate
        
        int result = dec6.process(input_6x, output_6x, 144);
        LOG_INF("6x CascadedDecimator processed %d frames -> %d frames", 144, result);
    }
    
    // Example 5: Cascaded decimation by factor 8 (2x -> 2x -> 2x)
    CascadedDecimator dec8(8);
    if (dec8.init() == 0) {
        LOG_INF("8x CascadedDecimator (2x->2x->2x) initialized successfully");
        
        int16_t input_8x[384]; // 192 stereo frames at input rate
        int16_t output_8x[48];  // 24 stereo frames at output rate
        
        int result = dec8.process(input_8x, output_8x, 192);
        LOG_INF("8x CascadedDecimator processed %d frames -> %d frames", 192, result);
    }
    
    // Example 6: Cascaded decimation by factor 12 (3x -> 2x -> 2x)
    CascadedDecimator dec12(12);
    if (dec12.init() == 0) {
        LOG_INF("12x CascadedDecimator (3x->2x->2x) initialized successfully");
        
        int16_t input_12x[576]; // 288 stereo frames at input rate
        int16_t output_12x[48];  // 24 stereo frames at output rate
        
        int result = dec12.process(input_12x, output_12x, 288);
        LOG_INF("12x CascadedDecimator processed %d frames -> %d frames", 288, result);
    }
}