/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include <cstring>
#include <memory>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(decimator_cpp, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

// Filter coefficients for different decimation factors
static const float32_t coeff_dec2[2 * 5] = {
    /* Stage 1: Butterworth LP, Fc=0.4*Fs_out */
    9.39808514e-02f, 1.87961703e-01f, 9.39808514e-02f, 1.38777878e-16f, -3.95661299e-02f,
    /* Stage 2 */
    1.0f, 2.0f, 1.0f, 1.11022302e-16f, -4.46462692e-01f
};

static const float32_t coeff_dec3[2 * 5] = {
    /* Stage 1: Butterworth LP, Fc=0.33*Fs_out */
    0.01020948f, 0.02041896f, 0.01020948f, 0.85539793f, -0.20971536f,
    /* Stage 2 */
    1.0f, 2.0f, 1.0f, 0.75108142f, -0.50216284f
};

// Decimator class implementation
Decimator::Decimator(uint8_t factor) 
    : factor_(factor), initialized_(false) {
    memset(state_, 0, sizeof(state_));
    memset(temp_f32_, 0, sizeof(temp_f32_));
}

int Decimator::init() {
    if (factor_ != 2 && factor_ != 3) {
        LOG_ERR("Invalid decimation factor: %d (only 2 or 3 supported)", factor_);
        return -EINVAL;
    }
    
    const float32_t* coeffs = getCoefficients();
    if (!coeffs) {
        LOG_ERR("Failed to get coefficients for factor %d", factor_);
        return -EINVAL;
    }
    
    arm_biquad_cascade_stereo_df2T_init_f32(&biquad_, NUM_STAGES, coeffs, state_);
    memset(state_, 0, sizeof(state_));
    
    initialized_ = true;
    LOG_DBG("Decimator initialized with factor %d", factor_);
    
    return 0;
}

int Decimator::process(const int16_t* input, int16_t* output, uint32_t num_frames) {
    if (!initialized_ || !input || !output || num_frames == 0 || num_frames > MAX_FRAMES) {
        return -EINVAL;
    }
    
    uint32_t num_samples = num_frames * 2;
    
    // Convert int16 to float32
    for (uint32_t i = 0; i < num_samples; i++) {
        temp_f32_[i] = static_cast<float32_t>(input[i]);
    }
    
    // Apply anti-aliasing filter using stereo DF2T
    static float32_t filtered[MAX_FRAMES * 2];
    arm_biquad_cascade_stereo_df2T_f32(&biquad_, temp_f32_, filtered, num_frames);
    
    // Clip to prevent overflow
    arm_clip_f32(filtered, filtered, -32768.0f, 32767.0f, num_samples);
    
    // Decimate and convert back to int16
    uint32_t out_frames = num_frames / factor_;
    uint32_t step = factor_ * 2;
    
    for (uint32_t i = 0; i < out_frames; i++) {
        output[i * 2] = static_cast<int16_t>(filtered[i * step]);
        output[i * 2 + 1] = static_cast<int16_t>(filtered[i * step + 1]);
    }
    
    return out_frames;
}

void Decimator::reset() {
    if (initialized_) {
        memset(state_, 0, sizeof(state_));
    }
}

const float32_t* Decimator::getCoefficients() const {
    switch (factor_) {
        case 2: return coeff_dec2;
        case 3: return coeff_dec3;
        default: return nullptr;
    }
}

// CascadedDecimator class implementation
CascadedDecimator::CascadedDecimator(uint8_t total_factor)
    : total_factor_(total_factor), num_stages_(0) {
    memset(stages_, 0, sizeof(stages_));
    memset(temp_buffers_, 0, sizeof(temp_buffers_));
    setupStages();
}

CascadedDecimator::~CascadedDecimator() {
    cleanupStages();
}

void CascadedDecimator::setupStages() {
    switch (total_factor_) {
        case 1: // No decimation
            num_stages_ = 0;
            break;
        case 2: // 2x
            num_stages_ = 1;
            stages_[0] = new Decimator(2);
            break;
        case 3: // 3x
            num_stages_ = 1;
            stages_[0] = new Decimator(3);
            break;
        case 4: // 2x -> 2x
            num_stages_ = 2;
            stages_[0] = new Decimator(2);
            stages_[1] = new Decimator(2);
            break;
            
        case 6: // 3x -> 2x
            num_stages_ = 2;
            stages_[0] = new Decimator(3);
            stages_[1] = new Decimator(2);
            break;
            
        case 8: // 2x -> 2x -> 2x
            num_stages_ = 3;
            stages_[0] = new Decimator(2);
            stages_[1] = new Decimator(2);
            stages_[2] = new Decimator(2);
            break;
            
        case 12: // 3x -> 2x -> 2x
            num_stages_ = 3;
            stages_[0] = new Decimator(3);
            stages_[1] = new Decimator(2);
            stages_[2] = new Decimator(2);
            break;

        case 16: // 2x -> 2x -> 2x -> 2x
            num_stages_ = 4;
            stages_[0] = new Decimator(2);
            stages_[1] = new Decimator(2);
            stages_[2] = new Decimator(2);
            stages_[3] = new Decimator(2);
            break;

        case 24: // 3x -> 2x -> 2x -> 2x
            num_stages_ = 4;
            stages_[0] = new Decimator(3);
            stages_[1] = new Decimator(2);
            stages_[2] = new Decimator(2);
            stages_[3] = new Decimator(2);
            break;
            
        default:
            LOG_ERR("Unsupported total decimation factor: %d", total_factor_);
            num_stages_ = 0;
            return;
    }
    
    // Allocate temporary buffers between stages
    for (uint8_t i = 0; i < num_stages_ - 1; i++) {
        temp_buffers_[i] = new int16_t[MAX_FRAMES * 2];
    }
    
    LOG_DBG("CascadedDecimator setup for factor %d with %d stages", total_factor_, num_stages_);
}

void CascadedDecimator::cleanupStages() {
    for (uint8_t i = 0; i < num_stages_; i++) {
        delete stages_[i];
        stages_[i] = nullptr;
    }
    
    for (uint8_t i = 0; i < MAX_STAGES - 1; i++) {
        delete[] temp_buffers_[i];
        temp_buffers_[i] = nullptr;
    }
}

int CascadedDecimator::init() {
    if (num_stages_ == 0) {
        LOG_ERR("No stages configured for total factor %d", total_factor_);
        return -EINVAL;
    }
    
    for (uint8_t i = 0; i < num_stages_; i++) {
        if (!stages_[i]) {
            LOG_ERR("Stage %d is null", i);
            return -EINVAL;
        }
        
        int ret = stages_[i]->init();
        if (ret != 0) {
            LOG_ERR("Failed to initialize stage %d: %d", i, ret);
            return ret;
        }
    }
    
    LOG_INF("CascadedDecimator initialized: total factor %d", total_factor_);
    return 0;
}

int CascadedDecimator::process(const int16_t* input, int16_t* output, uint32_t num_frames) {
    if (num_stages_ == 0) {
        return num_frames;
    }
    
    if (!input || !output || num_frames == 0) {
        return -EINVAL;
    }
    
    const int16_t* stage_input = input;
    int16_t* stage_output = nullptr;
    int frames = num_frames;
    
    for (uint8_t i = 0; i < num_stages_; i++) {
        // Determine output buffer for this stage
        if (i == num_stages_ - 1) {
            // Last stage outputs to final output buffer
            stage_output = output;
        } else {
            // Intermediate stage outputs to temp buffer
            stage_output = temp_buffers_[i];
        }
        
        // Process this stage
        frames = stages_[i]->process(stage_input, stage_output, frames);
        if (frames < 0) {
            LOG_ERR("Stage %d processing failed: %d", i, frames);
            return frames;
        }
        
        // Next stage input is this stage's output
        stage_input = stage_output;
    }
    
    return frames;
}

void CascadedDecimator::reset() {
    for (uint8_t i = 0; i < num_stages_; i++) {
        if (stages_[i]) {
            stages_[i]->reset();
        }
    }
}