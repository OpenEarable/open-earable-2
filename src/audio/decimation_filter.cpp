/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include <cstring>
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
Decimator::Decimator()
    : factor_(0), initialized_(false) {
    memset(state_, 0, sizeof(state_));
}

void Decimator::configure(uint8_t factor) {
    factor_ = factor;
    initialized_ = false;
    memset(&biquad_, 0, sizeof(biquad_));
    memset(state_, 0, sizeof(state_));
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

int Decimator::process(const int16_t* input, int16_t* output, uint32_t num_frames,
                       float32_t* processing_buffer) {
    if (!initialized_ || !input || !output || !processing_buffer || num_frames == 0 ||
        num_frames > MAX_FRAMES) {
        return -EINVAL;
    }
    
    uint32_t num_samples = num_frames * 2;
    
    // Convert int16 to float32
    for (uint32_t i = 0; i < num_samples; i++) {
        processing_buffer[i] = static_cast<float32_t>(input[i]);
    }

    // CMSIS-DSP stereo DF2T supports identical source and destination buffers.
    arm_biquad_cascade_stereo_df2T_f32(&biquad_, processing_buffer, processing_buffer,
                                       num_frames);

    // Clip to prevent overflow
    arm_clip_f32(processing_buffer, processing_buffer, -32768.0f, 32767.0f, num_samples);
    
    // Decimate and convert back to int16
    uint32_t out_frames = num_frames / factor_;
    uint32_t step = factor_ * 2;
    
    for (uint32_t i = 0; i < out_frames; i++) {
        output[i * 2] = static_cast<int16_t>(processing_buffer[i * step]);
        output[i * 2 + 1] = static_cast<int16_t>(processing_buffer[i * step + 1]);
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
CascadedDecimator::CascadedDecimator()
    : total_factor_(0), num_stages_(0), configured_(false) {
    memset(intermediate_buffer_, 0, sizeof(intermediate_buffer_));
}

int CascadedDecimator::configure(uint8_t total_factor) {
    uint8_t stage_factors[MAX_STAGES] = {};

    cleanup();
    switch (total_factor) {
        case 1: // No decimation
            num_stages_ = 0;
            break;
        case 2: // 2x
            num_stages_ = 1;
            stage_factors[0] = 2;
            break;
        case 3: // 3x
            num_stages_ = 1;
            stage_factors[0] = 3;
            break;
        case 4: // 2x -> 2x
            num_stages_ = 2;
            stage_factors[0] = 2;
            stage_factors[1] = 2;
            break;
        case 6: // 3x -> 2x
            num_stages_ = 2;
            stage_factors[0] = 3;
            stage_factors[1] = 2;
            break;
        case 8: // 2x -> 2x -> 2x
            num_stages_ = 3;
            stage_factors[0] = 2;
            stage_factors[1] = 2;
            stage_factors[2] = 2;
            break;
        case 12: // 3x -> 2x -> 2x
            num_stages_ = 3;
            stage_factors[0] = 3;
            stage_factors[1] = 2;
            stage_factors[2] = 2;
            break;
        case 16: // 2x -> 2x -> 2x -> 2x
            num_stages_ = 4;
            stage_factors[0] = 2;
            stage_factors[1] = 2;
            stage_factors[2] = 2;
            stage_factors[3] = 2;
            break;
        case 24: // 3x -> 2x -> 2x -> 2x
            num_stages_ = 4;
            stage_factors[0] = 3;
            stage_factors[1] = 2;
            stage_factors[2] = 2;
            stage_factors[3] = 2;
            break;
        default:
            LOG_ERR("Unsupported total decimation factor: %d", total_factor);
            return -EINVAL;
    }

    for (uint8_t i = 0; i < num_stages_; i++) {
        stages_[i].configure(stage_factors[i]);
    }

    total_factor_ = total_factor;
    configured_ = true;
    LOG_DBG("CascadedDecimator setup for factor %d with %d stages", total_factor_, num_stages_);
    return 0;
}

void CascadedDecimator::cleanup() {
    for (uint8_t i = 0; i < MAX_STAGES; i++) {
        stages_[i].configure(0);
    }
    total_factor_ = 0;
    num_stages_ = 0;
    configured_ = false;
}

int CascadedDecimator::init() {
    if (!configured_) {
        return -EINVAL;
    }

    for (uint8_t i = 0; i < num_stages_; i++) {
        int ret = stages_[i].init();
        if (ret != 0) {
            LOG_ERR("Failed to initialize stage %d: %d", i, ret);
            return ret;
        }
    }
    
    LOG_INF("CascadedDecimator initialized: total factor %d", total_factor_);
    return 0;
}

int CascadedDecimator::process(const int16_t* input, int16_t* output, uint32_t num_frames) {
    if (!configured_ || !input || !output || num_frames == 0 || num_frames > MAX_FRAMES) {
        return -EINVAL;
    }

    if (num_stages_ == 0) {
        if (output != input) {
            memcpy(output, input, num_frames * 2U * sizeof(int16_t));
        }
        return num_frames;
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
            // Alternate storage so adjacent stages never share input and output.
            stage_output = (i % 2 == 0) ? intermediate_buffer_ : output;
        }
        
        // Process this stage
        frames = stages_[i].process(stage_input, stage_output, frames, processing_buffer_);
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
        stages_[i].reset();
    }
}
