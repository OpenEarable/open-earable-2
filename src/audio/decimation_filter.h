/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#ifndef _DECIMATION_FILTER_H_
#define _DECIMATION_FILTER_H_

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

#ifdef __cplusplus

/**
 * @brief Single stage decimation filter class
 */
class Decimator {
public:
    /**
     * @brief Constructor
     * @param factor Decimation factor (2 or 3)
     */
    Decimator(uint8_t factor);
    
    /**
     * @brief Destructor
     */
    ~Decimator() = default;
    
    /**
     * @brief Initialize the decimator
     * @return 0 on success, negative on error
     */
    int init();
    
    /**
     * @brief Process stereo int16 audio with decimation
     * @param input Input buffer (interleaved stereo int16)
     * @param output Output buffer (interleaved stereo int16)
     * @param num_frames Number of input stereo frames
     * @return Number of output frames, or negative on error
     */
    int process(const int16_t* input, int16_t* output, uint32_t num_frames);
    
    /**
     * @brief Reset filter state
     */
    void reset();
    
    /**
     * @brief Get decimation factor
     * @return Decimation factor
     */
    uint8_t getFactor() const { return factor_; }

private:
    static constexpr uint32_t MAX_FRAMES = 512;
    static constexpr uint32_t NUM_STAGES = 2;
    
    uint8_t factor_;
    bool initialized_;
    arm_biquad_cascade_stereo_df2T_instance_f32 biquad_;
    float32_t state_[4 * NUM_STAGES];
    float32_t temp_f32_[MAX_FRAMES * 2];
    
    const float32_t* getCoefficients() const;
};

/**
 * @brief Cascaded decimation filter class
 */
class CascadedDecimator {
public:
    /**
     * @brief Constructor for predefined cascaded decimation factors
     * @param total_factor Total decimation factor (4, 6, 8, or 12)
     */
    CascadedDecimator(uint8_t total_factor);
    
    /**
     * @brief Destructor
     */
    ~CascadedDecimator();
    
    /**
     * @brief Initialize all cascaded decimators
     * @return 0 on success, negative on error
     */
    int init();
    
    /**
     * @brief Process stereo int16 audio with cascaded decimation
     * @param input Input buffer (interleaved stereo int16)
     * @param output Output buffer (interleaved stereo int16)
     * @param num_frames Number of input stereo frames
     * @return Number of output frames, or negative on error
     */
    int process(const int16_t* input, int16_t* output, uint32_t num_frames);
    
    /**
     * @brief Reset all filter states
     */
    void reset();
    
    /**
     * @brief Get total decimation factor
     * @return Total decimation factor
     */
    uint8_t getTotalFactor() const { return total_factor_; }

private:
    static constexpr uint32_t MAX_STAGES = 4;
    static constexpr uint32_t MAX_FRAMES = 512;
    
    uint8_t total_factor_;
    uint8_t num_stages_;
    Decimator* stages_[MAX_STAGES];
    int16_t* temp_buffers_[MAX_STAGES - 1];
    
    void setupStages();
    void cleanupStages();
};
#endif
#endif /* _DECIMATION_FILTER_H_ */
