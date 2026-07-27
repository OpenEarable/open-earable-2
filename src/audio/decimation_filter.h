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
#include "audio_i2s.h"

#ifdef __cplusplus

static_assert(BLOCK_SIZE_BYTES % (sizeof(int16_t) * 2U) == 0,
              "The audio block must contain complete stereo int16 frames");

/**
 * @brief Single stage decimation filter class
 */
class Decimator {
public:
    /** Maximum number of stereo frames supplied by one audio datapath block. */
    static constexpr uint32_t MAX_FRAMES =
        BLOCK_SIZE_BYTES / (sizeof(int16_t) * 2U);

    /**
     * @brief Construct an unconfigured decimation stage.
     */
    Decimator();
    
    /**
     * @brief Destructor
     */
    ~Decimator() = default;

    /**
     * @brief Configure this stage for a decimation factor.
     * @param factor Decimation factor (2 or 3)
     */
    void configure(uint8_t factor);
    
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
     * @param processing_buffer In-place float processing buffer for stereo frames
     * @return Number of output frames, or negative on error
     */
    int process(const int16_t* input, int16_t* output, uint32_t num_frames,
                float32_t* processing_buffer);
    
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
    static constexpr uint32_t NUM_STAGES = 2;
    
    uint8_t factor_;
    bool initialized_;
    arm_biquad_cascade_stereo_df2T_instance_f32 biquad_;
    float32_t state_[4 * NUM_STAGES];
    
    const float32_t* getCoefficients() const;
};

/**
 * @brief Cascaded decimation filter class
 */
class CascadedDecimator {
public:
    /**
     * @brief Construct an unconfigured cascaded decimator.
     */
    CascadedDecimator();
    
    /**
     * @brief Destructor
     */
    ~CascadedDecimator() = default;

    /**
     * @brief Configure the statically allocated filter stages.
     * @param total_factor Total decimation factor (1, 2, 3, 4, 6, 8, 12, 16, or 24)
     * @return 0 on success, or -EINVAL for an unsupported factor.
     */
    int configure(uint8_t total_factor);

    /**
     * @brief Clear configuration and filter state without releasing memory.
     */
    void cleanup();
    
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
    static constexpr uint32_t MAX_FRAMES = Decimator::MAX_FRAMES;
    
    uint8_t total_factor_;
    uint8_t num_stages_;
    bool configured_;
    Decimator stages_[MAX_STAGES];
    /* Stage output is interleaved stereo, so each frame needs two samples. */
    int16_t intermediate_buffer_[MAX_FRAMES * 2U];
    float32_t processing_buffer_[MAX_FRAMES * 2];
};
#endif
#endif /* _DECIMATION_FILTER_H_ */
