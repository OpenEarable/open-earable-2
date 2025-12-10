/*
 * Audio datapath utility functions for CascadedDecimator
 */

#include "decimation_filter.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(audio_datapath);

#ifdef __cplusplus

/* CascadedDecimator instance for direct C++ usage */
static CascadedDecimator* g_audio_decimator = nullptr;

extern "C" {
/**
 * @brief Reset the audio decimator filter state
 */
void audio_datapath_decimator_reset(void) {
    if (g_audio_decimator) {
        g_audio_decimator->reset();
        LOG_DBG("CascadedDecimator state reset");
    }
}

/**
 * @brief Get current decimation factor
 * @return Current total decimation factor, or 0 if not initialized
 */
uint8_t audio_datapath_decimator_get_factor(void) {
    return g_audio_decimator ? g_audio_decimator->getTotalFactor() : 0;
}

/**
 * @brief Initialize the audio decimator with specified factor
 * @param factor Decimation factor (4, 6, 8, or 12)
 * @return 0 on success, negative on error
 */
int audio_datapath_decimator_init(uint8_t factor) {
    if (g_audio_decimator) {
        delete g_audio_decimator;
        g_audio_decimator = nullptr;
    }
    
    g_audio_decimator = new CascadedDecimator(factor);
    if (!g_audio_decimator) {
        LOG_ERR("Failed to create CascadedDecimator with factor %d", factor);
        return -ENOMEM;
    }
    
    int ret = g_audio_decimator->init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize CascadedDecimator: %d", ret);
        delete g_audio_decimator;
        g_audio_decimator = nullptr;
        return ret;
    }
    
    LOG_DBG("CascadedDecimator (%dx) initialized successfully", factor);
    return 0;
}

/**
 * @brief Process audio data through the decimator
 * @param input Input buffer (interleaved stereo int16)
 * @param output Output buffer (interleaved stereo int16)
 * @param num_frames Number of input stereo frames
 * @return Number of output frames, or negative on error
 */
int audio_datapath_decimator_process(const int16_t* input, int16_t* output, uint32_t num_frames) {
    if (!g_audio_decimator) {
        LOG_ERR("CascadedDecimator not initialized");
        return -EINVAL;
    }
    
    return g_audio_decimator->process(input, output, num_frames);
}

/**
 * @brief Cleanup the audio decimator
 */
void audio_datapath_decimator_cleanup(void) {
    if (g_audio_decimator) {
        LOG_DBG("Cleaning up CascadedDecimator");
        delete g_audio_decimator;
        g_audio_decimator = nullptr;
    }
}

};
#endif