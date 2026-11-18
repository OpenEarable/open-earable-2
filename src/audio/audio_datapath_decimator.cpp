/*
 * Audio datapath utility functions for CascadedDecimator
 */

#include "decimation_filter.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(audio_datapath);

#ifdef __cplusplus

/* Fixed-capacity storage avoids C++ allocation failure in the audio work queue. */
static CascadedDecimator g_audio_decimator;
static bool g_audio_decimator_initialized;

/* Mutex to protect access to the decimator during cleanup */
K_MUTEX_DEFINE(decimator_mutex);

extern "C" {
/**
 * @brief Reset the audio decimator filter state
 */
void audio_datapath_decimator_reset(void) {
    k_mutex_lock(&decimator_mutex, K_FOREVER);
    if (g_audio_decimator_initialized) {
        g_audio_decimator.reset();
        LOG_DBG("CascadedDecimator state reset");
    }
    k_mutex_unlock(&decimator_mutex);
}

/**
 * @brief Get current decimation factor
 * @return Current total decimation factor, or 0 if not initialized
 */
uint8_t audio_datapath_decimator_get_factor(void) {
    k_mutex_lock(&decimator_mutex, K_FOREVER);
    uint8_t factor = g_audio_decimator_initialized ? g_audio_decimator.getTotalFactor() : 0;
    k_mutex_unlock(&decimator_mutex);
    return factor;
}

/**
 * @brief Initialize the audio decimator with specified factor
 * @param factor Decimation factor (4, 6, 8, or 12)
 * @return 0 on success, negative on error
 */
int audio_datapath_decimator_init(uint8_t factor) {
    k_mutex_lock(&decimator_mutex, K_FOREVER);

    g_audio_decimator_initialized = false;
    g_audio_decimator.cleanup();

    int ret = g_audio_decimator.configure(factor);
    if (ret != 0) {
        LOG_ERR("Failed to configure CascadedDecimator: %d", ret);
        k_mutex_unlock(&decimator_mutex);
        return ret;
    }

    ret = g_audio_decimator.init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize CascadedDecimator: %d", ret);
        g_audio_decimator.cleanup();
        k_mutex_unlock(&decimator_mutex);
        return ret;
    }

    g_audio_decimator_initialized = true;
    LOG_DBG("CascadedDecimator (%dx) initialized successfully", factor);
    k_mutex_unlock(&decimator_mutex);
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
    /* Lock mutex to prevent cleanup during processing */
    if (k_mutex_lock(&decimator_mutex, K_NO_WAIT) != 0) {
        /* Mutex is held by cleanup - decimator is being deleted */
        LOG_DBG("Decimator locked for cleanup, skipping process");
        return 0;
    }
    
    if (!g_audio_decimator_initialized) {
        k_mutex_unlock(&decimator_mutex);
        LOG_WRN("CascadedDecimator not available, returning 0 frames");
        return 0;
    }

    int result = g_audio_decimator.process(input, output, num_frames);

    k_mutex_unlock(&decimator_mutex);
    return result;
}

/**
 * @brief Cleanup the audio decimator
 */
void audio_datapath_decimator_cleanup(void) {
    k_mutex_lock(&decimator_mutex, K_FOREVER);

    if (g_audio_decimator_initialized) {
        LOG_DBG("Cleaning up CascadedDecimator");
        g_audio_decimator.cleanup();
        g_audio_decimator_initialized = false;
    }

    k_mutex_unlock(&decimator_mutex);
}

};
#endif
