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

/* Mutex to protect access to the decimator during cleanup */
K_MUTEX_DEFINE(decimator_mutex);

/* Flag to indicate decimator is being used - provides fast-path check */
static volatile bool g_decimator_in_use = false;

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
    /* Lock mutex to ensure no concurrent access during init */
    k_mutex_lock(&decimator_mutex, K_FOREVER);
    
    if (g_audio_decimator) {
        delete g_audio_decimator;
        g_audio_decimator = nullptr;
    }
    
    g_audio_decimator = new CascadedDecimator(factor);
    if (!g_audio_decimator) {
        LOG_ERR("Failed to create CascadedDecimator with factor %d", factor);
        k_mutex_unlock(&decimator_mutex);
        return -ENOMEM;
    }
    
    int ret = g_audio_decimator->init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize CascadedDecimator: %d", ret);
        delete g_audio_decimator;
        g_audio_decimator = nullptr;
        k_mutex_unlock(&decimator_mutex);
        return ret;
    }
    
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
    
    if (!g_audio_decimator) {
        k_mutex_unlock(&decimator_mutex);
        LOG_WRN("CascadedDecimator not available, returning 0 frames");
        return 0;
    }
    
    g_decimator_in_use = true;
    int result = g_audio_decimator->process(input, output, num_frames);
    g_decimator_in_use = false;
    
    k_mutex_unlock(&decimator_mutex);
    return result;
}

/**
 * @brief Cleanup the audio decimator
 */
void audio_datapath_decimator_cleanup(void) {
    /* Acquire mutex to ensure no processing is happening */
    k_mutex_lock(&decimator_mutex, K_FOREVER);
    
    if (g_audio_decimator) {
        /* Wait briefly if decimator was recently in use (safety margin) */
        if (g_decimator_in_use) {
            LOG_WRN("Decimator still in use, waiting...");
            k_sleep(K_MSEC(5));
        }
        
        LOG_DBG("Cleaning up CascadedDecimator");
        delete g_audio_decimator;
        g_audio_decimator = nullptr;
    }
    
    k_mutex_unlock(&decimator_mutex);
}

};
#endif