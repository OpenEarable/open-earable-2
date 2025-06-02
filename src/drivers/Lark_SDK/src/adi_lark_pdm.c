/*!
 * @brief     Programmable Droop Compenstation (PDM) APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#define LARK_PDM_CHANNELS       2

/*============= C O D E ====================*/
int32_t adi_lark_pdm_enable_power_on(adi_lark_device_t *device, uint8_t pdm_channel, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pdm_channel >= LARK_PDM_CHANNELS);

    err = adi_lark_hal_bf_write(device, BF_PDM0_EN_INFO + pdm_channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_set_sample_rate(adi_lark_device_t *device, adi_lark_pdm_sample_rate_e rate)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(rate > API_LARK_PDM_SAMPLE_RATE_768KHz);
    
    err = adi_lark_hal_bf_write(device, BF_PDM_FS_INFO, rate);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_set_output_rate(adi_lark_device_t *device, adi_lark_pdm_output_rate_e rate)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_PDM_RATE_INFO, rate);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_enable_high_pass_filter(adi_lark_device_t *device, uint8_t pdm_channel, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_PDM0_HPF_EN_INFO + pdm_channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_enable_high_freq_response_compensation(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_PDM_FCOMP_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_enable_more_interp_filter(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_PDM_MORE_FILT_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool use_chan0_vol, bool change_at_zc)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_PDM_HARD_VOL_INFO, hard_vol_change ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PDM_VOL_LINK_INFO, use_chan0_vol ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PDM_VOL_ZC_INFO, change_at_zc ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_enable_mute(adi_lark_device_t *device, uint8_t pdm_channel, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pdm_channel >= LARK_PDM_CHANNELS);

    err = adi_lark_hal_bf_write(device, BF_PDM0_MUTE_INFO + pdm_channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_set_volume(adi_lark_device_t *device, uint8_t pdm_channel, uint8_t volume)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pdm_channel >= LARK_PDM_CHANNELS);

    err = adi_lark_hal_bf_write(device, pdm_channel + BF_PDM0_VOL_INFO, volume);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pdm_set_volume_db(adi_lark_device_t *device, uint8_t pdm_channel, float vol_db)
{
    #ifdef LARK_SDK
    int32_t err;
    float volume;
    uint8_t vol_code;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pdm_channel >= LARK_PDM_CHANNELS);
    
    volume = (24 - vol_db) / 0.375;
    volume = (volume < 0) ? 0 : volume;
    volume = (volume > 255) ? 255 : volume;
    vol_code = (uint8_t)(volume + 0.5);
    
    err = adi_lark_pdm_set_volume(device, pdm_channel, vol_code);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
} 

int32_t adi_lark_pdm_select_input_route(adi_lark_device_t *device, uint8_t pdm_channel, adi_lark_pdm_input_route_e route)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pdm_channel >= LARK_PDM_CHANNELS);
    LARK_INVALID_PARAM_RETURN(route >= API_LARK_PDM_ROUTE_COUNT);

    err = adi_lark_hal_bf_write(device, pdm_channel + BF_PDM0_ROUTE_INFO, route);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

/*! @} */
