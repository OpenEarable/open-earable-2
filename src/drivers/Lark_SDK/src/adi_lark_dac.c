/*!
 * @brief     DAC APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */

/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_dac_enable_power_on(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_PB0_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_set_sample_rate(adi_lark_device_t *device, adi_lark_dac_sample_rate_e rate)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(rate > API_LARK_DAC_SAMPLE_RATE_768KHz);

    err = adi_lark_hal_bf_write(device, BF_DAC_FS_INFO, rate);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_set_filter(adi_lark_device_t *device, bool hpf_enable, bool high_freq_comp_enable, bool interp_enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_DAC0_HPF_EN_INFO, hpf_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DAC_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DAC_MORE_FILT_INFO, interp_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_enable_low_power_mode(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DAC_LPM_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool change_at_zc)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DAC_HARD_VOL_INFO, hard_vol_change ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DAC_VOL_ZC_INFO, change_at_zc ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_enable_mute(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DAC0_MUTE_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DAC0_FORCE_MUTE_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_set_volume(adi_lark_device_t *device, uint8_t volume)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DAC0_VOL_INFO, volume);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_set_volume_db(adi_lark_device_t *device, float vol_db)
{
    int32_t err;
    float volume;
    uint8_t vol_code;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    volume = (24 - vol_db) / 0.375;
    volume = (volume < 0) ? 0 : volume;
    volume = (volume > 255) ? 255 : volume;
    vol_code = (uint8_t)(volume + 0.5);
    
    err = adi_lark_dac_set_volume(device, vol_code);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dac_select_input_route(adi_lark_device_t *device, adi_lark_dac_input_route_e route)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(route >= API_LARK_DAC_ROUTE_COUNT);
    #endif
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(route > API_LARK_DAC_ROUTE_FINT1);
    #endif

    err = adi_lark_hal_bf_write(device, BF_DAC0_ROUTE_INFO, route);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

/*! @} */
