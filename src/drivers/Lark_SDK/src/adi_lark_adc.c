/*!
 * @brief     ADC APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */

/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#define LARK_ADC_CHANNELS       3

/*============= C O D E ====================*/
int32_t adi_lark_adc_enable_power_on(adi_lark_device_t *device, uint8_t adc_channel, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    err = adi_lark_hal_bf_write(device, BF_ADC0_EN_INFO + adc_channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_get_power_state(adi_lark_device_t *device, uint8_t adc_channel, uint32_t *power_state)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    err = adi_lark_hal_bf_read(device, BF_ADC0_EN_INFO + adc_channel, power_state);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_sample_rate(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_sample_rate_e rate)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);
    LARK_INVALID_PARAM_RETURN(rate > API_LARK_ADC_SAMPLE_RATE_768KHz);

    if (adc_channel < 2)
    {
        err = adi_lark_hal_bf_write(device, BF_ADC01_FS_INFO, rate);
    }
    else
    {
        err = adi_lark_hal_bf_write(device, BF_ADC2_FS_INFO, rate);
    }
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_get_sample_rate(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_sample_rate_e *rate)
{
    int32_t  err;
    uint32_t bfvalue;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    if (adc_channel < 2)
    {
        err = adi_lark_hal_bf_read(device, BF_ADC01_FS_INFO, &bfvalue);
    }
    else
    {
        err = adi_lark_hal_bf_read(device, BF_ADC2_FS_INFO, &bfvalue);
    }
    *rate = (adi_lark_adc_sample_rate_e)bfvalue;
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_filter(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_hpf_e filter, bool higher_order_enable, bool high_freq_comp_enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    err = adi_lark_hal_bf_write(device, BF_ADC0_HPF_EN_INFO + 2 * adc_channel, filter);
    LARK_ERROR_RETURN(err);
    if (adc_channel < 2)
    {
        err = adi_lark_hal_bf_write(device, BF_ADC01_DEC_ORDER_INFO, higher_order_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_ADC01_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
    }
    else
    {
        err = adi_lark_hal_bf_write(device, BF_ADC2_DEC_ORDER_INFO, higher_order_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_ADC2_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
    }

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool use_chan0_vol, bool change_at_zc)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ADC_HARD_VOL_INFO, hard_vol_change ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_ADC_VOL_LINK_INFO, use_chan0_vol ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_ADC_VOL_ZC_INFO, change_at_zc ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_ibias(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_ibias_e ibias)
{
    int32_t err = 0;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);
    LARK_INVALID_PARAM_RETURN(ibias > API_LARK_ADC_IBIAS_BYPASS);

    switch (adc_channel)
    {
        case 0:
            err = adi_lark_hal_bf_write(device, BF_ADC0_IBIAS_INFO, ibias);
            break;
        case 1:
            err = adi_lark_hal_bf_write(device, BF_ADC1_IBIAS_INFO, ibias);
            break;
        case 2:
            err = adi_lark_hal_bf_write(device, BF_ADC2_IBIAS_INFO, ibias);
            break;
    }
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_input_precharge_time(adi_lark_device_t *device, adi_lark_adc_precharge_time_e time)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(time > API_LARK_ADC_PRECHARGE_400MS);

    err = adi_lark_hal_bf_write(device, BF_ADC_AIN_CHRG_TIME_INFO, time);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_voice_wakeup_mode(adi_lark_device_t *device, bool voicewakeup_enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ADC_VOC_WKUP_INFO, voicewakeup_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_input_mode(adi_lark_device_t *device, uint8_t adc_channel, bool single_enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    err = adi_lark_hal_bf_write(device, BF_AIN0_INPUT_MODE_INFO + adc_channel, single_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_enable_mute(adi_lark_device_t *device, uint8_t adc_channel, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    err = adi_lark_hal_bf_write(device, BF_ADC0_MUTE_INFO + adc_channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_volume(adi_lark_device_t *device, uint8_t adc_channel, uint8_t volume)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);

    err = adi_lark_hal_bf_write(device, adc_channel + BF_ADC0_VOL_INFO, volume);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_adc_set_volume_db(adi_lark_device_t *device, uint8_t adc_channel, float vol_db)
{
    int32_t err;
    float volume;
    uint8_t vol_code;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(adc_channel >= LARK_ADC_CHANNELS);
    
    volume = (24 - vol_db) / 0.375;
    volume = (volume < 0) ? 0 : volume;
    volume = (volume > 255) ? 255 : volume;
    vol_code = (uint8_t)(volume + 0.5);
    
    err = adi_lark_adc_set_volume(device, adc_channel, vol_code);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
} 

int32_t adi_lark_adc_set_dither_level(adi_lark_device_t *device, adi_lark_adc_dither_level_e dither)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dither > API_LARK_ADC_DITHER_HALF_LSB);

    err = adi_lark_hal_bf_write(device, BF_ADC_DITHER_LEV_INFO, dither);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

/*! @} */
