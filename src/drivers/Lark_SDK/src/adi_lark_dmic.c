/*!
 * @brief     Digital Microphone Stereo Input APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */

/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_dmic_enable_channel_power_on(adi_lark_device_t *device, uint8_t channel, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);

    err = adi_lark_hal_bf_write(device, BF_DMIC0_EN_INFO + channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t reg_data;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_read(device, REG_DMIC_PWR_ADDR, &reg_data);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMIC_PWR_ADDR, enable ? (reg_data | chnl_bits_set) : (reg_data & (~chnl_bits_set)));
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_enable_clock_power_on(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DMIC_CLK_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_enable_clock1_power_on(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DMIC_CLK1_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_set_channel_out_sample_rate(adi_lark_device_t *device, uint8_t channel, adi_lark_dmic_out_sample_rate_e out_sample_rate)
{
    #ifdef LARK_SDK
    int32_t err = 0;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);
    LARK_INVALID_PARAM_RETURN(out_sample_rate > API_LARK_DMIC_OUT_SAMPLE_RATE_768KHZ);

    switch (channel)
    {
    case 0:
    case 1:
        err = adi_lark_hal_bf_write(device, BF_DMIC01_FS_INFO, out_sample_rate);
        break;
    case 2:
    case 3:
        err = adi_lark_hal_bf_write(device, BF_DMIC23_FS_INFO, out_sample_rate);
        break;
    case 4:
    case 5:
        err = adi_lark_hal_bf_write(device, BF_DMIC45_FS_INFO, out_sample_rate);
        break;
    case 6:
    case 7:
        err = adi_lark_hal_bf_write(device, BF_DMIC67_FS_INFO, out_sample_rate);
        break;

    }
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_set_channel_filter(adi_lark_device_t *device, uint8_t channel, adi_lark_dmic_dec_order_e dec_order, adi_lark_dmic_hpf_cutoff_freq_e hpf_cutoff_freq, bool high_freq_comp_enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);
    switch (channel)
    {
    case 0:
    case 1:
        err = adi_lark_hal_bf_write(device, BF_DMIC01_DEC_ORDER_INFO, dec_order);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC01_HPF_EN_INFO, hpf_cutoff_freq);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC01_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
        break;
    case 2:
    case 3:
        err = adi_lark_hal_bf_write(device, BF_DMIC23_DEC_ORDER_INFO, dec_order);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC23_HPF_EN_INFO, hpf_cutoff_freq);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC23_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
        break;
    case 4:
    case 5:
        err = adi_lark_hal_bf_write(device, BF_DMIC45_DEC_ORDER_INFO, dec_order);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC45_HPF_EN_INFO, hpf_cutoff_freq);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC45_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
        break;
    case 6:
    case 7:
        err = adi_lark_hal_bf_write(device, BF_DMIC67_DEC_ORDER_INFO, dec_order);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC67_HPF_EN_INFO, hpf_cutoff_freq);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_DMIC67_FCOMP_INFO, high_freq_comp_enable ? 1 : 0);
        LARK_ERROR_RETURN(err);
        break;
    }
    #endif


    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool use_chan0_vol, bool change_at_zc)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_DMIC_HARD_VOL_INFO, hard_vol_change ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DMIC_VOL_LINK_INFO, use_chan0_vol ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DMIC_VOL_ZC_INFO, change_at_zc ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_set_channel_volume(adi_lark_device_t *device, uint8_t channel, int32_t volume)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);
    
    if (channel < 4)
    {
        err = adi_lark_hal_bf_write(device, channel + BF_DMIC0_VOL_INFO, volume);
        LARK_ERROR_RETURN(err);
    }
    else
    {
        err = adi_lark_hal_bf_write(device, channel - 4 + BF_DMIC4_VOL_INFO, volume);
        LARK_ERROR_RETURN(err);
    }
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_set_channel_volume_db(adi_lark_device_t *device, uint8_t channel, float vol_db)
{
    #ifdef LARK_SDK
    int32_t err;
    float volume;
    uint8_t vol_code;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);
    
    volume = (24 - vol_db) / 0.375;
    volume = (volume < 0) ? 0 : volume;
    volume = (volume > 255) ? 255 : volume;
    vol_code = (uint8_t)(volume + 0.5);
    
    err = adi_lark_dmic_set_channel_volume(device, channel, vol_code);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
} 

int32_t adi_lark_dmic_enable_channel_mute(adi_lark_device_t *device, uint8_t channel, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);
    
    err = adi_lark_hal_bf_write(device, BF_DMIC0_MUTE_INFO + channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_enable_dmic1_from_mp24(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_mp_set_pin_mode(device, 24, API_LARK_MP_MODE_DMIC_DATA1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DMIC01_DAT_WIRE_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_enable_dmic3_from_mp25(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_mp_set_pin_mode(device, 25, API_LARK_MP_MODE_DMIC_DATA3);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_DMIC23_DAT_WIRE_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_sel_clk_source(adi_lark_device_t *device, uint8_t channel, uint8_t src)
{
    #ifdef LARK_SDK
    int32_t err = 0;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(channel > 7);

    switch (channel)
    {
    case 0:
    case 1:
        err = adi_lark_hal_bf_write(device, BF_DMIC01_MAP_INFO, src);
        break;
    case 2:
    case 3:
        err = adi_lark_hal_bf_write(device, BF_DMIC23_MAP_INFO, src);
        break;
    case 4:
    case 5:
        err = adi_lark_hal_bf_write(device, BF_DMIC45_MAP_INFO, src);
        break;
    case 6:
    case 7:
        err = adi_lark_hal_bf_write(device, BF_DMIC67_MAP_INFO, src);
        break;
    }
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dmic_sel_clk1_pin_src(adi_lark_device_t *device, adi_lark_dmic_clk1_pin_e pin)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DMIC_CLK1_PIN_SRC_INFO, pin);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}
/*! @} */
