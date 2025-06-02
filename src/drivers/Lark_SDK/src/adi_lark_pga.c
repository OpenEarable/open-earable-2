/*!
 * @brief     PGA APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#define LARK_PGA_CHANNELS       3

/*============= C O D E ====================*/
int32_t adi_lark_pga_enable_power_on(adi_lark_device_t *device, uint8_t pga_channel, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pga_channel >= LARK_PGA_CHANNELS);
    
    err = adi_lark_hal_bf_write(device, BF_PGA0_EN_INFO + pga_channel, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pga_set_gain(adi_lark_device_t *device, uint8_t pga_channel, uint8_t gain)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pga_channel >= LARK_PGA_CHANNELS);

    err = adi_lark_hal_reg_write(device, REG_PGA0_CTRL1_ADDR + 1 + 2 * pga_channel, gain);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pga_set_auto_slew_rate(adi_lark_device_t *device, adi_lark_pga_auto_slew_rate_e rate)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_PGA_SLEW_RATE_INFO, rate);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pga_enable_gain_link(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_PGA_GAIN_LINK_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pga_set_input_resistance(adi_lark_device_t *device, uint8_t pga_channel, bool low_resistance_enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pga_channel >= LARK_PGA_CHANNELS);
    
    err = adi_lark_hal_bf_write(device, BF_PGA0_RIN_INFO + pga_channel, low_resistance_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pga_set_power_mode(adi_lark_device_t *device, uint8_t pga_channel, bool enhanced_enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pga_channel >= LARK_PGA_CHANNELS);
    
    err = adi_lark_hal_bf_write(device, BF_PGA0_POWER_MODE_INFO + pga_channel, enhanced_enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

/*! @} */
