/*!
 * @brief     MP (Multi-purpose Pin) APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_mp_set_pin_mode(adi_lark_device_t *device, uint8_t pin_num, adi_lark_mp_pin_mode_e mode)
{
    #ifdef LARK_SDK
    int32_t err;
    uint8_t mp_mode;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    LARK_INVALID_PARAM_RETURN(pin_num > 25);
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_MP_MODE_DMIC_DATA3);
    LARK_INVALID_PARAM_RETURN((mode == API_LARK_MP_MODE_DMIC_DATA1) && (pin_num != 24));
    LARK_INVALID_PARAM_RETURN((mode == API_LARK_MP_MODE_DMIC_DATA3) && (pin_num != 25));
    LARK_INVALID_PARAM_RETURN((mode == API_LARK_MP_MODE_DMIC_CLK0) && (pin_num != 8) && (pin_num != 9) && (pin_num != 10) && (pin_num != 22));
    LARK_INVALID_PARAM_RETURN((mode == API_LARK_MP_MODE_DMIC_CLK1) && (pin_num != 7) && (pin_num != 11) && (pin_num != 17) && (pin_num != 22));
    
    mp_mode = mode;
    if ((mode == API_LARK_MP_MODE_DMIC_DATA1) && (pin_num == 24))
    {
        mp_mode = 12;
    }
    else if ((mode == API_LARK_MP_MODE_DMIC_DATA3) && (pin_num == 25))
    {
        mp_mode = 12;
    }
    else if ((mode == API_LARK_MP_MODE_DMIC_CLK0) && ((pin_num == 8) || (pin_num == 9) || (pin_num == 10) || (pin_num == 22)))
    {
        mp_mode = 4;
    }
    
    err = adi_lark_hal_bf_write(device, (pin_num >> 1) + BF_MP0_MODE_INFO + (((pin_num & 0x1) == 0) ? 0 : 4), mp_mode);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_mp_set_output_level(adi_lark_device_t *device, uint8_t pin_num, uint8_t level)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pin_num > 25);
    LARK_INVALID_PARAM_RETURN(level   > 1);
    
    err = adi_lark_hal_bf_write(device, (pin_num >> 3) + BF_GPIO0_OUT_INFO + (pin_num & 0x7), level);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_mp_get_input_level(adi_lark_device_t *device, uint8_t pin_num, uint8_t *level)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t gpi_level;
    LARK_NULL_POINTER_RETURN(device);
    LARK_INVALID_PARAM_RETURN(pin_num > 25);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, (pin_num >> 3) + BF_GPIO0_IN_INFO + (pin_num & 0x7), &gpi_level);
    LARK_ERROR_RETURN(err);
    *level = gpi_level & 0x01;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_mp_set_gpi_debounce(adi_lark_device_t *device, adi_lark_mp_input_debounce_e debounce)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(debounce > API_LARK_MP_DEBOUNCE_20MS);
    
    err = adi_lark_hal_bf_write(device, BF_GPI_DB_INFO, debounce);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_mp_set_irq_input_debounce(adi_lark_device_t *device, adi_lark_mp_input_debounce_e debounce)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(debounce > API_LARK_MP_DEBOUNCE_20MS);
    
    err = adi_lark_hal_bf_write(device, BF_IRQ_IN_DB_INFO, debounce);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

/*! @} */
