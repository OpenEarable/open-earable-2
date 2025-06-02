/*!
 * @brief     General Purpose Timer APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_gpt_set_mode(adi_lark_device_t *device, adi_lark_gpt_run_mode_e run_mode, adi_lark_gpt_count_mode_e count_mode)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_GPT_MODE_INFO, run_mode);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_GPT_UP_DOWN_INFO, count_mode);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_enable(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_GPT_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_set_load_value(adi_lark_device_t *device, uint16_t value)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_GPT_LD_VAL_INFO, value);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_get_current_value(adi_lark_device_t *device, uint16_t *value)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t current_val;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(value);
    
    err = adi_lark_hal_bf_read(device, BF_GPT_CURR_VAL_INFO, &current_val);
    LARK_ERROR_RETURN(err);
    *value = (uint16_t)current_val;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_select_clk_source(adi_lark_device_t *device, adi_lark_gpt_clk_source_e source)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(source > API_LARK_GPT_CLOCK_CLK32K);

    err = adi_lark_hal_bf_write(device, BF_GPT_CLK_SEL_INFO, source << 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_set_pre_scaler(adi_lark_device_t *device, adi_lark_gpt_pre_scaler_e div)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(div > API_LARK_GPT_PRE_SCALER_32768);

    err = adi_lark_hal_bf_write(device, BF_GPT_PRE_SCALER_INFO, div);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_clr_timeout_int(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_GPT_INT_TMOUT_INFO, 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_get_timeout_status(adi_lark_device_t *device, bool *timeout)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(timeout);
    
    err = adi_lark_hal_bf_read(device, BF_GPT_STATUS_TMOUT_INFO, &status);
    LARK_ERROR_RETURN(err);
    *timeout = (status == 1) ? true : false;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_get_busy_status(adi_lark_device_t *device, bool *busy)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(busy);
    
    err = adi_lark_hal_bf_read(device, BF_GPT_STATUS_CTRL_BUSY_INFO, &status);
    LARK_ERROR_RETURN(err);
    *busy = (status == 1) ? true : false;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_gpt_get_clrdone_status(adi_lark_device_t *device, bool *clrdone)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(clrdone);
    
    err = adi_lark_hal_bf_read(device, BF_GPT_STATUS_INT_CLR_DONE_INFO, &status);
    LARK_ERROR_RETURN(err);
    *clrdone = (status == 1) ? true : false;
    #endif
    
    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
#if LARK_SDK_XOS_TEST_CODE
#include <xtensa/xos.h>
#else
#include <xtensa/xtruntime.h>
#endif

uint8_t gpio_level = 0;
void    adi_lark_gpt_handler(void *arg)
{
    *(uint32_t*)REG_GPT_INT_CLR_ADDR = 1;

    if (gpio_level == 0)
    {
        gpio_level = 1;
        *(uint32_t*)REG_MP_GPIO_CTRL1_ADDR |= 0x80;
    }
    else
    {
        gpio_level = 0;
        *(uint32_t*)REG_MP_GPIO_CTRL1_ADDR &= 0x7F;
    }
}

int32_t adi_lark_gpt_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq)
{
    int32_t err;
    uint16_t current_value;
    uint32_t irq_id;

    err = adi_lark_gpt_enable(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_gpt_set_mode(device, API_LARK_GPT_MODE_PERIODIC, API_LARK_GPT_COUNT_MODE_DOWN);
    LARK_ERROR_RETURN(err);
    err = adi_lark_gpt_select_clk_source(device, API_LARK_GPT_CLOCK_CLK32K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_gpt_set_pre_scaler(device, API_LARK_GPT_PRE_SCALER_1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_gpt_set_load_value(device, 0x4000);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_mp_set_pin_mode(device, 7, API_LARK_MP_MODE_GPO_REG);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_int_get_soc_irq_id(device, API_LARK_SOC_IRQ_GPT, &irq_id);
    LARK_ERROR_RETURN(err);
    #if LARK_SDK_XOS_TEST_CODE
    err = xos_register_interrupt_handler(irq_id, adi_lark_gpt_handler, NULL);
    LARK_ERROR_RETURN(err != 0 ? API_LARK_ERROR_REGISTER_ISR : API_LARK_ERROR_OK);
    xos_interrupt_enable(irq_id);
    #else
    _xtos_set_interrupt_handler_arg(irq_id, adi_lark_gpt_handler, NULL);
    _xtos_interrupt_enable(irq_id);
    #endif
    err = adi_lark_gpt_enable(device, true);
    LARK_ERROR_RETURN(err);
    
    while (1);
    return API_LARK_ERROR_OK;
}
#endif

/*! @} */
