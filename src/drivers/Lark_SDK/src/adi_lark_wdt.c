/*!
 * @brief     Watch Dog Timer APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_wdt_get_current_value(adi_lark_device_t *device, uint16_t *cur_val)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t value;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, BF_WDT_CURR_VAL_INFO, &value);
    LARK_ERROR_RETURN(err);
    *cur_val = (uint16_t)value;
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_wdt_enable(adi_lark_device_t *device, adi_lark_wdt_mode_e mode, uint32_t ld_val, adi_lark_wdt_pre_scaler_factor_e pre_scaler_fac, bool wdt_en)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t status;
    uint32_t wdt_ctrl;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_SOC_CLKEN_WDT_INFO, 1);
    LARK_ERROR_RETURN(err);
    
    if(mode == API_LARK_WDT_MODE_FREE_RUNNING)
    {
        if(ld_val != 0x1000)
        {
             LARK_INVALID_PARAM_RETURN(ld_val != 0x1000);
        }
        else
        {
            err = adi_lark_hal_bf_write(device, BF_WDT_LD_VAL_INFO, 0x1000);
            LARK_ERROR_RETURN(err);
        }
    }
    else
    {
        err = adi_lark_hal_bf_write(device, BF_WDT_LD_VAL_INFO, ld_val);
        LARK_ERROR_RETURN(err);
    }

    err = adi_lark_hal_reg_read(device, REG_WDT_CTRL_ADDR, &status);
    LARK_ERROR_RETURN(err);
    
    if(wdt_en)
    {
        if(mode == API_LARK_WDT_MODE_FREE_RUNNING)
        {
            wdt_ctrl = (status | 0x0422) & 0xffbf;
        }
        else
        {
            wdt_ctrl = status | 0x462;
        }
    }
    else
    {
        if(mode == API_LARK_WDT_MODE_FREE_RUNNING)
        {
            wdt_ctrl = (status | 0x0420) & 0xffbd;
        }
        else
        {
            wdt_ctrl = (status | 0x0460) & 0xfffd;
        }
    }
    
    switch (pre_scaler_fac)
    {
        case API_LARK_WDT_PRE_SCALER_FACTOR_1:
            wdt_ctrl = wdt_ctrl & 0xfff3;
            break;
        case API_LARK_WDT_PRE_SCALER_FACTOR_16:
            wdt_ctrl = (wdt_ctrl | 0x0004) & 0xfff7;
            break;
        case API_LARK_WDT_PRE_SCALER_FACTOR_256:
            wdt_ctrl = (wdt_ctrl | 0x0008) & 0xfffb;
            break;
        case API_LARK_WDT_PRE_SCALER_FACTOR_4096:
            wdt_ctrl = wdt_ctrl | 0xfffc;
            break;
    }
        
    err = adi_lark_hal_reg_write(device, REG_WDT_CTRL_ADDR, wdt_ctrl);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_wdt_refresh(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_WDT_CLR_INFO, 0xcccc);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_wdt_get_irq_status(adi_lark_device_t *device, uint8_t *irq)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, BF_WDT_IRQ_INFO, &status);
    LARK_ERROR_RETURN(err);
    *irq = (status == 1) ? true : false;
    #endif

    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
#if LARK_SDK_XOS_TEST_CODE
#include <xtensa/xos.h>
#else
#include <xtensa/xtruntime.h>
#endif

uint8_t wdt_flag = 0;
void    adi_lark_wdt_handler(void *arg);

int32_t adi_lark_wdt_irq_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t irq_id;
    uint8_t  irq_status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    LARK_LOG_MSG("WDT interrupt test starts ... ");
    err = adi_lark_wdt_enable(device, API_LARK_WDT_MODE_FREE_RUNNING, 0x1000, API_LARK_WDT_PRE_SCALER_FACTOR_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_int_get_soc_irq_id(device, API_LARK_SOC_IRQ_WDT, &irq_id);
    LARK_ERROR_RETURN(err);
    #if LARK_SDK_XOS_TEST_CODE
    err = xos_register_interrupt_handler(irq_id, adi_lark_wdt_handler, NULL);
    LARK_ERROR_RETURN(err != 0 ? API_LARK_ERROR_REGISTER_ISR : API_LARK_ERROR_OK);
    xos_interrupt_enable(irq_id);
    #else
    _xtos_set_interrupt_handler_arg(irq_id, adi_lark_wdt_handler, NULL);
    _xtos_interrupt_enable(irq_id);
    #endif
    if(wdt_flag)
    {
        LARK_LOG_MSG("WDT interrupt generated.");
    }
    err = adi_lark_wdt_get_irq_status(device, &irq_status);
    LARK_ERROR_RETURN(err);
    if(irq_status)
    {
        LARK_LOG_MSG("WDT interrupt test passed.");
    }
    else
    {
       LARK_LOG_MSG("WDT interrupt is cleared.");
    }

    if (err != API_LARK_ERROR_OK)
    {
        LARK_LOG_ERR("WDT interrupt test failed.\n");
        return err;
    }
    #endif

    return API_LARK_ERROR_OK;
}

void adi_lark_wdt_handler(void *arg)
{
    wdt_flag = 1;
}
#endif

/*! @} */
