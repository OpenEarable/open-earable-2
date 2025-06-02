/*!
 * @brief     Power Management Unit APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_pmu_get_chip_power_mode(adi_lark_device_t *device, adi_lark_pwr_mode_e *mode)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t pmu_mode;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, BF_POWER_MODE_STA_INFO, &pmu_mode);
    LARK_ERROR_RETURN(err);
    *mode = pmu_mode & 0x03;
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_set_chip_power_mode(adi_lark_device_t *device, adi_lark_pwr_mode_e mode)
{
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_PWR_MODE_ACTIVE);
    
    #ifdef LARK_SDK
    err = adi_lark_hal_bf_write(device, BF_PWR_MODE_INFO, mode);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_hal_bf_write(device, BF_POWER_EN_INFO, (mode == API_LARK_PWR_MODE_ACTIVE) ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_enable_master_block(adi_lark_device_t *device, bool enable_flag)
{
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_MASTER_BLOCK_EN_INFO, enable_flag ? 1:0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_set_memory_voltage_mode(adi_lark_device_t *device, adi_lark_pmu_memory_voltage_mode_e mode)
{
    #ifdef LARK_SDK
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_MEM_VOLT_MODE_INFO, mode);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_get_memory_voltage_mode(adi_lark_device_t *device, adi_lark_pmu_memory_voltage_mode_e *mode)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t vol_mode;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, BF_MEM_VOLT_STA_INFO, &vol_mode);
    LARK_ERROR_RETURN(err);
    *mode = vol_mode & 0x03;
    #endif 
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_retain_memory_power(adi_lark_device_t *device, adi_lark_pmu_mem_retain_e memory_mask)
{
    #ifdef LARK_SDK
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_reg_write(device, REG_MEM_RETAIN_ADDR, memory_mask);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_enable_cm_pin_fast_charge(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_CM_STARTUP_OVER_INFO, enable ? 0 : 1);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_set_dldo_output_voltage(adi_lark_device_t *device, adi_lark_pmu_dldo_output_e output)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(output > API_LARK_PMU_DLDO_OUTPUT_1P1V);
    
    err = adi_lark_hal_bf_write(device, BF_DLDO_CTRL_INFO, output);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_enable_irq_wakeup(adi_lark_device_t *device, adi_lark_pmu_wakeup_irq_e enable_mask)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_IRQ_WKUP_EN_INFO, (uint8_t)enable_mask);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_get_irq_wakeup_en(adi_lark_device_t *device, uint32_t *enable_mask)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, BF_IRQ_WKUP_EN_INFO, enable_mask);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_pmu_enable_dlycnt_byp(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_STARTUP_DLYCNT_BYP_INFO, enable? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

/*! @} */
