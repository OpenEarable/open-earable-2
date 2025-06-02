/*!
 * @brief     FastDSP APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_fdsp_enable_power_on(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_FDSP_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_enable_run(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_FDSP_RUN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_select_param_bank(adi_lark_device_t *device, adi_lark_fdsp_param_bank_e sel)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(sel > API_LARK_FDSP_PARAM_BANK_C);

    err = adi_lark_hal_bf_write(device, BF_FDSP_BANK_SEL_INFO, sel);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_config_bank_switch_settings(adi_lark_device_t *device, adi_lark_fdsp_param_switch_mode_e mode, adi_lark_fdsp_ramp_rate_e rate, bool zero, uint8_t stop_point)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_FDSP_PARAM_SWICTH_INSTANT);
    LARK_INVALID_PARAM_RETURN(rate > API_LARK_FDSP_RAMP_RATE_2SEC);
    LARK_INVALID_PARAM_RETURN(stop_point > 63);

    err = adi_lark_hal_bf_write(device, BF_FDSP_RAMP_MODE_INFO, mode);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_RAMP_RATE_INFO, rate);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_ZERO_STATE_INFO, zero ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_LAMBDA_INFO, stop_point);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_copy_param_bank(adi_lark_device_t *device, adi_lark_fdsp_copy_param_bank_e mask)
{
    int32_t  err;
    uint32_t sel, loop_count = 0;
    uint32_t  copy_done;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mask > 5);

    err = adi_lark_hal_bf_read(device, BF_FDSP_BANK_SEL_INFO, &sel);
    LARK_ERROR_RETURN(err);
    
    /* Cannot copy to active bank */
    if (((sel == API_LARK_FDSP_PARAM_BANK_A) && ((mask == API_LARK_FDSP_COPY_PARAM_BANK_B2A) || (mask == API_LARK_FDSP_COPY_PARAM_BANK_C2A))) || 
        ((sel == API_LARK_FDSP_PARAM_BANK_B) && ((mask == API_LARK_FDSP_COPY_PARAM_BANK_A2B) || (mask == API_LARK_FDSP_COPY_PARAM_BANK_C2B))) || 
        ((sel == API_LARK_FDSP_PARAM_BANK_C) && ((mask == API_LARK_FDSP_COPY_PARAM_BANK_A2C) || (mask == API_LARK_FDSP_COPY_PARAM_BANK_B2C))))
    {
        return API_LARK_ERROR_INVALID_PARAM;
    }
    
    err = adi_lark_hal_reg_write(device, REG_FDSP_CTRL3_ADDR, 1 << mask);
    LARK_ERROR_RETURN(err);
    
    do
    {
        err = adi_lark_hal_bf_read(device, BF_FDSP_BANK_COPY_DONE_INFO, &copy_done);
        LARK_ERROR_RETURN(err);
    } while((copy_done == 0) && (loop_count++ < 1000));

    err = adi_lark_hal_bf_write(device, BF_FDSP_COPY_DONE_CLEAR_INFO, 1);
    LARK_ERROR_RETURN(err);
    
    return err;
}

int32_t adi_lark_fdsp_set_rate(adi_lark_device_t *device, adi_lark_fdsp_rate_src_e source,  uint16_t div)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(source > API_LARK_FDSP_RATE_SRC_FIXED);

    err = adi_lark_hal_bf_write(device, BF_FDSP_RATE_SOURCE_INFO, source);
    LARK_ERROR_RETURN(err);
     err = adi_lark_hal_bf_write(device, BF_FDSP_RATE_DIV_INFO, div);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_set_modulo_n(adi_lark_device_t *device, uint8_t n)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(n > 63);

    err = adi_lark_hal_bf_write(device, BF_FDSP_MOD_N_INFO, n);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_safeload(adi_lark_device_t *device, uint8_t inst_index, uint32_t param[5])
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(inst_index > 95);

    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_ADDR_INFO, inst_index);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_P0_INFO, param[0]);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_P1_INFO, param[1]);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_P2_INFO, param[2]);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_P3_INFO, param[3]);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_P4_INFO, param[4]);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_UPDATE_INFO, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_FDSP_SL_UPDATE_INFO, 1);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_set_mode(adi_lark_device_t *device, adi_lark_fdsp_mode_e mode)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_FDSP_MODE_INFO, mode);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_get_mode(adi_lark_device_t *device, uint8_t *mode)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t data;
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(mode);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_read(device, BF_FDSP_MODE_INFO, &data);
    LARK_ERROR_RETURN(err);
    *mode = data;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdsp_get_cur_lambda(adi_lark_device_t *device, uint8_t *lambda)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t data;
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(lambda);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_read(device, BF_FDSP_CURRENT_LAMBDA_INFO, &data);
    LARK_ERROR_RETURN(err);
    *lambda = (uint8_t)data;
    #endif
    return API_LARK_ERROR_OK;
}

/*! @} */
