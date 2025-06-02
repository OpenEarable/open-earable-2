/*!
 * @brief     EQ APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_eq_select_route(adi_lark_device_t *device, adi_lark_eq_route_e route)
{
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_EQ_ROUTE_INFO, route);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_eq_enable_run(adi_lark_device_t *device, bool enable)
{
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_EQ_RUN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_eq_clear_param_ram(adi_lark_device_t *device)
{
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_EQ_CLEAR_INFO, 1);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_eq_get_param_ram_clear_status(adi_lark_device_t *device, uint8_t *clear_done)
{
    int32_t  err;
    uint32_t done;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_read(device, BF_IRQ1_EQ_CLEAR_DONE_INFO, &done);
    LARK_ERROR_RETURN(err);
    *clear_done = done & 0x01;

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_eq_sel_param_ram(adi_lark_device_t *device, uint8_t bank_id)
{
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(bank_id > 1);
    
    err = adi_lark_hal_bf_write(device, BF_EQ_BANK_SEL_INFO, bank_id);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

/*! @} */
