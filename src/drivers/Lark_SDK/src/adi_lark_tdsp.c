/*!
 * @brief     TDSP APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_tdsp_reset(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_TDSP_SOFT_RESET_INFO, 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_tdsp_set_alt_vec(adi_lark_device_t *device, bool enable, uint32_t alt_vec_addr)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_TDSP_ALTVEC_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_TDSP_ALTVEC_ADDR_INFO, alt_vec_addr);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_tdsp_enable_run(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_TDSP_RUN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

/*! @} */
