/*!
 * @brief     Decimator APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#ifdef  LARK_SDK
#define LARK_FDEC_CHANNELS  8
#endif
#ifdef  LARK_LITE_SDK
#define LARK_FDEC_CHANNELS  4
#endif

/*============= C O D E ====================*/
int32_t adi_lark_fdec_enable_chnl_power_on(adi_lark_device_t *device, uint8_t chnl_no, bool enable_flag)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(chnl_no >= LARK_FDEC_CHANNELS);

    err = adi_lark_hal_bf_write(device, REG_FDEC_PWR_ADDR, (1 << 8) + chnl_no, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdec_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable_flag)
{
    int32_t  err;
    uint32_t reg_data;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_read(device, REG_FDEC_PWR_ADDR, &reg_data);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_FDEC_PWR_ADDR, enable_flag ? (reg_data | chnl_bits_set) : (reg_data & (~chnl_bits_set)));
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdec_select_pair_chnls_fs(adi_lark_device_t *device, adi_lark_fdec_pair_chnl_e pair_chnls, adi_lark_fdec_in_fs_e in_fs, adi_lark_fdec_out_fs_e out_fs)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pair_chnls >= (LARK_FDEC_CHANNELS / 2));
    LARK_INVALID_PARAM_RETURN(in_fs  > API_LARK_FDEC_IN_FS_768K || in_fs  < API_LARK_FDEC_IN_FS_24K);
    LARK_INVALID_PARAM_RETURN(out_fs > API_LARK_FDEC_OUT_FS_16K || out_fs < API_LARK_FDEC_OUT_FS_12K);

    err = adi_lark_hal_reg_write(device, REG_FDEC_CTRL1_ADDR + pair_chnls, (out_fs << 4) + in_fs);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_fdec_select_chnl_route(adi_lark_device_t *device, uint8_t dec_chnl, adi_lark_fdec_route_chnl_e route_chnl)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dec_chnl >= LARK_FDEC_CHANNELS);
    LARK_INVALID_PARAM_RETURN(route_chnl >= API_LARK_FDEC_ROUTE_COUNT);
    
    err = adi_lark_hal_reg_write(device, REG_FDEC_ROUTE0_ADDR + dec_chnl, route_chnl);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

/*! @} */
