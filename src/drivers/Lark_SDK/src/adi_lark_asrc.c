/*!
 * @brief     Async Sample Rate Converter (ASRC) APIs Implementation
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
#define LARK_ASRCI_CHANNELS         4
#define LARK_ASRCO_CHANNELS         4
#define LARK_SAP_PORT_NUMBER        2
#endif
#ifdef  LARK_LITE_SDK
#define LARK_ASRCI_CHANNELS         1
#define LARK_ASRCO_CHANNELS         3
#define LARK_SAP_PORT_NUMBER        1
#endif

/*============= C O D E ====================*/
int32_t adi_lark_asrc_enable_chnl_power_on(adi_lark_device_t *device, adi_lark_asrc_chnl_e chnl_no, bool enable_flag)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(chnl_no >= API_LARK_ASRC_CHNL_COUNT);
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN((chnl_no >= 1 && chnl_no <= 3) || chnl_no == 7);
    #endif

    err = adi_lark_hal_bf_write(device, BF_ASRCI0_EN_INFO + chnl_no, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable_flag)
{
    int32_t  err;
    uint32_t reg_data;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_read(device, REG_ASRC_PWR_ADDR, &reg_data);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_ASRC_PWR_ADDR, enable_flag ? (reg_data | chnl_bits_set) : (reg_data & (~chnl_bits_set)));
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_select_asrci_route(adi_lark_device_t *device, uint8_t asrci_chnl, uint8_t route_from)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(asrci_chnl >= LARK_ASRCI_CHANNELS);
    LARK_INVALID_PARAM_RETURN(route_from >= 16);

    err = adi_lark_hal_bf_write(device, asrci_chnl / 2 + BF_ASRCI0_ROUTE_INFO + (asrci_chnl % 2) * 4, route_from);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_select_asrco_route(adi_lark_device_t *device, uint8_t asrco_chnl, adi_lark_ascro_route_chnl_e route_from)
{
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(asrco_chnl >= LARK_ASRCO_CHANNELS);
    LARK_INVALID_PARAM_RETURN(route_from >= API_LARK_ASRCO_ROUTE_COUNT);

    err = adi_lark_hal_reg_write(device, REG_ASRCO_ROUTE0_ADDR + asrco_chnl, route_from);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_select_asrci_fs(adi_lark_device_t *device, adi_lark_asrc_fs_e out_fs)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(out_fs > API_LARK_ASRC_FS_192K || out_fs < API_LARK_ASRC_FS_12K);

    err = adi_lark_hal_bf_write(device, BF_ASRCI_OUT_FS_INFO, out_fs);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_select_asrco_fs(adi_lark_device_t *device, adi_lark_asrc_fs_e in_fs)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(in_fs > API_LARK_ASRC_FS_192K || in_fs < API_LARK_ASRC_FS_12K);

    err = adi_lark_hal_bf_write(device, BF_ASRCO_IN_FS_INFO, in_fs);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_select_asrci_sai(adi_lark_device_t *device, uint8_t sai_port)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(sai_port >= LARK_SAP_PORT_NUMBER);

    err = adi_lark_hal_bf_write(device, BF_ASRCI_SOURCE_INFO, sai_port);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_select_asrco_sai(adi_lark_device_t *device, uint8_t sai_port)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(sai_port > LARK_SAP_PORT_NUMBER);

    err = adi_lark_hal_bf_write(device, BF_ASRCO_SAI_SEL_INFO, sai_port);
    LARK_ERROR_RETURN(err);
    #endif /* LARK_SDK */

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_set_asrci_power_level(adi_lark_device_t *device, uint8_t power_level)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    switch(power_level)
    {
        case 0:
            err = adi_lark_hal_bf_write(device, BF_ASRCI_LPM_II_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 1:
            err = adi_lark_hal_bf_write(device, BF_ASRCI_LPM_INFO, 1);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_bf_write(device, BF_ASRCI_LPM_II_INFO, 0);
            LARK_ERROR_RETURN(err);
            break;
        default:
            err = adi_lark_hal_bf_write(device, BF_ASRCI_LPM_INFO, 0);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_bf_write(device, BF_ASRCI_LPM_II_INFO, 0);
            LARK_ERROR_RETURN(err);
            break;
    }

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_set_asrco_power_level(adi_lark_device_t *device, uint8_t power_level)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    switch(power_level)
    {
        case 0:
            err = adi_lark_hal_bf_write(device, BF_ASRCO_LPM_II_INFO, 1);           /*LPM II has higer priority over LPM. Don't need set LPM when LPM II is 1*/
            LARK_ERROR_RETURN(err);
            break;
        case 1:
            err = adi_lark_hal_bf_write(device, BF_ASRCO_LPM_INFO, 1);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_bf_write(device, BF_ASRCO_LPM_II_INFO, 0);
            LARK_ERROR_RETURN(err);
            break;
        default:
            err = adi_lark_hal_bf_write(device, BF_ASRCO_LPM_INFO, 0);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_bf_write(device, BF_ASRCO_LPM_II_INFO, 0);
            LARK_ERROR_RETURN(err);
            break;
    }

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_asrc_enable_asrci_vfilter(adi_lark_device_t *device, bool enable_flag)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ASRCI_VFILT_INFO, enable_flag? 1 : 0);

    return err;
}

int32_t adi_lark_asrc_enable_asrci_addi_vfilter(adi_lark_device_t *device, bool enable_flag)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ASRCI_MORE_FILT_INFO, enable_flag ? 1 : 0);

    return err;
}

int32_t adi_lark_asrc_enable_asrco_vfilter(adi_lark_device_t *device, bool enable_flag)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ASRCO_VFILT_INFO, enable_flag ? 1 : 0);

    return err;
}

int32_t adi_lark_asrc_enable_asrco_addi_vfilter(adi_lark_device_t *device, bool enable_flag)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ASRCO_MORE_FILT_INFO, enable_flag ? 1 : 0);

    return err;
}

/*! @} */
