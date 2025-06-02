/*!
 * @brief     Serial Audio Port APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_sap_init_config_as_i2sdefault(adi_lark_device_t *device, adi_lark_sap_config_t *config_ptr)
{
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(config_ptr);
    config_ptr->sai_mode = API_LARK_SAP_CTRL_SAI_MODE_STEREO;
    config_ptr->data_format = API_LARK_SAP_CTRL_DATA_FORMAT_I2S;
    config_ptr->slot_width = API_LARK_SAP_CTRL_SLOT_WIDTH_32;
    config_ptr->tri_state = API_LARK_SAP_CTRL_TRI_STATE_ENABLE;
    config_ptr->data_width = API_LARK_SAP_CTRL_DATA_WIDTH_24;
    config_ptr->bclk_src = API_LARK_SAP_CTRL_BCLK_SRC_SLAVE;    /*API_LARK_SAP_CTRL_BCLK_SRC_3072K; Slave for the sound card*/
    config_ptr->bclk_pol = API_LARK_SAP_CTRL_BCLK_POL_NEG;      /*API_LARK_SAP_CTRL_BCLK_POL_NEG; POS for the sound card*/
    config_ptr->lrclk_src = API_LARK_SAP_CTRL_LRCLK_SRC_SLAVE;  /*API_LARK_SAP_CTRL_LRCLK_SRC_48K; Slave for the sound card*/
    config_ptr->lrclk_pol = API_LARK_SAP_CTRL_LRCLK_POL_NORM;
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_sap_config_all(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_config_t *config_ptr)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(config_ptr);
    
    err = adi_lark_sap_set_sai_mode(device, port_number, config_ptr->sai_mode);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_data_format(device, port_number, config_ptr->data_format);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_slot_width(device, port_number, config_ptr->slot_width);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_tri_state(device, port_number, config_ptr->tri_state);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_bclk_src(device, port_number, config_ptr->bclk_src);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_bclk_pol(device, port_number, config_ptr->bclk_pol);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_lrclk_src(device, port_number, config_ptr->lrclk_src);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_set_lrclk_pol(device, port_number, config_ptr->lrclk_pol);
    LARK_ERROR_RETURN(err);
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_sap_set_sai_mode(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_sai_mode_e sai_mode)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_SAI_MODE_INFO, sai_mode);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_SAI_MODE_INFO, sai_mode);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_data_format(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_data_format_e data_format)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_DATA_FORMAT_INFO, data_format);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_DATA_FORMAT_INFO, data_format);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_slot_width(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_slot_width_e slot_width)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_SLOT_WIDTH_INFO, slot_width);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_SLOT_WIDTH_INFO, slot_width);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_tri_state(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_tristate_e tri_state)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_TRI_STATE_INFO, tri_state);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_TRI_STATE_INFO, tri_state);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_bclk_src(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_bclk_src_e bclk_src)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_BCLK_SRC_INFO, bclk_src);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_BCLK_SRC_INFO, bclk_src);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_bclk_pol(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_bclk_polarity_e bclk_pol)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_BCLK_POL_INFO, bclk_pol);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_BCLK_POL_INFO, bclk_pol);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_lrclk_src(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_lrclk_src_e lrclk_src)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_LRCLK_SRC_INFO, lrclk_src);
    #ifdef  LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_LRCLK_SRC_INFO, lrclk_src);
    #endif 
    return err;
}

int32_t adi_lark_sap_set_lrclk_pol(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_lrclk_polarity_e lrclk_pol)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_LRCLK_POL_INFO, lrclk_pol);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_LRCLK_POL_INFO, lrclk_pol);
    #endif 
    return err;
}

int32_t adi_lark_sap_select_out_route(adi_lark_device_t *device, uint32_t port_number, uint32_t slot_id, adi_lark_sap_output_route_from_e route_from)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN((port_number != 0) || (route_from > API_LARK_SAP_OUT_ROUTE_COUNT) || (slot_id > 15));
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN((port_number > 1) || (route_from > API_LARK_SAP_OUT_ROUTE_COUNT) || (slot_id > 15));
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_reg_write(device, REG_SPT0_ROUTE0_ADDR + slot_id, route_from);
    #ifdef  LARK_SDK
    else
        err = adi_lark_hal_reg_write(device, REG_SPT1_ROUTE0_ADDR + slot_id, route_from);
    #endif 
    return err;
}

int32_t adi_lark_sap_enable_chnl_input(adi_lark_device_t *device, uint32_t port_number, bool enable_flag)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_IN_EN_INFO, enable_flag ? 1 : 0);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_IN_EN_INFO, enable_flag ? 1 : 0);
    #endif 
    return err;
}

int32_t adi_lark_sap_enable_chnl_output(adi_lark_device_t *device, uint32_t port_number, bool enable_flag)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(port_number != 0);
    #endif
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(port_number > 1);
    #endif 

    if(port_number == 0)
        err = adi_lark_hal_bf_write(device, BF_SPT0_OUT_EN_INFO, enable_flag ? 1 : 0);
    #ifdef LARK_SDK
    else
        err = adi_lark_hal_bf_write(device, BF_SPT1_OUT_EN_INFO, enable_flag ? 1 : 0);
    #endif 
    return err;
}

/*! @} */
