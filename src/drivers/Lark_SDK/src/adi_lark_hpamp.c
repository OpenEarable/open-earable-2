/*!
 * @brief     Headphone Amp APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_hpamp_set_output_mode(adi_lark_device_t *device, adi_lark_hpamp_mode_e mode)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_HPAMP_LINE_OUT);

    err = adi_lark_hal_bf_write(device, BF_HP0_MODE_INFO, mode);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_enable_lvm_mode(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_enable_lvm_mode_cm(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_CM_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_set_voltage_switch_mode(adi_lark_device_t *device, adi_lark_hpamp_voltage_switch_mode_e mode)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_HPAMP_HOT_SWITCH);

    switch (mode)
    {
    case API_LARK_HPAMP_AUTO_SWITCH:
        err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_AUTOSW_MODE_INFO, 1);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, 0x4000C064, 0x00000100, 0);
        LARK_ERROR_RETURN(err);
        break;
    case API_LARK_HPAMP_MANNUAL_SWITCH:
        err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_AUTOSW_MODE_INFO, 0);
        LARK_ERROR_RETURN(err);
        break;
    case API_LARK_HPAMP_HOT_SWITCH:
        err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_AUTOSW_MODE_INFO, 1);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, 0x4000C064, 0x00000100, 1);
        LARK_ERROR_RETURN(err);
        break;
    }

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_select_voltage_mode(adi_lark_device_t *device, adi_lark_hpamp_voltage_mode_e mode)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_HPAMP_SWITCH_TO_LVM);

    err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_SWITCH_INFO, mode);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_set_lvmmode_delay(adi_lark_device_t *device, uint8_t delay)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(delay > 7);

    err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_AUTO_DLY_INFO, delay);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_set_lvmmode_cm_delay(adi_lark_device_t *device, uint8_t delay)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(delay > 3);

    err = adi_lark_hal_bf_write(device, BF_HP_LVMODE_CM_DLY_INFO, delay);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_enable_ldo(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_HPLDO_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_set_ldo_bypass(adi_lark_device_t *device, bool bypass)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_HPLDO_BYPASS_INFO, bypass ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hpamp_set_ldo_vout(adi_lark_device_t *device, adi_lark_hpamp_ldo_vout_e vout)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(vout > API_LARK_HPAMP_LDO_OUT_1P05);

    err = adi_lark_hal_bf_write(device, BF_HPLDO_VOUT_SET_INFO, vout);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

/*! @} */
