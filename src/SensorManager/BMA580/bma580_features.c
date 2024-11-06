/**
* Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bma580_features.c
* @date       2024-07-29
* @version    v4.2.0
*
*/

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/

#include "bma580_features.h"

/******************************************************************************/
/*********************** Static function declarations *************************/
/******************************************************************************/

/*!
 * @brief Internal API to verify the validity of the primary device handle which
 *        is passed as argument.
 *
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 * @retval BMA5_OK -> device handle is valid
 * @retval BMA5_E_NULL_PTR -> Null pointer error
 */
static int8_t verify_handle(const struct bma5_dev *dev);

/******************************************************************************/
/*********************** User function definitions ****************************/
/******************************************************************************/

/*!
 *  @brief This API is the entry point.
 *  Call this API before using all other APIs.
 *  This API reads the chip-id of the sensor which is the first step to
 *  verify the sensor and also it configures the read mechanism of SPI and
 *  I2C interface.
 */
int8_t bma580_init(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to get chip id */
    uint8_t data = 0;
    uint8_t sensor_health_status = 0;

    /* Null-pointer check */
    result = verify_handle(dev);

    if (result == BMA5_OK)
    {
        dev->chip_id = 0;

        if (dev->intf == BMA5_SPI_INTF)
        {
            dev->dummy_byte = 1;
        }
        else
        {
            dev->dummy_byte = 0;
        }

        /* Dummy read results in NACK. Hence result is not checked */
        (void) bma5_get_regs(BMA580_REG_CHIP_ID, &data, 1, dev);

        result = bma5_get_regs(BMA580_REG_CHIP_ID, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Assign Chip Id */
            dev->chip_id = data;
        }

        if (dev->chip_id != BMA580_CHIP_ID)
        {
            result = BMA5_E_DEV_NOT_FOUND;
        }

        /* Bring sensor from Suspend Mode to Normal Mode */
        if (result == BMA5_OK)
        {
            result = bma5_set_cmd_suspend(BMA5_SUSPEND_DISABLE, dev);

            if (result != BMA5_OK)
            {
                return result;
            }

            /* Wait until the sensor wakesup and internal health status is good */
            for (;;)
            {
                result = bma5_get_health_status(&sensor_health_status, dev);

                if (result != BMA5_OK)
                {
                    break;
                }

                if (sensor_health_status == BMA5_SENSOR_HEALTH_STATUS_MSK)
                {
                    break;
                }
            }
        }
    }
    else
    {
        result = BMA5_E_NULL_PTR;
    }

    return result;
}

/*!
 * @brief This API sets feature axis configurations
 */
int8_t bma580_set_feature_axis_config(const struct bma580_feat_axis *feat_axis, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA580_BASE_ADDR_GENERAL_SETTINGS;

    /* Array to store feature axis data */
    uint8_t feat_axis_data[4] = { 0 };

    if (feat_axis == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_axis_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA580_FEAT_AXIS_EX, feat_axis->feat_axis_ex);

                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA580_FEAT_X_INV, feat_axis->feat_x_inv);

                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA580_FEAT_Y_INV, feat_axis->feat_y_inv);

                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA580_FEAT_Z_INV, feat_axis->feat_z_inv);

                feat_axis_data[0] = feat_axis_data[2];
                feat_axis_data[1] = feat_axis_data[3];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, feat_axis_data, 2, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets feature axis configurations
 */
int8_t bma580_get_feature_axis_config(struct bma580_feat_axis *feat_axis, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA580_BASE_ADDR_GENERAL_SETTINGS;

    /* Array to store feature axis data */
    uint8_t feat_axis_data[4] = { 0 };

    if (feat_axis == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_axis_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_axis->feat_axis_ex = BMA5_GET_BITS(feat_axis_data[2], BMA580_FEAT_AXIS_EX);

                feat_axis->feat_x_inv = BMA5_GET_BITS(feat_axis_data[2], BMA580_FEAT_X_INV);

                feat_axis->feat_y_inv = BMA5_GET_BITS(feat_axis_data[2], BMA580_FEAT_Y_INV);

                feat_axis->feat_z_inv = BMA5_GET_BITS(feat_axis_data[2], BMA580_FEAT_Z_INV);
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets feature configuration error status
 */
int8_t bma580_set_feat_conf_err(const struct bma580_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature config error */
    uint8_t data = BMA580_BASE_ADDR_FEAT_CONF_ERR;

    /* Array to store feature config error data */
    uint8_t feat_conf_err_data[4] = { 0 };

    if (feat_conf_err == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature config error base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_conf_err_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_GEN_INT1_CONF_ERR,
                                                      feat_conf_err->gen_int1_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_GEN_INT2_CONF_ERR,
                                                      feat_conf_err->gen_int2_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_GEN_INT3_CONF_ERR,
                                                      feat_conf_err->gen_int3_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_ACC_FOC_CONF_ERR,
                                                      feat_conf_err->acc_foc_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_TAP_CONF_ERR,
                                                      feat_conf_err->tap_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_VAD_CONF_ERR,
                                                      feat_conf_err->vad_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA580_SELF_WAKE_UP_ERR,
                                                      feat_conf_err->self_wake_up_err);

                feat_conf_err_data[0] = feat_conf_err_data[2];
                feat_conf_err_data[1] = feat_conf_err_data[3];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, feat_conf_err_data, 2, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets feature configuration error status
 */
int8_t bma580_get_feat_conf_err(struct bma580_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature config error */
    uint8_t data = BMA580_BASE_ADDR_FEAT_CONF_ERR;

    /* Array to store feature config error data */
    uint8_t feat_conf_err_data[4] = { 0 };

    if (feat_conf_err == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature config error base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_conf_err_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_conf_err->gen_int1_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_GEN_INT1_CONF_ERR);
                feat_conf_err->gen_int2_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_GEN_INT2_CONF_ERR);
                feat_conf_err->gen_int3_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_GEN_INT3_CONF_ERR);
                feat_conf_err->acc_foc_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_ACC_FOC_CONF_ERR);
                feat_conf_err->tap_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_TAP_CONF_ERR);
                feat_conf_err->vad_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_VAD_CONF_ERR);
                feat_conf_err->self_wake_up_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA580_SELF_WAKE_UP_ERR);

            }
        }
    }

    return result;
}

/*!
 * @brief This API gets default generic interrupt configurations for Generic Interrupt 1 and 2
 */
int8_t bma580_get_default_generic_int_config(struct bma580_generic_interrupt_types *gen_int,
                                             uint8_t n_ints,
                                             struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA580_GEN_INT_1:
                    data = BMA580_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA580_GEN_INT_2:
                    data = BMA580_BASE_ADDR_GENERIC_INT2;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                gen_int[loop].gen_int.comb_sel = BMA580_GEN_INT_COMB_SEL_LOGICAL_OR;

                gen_int[loop].gen_int.axis_sel = BMA580_AXIS_SEL_DEFAULT;

                gen_int[loop].gen_int.criterion_sel = BMA580_GEN_INT_CRI_SEL_ACT;

                gen_int[loop].gen_int.acc_ref_up = BMA580_GEN_INT_ACC_REF_UP_ON_ALWAYS;

                gen_int[loop].gen_int.duration = BMA580_DURATION_DEFAULT;

                gen_int[loop].gen_int.wait_time = BMA580_WAIT_TIME_DEFAULT;

                gen_int[loop].gen_int.quiet_time = BMA580_QUIET_TIME_DEFAULT;

                gen_int[loop].gen_int.ref_acc_x = BMA580_ACC_REF_X_DEFAULT;

                gen_int[loop].gen_int.ref_acc_y = BMA580_ACC_REF_Y_DEFAULT;

                gen_int[loop].gen_int.ref_acc_z = BMA580_ACC_REF_Z_DEFAULT;

                if (dev->context == BMA5_HEARABLE && gen_int[loop].generic_interrupt == BMA580_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA580_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_H;
                    gen_int[loop].gen_int.hysteresis = BMA580_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_H;
                }
                else if (dev->context == BMA5_WEARABLE && gen_int[loop].generic_interrupt == BMA580_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA580_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_W;
                    gen_int[loop].gen_int.hysteresis = BMA580_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_W;
                }
                else if (dev->context == BMA5_SMARTPHONE && gen_int[loop].generic_interrupt == BMA580_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA580_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_S;
                    gen_int[loop].gen_int.hysteresis = BMA580_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_S;
                }
                else if (dev->context == BMA5_HEARABLE && gen_int[loop].generic_interrupt == BMA580_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA580_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_H;
                    gen_int[loop].gen_int.hysteresis = BMA580_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_H;
                }
                else if (dev->context == BMA5_WEARABLE && gen_int[loop].generic_interrupt == BMA580_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA580_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_W;
                    gen_int[loop].gen_int.hysteresis = BMA580_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_W;
                }
                else if (dev->context == BMA5_SMARTPHONE && gen_int[loop].generic_interrupt == BMA580_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA580_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_S;
                    gen_int[loop].gen_int.hysteresis = BMA580_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_S;
                }
                else
                {
                    result = BMA5_E_INVALID_CONTEXT_PARAM;
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets generic interrupt configurations
 */
int8_t bma580_set_generic_int_config(const struct bma580_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt */
    uint8_t data;

    /* Array to store generic interrupt data */
    uint8_t int_data[16] = { 0 };

    uint16_t slope_thres_1, slope_thres_2, comb_sel, axis_sel, hysteresis_1, hysteresis_2, criterion_sel;
    uint16_t acc_ref_up, duration_1, duration_2, wait_time, quiet_time_1, quiet_time_2;
    uint16_t ref_acc_x_1, ref_acc_x_2, ref_acc_y_1, ref_acc_y_2, ref_acc_z_1, ref_acc_z_2;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA580_GEN_INT_1:
                    data = BMA580_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA580_GEN_INT_2:
                    data = BMA580_BASE_ADDR_GENERIC_INT2;
                    break;

                case BMA580_GEN_INT_3:
                    data = BMA580_BASE_ADDR_GENERIC_INT3;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                /* Get the configuration from the feature engine register */
                result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, int_data, 16, dev);

                if (result == BMA5_OK)
                {
                    /* Settings 1 */
                    slope_thres_1 =
                        (BMA5_SET_BITS_POS_0(int_data[2], BMA580_GEN_INT_SLOPE_THRES,
                                             gen_int[loop].gen_int.slope_thres) & BMA580_GEN_INT_SLOPE_THRES_MSK);

                    slope_thres_2 = (uint16_t)(int_data[3] << 8);

                    slope_thres_2 =
                        (BMA5_SET_BITS_POS_0(slope_thres_2, BMA580_GEN_INT_SLOPE_THRES,
                                             gen_int[loop].gen_int.slope_thres) & BMA580_GEN_INT_SLOPE_THRES_MSK);

                    comb_sel =
                        (BMA5_SET_BITS(int_data[3], BMA580_GEN_INT_COMB_SEL,
                                       gen_int[loop].gen_int.comb_sel) & BMA580_GEN_INT_COMB_SEL_MSK);

                    axis_sel =
                        (BMA5_SET_BITS(int_data[3], BMA580_GEN_INT_AXIS_SEL,
                                       gen_int[loop].gen_int.axis_sel) & BMA580_GEN_INT_AXIS_SEL_MSK);

                    /* Settings 2 */
                    hysteresis_1 =
                        (BMA5_SET_BITS_POS_0(int_data[4], BMA580_GEN_INT_HYST,
                                             gen_int[loop].gen_int.hysteresis) & BMA580_GEN_INT_HYST_MSK);

                    hysteresis_2 = (uint16_t)(int_data[5] << 8);

                    hysteresis_2 =
                        (BMA5_SET_BITS_POS_0(hysteresis_2, BMA580_GEN_INT_HYST,
                                             gen_int[loop].gen_int.hysteresis) & BMA580_GEN_INT_HYST_MSK);

                    criterion_sel =
                        (BMA5_SET_BITS(int_data[5], BMA580_GEN_INT_CRIT_SEL,
                                       gen_int[loop].gen_int.criterion_sel) & BMA580_GEN_INT_CRIT_SEL_MSK);

                    acc_ref_up =
                        (BMA5_SET_BITS(int_data[5], BMA580_GEN_INT_ACC_REF_UP,
                                       gen_int[loop].gen_int.acc_ref_up) & BMA580_GEN_INT_ACC_REF_UP_MSK);

                    /* Settings 3 */
                    duration_1 =
                        (BMA5_SET_BITS_POS_0(int_data[6], BMA580_GEN_INT_DURATION,
                                             gen_int[loop].gen_int.duration) & BMA580_GEN_INT_DURATION_MSK);

                    duration_2 = (uint16_t)(int_data[7] << 8);

                    duration_2 =
                        (BMA5_SET_BITS_POS_0(duration_2, BMA580_GEN_INT_DURATION,
                                             gen_int[loop].gen_int.duration) & BMA580_GEN_INT_DURATION_MSK);

                    wait_time =
                        (BMA5_SET_BITS(int_data[7], BMA580_GEN_INT_WAIT_TIME,
                                       gen_int[loop].gen_int.wait_time) & BMA580_GEN_INT_WAIT_TIME_MSK);

                    /* Settings 4 */
                    quiet_time_1 =
                        (BMA5_SET_BITS_POS_0(int_data[8], BMA580_GEN_INT_QUIET_TIME,
                                             gen_int[loop].gen_int.quiet_time) & BMA580_GEN_INT_QUIET_TIME_MSK);

                    quiet_time_2 = (uint16_t)(int_data[9] << 8);

                    quiet_time_2 =
                        (BMA5_SET_BITS_POS_0(quiet_time_2, BMA580_GEN_INT_QUIET_TIME,
                                             gen_int[loop].gen_int.quiet_time) & BMA580_GEN_INT_QUIET_TIME_MSK);

                    /* Settings 5 */
                    ref_acc_x_1 =
                        BMA5_SET_BITS_POS_0(int_data[10], BMA580_GEN_INT_REF_ACC_X,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_x & BMA580_GEN_INT_REF_ACC_X_MSK));

                    ref_acc_x_2 = (uint16_t)(int_data[11] << 8);

                    ref_acc_x_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_x_2, BMA580_GEN_INT_REF_ACC_X,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_x & BMA580_GEN_INT_REF_ACC_X_MSK));

                    /* Settings 6 */
                    ref_acc_y_1 =
                        BMA5_SET_BITS_POS_0(int_data[12], BMA580_GEN_INT_REF_ACC_Y,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_y & BMA580_GEN_INT_REF_ACC_Y_MSK));

                    ref_acc_y_2 = (uint16_t)(int_data[13] << 8);

                    ref_acc_y_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_y_2, BMA580_GEN_INT_REF_ACC_Y,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_y & BMA580_GEN_INT_REF_ACC_Y_MSK));

                    /* Settings 7 */
                    ref_acc_z_1 =
                        BMA5_SET_BITS_POS_0(int_data[14], BMA580_GEN_INT_REF_ACC_Z,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_z & BMA580_GEN_INT_REF_ACC_Z_MSK));

                    ref_acc_z_2 = (uint16_t)(int_data[15] << 8);

                    ref_acc_z_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_z_2, BMA580_GEN_INT_REF_ACC_Z,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_z & BMA580_GEN_INT_REF_ACC_Z_MSK));

                    int_data[0] = (uint8_t)slope_thres_1;
                    int_data[1] = (uint8_t)((slope_thres_2 | comb_sel | axis_sel) >> 8);
                    int_data[2] = (uint8_t)hysteresis_1;
                    int_data[3] = (uint8_t)((hysteresis_2 | criterion_sel | acc_ref_up) >> 8);
                    int_data[4] = (uint8_t)duration_1;
                    int_data[5] = (uint8_t)((duration_2 | wait_time) >> 8);
                    int_data[6] = (uint8_t)quiet_time_1;
                    int_data[7] = (uint8_t)(quiet_time_2 >> 8);
                    int_data[8] = (uint8_t)ref_acc_x_1;
                    int_data[9] = (uint8_t)(ref_acc_x_2 >> 8);
                    int_data[10] = (uint8_t)ref_acc_y_1;
                    int_data[11] = (uint8_t)(ref_acc_y_2 >> 8);
                    int_data[12] = (uint8_t)ref_acc_z_1;
                    int_data[13] = (uint8_t)(ref_acc_z_2 >> 8);

                    /* Set the configuration from the feature engine register */
                    result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, int_data, 14, dev);
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets generic interrupt configurations
 */
int8_t bma580_get_generic_int_config(struct bma580_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt */
    uint8_t data;

    /* Array to store generic interrupt data */
    uint8_t int_data[16] = { 0 };

    /* Variable to define array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA580_GEN_INT_1:
                    data = BMA580_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA580_GEN_INT_2:
                    data = BMA580_BASE_ADDR_GENERIC_INT2;
                    break;

                case BMA580_GEN_INT_3:
                    data = BMA580_BASE_ADDR_GENERIC_INT3;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                /* Get the configuration from the feature engine register */
                result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, int_data, 16, dev);

                if (result == BMA5_OK)
                {
                    /* First two bytes are dummy bytes */
                    idx = 2;

                    /* Settings 1 */
                    /* Get word to calculate slope threshold, comb_sel and axis select from same word */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.slope_thres = lsb_msb & BMA580_GEN_INT_SLOPE_THRES_MSK;

                    gen_int[loop].gen_int.comb_sel = (lsb_msb & BMA580_GEN_INT_COMB_SEL_MSK) >>
                                                     BMA580_GEN_INT_COMB_SEL_POS;

                    gen_int[loop].gen_int.axis_sel = (lsb_msb & BMA580_GEN_INT_AXIS_SEL_MSK) >>
                                                     BMA580_GEN_INT_AXIS_SEL_POS;

                    /* Settings 2 */
                    /* Get word to calculate hysteresis, criterion_sel and acc_ref_up from same word */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.hysteresis = lsb_msb & BMA580_GEN_INT_HYST_MSK;

                    gen_int[loop].gen_int.criterion_sel = (lsb_msb & BMA580_GEN_INT_CRIT_SEL_MSK) >>
                                                          BMA580_GEN_INT_CRIT_SEL_POS;

                    gen_int[loop].gen_int.acc_ref_up = (lsb_msb & BMA580_GEN_INT_ACC_REF_UP_MSK) >>
                                                       BMA580_GEN_INT_ACC_REF_UP_POS;

                    /* Settings 3 */
                    /* Get word to calculate duration and wait time from same word */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.duration = lsb_msb & BMA580_GEN_INT_DURATION_MSK;

                    gen_int[loop].gen_int.wait_time = (lsb_msb & BMA580_GEN_INT_WAIT_TIME_MSK) >>
                                                      BMA580_GEN_INT_WAIT_TIME_POS;

                    /* Settings 4 */
                    /* Get word to calculate quiet time */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.quiet_time = lsb_msb & BMA580_GEN_INT_QUIET_TIME_MSK;

                    /* Settings 5 */
                    /* Get word to calculate ref_acc_x */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_x = (int16_t)(lsb_msb & BMA580_GEN_INT_REF_ACC_X_MSK);

                    /* Settings 6 */
                    /* Get word to calculate ref_acc_y */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_y = (int16_t)(lsb_msb & BMA580_GEN_INT_REF_ACC_Y_MSK);

                    /* Settings 7 */
                    /* Get word to calculate ref_acc_z */
                    lsb = (uint16_t) int_data[idx++];
                    msb = ((uint16_t) int_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_z = (int16_t)(lsb_msb & BMA580_GEN_INT_REF_ACC_Z_MSK);
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets accel foc configuration
 */
int8_t bma580_set_accel_foc_config(const struct bma580_accel_foc_config *acc_foc, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of accel foc */
    uint8_t data = BMA580_BASE_ADDR_ACC_FOC;

    /* Array to store accel foc config data */
    uint8_t acc_foc_data[10] = { 0 };

    uint16_t reg_data;

    uint16_t foc_off_x, foc_off_y, foc_off_z;
    uint8_t foc_apply_corr, foc_filter_coeff, foc_axis_1g;

    if (acc_foc == NULL)
    {
        result = BMA5_E_NULL_PTR;

    }
    else
    {
        /* Set the accel foc base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, acc_foc_data, 10, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[2] | ((uint16_t)acc_foc_data[3] << 8)));
                foc_off_x = BMA5_SET_BITS_POS_0(reg_data, BMA580_ACC_FOC_OFF_X, acc_foc->foc_off_x);

                /* Settings 2 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[4] | ((uint16_t)acc_foc_data[5] << 8)));
                foc_off_y = BMA5_SET_BITS_POS_0(reg_data, BMA580_ACC_FOC_OFF_Y, acc_foc->foc_off_y);

                /* Settings 3 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[6] | ((uint16_t)acc_foc_data[7] << 8)));
                foc_off_z = BMA5_SET_BITS_POS_0(reg_data, BMA580_ACC_FOC_OFF_Z, acc_foc->foc_off_z);

                /* Settings 4 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[8] | ((uint16_t)acc_foc_data[9] << 8)));
                foc_apply_corr = (uint8_t)BMA5_SET_BITS_POS_0(reg_data,
                                                              BMA580_ACC_FOC_APPLY_CORR,
                                                              acc_foc->foc_apply_corr);

                foc_filter_coeff = (uint8_t)BMA5_SET_BITS(reg_data,
                                                          BMA580_ACC_FOC_FILTER_COEFF,
                                                          acc_foc->foc_filter_coeff);

                foc_axis_1g = (uint8_t)BMA5_SET_BITS(reg_data, BMA580_ACC_FOC_AXIS_1G, acc_foc->foc_axis_1g);

                acc_foc_data[0] = (uint8_t)(foc_off_x & 0x00FF);
                acc_foc_data[1] = (uint8_t)((foc_off_x & 0xFF00) >> 8);
                acc_foc_data[2] = (uint8_t)(foc_off_y & 0x00FF);
                acc_foc_data[3] = (uint8_t)((foc_off_y & 0xFF00) >> 8);
                acc_foc_data[4] = (uint8_t)(foc_off_z & 0x00FF);
                acc_foc_data[5] = (uint8_t)((foc_off_z & 0xFF00) >> 8);
                acc_foc_data[6] = (uint8_t)(foc_apply_corr | foc_filter_coeff | foc_axis_1g);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, acc_foc_data, 8, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets accel foc configuration
 */
int8_t bma580_get_accel_foc_config(struct bma580_accel_foc_config *acc_foc, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of accel foc */
    uint8_t data = BMA580_BASE_ADDR_ACC_FOC;

    /* Array to store accel foc config data */
    uint8_t acc_foc_data[10] = { 0 };

    /* Variable to define array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (acc_foc == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the accel foc base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, acc_foc_data, 10, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate foc_off_x */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_off_x = lsb_msb & BMA580_ACC_FOC_OFF_X_MSK;

                /* Settings 2 */
                /* Get word to calculate foc_off_y */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_off_y = lsb_msb & BMA580_ACC_FOC_OFF_Y_MSK;

                /* Settings 3 */
                /* Get word to calculate foc_off_z */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_off_z = lsb_msb & BMA580_ACC_FOC_OFF_Z_MSK;

                /* Settings 4 */
                /* Get word to calculate foc_apply_corr, foc_filter_coeff and foc_axis_1g from same word  */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_apply_corr = lsb_msb & BMA580_ACC_FOC_APPLY_CORR_MSK;

                acc_foc->foc_filter_coeff = (lsb_msb & BMA580_ACC_FOC_FILTER_COEFF_MSK) >>
                                            BMA580_ACC_FOC_FILTER_COEFF_POS;

                acc_foc->foc_axis_1g = (lsb_msb & BMA580_ACC_FOC_AXIS_1G_MSK) >> BMA580_ACC_FOC_AXIS_1G_POS;
            }
        }
    }

    return result;
}

int8_t bma580_set_tap_config(const struct bma580_tap_config *tap_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of tap config */
    uint8_t data = BMA580_BASE_ADDR_TAP_DETECTOR;

    /* Array to store tap config data */
    uint8_t tap_conf_data[8] = { 0 };

    /* variables to store the data */
    uint16_t axis_sel, wait_for_timeout, max_peaks_for_tap, mode, s_tap_en, d_tap_en, t_tap_en;
    uint16_t max_gesture_dur, max_dur_between_peaks, tap_shock_settling_dur, min_quite_dur_between_taps;
    uint16_t tap_peak_thres_1, tap_peak_thres_2, quite_time_after_gesture;

    if (tap_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the tap config base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, tap_conf_data, 8, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */
                /* axis_sel */
                axis_sel =
                    (BMA5_SET_BITS_POS_0(tap_conf_data[2], BMA580_TAP_AXIS_SEL,
                                         tap_config->axis_sel) & BMA580_TAP_AXIS_SEL_MSK);

                /* wait_for_timeout */
                wait_for_timeout =
                    (BMA5_SET_BITS(tap_conf_data[2], BMA580_TAP_WAIT_FOR_TIMEOUT,
                                   tap_config->wait_for_timeout) & BMA580_TAP_WAIT_FOR_TIMEOUT_MSK);

                /* max_peaks_for_tap */
                max_peaks_for_tap =
                    (BMA5_SET_BITS(tap_conf_data[2], BMA580_TAP_MAX_PEAKS_FOR_TAP,
                                   tap_config->max_peaks_for_tap) & BMA580_TAP_MAX_PEAKS_FOR_TAP_MSK);

                /* mode */
                mode = (BMA5_SET_BITS(tap_conf_data[2], BMA580_TAP_MODE, tap_config->mode) & BMA580_TAP_MODE_MSK);

                /* s_tap_en */
                s_tap_en =
                    (BMA5_SET_BITS(tap_conf_data[3], BMA580_TAP_SINGLE_EN,
                                   tap_config->s_tap_en) & BMA580_TAP_SINGLE_EN_MSK);

                /* d_tap_en */
                d_tap_en =
                    (BMA5_SET_BITS(tap_conf_data[3], BMA580_TAP_DOUBLE_EN,
                                   tap_config->d_tap_en) & BMA580_TAP_DOUBLE_EN_MSK);

                /* t_tap_en */
                t_tap_en =
                    (BMA5_SET_BITS(tap_conf_data[3], BMA580_TAP_TRIBLE_EN,
                                   tap_config->t_tap_en) & BMA580_TAP_TRIBLE_EN_MSK);

                /* Settings 2 */
                /* tap_peak_thres */
                tap_peak_thres_1 =
                    (BMA5_SET_BITS_POS_0(tap_conf_data[4], BMA580_TAP_PEAK_THRES,
                                         tap_config->tap_peak_thres) & BMA580_TAP_PEAK_THRES_MSK);

                tap_peak_thres_2 = (uint16_t)(tap_conf_data[5] << 8);

                tap_peak_thres_2 =
                    (BMA5_SET_BITS_POS_0(tap_peak_thres_2, BMA580_TAP_PEAK_THRES,
                                         tap_config->tap_peak_thres) & BMA580_TAP_PEAK_THRES_MSK);

                /* max_gesture_dur */
                max_gesture_dur =
                    (BMA5_SET_BITS(tap_conf_data[5], BMA580_TAP_MAX_GESTURE_DUR,
                                   tap_config->max_gesture_dur) & BMA580_TAP_MAX_GESTURE_DUR_MSK);

                /* Settings 3 */
                /* max_dur_between_peaks */
                max_dur_between_peaks =
                    (BMA5_SET_BITS_POS_0(tap_conf_data[6], BMA580_TAP_MAX_DUR_BET_PEAKS,
                                         tap_config->max_dur_between_peaks) & BMA580_TAP_MAX_DUR_BET_PEAKS_MSK);

                /* tap_shock_settling_dur */
                tap_shock_settling_dur =
                    (BMA5_SET_BITS(tap_conf_data[6], BMA580_TAP_SHOCK_DET_DUR,
                                   tap_config->tap_shock_settling_dur) & BMA580_TAP_SHOCK_DET_DUR_MSK);

                /* min_quite_dur_between_taps */
                min_quite_dur_between_taps =
                    (BMA5_SET_BITS(tap_conf_data[7], BMA580_TAP_MIN_QUITE_DUR,
                                   tap_config->min_quite_dur_between_taps) & BMA580_TAP_MIN_QUITE_DUR_MSK);

                /* quite_time_after_gesture */
                quite_time_after_gesture =
                    (BMA5_SET_BITS(tap_conf_data[7], BMA580_TAP_QUITE_TIME,
                                   tap_config->quite_time_after_gesture) & BMA580_TAP_QUITE_TIME_MSK);

                tap_conf_data[0] = (uint8_t)(axis_sel | wait_for_timeout | max_peaks_for_tap | mode);
                tap_conf_data[1] = (uint8_t)((s_tap_en | d_tap_en | t_tap_en) >> 8);
                tap_conf_data[2] = (uint8_t)(tap_peak_thres_1);
                tap_conf_data[3] = (uint8_t)((tap_peak_thres_2 >> 8) | (max_gesture_dur >> 8));
                tap_conf_data[4] = (uint8_t)(max_dur_between_peaks | tap_shock_settling_dur);
                tap_conf_data[5] = (uint8_t)((min_quite_dur_between_taps | quite_time_after_gesture) >> 8);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, tap_conf_data, 6, dev);
            }
        }
    }

    return result;
}

int8_t bma580_get_tap_config(struct bma580_tap_config *tap_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of tap config */
    uint8_t data = BMA580_BASE_ADDR_TAP_DETECTOR;

    /* Array to store tap config data */
    uint8_t tap_conf_data[8] = { 0 };

    /* Variable to define array offset */
    uint8_t idx;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (tap_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the tap base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, tap_conf_data, 8, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */

                /* Get word to calculate axis_sel, wait_for_timeout, max_peaks_for_tap, mode,
                 * s_tap_en, d_tap_en and t_tap_en from same word */
                lsb = (uint16_t) tap_conf_data[idx++];
                msb = ((uint16_t) tap_conf_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                /* axis_sel */
                tap_config->axis_sel = lsb_msb & BMA580_TAP_AXIS_SEL_MSK;

                /* wait_for_timeout */
                tap_config->wait_for_timeout = BMA5_GET_BITS(lsb_msb, BMA580_TAP_WAIT_FOR_TIMEOUT);

                /* max_peaks_for_tap */
                tap_config->max_peaks_for_tap = BMA5_GET_BITS(lsb_msb, BMA580_TAP_MAX_PEAKS_FOR_TAP);

                /* mode */
                tap_config->mode = BMA5_GET_BITS(lsb_msb, BMA580_TAP_MODE);

                /* s_tap_en */
                tap_config->s_tap_en = BMA5_GET_BITS(lsb_msb, BMA580_TAP_SINGLE_EN);

                /* d_tap_en */
                tap_config->d_tap_en = BMA5_GET_BITS(lsb_msb, BMA580_TAP_DOUBLE_EN);

                /* t_tap_en */
                tap_config->t_tap_en = BMA5_GET_BITS(lsb_msb, BMA580_TAP_TRIBLE_EN);

                /* Settings 2 */
                /* Get word to calculate tap_peak_thres and max_gesture_dur  */
                lsb = (uint16_t) tap_conf_data[idx++];
                msb = ((uint16_t) tap_conf_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                /* tap_peak_thres */
                tap_config->tap_peak_thres = lsb_msb & BMA580_TAP_PEAK_THRES_MSK;

                /* max_gesture_dur */
                tap_config->max_gesture_dur = BMA5_GET_BITS(lsb_msb, BMA580_TAP_MAX_GESTURE_DUR);

                /* Settings 3 */

                /* Get word to calculate max_dur_between_peaks, tap_shock_settling_dur,
                 * min_quite_dur_between_taps and quite_time_after_gesture */
                lsb = (uint16_t) tap_conf_data[idx++];
                msb = ((uint16_t) tap_conf_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                /* max_dur_between_peaks */
                tap_config->max_dur_between_peaks = lsb_msb & BMA580_TAP_MAX_DUR_BET_PEAKS_MSK;

                /* tap_shock_settling_dur */
                tap_config->tap_shock_settling_dur = BMA5_GET_BITS(lsb_msb, BMA580_TAP_SHOCK_DET_DUR);

                /* min_quite_dur_between_taps */
                tap_config->min_quite_dur_between_taps = BMA5_GET_BITS(lsb_msb, BMA580_TAP_MIN_QUITE_DUR);

                /* quite_time_after_gesture */
                tap_config->quite_time_after_gesture = BMA5_GET_BITS(lsb_msb, BMA580_TAP_QUITE_TIME);
            }
        }
    }

    return result;
}

int8_t bma580_get_default_tap_config(struct bma580_tap_config *tap_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of tap config */
    uint8_t data = BMA580_BASE_ADDR_TAP_DETECTOR;

    if (tap_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the tap base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            if (dev->context == BMA5_HEARABLE)
            {
                /* Settings 1 */

                /* axis_sel */
                tap_config->axis_sel = BMA580_TAP_DETECTOR_1_AXIS_SEL_H;

                /* wait_for_timeout */
                tap_config->wait_for_timeout = BMA580_TAP_DETECTOR_1_WAIT_FOR_TIMEOUT_H;

                /* max_peaks_for_tap */
                tap_config->max_peaks_for_tap = BMA580_TAP_DETECTOR_1_MAX_PEAKS_FOR_TAP_H;

                /* mode */
                tap_config->mode = BMA580_TAP_DETECTOR_1_MODE_H;

                /* Settings 2 */

                /* tap_peak_thres */
                tap_config->tap_peak_thres = BMA580_TAP_DETECTOR_2_TAP_PEAK_THRES_H;

                /* max_gesture_dur */
                tap_config->max_gesture_dur = BMA580_TAP_DETECTOR_2_MAX_GESTURE_DUR_H;

                /* Settings 3 */

                /* max_dur_between_peaks */
                tap_config->max_dur_between_peaks = BMA580_TAP_DETECTOR_3_MAX_DUR_BETWEEN_PEAKS_H;

                /* tap_shock_settling_dur */
                tap_config->tap_shock_settling_dur = BMA580_TAP_DETECTOR_3_TAP_SHOCK_SETTLING_DUR_H;

                /* min_quite_dur_between_taps */
                tap_config->min_quite_dur_between_taps = BMA580_TAP_DETECTOR_3_MIN_QUITE_DUR_BETWEEN_TAPS_H;

                /* quite_time_after_gesture */
                tap_config->quite_time_after_gesture = BMA580_TAP_DETECTOR_3_QUITE_TIME_AFTER_GESTURE_H;
            }
            else if (dev->context == BMA5_WEARABLE)
            {
                /* Settings 1 */

                /* axis_sel */
                tap_config->axis_sel = BMA580_TAP_DETECTOR_1_AXIS_SEL_W;

                /* wait_for_timeout */
                tap_config->wait_for_timeout = BMA580_TAP_DETECTOR_1_WAIT_FOR_TIMEOUT_W;

                /* max_peaks_for_tap */
                tap_config->max_peaks_for_tap = BMA580_TAP_DETECTOR_1_MAX_PEAKS_FOR_TAP_W;

                /* mode */
                tap_config->mode = BMA580_TAP_DETECTOR_1_MODE_W;

                /* Settings 2 */

                /* tap_peak_thres */
                tap_config->tap_peak_thres = BMA580_TAP_DETECTOR_2_TAP_PEAK_THRES_W;

                /* max_gesture_dur */
                tap_config->max_gesture_dur = BMA580_TAP_DETECTOR_2_MAX_GESTURE_DUR_W;

                /* Settings 3 */

                /* max_dur_between_peaks */
                tap_config->max_dur_between_peaks = BMA580_TAP_DETECTOR_3_MAX_DUR_BETWEEN_PEAKS_W;

                /* tap_shock_settling_dur */
                tap_config->tap_shock_settling_dur = BMA580_TAP_DETECTOR_3_TAP_SHOCK_SETTLING_DUR_W;

                /* min_quite_dur_between_taps */
                tap_config->min_quite_dur_between_taps = BMA580_TAP_DETECTOR_3_MIN_QUITE_DUR_BETWEEN_TAPS_W;

                /* quite_time_after_gesture */
                tap_config->quite_time_after_gesture = BMA580_TAP_DETECTOR_3_QUITE_TIME_AFTER_GESTURE_W;
            }
            else if (dev->context == BMA5_SMARTPHONE)
            {
                /* Settings 1 */

                /* axis_sel */
                tap_config->axis_sel = BMA580_TAP_DETECTOR_1_AXIS_SEL_S;

                /* wait_for_timeout */
                tap_config->wait_for_timeout = BMA580_TAP_DETECTOR_1_WAIT_FOR_TIMEOUT_S;

                /* max_peaks_for_tap */
                tap_config->max_peaks_for_tap = BMA580_TAP_DETECTOR_1_MAX_PEAKS_FOR_TAP_S;

                /* mode */
                tap_config->mode = BMA580_TAP_DETECTOR_1_MODE_S;

                /* Settings 2 */

                /* tap_peak_thres */
                tap_config->tap_peak_thres = BMA580_TAP_DETECTOR_2_TAP_PEAK_THRES_S;

                /* max_gesture_dur */
                tap_config->max_gesture_dur = BMA580_TAP_DETECTOR_2_MAX_GESTURE_DUR_S;

                /* Settings 3 */

                /* max_dur_between_peaks */
                tap_config->max_dur_between_peaks = BMA580_TAP_DETECTOR_3_MAX_DUR_BETWEEN_PEAKS_S;

                /* tap_shock_settling_dur */
                tap_config->tap_shock_settling_dur = BMA580_TAP_DETECTOR_3_TAP_SHOCK_SETTLING_DUR_S;

                /* min_quite_dur_between_taps */
                tap_config->min_quite_dur_between_taps = BMA580_TAP_DETECTOR_3_MIN_QUITE_DUR_BETWEEN_TAPS_S;

                /* quite_time_after_gesture */
                tap_config->quite_time_after_gesture = BMA580_TAP_DETECTOR_3_QUITE_TIME_AFTER_GESTURE_S;
            }
            else
            {
                result = BMA5_E_INVALID_CONTEXT_PARAM;
            }

            /* s_tap_en */
            tap_config->s_tap_en = BMA580_DEFAULT_S_TAP_EN;

            /* d_tap_en */
            tap_config->d_tap_en = BMA580_DEFAULT_D_TAP_EN;

            /* t_tap_en */
            tap_config->t_tap_en = BMA580_DEFAULT_T_TAP_EN;
        }
    }

    return result;
}

/*!
 * @brief This API sets vad configurations
 */
int8_t bma580_set_vad_config(const struct bma580_vad_config *vad_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA580_BASE_ADDR_VAD;

    /* Array to store feature axis data */
    uint8_t vad_data[8] = { 0 };

    if (vad_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Set the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, vad_data, 8, dev);

            if (result == BMA5_OK)
            {
                vad_data[0] = BMA5_SET_BITS_POS_0(vad_data[2], BMA580_VAD_EN_VAD_X, vad_config->en_vad_x);

                vad_data[0] = BMA5_SET_BITS(vad_data[2], BMA580_VAD_EN_VAD_Y, vad_config->en_vad_y);

                vad_data[0] = BMA5_SET_BITS(vad_data[2], BMA580_VAD_EN_VAD_Z, vad_config->en_vad_z);

                vad_data[1] = vad_data[3];
                vad_data[2] = vad_data[4];
                vad_data[3] = vad_data[5];
                vad_data[4] = vad_data[6];
                vad_data[5] = vad_data[7];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, vad_data, 6, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets vad configurations
 */
int8_t bma580_get_vad_config(struct bma580_vad_config *vad_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA580_BASE_ADDR_VAD;

    /* Array to store feature axis data */
    uint8_t vad_data[8] = { 0 };

    if (vad_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, vad_data, 8, dev);

            if (result == BMA5_OK)
            {
                vad_config->en_vad_x = BMA5_GET_BITS_POS_0(vad_data[2], BMA580_VAD_EN_VAD_X);

                vad_config->en_vad_y = BMA5_GET_BITS(vad_data[2], BMA580_VAD_EN_VAD_Y);

                vad_config->en_vad_z = BMA5_GET_BITS(vad_data[2], BMA580_VAD_EN_VAD_Z);
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets self wake-up configurations
 */
int8_t bma580_set_self_wakeup_config(const struct bma580_self_wakeup_config *self_wakeup_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA580_BASE_ADDR_SELF_WAKE_UP;

    /* Array to store feature axis data */
    uint8_t self_wake_up_data[4] = { 0 };

    if (self_wakeup_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, self_wake_up_data, 4, dev);

            if (result == BMA5_OK)
            {
                self_wake_up_data[2] = BMA5_SET_BITS_POS_0(self_wake_up_data[2],
                                                           BMA580_SELF_WAKE_UP_ACC_ODR,
                                                           self_wakeup_config->acc_odr);

                self_wake_up_data[2] = BMA5_SET_BITS(self_wake_up_data[2],
                                                     BMA580_SELF_WAKE_UP_ACC_BWP,
                                                     self_wakeup_config->acc_bwp);

                self_wake_up_data[2] = BMA5_SET_BITS(self_wake_up_data[2],
                                                     BMA580_SELF_WAKE_UP_ACC_PERF_MODE,
                                                     self_wakeup_config->acc_perf_mode);

                self_wake_up_data[0] = self_wake_up_data[2];
                self_wake_up_data[1] = self_wake_up_data[3];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, self_wake_up_data, 2, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets self wake-up configurations
 */
int8_t bma580_get_self_wakeup_config(struct bma580_self_wakeup_config *self_wakeup_config, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA580_BASE_ADDR_SELF_WAKE_UP;

    /* Array to store feature axis data */
    uint8_t self_wake_up_data[4] = { 0 };

    if (self_wakeup_config == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, self_wake_up_data, 4, dev);

            if (result == BMA5_OK)
            {
                self_wakeup_config->acc_odr = BMA5_GET_BITS_POS_0(self_wake_up_data[2], BMA580_SELF_WAKE_UP_ACC_ODR);

                self_wakeup_config->acc_bwp = BMA5_GET_BITS(self_wake_up_data[2], BMA580_SELF_WAKE_UP_ACC_BWP);

                self_wakeup_config->acc_perf_mode = BMA5_GET_BITS(self_wake_up_data[2],
                                                                  BMA580_SELF_WAKE_UP_ACC_PERF_MODE);
            }
        }
    }

    return result;
}

/*!
 * @brief This API resets sensor. All registers are overwritten with
 * their default values.
 */
int8_t bma580_soft_reset(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to perform dummy read */
    uint8_t dummy_read;

    /* Assign soft-reset command */
    uint8_t cmd = BMA5_CMD_SOFTRESET;

    /* Null-pointer check */
    result = verify_handle(dev);

    if (result != BMA5_OK)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        if (dev->intf == BMA5_SPI_INTF)
        {
            dev->dummy_byte = 1;
        }
        else
        {
            dev->dummy_byte = 0;
        }

        /* Set soft-reset command */
        result = bma5_set_regs(BMA5_REG_CMD, &cmd, 1, dev);

        if (result == BMA5_OK)
        {
            /* Provide 2ms delay after soft-reset */
            dev->delay_us(BMA580_SOFT_RESET_DELAY, dev->intf_ptr);

            /* Dummy read is performed after soft-reset */
            result = bma5_get_regs(BMA580_REG_CHIP_ID, &dummy_read, 1, dev);
        }
    }

    return result;
}

/******************************************************************************/
/*********************** Static function definitions **************************/
/******************************************************************************/

static int8_t verify_handle(const struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_E_NULL_PTR;

    if (NULL != dev)
    {
        if ((NULL != dev->bus_read) && (NULL != dev->bus_write) && (NULL != dev->delay_us))
        {
            result = BMA5_OK;
        }
    }

    return result;
}
