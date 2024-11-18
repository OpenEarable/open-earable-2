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
* @file       bma580.c
* @date       2024-07-29
* @version    v4.2.0
*
*/

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#include "bma580.h"

/******************************************************************************/
/*********************** User function definitions ****************************/
/******************************************************************************/
int8_t bma580_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == chip_id)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_CHIP_ID, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *chip_id = BMA5_GET_BITS_POS_0(reg_value, BMA580_CHIP_ID);
        }
    }

    return result;
}

int8_t bma580_get_int_status(struct bma580_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_status; loop++)
        {
            switch (config[loop].int_src)
            {
                case BMA580_INT_STATUS_INT1:
                    result = bma5_get_regs(BMA580_REG_INT_STATUS_INT1_0, reg_value, 2, dev);
                    break;

                case BMA580_INT_STATUS_INT2:
                    result = bma5_get_regs(BMA580_REG_INT_STATUS_INT2_0, reg_value, 2, dev);
                    break;

                case BMA580_INT_STATUS_I3C:
                    result = bma5_get_regs(BMA580_REG_INT_STATUS_I3C_0, reg_value, 2, dev);
                    break;

                default:
                    result = BMA5_E_INVALID_INT_STATUS;
                    break;
            }

            if (BMA5_OK != result)
            {
                break;
            }

            /* Parse needed details from received serial data */
            config[loop].int_status.acc_drdy_int_status = BMA5_GET_BITS_POS_0(reg_value[0], BMA580_ACC_DRDY_INT_STATUS);
            config[loop].int_status.fifo_wm_int_status = BMA5_GET_BITS(reg_value[0], BMA580_FIFO_WM_INT_STATUS);
            config[loop].int_status.fifo_full_int_status = BMA5_GET_BITS(reg_value[0], BMA580_FIFO_FULL_INT_STATUS);
            config[loop].int_status.gen_int1_int_status = BMA5_GET_BITS(reg_value[0], BMA580_GEN_INT1_INT_STATUS);
            config[loop].int_status.gen_int2_int_status = BMA5_GET_BITS(reg_value[0], BMA580_GEN_INT2_INT_STATUS);
            config[loop].int_status.gen_int3_int_status = BMA5_GET_BITS(reg_value[0], BMA580_GEN_INT3_INT_STATUS);
            config[loop].int_status.acc_foc_int_status = BMA5_GET_BITS(reg_value[0], BMA580_ACC_FOC_INT_STATUS);
            config[loop].int_status.stap_int_status = BMA5_GET_BITS(reg_value[0], BMA580_STAP_INT_STATUS);

            config[loop].int_status.dtap_int_status = BMA5_GET_BITS_POS_0(reg_value[1], BMA580_DTAP_INT_STATUS);
            config[loop].int_status.ttap_int_status = BMA5_GET_BITS(reg_value[1], BMA580_TTAP_INT_STATUS);
            config[loop].int_status.vad_int_status = BMA5_GET_BITS(reg_value[1], BMA580_VAD_INT_STATUS);
            config[loop].int_status.self_wake_up_int_status =
                BMA5_GET_BITS(reg_value[1], BMA580_SELF_WAKE_UP_INT_STATUS);
            config[loop].int_status.feat_eng_err_int_status =
                BMA5_GET_BITS(reg_value[1], BMA580_FEAT_ENG_ERR_INT_STATUS);
        }
    }

    return result;
}

int8_t bma580_set_int_status(const struct bma580_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_status; loop++)
        {
            /* Bring up the register value to be set, as per the input details */
            reg_value[0] = BMA5_SET_BITS_POS_0(reg_value[0],
                                               BMA580_ACC_DRDY_INT_STATUS,
                                               config[loop].int_status.acc_drdy_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA580_FIFO_WM_INT_STATUS,
                                         config[loop].int_status.fifo_wm_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA580_FIFO_FULL_INT_STATUS,
                                         config[loop].int_status.fifo_full_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA580_GEN_INT1_INT_STATUS,
                                         config[loop].int_status.gen_int1_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA580_GEN_INT2_INT_STATUS,
                                         config[loop].int_status.gen_int2_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA580_GEN_INT3_INT_STATUS,
                                         config[loop].int_status.gen_int3_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA580_ACC_FOC_INT_STATUS,
                                         config[loop].int_status.acc_foc_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA580_STAP_INT_STATUS, config[loop].int_status.stap_int_status);

            reg_value[1] = BMA5_SET_BITS_POS_0(reg_value[1],
                                               BMA580_DTAP_INT_STATUS,
                                               config[loop].int_status.dtap_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA580_TTAP_INT_STATUS, config[loop].int_status.ttap_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA580_VAD_INT_STATUS, config[loop].int_status.vad_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA580_SELF_WAKE_UP_INT_STATUS,
                                         config[loop].int_status.self_wake_up_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA580_FEAT_ENG_ERR_INT_STATUS,
                                         config[loop].int_status.feat_eng_err_int_status);

            switch (config[loop].int_src)
            {
                case BMA580_INT_STATUS_INT1:
                    result = bma5_set_regs(BMA580_REG_INT_STATUS_INT1_0, reg_value, 2, dev);
                    break;

                case BMA580_INT_STATUS_INT2:
                    result = bma5_set_regs(BMA580_REG_INT_STATUS_INT2_0, reg_value, 2, dev);
                    break;

                case BMA580_INT_STATUS_I3C:
                    result = bma5_set_regs(BMA580_REG_INT_STATUS_I3C_0, reg_value, 2, dev);
                    break;
                default:
                    result = BMA5_E_INVALID_INT_STATUS;
                    break;
            }

            if (BMA5_OK != result)
            {
                break;
            }
        }
    }

    return result;
}

int8_t bma580_get_int_map(struct bma580_int_map *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[4] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_INT_MAP_0, reg_value, 4, dev);

        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->acc_drdy_int_map = BMA5_GET_BITS_POS_0(reg_value[0], BMA580_ACC_DRDY_INT_MAP);
            config->fifo_wm_int_map = BMA5_GET_BITS(reg_value[0], BMA580_FIFO_WM_INT_MAP);
            config->fifo_full_int_map = BMA5_GET_BITS(reg_value[0], BMA580_FIFO_FULL_INT_MAP);
            config->gen_int1_int_map = BMA5_GET_BITS(reg_value[0], BMA580_GEN_INT1_INT_MAP);

            config->gen_int2_int_map = BMA5_GET_BITS_POS_0(reg_value[1], BMA580_GEN_INT2_INT_MAP);
            config->gen_int3_int_map = BMA5_GET_BITS(reg_value[1], BMA580_GEN_INT3_INT_MAP);
            config->acc_foc_int_map = BMA5_GET_BITS(reg_value[1], BMA580_ACC_FOC_INT_MAP);
            config->stap_int_map = BMA5_GET_BITS(reg_value[1], BMA580_STAP_INT_MAP);

            config->dtap_int_map = BMA5_GET_BITS_POS_0(reg_value[2], BMA580_DTAP_INT_MAP);
            config->ttap_int_map = BMA5_GET_BITS(reg_value[2], BMA580_TTAP_INT_MAP);
            config->vad_int_map = BMA5_GET_BITS(reg_value[2], BMA580_VAD_INT_MAP);
            config->self_wake_up_int_map = BMA5_GET_BITS(reg_value[2], BMA580_SELF_WAKE_UP_INT_MAP);

            config->feat_eng_err_int_map = BMA5_GET_BITS_POS_0(reg_value[3], BMA580_FEAT_ENG_ERR_INT_MAP);
        }
    }

    return result;
}

int8_t bma580_set_int_map(const struct bma580_int_map *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[4] = { 0 };

    uint8_t acc_drdy_int_map, fifo_wm_int_map, fifo_full_int_map, gen_int1_int_map;
    uint8_t gen_int2_int_map, gen_int3_int_map, acc_foc_int_map, stap_int_map;
    uint8_t dtap_int_map, ttap_int_map, vad_int_map, self_wake_up_int_map;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_INT_MAP_0, reg_value, 4, dev);

        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            acc_drdy_int_map =
                (BMA5_SET_BITS_POS_0(reg_value[0], BMA580_ACC_DRDY_INT_MAP,
                                     config->acc_drdy_int_map) & BMA580_ACC_DRDY_INT_MAP_MSK);
            fifo_wm_int_map =
                (BMA5_SET_BITS(reg_value[0], BMA580_FIFO_WM_INT_MAP,
                               config->fifo_wm_int_map) & BMA580_FIFO_WM_INT_MAP_MSK);
            fifo_full_int_map =
                (BMA5_SET_BITS(reg_value[0], BMA580_FIFO_FULL_INT_MAP,
                               config->fifo_full_int_map) & BMA580_FIFO_FULL_INT_MAP_MSK);
            gen_int1_int_map =
                (BMA5_SET_BITS(reg_value[0], BMA580_GEN_INT1_INT_MAP,
                               config->gen_int1_int_map) & BMA580_GEN_INT1_INT_MAP_MSK);

            reg_value[0] = (uint8_t)(acc_drdy_int_map | fifo_wm_int_map | fifo_full_int_map | gen_int1_int_map);

            gen_int2_int_map =
                (BMA5_SET_BITS_POS_0(reg_value[1], BMA580_GEN_INT2_INT_MAP,
                                     config->gen_int2_int_map) & BMA580_GEN_INT2_INT_MAP_MSK);
            gen_int3_int_map =
                (BMA5_SET_BITS(reg_value[1], BMA580_GEN_INT3_INT_MAP,
                               config->gen_int3_int_map) & BMA580_GEN_INT3_INT_MAP_MSK);
            acc_foc_int_map =
                (BMA5_SET_BITS(reg_value[1], BMA580_ACC_FOC_INT_MAP,
                               config->acc_foc_int_map) & BMA580_ACC_FOC_INT_MAP_MSK);
            stap_int_map =
                (BMA5_SET_BITS(reg_value[1], BMA580_STAP_INT_MAP, config->stap_int_map) & BMA580_STAP_INT_MAP_MSK);

            reg_value[1] = (gen_int2_int_map | gen_int3_int_map | acc_foc_int_map | stap_int_map);

            dtap_int_map =
                (BMA5_SET_BITS_POS_0(reg_value[2], BMA580_DTAP_INT_MAP,
                                     config->dtap_int_map) & BMA580_DTAP_INT_MAP_MSK);
            ttap_int_map =
                (BMA5_SET_BITS(reg_value[2], BMA580_TTAP_INT_MAP, config->ttap_int_map) & BMA580_TTAP_INT_MAP_MSK);
            vad_int_map =
                (BMA5_SET_BITS(reg_value[2], BMA580_VAD_INT_MAP, config->vad_int_map) & BMA580_VAD_INT_MAP_MSK);
            self_wake_up_int_map =
                (BMA5_SET_BITS(reg_value[2], BMA580_SELF_WAKE_UP_INT_MAP,
                               config->self_wake_up_int_map) & BMA580_SELF_WAKE_UP_INT_MAP_MSK);

            reg_value[2] = (dtap_int_map | ttap_int_map | vad_int_map | self_wake_up_int_map);

            reg_value[3] =
                (BMA5_SET_BITS_POS_0(reg_value[3], BMA580_FEAT_ENG_ERR_INT_MAP,
                                     config->feat_eng_err_int_map) & BMA580_FEAT_ENG_ERR_INT_MAP_MSK);

            result = bma5_set_regs(BMA580_REG_INT_MAP_0, reg_value, 4, dev);
        }
    }

    return result;
}

int8_t bma580_get_feat_eng_gp_flags(struct bma580_feat_eng_gp_flags *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_FEAT_ENG_GP_FLAGS, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->feat_init_stat = BMA5_GET_BITS_POS_0(reg_value, BMA580_FEAT_INIT_STAT);
            config->foc_running = BMA5_GET_BITS(reg_value, BMA580_FOC_RUNNING);
        }
    }

    return result;
}

int8_t bma580_get_feat_eng_gpr_0(struct bma580_feat_eng_gpr_0 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_FEAT_ENG_GPR_0, &reg_value, sizeof(reg_value), dev);

        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->gen_int1_en = BMA5_GET_BITS_POS_0(reg_value, BMA580_GEN_INT1_EN);
            config->gen_int2_en = BMA5_GET_BITS(reg_value, BMA580_GEN_INT2_EN);
            config->gen_int3_en = BMA5_GET_BITS(reg_value, BMA580_GEN_INT3_EN);
            config->acc_foc_en = BMA5_GET_BITS(reg_value, BMA580_ACC_FOC_EN);
            config->tap_en = BMA5_GET_BITS(reg_value, BMA580_TAP_EN);
            config->vad_en = BMA5_GET_BITS(reg_value, BMA580_VAD_EN);
            config->self_wake_up_en = BMA5_GET_BITS(reg_value, BMA580_SELF_WAKE_UP_EN);
        }
    }

    return result;
}

int8_t bma580_set_feat_eng_gpr_0(const struct bma580_feat_eng_gpr_0 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value, get_reg_value;

    uint8_t gen_int1_en, gen_int2_en, gen_int3_en, acc_foc_en, tap_en, vad_en, self_wake_up_en;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_FEAT_ENG_GPR_0, &get_reg_value, sizeof(get_reg_value), dev);

        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            gen_int1_en =
                (BMA5_SET_BITS_POS_0(get_reg_value, BMA580_GEN_INT1_EN, config->gen_int1_en) & BMA580_GEN_INT1_EN_MSK);
            gen_int2_en =
                (BMA5_SET_BITS(get_reg_value, BMA580_GEN_INT2_EN, config->gen_int2_en) & BMA580_GEN_INT2_EN_MSK);
            gen_int3_en =
                (BMA5_SET_BITS(get_reg_value, BMA580_GEN_INT3_EN, config->gen_int3_en) & BMA580_GEN_INT3_EN_MSK);
            acc_foc_en = (BMA5_SET_BITS(get_reg_value, BMA580_ACC_FOC_EN, config->acc_foc_en) & BMA580_ACC_FOC_EN_MSK);
            tap_en = (BMA5_SET_BITS(get_reg_value, BMA580_TAP_EN, config->tap_en) & BMA580_TAP_EN_MSK);
            vad_en = (BMA5_SET_BITS(get_reg_value, BMA580_VAD_EN, config->vad_en) & BMA580_VAD_EN_MSK);
            self_wake_up_en =
                (BMA5_SET_BITS(get_reg_value, BMA580_SELF_WAKE_UP_EN,
                               config->self_wake_up_en) & BMA580_SELF_WAKE_UP_EN_MSK);

            reg_value = (gen_int1_en | gen_int2_en | gen_int3_en | acc_foc_en | tap_en | vad_en | self_wake_up_en);

            result = bma5_set_regs(BMA580_REG_FEAT_ENG_GPR_0, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
        }
    }

    return result;
}

int8_t bma580_get_feat_eng_gpr_1(struct bma580_feat_eng_gpr_1 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_FEAT_ENG_GPR_1, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->gen_int1_data_src = BMA5_GET_BITS_POS_0(reg_value, BMA580_GEN_INT1_DATA_SRC);
            config->gen_int2_data_src = BMA5_GET_BITS(reg_value, BMA580_GEN_INT2_DATA_SRC);
            config->gen_int3_data_src = BMA5_GET_BITS(reg_value, BMA580_GEN_INT3_DATA_SRC);
        }
    }

    return result;
}

int8_t bma580_set_feat_eng_gpr_1(const struct bma580_feat_eng_gpr_1 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    uint8_t gen_int1_data_src, gen_int2_data_src, gen_int3_data_src;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_FEAT_ENG_GPR_1, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            gen_int1_data_src =
                (BMA5_SET_BITS_POS_0(reg_value, BMA580_GEN_INT1_DATA_SRC,
                                     config->gen_int1_data_src) & BMA580_GEN_INT1_DATA_SRC_MSK);
            gen_int2_data_src =
                (BMA5_SET_BITS(reg_value, BMA580_GEN_INT2_DATA_SRC,
                               config->gen_int2_data_src) & BMA580_GEN_INT2_DATA_SRC_MSK);
            gen_int3_data_src =
                (BMA5_SET_BITS(reg_value, BMA580_GEN_INT3_DATA_SRC,
                               config->gen_int3_data_src) & BMA580_GEN_INT3_DATA_SRC_MSK);

            reg_value = (uint8_t)(gen_int1_data_src | gen_int2_data_src | gen_int3_data_src);

            result = bma5_set_regs(BMA580_REG_FEAT_ENG_GPR_1, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
        }
    }

    return result;
}

int8_t bma580_get_feat_eng_gpr_2(struct bma580_feat_eng_gpr_2 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_FEAT_ENG_GPR_2, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->gen_int1_stat = BMA5_GET_BITS_POS_0(reg_value, BMA580_GEN_INT1_STAT);
            config->gen_int2_stat = BMA5_GET_BITS(reg_value, BMA580_GEN_INT2_STAT);
            config->gen_int3_stat = BMA5_GET_BITS(reg_value, BMA580_GEN_INT3_STAT);
            config->self_wake_up_stat = BMA5_GET_BITS(reg_value, BMA580_SELF_WAKE_UP_STAT);
            config->vad_stat = BMA5_GET_BITS(reg_value, BMA580_VAD_STAT);
        }
    }

    return result;
}

int8_t bma580_get_aux_data(uint16_t *aux_data, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary buffer to receive the serial data from sensor */
    uint8_t data[2] = { 0 };

    if (NULL == aux_data)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA580_REG_AUX_DATA_0, (uint8_t *)data, sizeof(data), dev);
        if (BMA5_OK == result)
        {
            /* Group the serial data to get needed detail */
            *aux_data = (uint16_t)(((uint16_t)data[1] << 8) | ((uint16_t)data[0]));
        }
    }

    return result;
}
