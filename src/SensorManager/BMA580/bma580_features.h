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
* @file       bma580_features.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA580_FEATURES_H
#define _BMA580_FEATURES_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/

#include "bma580.h"
#include "bma580_context.h"

/******************************************************************************/
/********************** Register macros for bit masking ***********************/
/******************************************************************************/

/*! Android compatibility mode macros */
#define BMA580_ANDROID_COMP_MSK                    UINT8_C(0x01)

/*! Feature axis macros */
#define BMA580_FEAT_AXIS_EX_MSK                    UINT8_C(0x0E)
#define BMA580_FEAT_AXIS_EX_POS                    UINT8_C(1)

#define BMA580_FEAT_X_INV_MSK                      UINT8_C(0x10)
#define BMA580_FEAT_X_INV_POS                      UINT8_C(4)

#define BMA580_FEAT_Y_INV_MSK                      UINT8_C(0x20)
#define BMA580_FEAT_Y_INV_POS                      UINT8_C(5)

#define BMA580_FEAT_Z_INV_MSK                      UINT8_C(0x40)
#define BMA580_FEAT_Z_INV_POS                      UINT8_C(6)

/*! Feature conf error macros */
#define BMA580_GEN_INT1_CONF_ERR_MSK               UINT8_C(0x01)
#define BMA580_GEN_INT1_CONF_ERR_POS               UINT8_C(0)

#define BMA580_GEN_INT2_CONF_ERR_MSK               UINT8_C(0x02)
#define BMA580_GEN_INT2_CONF_ERR_POS               UINT8_C(1)

#define BMA580_GEN_INT3_CONF_ERR_MSK               UINT8_C(0x04)
#define BMA580_GEN_INT3_CONF_ERR_POS               UINT8_C(2)

#define BMA580_ACC_FOC_CONF_ERR_MSK                UINT8_C(0x08)
#define BMA580_ACC_FOC_CONF_ERR_POS                UINT8_C(3)

#define BMA580_TAP_CONF_ERR_MSK                    UINT8_C(0x10)
#define BMA580_TAP_CONF_ERR_POS                    UINT8_C(4)

#define BMA580_VAD_CONF_ERR_MSK                    UINT8_C(0x20)
#define BMA580_VAD_CONF_ERR_POS                    UINT8_C(5)

#define BMA580_SELF_WAKE_UP_ERR_MSK                UINT8_C(0x40)
#define BMA580_SELF_WAKE_UP_ERR_POS                UINT8_C(6)

/*! Generic interrupt macors */
/*! Minimum/maximum slope of acceleration signal for interrupt detection based on selected motion criterion. */
#define BMA580_GEN_INT_SLOPE_THRES_MSK             UINT16_C(0x0FFF)
#define BMA580_GEN_INT_SLOPE_THRES_POS             UINT8_C(0)

/*! Logical evaluation condition between enabled axis status */
#define BMA580_GEN_INT_COMB_SEL_MSK                UINT16_C(0x1000)
#define BMA580_GEN_INT_COMB_SEL_POS                UINT8_C(12)

/*! Enabling of axis for generic interrupt detection */
#define BMA580_GEN_INT_AXIS_SEL_MSK                UINT16_C(0xE000)
#define BMA580_GEN_INT_AXIS_SEL_POS                UINT8_C(13)

/*! Hysteresis for the slope of the acceleration signal */
#define BMA580_GEN_INT_HYST_MSK                    UINT16_C(0x03FF)
#define BMA580_GEN_INT_HYST_POS                    UINT8_C(0)

/*! Logical evaluation condition between enabled axis status */
#define BMA580_GEN_INT_CRIT_SEL_MSK                UINT16_C(0x0400)
#define BMA580_GEN_INT_CRIT_SEL_POS                UINT8_C(10)

/*! Mode of the acceleration reference update */
#define BMA580_GEN_INT_ACC_REF_UP_MSK              UINT16_C(0x1800)
#define BMA580_GEN_INT_ACC_REF_UP_POS              UINT8_C(11)

/*! Minimum duration for which the selected criterion is true for interrupt detection. */
#define BMA580_GEN_INT_DURATION_MSK                UINT16_C(0x1FFF)
#define BMA580_GEN_INT_DURATION_POS                UINT8_C(0)

/*! Wait time for clearing the event after condition evaluates false */
#define BMA580_GEN_INT_WAIT_TIME_MSK               UINT16_C(0xE000)
#define BMA580_GEN_INT_WAIT_TIME_POS               UINT8_C(13)

/*! Quiet time after an interrupt where no additional interrupts are detected */
#define BMA580_GEN_INT_QUIET_TIME_MSK              UINT16_C(0x1FFF)
#define BMA580_GEN_INT_QUIET_TIME_POS              UINT8_C(0)

/*! Reference acceleration signal for x-axis */
#define BMA580_GEN_INT_REF_ACC_X_MSK               UINT16_C(0xFFFF)
#define BMA580_GEN_INT_REF_ACC_X_POS               UINT8_C(0)

/*! Reference acceleration signal for y-axis */
#define BMA580_GEN_INT_REF_ACC_Y_MSK               UINT16_C(0xFFFF)
#define BMA580_GEN_INT_REF_ACC_Y_POS               UINT8_C(0)

/*! Reference acceleration signal for z-axis */
#define BMA580_GEN_INT_REF_ACC_Z_MSK               UINT16_C(0xFFFF)
#define BMA580_GEN_INT_REF_ACC_Z_POS               UINT8_C(0)

/*! Tap detector configuration macros */
#define BMA580_DEFAULT_S_TAP_EN                    UINT8_C(1)
#define BMA580_DEFAULT_D_TAP_EN                    UINT8_C(1)
#define BMA580_DEFAULT_T_TAP_EN                    UINT8_C(1)

#define BMA580_TAP_AXIS_SEL_MSK                    UINT16_C(0x0003)
#define BMA580_TAP_AXIS_SEL_POS                    UINT8_C(0x00)

#define BMA580_TAP_WAIT_FOR_TIMEOUT_MSK            UINT16_C(0x0004)
#define BMA580_TAP_WAIT_FOR_TIMEOUT_POS            UINT8_C(0x02)

#define BMA580_TAP_MAX_PEAKS_FOR_TAP_MSK           UINT16_C(0x0038)
#define BMA580_TAP_MAX_PEAKS_FOR_TAP_POS           UINT8_C(0x03)

#define BMA580_TAP_MODE_MSK                        UINT16_C(0x00C0)
#define BMA580_TAP_MODE_POS                        UINT8_C(0x06)

#define BMA580_TAP_SINGLE_EN_MSK                   UINT16_C(0x0100)
#define BMA580_TAP_SINGLE_EN_POS                   UINT8_C(0x08)

#define BMA580_TAP_DOUBLE_EN_MSK                   UINT16_C(0x0200)
#define BMA580_TAP_DOUBLE_EN_POS                   UINT8_C(0x09)

#define BMA580_TAP_TRIBLE_EN_MSK                   UINT16_C(0x0400)
#define BMA580_TAP_TRIBLE_EN_POS                   UINT8_C(0x0A)

#define BMA580_TAP_PEAK_THRES_MSK                  UINT16_C(0x03FF)
#define BMA580_TAP_PEAK_THRES_POS                  UINT8_C(0x00)

#define BMA580_TAP_MAX_GESTURE_DUR_MSK             UINT16_C(0xFC00)
#define BMA580_TAP_MAX_GESTURE_DUR_POS             UINT8_C(0x0A)

#define BMA580_TAP_MAX_DUR_BET_PEAKS_MSK           UINT16_C(0x000F)
#define BMA580_TAP_MAX_DUR_BET_PEAKS_POS           UINT8_C(0x00)

#define BMA580_TAP_SHOCK_DET_DUR_MSK               UINT16_C(0x00F0)
#define BMA580_TAP_SHOCK_DET_DUR_POS               UINT8_C(0x04)

#define BMA580_TAP_MIN_QUITE_DUR_MSK               UINT16_C(0x0F00)
#define BMA580_TAP_MIN_QUITE_DUR_POS               UINT8_C(0x08)

#define BMA580_TAP_QUITE_TIME_MSK                  UINT16_C(0xF000)
#define BMA580_TAP_QUITE_TIME_POS                  UINT8_C(0x0C)

/*! Accel foc configuration macros */
#define BMA580_ACC_FOC_OFF_X_MSK                   UINT16_C(0x01FF)
#define BMA580_ACC_FOC_OFF_X_POS                   UINT8_C(0)

#define BMA580_ACC_FOC_OFF_Y_MSK                   UINT16_C(0x01FF)
#define BMA580_ACC_FOC_OFF_Y_POS                   UINT8_C(0)

#define BMA580_ACC_FOC_OFF_Z_MSK                   UINT16_C(0x01FF)
#define BMA580_ACC_FOC_OFF_Z_POS                   UINT8_C(0)

#define BMA580_ACC_FOC_APPLY_CORR_MSK              UINT16_C(0x0001)
#define BMA580_ACC_FOC_APPLY_CORR_POS              UINT8_C(0)

#define BMA580_ACC_FOC_FILTER_COEFF_MSK            UINT16_C(0x000E)
#define BMA580_ACC_FOC_FILTER_COEFF_POS            UINT8_C(1)

#define BMA580_ACC_FOC_AXIS_1G_MSK                 UINT16_C(0x0070)
#define BMA580_ACC_FOC_AXIS_1G_POS                 UINT8_C(4)

/*! Self wake-up macros */
#define BMA580_SELF_WAKE_UP_ACC_ODR_MSK            UINT8_C(0x0F)
#define BMA580_SELF_WAKE_UP_ACC_ODR_POS            UINT8_C(0)

#define BMA580_SELF_WAKE_UP_ACC_BWP_MSK            UINT8_C(0x70)
#define BMA580_SELF_WAKE_UP_ACC_BWP_POS            UINT8_C(4)

#define BMA580_SELF_WAKE_UP_ACC_PERF_MODE_MSK      UINT8_C(0x80)
#define BMA580_SELF_WAKE_UP_ACC_PERF_MODE_POS      UINT8_C(7)

/*! acc_perf_mode = 1 -> reserved; acc_perf_mode = 0 -> average 16 samples. */
#define BMA580_SELF_WAKE_UP_ACC_BWP_RES_AVG16      UINT8_C(0x04)

/*! acc_perf_mode = 1 -> reserved; acc_perf_mode = 0 -> average 32 samples. */
#define BMA580_SELF_WAKE_UP_ACC_BWP_RES_AVG32      UINT8_C(0x05)

/*! acc_perf_mode = 1 -> reserved; acc_perf_mode = 0 -> average 64 samples. */
#define BMA580_SELF_WAKE_UP_ACC_BWP_RES_AVG64      UINT8_C(0x06)

/*! Duty Cycling mode (CIC averaging). */
#define BMA580_SELF_WAKE_UP_ACC_PERF_MODE_CIC_AVG  UINT8_C(0x00)

/*! Continuous mode (IIR filtering). */
#define BMA580_SELF_WAKE_UP_ACC_PERF_MODE_CONT     UINT8_C(0x01)

/*! VAD Macros */
#define BMA580_VAD_EN_VAD_X_MSK                    UINT8_C(0x01)
#define BMA580_VAD_EN_VAD_X_POS                    UINT8_C(0x00)

#define BMA580_VAD_EN_VAD_Y_MSK                    UINT8_C(0x02)
#define BMA580_VAD_EN_VAD_Y_POS                    UINT8_C(0x01)

#define BMA580_VAD_EN_VAD_Z_MSK                    UINT8_C(0x04)
#define BMA580_VAD_EN_VAD_Z_POS                    UINT8_C(0x02)

/******************************************************************************/
/********************* Macros for supported field values **********************/
/******************************************************************************/

#define BMA580_GEN_INT_COMB_SEL_LOGICAL_OR         UINT8_C(0x00)
#define BMA580_GEN_INT_COMB_SEL_LOGICAL_AND        UINT8_C(0x01)

#define BMA580_GEN_INT_CRI_SEL_INACT               UINT8_C(0x00)
#define BMA580_GEN_INT_CRI_SEL_ACT                 UINT8_C(0x01)

#define BMA580_GEN_INT_ACC_REF_UP_ON_EVENT         UINT8_C(0x00)
#define BMA580_GEN_INT_ACC_REF_UP_ON_ALWAYS        UINT8_C(0x01)
#define BMA580_GEN_INT_ACC_REF_UP_ON_MANUAL        UINT8_C(0x02)

#define BMA580_FEAT_AXIS_EX_DEFAULT_0              UINT8_C(0x00)
#define BMA580_FEAT_AXIS_EX_YXZ                    UINT8_C(0x01)
#define BMA580_FEAT_AXIS_EX_XZY                    UINT8_C(0x02)
#define BMA580_FEAT_AXIS_EX_ZXY                    UINT8_C(0x03)
#define BMA580_FEAT_AXIS_EX_YZX                    UINT8_C(0x04)
#define BMA580_FEAT_AXIS_EX_ZYX                    UINT8_C(0x05)
#define BMA580_FEAT_AXIS_EX_DEFAULT_6              UINT8_C(0x06)
#define BMA580_FEAT_AXIS_EX_DEFAULT_7              UINT8_C(0x07)

#define BMA580_FEAT_X_INV_DEFAULT                  UINT8_C(0)
#define BMA580_FEAT_X_INV_INVERT                   UINT8_C(1)

#define BMA580_FEAT_Y_INV_DEFAULT                  UINT8_C(0)
#define BMA580_FEAT_Y_INV_INVERT                   UINT8_C(1)

#define BMA580_FEAT_Z_INV_DEFAULT                  UINT8_C(0)
#define BMA580_FEAT_Z_INV_INVERT                   UINT8_C(1)

#define BMA580_AXIS_SEL_DEFAULT                    UINT8_C(0x07)
#define BMA580_DURATION_DEFAULT                    UINT8_C(0x0A)
#define BMA580_WAIT_TIME_DEFAULT                   UINT8_C(0x03)
#define BMA580_QUIET_TIME_DEFAULT                  UINT8_C(0x40)
#define BMA580_ACC_REF_X_DEFAULT                   UINT8_C(0x00)
#define BMA580_ACC_REF_Y_DEFAULT                   UINT8_C(0x00)
#define BMA580_ACC_REF_Z_DEFAULT                   UINT8_C(0x800)

#define BMA580_TAP_SEL_AXIS_X                      UINT8_C(0)
#define BMA580_TAP_SEL_AXIS_Y                      UINT8_C(1)
#define BMA580_TAP_SEL_AXIS_Z                      UINT8_C(2)

#define BMA580_TAP_MODE_SENSITIVE                  UINT8_C(0)
#define BMA580_TAP_MODE_NORMAL                     UINT8_C(1)
#define BMA580_TAP_MODE_ROBUST                     UINT8_C(2)

/*! Feature interrupts base address definitions */
#define BMA580_BASE_ADDR_CONFIG_VERSION            UINT8_C(0x00)
#define BMA580_BASE_ADDR_FEAT_CONF_ERR             UINT8_C(0x02)
#define BMA580_BASE_ADDR_GENERAL_SETTINGS          UINT8_C(0x03)
#define BMA580_BASE_ADDR_GENERIC_INT1              UINT8_C(0x04)
#define BMA580_BASE_ADDR_GENERIC_INT2              UINT8_C(0x0B)
#define BMA580_BASE_ADDR_GENERIC_INT3              UINT8_C(0x12)
#define BMA580_BASE_ADDR_TAP_DETECTOR              UINT8_C(0x19)
#define BMA580_BASE_ADDR_VAD                       UINT8_C(0x1C)
#define BMA580_BASE_ADDR_SELF_WAKE_UP              UINT8_C(0x24)
#define BMA580_BASE_ADDR_ACC_FOC                   UINT8_C(0x25)

/*! Error status of accel config for features */
#define BMA580_GEN_INT1_CONF_ERR_OKAY              UINT8_C(0)
#define BMA580_GEN_INT1_CONF_ERR_ERR               UINT8_C(1)
#define BMA580_GEN_INT2_CONF_ERR_OKAY              UINT8_C(0)
#define BMA580_GEN_INT2_CONF_ERR_ERR               UINT8_C(1)
#define BMA580_GEN_INT3_CONF_ERR_OKAY              UINT8_C(0)
#define BMA580_GEN_INT3_CONF_ERR_ERR               UINT8_C(1)
#define BMA580_ACC_FOC_CONF_ERR_OKAY               UINT8_C(0)
#define BMA580_ACC_FOC_CONF_ERR_ERR                UINT8_C(1)
#define BMA580_TAP_CONF_ERR_OKAY                   UINT8_C(0)
#define BMA580_TAP_CONF_ERR_ERR                    UINT8_C(1)
#define BMA580_VAD_CONF_ERR_OKAY                   UINT8_C(0)
#define BMA580_VAD_CONF_ERR_ERR                    UINT8_C(1)
#define BMA580_SELF_WAKE_UP_ERR_OKAY               UINT8_C(0)
#define BMA580_SELF_WAKE_UP_ERR_ERR                UINT8_C(1)

/*! Accel foc axis 1G macros */
#define BMA580_ACC_FOC_AXIS_Z_PLUS_1G              UINT8_C(0)
#define BMA580_ACC_FOC_AXIS_Z_MINUS_1G             UINT8_C(1)
#define BMA580_ACC_FOC_AXIS_Y_PLUS_1G              UINT8_C(2)
#define BMA580_ACC_FOC_AXIS_Y_MINUS_1G             UINT8_C(3)
#define BMA580_ACC_FOC_AXIS_X_PLUS_1G              UINT8_C(4)
#define BMA580_ACC_FOC_AXIS_X_MINUS_1G             UINT8_C(5)

/*! Feature axis Exchange macros */
#define BMA580_FEAT_AXIS_EX_SEL_X                  UINT8_C(0x01)
#define BMA580_FEAT_AXIS_EX_SEL_Y                  UINT8_C(0x02)
#define BMA580_FEAT_AXIS_EX_SEL_Z                  UINT8_C(0x04)
#define BMA580_FEAT_AXIS_EX_SEL_XYZ                UINT8_C(0x07)

/*! Soft-reset delay is 2ms */
#define BMA580_SOFT_RESET_DELAY                    UINT16_C(2000)

#define BMA580_GEN_INT_1                           UINT8_C(0)
#define BMA580_GEN_INT_2                           UINT8_C(1)
#define BMA580_GEN_INT_3                           UINT8_C(2)

/******************************************************************************/
/***************** Structures for handling register content *******************/
/******************************************************************************/

/*!
 *  @brief  Bits reflects the error status of accel config for features
 */
struct bma580_feat_conf_err
{
    /*! Internal filter cannot produce enough samples for generic interrupt 1 feature, or this feature is enabled in
     * parallel to VAD feature  */
    uint8_t gen_int1_conf_err;

    /*! Internal filter cannot produce enough samples for generic interrupt 2 feature, or this feature is enabled in
     * parallel to VAD feature */
    uint8_t gen_int2_conf_err;

    /*! Internal filter cannot produce enough samples for generic interrupt 3 feature, or this feature is enabled in
     * parallel to VAD feature */
    uint8_t gen_int3_conf_err;

    /*! Internal filter cannot produce enough samples for accelerometer fast-offset compensation feature */
    uint8_t acc_foc_conf_err;

    /*! Internal filter cannot produce enough samples for tap detection feature, or this feature is enabled in parallel
     * to VAD feature*/
    uint8_t tap_conf_err;

    /*! VAD feature is enabled in parallel to generic interrupt or tap feature.*/
    uint8_t vad_conf_err;

    /*!Self wake-up configuration is invalid.*/
    uint8_t self_wake_up_err;
};

/*!
 *  @brief Structure to store feature axis config
 */
struct bma580_feat_axis
{
    /*! Axes exchange scheme that is applied in host software  */
    uint8_t feat_axis_ex;

    /*! Invert polarity of X-axis data after axis exchange */
    uint8_t feat_x_inv;

    /*! Invert polarity of Y-axis data after axis exchange */
    uint8_t feat_y_inv;

    /*! Invert polarity of Z-axis data after axis exchange */
    uint8_t feat_z_inv;

    /*! This setting should only be set, if VAD feature is used.*/
    uint8_t vad_acc_data_src;
};

/*!
 *  @brief Structure to store generic interrupt config
 */
struct bma580_generic_interrupt
{
    /*! Minimum/maximum slope of acceleration signal for interrupt detection based on selected motion criterion. */
    uint16_t slope_thres;

    /*! Logical evaluation condition between enabled axis status
     * BMA580_GEN_INT_COMB_SEL_LOGICAL_OR   - 0x00
     * BMA580_GEN_INT_COMB_SEL_LOGICAL_AND  - 0x01
     */
    uint8_t comb_sel;

    /*! Enabling of axis for generic interrupt detection */
    uint8_t axis_sel;

    /*! Hysteresis for the slope of the acceleration signal */
    uint16_t hysteresis;

    /*! Logical evaluation condition between enabled axis status
     * BMA580_GEN_INT_CRI_SEL_INACT - 0x00
     * BMA580_GEN_INT_CRI_SEL_ACT   - 0x01
     */
    uint8_t criterion_sel;

    /*! Mode of the acceleration reference update
     * BMA580_GEN_INT_ACC_REF_UP_ON_EVENT   -  0x00
     * BMA580_GEN_INT_ACC_REF_UP_ON_ALWAYS  -  0x01
     * BMA580_GEN_INT_ACC_REF_UP_ON_MANUAL  -  0x02
     */
    uint8_t acc_ref_up;

    /*! Minimum duration for which the selected criterion is true for interrupt detection. */
    uint16_t duration;

    /*! Wait time for clearing the event after condition evaluates false */
    uint8_t wait_time;

    /*! Quiet time after an interrupt where no additional interrupts are detected */
    uint16_t quiet_time;

    /*! Reference acceleration signal for x-axis */
    int16_t ref_acc_x;

    /*! Reference acceleration signal for y-axis */
    int16_t ref_acc_y;

    /*! Reference acceleration signal for z-axis */
    int16_t ref_acc_z;
};

/*!
 *  @brief Structure to holding generic interrupt configuration.
 */
struct bma580_generic_interrupt_types
{
    /*! Specify generic interrupt 1, 2, or 3 */
    uint8_t generic_interrupt;

    /*! Generic Interrupt Configuration */
    struct bma580_generic_interrupt gen_int;
};

/*!
 *  @brief Structure to store accel FOC config
 */
struct bma580_accel_foc_config
{
    /*! Accel foc offset x-axis */
    uint16_t foc_off_x;

    /*! Accel foc offset y-axis */
    uint16_t foc_off_y;

    /*! Accel foc offset z-axis */
    uint16_t foc_off_z;

    /*! Accel foc correlation */
    uint8_t foc_apply_corr;

    /*! Accel foc filter coefficient */
    uint8_t foc_filter_coeff;

    /*! Accel foc axis for 1G
     * BMA580_ACC_FOC_AXIS_Z_PLUS_1G   -   0x00
     * BMA580_ACC_FOC_AXIS_Z_MINUS_1G   -   0x01
     * BMA580_ACC_FOC_AXIS_Y_PLUS_1G   -   0x02
     * BMA580_ACC_FOC_AXIS_Y_MINUS_1G   -   0x03
     * BMA580_ACC_FOC_AXIS_X_PLUS_1G   -   0x04
     * BMA580_ACC_FOC_AXIS_X_MINUS_1G   -   0x05
     */
    uint8_t foc_axis_1g;
};

/*!
 *  @brief Structure to store tap config
 */
struct bma580_tap_config
{
    /*! Dominant sensing axis of accelerometer along which tap gesture is performed */
    uint8_t axis_sel;

    /*! Perform gesture confirmation with wait time set by max_gesture_dur */
    uint8_t wait_for_timeout;

    /*! Maximum number of threshold crossing expected around a tap */
    uint8_t max_peaks_for_tap;

    /*! Mode for detection of tap gesture. Default value = Normal */
    uint8_t mode;

    /*! Enable single tap feature */
    uint8_t s_tap_en;

    /*! Enable double tap feature */
    uint8_t d_tap_en;

    /*! Enable triple tap feature */
    uint8_t t_tap_en;

    /*! Minimum threshold for peak resulting from the tap */
    uint16_t tap_peak_thres;

    /*! Maximum duration from first tap within the second and/or third tap is expected to happen */
    uint8_t max_gesture_dur;

    /*! Maximum duration between positive and negative peaks to tap */
    uint8_t max_dur_between_peaks;

    /*! Maximum duration for which tap impact is observed */
    uint8_t tap_shock_settling_dur;

    /*! Mimimum duration between two consecutive tap impact */
    uint8_t min_quite_dur_between_taps;

    /*! Minimum quite duration between two gestures */
    uint8_t quite_time_after_gesture;
};

/*!
 *  @brief Structure to store self wake-up config
 */
struct bma580_self_wakeup_config
{
    /*! ODR in Hz */
    uint8_t acc_odr;

    /*! Bandwidth parameter, determines filter configuration */
    uint8_t acc_bwp;

    /*! Performance mode configuration */
    uint8_t acc_perf_mode;
};

/*!
 *  @brief Structure to store vad config
 */
struct bma580_vad_config
{
    /*! Enable detection of voice activity for x axis */
    uint8_t en_vad_x;

    /*! Enable detection of voice activity for y axis */
    uint8_t en_vad_y;

    /*! Enable detection of voice activity for z axis */
    uint8_t en_vad_z;
};

/******************************************************************************/
/********************** Function prototype declarations ***********************/
/******************************************************************************/

/**
 * \ingroup bma580
 * \defgroup bma580ApiInit BMA580 Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma580ApiInit
 * \page bma580_api_bma580_init bma580_init
 * \code
 * int8_t bma580_init(struct bma5_dev *dev);
 * \endcode
 * @details This API reads the chip-id of the sensor which is the first step to
 * verify the sensor and also it configures the read mechanism of SPI and
 * I2C interface. As this API is the entry point, call this API before using other APIs.
 *
 * @param[in,out] dev : Structure instance of bma5_dev
 *
 * @return Result of API execution status
 *
 * @retval Zero     -> Success
 * @retval Positive -> Warning
 * @retval Negative -> Error/Failure
 */
int8_t bma580_init(struct bma5_dev *dev);

/**
 * \ingroup bma580
 * \defgroup bma580FeatApiRegs BMA580 Feature configuration registers
 * @brief Set / Get data from the given Feature configuration register address of the sensor
 */

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_feature_axis_config bma580_set_feature_axis_config
 * \code
 * int8_t bma580_set_feature_axis_config(const struct bma580_feat_axis *feat_axis, struct bma5_dev *dev);
 * \endcode
 * @details This API sets feature axis configurations.
 *
 * @param[in] feat_axis      : Structure instance of bma580_feat_axis.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_feature_axis_config(const struct bma580_feat_axis *feat_axis, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_feature_axis_config bma580_get_feature_axis_config
 * \code
 * int8_t bma580_get_feature_axis_config(struct bma580_feat_axis *feat_axis, struct bma5_dev *dev);
 * \endcode
 * @details This API gets feature axis configurations.
 *
 * @param[out] feat_axis      : Structure instance of bma580_feat_axis.
 * @param[in,out] dev         : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_feature_axis_config(struct bma580_feat_axis *feat_axis, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_feat_conf_err bma580_set_feat_conf_err
 * \code
 * int8_t bma580_set_feat_conf_err(const struct bma580_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
 * \endcode
 * @details This API sets Bits which reflects the error status of accel config for features.
 *
 * @param[in] feat_conf_err      : Structure instance of bma580_feat_conf_err.
 * @param[in] dev                : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_feat_conf_err(const struct bma580_feat_conf_err *feat_conf_err, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_feat_conf_err bma580_get_feat_conf_err
 * \code
 * int8_t bma580_get_feat_conf_err(const struct bma580_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
 * \endcode
 * @details This API gets Bits which reflects the error status of accel config for features.
 *
 * @param[in] feat_conf_err      : Structure instance of bma580_feat_conf_err.
 * @param[in] dev                : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_feat_conf_err(struct bma580_feat_conf_err *feat_conf_err, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_default_generic_int_config bma580_get_default_generic_int_config
 * \code
 * int8_t bma580_get_default_generic_int_1_config(struct bma580_generic_interrupt_types *gen_int, uint8_t n_ints,  struct bma5_dev *dev);
 * \endcode
 * @details This API gets default values generic interrupt configurations.
 *
 * @param[out] gen_int         : Structure instance of bma580_generic_interrupt types.
 * @param[in]  n_ints          : Number of interrupt sources to iterate through.
 * @param[in,out] dev          : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_default_generic_int_config(struct bma580_generic_interrupt_types *gen_int,
                                             uint8_t n_ints,
                                             struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_generic_int_config bma580_set_generic_int_config
 * \code
 * int8_t bma580_set_generic_int_config(const struct bma580_generic_interrupt_types *gen_int, uint8_t n_ints, struct bma5_dev *dev);
 * \endcode
 * @details This API sets generic interrupt 1 configurations.
 *
 * @param[in] gen_int      : Structure instance of bma580_generic_interrupt types.
 * @param[in]  n_ints          : Number of interrupt sources to iterate through.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_generic_int_config(const struct bma580_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_generic_int_config bma580_get_generic_int_config
 * \code
 * int8_t bma580_get_generic_int_config(struct bma580_generic_interrupt_types *gen_int, uint8_t n_ints, struct bma5_dev *dev);
 * \endcode
 * @details This API gets generic interrupt configurations.
 *
 * @param[out] gen_int      : Structure instance of bma580_generic_interrupt.
 * @param[in]  n_ints          : Number of interrupt sources to iterate through.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_generic_int_config(struct bma580_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_accel_foc_config bma580_set_accel_foc_config
 * \code
 * int8_t bma580_set_accel_foc_config(const struct bma580_accel_foc_config *acc_foc, struct bma5_dev *dev);
 * \endcode
 * @details This API sets accel foc configuration
 *
 * @param[in] acc_foc        : Structure instance of bma580_accel_foc_config.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_accel_foc_config(const struct bma580_accel_foc_config *acc_foc, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_accel_foc_config bma580_get_accel_foc_config
 * \code
 * int8_t bma580_get_accel_foc_config(struct bma580_accel_foc_config *acc_foc, struct bma5_dev *dev);
 * \endcode
 * @details This API gets accel foc configuration
 *
 * @param[out] acc_foc        : Structure instance of bma580_accel_foc_config.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_accel_foc_config(struct bma580_accel_foc_config *acc_foc, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_tap_config bma580_set_tap_config
 * \code
 * int8_t bma580_set_tap_config(const struct bma580_tap_config *tap_config, struct bma5_dev *dev);
 * \endcode
 * @details This API sets tap configurations
 *
 * @param[in] tap_config     : Structure instance of bma580_tap_config.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_tap_config(const struct bma580_tap_config *tap_config, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_tap_config bma580_get_tap_config
 * \code
 * int8_t bma580_get_tap_config(struct bma580_tap_config *tap_config, struct bma5_dev *dev);
 * \endcode
 * @details This API gets tap configurations
 *
 * @param[out] tap_config        : Structure instance of bma580_tap_config.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_tap_config(struct bma580_tap_config *tap_config, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_default_tap_config bma580_get_tap_config
 * \code
 * int8_t bma580_get_default_tap_config(struct bma580_tap_config *tap_config, struct bma5_dev *dev);
 * \endcode
 * @details This API gets default tap configurations
 *
 * @param[out] tap_config        : Structure instance of bma580_tap_config.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_default_tap_config(struct bma580_tap_config *tap_config, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_self_wakeup_config bma580_set_self_wakeup_config
 * \code
 * int8_t bma580_set_self_wakeup_config(const struct bma580_self_wakeup_config *self_wakeup_config, struct bma5_dev *dev);
 * \endcode
 * @details This API sets self wake-up configurations
 *
 * @param[in] self_wakeup_config     : Structure instance of bma580_self_wakeup_config.
 * @param[in] dev                    : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_self_wakeup_config(const struct bma580_self_wakeup_config *self_wakeup_config, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_self_wakeup_config bma580_get_self_wakeup_config
 * \code
 * int8_t bma580_get_self_wakeup_config(struct bma580_self_wakeup_config *self_wakeup_config, struct bma5_dev *dev);
 * \endcode
 * @details This API gets self wake-up configurations
 *
 * @param[out] self_wakeup_config        : Structure instance of bma580_self_wakeup_config.
 * @param[in,out] dev                    : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_self_wakeup_config(struct bma580_self_wakeup_config *self_wakeup_config, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_set_vad_config bma580_set_vad_config
 * \code
 * int8_t bma580_set_vad_config(const struct bma580_vad_config *vad_config, struct bma5_dev *dev);
 * \endcode
 * @details This API sets vad configurations
 *
 * @param[in] vad_config     : Structure instance of bma580_vad_config.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_vad_config(const struct bma580_vad_config *vad_config, struct bma5_dev *dev);

/*!
 * \ingroup bma580FeatApiRegs
 * \page bma580_api_bma580_get_vad_config bma580_get_vad_config
 * \code
 * int8_t bma580_get_vad_config(struct bma580_vad_config *vad_config, struct bma5_dev *dev);
 * \endcode
 * @details This API gets vad configurations
 *
 * @param[out] vad_config        : Structure instance of bma580_vad_config.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_vad_config(struct bma580_vad_config *vad_config, struct bma5_dev *dev);

/**
 * \ingroup bma580
 * \defgroup bma580ApiSR BMA580 Soft-reset
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bma580ApiSR
 * \page bma580_api_bma580_soft_reset bma580_soft_reset
 * \code
 * int8_t bma580_soft_reset(struct bma5_dev *dev);
 * \endcode
 * @details This API resets sensor. All registers are overwritten with
 * their default values.
 *
 * @param[in] dev : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_soft_reset(struct bma5_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMA580_FEATURES_H */
