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
* @file       bma580.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA580_H
#define _BMA580_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#include  "bma5.h"

/******************************************************************************/
/************************* General macro definitions **************************/
/******************************************************************************/
/*! Chip ID of BMA580 */
#define BMA580_CHIP_ID                        UINT8_C(0xC4)

/******************************************************************************/
/***************************** Register addresses *****************************/
/******************************************************************************/
/*! The product chip_id. Upper four bits are fix and lower four bits are boot loaded from OTP. */
#define BMA580_REG_CHIP_ID                    UINT8_C(0x00)

/*! INT1 interrupt status register 0 */
#define BMA580_REG_INT_STATUS_INT1_0          UINT8_C(0x12)

/*! INT1 interrupt status register 1 */
#define BMA580_REG_INT_STATUS_INT1_1          UINT8_C(0x13)

/*! INT2 interrupt status register 0 */
#define BMA580_REG_INT_STATUS_INT2_0          UINT8_C(0x14)

/*! INT2 interrupt status register 1 */
#define BMA580_REG_INT_STATUS_INT2_1          UINT8_C(0x15)

/*! I3C interrupt status register 0 */
#define BMA580_REG_INT_STATUS_I3C_0           UINT8_C(0x16)

/*! I3C interrupt status register 1 */
#define BMA580_REG_INT_STATUS_I3C_1           UINT8_C(0x17)

/*! Auxilary data */
#define BMA580_REG_AUX_DATA_0                 UINT8_C(0x2B)

/*! Auxilary data */
#define BMA580_REG_AUX_DATA_1                 UINT8_C(0x2C)

/*! Interrupt mapping register 0 */
#define BMA580_REG_INT_MAP_0                  UINT8_C(0x36)

/*! Interrupt mapping register 1 */
#define BMA580_REG_INT_MAP_1                  UINT8_C(0x37)

/*! Interrupt mapping register 2 */
#define BMA580_REG_INT_MAP_2                  UINT8_C(0x38)

/*! Interrupt mapping register 3 */
#define BMA580_REG_INT_MAP_3                  UINT8_C(0x39)

/*! feature engine ('feat_eng') general purpose flags */
#define BMA580_REG_FEAT_ENG_GP_FLAGS          UINT8_C(0x52)

/*! feature engine ('feat_eng') general purpose register 0 */
#define BMA580_REG_FEAT_ENG_GPR_0             UINT8_C(0x55)

/*! feature engine ('feat_eng') general purpose register 1 */
#define BMA580_REG_FEAT_ENG_GPR_1             UINT8_C(0x56)

/*! feature engine ('feat_eng') general purpose register 2 */
#define BMA580_REG_FEAT_ENG_GPR_2             UINT8_C(0x57)

/******************************************************************************/
/********************** Register macros for bit masking ***********************/
/******************************************************************************/
/*! Chip id */
#define BMA580_CHIP_ID_MSK                    UINT8_C(0xFF)
#define BMA580_CHIP_ID_POS                    UINT8_C(0x00)

/*|
 * @brief Mask and Position values for Interrupt Status for INT1, INT2, I3C Interrupts
 */

/*! Accelerometer data ready interrupt status */
#define BMA580_ACC_DRDY_INT_STATUS_MSK        UINT8_C(0x01)
#define BMA580_ACC_DRDY_INT_STATUS_POS        UINT8_C(0x00)

/*! FIFO watermark interrupt status */
#define BMA580_FIFO_WM_INT_STATUS_MSK         UINT8_C(0x02)
#define BMA580_FIFO_WM_INT_STATUS_POS         UINT8_C(0x01)

/*! FIFO full interrupt status */
#define BMA580_FIFO_FULL_INT_STATUS_MSK       UINT8_C(0x04)
#define BMA580_FIFO_FULL_INT_STATUS_POS       UINT8_C(0x02)

/*! Generic interrupt 1 interrupt status */
#define BMA580_GEN_INT1_INT_STATUS_MSK        UINT8_C(0x08)
#define BMA580_GEN_INT1_INT_STATUS_POS        UINT8_C(0x03)

/*! Generic interrupt 2 interrupt status */
#define BMA580_GEN_INT2_INT_STATUS_MSK        UINT8_C(0x10)
#define BMA580_GEN_INT2_INT_STATUS_POS        UINT8_C(0x04)

/*! Generic interrupt 3 interrupt status */
#define BMA580_GEN_INT3_INT_STATUS_MSK        UINT8_C(0x20)
#define BMA580_GEN_INT3_INT_STATUS_POS        UINT8_C(0x05)

/*! Accelerometer fast offset compensation interrupt status */
#define BMA580_ACC_FOC_INT_STATUS_MSK         UINT8_C(0x40)
#define BMA580_ACC_FOC_INT_STATUS_POS         UINT8_C(0x06)

/*! Single tap interrupt status */
#define BMA580_STAP_INT_STATUS_MSK            UINT8_C(0x80)
#define BMA580_STAP_INT_STATUS_POS            UINT8_C(0x07)

/*! Double tap interrupt status */
#define BMA580_DTAP_INT_STATUS_MSK            UINT8_C(0x01)
#define BMA580_DTAP_INT_STATUS_POS            UINT8_C(0x00)

/*! Triple tap interrupt status */
#define BMA580_TTAP_INT_STATUS_MSK            UINT8_C(0x02)
#define BMA580_TTAP_INT_STATUS_POS            UINT8_C(0x01)

/*! Voice activity detection interrupt status */
#define BMA580_VAD_INT_STATUS_MSK             UINT8_C(0x04)
#define BMA580_VAD_INT_STATUS_POS             UINT8_C(0x02)

/*! Self wake-up interrupt status */
#define BMA580_SELF_WAKE_UP_INT_STATUS_MSK    UINT8_C(0x08)
#define BMA580_SELF_WAKE_UP_INT_STATUS_POS    UINT8_C(0x03)

/*! Feature engine error interrupt status */
#define BMA580_FEAT_ENG_ERR_INT_STATUS_MSK    UINT8_C(0x10)
#define BMA580_FEAT_ENG_ERR_INT_STATUS_POS    UINT8_C(0x04)

/*! Data ready interrupt mapping */
#define BMA580_ACC_DRDY_INT_MAP_MSK           UINT8_C(0x03)
#define BMA580_ACC_DRDY_INT_MAP_POS           UINT8_C(0x00)

/*! FIFO watermark interrupt mapping */
#define BMA580_FIFO_WM_INT_MAP_MSK            UINT8_C(0x0C)
#define BMA580_FIFO_WM_INT_MAP_POS            UINT8_C(0x02)

/*! FIFO full interrupt mapping */
#define BMA580_FIFO_FULL_INT_MAP_MSK          UINT8_C(0x30)
#define BMA580_FIFO_FULL_INT_MAP_POS          UINT8_C(0x04)

/*! Generic interrupt 1 interrupt mapping */
#define BMA580_GEN_INT1_INT_MAP_MSK           UINT8_C(0xC0)
#define BMA580_GEN_INT1_INT_MAP_POS           UINT8_C(0x06)

/*! Generic interrupt 2 interrupt mapping */
#define BMA580_GEN_INT2_INT_MAP_MSK           UINT8_C(0x03)
#define BMA580_GEN_INT2_INT_MAP_POS           UINT8_C(0x00)

/*! Generic interrupt 3 interrupt mapping */
#define BMA580_GEN_INT3_INT_MAP_MSK           UINT8_C(0x0C)
#define BMA580_GEN_INT3_INT_MAP_POS           UINT8_C(0x02)

/*! Accelerometer fast offset compensation interrupt mapping */
#define BMA580_ACC_FOC_INT_MAP_MSK            UINT8_C(0x30)
#define BMA580_ACC_FOC_INT_MAP_POS            UINT8_C(0x04)

/*! Single tap interrupt mapping */
#define BMA580_STAP_INT_MAP_MSK               UINT8_C(0xC0)
#define BMA580_STAP_INT_MAP_POS               UINT8_C(0x06)

/*! Double tap interrupt mapping */
#define BMA580_DTAP_INT_MAP_MSK               UINT8_C(0x03)
#define BMA580_DTAP_INT_MAP_POS               UINT8_C(0x00)

/*! Triple tap interrupt mapping */
#define BMA580_TTAP_INT_MAP_MSK               UINT8_C(0x0C)
#define BMA580_TTAP_INT_MAP_POS               UINT8_C(0x02)

/*! Voice activity detection interrupt mapping */
#define BMA580_VAD_INT_MAP_MSK                UINT8_C(0x30)
#define BMA580_VAD_INT_MAP_POS                UINT8_C(0x04)

/*! Self wake-up interrupt mapping */
#define BMA580_SELF_WAKE_UP_INT_MAP_MSK       UINT8_C(0xC0)
#define BMA580_SELF_WAKE_UP_INT_MAP_POS       UINT8_C(0x06)

/*! Feature engine error interrupt mapping */
#define BMA580_FEAT_ENG_ERR_INT_MAP_MSK       UINT8_C(0x03)
#define BMA580_FEAT_ENG_ERR_INT_MAP_POS       UINT8_C(0x00)

/*!  Feature engine initialization status  */
#define BMA580_FEAT_INIT_STAT_MSK             UINT8_C(0x03)
#define BMA580_FEAT_INIT_STAT_POS             UINT8_C(0x00)

/*!  Bit is set to '1' if fast-offset compensation feature is being executed. Bit is cleared to '0' at the end of
 * feature compensation. User should not change the accelerometer configuration while the feature is running.  */
#define BMA580_FOC_RUNNING_MSK                UINT8_C(0x04)
#define BMA580_FOC_RUNNING_POS                UINT8_C(0x02)

/*! Enables generic interrupt 1 feature */
#define BMA580_GEN_INT1_EN_MSK                UINT8_C(0x01)
#define BMA580_GEN_INT1_EN_POS                UINT8_C(0x00)

/*! Enables generic interrupt 2 feature */
#define BMA580_GEN_INT2_EN_MSK                UINT8_C(0x02)
#define BMA580_GEN_INT2_EN_POS                UINT8_C(0x01)

/*! Enables generic interrupt 3 feature */
#define BMA580_GEN_INT3_EN_MSK                UINT8_C(0x04)
#define BMA580_GEN_INT3_EN_POS                UINT8_C(0x02)

/*! Enables accelerometer fast offset compensation feature */
#define BMA580_ACC_FOC_EN_MSK                 UINT8_C(0x08)
#define BMA580_ACC_FOC_EN_POS                 UINT8_C(0x03)

/*! Enables tap feature */
#define BMA580_TAP_EN_MSK                     UINT8_C(0x10)
#define BMA580_TAP_EN_POS                     UINT8_C(0x04)

/*! Enables voice activity detection feature */
#define BMA580_VAD_EN_MSK                     UINT8_C(0x20)
#define BMA580_VAD_EN_POS                     UINT8_C(0x05)

/*! Enables self wake-up feature */
#define BMA580_SELF_WAKE_UP_EN_MSK            UINT8_C(0x40)
#define BMA580_SELF_WAKE_UP_EN_POS            UINT8_C(0x06)

/*! Data source selection for gen_int1 feature */
#define BMA580_GEN_INT1_DATA_SRC_MSK          UINT8_C(0x03)
#define BMA580_GEN_INT1_DATA_SRC_POS          UINT8_C(0x00)

/*! Data source selection for gen_int2 feature */
#define BMA580_GEN_INT2_DATA_SRC_MSK          UINT8_C(0x0C)
#define BMA580_GEN_INT2_DATA_SRC_POS          UINT8_C(0x02)

/*! Data source selection for gen_int3 feature */
#define BMA580_GEN_INT3_DATA_SRC_MSK          UINT8_C(0x30)
#define BMA580_GEN_INT3_DATA_SRC_POS          UINT8_C(0x04)

/*! Status of generic interrupt 1 feature */
#define BMA580_GEN_INT1_STAT_MSK              UINT8_C(0x01)
#define BMA580_GEN_INT1_STAT_POS              UINT8_C(0x00)

/*! Status of generic interrupt 2 feature */
#define BMA580_GEN_INT2_STAT_MSK              UINT8_C(0x02)
#define BMA580_GEN_INT2_STAT_POS              UINT8_C(0x01)

/*! Status of generic interrupt 3 feature */
#define BMA580_GEN_INT3_STAT_MSK              UINT8_C(0x04)
#define BMA580_GEN_INT3_STAT_POS              UINT8_C(0x02)

/*! Status of self wake-up feature */
#define BMA580_SELF_WAKE_UP_STAT_MSK          UINT8_C(0x08)
#define BMA580_SELF_WAKE_UP_STAT_POS          UINT8_C(0x03)

/*! Status of vad feature */
#define BMA580_VAD_STAT_MSK                   UINT8_C(0x10)
#define BMA580_VAD_STAT_POS                   UINT8_C(0x04)

/*! Auxilary data [7:0]. Value from measurement voltage on INT1/2 pin. */
#define BMA580_AUX_DATA_7_0_MSK               UINT8_C(0xFF)
#define BMA580_AUX_DATA_7_0_POS               UINT8_C(0x00)

/*! Auxilary data [15:8]. Value from measurement voltage on INT1/2 pin. */
#define BMA580_AUX_DATA_15_8_MSK              UINT8_C(0xFF)
#define BMA580_AUX_DATA_15_8_POS              UINT8_C(0x00)

/******************************************************************************/
/********************* Macros for supported field values **********************/
/******************************************************************************/
/* Macros to define the supported chip_id values */
#define BMA580_CHIP_ID_BMA580                 UINT8_C(0xC4)             /*! product identifier for BMA580 */

/* Macros to define the supported acc_drdy_int_map values */
#define BMA580_ACC_DRDY_INT_MAP_UNMAPPED      UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_ACC_DRDY_INT_MAP_INT1          UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_ACC_DRDY_INT_MAP_INT2          UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_ACC_DRDY_INT_MAP_I3C           UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported fifo_wm_int_map values */
#define BMA580_FIFO_WM_INT_MAP_UNMAPPED       UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_FIFO_WM_INT_MAP_INT1           UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_FIFO_WM_INT_MAP_INT2           UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_FIFO_WM_INT_MAP_I3C            UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported fifo_full_int_map values */
#define BMA580_FIFO_FULL_INT_MAP_UNMAPPED     UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_FIFO_FULL_INT_MAP_INT1         UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_FIFO_FULL_INT_MAP_INT2         UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_FIFO_FULL_INT_MAP_I3C          UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported gen_int1_int_map values */
#define BMA580_GEN_INT1_INT_MAP_UNMAPPED      UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_GEN_INT1_INT_MAP_INT1          UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_GEN_INT1_INT_MAP_INT2          UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_GEN_INT1_INT_MAP_I3C           UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported gen_int2_int_map values */
#define BMA580_GEN_INT2_INT_MAP_UNMAPPED      UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_GEN_INT2_INT_MAP_INT1          UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_GEN_INT2_INT_MAP_INT2          UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_GEN_INT2_INT_MAP_I3C           UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported gen_int3_int_map values */
#define BMA580_GEN_INT3_INT_MAP_UNMAPPED      UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_GEN_INT3_INT_MAP_INT1          UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_GEN_INT3_INT_MAP_INT2          UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_GEN_INT3_INT_MAP_I3C           UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported acc_foc_int_map values */
#define BMA580_ACC_FOC_INT_MAP_UNMAPPED       UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_ACC_FOC_INT_MAP_INT1           UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_ACC_FOC_INT_MAP_INT2           UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_ACC_FOC_INT_MAP_I3C            UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported stap_int_map values */
#define BMA580_STAP_INT_MAP_UNMAPPED          UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_STAP_INT_MAP_INT1              UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_STAP_INT_MAP_INT2              UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_STAP_INT_MAP_I3C               UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported dtap_int_map values */
#define BMA580_DTAP_INT_MAP_UNMAPPED          UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_DTAP_INT_MAP_INT1              UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_DTAP_INT_MAP_INT2              UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_DTAP_INT_MAP_I3C               UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported ttap_int_map values */
#define BMA580_TTAP_INT_MAP_UNMAPPED          UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_TTAP_INT_MAP_INT1              UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_TTAP_INT_MAP_INT2              UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_TTAP_INT_MAP_I3C               UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported vad_int_map values */
#define BMA580_VAD_INT_MAP_UNMAPPED           UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_VAD_INT_MAP_INT1               UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_VAD_INT_MAP_INT2               UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_VAD_INT_MAP_I3C                UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported self_wake_up_int_map values */
#define BMA580_SELF_WAKE_UP_INT_MAP_UNMAPPED  UINT8_C(0x00)                /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_SELF_WAKE_UP_INT_MAP_INT1      UINT8_C(0x01)                /*! Interrupt is mapped to INT1 pin */
#define BMA580_SELF_WAKE_UP_INT_MAP_INT2      UINT8_C(0x02)                /*! Interrupt is mapped to INT2 pin */
#define BMA580_SELF_WAKE_UP_INT_MAP_I3C       UINT8_C(0x03)                /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported feat_eng_err_int_map values */
#define BMA580_FEAT_ENG_ERR_INT_MAP_UNMAPPED  UINT8_C(0x00)             /*! Interrupt is not mapped to any destination
                                                                         * */
#define BMA580_FEAT_ENG_ERR_INT_MAP_INT1      UINT8_C(0x01)             /*! Interrupt is mapped to INT1 pin */
#define BMA580_FEAT_ENG_ERR_INT_MAP_INT2      UINT8_C(0x02)             /*! Interrupt is mapped to INT2 pin */
#define BMA580_FEAT_ENG_ERR_INT_MAP_I3C       UINT8_C(0x03)             /*! Interrupt is mapped to I3C in-band
                                                                         * interrupts */

/* Macros to define the supported feat_init_stat values */
#define BMA580_FEAT_INIT_STAT_INIT_NOT_OK     UINT8_C(0x00)             /*! Feature engine is not initialized */
#define BMA580_FEAT_INIT_STAT_INIT_OK         UINT8_C(0x01)             /*! Feature engine is initialized */
#define BMA580_FEAT_INIT_STAT_UN_DEF_1        UINT8_C(0x02)             /*! Reserved */
#define BMA580_FEAT_INIT_STAT_UN_DEF_2        UINT8_C(0x03)             /*! Reserved */

/* Macros to define the supported gen_int1_data_src values */
#define BMA580_GEN_INT1_DATA_SRC_DATA_SRC_1   UINT8_C(0x00)             /*! Uses 50Hz filter data */
#define BMA580_GEN_INT1_DATA_SRC_DATA_SRC_2   UINT8_C(0x01)             /*! Uses 200Hz filter data */
#define BMA580_GEN_INT1_DATA_SRC_DATA_SRC_3   UINT8_C(0x02)             /*! Uses user filter data */
#define BMA580_GEN_INT1_DATA_SRC_DATA_SRC_4   UINT8_C(0x03)             /*! Uses 50Hz filter data. Same as data_src_1 */

/* Macros to define the supported gen_int2_data_src values */
#define BMA580_GEN_INT2_DATA_SRC_DATA_SRC_1   UINT8_C(0x00)             /*! Uses 50Hz filter data */
#define BMA580_GEN_INT2_DATA_SRC_DATA_SRC_2   UINT8_C(0x01)             /*! Uses 200Hz filter data */
#define BMA580_GEN_INT2_DATA_SRC_DATA_SRC_3   UINT8_C(0x02)             /*! Uses user filter data */
#define BMA580_GEN_INT2_DATA_SRC_DATA_SRC_4   UINT8_C(0x03)             /*! Uses 50Hz filter data. Same as data_src_1 */

/* Macros to define the supported gen_int3_data_src values */
#define BMA580_GEN_INT3_DATA_SRC_DATA_SRC_1   UINT8_C(0x00)             /*! Uses 50Hz filter data */
#define BMA580_GEN_INT3_DATA_SRC_DATA_SRC_2   UINT8_C(0x01)             /*! Uses 200Hz filter data */
#define BMA580_GEN_INT3_DATA_SRC_DATA_SRC_3   UINT8_C(0x02)             /*! Uses user filter data */
#define BMA580_GEN_INT3_DATA_SRC_DATA_SRC_4   UINT8_C(0x03)             /*! Uses 50Hz filter data. Same as data_src_1 */

/* Macros to define the supported self_wake_up_stat values */
#define BMA580_SELF_WAKE_UP_STAT_NOR_MODE     UINT8_C(0x00)                /*! Normal power mode */
#define BMA580_SELF_WAKE_UP_STAT_LP_MODE      UINT8_C(0x01)                /*! low power mode */

#define BMA580_INT_STATUS_INT1                UINT8_C(1)
#define BMA580_INT_STATUS_INT2                UINT8_C(2)
#define BMA580_INT_STATUS_I3C                 UINT8_C(3)

/******************************************************************************/
/***************** Structures for handling register content *******************/
/******************************************************************************/

/*!
 * @brief Structure holding INT1 interrupt status register 0
 */
struct bma580_int_status
{
    /*! Accelerometer data ready interrupt status */
    uint8_t acc_drdy_int_status;

    /*! FIFO watermark interrupt status */
    uint8_t fifo_wm_int_status;

    /*! FIFO full interrupt status */
    uint8_t fifo_full_int_status;

    /*! Generic interrupt 1 interrupt status */
    uint8_t gen_int1_int_status;

    /*! Generic interrupt 2 interrupt status */
    uint8_t gen_int2_int_status;

    /*! Generic interrupt 3 interrupt status */
    uint8_t gen_int3_int_status;

    /*! Accelerometer fast offset compensation interrupt status */
    uint8_t acc_foc_int_status;

    /*! Single tap interrupt status */
    uint8_t stap_int_status;

    /*! Double tap interrupt status */
    uint8_t dtap_int_status;

    /*! Triple tap interrupt status */
    uint8_t ttap_int_status;

    /*! Voice activity detection interrupt status */
    uint8_t vad_int_status;

    /*! Self wake-up interrupt status */
    uint8_t self_wake_up_int_status;

    /*! Feature engine error interrupt status */
    uint8_t feat_eng_err_int_status;
};

/*!
 * @brief Structure holding Interrupt source and its configuration
 */
struct bma580_int_status_types
{
    /*! Interrupt soruce */
    uint8_t int_src;

    /*! Interrupt status configuration */
    struct bma580_int_status int_status;
};

/*!
 * @brief Structure holding Interrupt mapping register 0
 */
struct bma580_int_map
{
    /*! Data ready interrupt mapping */
    uint8_t acc_drdy_int_map;

    /*! FIFO watermark interrupt mapping */
    uint8_t fifo_wm_int_map;

    /*! FIFO full interrupt mapping */
    uint8_t fifo_full_int_map;

    /*! Generic interrupt 1 interrupt mapping */
    uint8_t gen_int1_int_map;

    /*! Generic interrupt 2 interrupt mapping */
    uint8_t gen_int2_int_map;

    /*! Generic interrupt 3 interrupt mapping */
    uint8_t gen_int3_int_map;

    /*! Accelerometer fast offset compensation interrupt mapping */
    uint8_t acc_foc_int_map;

    /*! Single tap interrupt mapping */
    uint8_t stap_int_map;

    /*! Double tap interrupt mapping */
    uint8_t dtap_int_map;

    /*! Triple tap interrupt mapping */
    uint8_t ttap_int_map;

    /*! Voice activity detection interrupt mapping */
    uint8_t vad_int_map;

    /*! Self wake-up interrupt mapping */
    uint8_t self_wake_up_int_map;

    /*! MCU error interrupt mapping */
    uint8_t feat_eng_err_int_map;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose flags
 */
struct bma580_feat_eng_gp_flags
{
    /*!  Feature engine initialization status  */
    uint8_t feat_init_stat;

    /*!  Bit is set to '1' if fast-offset compensation feature is being executed. Bit is cleared to '0' at the end of
     * feature compensation. User should not change the accelerometer configuration while the feature is running.  */
    uint8_t foc_running;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register 0
 */
struct bma580_feat_eng_gpr_0
{
    /*! Enables generic interrupt 1 feature */
    uint8_t gen_int1_en;

    /*! Enables generic interrupt 2 feature */
    uint8_t gen_int2_en;

    /*! Enables generic interrupt 3 feature */
    uint8_t gen_int3_en;

    /*! Enables accelerometer fast offset compensation feature */
    uint8_t acc_foc_en;

    /*! Enables tap feature */
    uint8_t tap_en;

    /*! Enables voice activity detection feature */
    uint8_t vad_en;

    /*! Enables self wake-up feature */
    uint8_t self_wake_up_en;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register 1
 */
struct bma580_feat_eng_gpr_1
{
    /*! Data source selection for gen_int1 feature */
    uint8_t gen_int1_data_src;

    /*! Data source selection for gen_int2 feature */
    uint8_t gen_int2_data_src;

    /*! Data source selection for gen_int3 feature */
    uint8_t gen_int3_data_src;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register 2
 */
struct bma580_feat_eng_gpr_2
{
    /*! Status of generic interrupt 1 feature */
    uint8_t gen_int1_stat;

    /*! Status of generic interrupt 2 feature */
    uint8_t gen_int2_stat;

    /*! Status of generic interrupt 3 feature */
    uint8_t gen_int3_stat;

    /*! Status of self wake-up feature */
    uint8_t self_wake_up_stat;

    /*! Status of vad feature */
    uint8_t vad_stat;
};

/******************************************************************************/
/********************** Function prototype declarations ***********************/
/******************************************************************************/

/**
 * \ingroup bma580
 * \defgroup bma580ApiRegs BMA580 Registers
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_chip_id bma580_get_chip_id
 * \code
 * int8_t bma580_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the The product chip_id. Upper four bits are fix and lower four bits are boot loaded from OTP.
 *
 * @param[out] chip_id : Chip id
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_int_status bma580_get_int_status
 * \code
 * int8_t bma580_get_int_status(struct bma580_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the INT1 interrupt status register 0
 *
 * @param[out] config : Structure instance of bma580_int_status_types
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_int_status(struct bma580_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_int_status bma580_get_int_status
 * \code
 * int8_t bma580_set_int_status(const struct bma580_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the INT1 interrupt status register 0
 *
 * @param[in] config : Structure instance of bma580_int_status
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_int_status(const struct bma580_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_int_map bma580_get_int_map
 * \code
 * int8_t bma580_get_int_map(struct bma580_int_map *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Interrupt mapping register
 *
 * @param[out] config : Structure instance of bma580_int_map
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_int_map(struct bma580_int_map *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_int_map_0 bma580_get_int_map_0
 * \code
 * int8_t bma580_set_int_map(const struct bma580_int_map *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Interrupt mapping register 0
 *
 * @param[in] config : Structure instance of bma580_int_map_0
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_int_map(const struct bma580_int_map *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_feat_eng_gp_flags bma580_get_feat_eng_gp_flags
 * \code
 * int8_t bma580_get_feat_eng_gp_flags(struct bma580_feat_eng_gp_flags *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose flags
 *
 * @param[out] config : Structure instance of bma580_feat_eng_gp_flags
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_feat_eng_gp_flags(struct bma580_feat_eng_gp_flags *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_feat_eng_gpr_0 bma580_get_feat_eng_gpr_0
 * \code
 * int8_t bma580_get_feat_eng_gpr_0(struct bma580_feat_eng_gpr_0 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 0
 *
 * @param[out] config : Structure instance of bma580_feat_eng_gpr_0
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_feat_eng_gpr_0(struct bma580_feat_eng_gpr_0 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_feat_eng_gpr_0 bma580_get_feat_eng_gpr_0
 * \code
 * int8_t bma580_set_feat_eng_gpr_0(const struct bma580_feat_eng_gpr_0 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') general purpose register 0
 *
 * @param[in] config : Structure instance of bma580_feat_eng_gpr_0
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_feat_eng_gpr_0(const struct bma580_feat_eng_gpr_0 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_feat_eng_gpr_1 bma580_get_feat_eng_gpr_1
 * \code
 * int8_t bma580_get_feat_eng_gpr_1(struct bma580_feat_eng_gpr_1 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 1
 *
 * @param[out] config : Structure instance of bma580_feat_eng_gpr_1
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_feat_eng_gpr_1(struct bma580_feat_eng_gpr_1 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_feat_eng_gpr_1 bma580_get_feat_eng_gpr_1
 * \code
 * int8_t bma580_set_feat_eng_gpr_1(const struct bma580_feat_eng_gpr_1 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') general purpose register 1
 *
 * @param[in] config : Structure instance of bma580_feat_eng_gpr_1
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_set_feat_eng_gpr_1(const struct bma580_feat_eng_gpr_1 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_feat_eng_gpr_2 bma580_get_feat_eng_gpr_2
 * \code
 * int8_t bma580_get_feat_eng_gpr_2(struct bma580_feat_eng_gpr_2 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 2
 *
 * @param[out] config : Structure instance of bma580_feat_eng_gpr_2
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_feat_eng_gpr_2(struct bma580_feat_eng_gpr_2 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma580ApiRegs
 * \page bma580_api_bma580_get_aux_data bma580_get_aux_data
 * \code
 * int8_t bma580_get_aux_data(uint16_t *aux_data, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Data from the measurement a voltage via the external INT1 or INT2 pin.
 *
 * @param[out] aux_data : Data from the measurement a voltage via the external INT1 or INT2 pin.
 * @param[in]  dev : Structure instance of bma580_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma580_get_aux_data(uint16_t *aux_data, struct bma5_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /*_BMA580_H */
