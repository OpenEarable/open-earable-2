/*!
 * @file DFRobot_BMX160.h
 * @brief DFRobot_BMX160 class infrastructure
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_BMX160
 */
//#include <Wire.h>
#include <TWIM.h>

#define LITTLE_ENDIAN 1

/** Mask definitions */
#define BMX160_MAGN_ODR_MASK                     0x0F
#define BMX160_ACCEL_BW_MASK                     0x70
#define BMX160_ACCEL_ODR_MASK                    0x0F
#define BMX160_ACCEL_UNDERSAMPLING_MASK          0x80
#define BMX160_ACCEL_RANGE_MASK                  0x0F
#define BMX160_GYRO_BW_MASK                      0x30
#define BMX160_GYRO_ODR_MASK                     0x0F
#define BMX160_GYRO_RANGE_MSK                    0x07

/** Mask definitions for INT_EN registers */
#define BMX160_ANY_MOTION_X_INT_EN_MASK          0x01
#define BMX160_HIGH_G_X_INT_EN_MASK              0x01
#define BMX160_NO_MOTION_X_INT_EN_MASK           0x01
#define BMX160_ANY_MOTION_Y_INT_EN_MASK          0x02
#define BMX160_HIGH_G_Y_INT_EN_MASK              0x02
#define BMX160_NO_MOTION_Y_INT_EN_MASK           0x02
#define BMX160_ANY_MOTION_Z_INT_EN_MASK          0x04
#define BMX160_HIGH_G_Z_INT_EN_MASK              0x04
#define BMX160_NO_MOTION_Z_INT_EN_MASK           0x04
#define BMX160_SIG_MOTION_INT_EN_MASK            0x07
#define BMX160_ANY_MOTION_ALL_INT_EN_MASK        0x07
#define BMX160_STEP_DETECT_INT_EN_MASK           0x08
#define BMX160_DOUBLE_TAP_INT_EN_MASK            0x10
#define BMX160_SINGLE_TAP_INT_EN_MASK            0x20
#define BMX160_FIFO_FULL_INT_EN_MASK             0x20
#define BMX160_ORIENT_INT_EN_MASK                0x40
#define BMX160_FIFO_WATERMARK_INT_EN_MASK        0x40
#define BMX160_LOW_G_INT_EN_MASK                 0x08
#define BMX160_STEP_DETECT_EN_MASK               0x08
#define BMX160_FLAT_INT_EN_MASK                  0x80
#define BMX160_DATA_RDY_INT_EN_MASK              0x10

/** Mask definitions for INT_OUT_CTRL register */
#define BMX160_INT1_EDGE_CTRL_MASK               0x01
#define BMX160_INT1_OUTPUT_MODE_MASK             0x04
#define BMX160_INT1_OUTPUT_TYPE_MASK             0x02
#define BMX160_INT1_OUTPUT_EN_MASK               0x08
#define BMX160_INT2_EDGE_CTRL_MASK               0x10
#define BMX160_INT2_OUTPUT_MODE_MASK             0x40
#define BMX160_INT2_OUTPUT_TYPE_MASK             0x20
#define BMX160_INT2_OUTPUT_EN_MASK               0x80

/** Mask definitions for INT_LATCH register */
#define BMX160_INT1_INPUT_EN_MASK                0x10
#define BMX160_INT2_INPUT_EN_MASK                0x20
#define BMX160_INT_LATCH_MASK                    0x0F

/** Mask definitions for INT_MAP register */
#define BMX160_INT1_LOW_G_MASK                   0x01
#define BMX160_INT1_HIGH_G_MASK                  0x02
#define BMX160_INT1_SLOPE_MASK                   0x04
#define BMX160_INT1_NO_MOTION_MASK               0x08
#define BMX160_INT1_DOUBLE_TAP_MASK              0x10
#define BMX160_INT1_SINGLE_TAP_MASK              0x20
#define BMX160_INT1_FIFO_FULL_MASK               0x20
#define BMX160_INT1_FIFO_WM_MASK                 0x40
#define BMX160_INT1_ORIENT_MASK                  0x40
#define BMX160_INT1_FLAT_MASK                    0x80
#define BMX160_INT1_DATA_READY_MASK              0x80
#define BMX160_INT2_LOW_G_MASK                   0x01
#define BMX160_INT1_LOW_STEP_DETECT_MASK         0x01
#define BMX160_INT2_LOW_STEP_DETECT_MASK         0x01
#define BMX160_INT2_HIGH_G_MASK                  0x02
#define BMX160_INT2_FIFO_FULL_MASK               0x02
#define BMX160_INT2_FIFO_WM_MASK                 0x04
#define BMX160_INT2_SLOPE_MASK                   0x04
#define BMX160_INT2_DATA_READY_MASK              0x08
#define BMX160_INT2_NO_MOTION_MASK               0x08
#define BMX160_INT2_DOUBLE_TAP_MASK              0x10
#define BMX160_INT2_SINGLE_TAP_MASK              0x20
#define BMX160_INT2_ORIENT_MASK                  0x40
#define BMX160_INT2_FLAT_MASK                    0x80

/** Mask definitions for INT_DATA register */
#define BMX160_TAP_SRC_INT_MASK                  0x08
#define BMX160_LOW_HIGH_SRC_INT_MASK             0x80
#define BMX160_MOTION_SRC_INT_MASK               0x80

/** Mask definitions for INT_MOTION register */
#define BMX160_SLOPE_INT_DUR_MASK                0x03
#define BMX160_NO_MOTION_INT_DUR_MASK            0xFC
#define BMX160_NO_MOTION_SEL_BIT_MASK            0x01

/** Mask definitions for INT_TAP register */
#define BMX160_TAP_DUR_MASK                      0x07
#define BMX160_TAP_SHOCK_DUR_MASK                0x40
#define BMX160_TAP_QUIET_DUR_MASK                0x80
#define BMX160_TAP_THRES_MASK                    0x1F

/** Mask definitions for INT_FLAT register */
#define BMX160_FLAT_THRES_MASK                   0x3F
#define BMX160_FLAT_HOLD_TIME_MASK               0x30
#define BMX160_FLAT_HYST_MASK                    0x07

/** Mask definitions for INT_LOWHIGH register */
#define BMX160_LOW_G_HYST_MASK                   0x03
#define BMX160_LOW_G_LOW_MODE_MASK               0x04
#define BMX160_HIGH_G_HYST_MASK                  0xC0

/** Mask definitions for INT_SIG_MOTION register */
#define BMX160_SIG_MOTION_SEL_MASK               0x02
#define BMX160_SIG_MOTION_SKIP_MASK              0x0C
#define BMX160_SIG_MOTION_PROOF_MASK             0x30

/** Mask definitions for INT_ORIENT register */
#define BMX160_ORIENT_MODE_MASK                  0x03
#define BMX160_ORIENT_BLOCK_MASK                 0x0C
#define BMX160_ORIENT_HYST_MASK                  0xF0
#define BMX160_ORIENT_THETA_MASK                 0x3F
#define BMX160_ORIENT_UD_ENABLE                  0x40
#define BMX160_AXES_EN_MASK                      0x80

/** Mask definitions for FIFO_CONFIG register */
#define BMX160_FIFO_GYRO                         0x80
#define BMX160_FIFO_ACCEL                        0x40
#define BMX160_FIFO_MAGN                         0x20
#define BMX160_FIFO_TAG_INT1                     0x08
#define BMX160_FIFO_TAG_INT2                     0x04
#define BMX160_FIFO_TIME                         0x02
#define BMX160_FIFO_HEADER                       0x10
#define BMX160_FIFO_CONFIG_1_MASK                0xFE


/** Mask definitions for STEP_CONF register */
#define BMX160_STEP_COUNT_EN_BIT_MASK            0x08
#define BMX160_STEP_DETECT_MIN_THRES_MASK        0x18
#define BMX160_STEP_DETECT_STEPTIME_MIN_MASK     0x07
#define BMX160_STEP_MIN_BUF_MASK                 0x07

/** Mask definition for FIFO Header Data Tag */
#define BMX160_FIFO_TAG_INTR_MASK                0xFC

/** Fifo byte counter mask definitions */
#define BMX160_FIFO_BYTE_COUNTER_MASK            0x07

/** Enable/disable bit value */
#define BMX160_ENABLE                            0x01
#define BMX160_DISABLE                           0x00

/** Latch Duration */
#define BMX160_LATCH_DUR_NONE                    0x00
#define BMX160_LATCH_DUR_312_5_MICRO_SEC         0x01
#define BMX160_LATCH_DUR_625_MICRO_SEC           0x02
#define BMX160_LATCH_DUR_1_25_MILLI_SEC          0x03
#define BMX160_LATCH_DUR_2_5_MILLI_SEC           0x04
#define BMX160_LATCH_DUR_5_MILLI_SEC             0x05
#define BMX160_LATCH_DUR_10_MILLI_SEC            0x06
#define BMX160_LATCH_DUR_20_MILLI_SEC            0x07
#define BMX160_LATCH_DUR_40_MILLI_SEC            0x08
#define BMX160_LATCH_DUR_80_MILLI_SEC            0x09
#define BMX160_LATCH_DUR_160_MILLI_SEC           0x0A
#define BMX160_LATCH_DUR_320_MILLI_SEC           0x0B
#define BMX160_LATCH_DUR_640_MILLI_SEC           0x0C
#define BMX160_LATCH_DUR_1_28_SEC                0x0D
#define BMX160_LATCH_DUR_2_56_SEC                0x0E
#define BMX160_LATCHED                           0x0F

/** bmx160 Register map */
#define BMX160_CHIP_ID_ADDR                      0x00
#define BMX160_ERROR_REG_ADDR                    0x02
#define BMX160_MAG_DATA_ADDR                     0x04
#define BMX160_GYRO_DATA_ADDR                    0x0C
#define BMX160_ACCEL_DATA_ADDR                   0x12
#define BMX160_STATUS_ADDR                       0x1B
#define BMX160_INT_STATUS_ADDR                   0x1C
#define BMX160_FIFO_LENGTH_ADDR                  0x22
#define BMX160_FIFO_DATA_ADDR                    0x24
#define BMX160_ACCEL_CONFIG_ADDR                 0x40
#define BMX160_ACCEL_RANGE_ADDR                  0x41
#define BMX160_GYRO_CONFIG_ADDR                  0x42
#define BMX160_GYRO_RANGE_ADDR                   0x43
#define BMX160_MAGN_CONFIG_ADDR                  0x44
#define BMX160_FIFO_DOWN_ADDR                    0x45
#define BMX160_FIFO_CONFIG_0_ADDR                0x46
#define BMX160_FIFO_CONFIG_1_ADDR                0x47
#define BMX160_MAGN_RANGE_ADDR                   0x4B
#define BMX160_MAGN_IF_0_ADDR                    0x4C
#define BMX160_MAGN_IF_1_ADDR                    0x4D
#define BMX160_MAGN_IF_2_ADDR                    0x4E
#define BMX160_MAGN_IF_3_ADDR                    0x4F
#define BMX160_INT_ENABLE_0_ADDR                 0x50
#define BMX160_INT_ENABLE_1_ADDR                 0x51
#define BMX160_INT_ENABLE_2_ADDR                 0x52
#define BMX160_INT_OUT_CTRL_ADDR                 0x53
#define BMX160_INT_LATCH_ADDR                    0x54
#define BMX160_INT_MAP_0_ADDR                    0x55
#define BMX160_INT_MAP_1_ADDR                    0x56
#define BMX160_INT_MAP_2_ADDR                    0x57
#define BMX160_INT_DATA_0_ADDR                   0x58
#define BMX160_INT_DATA_1_ADDR                   0x59
#define BMX160_INT_LOWHIGH_0_ADDR                0x5A
#define BMX160_INT_LOWHIGH_1_ADDR                0x5B
#define BMX160_INT_LOWHIGH_2_ADDR                0x5C
#define BMX160_INT_LOWHIGH_3_ADDR                0x5D
#define BMX160_INT_LOWHIGH_4_ADDR                0x5E
#define BMX160_INT_MOTION_0_ADDR                 0x5F
#define BMX160_INT_MOTION_1_ADDR                 0x60
#define BMX160_INT_MOTION_2_ADDR                 0x61
#define BMX160_INT_MOTION_3_ADDR                 0x62
#define BMX160_INT_TAP_0_ADDR                    0x63
#define BMX160_INT_TAP_1_ADDR                    0x64
#define BMX160_INT_ORIENT_0_ADDR                 0x65
#define BMX160_INT_ORIENT_1_ADDR                 0x66
#define BMX160_INT_FLAT_0_ADDR                   0x67
#define BMX160_INT_FLAT_1_ADDR                   0x68
#define BMX160_FOC_CONF_ADDR                     0x69
#define BMX160_CONF_ADDR                         0x6A

#define BMX160_IF_CONF_ADDR                      0x6B
#define BMX160_SELF_TEST_ADDR                    0x6D
#define BMX160_OFFSET_ADDR                       0x71
#define BMX160_OFFSET_CONF_ADDR                  0x77
#define BMX160_INT_STEP_CNT_0_ADDR               0x78
#define BMX160_INT_STEP_CONFIG_0_ADDR            0x7A
#define BMX160_INT_STEP_CONFIG_1_ADDR            0x7B
#define BMX160_COMMAND_REG_ADDR                  0x7E
#define BMX160_SPI_COMM_TEST_ADDR                0x7F
#define BMX160_INTL_PULLUP_CONF_ADDR             0x85

/** Error code definitions */
#define BMX160_OK                                0
#define BMX160_E_NULL_PTR                        -1
#define BMX160_E_COM_FAIL                        -2
#define BMX160_E_DEV_NOT_FOUND                   -3
#define BMX160_E_OUT_OF_RANGE                    -4
#define BMX160_E_INVALID_INPUT                   -5
#define BMX160_E_ACCEL_ODR_BW_INVALID            -6
#define BMX160_E_GYRO_ODR_BW_INVALID             -7
#define BMX160_E_LWP_PRE_FLTR_INT_INVALID        -8
#define BMX160_E_LWP_PRE_FLTR_INVALID            -9
#define BMX160_E_MAGN_NOT_FOUND                  -10
#define BMX160_FOC_FAILURE                       -11
#define BMX160_ERR_CHOOSE                        -12

/**\name API warning codes */
#define BMX160_W_GYRO_SELF_TEST_FAIL  (1)
#define BMX160_W_ACCEl_SELF_TEST_FAIL (2)

/** bmx160 unique chip identifier */
#define BMX160_CHIP_ID                           0xD8

/** Soft reset command */
#define BMX160_SOFT_RESET_CMD                    0xb6
#define BMX160_SOFT_RESET_DELAY_MS               15
/** Start FOC command */
#define BMX160_START_FOC_CMD                     0x03
/** NVM backup enabling command */
#define BMX160_NVM_BACKUP_EN                     0xA0

/* Delay in ms settings */
#define BMX160_ACCEL_DELAY_MS                    5
#define BMX160_GYRO_DELAY_MS                     81
#define BMX160_ONE_MS_DELAY                      1
#define BMX160_MAGN_COM_DELAY                    10
#define BMX160_GYRO_SELF_TEST_DELAY              20
#define BMX160_ACCEL_SELF_TEST_DELAY             50

/** Self test configurations */
#define BMX160_ACCEL_SELF_TEST_CONFIG            0x2C
#define BMX160_ACCEL_SELF_TEST_POSITIVE_EN       0x0D
#define BMX160_ACCEL_SELF_TEST_NEGATIVE_EN       0x09
#define BMX160_ACCEL_SELF_TEST_LIMIT             8192

/** Power mode settings */
/* Accel power mode */
#define BMX160_ACCEL_NORMAL_MODE                 0x11
#define BMX160_ACCEL_LOWPOWER_MODE               0x12
#define BMX160_ACCEL_SUSPEND_MODE                0x10

/* Gyro power mode */
#define BMX160_GYRO_SUSPEND_MODE                 0x14
#define BMX160_GYRO_NORMAL_MODE                  0x15
#define BMX160_GYRO_FASTSTARTUP_MODE             0x17

/* Magn power mode */
#define BMX160_MAGN_SUSPEND_MODE                 0x18
#define BMX160_MAGN_NORMAL_MODE                  0x19
#define BMX160_MAGN_LOWPOWER_MODE                0x1A

/** Range settings */
/* Accel Range */
#define BMX160_ACCEL_RANGE_2G                    0x03
#define BMX160_ACCEL_RANGE_4G                    0x05
#define BMX160_ACCEL_RANGE_8G                    0x08
#define BMX160_ACCEL_RANGE_16G                   0x0C

/* Gyro Range */
#define BMX160_GYRO_RANGE_2000_DPS               0x00
#define BMX160_GYRO_RANGE_1000_DPS               0x01
#define BMX160_GYRO_RANGE_500_DPS                0x02
#define BMX160_GYRO_RANGE_250_DPS                0x03
#define BMX160_GYRO_RANGE_125_DPS                0x04


#define BMX160_ACCEL_MG_LSB_2G      0.000061035F   ///< Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
#define BMX160_ACCEL_MG_LSB_4G      0.000122070F   ///< Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
#define BMX160_ACCEL_MG_LSB_8G      0.000244141F   ///< Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
#define BMX160_ACCEL_MG_LSB_16G     0.000488281F   ///< Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */

#define BMX160_GYRO_SENSITIVITY_125DPS  0.0038110F ///< Gyroscope sensitivity at 125dps */
#define BMX160_GYRO_SENSITIVITY_250DPS  0.0076220F ///< Gyroscope sensitivity at 250dps */
#define BMX160_GYRO_SENSITIVITY_500DPS  0.0152439F ///< Gyroscope sensitivity at 500dps */
#define BMX160_GYRO_SENSITIVITY_1000DPS 0.0304878F ///< Gyroscope sensitivity at 1000dps */
#define BMX160_GYRO_SENSITIVITY_2000DPS 0.0609756F ///< Gyroscope sensitivity at 2000dps */

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMX160_ACCEL_BW_OSR4_AVG1         0x00
#define BMX160_ACCEL_BW_OSR2_AVG2         0x01
#define BMX160_ACCEL_BW_NORMAL_AVG4       0x02
#define BMX160_ACCEL_BW_RES_AVG8          0x03
#define BMX160_ACCEL_BW_RES_AVG16         0x04
#define BMX160_ACCEL_BW_RES_AVG32         0x05
#define BMX160_ACCEL_BW_RES_AVG64         0x06
#define BMX160_ACCEL_BW_RES_AVG128        0x07

#define BMX160_GYRO_BW_OSR4_MODE          0x00
#define BMX160_GYRO_BW_OSR2_MODE          0x01
#define BMX160_GYRO_BW_NORMAL_MODE        0x02

#define BMX160_MAGN_UT_LSB      (0.3F)  ///< Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
/* Output Data Rate settings */
/* Accel Output data rate */
#define BMX160_ACCEL_ODR_RESERVED         0x00
#define BMX160_ACCEL_ODR_0_78HZ           0x01
#define BMX160_ACCEL_ODR_1_56HZ           0x02
#define BMX160_ACCEL_ODR_3_12HZ           0x03
#define BMX160_ACCEL_ODR_6_25HZ           0x04
#define BMX160_ACCEL_ODR_12_5HZ           0x05
#define BMX160_ACCEL_ODR_25HZ             0x06
#define BMX160_ACCEL_ODR_50HZ             0x07
#define BMX160_ACCEL_ODR_100HZ            0x08
#define BMX160_ACCEL_ODR_200HZ            0x09
#define BMX160_ACCEL_ODR_400HZ            0x0A
#define BMX160_ACCEL_ODR_800HZ            0x0B
#define BMX160_ACCEL_ODR_1600HZ           0x0C
#define BMX160_ACCEL_ODR_RESERVED0        0x0D
#define BMX160_ACCEL_ODR_RESERVED1        0x0E
#define BMX160_ACCEL_ODR_RESERVED2        0x0F

/* Gyro Output data rate */
#define BMX160_GYRO_ODR_RESERVED          0x00
#define BMX160_GYRO_ODR_25HZ              0x06
#define BMX160_GYRO_ODR_50HZ              0x07
#define BMX160_GYRO_ODR_100HZ             0x08
#define BMX160_GYRO_ODR_200HZ             0x09
#define BMX160_GYRO_ODR_400HZ             0x0A
#define BMX160_GYRO_ODR_800HZ             0x0B
#define BMX160_GYRO_ODR_1600HZ            0x0C
#define BMX160_GYRO_ODR_3200HZ            0x0D

/* Magnetometer sensor Output data rate */
#define BMX160_MAGN_ODR_RESERVED         0x00
#define BMX160_MAGN_ODR_0_78HZ           0x01
#define BMX160_MAGN_ODR_1_56HZ           0x02
#define BMX160_MAGN_ODR_3_12HZ           0x03
#define BMX160_MAGN_ODR_6_25HZ           0x04
#define BMX160_MAGN_ODR_12_5HZ           0x05
#define BMX160_MAGN_ODR_25HZ             0x06
#define BMX160_MAGN_ODR_50HZ             0x07
#define BMX160_MAGN_ODR_100HZ            0x08
#define BMX160_MAGN_ODR_200HZ            0x09
#define BMX160_MAGN_ODR_400HZ            0x0A
#define BMX160_MAGN_ODR_800HZ            0x0B

/* Maximum limits definition */
#define BMX160_MAGN_ODR_MAX               14
#define BMX160_MAGN_BW_MAX                2
#define BMX160_MAGN_RANGE_MAX             8
#define BMX160_GYRO_ODR_MAX               13
#define BMX160_GYRO_BW_MAX                2
#define BMX160_GYRO_RANGE_MAX             4
#define BMX160_ACCEL_ODR_MAX              15
#define BMX160_ACCEL_BW_MAX               2
#define BMX160_ACCEL_RANGE_MAX            12



/** FIFO_CONFIG Definitions */
#define BMX160_FIFO_TIME_ENABLE           0x02
#define BMX160_FIFO_TAG_INT2_ENABLE       0x04
#define BMX160_FIFO_TAG_INT1_ENABLE       0x08
#define BMX160_FIFO_HEAD_ENABLE           0x10
#define BMX160_FIFO_M_ENABLE              0x20
#define BMX160_FIFO_A_ENABLE              0x40
#define BMX160_FIFO_M_A_ENABLE            0x60
#define BMX160_FIFO_G_ENABLE              0x80
#define BMX160_FIFO_M_G_ENABLE            0xA0
#define BMX160_FIFO_G_A_ENABLE            0xC0
#define BMX160_FIFO_M_G_A_ENABLE          0xE0


/* Accel, gyro and magn. sensor length and also their combined
 * length definitions in FIFO */
#define BMX160_FIFO_G_LENGTH              6
#define BMX160_FIFO_A_LENGTH              6
#define BMX160_FIFO_M_LENGTH              8
#define BMX160_FIFO_GA_LENGTH             12
#define BMX160_FIFO_MA_LENGTH             14
#define BMX160_FIFO_MG_LENGTH             14
#define BMX160_FIFO_MGA_LENGTH            20


/** FIFO Header Data definitions */
#define BMX160_FIFO_HEAD_SKIP_FRAME       0x40
#define BMX160_FIFO_HEAD_SENSOR_TIME      0x44
#define BMX160_FIFO_HEAD_INPUT_CONFIG     0x48
#define BMX160_FIFO_HEAD_OVER_READ        0x80
#define BMX160_FIFO_HEAD_A                0x84
#define BMX160_FIFO_HEAD_G                0x88
#define BMX160_FIFO_HEAD_G_A              0x8C
#define BMX160_FIFO_HEAD_M                0x90
#define BMX160_FIFO_HEAD_M_A              0x94
#define BMX160_FIFO_HEAD_M_G              0x98
#define BMX160_FIFO_HEAD_M_G_A            0x9C


/** FIFO sensor time length definitions */
#define BMX160_SENSOR_TIME_LENGTH         3)


/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
#define  BMX160_ACCEL_FIFO_DOWN_ZERO      0x00
#define  BMX160_ACCEL_FIFO_DOWN_ONE       0x10
#define  BMX160_ACCEL_FIFO_DOWN_TWO       0x20
#define  BMX160_ACCEL_FIFO_DOWN_THREE     0x30
#define  BMX160_ACCEL_FIFO_DOWN_FOUR      0x40
#define  BMX160_ACCEL_FIFO_DOWN_FIVE      0x50
#define  BMX160_ACCEL_FIFO_DOWN_SIX       0x60
#define  BMX160_ACCEL_FIFO_DOWN_SEVEN     0x70

/* Gyro fifo down-smapling values*/
#define  BMX160_GYRO_FIFO_DOWN_ZERO       0x00
#define  BMX160_GYRO_FIFO_DOWN_ONE        0x01
#define  BMX160_GYRO_FIFO_DOWN_TWO        0x02
#define  BMX160_GYRO_FIFO_DOWN_THREE      0x03
#define  BMX160_GYRO_FIFO_DOWN_FOUR       0x04
#define  BMX160_GYRO_FIFO_DOWN_FIVE       0x05
#define  BMX160_GYRO_FIFO_DOWN_SIX        0x06
#define  BMX160_GYRO_FIFO_DOWN_SEVEN      0x07


#define  BMX160_ACCEL_FIFO_FILT_EN        0x80  //< Accel Fifo filter enable*/

#define  BMX160_GYRO_FIFO_FILT_EN         0x08  //< Gyro Fifo filter enable*/

/** Definitions to check validity of FIFO frames */
#define FIFO_CONFIG_MSB_CHECK             0x80
#define FIFO_CONFIG_LSB_CHECK             0x00

/*! bmx160 accel FOC configurations */
#define BMX160_FOC_ACCEL_DISABLED         0x00
#define BMX160_FOC_ACCEL_POSITIVE_G       0x01
#define BMX160_FOC_ACCEL_NEGATIVE_G       0x02
#define BMX160_FOC_ACCEL_0G               0x03

/** Array Parameter DefinItions */
#define BMX160_SENSOR_TIME_LSB_BYTE       0
#define BMX160_SENSOR_TIME_XLSB_BYTE      1
#define BMX160_SENSOR_TIME_MSB_BYTE       2


/** Interface settings */
#define BMX160_SPI_INTF                   1
#define BMX160_I2C_INTF                   0
#define BMX160_SPI_RD_MASK                0x80
#define BMX160_SPI_WR_MASK                0x7F

/* Sensor & time select definition*/
#define BMX160_MAG_SEL              0x01
#define BMX160_GYRO_SEL             0x02
#define BMX160_ACCEL_SEL            0x03
#define BMX160_TIME_SEL             0x04


#define BMX160_SEN_SEL_MASK    0x07            //< Sensor select mask*/
#define BMX160_ERR_REG_MASK    0x0F            //< Error code mask */
#define BMX160_I2C_ADDR                  0x68  //< bmx160 I2C address */
#define BMX160_MAGN_BMM150_I2C_ADDR      0x10  //< bmx160 secondary IF address */

/** bmx160 Length definitions */
#define BMX160_ONE                        1
#define BMX160_TWO                        2
#define BMX160_THREE                      3
#define BMX160_FOUR                       4
#define BMX160_FIVE                       5


#define BMX160_FIFO_LEVEL_MARGIN          16  //< bmx160 fifo level Margin */
#define BMX160_FIFO_FLUSH_VALUE           0xB0  //< bmx160 fifo flush Command */

/** bmx160 offset values for xyz axes of accel */
#define BMX160_ACCEL_MIN_OFFSET         -128
#define BMX160_ACCEL_MAX_OFFSET         127

/** bmx160 offset values for xyz axes of gyro */
#define BMX160_GYRO_MIN_OFFSET         -512
#define BMX160_GYRO_MAX_OFFSET         511

/** bmx160 fifo full interrupt position and mask */
#define BMX160_FIFO_FULL_INT_POS   5
#define BMX160_FIFO_FULL_INT_MSK   0x20
#define BMX160_FIFO_WTM_INT_POS    6
#define BMX160_FIFO_WTM_INT_MSK    0x40

#define BMX160_FIFO_FULL_INT_PIN1_POS  5
#define BMX160_FIFO_FULL_INT_PIN1_MSK  0x20
#define BMX160_FIFO_FULL_INT_PIN2_POS  1
#define BMX160_FIFO_FULL_INT_PIN2_MSK  0x02

#define BMX160_FIFO_WTM_INT_PIN1_POS   6
#define BMX160_FIFO_WTM_INT_PIN1_MSK   0x40
#define BMX160_FIFO_WTM_INT_PIN2_POS   2
#define BMX160_FIFO_WTM_INT_PIN2_MSK   0x04

#define BMX160_MANUAL_MODE_EN_POS  7
#define BMX160_MANUAL_MODE_EN_MSK  0x80
#define BMX160_MAGN_READ_BURST_POS  0
#define BMX160_MAGN_READ_BURST_MSK  0x03

#define BMX160_GYRO_SELF_TEST_POS  4
#define BMX160_GYRO_SELF_TEST_MSK  0x10
#define BMX160_GYRO_SELF_TEST_STATUS_POS   1
#define BMX160_GYRO_SELF_TEST_STATUS_MSK   0x02

#define BMX160_GYRO_FOC_EN_POS   6
#define BMX160_GYRO_FOC_EN_MSK   0x40

#define BMX160_ACCEL_FOC_X_CONF_POS  4
#define BMX160_ACCEL_FOC_X_CONF_MSK  0x30

#define BMX160_ACCEL_FOC_Y_CONF_POS  2
#define BMX160_ACCEL_FOC_Y_CONF_MSK  0x0C

#define BMX160_ACCEL_FOC_Z_CONF_MSK  0x03

#define BMX160_FOC_STATUS_POS  3
#define BMX160_FOC_STATUS_MSK  0x08

#define BMX160_GYRO_OFFSET_X_MSK   0x03

#define BMX160_GYRO_OFFSET_Y_POS   2
#define BMX160_GYRO_OFFSET_Y_MSK   0x0C

#define BMX160_GYRO_OFFSET_Z_POS   4
#define BMX160_GYRO_OFFSET_Z_MSK   0x30

#define BMX160_GYRO_OFFSET_EN_POS  7
#define BMX160_GYRO_OFFSET_EN_MSK  0x80

#define BMX160_ACCEL_OFFSET_EN_POS   6
#define BMX160_ACCEL_OFFSET_EN_MSK   0x40


#define BMX160_GYRO_OFFSET_POS          8
#define BMX160_GYRO_OFFSET_MSK          0x0300

#define BMX160_NVM_UPDATE_POS          1
#define BMX160_NVM_UPDATE_MSK          0x02

#define BMX160_NVM_STATUS_POS          4
#define BMX160_NVM_STATUS_MSK          0x10

/* BIT SLICE GET AND SET FUNCTIONS */
#define  BMX160_GET_BITS(regvar, bitname)\
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMX160_SET_BITS(regvar, bitname, val)\
    ((regvar & ~bitname##_MSK) | \
    ((val<<bitname##_POS)&bitname##_MSK))

#define BMX160_SET_BITS_POS_0(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | \
        (data & bitname##_MSK))

#define BMX160_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name UTILITY MACROS */
#define BMX160_SET_LOW_BYTE     0x00FF
#define BMX160_SET_HIGH_BYTE    0xFF00

#define BMX160_GET_LSB(var) (uint8_t)(var & BMX160_SET_LOW_BYTE)
#define BMX160_GET_MSB(var) (uint8_t)((var & BMX160_SET_HIGH_BYTE) >> 8)

/**
 * @struct sBmx160FifoFrame_t
 * @brief This structure holds the information for usage of FIFO by the user.
 */
typedef struct  {
  uint8_t *data;             /**< Data buffer of user defined length is to be mapped here */
  uint16_t length;           /**< While calling the API  "BMX160_get_fifo_data" , length stores number of bytes in FIFO to be read (specified by user as input) and after execution of the API ,number of FIFO data bytes available is provided as an output to user */
  uint8_t fifoTimeEnable;    /**< FIFO time enable */
  uint8_t fifoHeaderEnable;  /**< Enabling of the FIFO header to stream in header mode */
  uint8_t fifoDataEnable;    /**< Streaming of the Accelerometer, Gyroscope sensor data or both in FIFO */
  uint16_t accelByteStartIdx;/**< Will be equal to length when no more frames are there to parse */
  uint16_t gyroByteStartIdx; /**< Will be equal to length when no more frames are there to parse */
  uint16_t magnByteStartIdx; /**< Will be equal to length when no more frames are there to parse */
  uint32_t sensorTime;       /**< Value of FIFO sensor time time */
  uint8_t skippedFrameCount; /**< Value of Skipped frame counts */
}sBmx160FifoFrame_t;

/**
 * @enum eBmx160AnySigMotionActiveInterruptState_t
 * @brief bmx160 active state of any & sig motion interrupt.
 */
typedef enum {
  eBmx160BothAnySigMotionDisabled = -1, /**< Both any & sig motion are disabled */
  eBmx160AnyMotionEnabled,              /**< Any-motion selected */
  eBmx160SigMotionEnabled               /**<  Sig-motion selected */
}eBmx160AnySigMotionActiveInterruptState_t;


/**
 * @struct sBmx160Cfg_t
 * @brief bmx160 sensor configuration structure
 */
typedef struct  {
  uint8_t power;  /**< power mode */
  uint8_t odr;    /**< output data rate */
  uint8_t range;  /**< range */
  uint8_t bw;     /**< bandwidth */
}sBmx160Cfg_t;



/* type definitions */
typedef int8_t (*bmx160ComFptrT)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmx160DelayFptrT)(uint32_t period);

/**
 * @struct sBmx160Dev_t
 */
typedef struct {
  uint8_t chipId;                                        /**< Chip Id */
  uint8_t id;                                            /**< Device Id */
  uint8_t interface;                                     /**< 0 - I2C , 1 - SPI Interface */
  eBmx160AnySigMotionActiveInterruptState_t any_sig_sel; /**< Hold active interrupts status for any and sig motion 0 - Any-motion enable, 1 - Sig-motion enable,  -1 neither any-motion nor sig-motion selected */
  sBmx160Cfg_t magnCfg;                                  /**< Structure to configure Magnetometer sensor */
  sBmx160Cfg_t prevMagnCfg;                              /**< Structure to hold previous/old magn config parameters.This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  sBmx160Cfg_t accelCfg;                                 /**< Structure to configure Accel sensor */
  sBmx160Cfg_t prevAccelCfg;                             /**< Structure to hold previous/old accel config parameters.This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  sBmx160Cfg_t gyroCfg;                                  /**< Structure to configure Gyro sensor */
  sBmx160Cfg_t prevGyroCfg;                              /**< Structure to hold previous/old gyro config parameters. This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  sBmx160FifoFrame_t *fifo;                              /**< FIFO related configurations */
  bmx160ComFptrT read;                                   /**< Read function pointer */
  bmx160ComFptrT write;                                  /**< Write function pointer */
  bmx160DelayFptrT delayMs;                              /**<  Delay function pointer */
}sBmx160Dev_t;

/**
 * @struct sBmx160SensorData_t
 * @brief bmx160 sensor data structure which comprises of accel data
 */
typedef struct {
  float x;           /**< X-axis sensor data */
  float y;           /**< Y-axis sensor data */
  float z;           /**< Z-axis sensor data */
  uint32_t sensortime; /**< sensor time */
}sBmx160SensorData_t; // __attribute__((packed));

/**
 * @enum eBmx160IntChannel_t
 * @brief Interrupt channel
 */
 typedef enum {
  BMX160_INT_CHANNEL_NONE, /**< Un-map both channels */
  BMX160_INT_CHANNEL_1,    /**< interrupt Channel 1 */
  BMX160_INT_CHANNEL_2,    /**< interrupt Channel 2 */
  BMX160_INT_CHANNEL_BOTH  /**< Map both channels */
}eBmx160IntChannel_t;

/**
 * @enum eBmx160IntTypes_t
 * @brief Select Interrupt
 */
typedef enum {
  BMX160_ACC_ANY_MOTION_INT,          /**< Slope/Any-motion interrupt */
  BMX160_ACC_SIG_MOTION_INT,          /**< Significant motion interrupt */
  BMX160_STEP_DETECT_INT,             /**< Step detector interrupt */
  BMX160_ACC_DOUBLE_TAP_INT,          /**< double tap interrupt */
  BMX160_ACC_SINGLE_TAP_INT,          /**< single tap interrupt */
  BMX160_ACC_ORIENT_INT,              /**< orientation interrupt */
  BMX160_ACC_FLAT_INT,                /**< flat interrupt */
  BMX160_ACC_HIGH_G_INT,              /**< high-g interrupt */
  BMX160_ACC_LOW_G_INT,               /**< low-g interrupt */
  BMX160_ACC_SLOW_NO_MOTION_INT,      /**< slow/no-motion interrupt */
  BMX160_ACC_GYRO_DATA_RDY_INT,       /**< data ready interrupt  */
  BMX160_ACC_GYRO_FIFO_FULL_INT,      /**< fifo full interrupt */
  BMX160_ACC_GYRO_FIFO_WATERMARK_INT  /**< fifo watermark interrupt */
}eBmx160IntTypes_t;

/**
 * @struct sBmx160IntPinSettg_t
 * @brief  Structure configuring Interrupt pins
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint16_t outputEn :1;    /**< To enable either INT1 or INT2 pin as output. 0- output disabled ,1- output enabled */
  uint16_t outputMode :1;  /**< 0 - push-pull 1- open drain,only valid if outputEn is set 1 */
  uint16_t outputType :1;  /**< 0 - active low , 1 - active high level.if outputEn is 1,this applies to interrupts,else PMU_trigger */
  uint16_t edgeCtrl :1;    /**< 0 - level trigger , 1 - edge trigger  */
  uint16_t inputEn :1;     /**< To enable either INT1 or INT2 pin as input. 0 - input disabled ,1 - input enabled */
  uint16_t latchDur :4;    /**< latch duration*/
#elif BIG_ENDIAN == 1 
  uint16_t latchDur : 4;   /**< latch duration*/
  uint16_t inputEn : 1;    /**< Latched,non-latched or temporary interrupt modes */
  uint16_t edgeCtrl : 1;   /**< 1 - edge trigger, 0 - level trigger */
  uint16_t outputType : 1; /**< 0 - active low , 1 - active high level. if outputEn is 1,this applies to interrupts,else PMU_trigger */
  uint16_t outputMode : 1; /**< 0 - push-pull , 1 - open drain,only valid if outputEn is set 1 */
  uint16_t outputEn : 1;   /**< To enable either INT1 or INT2 pin as output. 0 - output disabled , 1 - output enabled */
#endif
}sBmx160IntPinSettg_t;

/**
 * @struct sBmx160AccTapIntCfg_t
 * @brief  Tap interrupt structure
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint16_t tapThr :5;     /**< tap threshold */
  uint16_t tapShock :1;   /**< tap shock */
  uint16_t tapQuiet :1;   /**< tap quiet */
  uint16_t tapDur :3;     /**< tap duration */
  uint16_t tapDataSrc :1; /**< data source 0- filter & 1 pre-filter*/
  uint16_t tapEn :1;      /**< tap enable, 1 - enable, 0 - disable */
#elif BIG_ENDIAN == 1
  uint16_t tapEn :1;      /**< tap enable, 1 - enable, 0 - disable */
  uint16_t tapDataSrc :1; /**< data source 0- filter & 1 pre-filter*/
  uint16_t tapDur : 3;    /**< tap duration */
  uint16_t tapQuiet : 1;  /**< tap quiet */
  uint16_t tapShock : 1;  /**< tap shock */
  uint16_t tapThr : 5;    /**< tap threshold */
#endif
}sBmx160AccTapIntCfg_t;

/**
 * @struct sBmx160AccAnyMotIntCfg_t
 * @brief  Slope interrupt structure
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint8_t anymotionEn :1;      /**< 1 any-motion enable, 0 - any-motion disable */
  uint8_t anymotionX :1;       /**< slope interrupt x, 1 - enable, 0 - disable */
  uint8_t anymotionY :1;       /**< slope interrupt y, 1 - enable, 0 - disable */
  uint8_t anymotionZ :1;       /**< slope interrupt z, 1 - enable, 0 - disable */
  uint8_t anymotionDur :2;     /**< slope duration */
  uint8_t anymotionDataSrc :1; /**< data source 0- filter & 1 pre-filter*/
  uint8_t anymotionThr;        /**< slope threshold */
#elif BIG_ENDIAN == 1
  uint8_t anymotionThr;        /**< slope threshold */
  uint8_t anymotionDataSrc :1; /**< data source 0- filter & 1 pre-filter*/
  uint8_t anymotionDur : 2;    /**< slope duration */
  uint8_t anymotionZ : 1;      /**< slope interrupt z, 1 - enable, 0 - disable */
  uint8_t anymotionY : 1;      /**< slope interrupt y, 1 - enable, 0 - disable */
  uint8_t anymotionX : 1;      /**< slope interrupt x, 1 - enable, 0 - disable */
  uint8_t anymotionEn :1;      /**< 1 any-motion enable, 0 - any-motion disable */
#endif
}sBmx160AccAnyMotIntCfg_t;

/**
 * @struct sBmx160AccSigMotIntCfg_t
 * @brief  Significant motion interrupt structure
 */
typedef struct  {
#if LITTLE_ENDIAN == 1
  uint8_t sigMotSkip :2;  /**< skip time of sig-motion interrupt */
  uint8_t sigMotProof :2; /**< proof time of sig-motion interrupt */
  uint8_t sigDataSrc :1;  /**< data source 0- filter & 1 pre-filter*/
  uint8_t sigEn :1;       /**< 1 - enable sig, 0 - disable sig & enable anymotion */
  uint8_t sigMotThres;    /**< sig-motion threshold */
#elif BIG_ENDIAN == 1
  uint8_t sigMotThres;    /**< sig-motion threshold */
  uint8_t sigEn :1;       /**< 1 - enable sig, 0 - disable sig & enable anymotion */
  uint8_t sigDataSrc :1;  /**< data source 0- filter & 1 pre-filter*/
  uint8_t sigMotProof : 2;/**< proof time of sig-motion interrupt */
  uint8_t sigMotSkip : 2; /**< skip time of sig-motion interrupt */
#endif
}sBmx160AccSigMotIntCfg_t;

/**
 * @struct sBmx160AccStepDetectIntCfg_t
 * @brief  Step detector interrupt structure
 */
typedef struct  {
#if LITTLE_ENDIAN == 1
  uint16_t stepDetectorEn :1;    /**< 1- step detector enable, 0- step detector disable */
  uint16_t minThreshold :2;      /**< minimum threshold */
  uint16_t steptimeMin :3;       /**< minimal detectable step time */
  uint16_t stepDetectorMode :2;  /**< enable step counter mode setting */
  uint16_t stepMinBuf :3;        /**< minimum step buffer size*/
#elif BIG_ENDIAN == 1 
  uint16_t stepMinBuf :3;        /**< minimum step buffer size*/
  uint16_t stepDetectorMode : 2; /**< enable step counter mode setting */
  uint16_t steptimeMin : 3;      /**< minimal detectable step time */
  uint16_t minThreshold : 2;     /**< minimum threshold */
  uint16_t stepDetectorEn :1;    /**< 1- step detector enable, 0- step detector disable */
#endif
}sBmx160AccStepDetectIntCfg_t;

/**
 * @struct sBmx160AccNoMotionIntCfg_t
 * @brief  No motion interrupt structure
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint16_t noMotionX :1;     /**< no motion interrupt x */
  uint16_t noMotionY :1;     /**< no motion interrupt y */
  uint16_t noMotionZ :1;     /**< no motion interrupt z */
  uint16_t noMotionDur :6;   /**< no motion duration */
  uint16_t noMotionSel :1;   /**< no motion sel , 1 - enable no-motion ,0- enable slow-motion */
  uint16_t noMotionSrc :1;   /**< data source 0- filter & 1 pre-filter*/
  uint8_t noMotionThres;     /**< no motion threshold */
#elif BIG_ENDIAN == 1
  uint8_t noMotionThres;     /**< no motion threshold */
  uint16_t noMotionSrc :1;   /**< data source 0- filter & 1 pre-filter*/
  uint16_t noMotionSel : 1;  /**< no motion sel , 1 - enable no-motion ,0- enable slow-motion */
  uint16_t noMotionDur : 6;  /**< no motion duration */
  uint16_t noMotionZ :1;     /**<no motion interrupt z */
  uint16_t noMotionY :1;     /**< no motion interrupt y */
  uint16_t noMotionX :1;     /**< no motion interrupt x */
#endif
}sBmx160AccNoMotionIntCfg_t;

/**
 * @struct sBmx160AccOrientIntCfg_t
 * @brief  Orientation interrupt structure
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint16_t orientMode :2;      /**< thresholds for switching between the different orientations */
  uint16_t orientBlocking :2;  /**< blocking_mode */
  uint16_t orientHyst :4;      /**< Orientation interrupt hysteresis */
  uint16_t orientTheta :6;     /**< Orientation interrupt theta */
  uint16_t orientUdEn :1;      /**< Enable/disable Orientation interrupt */
  uint16_t axesEx :1;          /**< exchange x- and z-axis in algorithm ,0 - z, 1 - x */
  uint8_t orientEn :1;         /**< 1 - orient enable, 0 - orient disable */
#elif BIG_ENDIAN == 1
  uint8_t orientEn :1;         /**< 1 - orient enable, 0 - orient disable */
  uint16_t axesEx : 1;         /**< exchange x- and z-axis in algorithm ,0 - z, 1 - x */
  uint16_t orientUdEn : 1;     /**< Enable/disable Orientation interrupt */
  uint16_t orientTheta : 6;    /**< Orientation interrupt theta */
  uint16_t orientHyst : 4;     /**< Orientation interrupt hysteresis */
  uint16_t orientBlocking : 2; /**< blocking_mode */
  uint16_t orientMode : 2;     /**< thresholds for switching between the different orientations */
#endif
}sBmx160AccOrientIntCfg_t;

/**
 * @struct sBmx160AccFlatDetectIntCfg_t
 * @brief  Flat interrupt structure 
 */
typedef struct  {
#if LITTLE_ENDIAN == 1
  uint16_t flatTheta :6;        /**< flat threshold */
  uint16_t flatHy :3;           /**< flat interrupt hysteresis */
  uint16_t flatHoldTime :2;     /**< delay time for which the flat value must remain stable for the flat interrupt to be generated */
  uint16_t flatEn :1;           /**< 1 - flat enable, 0 - flat disable */
#elif BIG_ENDIAN == 1
  uint16_t flatEn :1;           /**< 1 - flat enable, 0 - flat disable */
  uint16_t flatHoldTime : 2;    /**< delay time for which the flat value must remain stable for the flat interrupt to be generated */
  uint16_t flatHy : 3;          /**< flat interrupt hysteresis */
  uint16_t flatTheta : 6;       /**< flat threshold */
#endif
}sBmx160AccFlatDetectIntCfg_t;

/**
 * @struct sBmx160AccLowGIntCfg_t
 * @brief  Low-g interrupt structure
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint8_t lowDur;        /**< low-g interrupt trigger delay */
  uint8_t lowThres;      /**< low-g interrupt trigger threshold */
  uint8_t lowHyst :2;    /**< hysteresis of low-g interrupt */
  uint8_t lowMode :1;    /**< 0 - single-axis mode ,1 - axis-summing mode */
  uint8_t lowDataSrc :1; /**< data source 0- filter & 1 pre-filter */
  uint8_t lowEn :1;      /**< 1 - enable low-g, 0 - disable low-g */
#elif BIG_ENDIAN == 1 
  uint8_t lowEn :1;      /**< 1 - enable low-g, 0 - disable low-g */
  uint8_t lowDataSrc :1; /**< data source 0- filter & 1 pre-filter */
  uint8_t lowMode : 1;   /**< 0 - single-axis mode ,1 - axis-summing mode */
  uint8_t lowHyst : 2;   /**< hysteresis of low-g interrupt */
  uint8_t lowThres;      /**< low-g interrupt trigger threshold */
  uint8_t lowDur;        /**< low-g interrupt trigger delay */
#endif
}sBmx160AccLowGIntCfg_t;

/**
 * @struct sBmx160AccHighGIntCfg_t
 * @brief  High-g interrupt structure
 */
typedef struct {
#if LITTLE_ENDIAN == 1
  uint8_t high_g_x :1;      /**< High-g interrupt x, 1 - enable, 0 - disable */
  uint8_t high_g_y :1;      /**< High-g interrupt y, 1 - enable, 0 - disable */
  uint8_t high_g_z :1;      /**< High-g interrupt z, 1 - enable, 0 - disable */
  uint8_t highHy :2;        /**< High-g hysteresis  */
  uint8_t highDataSrc :1;   /**< data source 0- filter & 1 pre-filter */
  uint8_t highThres;        /**< High-g threshold */
  uint8_t highDur;          /**< High-g duration */
#elif BIG_ENDIAN == 1
  uint8_t highDur;          /**< High-g duration */
  uint8_t highThres;        /**< High-g threshold */
  uint8_t highDataSrc :1;   /**< data source 0- filter & 1 pre-filter */
  uint8_t highHy : 2;       /**< High-g hysteresis  */
  uint8_t high_g_z : 1;     /**< High-g interrupt z, 1 - enable, 0 - disable */
  uint8_t high_g_y : 1;     /**< High-g interrupt y, 1 - enable, 0 - disable */
  uint8_t high_g_x : 1;     /**< High-g interrupt x, 1 - enable, 0 - disable */
#endif
}sBmx160AccHighGIntCfg_t;

/**
 * @union uBmx160IntTypeCfg_t
 * @brief Union configures required interrupt 
 */
typedef union  {
  sBmx160AccTapIntCfg_t accTapInt;               /**< Tap interrupt structure */
  sBmx160AccAnyMotIntCfg_t accAnyMotionInt;      /**< Slope interrupt structure */
  sBmx160AccSigMotIntCfg_t accSigMotionInt;      /**< Significant motion interrupt structure */
  sBmx160AccStepDetectIntCfg_t accStepDetectInt; /**< Step detector interrupt structure */
  sBmx160AccNoMotionIntCfg_t accNoMotionInt;     /**< No motion interrupt structure */
  sBmx160AccOrientIntCfg_t accOrientInt;         /**< Orientation interrupt structure */
  sBmx160AccFlatDetectIntCfg_t accFlatInt;       /**< Flat interrupt structure */
  sBmx160AccLowGIntCfg_t accLowGInt;             /**< Low-g interrupt structure */
  sBmx160AccHighGIntCfg_t accHighGInt;           /**< High-g interrupt structure */
}uBmx160IntTypeCfg_t;

/**
 * @union sBmx160IntSettg_t
 */
typedef struct  {
  eBmx160IntChannel_t intChannel;     /**< Interrupt channel */
  eBmx160IntTypes_t intType;          /**< Select Interrupt */
  sBmx160IntPinSettg_t intPinSettg;   /**< Structure configuring Interrupt pins */
  uBmx160IntTypeCfg_t intTypeCfg;     /**< Union configures required interrupt */
  uint8_t fifoFullIntEn :1;           /**< FIFO FULL INT 1-enable, 0-disable */
  uint8_t fifoWTMIntEn :1;            /**< FIFO WTM INT 1-enable, 0-disable */
}sBmx160IntSettg_t;

/**
 * @enum eGyroRange_t
 */
typedef enum{
    eGyroRange_2000DPS,   /**< Gyroscope sensitivity at 2000dps*/
    eGyroRange_1000DPS,   /**< Gyroscope sensitivity at 1000dps*/
    eGyroRange_500DPS,    /**< Gyroscope sensitivity at 500dps*/
    eGyroRange_250DPS,    /**< Gyroscope sensitivity at 250dps*/
    eGyroRange_125DPS     /**< Gyroscope sensitivity at 125dps*/
}eGyroRange_t;

/**
 * @enum eAccelRange_t
 */
typedef enum{
    eAccelRange_2G  = 0b0011,   /**< Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
    eAccelRange_4G  = 0b0101,   /**< Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
    eAccelRange_8G  = 0b1000,   /**< Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
    eAccelRange_16G = 0b1100,   /**< Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */
}eAccelRange_t;

#define EARTH_ACC 9.81f

class DFRobot_BMX160{
  public:
    DFRobot_BMX160(TWIM *i2c=&I2C2);
    
    /**
     * @fn begin
     * @brief set the i2c addr and init the i2c.
     * @return returns the initialization status
     * @retval true Initialization succeeded
     * @retval false Initialization  failed
     */
    bool begin();

    /**
     * @fn setGyroRange
     * @brief set gyroscope angular rate range and resolution.
     * @param bits 
     * @n       eGyroRange_2000DPS     Gyroscope sensitivity at 2000dps
     * @n       eGyroRange_1000DPS     Gyroscope sensitivity at 1000dps
     * @n       eGyroRange_500DPS      Gyroscope sensitivity at 500dps
     * @n       eGyroRange_250DPS      Gyroscope sensitivity at 250dps
     * @n       eGyroRange_125DPS      Gyroscope sensitivity at 125dps
     */
    void setGyroRange(eGyroRange_t bits);

    /**
     * @fn setAccelRange
     * @brief allow the selection of the accelerometer g-range.
     * @param bits 
     * @n       eAccelRange_2G        Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) 
     * @n       eAccelRange_4G        Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) 
     * @n       eAccelRange_8G        Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) 
     * @n       eAccelRange_16G       Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg)
     */
    void setAccelRange(eAccelRange_t bits);

    void setMagnODR(uint8_t val);
    void setGyroODR(uint8_t val);
    void setAccelODR(uint8_t val);
    
    /**
     * @fn getAllData
     * @brief get the magn, gyro and accel data 
     * @param magn  to store the magn data
     * @param gyro  to store the gyro data
     * @param accel  to store the accel data
     */
    void getAllData( sBmx160SensorData_t *magn,  sBmx160SensorData_t *gyro,  sBmx160SensorData_t *accel);

    /**
     * @fn softReset
     * @brief reset bmx160 hardware
     * @return returns the reset status
     * @retval true reset succeeded
     * @retval false reset  failed
     */
    bool softReset();

    /**
     * @fn setLowPower
     * @brief disabled the the magn, gyro sensor to reduce power consumption
     */
    void setLowPower();

    /**
     * @fn wakeUp
     * @brief enabled the the magn, gyro sensor
     */
    void wakeUp();
    
  private:

    /**
     * @fn softReset
     * @brief reset bmx160 hardware
     * @return reset result
     * @retval     BMX160_OK          0
     * @retval     BMX160_E_NULL_PTR  -1
     */
    int8_t _softReset(sBmx160Dev_t *dev);

    /**
     * @fn defaultParamSettg
     * @brief Setting default Parameters
     * @param dev
     */
    void   defaultParamSettg(sBmx160Dev_t *dev);

    /**
     * @fn readReg
     * @brief get the sensor IIC data
     * @param reg register
     * @param pBuf write the store and buffer of the data
     * @param len data length to be readed
     */
    void readReg(uint8_t reg, uint8_t *pBuf, uint16_t len);

    /**
     * @fn writeReg
     * @brief write the sensor IIC data
     * @param reg register
     * @param pBuf write the store and buffer of the data
     * @param len data length to be written 
     * @return return the actually written length
     */
    void writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len);

    /**
     * @fn writeBmxReg
     * @brief Write data to the BMX register
     * @param reg register
     * @param value  Data written to the BMX register
     * @return return the actually written length
     */
    void writeBmxReg(uint8_t reg, uint8_t value);

    /**
     * @fn scan
     * @brief  iic scan function
     * @return scan result
     * @retval true sensor exist
     * @retval false There is no sensor
     */
    bool scan();

    /**
     * @fn setMagnConf
     * @brief  set magnetometer Config
     */
    void setMagnConf();

    float accelRange = BMX160_ACCEL_MG_LSB_2G * EARTH_ACC;
    float gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
    uint8_t _addr = DT_REG_ADDR(DT_NODELABEL(bmx160));
    
    sBmx160Dev_t* Obmx160;

    sBmx160SensorData_t* Omagn;
    sBmx160SensorData_t* Oaccel;
    sBmx160SensorData_t* Ogyro; 
    TWIM *_i2c;
};
