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
 */

#ifndef _BMA580_SENSOR_H
#define _BMA580_SENSOR_H

#include <stdio.h>
#include "bma5.h"
//#include "Wire.h"
#include <TWIM.h>

/******************************************************************************/
/*!                Macro definition                                           */

/*! FIFO raw data buffer size */
#define BMA580_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(520)

/*! Number of accel frames to be extracted from FIFO
 * Calculation:
 * fifo_watermark_level = 300, accel_frame_len = 6, header_byte = 1, sensortime = 3.
 * fifo_accel_frame_count = (300 / (6 + 1 + 3)) = 30 frames
 *
 * Additional frames given if received FIFO frames is more than WATERMARK_LEVEL set.
 */

/*! FIFO watermark level */
#define WATERMARK_LEVEL                   UINT16_C(128)

/*! Sensortime resolution in seconds */
#define SENSORTIME_RESOLUTION             (0.0003125f)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH                     (9.80665f)

/******************************************************************************/
/*!                       Macro definitions                                   */

/*! Enum to string converter*/
#ifndef enum_to_string
#define enum_to_string(a)  #a
#endif

/***************************************************************************/

/*!                 User function prototypes
 ****************************************************************************/

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr      : Register address.
 *  @param[out] reg_data     : Pointer to the data buffer to store the read data.
 *  @param[in] length        : No of bytes to read.
 *  @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA5_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA5_INTF_RET_SUCCESS -> Fail.
 *
 */
BMA5_INTF_RET_TYPE bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr      : Register address.
 *  @param[in] reg_data      : Pointer to the data buffer whose value is to be written.
 *  @param[in] length        : No of bytes to write.
 *  @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA5_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA5_INTF_RET_SUCCESS -> Failure.
 *
 */
BMA5_INTF_RET_TYPE bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us      : The required wait time in microsecond.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *  @return void.
 *
 */
void bma5_delay_us(uint32_t period_us, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *         Also to initialize coines platform.
 *
 *  @param[in] bma5     : Structure instance of bma5_dev
 *  @param[in] intf     : Interface selection parameter
 *  @param[in] context  : Context parameter selection
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bma5_interface_init(struct bma5_dev *bma5, uint8_t intf, enum bma5_context context);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
bool bma5_check_rslt(const char api_name[], int8_t rslt);

class BMA580 {
public:
    int init(int odr = BMA5_ACC_ODR_HZ_100, int fifo_watermark_level = WATERMARK_LEVEL);
    int start();
    int stop();
    int read(bma5_sens_fifo_axes_data_16_bit *fifo_accel_data);
private:
    //TWIM & _i2cPort = I2C3;

    int _odr;

    struct bma5_dev dev;
    struct bma5_fifo_conf fifo_conf;
    
    /*! Number of bytes of FIFO data */
    uint8_t fifo_data[BMA580_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Initialize FIFO frame structure */
    struct bma5_fifo_frame fifoframe = { 0 };

    /* Set FIFO water-mark level */
    uint16_t fifo_watermark_level = WATERMARK_LEVEL;

    int8_t get_accel_and_int_settings(struct bma5_dev *dev);

    int8_t get_fifo_conf(const struct bma5_fifo_conf *fifo_conf, struct bma5_dev *dev);
};

#endif /* _COMMON_H */
