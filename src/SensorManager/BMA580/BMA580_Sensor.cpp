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
#include <stdint.h>

#include "bma5.h"
#include "BMA580_Sensor.h"

#include "bma580_features.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BMA580, 3);

/* Variable to store the device address */

struct BMA580_dev_inf {
    uint8_t addr;
    TWIM * i2c_dev;
};

BMA580_dev_inf dev_info = {
    .addr = BMA5_I2C_ADDRESS,
    .i2c_dev = &I2C3
};

/*!
 * @brief I2C read function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int ret;
    BMA580_dev_inf * dev_info = (BMA580_dev_inf *) intf_ptr;

    dev_info->i2c_dev->acquire();

    ret = i2c_burst_read(dev_info->i2c_dev->master, dev_info->addr, reg_addr, reg_data, len);
    if (ret) LOG_WRN("I2C read failed: %d\n", ret);

    dev_info->i2c_dev->release();
    return 0;
}

/*!
 * @brief I2C write function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    BMA580_dev_inf * dev_info = (BMA580_dev_inf *) intf_ptr; 

    dev_info->i2c_dev->acquire();
    
    int ret = i2c_burst_write(dev_info->i2c_dev->master, dev_info->addr, reg_addr, reg_data, len);
    if (ret) LOG_WRN("I2C write failed: %d", ret);

    dev_info->i2c_dev->release();

    return 0;
}

/*!
 * @brief Delay function map to COINES platform
 */
void bma5_delay_us(uint32_t period, void *intf_ptr)
{
    k_usleep(period);
}

bool bma5_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMA5_OK:

            /* Do nothing */
            return true;
        case BMA5_E_NULL_PTR:
            LOG_ERR("API name %s\tError  [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMA5_E_COM_FAIL:
            LOG_ERR("API name %s\tError  [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMA5_E_DEV_NOT_FOUND:
            LOG_ERR("API name %s\tError  [%d] : Device not found\r\n", api_name, rslt);
            break;
        default:
            LOG_ERR("API name %s\tError  [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }

    return false;
}

int8_t bma5_interface_init(struct bma5_dev *bma5, uint8_t intf, enum bma5_context context)
{
    int8_t rslt = BMA5_OK;

    if (bma5 != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMA5_I2C_INTF)
        {
            dev_info.addr = DT_REG_ADDR(DT_NODELABEL(bma580)); //BMA5_I2C_ADDRESS;
            bma5->bus_read = bma5_i2c_read;
            bma5->bus_write = bma5_i2c_write;
            bma5->intf = BMA5_I2C_INTF;

            dev_info.i2c_dev->begin();
        }

        /* Holds the I2C device addr or SPI chip selection */
        bma5->intf_ptr = &dev_info;

        /* Configure delay in microseconds */
        bma5->delay_us = bma5_delay_us;

        /* Assign context parameter */
        bma5->context = context;
    }
    else
    {
        rslt = BMA5_E_NULL_PTR;
    }

    return rslt;
}


/*!
 * @brief This internal API is used to enable accel and interrupt configuration settings.
 */
int8_t BMA580::get_accel_and_int_settings(struct bma5_dev *dev)
{
    int8_t rslt;
    uint8_t n_ints = 1;
    uint8_t sensor_ctrl;
    struct bma5_acc_conf acc_cfg, get_acc_cfg;
    struct bma5_int_conf_types int_config;

    int_config.int_src = BMA5_INT_1;

    /* Get accel configurations */
    rslt = bma5_get_acc_conf_0(&sensor_ctrl, dev);
    bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    rslt = bma5_get_acc_conf(&acc_cfg, dev);
    bma5_check_rslt("bma5_get_acc_conf", rslt);

    /* Set accel configurations */
    //acc_cfg.acc_odr = BMA5_ACC_ODR_HZ_6K4;
	acc_cfg.acc_odr = _odr;
    acc_cfg.acc_bwp = BMA5_ACC_BWP_OSR4_AVG1;
    acc_cfg.power_mode = BMA5_POWER_MODE_HPM;

    acc_cfg.acc_range = BMA5_ACC_RANGE_MAX_2G;
    acc_cfg.acc_iir_ro = BMA5_ACC_IIR_RO_DB_60;
    acc_cfg.noise_mode = BMA5_NOISE_MODE_LOWER_NOISE;
    acc_cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_DISABLED;

    rslt = bma5_set_acc_conf(&acc_cfg, dev);
    bma5_check_rslt("bma5_get_acc_conf", rslt);

    /*LOG_DBG("Accel configurations");
    LOG_DBG("ODR : %s\t", enum_to_string(acc_cfg.acc_odr));
    LOG_DBG("Bandwidth : %s\t", enum_to_string(BMA5_ACC_BWP_NORM_AVG4));
    LOG_DBG("Power mode : %s\t", enum_to_string(BMA5_POWER_MODE_HPM));
    LOG_DBG("Range : %s\t", enum_to_string(BMA5_ACC_RANGE_MAX_2G));
    LOG_DBG("IIR RO : %s\t", enum_to_string(BMA5_ACC_IIR_RO_DB_60));
    LOG_DBG("Noise mode : %s\t", enum_to_string(BMA5_NOISE_MODE_LOWER_POWER));
    LOG_DBG("Auto Int clear : %s\t", enum_to_string(BMA5_ACC_DRDY_INT_AUTO_CLEAR_DISABLED));*/

    /* Enable accel */
    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;

    rslt = bma5_set_acc_conf_0(sensor_ctrl, dev);
    bma5_check_rslt("bma5_set_acc_conf_0", rslt);

    if (rslt == BMA5_OK)
    {
        LOG_DBG("Accel enabled");
    }

    rslt = bma5_get_acc_conf_0(&sensor_ctrl, dev);
    bma5_check_rslt("bma5_set_acc_conf_0", rslt);

    rslt = bma5_get_acc_conf(&get_acc_cfg, dev);
    bma5_check_rslt("bma5_get_acc_conf", rslt);

    rslt = bma5_get_int_conf(&int_config, n_ints, dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    int_config.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;
    int_config.int_conf.int_od = BMA5_INT1_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_HIGH;

    rslt = bma5_set_int_conf(&int_config, n_ints, dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    /*LOG_DBG("\nInt Configurations");
    LOG_DBG("INT2 mode : %s\t", enum_to_string(BMA5_INT2_MODE_LATCHED));
    LOG_DBG("INT2 OD : %s\t", enum_to_string(BMA5_INT2_OD_PUSH_PULL));
    LOG_DBG("INT2 level : %s\t", enum_to_string(BMA5_INT2_LVL_ACTIVE_HIGH));*/

    return rslt;
}

/*!
 * @brief This internal API sets FIFO configurations and verifies the readback
 * matches what we asked for. The chip silently clamps fifo_size when the
 * feature engine is enabled (datasheet section 4.6.1.5), so a missing log here
 * is the only way to notice that condition without a logic analyzer.
 */
int8_t BMA580::get_fifo_conf(const struct bma5_fifo_conf *fifo_conf, struct bma5_dev *dev)
{
    int8_t rslt;
    struct bma5_fifo_conf read_fifo_conf = { 0 };

    /* Set FIFO configuration.
     * NOTE 1: FIFO works only on header mode */
    rslt = bma5_set_fifo_conf(fifo_conf, dev);
    bma5_check_rslt("bma5_set_fifo_conf", rslt);

    /* Get FIFO configuration register */
    rslt = bma5_get_fifo_conf(&read_fifo_conf, dev);
    bma5_check_rslt("bma5_get_fifo_conf", rslt);

    /* Verify that the chip accepted the size we asked for. */
    if (read_fifo_conf.fifo_size != fifo_conf->fifo_size) {
        LOG_ERR("FIFO size mismatch: requested 0x%02X, readback 0x%02X "
                "(feature engine likely still enabled)",
                fifo_conf->fifo_size, read_fifo_conf.fifo_size);
    }

    return rslt;
}

int BMA580::init(int odr, int fifo_watermark_level) {
    int8_t rslt;

    /* Assign context parameter selection */
    enum bma5_context context = BMA5_HEARABLE;

    _odr = odr;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_I2C_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

	rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);
    LOG_DBG("Chip ID :0x%X", dev.chip_id);

    /* Disable the feature engine. The BMA580 powers up with feat_eng enabled
     * (FEAT_ENG_CONF reset value = 0x01), and feat_eng shares its 1024-byte
     * RAM with the FIFO. While feat_eng is enabled the FIFO is capped at
     * 512 bytes and any write of fifo_size = 0x03 (1024 B) is silently
     * clamped to 0x02 (512 B) — verified empirically via the readback log
     * in get_fifo_conf(). We don't use any feature-engine features
     * (VAD, tap, generic interrupts, FOC, self-wakeup), so disabling it is
     * safe and gives the FIFO its full 1024 bytes. */
    rslt = bma5_set_feat_eng_conf(BMA5_FEAT_ENG_CTRL_DISABLE, &dev);
    bma5_check_rslt("bma5_set_feat_eng_conf", rslt);

    /* INT1/INT2 hardware interrupt pins are unused — FIFO is drained by a
     * timer-based poll in BoneConduction::update_sensor, so we don't map
     * the watermark or full interrupts to any pin. */

    rslt = get_accel_and_int_settings(&dev);
    bma5_check_rslt("get_accel_and_int_settings", rslt);

    /* Get FIFO configuration register */
    rslt = bma5_get_fifo_conf(&fifo_conf, &dev);
    bma5_check_rslt("bma5_get_fifo_conf", rslt);

    fifo_conf.fifo_cfg = BMA5_FIFO_CFG_ENABLE;
    fifo_conf.fifo_acc_x = BMA5_FIFO_ACC_X_ENABLE;
    fifo_conf.fifo_acc_y = BMA5_FIFO_ACC_Y_ENABLE;
    fifo_conf.fifo_acc_z = BMA5_FIFO_ACC_Z_ENABLE;
    fifo_conf.fifo_compression = BMA5_FIFO_COMPRESSION_ACC_16BIT;
    fifo_conf.fifo_sensor_time = BMA5_FIFO_SENSOR_TIME_OFF;
    fifo_conf.fifo_size = BMA5_FIFO_SIZE_MAX_1024_BYTES;
    fifo_conf.fifo_stop_on_full = BMA5_DISABLE;

    rslt = get_fifo_conf(&fifo_conf, &dev);
    bma5_check_rslt("get_fifo_conf", rslt);

    /* Update FIFO structure */
    fifoframe.data = fifo_data;

    rslt = bma5_set_fifo_wm(fifo_watermark_level, &dev);
    bma5_check_rslt("bma5_set_fifo_wm", rslt);

    return rslt;
}

int BMA580::start() {
    uint8_t on = 0;
    int8_t ret = bma5_set_cmd_suspend(on, &dev);
    return ret;
}

int BMA580::stop() {
    uint8_t off = 1;
    int8_t ret = bma5_set_cmd_suspend(off, &dev);
    return ret;
}

int BMA580::read(bma5_sens_fifo_axes_data_16_bit *fifo_accel_data) {
    int8_t rslt = BMA5_OK;

    fifoframe.fifo_avail_frames = 0;

    /* Read whatever is in the FIFO — don't wait for watermark interrupt.
     * When using a timer-based read at the FIFO fill rate, the watermark
     * may not have triggered yet due to clock drift between the nRF5340
     * and BMA580.  Reading unconditionally avoids this race. */
    rslt = bma5_read_fifo_data(&fifoframe, &fifo_conf, &dev);
    bma5_check_rslt("bma5_read_fifo_data", rslt);

    if (rslt == BMA5_OK && fifoframe.fifo_avail_len > 0)
    {
        (void)bma5_extract_acc_sens_time_16_bit(fifo_accel_data, &fifoframe, &fifo_conf, &dev);
    }

    return fifoframe.fifo_avail_frames;
}