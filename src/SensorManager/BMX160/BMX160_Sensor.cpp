#include "BMX160_Sensor.h"

#include "bmi160.h"
#include "bmi160_defs.h"

#include <TWIM.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BMX160, 3);

/* BMX160 combo chip: I2C address 0x68, BMM150 accessed via BMI160 aux bus.
 * Mag-interface register layout (per BMI160 datasheet):
 *   0x4B MAG_IF_0  aux device I2C address
 *   0x4C MAG_IF_1  control (bit7 = manual mode)
 *   0x4D MAG_IF_2  read register address
 *   0x4E MAG_IF_3  write data
 *   0x4F MAG_IF_4  write register address
 * BMX160 has BMM150 wired internally at aux address 0x10 which is the
 * reset default in MAG_IF_0, so we don't need to set it explicitly.
 * The configuration sequence below mirrors the working DFRobot driver
 * and puts BMM150 into "data mode" so its X/Y/Z/Rhall are auto-shadowed
 * into BMI160 data registers 0x04..0x0B. */
#define BMX160_I2C_ADDR           0x68
#define BMX160_DATA_MAG_ADDR      0x04
#define BMX160_MAG_IF_1           0x4C
#define BMX160_MAG_IF_2           0x4D
#define BMX160_MAG_IF_3           0x4E
#define BMX160_MAG_IF_4           0x4F
#define BMX160_MAG_CONF           0x44
#define BMX160_CMD_REG            0x7E

/* Scaling factors chosen to preserve the unit output of the legacy
 * DFRobot driver (accel in m/s^2 at 2G, gyro in deg/s scaled as if
 * 250 dps, mag in uT). Gyro scale matches what the legacy driver
 * reported even though the chip is actually running at 2000 dps —
 * preserving this so downstream consumers see no step change. */
#define BMX160_ACCEL_LSB_TO_MS2_2G    (0.000061035f * 9.81f)
#define BMX160_GYRO_LSB_TO_DPS_250    (0.0076220f)
#define BMX160_MAG_LSB_TO_UT          (0.3f)

/* Static context so Bosch callbacks can reach the bus. Bosch's callback
 * typedef doesn't take a user context pointer, so we stash it here. */
static TWIM * s_i2c = &I2C3;

static int8_t bmi160_i2c_read(uint8_t dev_addr, uint8_t reg_addr,
                              uint8_t *data, uint16_t len)
{
    s_i2c->acquire();
    int ret = i2c_burst_read(s_i2c->master, dev_addr, reg_addr, data, len);
    s_i2c->release();
    if (ret) {
        LOG_WRN("BMX160 I2C read failed: %d (reg 0x%02X len %u)",
                ret, reg_addr, len);
        return -1;
    }
    return 0;
}

static int8_t bmi160_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
                               uint8_t *data, uint16_t len)
{
    s_i2c->acquire();
    int ret = i2c_burst_write(s_i2c->master, dev_addr, reg_addr, data, len);
    s_i2c->release();
    if (ret) {
        LOG_WRN("BMX160 I2C write failed: %d (reg 0x%02X len %u)",
                ret, reg_addr, len);
        return -1;
    }
    return 0;
}

static void bmi160_delay_ms(uint32_t period)
{
    k_msleep(period);
}

int8_t BMX160::configure_magnetometer()
{
#ifndef CONFIG_IMU_ENABLE_MAGNETOMETER
    /* Magnetometer gated off: keep the BMM150 subsystem in suspend so it
     * doesn't burn ~170 uA for readings that the speaker magnet swamps
     * anyway. The data registers will stay at reset (zero); the drain
     * path skips read_mag_latched() and zero-fills the mag fields. */
    uint8_t mag_suspend = BMI160_AUX_SUSPEND_MODE;
    int8_t rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &mag_suspend, 1, &dev);
    if (rslt != BMI160_OK) {
        LOG_WRN("mag suspend CMD failed: %d", rslt);
    }
    return rslt;
#else
    /* Power on the internal BMM150 subsystem. CMD 0x19 = aux/mag interface
     * normal power mode. bmi160_set_sens_conf only issues CMDs for accel
     * and gyro; without this write the MAG_IF writes below go to a
     * powered-off magnetometer and produce all-zero readings. */
    uint8_t mag_normal = BMI160_AUX_NORMAL_MODE;
    int8_t rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &mag_normal, 1, &dev);
    if (rslt != BMI160_OK) {
        LOG_WRN("mag power-on CMD failed: %d", rslt);
        return rslt;
    }
    k_msleep(10);

    /* Replicates DFRobot_BMX160::setMagnConf() register sequence. The
     * BMM150 init dance puts the magnetometer into "data mode" where
     * BMI160's MAG_IF interface auto-polls BMM150 at the configured
     * rate and shadows the result to data registers 0x04..0x0B. */
    struct {
        uint8_t reg;
        uint8_t val;
        uint32_t post_delay_ms;
    } seq[] = {
        { BMX160_MAG_IF_1, 0x80, 50 }, /* enter aux setup mode */
        { BMX160_MAG_IF_4, 0x01,  0 }, /* BMM150 sleep */
        { BMX160_MAG_IF_3, 0x4B,  0 },
        { BMX160_MAG_IF_4, 0x04,  0 }, /* REPXY regular preset */
        { BMX160_MAG_IF_3, 0x51,  0 },
        { BMX160_MAG_IF_4, 0x0E,  0 }, /* REPZ regular preset */
        { BMX160_MAG_IF_3, 0x52,  0 },
        { BMX160_MAG_IF_4, 0x02,  0 }, /* put BMM150 in forced-mode */
        { BMX160_MAG_IF_3, 0x4C,  0 },
        { BMX160_MAG_IF_2, 0x42,  0 }, /* read start addr = DATAX_LSB */
        { BMX160_MAG_CONF, 0x08,  0 }, /* ODR = 100 Hz */
        { BMX160_MAG_IF_1, 0x03, 50 }, /* data mode, 8-byte burst */
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        rslt = bmi160_set_regs(seq[i].reg, &seq[i].val, 1, &dev);
        if (rslt != BMI160_OK) {
            LOG_WRN("mag setup step %u failed: %d", (unsigned)i, rslt);
            return rslt;
        }
        if (seq[i].post_delay_ms) k_msleep(seq[i].post_delay_ms);
    }
    return BMI160_OK;
#endif
}

int BMX160::read_mag_latched(float out_mag[3])
{
    /* Single 6-byte read from BMI160's shadowed mag data (X/Y/Z LSB+MSB,
     * skipping Rhall). BMM150 is running at 100 Hz via data mode so this
     * returns the most recent sample with no extra transactions per IMU
     * sample. */
    uint8_t buf[6] = { 0 };
    int8_t rslt = bmi160_get_regs(BMX160_DATA_MAG_ADDR, buf, 6, &dev);
    if (rslt != BMI160_OK) return rslt;

    int16_t x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    int16_t y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    out_mag[0] = (float)x * BMX160_MAG_LSB_TO_UT;
    out_mag[1] = (float)y * BMX160_MAG_LSB_TO_UT;
    out_mag[2] = (float)z * BMX160_MAG_LSB_TO_UT;
    return 0;
}

int BMX160::init(uint8_t accel_odr_reg, uint8_t gyro_odr_reg,
                 uint16_t fifo_watermark_bytes)
{
    s_i2c->begin();

    dev.id        = BMX160_I2C_ADDR;
    dev.intf      = BMI160_I2C_INTF;
    dev.read      = bmi160_i2c_read;
    dev.write     = bmi160_i2c_write;
    dev.delay_ms  = bmi160_delay_ms;
    dev.read_write_len = 128;

    int8_t rslt = bmi160_init(&dev);
    if (rslt != BMI160_OK) {
        LOG_ERR("bmi160_init failed: %d (chip_id=0x%02X)", rslt, dev.chip_id);
        return rslt;
    }

    dev.accel_cfg.odr   = accel_odr_reg;
    dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    dev.accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;
    dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    dev.gyro_cfg.odr   = gyro_odr_reg;
    dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev.gyro_cfg.bw    = BMI160_GYRO_BW_NORMAL_MODE;
    dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    rslt = bmi160_set_sens_conf(&dev);
    if (rslt != BMI160_OK) {
        LOG_ERR("bmi160_set_sens_conf failed: %d", rslt);
        return rslt;
    }

    _accel_lsb_to_ms2 = BMX160_ACCEL_LSB_TO_MS2_2G;
    _gyro_lsb_to_dps  = BMX160_GYRO_LSB_TO_DPS_250;

    /* BMM150 magnetometer setup through BMI160 aux interface */
    (void)configure_magnetometer();

    /* Headered FIFO with accel + gyro (mag is read separately). */
    fifo_frame.data                = fifo_buf;
    fifo_frame.length              = BMX160_FIFO_RAW_DATA_BUFFER_SIZE;
    fifo_frame.fifo_time_enable    = 0;
    fifo_frame.fifo_header_enable  = BMI160_FIFO_HEAD_ENABLE;
    fifo_frame.fifo_data_enable    = BMI160_FIFO_G_A_ENABLE;
    dev.fifo = &fifo_frame;

    /* BMI160 FIFO watermark reg is in units of 4 bytes. */
    uint8_t wm = (uint8_t)MIN(fifo_watermark_bytes / 4, 0xFFU);
    rslt = bmi160_set_fifo_wm(wm, &dev);
    if (rslt != BMI160_OK) LOG_WRN("bmi160_set_fifo_wm: %d", rslt);

    /* Enable A+G in FIFO + headered mode. Flush first. */
    rslt = bmi160_set_fifo_flush(&dev);
    if (rslt != BMI160_OK) LOG_WRN("bmi160_set_fifo_flush: %d", rslt);

    uint8_t fifo_cfg = BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO | BMI160_FIFO_HEADER;
    rslt = bmi160_set_fifo_config(fifo_cfg, BMI160_ENABLE, &dev);
    if (rslt != BMI160_OK) {
        LOG_ERR("bmi160_set_fifo_config: %d", rslt);
        return rslt;
    }

    return BMI160_OK;
}

int BMX160::start()
{
    /* sens_conf already put accel/gyro in NORMAL mode. Flush stale frames. */
    int8_t rslt = bmi160_set_fifo_flush(&dev);
    if (rslt != BMI160_OK) LOG_WRN("start: fifo flush: %d", rslt);
    return rslt;
}

int BMX160::stop()
{
    /* Park accel and gyro in suspend to drop power. */
    dev.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    dev.gyro_cfg.power  = BMI160_GYRO_SUSPEND_MODE;
    return bmi160_set_sens_conf(&dev);
}

int BMX160::read(BMX160Sample *out, int max_samples)
{
    if (out == nullptr || max_samples <= 0) return 0;

    /* Read whatever is in the FIFO — timer-polled at the batch rate. */
    fifo_frame.length = BMX160_FIFO_RAW_DATA_BUFFER_SIZE;
    int8_t rslt = bmi160_get_fifo_data(&dev);
    if (rslt != BMI160_OK) {
        LOG_WRN("bmi160_get_fifo_data: %d", rslt);
        return 0;
    }

    uint8_t accel_len = BMX160_FIFO_MAX_FRAMES;
    uint8_t gyro_len  = BMX160_FIFO_MAX_FRAMES;
    (void)bmi160_extract_accel(accel_frames, &accel_len, &dev);
    (void)bmi160_extract_gyro(gyro_frames, &gyro_len, &dev);

    /* Accel/gyro frame counts should match in A+G headered mode; if
     * they diverge (shouldn't in steady state, but can at startup or
     * overflow), use the smaller count so we never emit a sample with
     * mismatched halves. */
    int n = MIN((int)accel_len, (int)gyro_len);
    if (n > max_samples) n = max_samples;

    float mag[3] = { 0, 0, 0 };
#ifdef CONFIG_IMU_ENABLE_MAGNETOMETER
    if (n > 0) {
        (void)read_mag_latched(mag);
    }
#endif

    for (int i = 0; i < n; i++) {
        out[i].accel[0] = (float)accel_frames[i].x * _accel_lsb_to_ms2;
        out[i].accel[1] = (float)accel_frames[i].y * _accel_lsb_to_ms2;
        out[i].accel[2] = (float)accel_frames[i].z * _accel_lsb_to_ms2;
        out[i].gyro[0]  = (float)gyro_frames[i].x  * _gyro_lsb_to_dps;
        out[i].gyro[1]  = (float)gyro_frames[i].y  * _gyro_lsb_to_dps;
        out[i].gyro[2]  = (float)gyro_frames[i].z  * _gyro_lsb_to_dps;
        out[i].mag[0]   = mag[0];
        out[i].mag[1]   = mag[1];
        out[i].mag[2]   = mag[2];
    }
    return n;
}
