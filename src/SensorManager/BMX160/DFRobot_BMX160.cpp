/*!
 * @file DFRobot_BMX160.cpp
 * @brief define DFRobot_BMX160 class infrastructure, the implementation of basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_BMX160
 */
#include "DFRobot_BMX160.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BMX160, CONFIG_MAIN_LOG_LEVEL);

#define delay(ms) k_msleep(ms)
#define malloc(a) k_malloc(a)

DFRobot_BMX160::DFRobot_BMX160(TWIM *i2c) : _i2c(i2c)
{
  Obmx160 = (sBmx160Dev_t *)malloc(sizeof(sBmx160Dev_t));
  Oaccel = ( sBmx160SensorData_t*)malloc(sizeof( sBmx160SensorData_t));
  Ogyro = ( sBmx160SensorData_t*)malloc(sizeof( sBmx160SensorData_t));
  Omagn = ( sBmx160SensorData_t*)malloc(sizeof( sBmx160SensorData_t));
}

const uint8_t int_mask_lookup_table[13] = {
    BMX160_INT1_SLOPE_MASK,
    BMX160_INT1_SLOPE_MASK,
    BMX160_INT2_LOW_STEP_DETECT_MASK,
    BMX160_INT1_DOUBLE_TAP_MASK,
    BMX160_INT1_SINGLE_TAP_MASK,
    BMX160_INT1_ORIENT_MASK,
    BMX160_INT1_FLAT_MASK,
    BMX160_INT1_HIGH_G_MASK,
    BMX160_INT1_LOW_G_MASK,
    BMX160_INT1_NO_MOTION_MASK,
    BMX160_INT2_DATA_READY_MASK,
    BMX160_INT2_FIFO_FULL_MASK,
    BMX160_INT2_FIFO_WM_MASK
};

bool DFRobot_BMX160::begin()
{
    _i2c->begin();

    if (!detect() || !softReset()) {
        return false;
    }

    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    delay(50);
    /* Set gyro to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    delay(100);
    /* Set mag to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
    delay(10);

    return setMagnConf();
}

bool DFRobot_BMX160::detect()
{
    _i2c->begin();

    uint8_t chip_id = 0;

    /* Revision 2.1 selects the alternate address by pulling BMI160 SDO high. */
    if (probeAddress(BMI160_I2C_ADDR_SDO_HIGH, &chip_id) && chip_id == BMI160_CHIP_ID) {
        _addr = BMI160_I2C_ADDR_SDO_HIGH;
        _standaloneBmi160 = true;
        LOG_INF("Standalone BMI160 detected at 0x%02x", _addr);
        return true;
    }

    if (probeAddress(BMI160_I2C_ADDR_SDO_LOW, &chip_id) &&
        (chip_id == BMX160_CHIP_ID || chip_id == BMI160_CHIP_ID)) {
        _addr = BMI160_I2C_ADDR_SDO_LOW;
        _standaloneBmi160 = false;
        LOG_INF("%s detected at 0x%02x", chip_id == BMX160_CHIP_ID ? "BMX160" : "BMI160", _addr);
        return true;
    }

    _standaloneBmi160 = false;
    LOG_WRN("No BMI160/BMX160 found at 0x68 or 0x69");
    return false;
}

bool DFRobot_BMX160::isStandaloneBmi160() const
{
    return _standaloneBmi160;
}

bool DFRobot_BMX160::probeAddress(uint8_t addr, uint8_t *chip_id)
{
    _i2c->aquire();
    const int ret = i2c_burst_read(_i2c->master, addr, BMX160_CHIP_ID_ADDR, chip_id, 1);
    _i2c->release();

    return ret == 0;
}

void DFRobot_BMX160::setLowPower(){
    softReset();
    delay(100);
    setMagnConf();
    delay(100);
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x12);
    delay(100);
    /* Set gyro to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x17);
    delay(100);
    /* Set mag to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x1B);
    delay(100);
}

void DFRobot_BMX160::wakeUp(){
    softReset();
    delay(100);
    setMagnConf();
    delay(100);
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
    delay(100);
    /* Set gyro to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
    delay(100);
    /* Set mag to normal mode */
    writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
    delay(100);
}

bool DFRobot_BMX160::softReset()
{
  int8_t rslt=BMX160_OK;
  if (Obmx160 == NULL){
    rslt = BMX160_E_NULL_PTR;
  }  
  rslt = _softReset(Obmx160);
  if (rslt == 0)
    return true;
  else
    return false;
}

int8_t DFRobot_BMX160:: _softReset(sBmx160Dev_t *dev)
{
  int8_t rslt=BMX160_OK;
  uint8_t data = BMX160_SOFT_RESET_CMD;
  if (dev==NULL){
    rslt = BMX160_E_NULL_PTR;
  }
  writeBmxReg(BMX160_COMMAND_REG_ADDR, data);
  delay(BMX160_SOFT_RESET_DELAY_MS);
  if (rslt == BMX160_OK){
    DFRobot_BMX160::defaultParamSettg(dev);
  }  
  return rslt;
}

void DFRobot_BMX160::defaultParamSettg(sBmx160Dev_t *dev)
{
  // Initializing accel and gyro params with
  dev->gyroCfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMX160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMX160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMX160_GYRO_RANGE_2000_DPS;

  dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMX160_ACCEL_RANGE_2G;

  dev->magnCfg.odr = BMX160_MAGN_ODR_100HZ;
  dev->magnCfg.power = BMX160_MAGN_SUSPEND_MODE;

  dev->prevMagnCfg = dev->magnCfg;
  dev->prevGyroCfg = dev->gyroCfg;
  dev->prevAccelCfg = dev->accelCfg;
}

bool DFRobot_BMX160::setMagnConf()
{
    _bmm150Ready = false;

    if (setBmi160AuxMagnConf()) {
        LOG_INF("BMM150 magnetometer configured through BMI160 auxiliary interface");
        return true;
    }

    LOG_ERR("BMM150 auxiliary probe or configuration failed");
    return false;
}

bool DFRobot_BMX160::setBmi160AuxMagnConf()
{
    uint8_t reg_val = 0;
    uint8_t chip_id = 0;

    if (!writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_MAGN_NORMAL_MODE)) {
        return false;
    }
    delay(1);

    if (!readReg(BMX160_IF_CONF_ADDR, &reg_val, 1)) {
        return false;
    }

    reg_val |= BMX160_IF_CONF_SECONDARY_IF_EN;
    if (!writeBmxReg(BMX160_IF_CONF_ADDR, reg_val)) {
        return false;
    }
    delay(BMX160_MAGN_COM_DELAY);

    if (!setBmi160AuxMode(true, 0x03)) {
        return false;
    }

    if (!writeBmm150Reg(BMM150_POWER_CONTROL_ADDR, BMM150_POWER_CONTROL_ENABLE)) {
        return false;
    }
    delay(BMX160_MAGN_COM_DELAY);

    if (!readBmm150Reg(BMM150_CHIP_ID_ADDR, &chip_id) || chip_id != BMM150_CHIP_ID) {
        LOG_WRN("BMM150 chip id mismatch: 0x%02x", chip_id);
        return false;
    }

    if (!readBmm150Trim() ||
        !writeBmm150Reg(BMM150_REP_XY_ADDR, BMM150_REP_XY_REGULAR) ||
        !writeBmm150Reg(BMM150_REP_Z_ADDR, BMM150_REP_Z_REGULAR) ||
        !writeBmm150Reg(BMM150_OP_MODE_ADDR, BMM150_OP_MODE_FORCED) ||
        !setBmi160AuxReadAddr(BMM150_DATA_X_LSB_ADDR) ||
        !writeBmxReg(BMX160_MAGN_CONFIG_ADDR, BMX160_MAGN_ODR_100HZ) ||
        !setBmi160AuxMode(false, 0x03)) {
        return false;
    }

    delay(50);
    _bmm150Ready = true;

    return true;
}

void DFRobot_BMX160::setBmx160MagnConf()
{
    // puts magnetometer into mag_if setup mode
    writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x80);
    delay(50);
    // Sleep mode
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x01);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4B);
    // REPXY regular preset
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x04);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);
    // REPZ regular preset
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x0E);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);
    // Prepare MAG_IF[1-3] for mag_if data mode
    writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
    writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
    writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);
    // sets the sampling rate t0 100Hz
    writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x08);
    // puts magnetometer into mag_if data mode sets data length of read burst operation to 8 bytes
    writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x03);
    delay(50);
}

bool DFRobot_BMX160::setBmi160AuxMode(bool manual, uint8_t read_burst_len)
{
    uint8_t aux_if[2] = {
        static_cast<uint8_t>(BMX160_MAGN_BMM150_I2C_ADDR << 1),
        static_cast<uint8_t>((manual ? BMX160_MANUAL_MODE_EN_MSK : 0x00) |
                             (read_burst_len & BMX160_MAGN_READ_BURST_MSK))
    };

    return writeReg(BMX160_AUX_IF_0_ADDR, aux_if, sizeof(aux_if));
}

bool DFRobot_BMX160::setBmi160AuxReadAddr(uint8_t reg)
{
    return writeBmxReg(BMX160_AUX_IF_2_ADDR, reg);
}

bool DFRobot_BMX160::writeBmm150Reg(uint8_t reg, uint8_t value)
{
    if (!writeBmxReg(BMX160_AUX_IF_4_ADDR, value)) {
        return false;
    }
    delay(BMX160_MAGN_COM_DELAY);

    if (!writeBmxReg(BMX160_AUX_IF_3_ADDR, reg)) {
        return false;
    }
    delay(BMX160_MAGN_COM_DELAY);

    return true;
}

bool DFRobot_BMX160::readBmm150Reg(uint8_t reg, uint8_t *value)
{
    return readBmm150Regs(reg, value, 1);
}

bool DFRobot_BMX160::readBmm150Regs(uint8_t reg, uint8_t *values, uint8_t len)
{
    if (values == nullptr || len == 0 || len > 8) {
        return false;
    }

    if (!setBmi160AuxReadAddr(reg)) {
        return false;
    }
    delay(BMX160_MAGN_COM_DELAY);

    return readReg(BMX160_MAG_DATA_ADDR, values, len);
}

bool DFRobot_BMX160::readBmm150Trim()
{
    uint8_t x1_y1[2] = {0};
    uint8_t z4_x2_y2[4] = {0};
    uint8_t z2_z1_xyz1_z3[8] = {0};
    uint8_t xy2_xy1[2] = {0};

    if (!readBmm150Regs(BMM150_DIG_X1_ADDR, x1_y1, sizeof(x1_y1)) ||
        !readBmm150Regs(BMM150_DIG_Z4_LSB_ADDR, z4_x2_y2, sizeof(z4_x2_y2)) ||
        !readBmm150Regs(BMM150_DIG_Z2_LSB_ADDR, z2_z1_xyz1_z3, sizeof(z2_z1_xyz1_z3)) ||
        !readBmm150Regs(BMM150_DIG_XY2_ADDR, xy2_xy1, sizeof(xy2_xy1))) {
        return false;
    }

    bmm150Trim.x1 = static_cast<int8_t>(x1_y1[0]);
    bmm150Trim.y1 = static_cast<int8_t>(x1_y1[1]);
    bmm150Trim.z4 = static_cast<int16_t>((static_cast<uint16_t>(z4_x2_y2[1]) << 8) | z4_x2_y2[0]);
    bmm150Trim.x2 = static_cast<int8_t>(z4_x2_y2[2]);
    bmm150Trim.y2 = static_cast<int8_t>(z4_x2_y2[3]);
    bmm150Trim.z2 = static_cast<int16_t>((static_cast<uint16_t>(z2_z1_xyz1_z3[1]) << 8) |
                                        z2_z1_xyz1_z3[0]);
    bmm150Trim.z1 = static_cast<uint16_t>((static_cast<uint16_t>(z2_z1_xyz1_z3[3]) << 8) |
                                         z2_z1_xyz1_z3[2]);
    bmm150Trim.xyz1 = static_cast<uint16_t>((static_cast<uint16_t>(z2_z1_xyz1_z3[5] & 0x7F) << 8) |
                                           z2_z1_xyz1_z3[4]);
    bmm150Trim.z3 = static_cast<int16_t>((static_cast<uint16_t>(z2_z1_xyz1_z3[7]) << 8) |
                                        z2_z1_xyz1_z3[6]);
    bmm150Trim.xy2 = static_cast<int8_t>(xy2_xy1[0]);
    bmm150Trim.xy1 = xy2_xy1[1];

    return bmm150Trim.xyz1 != 0 && bmm150Trim.z1 != 0 && bmm150Trim.z2 != 0;
}

float DFRobot_BMX160::compensateBmm150X(int16_t raw_x, uint16_t rhall) const
{
    if (raw_x == BMM150_OVERFLOW_ADCVAL_XY || rhall == 0 || bmm150Trim.xyz1 == 0) {
        return 0.0f;
    }

    const float process0 = static_cast<float>(bmm150Trim.xyz1) * 16384.0f / rhall;
    const float centered = process0 - 16384.0f;
    const float process1 = static_cast<float>(bmm150Trim.xy2) *
                           (centered * centered / 268435456.0f);
    const float process2 = process1 + centered * static_cast<float>(bmm150Trim.xy1) / 16384.0f;
    const float process3 = static_cast<float>(bmm150Trim.x2) + 160.0f;
    const float process4 = raw_x * ((process2 + 256.0f) * process3);

    return ((process4 / 8192.0f) + static_cast<float>(bmm150Trim.x1) * 8.0f) / 16.0f;
}

float DFRobot_BMX160::compensateBmm150Y(int16_t raw_y, uint16_t rhall) const
{
    if (raw_y == BMM150_OVERFLOW_ADCVAL_XY || rhall == 0 || bmm150Trim.xyz1 == 0) {
        return 0.0f;
    }

    const float process0 = static_cast<float>(bmm150Trim.xyz1) * 16384.0f / rhall;
    const float centered = process0 - 16384.0f;
    const float process1 = static_cast<float>(bmm150Trim.xy2) *
                           (centered * centered / 268435456.0f);
    const float process2 = process1 + centered * static_cast<float>(bmm150Trim.xy1) / 16384.0f;
    const float process3 = static_cast<float>(bmm150Trim.y2) + 160.0f;
    const float process4 = raw_y * ((process2 + 256.0f) * process3);

    return ((process4 / 8192.0f) + static_cast<float>(bmm150Trim.y1) * 8.0f) / 16.0f;
}

float DFRobot_BMX160::compensateBmm150Z(int16_t raw_z, uint16_t rhall) const
{
    if (raw_z == BMM150_OVERFLOW_ADCVAL_Z || bmm150Trim.z2 == 0 ||
        bmm150Trim.z1 == 0 || bmm150Trim.xyz1 == 0 || rhall == 0) {
        return 0.0f;
    }

    const float process0 = static_cast<float>(raw_z) - static_cast<float>(bmm150Trim.z4);
    const float process1 = static_cast<float>(rhall) - static_cast<float>(bmm150Trim.xyz1);
    const float process2 = static_cast<float>(bmm150Trim.z3) * process1;
    const float process3 = static_cast<float>(bmm150Trim.z1) * static_cast<float>(rhall) / 32768.0f;
    const float process4 = static_cast<float>(bmm150Trim.z2) + process3;
    const float process5 = process0 * 131072.0f - process2;

    return (process5 / (process4 * 4.0f)) / 16.0f;
}

void DFRobot_BMX160::setGyroRange(eGyroRange_t bits){
    switch (bits){
        case eGyroRange_125DPS:
            gyroRange = BMX160_GYRO_SENSITIVITY_125DPS;
            break;
        case eGyroRange_250DPS:
            gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
            break;
        case eGyroRange_500DPS:
            gyroRange = BMX160_GYRO_SENSITIVITY_500DPS;
            break;
        case eGyroRange_1000DPS:
            gyroRange = BMX160_GYRO_SENSITIVITY_1000DPS;
            break;
        case eGyroRange_2000DPS:
            gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
            break;
        default:
            gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
            break;
    }
    writeBmxReg(BMX160_GYRO_RANGE_ADDR, bits);
}

void DFRobot_BMX160::setAccelRange(eAccelRange_t bits){
    switch (bits){
        case eAccelRange_2G:
            accelRange = BMX160_ACCEL_MG_LSB_2G * EARTH_ACC;
            break;
        case eAccelRange_4G:
            accelRange = BMX160_ACCEL_MG_LSB_4G * EARTH_ACC;
            break;
        case eAccelRange_8G:
            accelRange = BMX160_ACCEL_MG_LSB_8G * EARTH_ACC;
            break;
        case eAccelRange_16G:
            accelRange = BMX160_ACCEL_MG_LSB_16G * EARTH_ACC;
            break;
        default:
            accelRange = BMX160_ACCEL_MG_LSB_2G * EARTH_ACC;
            break;
    }

    writeBmxReg(BMX160_ACCEL_RANGE_ADDR, bits);
}

void DFRobot_BMX160::setMagnODR(uint8_t val){
    writeBmxReg(BMX160_MAGN_CONFIG_ADDR, BMX160_MAGN_ODR_MASK & val);
}

void DFRobot_BMX160::setGyroODR(uint8_t val){
    writeBmxReg(BMX160_GYRO_CONFIG_ADDR, BMX160_GYRO_ODR_MASK & val);
}

void DFRobot_BMX160::setAccelODR(uint8_t val){
    writeBmxReg(BMX160_ACCEL_CONFIG_ADDR, BMX160_ACCEL_ODR_MASK & val);
}

void DFRobot_BMX160::getAllData(sBmx160SensorData_t *magn, sBmx160SensorData_t *gyro, sBmx160SensorData_t *accel){

    uint8_t data[23] = {0};
    int16_t x=0,y=0,z=0;
    // put your main code here, to run repeatedly:
    if (!readReg(BMX160_MAG_DATA_ADDR, data, sizeof(data))) {
        if (magn) *magn = {};
        if (gyro) *gyro = {};
        if (accel) *accel = {};
        return;
    }
    if(magn){
        const int16_t raw_x = static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(data[1])) * 32 |
                                                   ((data[0] & 0xF8) >> 3));
        const int16_t raw_y = static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(data[3])) * 32 |
                                                   ((data[2] & 0xF8) >> 3));
        const int16_t raw_z = static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(data[5])) * 128 |
                                                   ((data[4] & 0xFE) >> 1));
        const uint16_t rhall = static_cast<uint16_t>((static_cast<uint16_t>(data[7]) << 6) |
                                                     ((data[6] & 0xFC) >> 2));

        if (_bmm150Ready) {
            const float bmm_x = compensateBmm150X(raw_x, rhall);
            const float bmm_y = compensateBmm150Y(raw_y, rhall);
            const float bmm_z = compensateBmm150Z(raw_z, rhall);

            if (_standaloneBmi160) {
                /*
                 * The new PCB placement keeps BMM150 +X aligned with BMI160 +X.
                 * Y and Z point in the opposite direction, so rotate the magnetic
                 * vector into the accelerometer/gyroscope coordinate frame.
                 */
                magn->x = bmm_x;
                magn->y = -bmm_y;
                magn->z = -bmm_z;
            } else {
                /* BMX160 internally aligns its BMM150 die with the IMU frame. */
                magn->x = bmm_x;
                magn->y = bmm_y;
                magn->z = bmm_z;
            }
        } else {
            *magn = {};
        }
    }
    if(gyro){
        x = (int16_t) (((uint16_t)data[9] << 8) | data[8]);
        y = (int16_t) (((uint16_t)data[11] << 8) | data[10]);
        z = (int16_t) (((uint16_t)data[13] << 8) | data[12]);
        gyro->x = x * gyroRange;
        gyro->y = y * gyroRange;
        gyro->z = z * gyroRange;
    }
    if(accel){
        x = (int16_t) (((uint16_t)data[15] << 8) | data[14]);
        y = (int16_t) (((uint16_t)data[17] << 8) | data[16]);
        z = (int16_t) (((uint16_t)data[19] << 8) | data[18]);
        accel->x = x * accelRange;
        accel->y = y * accelRange;
        accel->z = z * accelRange;
    }
}

bool DFRobot_BMX160::writeBmxReg(uint8_t reg, uint8_t value)
{
    uint8_t buffer[1] = {value};
    return writeReg(reg, buffer, 1);
}

bool DFRobot_BMX160::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
   _i2c->aquire();

    int ret = i2c_burst_write(_i2c->master, _addr, reg, pBuf, len);
    if (ret) LOG_WRN("I2C write failed: %d", ret);

    _i2c->release();

    return ret == 0;
}

bool DFRobot_BMX160::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    _i2c->aquire();

    int ret = i2c_burst_read(_i2c->master, _addr, reg, pBuf, len);
    if (ret) LOG_WRN("I2C read failed: %d", ret);

    _i2c->release();

    return ret == 0;
}

bool DFRobot_BMX160::scan()
{
   uint8_t chip_id = 0;
   return probeAddress(_addr, &chip_id);
}
