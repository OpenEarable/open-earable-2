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
    
    if (scan() == true){
        softReset();
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x11);
        delay(50);
        /* Set gyro to normal mode */
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x15);
        delay(100);
        /* Set mag to normal mode */
        writeBmxReg(BMX160_COMMAND_REG_ADDR, 0x19);
        delay(10);
        setMagnConf();
        return true;
    } else return false;
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
    if (setBmi160AuxMagnConf()) {
        LOG_INF("BMM150 magnetometer configured through BMI160 auxiliary interface");
        return true;
    }

    LOG_WRN("BMM150 auxiliary probe failed; using BMX160 magnetometer interface fallback");
    setBmx160MagnConf();

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

    if (!setBmi160AuxMode(true, 0x00)) {
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

    if (!writeBmm150Reg(BMM150_REP_XY_ADDR, BMM150_REP_XY_REGULAR) ||
        !writeBmm150Reg(BMM150_REP_Z_ADDR, BMM150_REP_Z_REGULAR) ||
        !writeBmm150Reg(BMM150_OP_MODE_ADDR, BMM150_OP_MODE_FORCED) ||
        !setBmi160AuxReadAddr(BMM150_DATA_X_LSB_ADDR) ||
        !writeBmxReg(BMX160_MAGN_CONFIG_ADDR, BMX160_MAGN_ODR_100HZ) ||
        !setBmi160AuxMode(false, 0x03)) {
        return false;
    }

    delay(50);

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
    if (!setBmi160AuxReadAddr(reg)) {
        return false;
    }
    delay(BMX160_MAGN_COM_DELAY);

    return readReg(BMX160_MAG_DATA_ADDR, value, 1);
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
    readReg(BMX160_MAG_DATA_ADDR, data, 23);
    if(magn){
        x = (int16_t) (((uint16_t)data[1] << 8) | data[0]);
        y = (int16_t) (((uint16_t)data[3] << 8) | data[2]);
        z = (int16_t) (((uint16_t)data[5] << 8) | data[4]);
        magn->x = x * BMX160_MAGN_UT_LSB;
        magn->y = y * BMX160_MAGN_UT_LSB;
        magn->z = z * BMX160_MAGN_UT_LSB;
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
   _i2c->aquire();

   uint8_t dummy = 0;
   int ret = i2c_write(_i2c->master, &dummy, 0, _addr);

    _i2c->release();
    
   return (ret == 0);
}
