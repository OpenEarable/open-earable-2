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

#define delay(ms) k_msleep(ms)
#define malloc(a) k_malloc(a)

DFRobot_BMX160::DFRobot_BMX160(TwoWire *pWire)
{
  _pWire = pWire;
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
    _pWire->begin();
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
    }
    else
        return false;
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
  

  dev->prevMagnCfg = dev->magnCfg;
  dev->prevGyroCfg = dev->gyroCfg;
  dev->prevAccelCfg = dev->accelCfg;
}

void DFRobot_BMX160::setMagnConf()
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
            gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
            break;
    }
}

void DFRobot_BMX160::setAccelRange(eAccelRange_t bits){
    switch (bits){
        case eAccelRange_2G:
            accelRange = BMX160_ACCEL_MG_LSB_2G * 10;
            break;
        case eAccelRange_4G:
            accelRange = BMX160_ACCEL_MG_LSB_4G * 10;
            break;
        case eAccelRange_8G:
            accelRange = BMX160_ACCEL_MG_LSB_8G * 10;
            break;
        case eAccelRange_16G:
            accelRange = BMX160_ACCEL_MG_LSB_16G * 10;
            break;
        default:
            accelRange = BMX160_ACCEL_MG_LSB_2G * 10;
            break;
    }

    writeBmxReg(BMX160_ACCEL_RANGE_ADDR, bits);
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

void DFRobot_BMX160::writeBmxReg(uint8_t reg, uint8_t value)
{
    uint8_t buffer[1] = {value};
    writeReg(reg, buffer, 1);
}

void DFRobot_BMX160::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
   _pWire->aquire();
   _pWire->beginTransmission(_addr);
   _pWire->write(reg);
    for(uint16_t i = 0; i < len; i ++)
       _pWire->write(pBuf[i]);
   _pWire->endTransmission();
   _pWire->release();
}

void DFRobot_BMX160::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
   _pWire->aquire();
   _pWire->beginTransmission(_addr);
   _pWire->write(reg);
    if(_pWire->endTransmission() != 0) {
        _pWire->release();
        return;
    }
   _pWire->requestFrom(_addr, (uint8_t) len);
    for(uint16_t i = 0; i < len; i ++) {
        pBuf[i] =_pWire->read();
    }
   _pWire->endTransmission();
   _pWire->release();
}

bool DFRobot_BMX160::scan()
{
   _pWire->aquire();
   _pWire->beginTransmission(_addr);
   int ret = _pWire->endTransmission();
   _pWire->release();
   return (ret == 0);
}

uint64_t DFRobot_BMX160::read_time() {
    uint8_t sensor_time_raw[3];
    uint8_t msb_before, msb_after;

    do {
        readReg(BMX160_SENSOR_TIME_ADDR_2, &msb_before, 1);

        readReg(BMX160_SENSOR_TIME_ADDR_0, sensor_time_raw, 3);

        readReg(BMX160_SENSOR_TIME_ADDR_2, &msb_after, 1);

        // If MSB changed, re-read (avoids overflow issues)
    } while (msb_before != msb_after);

    // Combine into a 24-bit value
    uint64_t sensor_time = ((uint64_t)sensor_time_raw[2] << 16) |
                           ((uint64_t)sensor_time_raw[1] << 8)  |
                           (uint64_t)sensor_time_raw[0];

    // Convert to microseconds
    return sensor_time * 39.0625; // datasheet say 39 uS but chatgpt suggests 39.0625 in most implementations
}

