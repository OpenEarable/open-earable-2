/*
  This is a library written for the MLX90632 Non-contact thermal sensor
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14569

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The MLX90632 can remotely measure object temperatures within 1 degree C.

  This library handles the initialization of the MLX90632 and the calculations
  to get the temperatures.

  https://github.com/sparkfun/SparkFun_MLX90632_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Calibration constants and iteration state live as float class members in
 * MLX90632.h — see the datasheet's object-temperature formula. Float keeps
 * the math on the Cortex-M33 single-precision FPU instead of emulated
 * double. */

#include "MLX90632.h"

#include <zephyr/kernel.h>
#include <math.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MLX90632, 3);

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
bool MLX90632::begin()
{
  uint8_t deviceAddress = DT_REG_ADDR(DT_NODELABEL(mlx90632));
  //TWI  &wirePort = Wire2;
  MLX90632::status returnError;
  if (begin(deviceAddress, I2C3, returnError) == true)
    return (true);
  return (false);
}

//This begins the communication with the device
//Returns a status error if anything went wrong
bool MLX90632::begin(uint8_t deviceAddress, TWIM &i2c, status &returnError)
{
  returnError = SENSOR_SUCCESS;
  _deviceAddress = deviceAddress; //Get the I2C address from user
  _i2c = &i2c; //Grab which port the user wants us to use

  //We require caller to begin their I2C port, with the speed of their choice
  //external to the library
  _i2c->begin();

  reset();

  setMode(MODE_STEP);

  //Check communication with IC
  uint16_t thisAddress;
  returnError = readRegister16(EE_I2C_ADDRESS, thisAddress);
  if (thisAddress != _deviceAddress >> 1)
  {
    LOG_WRN("Error: Communication failure. Check wiring. Expected device address: 0x%X instead read: 0x%X",_deviceAddress, thisAddress << 1);
    returnError = SENSOR_ID_ERROR;
    return (false); //Error
  }

  uint16_t thisVersion;
  returnError = readRegister16(EE_VERSION, thisVersion);
  LOG_DBG("Sensor EEPROM version (usually 0x205): 0x%X",thisVersion);

  //Wait for eeprom_busy to clear
  uint16_t counter = 0;
  while (eepromBusy())
  {
    k_msleep(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      LOG_WRN("EEPROM busy timeout");
      returnError = SENSOR_TIMEOUT_ERROR;
      return (false); //Error
    }
  }

  setMode(MODE_SLEEP); //Before reading EEPROM sensor needs to stop taking readings

  //Load all the static calibration factors. ldexpf(x, n) is x * 2^n — exact
  //for these power-of-two scalings and avoids the pow() library call.
  int16_t tempValue16;
  int32_t tempValue32;
  readRegister32(EE_P_R, (uint32_t&)tempValue32);
  P_R = ldexpf((float)tempValue32, -8);
  readRegister32(EE_P_G, (uint32_t&)tempValue32);
  P_G = ldexpf((float)tempValue32, -20);
  readRegister32(EE_P_T, (uint32_t&)tempValue32);
  P_T = ldexpf((float)tempValue32, -44);
  readRegister32(EE_P_O, (uint32_t&)tempValue32);
  P_O = ldexpf((float)tempValue32, -8);
  readRegister32(EE_Ea, (uint32_t&)tempValue32);
  Ea = ldexpf((float)tempValue32, -16);
  readRegister32(EE_Eb, (uint32_t&)tempValue32);
  Eb = ldexpf((float)tempValue32, -8);
  readRegister32(EE_Fa, (uint32_t&)tempValue32);
  Fa = ldexpf((float)tempValue32, -46);
  readRegister32(EE_Fb, (uint32_t&)tempValue32);
  Fb = ldexpf((float)tempValue32, -36);
  readRegister32(EE_Ga, (uint32_t&)tempValue32);
  Ga = ldexpf((float)tempValue32, -36);

  readRegister16(EE_Gb, (uint16_t&)tempValue16);
  Gb = ldexpf((float)tempValue16, -10);
  readRegister16(EE_Ka, (uint16_t&)tempValue16);
  Ka = ldexpf((float)tempValue16, -10);
  readRegister16(EE_Ha, (uint16_t&)tempValue16);
  Ha = ldexpf((float)tempValue16, -14); //Ha!
  readRegister16(EE_Hb, (uint16_t&)tempValue16);
  Hb = ldexpf((float)tempValue16, -14);

  LOG_DBG("MLX90632 online");

  //Note, sensor is in sleep mode

  return (true);
}

//Read all calibration values and calculate the temperature of the thing we are looking at
//Depending on mode, initiates a measurement
//If in sleep or step mode, clears the new_data bit, sets the SOC bit
float MLX90632::getObjectTemp()
{
  MLX90632::status returnError;
  return (getObjectTemp(returnError));
}
float MLX90632::getObjectTemp(status& returnError)
{
  returnError = SENSOR_SUCCESS;

  if (getMode() != MODE_CONTINUOUS) setSOC();

  // Single status read for data availability and cycle position
  uint16_t statusReg = getStatus();

  if (!(statusReg & (1 << BIT_NEW_DATA)))
  {
      LOG_WRN("Data available timeout");
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0);
  }

  // Clear new_data flag
  writeRegister16(REG_STATUS, statusReg & ~(1 << BIT_NEW_DATA));

  gatherSensorTemp(returnError);
  if (returnError != SENSOR_SUCCESS)
  {
    LOG_WRN("Sensor temperature not found");
    return (0.0);
  }

  int16_t lowerRAM = 0;
  int16_t upperRAM = 0;

  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t&)sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t&)nineRAM);

  int cyclePosition = (statusReg >> BIT_CYCLE_POS) & 0x1F;

  if (cyclePosition == 1)
  {
    readRegister16(RAM_4, (uint16_t&)lowerRAM);
    readRegister16(RAM_5, (uint16_t&)upperRAM);
  }
  else if (cyclePosition == 2)
  {
    readRegister16(RAM_7, (uint16_t&)lowerRAM);
    readRegister16(RAM_8, (uint16_t&)upperRAM);
  }
  else
  {
    LOG_DBG("Found a cycle position that was not 1 or 2");
    readRegister16(RAM_4, (uint16_t&)lowerRAM);
    readRegister16(RAM_5, (uint16_t&)upperRAM);
  }

  // Iterative object temperature computation (3 iterations for convergence)
  for (uint8_t i = 0 ; i < 3 ; i++)
  {
    float VRta = nineRAM + Gb * (sixRAM / 12.0f);
    float AMB = (sixRAM / 12.0f) / VRta * 524288.0f;
    float S = (lowerRAM + upperRAM) / 2.0f;
    float VRto = nineRAM + Ka * (sixRAM / 12.0f);
    float Sto = (S / 12.0f) / VRto * 524288.0f;
    float TAdut = (AMB - Eb) / Ea + 25.0f;
    float ambientTempK = TAdut + 273.15f;

    float bigFraction = Sto / (Fa * Ha * (1.0f + Ga * (TOdut - TO0) + Fb * (TAdut - TA0)));

    float objectTemp = bigFraction + ambientTempK * ambientTempK * ambientTempK * ambientTempK;
    objectTemp = sqrtf(sqrtf(objectTemp));
    objectTemp = objectTemp - 273.15f - Hb;

    TO0 = objectTemp;
  }

  return (TO0);
}

//Convert temp to F
float MLX90632::getObjectTempF()
{
  float tempC = getObjectTemp();
  float tempF = tempC * 9.0/5.0 + 32.0;
  return(tempF);
}

//Returns the current temperature of the sensor
float MLX90632::getSensorTemp()
{
  MLX90632::status tempFlag;
  return (getSensorTemp(tempFlag));
}

float MLX90632::getSensorTemp(status &returnError)
{
  returnError = SENSOR_SUCCESS;

  //If the sensor is not in continuous mode then the tell sensor to take reading
  if(getMode() != MODE_CONTINUOUS) setSOC();

  //Write new_data = 0
  clearNewData();

  //Wait for new data
  uint16_t counter = 0;
  while (dataAvailable() == false)
  {
    k_msleep(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0); //Error
    }
  }

  return (gatherSensorTemp(returnError));
}

//This reads all the temperature calibration factors for the sensor itself
//This is needed so that it can be called from getObjectTemp *and* users can call getSensorTemp 
//without causing a double read
float MLX90632::gatherSensorTemp(status &returnError)
{
  returnError = SENSOR_SUCCESS;

  //Get RAM_6 and RAM_9
  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t&)sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t&)nineRAM);

  float VRta = nineRAM + Gb * (sixRAM / 12.0f);

  float AMB = (sixRAM / 12.0f) / VRta * 524288.0f; // 2^19

  float dAMB = AMB - P_R;
  float sensorTemp = P_O + dAMB / P_G + P_T * dAMB * dAMB;

  return(sensorTemp);
}

//Returns true if device is busy doing measurement
//Always true in continuous mode
bool MLX90632::deviceBusy()
{
  if (getStatus() & ((uint16_t)1 << BIT_DEVICE_BUSY)) return (true);
  return (false);
}

//Returns true if eeprom is busy
//EEPROM is busy during boot up and during EEPROM write/erase
bool MLX90632::eepromBusy()
{
  if (getStatus() & ((uint16_t)1 << BIT_EEPROM_BUSY)) return (true);
  return (false);
}

//Returns the cycle_pos from status register. cycle_pos is 0 to 31
uint8_t MLX90632::getCyclePosition()
{
  uint16_t status = getStatus() >> BIT_CYCLE_POS; //Shave off last two bits
  status &= 0x1F; //Get lower 5 bits.
  return (status);
}

//Returns true if new data is available
bool MLX90632::dataAvailable()
{
  if (getStatus() & ((uint16_t)1 << BIT_NEW_DATA)) return (true);
  return (false);
}

//Sets the brown_out bit. Datasheet says 'Customer should set bit to 1'. Ok.
void MLX90632::setBrownOut()
{
  uint16_t reg = getStatus(); //Get current bits
  reg |= (1 << BIT_BROWN_OUT); //Set the bit
  writeRegister16(REG_STATUS, reg); //Set the mode bits
}

//Clear the new_data bit. This is done after a measurement is complete
void MLX90632::clearNewData()
{
  uint16_t reg = getStatus(); //Get current bits
  reg &= ~(1 << BIT_NEW_DATA); //Clear the bit
  writeRegister16(REG_STATUS, reg); //Set the mode bits
}

//Returns the REG_STATUS 16-bit register
uint16_t MLX90632::getStatus()
{
  MLX90632::status returnError;
  return (getStatus(returnError));
}
uint16_t MLX90632::getStatus(status& returnError)
{
  uint16_t deviceStatus;
  returnError = readRegister16(REG_STATUS, deviceStatus);
  return (deviceStatus);
}

//Changes the mode to sleep
void MLX90632::sleepMode()
{
  setMode(MODE_SLEEP);
}

//Changes the mode to step
void MLX90632::stepMode()
{
  setMode(MODE_STEP);
}

//Changes the mode to continuous read
void MLX90632::continuousMode()
{
  setMode(MODE_CONTINUOUS);
}

//Sets the Start of Conversion (SOC) bit
MLX90632::status MLX90632::setSOC()
{
  uint16_t reg;
  MLX90632::status returnError = readRegister16(REG_CONTROL, reg); //Get current bits
  reg |= (1 << 3); //Set the bit
  writeRegister16(REG_CONTROL, reg); //Set the bit
  return (returnError);
}

//Sets the sensing mode (3 modes availabe)
MLX90632::status MLX90632::setMode(uint8_t mode)
{
  uint16_t reg;
  MLX90632::status returnError = readRegister16(REG_CONTROL, reg); //Get current bits
  reg &= ~(0x0003 << 1); //Clear the mode bits
  reg |= (mode << 1); //Set the bits
  writeRegister16(REG_CONTROL, reg); //Set the mode bits
  return (returnError);
}

//Returns the mode of the sensor
uint8_t MLX90632::getMode()
{
  MLX90632::status returnError;
  return (getMode(returnError));
}
uint8_t MLX90632::getMode(status &returnError)
{
  uint16_t mode;
  returnError = readRegister16(REG_CONTROL, mode); //Get current register settings
  mode = (mode >> 1) & 0x0003; //Clear all other bits
  return (mode);
}

//Writes a new device I2C address into EEPROM
//This is probably a function to use carefully. If you set the address
//to something unknown you'll need to use the I2C scanner sketch to detect it again
void MLX90632::writeI2CAddress(uint8_t newAddress)
{

}

//Reads two consecutive bytes from a given location
//Stores the result at the provided outputPointer
MLX90632::status MLX90632::readRegister16(uint16_t addr, uint16_t &outputPointer)
{
  MLX90632::status returnError = SENSOR_SUCCESS; //By default, return success

  _i2c->acquire();

  // 32-Bit-Adresse in ein Byte-Array umwandeln (Big-Endian)
  uint8_t addr_buf[2] = {
    (uint8_t)(addr >> 8),
    (uint8_t)(addr & 0xFF),
  };

  uint8_t buffer[2];

  // Send address and read data
  int ret = i2c_write_read(_i2c->master, _deviceAddress, addr_buf, sizeof(addr_buf), buffer, sizeof(buffer));
  if (ret) {
      returnError = SENSOR_I2C_ERROR;
      LOG_WRN("I2C read failed: %d\n", ret);
  }

  outputPointer = (uint16_t)buffer[0] << 8 | buffer[1];

  _i2c->release();

  return (returnError); //Report whether we were successful or not
}

//Reads two 16-bit registers and combines them into 32 bits
MLX90632::status MLX90632::readRegister32(uint16_t addr, uint32_t &outputPointer)
{
  MLX90632::status returnError = SENSOR_SUCCESS; //By default, return success

  _i2c->acquire();

  uint8_t addr_buf[2] = {
    (uint8_t)((addr >> 8) & 0xFF),
    (uint8_t)(addr & 0xFF),
  };

  uint8_t buffer[4];

  // Send address and read data
  int ret = i2c_write_read(_i2c->master, _deviceAddress, addr_buf, sizeof(addr_buf), buffer, sizeof(buffer));

  uint16_t lower = (uint16_t)buffer[0] << 8 | buffer[1];
  uint16_t upper = (uint16_t)buffer[2] << 8 | buffer[3];

  outputPointer = (uint32_t)upper << 16 | lower;

  if (ret) {
    returnError = SENSOR_I2C_ERROR;
    LOG_WRN("I2C read failed: %d", ret);
  }

  _i2c->release();
 
  return (returnError); //Report whether we were successful or not
}

//Write two bytes to a spot
MLX90632::status MLX90632::writeRegister16(uint16_t addr, uint16_t val)
{
  MLX90632::status returnError = SENSOR_SUCCESS; //By default, return success

  _i2c->acquire();

  struct i2c_msg msg[2];

  uint8_t addr_buf[2] = {
    (uint8_t)(addr >> 8),
    (uint8_t)(addr & 0xFF),
  };

  uint8_t buffer[2] = {
    (uint8_t)(val >> 8),
    (uint8_t)(val & 0xFF),
  };

	msg[0].buf = addr_buf;
	msg[0].len = 2U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = buffer;
	msg[1].len = 2U;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	int ret = i2c_transfer(_i2c->master, msg, 2, _deviceAddress);

  if (ret != 0) {
    returnError = SENSOR_I2C_ERROR;
    LOG_WRN("I2C write failed: %d", ret);
  }

  _i2c->release();

  return (returnError); //Report whether we were successful or not
}

//Write a value to EEPROM
//Requires unlocking the EEPROM, writing 0x0000, unlocking again, then writing value
//The datasheet doesn't go a good job of explaining how writing to EEPROM works.
//This should work but doesn't. It seems the IC is very sensitive to I2C traffic while
//the sensor is recording the new EEPROM.
void MLX90632::writeEEPROM(uint16_t addr, uint16_t val) {
  writeRegister16(0x3005, 0x554C);
  writeRegister16(addr, 0x00);

  for (int i = 0; i < MAX_WAIT && eepromBusy(); i++) k_msleep(1);

  writeRegister16(0x3005, 0x554C);
  writeRegister16(addr, val);

  for (int i = 0; i < MAX_WAIT && eepromBusy(); i++) k_msleep(1);
}

void MLX90632::reset() {
  setMode(MODE_STEP);

  writeRegister16(0x3005, 0x0006);

  k_usleep(200);

  //writeRegister16(REG_CONTROL, reg_value);
}

void MLX90632::setSampleRateRegVal(uint8_t val) {
  uint8_t originalMode = getMode();
  setMode(MODE_SLEEP);

  writeEEPROM(EE_MEAS_1, 0x800D | (val << 8));
  writeEEPROM(EE_MEAS_2, 0x801D | (val << 8));

  setMode(originalMode);
}

void MLX90632::setSampleRate(float sample_rate) {
  uint16_t val;

  val = round(log2f(CLAMP(2 * sample_rate,1,128)));

  uint8_t originalMode = getMode();
  setMode(MODE_SLEEP);

  writeEEPROM(EE_MEAS_1, 0x800D | (val << 8));
  writeEEPROM(EE_MEAS_2, 0x801D | (val << 8));

  setMode(originalMode);
}

float MLX90632::getSampleRate() {
  uint16_t val1, val2;
  float result;

  readRegister16(EE_MEAS_1, val1);
  readRegister16(EE_MEAS_2, val2);

  if ((val1 | 0x0010) != val2) LOG_WRN("Unequal values for EE_MEAS_1 and EE_MEAS_2.");

  val1 = (val1 >> 8) & 0xF;

  result = (1 << val1) / 2;

  return result;
}