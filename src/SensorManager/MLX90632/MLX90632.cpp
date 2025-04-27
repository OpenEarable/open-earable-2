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

/*
  TODO:
  check EEPROM write
  check timing - how fast to take reading? Setting the SOC twice may be doubling time
  set emissivity
*/

//Declare global variables for the calibration values
double P_R;
double P_G;
double P_T;
double P_O;
double Ea;
double Eb;
double Fa;
double Fb;
double Ga;
double Gb;
double Ka;
double Ha;
double Hb;

double TOdut = 25.0; //Assume 25C for first iteration
double TO0 = 25.0; //object temp from previous calculation
double TA0 = 25.0; //ambient temp from previous calculation
double sensorTemp; //Internal temp of the MLX sensor

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
  LOG_WRN("Sensor EEPROM version (usually 0x205): 0x%X",thisVersion);

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

  //Load all the static calibration factors
  int16_t tempValue16;
  int32_t tempValue32;
  readRegister32(EE_P_R, (uint32_t&)tempValue32);
  P_R = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_P_G, (uint32_t&)tempValue32);
  P_G = (double)tempValue32 * pow(2, -20);
  readRegister32(EE_P_T, (uint32_t&)tempValue32);
  P_T = (double)tempValue32 * pow(2, -44);
  readRegister32(EE_P_O, (uint32_t&)tempValue32);
  P_O = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_Ea, (uint32_t&)tempValue32);
  Ea = (double)tempValue32 * pow(2, -16);
  readRegister32(EE_Eb, (uint32_t&)tempValue32);
  Eb = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_Fa, (uint32_t&)tempValue32);
  Fa = (double)tempValue32 * pow(2, -46);
  readRegister32(EE_Fb, (uint32_t&)tempValue32);
  Fb = (double)tempValue32 * pow(2, -36);
  readRegister32(EE_Ga, (uint32_t&)tempValue32);
  Ga = (double)tempValue32 * pow(2, -36);
  
  readRegister16(EE_Gb, (uint16_t&)tempValue16);
  Gb = (double)tempValue16 * pow(2, -10);
  readRegister16(EE_Ka, (uint16_t&)tempValue16);
  Ka = (double)tempValue16 * pow(2, -10);
  readRegister16(EE_Ha, (uint16_t&)tempValue16);
  Ha = (double)tempValue16 * pow(2, -14); //Ha!
  readRegister16(EE_Hb, (uint16_t&)tempValue16);
  Hb = (double)tempValue16 * pow(2, -14);

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

  //If the sensor is not in continuous mode then the tell sensor to take reading
  if(getMode() != MODE_CONTINUOUS) setSOC();

  // do not check blocking!
  if (dataAvailable() == false)
  {
      LOG_WRN("Data available timeout");
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0); //Error
  }

  //Check when new_data = 1
  /*uint16_t counter = 0;
  while (dataAvailable() == false)
  {
    k_msleep(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      LOG_WRN("Data available timeout");
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0); //Error
    }
  }*/

  //Write new_data = 0
  clearNewData();

  gatherSensorTemp(returnError);
  if (returnError != SENSOR_SUCCESS)
  {
    LOG_WRN("Sensor temperature not found");
    if(returnError == SENSOR_TIMEOUT_ERROR) LOG_WRN("Timeout");
    return (0.0); //Error
  }

  int16_t lowerRAM = 0;
  int16_t upperRAM = 0;

  //Get RAM_6 and RAM_9
  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t&)sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t&)nineRAM);

  //Read cycle_pos to get measurement pointer
  int cyclePosition = getCyclePosition();

  //If cycle_pos = 1
  //Calculate TA and TO based on RAM_4, RAM_5, RAM_6, RAM_9
  if (cyclePosition == 1)
  {
    readRegister16(RAM_4, (uint16_t&)lowerRAM);
    readRegister16(RAM_5, (uint16_t&)upperRAM);
  }
  //If cycle_pos = 2
  //Calculate TA and TO based on RAM_7, RAM_8, RAM_6, RAM_9
  else if (cyclePosition == 2)
  {
    readRegister16(RAM_7, (uint16_t&)lowerRAM);
    readRegister16(RAM_8, (uint16_t&)upperRAM);
  }
  else
  {
    LOG_WRN("Found a cycle position that was not 1 or 2");
    readRegister16(RAM_4, (uint16_t&)lowerRAM);
    readRegister16(RAM_5, (uint16_t&)upperRAM);
  }

  //Object temp requires 3 iterations
  for (uint8_t i = 0 ; i < 3 ; i++)
  {
    double VRta = nineRAM + Gb * (sixRAM / 12.0);

    double AMB = (sixRAM / 12.0) / VRta * pow(2, 19);

    double sensorTemp = P_O + (AMB - P_R) / P_G + P_T * pow((AMB - P_R), 2);

    float S = (float)(lowerRAM + upperRAM) / 2.0;
    double VRto = nineRAM + Ka * (sixRAM / 12.0);
    double Sto = (S / 12.0) / VRto * (double)pow(2, 19);

    double TAdut = (AMB - Eb) / Ea + 25.0;

    double ambientTempK = TAdut + 273.15;

    double bigFraction = Sto / (1 * Fa * Ha * (1 + Ga * (TOdut - TO0) + Fb * (TAdut - TA0)));

    double objectTemp = bigFraction + pow(ambientTempK, 4);
    objectTemp = pow(objectTemp, 0.25); //Take 4th root
    objectTemp = objectTemp - 273.15 - Hb;

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

  double VRta = nineRAM + Gb * (sixRAM / 12.0);

  double AMB = (sixRAM / 12.0) / VRta * pow(2, 19);

  double sensorTemp = P_O + (AMB - P_R) / P_G + P_T * pow((AMB - P_R), 2);

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

  _i2c->aquire();

  // 32-Bit-Adresse in ein Byte-Array umwandeln (Big-Endian)
  uint8_t addr_buf[2] = {
    (uint8_t)(addr >> 8),
    (uint8_t)(addr & 0xFF),
  };

  uint8_t buffer[2];

  // Adresse senden und Daten lesen
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

  _i2c->aquire();

  uint8_t addr_buf[2] = {
    (uint8_t)((addr >> 8) & 0xFF),
    (uint8_t)(addr & 0xFF),
  };

  uint8_t buffer[4];

  // Adresse senden und Daten lesen
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

  _i2c->aquire();

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
  
  //unlock EEPROM
  writeRegister16(0x3005, 0x554C);

  writeRegister16(addr, 0x00);

  // wait for EEPROM
  while (eepromBusy()) k_msleep(1);

  // unlock EEPROM
  writeRegister16(0x3005, 0x554C);

  writeRegister16(addr, val);

  // wait for EEPROM
  while (eepromBusy()) k_msleep(1);
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