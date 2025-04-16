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

#pragma once

//#include <Wire.h>
#include <TWIM.h>

//The default I2C address for the MLX90632 on the SparkX breakout is 0x3B. 0x3A is also possible.
#define MLX90632_DEFAULT_ADDRESS 0x3A 

//Registers
#define EE_VERSION 0x240B

//32 bit constants
#define EE_P_R 0x240C
#define EE_P_G 0x240E
#define EE_P_T 0x2410
#define EE_P_O 0x2412
#define EE_Aa 0x2414
#define EE_Ab 0x2416
#define EE_Ba 0x2418
#define EE_Bb 0x241A
#define EE_Ca 0x241C
#define EE_Cb 0x241E
#define EE_Da 0x2420
#define EE_Db 0x2422
#define EE_Ea 0x2424
#define EE_Eb 0x2426
#define EE_Fa 0x2428
#define EE_Fb 0x242A
#define EE_Ga 0x242C

//16 bit constants
#define EE_Ha 0x2481
#define EE_Hb 0x2482
#define EE_Gb 0x242E
#define EE_Ka 0x242F
#define EE_Kb 0x2430

//Control registers
#define EE_CONTROL 0x24D4
#define EE_I2C_ADDRESS 0x24D5
#define REG_I2C_ADDRESS 0x3000
#define REG_CONTROL 0x3001
#define REG_STATUS 0x3FFF

#define EE_MEAS_1 0x24E1
#define EE_MEAS_2 0x24E2

//User RAM
#define RAM_1 0x4000
#define RAM_2 0x4001
#define RAM_3 0x4002
#define RAM_4 0x4003
#define RAM_5 0x4004
#define RAM_6 0x4005
#define RAM_7 0x4006
#define RAM_8 0x4007
#define RAM_9 0x4008

//Three measurement modes available
#define MODE_SLEEP 0b01
#define MODE_STEP 0b10
#define MODE_CONTINUOUS 0b11

//REG_STATUS bits
#define BIT_DEVICE_BUSY 10
#define BIT_EEPROM_BUSY 9
#define BIT_BROWN_OUT 8
#define BIT_CYCLE_POS 2 //6:2 = 5 bits
#define BIT_NEW_DATA 0

//REG_CONTROL bits
#define BIT_SOC 3
#define BIT_MODE 1 //2:1 = 2 bits

const int MAX_WAIT = 750; //Number of ms to wait before giving up. Some sensor actions take 512ms.

class MLX90632 {
  public:

    // Return values
    typedef enum
    {
      SENSOR_SUCCESS,
      SENSOR_ID_ERROR,
      SENSOR_I2C_ERROR,
      SENSOR_INTERNAL_ERROR,
      SENSOR_GENERIC_ERROR,
      SENSOR_TIMEOUT_ERROR
      //...
    } status;

    bool begin(); //By default .begin() will use the default I2C addres, and use Wire port
    bool begin(uint8_t deviceAddress, TWIM &i2c, status &returnError);

    status readRegister16(uint16_t addr, uint16_t &outputPointer);
    status writeRegister16(uint16_t addr, uint16_t val);

    status readRegister32(uint16_t addr, uint32_t &outputPointer);

    void writeEEPROM(uint16_t addr, uint16_t val);
    void writeI2CAddress(uint8_t newAddress);

    float getObjectTemp();
    float getObjectTemp(status &returnError);
    float getObjectTempF();
    float getSensorTemp();
    float getSensorTemp(status &returnError);

    bool deviceBusy();
    bool eepromBusy();
    uint8_t getCyclePosition();
    bool dataAvailable();
    void setBrownOut();
    void clearNewData();
    uint16_t getStatus();
    uint16_t getStatus(status &returnError);

    void sleepMode();
    void stepMode();
    void continuousMode();
    MLX90632::status setMode(uint8_t mode);
    uint8_t getMode();
    uint8_t getMode(status &returnError);
    MLX90632::status setSOC(); //Set the start conversion bit

    void setSampleRateRegVal(uint8_t val);

    void setSampleRate(float sample_rate);
    float getSampleRate();

    void reset();

  private:

    float gatherSensorTemp(status &returnError);

    //Variables
    TWIM *_i2c = &I2C3; //The generic connection to user's chosen I2C hardware
    uint8_t _deviceAddress = DT_REG_ADDR(DT_NODELABEL(mlx90632)); //Keeps track of I2C address. setI2CAddress changes this. Either 0x3A or 0x3B (default)

};