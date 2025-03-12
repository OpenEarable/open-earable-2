/*!
 * @file Adafruit_BMP3XX.h
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"

//#include "../Adafruit_BusIO/Adafruit_I2CDevice.h"
//#include "Adafruit_SPIDevice.h"

//#include "Wire.h"
#include <TWIM.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP3XX_DEFAULT_ADDRESS (0x76) ///< The default I2C address
/*=========================================================================*/
//#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

/** Adafruit_BMP3XX Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */

struct BMP3XX_dev_inf {
    uint8_t addr;
    TWIM * i2c_dev;
};

class Adafruit_BMP3XX {
public:
  Adafruit_BMP3XX();

  bool begin_I2C(uint8_t addr = DT_REG_ADDR(DT_NODELABEL(bmp388)),
                 TWIM *_i2c = &I2C3);
  //bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI);
  //bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
  //               int8_t mosi_pin);
  uint8_t chipID(void);
  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel);

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setIIRFilterCoeff(uint8_t fs);
  bool setOutputDataRate(uint8_t odr);

  /// Perform a reading in blocking mode
  bool performReading(void);

  /// Temperature (Celsius) assigned after calling performReading()
  double temperature;
  /// Pressure (Pascals) assigned after calling performReading()
  double pressure;

private:
  //TwoWire *i2c_dev = NULL;

  bool _init(void);

  bool _filterEnabled, _tempOSEnabled, _presOSEnabled, _ODREnabled;
  uint8_t _i2caddr;
  int32_t _sensorID;
  int8_t _cs;
  unsigned long _meas_end;

  //uint8_t spixfer(uint8_t x);

  struct bmp3_dev the_sensor;

  bool detect(int address);

  BMP3XX_dev_inf dev_inf;

  //bool readReg(int reg, uint8_t * buffer, int len);
  //void writeReg(const uint8_t reg, const uint8_t *pBuf, uint16_t len);
};

#endif
