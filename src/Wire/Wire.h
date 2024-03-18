#include <zephyr/drivers/i2c.h>

#include "RingBuffer.h"

/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#define I2C_DEV_LABEL "I2C_1"

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrParamInt)(int);
typedef int PinName;

namespace arduino {

class MbedI2C //: public HardwareI2C
{
  public:
    MbedI2C(); //int sda, int scl
    //MbedI2C(PinName sda, PinName scl);
    virtual void begin();
    #ifndef DEVICE_I2CSLAVE
    virtual void __attribute__ ((error("I2C Slave mode is not supported"))) begin(uint8_t address);
    #else
    virtual void begin(uint8_t address);
    #endif
    virtual void end();

    virtual void setClock(uint32_t freq);
  
    virtual void beginTransmission(uint8_t address);
    virtual uint8_t endTransmission(bool stopBit);
    virtual uint8_t endTransmission(void);

    virtual size_t requestFrom(uint8_t address, size_t len, bool stopBit);
    virtual size_t requestFrom(uint8_t address, size_t len);

    virtual void onReceive(void(*)(int));
    virtual void onRequest(void(*)(void));

    virtual size_t write(uint8_t data);
    virtual size_t write(int data) {
      return write ((uint8_t)data);
    };
    virtual size_t write(const uint8_t* data, int len);
    //using Print::write;
    virtual int read();
    virtual int peek();
    virtual void flush();
    virtual int available();

private:

#ifdef DEVICE_I2CSLAVE
    mbed::I2CSlave* slave = NULL;
#endif
    const struct device * master = NULL;

    int master_read(int address, const char * buf, const uint8_t len, bool no_stop);
    int master_write(int address, const char * buf, const uint8_t len, bool no_stop);
    int i2c_message(uint8_t read_write, int address, const char * buf, const uint8_t len, bool no_stop);
    
    //PinName _sda;
    //PinName _scl;
    int _address;
    RingBufferN<256> rxBuffer;
    uint8_t txBuffer[256];
    uint32_t usedTxBuffer;
    voidFuncPtrParamInt onReceiveCb = NULL;
    voidFuncPtr onRequestCb = NULL;
#ifdef DEVICE_I2CSLAVE
    rtos::Thread* slave_th;
    void receiveThd();
#endif
};

}

extern arduino::MbedI2C Wire;

typedef arduino::MbedI2C TwoWire;