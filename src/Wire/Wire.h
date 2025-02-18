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

#define BUFFER_TX_SIZE 256
#define BUFFER_RX_SIZE 512

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrParamInt)(int);
typedef int PinName;

namespace arduino {

class MbedI2C //: public HardwareI2C
{
  public:
    MbedI2C(const struct device * master);
    virtual void begin(uint32_t speed = I2C_SPEED_FAST);
    virtual void end();

    virtual void setClock(uint32_t freq);
  
    virtual void beginTransmission(uint8_t address);
    virtual uint8_t endTransmission(bool stopBit = true);

    virtual size_t requestFrom(uint8_t address, size_t len, bool stopBit = true);

    virtual void onReceive(void(*)(int));
    virtual void onRequest(void(*)(void));

    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t* data, int len);
    
    virtual int read();
    virtual int peek();
    virtual void flush();
    virtual int available();

    void aquire();
    void release();

    const struct device * master = NULL;

private:
    struct k_mutex mutex;

    int master_read(int address, const char * buf, const uint32_t len, bool no_stop);
    int master_write(int address, const char * buf, const uint32_t len, bool no_stop);
    int i2c_message(uint8_t read_write, int address, const char * buf, const uint32_t len, bool no_stop);
    
    //PinName _sda;
    //PinName _scl;
    int _address;
    char buf[BUFFER_RX_SIZE];
    RingBufferN<BUFFER_RX_SIZE> rxBuffer;
    uint8_t txBuffer[BUFFER_TX_SIZE];
    uint32_t usedTxBuffer;
    voidFuncPtrParamInt onReceiveCb = NULL;
    voidFuncPtr onRequestCb = NULL;
};

}

extern arduino::MbedI2C Wire;
extern arduino::MbedI2C Wire1;
extern arduino::MbedI2C Wire2;

typedef arduino::MbedI2C TwoWire;