#include <zephyr/drivers/i2c.h>

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

class TWIM
{
  public:
    TWIM(const struct device * master);
    virtual void begin();
    virtual void end();

    virtual void setClock(uint32_t speed = I2C_SPEED_FAST);

    void aquire();
    void release();

    const struct device * master = NULL;

  private:
    struct k_mutex mutex;

    bool _active = false;
};

extern TWIM I2C1;
extern TWIM I2C2;
extern TWIM I2C3;

/** Serialize a complete transaction, including state checks around I/O.
 * Zephyr mutexes are recursive, so driver register helpers may use this too.
 */
class TWIMLock {
 public:
    explicit TWIMLock(TWIM &bus) : _bus(bus) { _bus.aquire(); }
    ~TWIMLock() { unlock(); }
    TWIMLock(const TWIMLock &) = delete;
    TWIMLock &operator=(const TWIMLock &) = delete;
    void unlock() { if (_locked) { _locked = false; _bus.release(); } }
 private:
    TWIM &_bus;
    bool _locked = true;
};
