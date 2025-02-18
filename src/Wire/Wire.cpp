/*
  Wire.cpp - wrapper over mbed I2C / I2CSlave
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2018-2019 Arduino SA

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "Wire.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wire, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

arduino::MbedI2C::MbedI2C(const struct device * _device) : master(_device), usedTxBuffer(0) {}

void arduino::MbedI2C::begin(uint32_t speed) {
	int ret;

	if (!device_is_ready(master)) {
		LOG_ERR("Failed to setup Wire.");
		return;
	}

	// -ERANGE
	uint32_t current_conf;

	ret = i2c_get_config(master, &current_conf);

	if (ret || (current_conf & I2C_SPEED_MASK) != I2C_SPEED_SET(speed)) {
		//LOG_INF("Start Wire");
		ret = i2c_configure(master, I2C_SPEED_SET(speed));
		__ASSERT(ret == 0, "Failed to set I2C speed!");
		k_mutex_init(&mutex);
	}

	if (ret) {
		LOG_WRN("Failed to configure I2C: %i", ret);
	}
}

void arduino::MbedI2C::end() {
	/*if (master != NULL) {
		delete master;
		master = NULL;
	}*/
}

void arduino::MbedI2C::setClock(uint32_t freq) {
	int ret;
	uint32_t current_conf;

	ret = i2c_get_config(master, &current_conf);

	if (ret || (current_conf & I2C_SPEED_MASK) != I2C_SPEED_SET(freq)) {
		//LOG_INF("Start Wire");
		ret = i2c_configure(master, I2C_SPEED_SET(freq));
		__ASSERT(ret == 0, "Failed to set I2C speed!");
		k_mutex_init(&mutex);
	}
	
	if (ret) {
		LOG_WRN("Failed to configure I2C: %i", ret);
	}
}

void arduino::MbedI2C::beginTransmission(uint8_t address) {
	//k_mutex_lock(&mutex, K_FOREVER);
	_address = address;
	usedTxBuffer = 0;
}

uint8_t arduino::MbedI2C::endTransmission(bool stopBit) {
	int ret;

	if (usedTxBuffer == 0) {
		// we are scanning, return 0 if the addresed device responds with an ACK
		char buf[1];
		ret = master_read(_address, buf, 1, !stopBit);
		//k_mutex_unlock(&mutex);
		return ret;
	}
	
	ret = master_write(_address, (const char *) txBuffer, usedTxBuffer, !stopBit);
	//k_mutex_unlock(&mutex);
	
	return ret;
}

size_t arduino::MbedI2C::requestFrom(uint8_t address, size_t len, bool stopBit) {
	//k_mutex_lock(&mutex, K_FOREVER);
	int ret = master_read(address, buf, len, !stopBit);
	if (ret != 0) {
		//k_mutex_unlock(&mutex);
		return 0;
	}
	for (size_t i=0; i<len; i++) {
		rxBuffer.store_char(buf[i]);
	}
	//k_mutex_unlock(&mutex);
	return len;
}

size_t arduino::MbedI2C::write(uint8_t data) {
	if (usedTxBuffer == BUFFER_TX_SIZE) return 0;
	txBuffer[usedTxBuffer++] = data;
	return 1;
}

size_t arduino::MbedI2C::write(const uint8_t* data, int len) {
	if (usedTxBuffer + len > BUFFER_TX_SIZE) len = BUFFER_TX_SIZE - usedTxBuffer;
	memcpy(txBuffer + usedTxBuffer, data, len);
	usedTxBuffer += len;
	return len;
}

int arduino::MbedI2C::read() {
	if (rxBuffer.available()) {
		return rxBuffer.read_char();
	}
	return -1;
}

int arduino::MbedI2C::available() {
	return rxBuffer.available();
}

int arduino::MbedI2C::peek() {
	return rxBuffer.peek();
}

void arduino::MbedI2C::flush() {
}

void arduino::MbedI2C::onReceive(voidFuncPtrParamInt cb) {
	onReceiveCb = cb;
}
void arduino::MbedI2C::onRequest(voidFuncPtr cb) {
	onRequestCb = cb;
}

void arduino::MbedI2C::aquire() {
	k_mutex_lock(&mutex, K_FOREVER);
}

void arduino::MbedI2C::release() {
	k_mutex_unlock(&mutex);
}

int arduino::MbedI2C::i2c_message(uint8_t read_write, int address, const char * buf, const uint32_t len, bool no_stop) {
    uint8_t stop = no_stop ? 0 : I2C_MSG_STOP;

    struct i2c_msg msg;

    msg.buf = (unsigned char *) buf;
    msg.len = len; // length buffer
    msg.flags = read_write | stop;

    return i2c_transfer(master, &msg, 1, address);
}

int arduino::MbedI2C::master_write(int address, const char * buf, const uint32_t len, bool no_stop) {
    return i2c_message(I2C_MSG_WRITE, address, buf, len, no_stop);
}

int arduino::MbedI2C::master_read(int address, const char * buf, const uint32_t len, bool no_stop) {
	return i2c_message(I2C_MSG_READ, address, buf, len, no_stop);
}

arduino::MbedI2C Wire(DEVICE_DT_GET(DT_NODELABEL(i2c1)));
arduino::MbedI2C Wire1(DEVICE_DT_GET(DT_NODELABEL(i2c2)));
arduino::MbedI2C Wire2(DEVICE_DT_GET(DT_NODELABEL(i2c3)));