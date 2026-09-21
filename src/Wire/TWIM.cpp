
#include "TWIM.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(twim, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

TWIM::TWIM(const struct device * _device) : master(_device) {}

void TWIM::begin() {
	int ret;

	if (_active) return;
	_active = true;

	k_mutex_init(&mutex);

	if (!device_is_ready(master)) {
		LOG_ERR("Failed to setup Wire.");
		_active = false;
	}
}

void TWIM::end() {
	_active = false;
}

void TWIM::setClock(uint32_t speed) {
	int ret;
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

void TWIM::aquire() {
	k_mutex_lock(&mutex, K_FOREVER);
}

void TWIM::release() {
	k_mutex_unlock(&mutex);
}

TWIM I2C1(DEVICE_DT_GET(DT_NODELABEL(i2c1)));
TWIM I2C2(DEVICE_DT_GET(DT_NODELABEL(i2c2)));
TWIM I2C3(DEVICE_DT_GET(DT_NODELABEL(i2c3)));