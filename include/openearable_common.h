#ifndef _OPEN_EARABLE_COMMON_H_
#define _OPEN_EARABLE_COMMON_H_

//#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "LoadSwitch.h"

#define ZBUS_READ_TIMEOUT_MS	K_MSEC(100)
#define ZBUS_ADD_OBS_TIMEOUT_MS K_MSEC(200)

#define SENSOR_DATA_FIXED_LENGTH 9

#define millis() k_cyc_to_ms_floor32(k_cycle_get_32())
#define micros() k_cyc_to_us_floor64(k_cycle_get_32())

#define load_switch_sd_id DT_NODELABEL(load_switch_sd)
#define load_switch_1_8_id DT_NODELABEL(load_switch)
#define load_switch_3_3_id DT_NODELABEL(bq25120a)

extern const struct device *const cons;
extern const struct device *const ls_1_8;
extern const struct device *const ls_3_3;
extern const struct device *const ls_sd;

struct boot_state {
	bool timer_reset;
	uint64_t device_id;
};

enum pairing_state {
	SET_PAIRING, //BINDING,
	BONDING,
	PAIRED, //DISONNECTED
	CONNECTED,
};

enum charging_state {
	DISCHARGING,
	CHARGING,
	FULLY_CHARGED,
	FAULT,
};

enum led_state {
	STATE_INDICATION,
	CUSTOM,
};

struct earable_state {
	enum pairing_state pairing_state;
	enum charging_state charging_state;
	enum led_state led_state;
};	 

enum sensor_id {
	ID_IMU=0,
	ID_TEMP_BARO=1,
	ID_PPG=4,
	ID_PULSOX=5,
	ID_OPTTEMP=6,
	ID_BONE_CONDUCTION=7,
};

struct battery_data {
    uint8_t battery_level;
    uint16_t charging_state;
};

struct sensor_data {
    uint8_t id;
    uint8_t size;
    uint32_t time;
    float data[SENSOR_DATA_FIXED_LENGTH];
    //uint8_t * data;
} __attribute__((packed));

struct sensor_config {
    uint8_t sensorId;
    float sampleRate;
    uint32_t latency;
} __attribute__((packed));

#endif