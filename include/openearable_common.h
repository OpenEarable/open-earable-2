#ifndef _OPEN_EARABLE_COMMON_H_
#define _OPEN_EARABLE_COMMON_H_

//#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#define ZBUS_READ_TIMEOUT_MS	K_MSEC(100)
#define ZBUS_ADD_OBS_TIMEOUT_MS K_MSEC(200)

#define SENSOR_DATA_FIXED_LENGTH 38

#define millis() k_cyc_to_ms_floor64(k_uptime_ticks())
#define micros() k_cyc_to_us_floor64(k_uptime_ticks())

#define load_switch_sd_id DT_NODELABEL(load_switch_sd)
#define load_switch_1_8_id DT_NODELABEL(load_switch)
#define load_switch_3_3_id DT_NODELABEL(bq25120a)

extern const struct device *const cons;
extern const struct device *const ls_1_8;
extern const struct device *const ls_3_3;
extern const struct device *const ls_sd;

typedef uint8_t RGBColor[3];

struct boot_state {
	bool timer_reset;
	uint64_t device_id;
};

enum pairing_state {
	SET_PAIRING,
	BONDING,
	PAIRED, //DISONNECTED
	CONNECTED,
};

enum charging_state {
	DISCHARGING,
	BATTERY_CRITICAL,
	BATTERY_LOW,
	POWER_CONNECTED,
	PRECHARGING,
	SLOW_CHARGING,
	CHARGING, // Constant current
	TRICKLE_CHARGING, //Constant voltage
	FULLY_CHARGED,
	FAULT,
};

enum led_mode {
	STATE_INDICATION,
	CUSTOM,
};

enum sd_state {
    SD_IDLE,
    SD_RECORDING,
	SD_FAULT,
};

struct earable_state {
	enum pairing_state pairing_state;
	enum charging_state charging_state;
	enum sd_state sd_state;
	enum led_mode led_mode;
};

enum sensor_id {
	ID_IMU=0,
	ID_TEMP_BARO=1,
	ID_MICRO=2,
	ID_PPG=4,
	ID_PULSOX=5,
	ID_OPTTEMP=6,
	ID_BONE_CONDUCTION=7,
};

struct battery_data {
    uint8_t battery_level;
	enum charging_state charging_state;
    //uint16_t charging_state;
};

struct sensor_data {
    uint8_t id;
    uint8_t size;
    uint64_t time;
    uint8_t data[SENSOR_DATA_FIXED_LENGTH];
} __attribute__((packed));

struct sensor_msg {
	bool sd;
	bool stream;
	struct sensor_data data;
};

struct sensor_config {
    uint8_t sensorId;
    uint8_t sampleRateIndex;
	uint8_t storageOptions;
} __attribute__((packed));


struct battery_settings {
    float u_nominal;
    float u_term;
    float u_vlo;
    float u_charge_prevent;

    float i_term;
    float i_charge;
    float i_max;

    float capacity;

    float temp_min;
    float temp_fast_min;
    float temp_fast_max;
    float temp_max;
};

struct sd_msg {
	bool removed;
};

#include "audio_i2s.h"

struct audio_rx_data {
    char data[FRAME_SIZE_BYTES];
    size_t size;
};

#endif