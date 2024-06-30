//#pragma once

#ifndef BATTERY_SERVICE_H
#define BATTERY_SERVICE_H

#include <zephyr/bluetooth/gatt.h>
#include "nrf5340_audio_common.h"

// as defined in GATT Specification Supplement
struct battery_level_status {
	uint8_t flags;
	uint16_t power_state;
	/*uint16_t identifier;
	uint8_t battery_level;
	uint8_t additional_status;*/
} __attribute__((packed));

// Battery Energy Status
struct battery_energy_status {
	uint8_t flags;
	//uint16_t external_power_source;
    __fp16 voltage;
	//__fp16 available_energy;
	__fp16 available_capacity;
	__fp16 charge_rate;
	//__fp16 available_energy_last;
} __attribute__((packed));

#ifdef __cplusplus
extern "C" {
#endif

int init_battery_service();

//float get_battery_level();

int bt_send_battery_level(struct battery_data * data);

#ifdef __cplusplus
}
#endif


#endif