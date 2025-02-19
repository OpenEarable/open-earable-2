//#pragma once

#ifndef BATTERY_SERVICE_H
#define BATTERY_SERVICE_H

#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"

#include <sfloat.h>

// Battery Level Status
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
	//struct sfloat external_power_source;
    struct sfloat voltage;
	//struct sfloat available_energy;
	struct sfloat available_capacity;
	struct sfloat charge_rate;
	//struct sfloat available_energy_last;
} __attribute__((packed));

// Battery Energy Status
struct battery_health_status {
	uint8_t flags;
	uint8_t battery_health_summary;
    uint16_t cycle_count;
	int8_t current_temperature;
	//uint16_t deep_discharge_count;
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