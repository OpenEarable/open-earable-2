#include "battery_service.h"

#include "../Battery/BQ27220.h"

float get_battery_level() {
	return fuel_gauge.state_of_charge();
}