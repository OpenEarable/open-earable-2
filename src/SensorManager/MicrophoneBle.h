#ifndef MICROPHONE_BLE_H
#define MICROPHONE_BLE_H

#include "EdgeMLSensor.h"
#include "openearable_common.h"

class MicrophoneBle : public EdgeMlSensor {
public:
	static MicrophoneBle inner;
	static MicrophoneBle outer;

	bool init(struct k_msgq *queue) override;
	void start(int sample_rate_idx) override;
	void stop() override;

	const static SampleRateSetting<1> sample_rates;

private:
	explicit MicrophoneBle(uint8_t sensor_id);

	uint8_t _sensor_id;
	bool _active = false;
};

#endif
