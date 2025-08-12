#include "ANCDamping.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(anc_damping, CONFIG_LOG_DEFAULT_LEVEL);

ANCDamping ANCDamping::sensor;
struct k_msgq * ANCDamping::sensor_queue = NULL;

static struct sensor_msg anc_damping_msg;

const SampleRateSetting<1> ANCDamping::sample_rates = {
    { 0 },
    { 1000 },  // 1 kHz dummy rate (real rate is determined by audio processing)
    { 1000.0 }
};

bool ANCDamping::init(struct k_msgq * queue) {
    _active = true;
    sensor_queue = queue;
    
    LOG_DBG("ANC Damping sensor initialized");
    return true;
}

void ANCDamping::start(int sample_rate_idx) {
    ARG_UNUSED(sample_rate_idx);
    
    if (!_active) return;
    
    _running = true;
    LOG_DBG("ANC Damping sensor started");
}

void ANCDamping::stop() {
    if (!_active) return;
    
    _active = false;
    _running = false;
    
    LOG_DBG("ANC Damping sensor stopped");
}

void ANCDamping::send_damping_data(float damping_value) {
    if (sensor_queue == NULL || !sensor.is_running()) {
        return;
    }
    
    anc_damping_msg.sd = sensor._sd_logging;
    anc_damping_msg.stream = sensor._ble_stream;
    anc_damping_msg.data.id = ID_ANC_DAMPING;
    anc_damping_msg.data.size = sizeof(float);
    anc_damping_msg.data.time = micros();
    memcpy(anc_damping_msg.data.data, &damping_value, sizeof(float));

    int ret = k_msgq_put(sensor_queue, &anc_damping_msg, K_NO_WAIT);
    if (ret) {
        LOG_WRN("ANC damping sensor msg queue full");
    }
}

// C wrapper function for use in C code
extern "C" void anc_damping_send_data(float damping) {
    ANCDamping::send_damping_data(damping);
}
