#include "ExG.h"

#include <zephyr/logging/log.h>

#include "SensorManager.h"
LOG_MODULE_REGISTER(ExG, 3);

ExG ExG::sensor;

AD7124* ExG::adc = nullptr;

static struct sensor_msg msg_exg;

// samples per second: 614400/(32*x) where x is the samplesPerSecondVal
const SampleRateSetting<8> ExG::sample_rates = {
    {75, 38, 19, 160, 60, 320, 384, 1},                        // reg_vals (FS values)
    {256, 505, 1010, 120, 320, 60, 50, 19200},                 // sample_rates (nominal)
    {256.0, 505.0, 1010.0, 120.0, 320.0, 60.0, 50.0, 19200.0}  // true_sample_rates
};

bool ExG::init(struct k_msgq* queue) {
    if (!_active) {
        // Power up the sensor power rails
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);

        k_msleep(5);

        _active = true;
    }

    // Get GPIO0 device for software SPI bit-banging
    const struct device* gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0)) {
        LOG_ERR("GPIO0 device not ready");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }

    if (adc == nullptr) {
        adc = new AD7124();
    }

    // Configure for SOFTWARE SPI (bit-banging) in 3-WIRE mode
    // Pins: SCK=P0.6, MOSI=P0.13, MISO=P0.12, CS=hardwired to GND
    adc->setSoftwareSPI(gpio0, 6, 13, 12);

    // Initialize ADC
    if (adc->init() != 0) {
        LOG_ERR("ADC init failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }

    if (adc->reset() != 0) {
        LOG_ERR("ADC reset failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }

    k_msleep(10);

    if (adc->setAdcControl(AD7124::OperatingMode::CONTINUOUS, AD7124::PowerMode::FULL_POWER, true) != 0) {
        LOG_ERR("Failed to set ADC to CONTINUOUS");
        return false;
    }

    if (adc->setConfig(0, AD7124::ReferenceSource::INTERNAL, AD7124::PGA::GAIN_1, true) != 0) {
        LOG_ERR("Failed to configure setup 0");
        return false;
    }

    k_msleep(10);

    // Configure filter for setup 0 (SINC4, 256 SPS by default)
    int samplesPerSecondVal = 75; // Default to 256 SPS
    if (adc->setFilter(0, AD7124::FilterType::SINC4, samplesPerSecondVal, false) != 0) {
        LOG_ERR("Failed to configure filter");
        return false;
    }

    if (adc->setChannel(0, 0, AD7124::AnalogInput::AIN0, AD7124::AnalogInput::AIN1, true) != 0) {
        LOG_ERR("Failed to configure channel 0");
        return false;
    }

    sensor_queue = queue;

    k_work_init(&sensor.sensor_work, update_sensor);
    k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

    return true;
}

void ExG::update_sensor(struct k_work* work) {
    int32_t raw_value = 0;
    if (adc->readRaw(&raw_value) != 0) {
        LOG_ERR("Failed to read raw ADC value");
        return;
    }

    // Read voltage and convert to microvolts (µV)
    // InAmp gain = 50 (as per hardware design)
    const float INAMP_GAIN = 50.0f;
    double voltage_volts = adc->readVolts(0);
    double voltage_microvolts = (voltage_volts / INAMP_GAIN) * 1e6f;

    msg_exg.stream = sensor._ble_stream;

    msg_exg.data.id = ID_EXG;
    msg_exg.data.size = sizeof(double);
    msg_exg.data.time = micros();

    memcpy(msg_exg.data.data, &voltage_microvolts, sizeof(double));

    sample_count++;
    if (sample_count % 100 == 0) {
        LOG_INF("Sample #%u: Raw=0x%06X (%d), V=%.6f, uV=%.2f", 
                sample_count, raw_value & 0xFFFFFF, raw_value, voltage_volts, voltage_microvolts);
    }

    int ret = k_msgq_put(sensor_queue, &msg_exg, K_NO_WAIT);
    if (ret) {
        LOG_WRN("sensor msg queue full");
    }
}

void ExG::sensor_timer_handler(struct k_timer* dummy) {
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

void ExG::start(int sample_rate_idx) {
    if (!_active) return;

    // Update filter configuration for new sample rate
    uint16_t fs_val = sample_rates.reg_vals[sample_rate_idx];
    adc->setFilter(0, AD7124::FilterType::SINC4, fs_val, false);

    // Calculate timer period
    k_timeout_t t = K_USEC(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);

    k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

    _running = true;
    LOG_INF("ExG sensor started at %.1f SPS", sample_rates.true_sample_rates[sample_rate_idx]);
}

void ExG::stop() {
    if (!_active) return;

    _running = false;

    k_timer_stop(&sensor.sensor_timer);

    // Put ADC in standby mode
    if (adc != nullptr) {
        adc->setAdcControl(AD7124::OperatingMode::STANDBY, AD7124::PowerMode::FULL_POWER, true);
    }

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);

    _active = false;
    LOG_INF("ExG sensor stopped");
}
