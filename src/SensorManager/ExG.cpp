#include "ExG.h"
#include "SensorManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ExG, 3);

ExG ExG::sensor;

AD7124 *ExG::adc = nullptr;

static struct sensor_msg msg_exg;

// Sample rate configuration matching OpenEarableEEG
// samples per second: 614400/(32*x) where x is the samplesPerSecondVal
// OpenEarableEEG uses 75 for 256 SPS
const SampleRateSetting<8> ExG::sample_rates = {
    { 75, 38, 19, 160, 60, 320, 384, 1 },           // reg_vals (FS values)
    { 256, 505, 1010, 120, 320, 60, 50, 19200 },    // sample_rates (nominal)
    { 256.0, 505.0, 1010.0, 120.0, 320.0, 60.0, 50.0, 19200.0 }  // true_sample_rates
};

bool ExG::init(struct k_msgq * queue) {
    if (!_active) {
        // Power up the sensor power rails
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);
        
        // Wait for power rails to stabilize - AD7124 needs significant time after power-on
        // Per datasheet: power-on time is typically 1ms, but can be longer
        // Increase to 200ms for reliable startup
        LOG_INF("Waiting for power rails to stabilize...");
        k_msleep(500);
        
        _active = true;
    }
    
    // Get SPI4 device from devicetree
    const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi4));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI4 device not ready");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }
    
    // Setup CS control (CS pin at P0.20 for AD7124)
    static struct spi_cs_control cs_ctrl;
    cs_ctrl.gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 0);
    cs_ctrl.delay = 500;  // delay after CS assertion (AD7124 setup time)
    
    if (!gpio_is_ready_dt(&cs_ctrl.gpio)) {
        LOG_ERR("CS GPIO (P0.20) not ready");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }
    
    gpio_pin_configure_dt(&cs_ctrl.gpio, GPIO_OUTPUT_INACTIVE);
    LOG_INF("Using 4-wire SPI mode with CS on P0.20");
    
    // Create ADC instance
    if (adc == nullptr) {
        adc = new AD7124(spi_dev, &cs_ctrl);
    }
    
    // Initialize ADC
    if (adc->init() != 0) {
        LOG_ERR("ADC init failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }
    
    // Additional delay before reset to ensure device is fully powered
    k_msleep(50);
    
    // Reset ADC
    LOG_INF("Resetting AD7124...");
    if (adc->reset() != 0) {
        LOG_ERR("ADC reset failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }
    
    // Wait for reset to complete and internal oscillator to stabilize
    // Increase delay to ensure device is fully ready
    k_msleep(500);
    
    // // Verify SPI communication by reading ID register
    // uint32_t chip_id;
    // if (adc->readRegister(AD7124_REG_ID, &chip_id, 1) != 0) {
    //     LOG_ERR("Failed to read chip ID - SPI communication error");
    //     pm_device_runtime_put(ls_1_8);
    //     pm_device_runtime_put(ls_3_3);
    //     _active = false;
    //     return false;
    // }
    // LOG_INF("AD7124 Chip ID: 0x%02X (expected 0x14 or 0x16)", chip_id);
    
    // if (chip_id != 0x14 && chip_id != 0x16) {
    //     LOG_ERR("Invalid chip ID! Check SPI connections and CS pin");
    //     pm_device_runtime_put(ls_1_8);
    //     pm_device_runtime_put(ls_3_3);
    //     _active = false;
    //     return false;
    // }
    
    LOG_INF("Configuring AD7124...");

    if (adc->setAdcControl(AD7124_OpMode_Continuous, AD7124_FullPower, true) != 0) {
        LOG_ERR("Failed to set ADC control");
        return false;
    }
    
    // Configure setup 0 FIRST (internal reference, gain 1, bipolar)
    if (adc->setConfig(0, AD7124_Ref_Internal, AD7124_Gain_128, true) != 0) {
        LOG_ERR("Failed to configure setup 0");
        return false;
    }
    LOG_DBG("Setup 0 configured");
    
    // Configure filter for setup 0 (SINC4 filter, 256 SPS by default)
    if (adc->setFilter(0, AD7124_Filter_SINC4, 75, AD7124_PostFilter_NoPost, false) != 0) {
        LOG_ERR("Failed to configure filter");
        return false;
    }
    
    // Configure channel 0 (AIN7 - AIN6, setup 0, enabled)
    //if (adc->setChannel(0, 0, AD7124_Input_AIN1, AD7124_Input_AIN0, true) != 0) {
    if (adc->setChannel(0, 0, AD7124_Input_AIN3, AD7124_Input_AIN4, true) != 0) {
        LOG_ERR("Failed to configure channel 0");
        return false;
    }
    
    sensor_queue = queue;
    
    k_work_init(&sensor.sensor_work, update_sensor);
    k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);
    
    LOG_INF("ExG sensor initialized");
    return true;
}

void ExG::update_sensor(struct k_work *work) {
    static uint32_t sample_count = 0;
    
    // Wait for conversion ready
    if (adc->waitForConvReady(100) != 0) {
        LOG_WRN("Conversion timeout");
        return;
    }
    
    // Read voltage and convert to microvolts (µV)
    // InAmp gain = 50 (as per hardware design)
    const float INAMP_GAIN = 50.0f;
    float voltage_volts = adc->readVolts(0);
    float voltage_microvolts = (voltage_volts / INAMP_GAIN) * 1e6f;
    
    // Log every 100th sample
    sample_count++;
    if (sample_count % 100 == 0) {
        LOG_INF("Sample #%u: Raw=%.6f V, uV=%.2f", sample_count, voltage_volts, voltage_microvolts);
    }
    
    msg_exg.stream = sensor._ble_stream;
    
    msg_exg.data.id = ID_EXG;
    msg_exg.data.size = sizeof(float);
    msg_exg.data.time = micros();
    
    memcpy(msg_exg.data.data, &voltage_microvolts, sizeof(float));
    
    int ret = k_msgq_put(sensor_queue, &msg_exg, K_NO_WAIT);
    if (ret) {
        LOG_WRN("sensor msg queue full");
    }
}

void ExG::sensor_timer_handler(struct k_timer *dummy) {
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

void ExG::start(int sample_rate_idx) {
    if (!_active) return;
    
    // Update filter configuration for new sample rate
    uint16_t fs_val = sample_rates.reg_vals[sample_rate_idx];
    adc->setFilter(0, AD7124_Filter_SINC4, fs_val, AD7124_PostFilter_NoPost, false);
    
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
        adc->setAdcControl(AD7124_OpMode_Standby, AD7124_FullPower, true);
    }
    
    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
    
    _active = false;
    LOG_INF("ExG sensor stopped");
}
