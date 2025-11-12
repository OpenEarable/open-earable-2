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
        
        k_msleep(5);
        
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
    
    // Setup CS control (CS pin at GPIO0.0 for AD7124)
    static struct spi_cs_control cs_ctrl;
    cs_ctrl.gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 0);
    cs_ctrl.delay = 0;
    
    if (!gpio_is_ready_dt(&cs_ctrl.gpio)) {
        LOG_ERR("CS GPIO not ready");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }
    
    gpio_pin_configure_dt(&cs_ctrl.gpio, GPIO_OUTPUT_INACTIVE);
    
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
    
    // Reset ADC
    if (adc->reset() != 0) {
        LOG_ERR("ADC reset failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        _active = false;
        return false;
    }
    
    // Configure ADC control register (continuous mode, full power, internal reference enabled)
    adc->setAdcControl(AD7124_OpMode_Continuous, AD7124_FullPower, true);
    
    // Configure setup 0 (internal reference, gain 1, bipolar)
    adc->setConfig(0, AD7124_Ref_Internal, AD7124_Gain_1, true);
    
    // Configure filter for setup 0 (SINC4 filter, 256 SPS by default)
    adc->setFilter(0, AD7124_Filter_SINC4, 75, AD7124_PostFilter_NoPost, false);
    
    // Configure channel 0 (AIN1 - AIN0, setup 0, enabled)
    adc->setChannel(0, 0, AD7124_Input_AIN1, AD7124_Input_AIN0, true);
    
    sensor_queue = queue;
    
    k_work_init(&sensor.sensor_work, update_sensor);
    k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);
    
    LOG_INF("ExG sensor initialized");
    return true;
}

void ExG::update_sensor(struct k_work *work) {
    // Wait for conversion ready
    if (adc->waitForConvReady(100) != 0) {
        LOG_WRN("Conversion timeout");
        return;
    }
    
    // Read voltage
    float voltage = adc->readVolts(0);
    
    msg_exg.sd = sensor._sd_logging;
    msg_exg.stream = sensor._ble_stream;
    
    msg_exg.data.id = ID_EXG;
    msg_exg.data.size = sizeof(float);
    msg_exg.data.time = micros();
    
    memcpy(msg_exg.data.data, &voltage, sizeof(float));
    
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
