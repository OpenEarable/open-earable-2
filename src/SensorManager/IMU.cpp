#include "IMU.h"

#include "SensorManager.h"
#include <SensorScheme.h>

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMX160);

static struct sensor_msg msg_imu;

BMX160Bosch IMU::imu(&I2C3);

IMU IMU::sensor;

const SampleRateSetting<6> IMU::sample_rates = {
    { BMI160_GYRO_ODR_25HZ, BMI160_GYRO_ODR_50HZ, BMI160_GYRO_ODR_100HZ,
	BMI160_GYRO_ODR_200HZ, BMI160_GYRO_ODR_400HZ, BMI160_GYRO_ODR_800HZ },

	{ 25, 50, 100, 200, 400, 800 },

	{ 25.0, 50.0, 100.0, 200.0, 400.0, 800.0 }
};

void IMU::update_sensor(struct k_work *work) {
	const uint64_t processing_started_us = micros();
	const int num_samples = imu.read(sensor.sample_buffer, sensor.MAX_BUFFERED_SAMPLES);
	const uint64_t read_finished_us = micros();

	if (num_samples < 0) {
		LOG_WRN("BMX160 FIFO read failed: %d", num_samples);
		return;
	}

	for (int i = 0; i < num_samples; ++i) {
		msg_imu.sd = sensor._sd_logging;
		msg_imu.stream = sensor._ble_stream;
		msg_imu.data.id = ID_IMU;
		msg_imu.data.size = 9 * sizeof(float);
		msg_imu.data.time = read_finished_us -
			(uint64_t)((num_samples - 1 - i) * sensor.t_sample_us);

		memcpy(msg_imu.data.data, sensor.sample_buffer[i].accel, 3 * sizeof(float));
		memcpy(msg_imu.data.data + 3 * sizeof(float), sensor.sample_buffer[i].gyro, 3 * sizeof(float));
		memcpy(msg_imu.data.data + 6 * sizeof(float), sensor.sample_buffer[i].mag, 3 * sizeof(float));

		const int ret = k_msgq_put(sensor_queue, &msg_imu, K_NO_WAIT);
		if (ret) {
			LOG_WRN("sensor msg queue full");
		}
	}

	if (sensor._benchmark_enabled) {
		sensor._samples_read += num_samples;
		if (num_samples > 0) sensor._batches_read++;
		sensor._processing_time_us += micros() - processing_started_us;
		sensor.logBenchmark(read_finished_us);
	}
}

void IMU::logBenchmark(uint64_t now_us) {
	if (!_benchmark_enabled) return;

	constexpr uint64_t REPORT_INTERVAL_US = 5000000;
	const uint64_t elapsed_us = now_us - _benchmark_started_us;
	if (elapsed_us < REPORT_INTERVAL_US || _samples_read == 0) return;

	const BMX160BusStats stats = imu.getBusStats();
	const uint32_t baseline_reads = _samples_read;
	const uint32_t baseline_bytes = _samples_read * 23U;
	const int32_t access_reduction = baseline_reads == 0 ? 0 :
		100 - (int32_t)(100ULL * stats.read_transactions / baseline_reads);
	const int32_t byte_reduction = baseline_bytes == 0 ? 0 :
		100 - (int32_t)(100ULL * stats.read_bytes / baseline_bytes);
	const uint32_t bus_load_permille = elapsed_us == 0 ? 0 :
		(uint32_t)(1000U * stats.bus_time_us / elapsed_us);
	const uint32_t worker_load_permille = elapsed_us == 0 ? 0 :
		(uint32_t)(1000U * _processing_time_us / elapsed_us);

	LOG_INF("BMX160 FIFO benchmark: samples=%u batches=%u reads=%u bytes=%u "
		"access_reduction=%d%% byte_reduction=%d%% bus_load=%u.%u%% worker_load=%u.%u%%",
		_samples_read, _batches_read, stats.read_transactions, stats.read_bytes,
		access_reduction, byte_reduction,
		bus_load_permille / 10, bus_load_permille % 10,
		worker_load_permille / 10, worker_load_permille % 10);

	imu.resetBusStats();
	_benchmark_started_us = now_us;
	_processing_time_us = 0;
	_samples_read = 0;
	_batches_read = 0;
}

/**
* @brief Submit a k_work on timer expiry.
*/
void IMU::sensor_timer_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
};

bool IMU::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

	if (!imu.init()) {
		LOG_ERR("Could not find a valid BMX160 sensor, check wiring!");
		pm_device_runtime_put(ls_1_8);
    	_active = false;
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void IMU::start(int sample_rate_idx) {
	if (!_active) return;

	t_sample_us = 1e6f / sample_rates.true_sample_rates[sample_rate_idx];
	_num_samples_buffered = MIN(MAX(1, (int)(CONFIG_SENSOR_LATENCY_MS * 1e3f / t_sample_us)),
		(int)MAX_BUFFERED_SAMPLES);

	const int result = imu.start(sample_rates.reg_vals[sample_rate_idx],
		sample_rates.true_sample_rates[sample_rate_idx], _num_samples_buffered);
	if (result != 0) {
		LOG_ERR("BMX160 FIFO start failed: %d", result);
		return;
	}

	const uint32_t poll_period_us = (uint32_t)(_num_samples_buffered * t_sample_us);
	const k_timeout_t t = K_USEC(poll_period_us);

	_running = true;
	_benchmark_started_us = micros();
	_processing_time_us = 0;
	_samples_read = 0;
	_batches_read = 0;

	// Let the FIFO collect the first complete batch before the first burst read.
	k_timer_start(&sensor.sensor_timer, t, t);
	LOG_INF("BMX160 FIFO started: rate=%dHz buffered=%u poll=%uus",
		(int)sample_rates.true_sample_rates[sample_rate_idx], _num_samples_buffered, poll_period_us);
}

void IMU::stop() {
    if (!_active) return;
    _active = false;

	_running = false;

	k_timer_stop(&sensor.sensor_timer);

	imu.stop();

    pm_device_runtime_put(ls_1_8);
}

void IMU::setBenchmarkEnabled(bool enabled)
{
	sensor._benchmark_enabled = enabled;
}

static int cmd_imu_fifo_start(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "usage: imu_fifo start <rate-index 0..5>");
		return -EINVAL;
	}

	char *end = nullptr;
	const unsigned long rate_index = strtoul(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || rate_index >= 6) {
		shell_error(shell, "rate-index must be 0..5 (25, 50, 100, 200, 400, 800 Hz)");
		return -EINVAL;
	}

	struct sensor_config config = { ID_IMU, static_cast<uint8_t>(rate_index), DATA_STREAMING };
	IMU::setBenchmarkEnabled(true);
	config_sensor(&config);
	shell_print(shell, "BMX160 FIFO benchmark requested at %d Hz",
		(int)IMU::sample_rates.true_sample_rates[rate_index]);
	return 0;
}

static int cmd_imu_fifo_stop(const struct shell *shell, size_t, char **)
{
	struct sensor_config config = { ID_IMU, 0, 0 };
	config_sensor(&config);
	IMU::setBenchmarkEnabled(false);
	shell_print(shell, "BMX160 FIFO benchmark stop requested");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(imu_fifo_commands,
	SHELL_CMD_ARG(start, NULL, "Start: imu_fifo start <rate-index 0..5>",
		cmd_imu_fifo_start, 2, 0),
	SHELL_CMD(stop, NULL, "Stop the BMX160 FIFO benchmark", cmd_imu_fifo_stop),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(imu_fifo, &imu_fifo_commands, "BMX160 FIFO benchmark", NULL);
