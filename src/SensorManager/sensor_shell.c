/*
 * Shell commands for sensor management and stress testing.
 * Provides the same functionality as the phone app's BLE sensor config,
 * accessible via RTT or CDC ACM console.
 *
 * Usage:
 *   sensor start <sensor> <rate_idx> [sd|stream|all]
 *   sensor stop <sensor|all>
 *   sensor info
 *   sensor stress <sd|stream|all> [duration_s]
 *   sensor nominal <sd|stream|all> [duration_s]
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>

#include "openearable_common.h"
#include "SensorManager.h"
#include "SensorScheme.h"

struct sensor_name_map {
	const char *name;
	enum sensor_id id;
};

static const struct sensor_name_map sensor_names[] = {
	{ "imu",    ID_IMU },
	{ "baro",   ID_TEMP_BARO },
	{ "micro",  ID_MICRO },
	{ "ppg",    ID_PPG },
	{ "temp",   ID_OPTTEMP },
	{ "bone",   ID_BONE_CONDUCTION },
};

#define SENSOR_COUNT ARRAY_SIZE(sensor_names)

static int lookup_sensor(const char *name, enum sensor_id *id)
{
	for (size_t i = 0; i < SENSOR_COUNT; i++) {
		if (strcmp(name, sensor_names[i].name) == 0) {
			*id = sensor_names[i].id;
			return 0;
		}
	}
	return -EINVAL;
}

static const char *sensor_id_to_name(enum sensor_id id)
{
	for (size_t i = 0; i < SENSOR_COUNT; i++) {
		if (sensor_names[i].id == id) {
			return sensor_names[i].name;
		}
	}
	return "unknown";
}

static uint8_t parse_storage_opts(const char *opt)
{
	if (opt == NULL || strcmp(opt, "sd") == 0) {
		return DATA_STORAGE;
	} else if (strcmp(opt, "stream") == 0) {
		return DATA_STREAMING;
	} else if (strcmp(opt, "all") == 0) {
		return DATA_STORAGE | DATA_STREAMING;
	}
	return 0;
}

static void configure_one_sensor(enum sensor_id id, uint8_t rate_idx, uint8_t storage)
{
	struct sensor_config cfg = {
		.sensorId = (uint8_t)id,
		.sampleRateIndex = rate_idx,
		.storageOptions = storage,
	};
	config_sensor(&cfg);
}

/* sensor start <sensor> <rate_idx> [sd|stream|all] */
static int cmd_sensor_start(const struct shell *sh, size_t argc, char **argv)
{
	enum sensor_id id;

	if (argc < 3) {
		shell_error(sh, "Usage: sensor start <sensor> <rate_idx> [sd|stream|all]");
		shell_print(sh, "Sensors: imu, baro, micro, ppg, temp, bone");
		return -EINVAL;
	}

	if (lookup_sensor(argv[1], &id) != 0) {
		shell_error(sh, "Unknown sensor '%s'", argv[1]);
		shell_print(sh, "Valid: imu, baro, micro, ppg, temp, bone");
		return -EINVAL;
	}

	uint8_t rate_idx = (uint8_t)atoi(argv[2]);
	uint8_t storage = parse_storage_opts(argc > 3 ? argv[3] : NULL);

	if (storage == 0) {
		shell_error(sh, "Invalid storage option '%s'. Use: sd, stream, all", argv[3]);
		return -EINVAL;
	}

	float rate = getSampleRateForSensorId((uint8_t)id, rate_idx);
	shell_print(sh, "Starting %s: rate_idx=%u (%.1f Hz), opts=0x%02x",
		    sensor_id_to_name(id), rate_idx, (double)rate, storage);

	configure_one_sensor(id, rate_idx, storage);
	return 0;
}

/* sensor stop <sensor|all> */
static int cmd_sensor_stop(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: sensor stop <sensor|all>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "all") == 0) {
		stop_sensor_manager();
		shell_print(sh, "All sensors stopped");
		return 0;
	}

	enum sensor_id id;

	if (lookup_sensor(argv[1], &id) != 0) {
		shell_error(sh, "Unknown sensor '%s'", argv[1]);
		return -EINVAL;
	}

	/* storageOptions=0 tells config_work_handler to stop the sensor */
	configure_one_sensor(id, 0, 0);
	shell_print(sh, "Stopped %s", sensor_id_to_name(id));
	return 0;
}

/* sensor info */
static int cmd_sensor_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct ParseInfoScheme *scheme = getParseInfoScheme();

	shell_print(sh, "%-8s  %-6s  %-10s  %s", "Sensor", "ID", "Rates", "Options");
	shell_print(sh, "--------------------------------------------");

	for (uint8_t i = 0; i < scheme->sensorCount; i++) {
		struct SensorScheme *ss = getSensorSchemeForId(scheme->sensorIds[i]);
		if (ss == NULL) {
			continue;
		}

		/* Build rate list string */
		char rates[64] = "";
		if (ss->configOptions.availableOptions & FREQUENCIES_DEFINED) {
			size_t pos = 0;
			for (uint8_t j = 0; j < ss->configOptions.frequencyOptions.frequencyCount && pos < sizeof(rates) - 8; j++) {
				int n = snprintf(rates + pos, sizeof(rates) - pos,
						 "%s%.0f",
						 j > 0 ? "," : "",
						 (double)ss->configOptions.frequencyOptions.frequencies[j]);
				if (n > 0) {
					pos += n;
				}
			}
		}

		shell_print(sh, "%-8s  %-6u  %-10s  freq|sd|stream",
			    ss->name ? ss->name : sensor_id_to_name(ss->id),
			    ss->id, rates);
	}

	return 0;
}

/* sensor stress <sd|stream|all> [duration_s] */
static int cmd_sensor_stress(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: sensor stress <sd|stream|all> [duration_s]");
		return -EINVAL;
	}

	uint8_t storage = parse_storage_opts(argv[1]);
	if (storage == 0) {
		shell_error(sh, "Invalid option '%s'. Use: sd, stream, all", argv[1]);
		return -EINVAL;
	}

	int duration_s = (argc > 2) ? atoi(argv[2]) : 0;

	shell_print(sh, "Starting all sensors at max rate, opts=0x%02x", storage);

	/* Start each sensor at its highest rate index */
	struct ParseInfoScheme *scheme = getParseInfoScheme();

	for (uint8_t i = 0; i < scheme->sensorCount; i++) {
		struct SensorScheme *ss = getSensorSchemeForId(scheme->sensorIds[i]);
		if (ss == NULL) {
			continue;
		}

		uint8_t max_idx = 0;
		if (ss->configOptions.availableOptions & FREQUENCIES_DEFINED) {
			max_idx = ss->configOptions.frequencyOptions.frequencyCount - 1;
		}

		float rate = getSampleRateForSensorId(ss->id, max_idx);
		shell_print(sh, "  %s: rate_idx=%u (%.1f Hz)",
			    ss->name ? ss->name : sensor_id_to_name(ss->id),
			    max_idx, (double)rate);

		configure_one_sensor(ss->id, max_idx, storage);

		/* Small delay between configs to avoid flooding the queue */
		k_msleep(50);
	}

	if (duration_s > 0) {
		shell_print(sh, "Running for %d seconds...", duration_s);
		k_sleep(K_SECONDS(duration_s));
		stop_sensor_manager();
		shell_print(sh, "Stress test complete, all sensors stopped");
	} else {
		shell_print(sh, "Sensors running. Use 'sensor stop all' to stop.");
	}

	return 0;
}

struct nominal_rate {
	enum sensor_id id;
	uint8_t rate_idx;
};

static const struct nominal_rate nominal_rates[] = {
	{ ID_IMU,              2 },  /* 100 Hz */
	{ ID_PPG,              0 },  /* 25 Hz */
	{ ID_OPTTEMP,          3 },  /* 4 Hz */
	{ ID_TEMP_BARO,       14 },  /* 25 Hz */
	{ ID_BONE_CONDUCTION,  9 },  /* 6400 Hz (max) */
	{ ID_MICRO,            0 },  /* 48000 Hz */
};

/* sensor nominal <sd|stream|all> [duration_s] */
static int cmd_sensor_nominal(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: sensor nominal <sd|stream|all> [duration_s]");
		return -EINVAL;
	}

	uint8_t storage = parse_storage_opts(argv[1]);
	if (storage == 0) {
		shell_error(sh, "Invalid option '%s'. Use: sd, stream, all", argv[1]);
		return -EINVAL;
	}

	int duration_s = (argc > 2) ? atoi(argv[2]) : 0;

	shell_print(sh, "Starting all sensors at nominal rates, opts=0x%02x", storage);

	for (size_t i = 0; i < ARRAY_SIZE(nominal_rates); i++) {
		const struct nominal_rate *nr = &nominal_rates[i];
		float rate = getSampleRateForSensorId((uint8_t)nr->id, nr->rate_idx);
		shell_print(sh, "  %s: rate_idx=%u (%.1f Hz)",
			    sensor_id_to_name(nr->id), nr->rate_idx, (double)rate);

		configure_one_sensor(nr->id, nr->rate_idx, storage);
		k_msleep(50);
	}

	if (duration_s > 0) {
		shell_print(sh, "Running for %d seconds...", duration_s);
		k_sleep(K_SECONDS(duration_s));
		stop_sensor_manager();
		shell_print(sh, "Nominal test complete, all sensors stopped");
	} else {
		shell_print(sh, "Sensors running. Use 'sensor stop all' to stop.");
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sensor_cmds,
	SHELL_CMD_ARG(start, NULL,
		"Start a sensor: sensor start <imu|baro|micro|ppg|temp|bone> <rate_idx> [sd|stream|all]",
		cmd_sensor_start, 3, 1),
	SHELL_CMD_ARG(stop, NULL,
		"Stop a sensor: sensor stop <imu|baro|micro|ppg|temp|bone|all>",
		cmd_sensor_stop, 2, 0),
	SHELL_CMD(info, NULL, "Show available sensors and sample rates", cmd_sensor_info),
	SHELL_CMD_ARG(stress, NULL,
		"Stress test: sensor stress <sd|stream|all> [duration_s]",
		cmd_sensor_stress, 2, 1),
	SHELL_CMD_ARG(nominal, NULL,
		"Nominal rates: sensor nominal <sd|stream|all> [duration_s]",
		cmd_sensor_nominal, 2, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensor, &sensor_cmds, "Sensor management commands", NULL);
