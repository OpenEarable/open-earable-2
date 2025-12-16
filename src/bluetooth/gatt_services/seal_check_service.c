#include "seal_check_service.h"
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include <data_fifo.h>
#include "arm_math.h"

#include "../../SensorManager/SensorManager.h"
#include "audio_datapath.h"
#include "decimation_filter.h"

#include "multitone.h"

LOG_MODULE_REGISTER(seal_check_service, CONFIG_LOG_DEFAULT_LEVEL);

#define NUM_SEAL_CHECK_SAMPLES 2048
#define INITIAL_SEAL_CHECK_DROP 128

int16_t seal_check_mic[NUM_SEAL_CHECK_SAMPLES];
//int seal_check_mic_index = 0;

static q15_t fft_output[NUM_SEAL_CHECK_SAMPLES * 2]; // Complex output needs double size
static q15_t magnitude[NUM_SEAL_CHECK_SAMPLES / 2]; // Magnitude spectrum

#define num_bins 9
const int bin_tolerance = 2;
static float avg_magnitude = 119.0f;
static float avg_slope = -0.07382279460490486;
static float target_frequencies[] = {40.0, 60.0, 90.0, 135.0, 202.5, 303.75, 455.625, 683.4375, 1025.15625};
static float target_magnitudes[] = {0.90833731, 1.18334124, 1.38796968, 1.16634027, 0.85781358, 0.65981396, 0.84768657, 0.98236069, 1.00633671};

// Service state
static uint8_t seal_check_start_value = 0x00;
static struct seal_check_data seal_check_result_data;
static bool ccc_enabled = false;

// Function prototypes
//extern int audio_datapath_multitone_play(uint16_t dur_ms, float amplitude);
extern int hw_codec_volume_set(uint8_t volume);

extern struct data_fifo fifo_rx;

// Work item for seal check completion
static struct k_work seal_check_complete_work;

void seal_check_callback();

// Callback for start characteristic write
static ssize_t write_seal_check_start(struct bt_conn *conn,
				      const struct bt_gatt_attr *attr,
				      const void *buf, uint16_t len, 
				      uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(seal_check_start_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t*)buf);
	
	if (value == 0xFF) {
		LOG_INF("Seal check started via BLE");
		seal_check_start_value = 0xFF;

		int ret;
		if (!fifo_rx.initialized) {
			ret = data_fifo_init(&fifo_rx);
			if (ret) {
				LOG_ERR("Failed to set up rx FIFO: %d", ret);
				return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
			}
		}

		// Set volume and start multitone
		hw_codec_volume_set(0xB0);

		audio_datapath_decimator_init(12); // 12 = 4kHz
		audio_datapath_aquire(&fifo_rx);
		
		// Start multitone playbook (1.0 amplitude)
		ret = audio_datapath_buffer_play((int16_t*)multitone, multitone_length, false, 1.0f, NULL);

		record_to_buffer(seal_check_mic, NUM_SEAL_CHECK_SAMPLES, INITIAL_SEAL_CHECK_DROP, false, true, seal_check_callback);
		
		if (ret != 0) {
			LOG_ERR("Failed to start seal check: %d", ret);
			seal_check_start_value = 0x00;
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
		
		LOG_INF("Seal check started successfully");
	}
	
	return len;
}

// Callback for start characteristic read
static ssize_t read_seal_check_start(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, 
				&seal_check_start_value, sizeof(seal_check_start_value));
}

// Callback for result characteristic CCC write
static void seal_check_result_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ccc_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Seal check result notifications %s", ccc_enabled ? "enabled" : "disabled");
}

// GATT service definition
BT_GATT_SERVICE_DEFINE(seal_check_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SEAL_CHECK_SERVICE),
	
	// Start Test Characteristic
	BT_GATT_CHARACTERISTIC(BT_UUID_SEAL_CHECK_START,
			      BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			      read_seal_check_start, write_seal_check_start, 
			      &seal_check_start_value),
			      
	// Result Data Characteristic
	BT_GATT_CHARACTERISTIC(BT_UUID_SEAL_CHECK_RESULT,
			      BT_GATT_CHRC_NOTIFY,
			      BT_GATT_PERM_NONE,
			      NULL, NULL, &seal_check_result_data),
	BT_GATT_CCC(seal_check_result_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Work handler for seal check completion
static void seal_check_complete_work_handler(struct k_work *work)
{
	on_seal_check_complete();
}

void on_seal_check_complete() {
	audio_datapath_buffer_stop();

	audio_datapath_release();
	//audio_datapath_decimator_cleanup();

	compute_seal_check_result();
}

void compute_seal_check_result()
{
	// Compute RFFT Q15 for seal check analysis
	static arm_rfft_instance_q15 rfft_instance;
	static bool rfft_initialized = false;
	
	if (!rfft_initialized) {
		arm_status status = arm_rfft_init_q15(&rfft_instance, NUM_SEAL_CHECK_SAMPLES, 0, 1);
		if (status == ARM_MATH_SUCCESS) {
			rfft_initialized = true;
			LOG_INF("RFFT Q15 initialized for %d samples", NUM_SEAL_CHECK_SAMPLES);
		} else {
			LOG_ERR("RFFT Q15 initialization failed with status %d", status);
			return;
		}
	}
	
	// Perform RFFT
	arm_rfft_q15(&rfft_instance, seal_check_mic, fft_output);
	
	// Calculate magnitude spectrum
	arm_cmplx_mag_q15(fft_output, magnitude, NUM_SEAL_CHECK_SAMPLES / 2);
	
	LOG_INF("Seal check RFFT completed, %d frequency bins calculated", NUM_SEAL_CHECK_SAMPLES / 2);
	
	// Calculate mean magnitude of the spectrum
	float spectrum_sum = 0.0f;
	int valid_bins = 0;
	for (int bin = 1; bin < NUM_SEAL_CHECK_SAMPLES / 2; bin++) {
		spectrum_sum += (float)magnitude[bin];
		valid_bins++;
	}
	float mean_magnitude = spectrum_sum / valid_bins;
	float peak_threshold = 4.0f * mean_magnitude;
	
	// Analyze center frequencies with magnitude weighting
	printk("Center frequency analysis (sampling rate: 4000 Hz, mean_mag: %.1f, threshold: %.1f):\n", 
		(double)mean_magnitude, (double)peak_threshold);
	
	// Arrays for linear regression
	float valid_frequencies[num_bins];
	float valid_amplitudes[num_bins];
	int valid_peak_count = 0;
	
	for (int center_idx = 0; center_idx < num_bins; center_idx++) {
		float center_freq = target_frequencies[center_idx];
		int center_bin = (int)(center_freq * NUM_SEAL_CHECK_SAMPLES / 4000.0f + 0.5f);
		
		// Define search range
		int start_bin = MAX(1, center_bin - bin_tolerance);
		int end_bin = MIN(NUM_SEAL_CHECK_SAMPLES / 2 - 1, center_bin + bin_tolerance);
		
		// Calculate weighted center frequency and total magnitude
		float weighted_freq_sum = 0.0f;
		float total_magnitude = 0.0f;
		q15_t peak_magnitude = 0;
		int peak_bin = center_bin;
		
		for (int bin = start_bin; bin <= end_bin; bin++) {
			float bin_freq = (float)bin * 4000.0f / NUM_SEAL_CHECK_SAMPLES;
			float magnitude_weight = (float)magnitude[bin];
			
			weighted_freq_sum += bin_freq * magnitude_weight;
			total_magnitude += magnitude_weight;
			
			// Track peak for amplitude calculation
			if (magnitude[bin] > peak_magnitude) {
				peak_magnitude = magnitude[bin];
				peak_bin = bin;
			}
		}
		
		// Calculate weighted center frequency
		float actual_center_freq = 0.0f;
		bool valid_peak = false;
		
		if (total_magnitude > 0) {
			actual_center_freq = weighted_freq_sum / total_magnitude;
		} else {
			actual_center_freq = center_freq; // fallback to expected center
		}
		
		// Check if peak is valid (higher than threshold)
		if (peak_magnitude > peak_threshold) {
			valid_peak = true;
		}
		
		// Interpolate peak amplitude for better accuracy (only for valid peaks)
		float interpolated_amplitude = (float)peak_magnitude;
		//if (valid_peak && peak_bin > 0 && peak_bin < NUM_SEAL_CHECK_SAMPLES / 2 - 1) {
		if (peak_bin > 0 && peak_bin < NUM_SEAL_CHECK_SAMPLES / 2 - 1) {
			// Parabolic interpolation for peak refinement
			float y1 = (float)magnitude[peak_bin - 1];
			float y2 = (float)magnitude[peak_bin];
			float y3 = (float)magnitude[peak_bin + 1];
			
			float a = (y1 - 2*y2 + y3) / 2;
			float b = (y3 - y1) / 2;
			
			if (a != 0) {
				float peak_offset = -b / (2*a);
				// Limit offset to reasonable range
				if (peak_offset > -1.0f && peak_offset < 1.0f) {
					interpolated_amplitude = y2 - (b*b)/(4*a);
					actual_center_freq += peak_offset * (4000.0f / NUM_SEAL_CHECK_SAMPLES);
				}
			}
		}
		
		// Store valid peaks for linear regression
		if (valid_peak) {
			valid_frequencies[valid_peak_count] = actual_center_freq;
			valid_amplitudes[valid_peak_count] = interpolated_amplitude;
			valid_peak_count++;
		}
		
		printk("Bin %d: Expected %.2f Hz, Found %.2f Hz, Amplitude: %.1f (raw: %d, total_mag: %.1f) %s\n", 
			center_idx, 
			(double)center_freq, 
			(double)actual_center_freq, 
			(double)interpolated_amplitude,
			peak_magnitude,
			(double)total_magnitude,
			valid_peak ? "VALID" : "WEAK");
	}
	
	// Perform linear regression on valid peaks (magnitude vs log(frequency))
	if (valid_peak_count >= 2) {
		// Calculate log frequencies for regression
		float log_frequencies[num_bins];
		for (int i = 0; i < valid_peak_count; i++) {
			log_frequencies[i] = logf(valid_frequencies[i]);
		}
		
		// Calculate means
		float mean_log_freq = 0.0f;
		float mean_amp = 0.0f;
		for (int i = 0; i < valid_peak_count; i++) {
			mean_log_freq += log_frequencies[i];
			mean_amp += valid_amplitudes[i];
		}
		mean_log_freq /= valid_peak_count;
		mean_amp /= valid_peak_count;
		
		// Calculate slope (linear regression: magnitude vs log(frequency))
		float numerator = 0.0f;
		float denominator = 0.0f;
		for (int i = 0; i < valid_peak_count; i++) {
			float log_freq_diff = log_frequencies[i] - mean_log_freq;
			float amp_diff = valid_amplitudes[i] - mean_amp;
			numerator += log_freq_diff * amp_diff;
			denominator += log_freq_diff * log_freq_diff;
		}
		
		float slope = 0.0f;
		if (denominator != 0.0f) {
			slope = numerator / denominator;
		}
		
		// Calculate correlation coefficient for quality assessment
		/*float sum_sq_log_freq = 0.0f;
		float sum_sq_amp = 0.0f;
		for (int i = 0; i < valid_peak_count; i++) {
			float log_freq_diff = log_frequencies[i] - mean_log_freq;
			float amp_diff = valid_amplitudes[i] - mean_amp;
			sum_sq_log_freq += log_freq_diff * log_freq_diff;
			sum_sq_amp += amp_diff * amp_diff;
		}
		
		float correlation = 0.0f;
		if (sum_sq_log_freq > 0.0f && sum_sq_amp > 0.0f) {
			correlation = numerator / (sqrtf(sum_sq_log_freq) * sqrtf(sum_sq_amp));
		}*/

		float avg_peak_mag = 0.0f;
		for (int i = 0; i < valid_peak_count; i++) {
			avg_peak_mag += valid_amplitudes[i];
		}
		avg_peak_mag /= valid_peak_count;

		float mse = 0.0f;
		for (int i = 0; i < valid_peak_count; i++) {
			float freq_error = valid_amplitudes[i] / avg_peak_mag - target_magnitudes[i];
			mse += freq_error * freq_error;
		}
		mse /= valid_peak_count;

		float seal_quality = fminf(avg_peak_mag / avg_magnitude, 1.f) - mse - (slope / avg_magnitude - avg_slope);
		seal_quality = fmaxf(0.0f, fminf(100.0f, seal_quality * 100.f)); // Clamp between 0 and 100
		
		printk("Linear Regression Results (magnitude vs log(frequency)):\n");
		printk("Valid peaks: %d, Slope: %.3f\n", //, Correlation: %.3f\n", 
			valid_peak_count, (double)slope / avg_magnitude); //, (double)correlation);
		printk("Seal Quality: %.3f\n", (double)seal_quality);
		
		// Prepare and send seal check data via GATT service
		struct seal_check_data gatt_data;
		gatt_data.version = 1;
		gatt_data.quality = (uint8_t)(seal_quality); // Scale to 0-255
		gatt_data.mean_magnitude = (uint8_t)(mean_magnitude * 8.0f > 255.0f ? 255 : (uint8_t)(mean_magnitude * 8.0f));
		gatt_data.num_peaks = valid_peak_count;
		
		// Fill frequency and magnitude arrays
		for (int i = 0; i < 9; i++) {
			if (i < valid_peak_count) {
				// Convert to 12.4 fixed point (multiply by 16)
				gatt_data.frequencies[i] = (uint16_t)(valid_frequencies[i] * 16.0f);
				gatt_data.magnitudes[i] = (uint16_t)(valid_amplitudes[i] > 65535.0f ? 65535 : (uint16_t)valid_amplitudes[i]);
			} else {
				gatt_data.frequencies[i] = 0;
				gatt_data.magnitudes[i] = 0;
			}
		}
		
		// Send via GATT service
		seal_check_notify_result(&gatt_data);
	} else {
		printk("Not enough valid peaks (%d) for linear regression\n", valid_peak_count);
		
		// Send minimal data even if regression failed
		struct seal_check_data gatt_data;
		gatt_data.version = 1;
		gatt_data.quality = 0; // No quality measurement possible
		gatt_data.mean_magnitude = (uint8_t)(mean_magnitude * 8.0f > 255.0f ? 255 : (uint8_t)(mean_magnitude * 8.0f));
		gatt_data.num_peaks = valid_peak_count;
		
		// Fill available data
		for (int i = 0; i < 9; i++) {
			if (i < valid_peak_count) {
				gatt_data.frequencies[i] = (uint16_t)(valid_frequencies[i] * 16.0f);
				gatt_data.magnitudes[i] = (uint16_t)(valid_amplitudes[i] > 65535.0f ? 65535 : (uint16_t)valid_amplitudes[i]);
			} else {
				gatt_data.frequencies[i] = 0;
				gatt_data.magnitudes[i] = 0;
			}
		}
		
		seal_check_notify_result(&gatt_data);
	}
}

// Function to notify result data
int seal_check_notify_result(const struct seal_check_data *data)
{
	if (!ccc_enabled) {
		LOG_WRN("Seal check result notifications not enabled");
		return -ENOENT;
	}
	
	// Copy data to local storage
	memcpy(&seal_check_result_data, data, sizeof(seal_check_result_data));
	
	// Reset start value to indicate test completion
	seal_check_start_value = 0x00;
	
	// Send notification
	int err = bt_gatt_notify(NULL, &seal_check_svc.attrs[4], 
			        &seal_check_result_data, sizeof(seal_check_result_data));
	
	if (err) {
		LOG_ERR("Failed to notify seal check result: %d", err);
		return err;
	}
	
	LOG_INF("Seal check result notified successfully");
	return 0;
}

void seal_check_callback() {
	k_work_submit(&seal_check_complete_work);
}

// Service initialization
int init_seal_check_service(void)
{	
	// Initialize work item
	k_work_init(&seal_check_complete_work, seal_check_complete_work_handler);
	
	LOG_INF("Seal check service initialized");
	return 0;
}