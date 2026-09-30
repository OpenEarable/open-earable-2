/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

 #include "audio_sync_timer.h"
 #include "audio_sync_clock.h"

 #include <zephyr/kernel.h>
 #include <zephyr/init.h>
 #include <nrfx_dppi.h>
 #include <nrfx_i2s.h>
 #include <nrfx_ipc.h>
 #include <nrfx_rtc.h>
 #include <nrfx_timer.h>
 #include <nrfx_egu.h>
 
 #include <zephyr/logging/log.h>
 LOG_MODULE_REGISTER(audio_sync_timer, CONFIG_AUDIO_SYNC_TIMER_LOG_LEVEL);
 
 #define AUDIO_SYNC_TIMER_NET_APP_IPC_EVT_CHANNEL 4
 #define AUDIO_SYNC_TIMER_NET_APP_IPC_EVT	 NRF_IPC_EVENT_RECEIVE_4
 
 #define AUDIO_SYNC_HF_TIMER_INSTANCE_NUMBER 1
 
 #define AUDIO_SYNC_HF_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL 0
 #define AUDIO_SYNC_HF_TIMER_I2S_FRAME_START_EVT_CAPTURE		NRF_TIMER_TASK_CAPTURE0
 #define AUDIO_SYNC_HF_TIMER_CURR_TIME_CAPTURE_CHANNEL		1
 #define AUDIO_SYNC_HF_TIMER_CURR_TIME_CAPTURE			NRF_TIMER_TASK_CAPTURE1
 
 static const nrfx_timer_t audio_sync_hf_timer_instance =
	 NRFX_TIMER_INSTANCE(AUDIO_SYNC_HF_TIMER_INSTANCE_NUMBER);

#if defined(CONFIG_AUDIO_SYNC_DIAGNOSTICS)
static const nrfx_timer_t audio_sync_probe_timer = NRFX_TIMER_INSTANCE(2);
#endif
 
 static uint8_t dppi_channel_i2s_frame_start;
 
 #define AUDIO_SYNC_LF_TIMER_INSTANCE_NUMBER 0
 
 #define AUDIO_SYNC_LF_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL 0
 #define AUDIO_SYNC_LF_TIMER_I2S_FRAME_START_EVT_CAPTURE		NRF_RTC_TASK_CAPTURE_0
 #define AUDIO_SYNC_LF_TIMER_CURR_TIME_CAPTURE_CHANNEL		1
 #define AUDIO_SYNC_LF_TIMER_CURR_TIME_CAPTURE			NRF_RTC_TASK_CAPTURE_1
 #define CC_GET_CALLS_MAX					30
 
 static uint8_t dppi_channel_curr_time_capture;
 
 static const nrfx_rtc_config_t rtc_cfg = NRFX_RTC_DEFAULT_CONFIG;
 
 static const nrfx_rtc_t audio_sync_lf_timer_instance =
	 NRFX_RTC_INSTANCE(AUDIO_SYNC_LF_TIMER_INSTANCE_NUMBER);
 
 static uint8_t dppi_channel_timer_sync_with_rtc;
 static uint8_t dppi_channel_rtc_start;
 static volatile uint32_t num_rtc_overflows;
 
 static nrfx_timer_config_t cfg = {.frequency = NRFX_MHZ_TO_HZ(1UL),
				   .mode = NRF_TIMER_MODE_TIMER,
				   .bit_width = NRF_TIMER_BIT_WIDTH_32,
				   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				   .p_context = NULL};
 
 /* TIMER1 runs continuously. Every RTC tick latches it into CC2. Reading
  * a stable RTC counter / CC2 pair gives one coherent reference, instead of
  * combining a delayed RTC capture with a remainder from the preceding tick.
  */
 static uint32_t timestamp_from_anchor_get(uint32_t captured_us)
 {
	 uint32_t ticks = 0, anchor = 0, overflows = 0;
	 bool stable = false;
	 unsigned int key = irq_lock();
	 for (unsigned int attempt = 0; attempt < 16; ++attempt) {
		 overflows = num_rtc_overflows + nrf_rtc_event_check(
			 audio_sync_lf_timer_instance.p_reg, NRF_RTC_EVENT_OVERFLOW);
		 ticks = nrf_rtc_counter_get(audio_sync_lf_timer_instance.p_reg);
		 anchor = nrf_timer_cc_get(NRF_TIMER1, 2);
		 nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE3);
		 uint32_t now = nrf_timer_cc_get(NRF_TIMER1, 3);
		 uint32_t age = now - anchor;
		 if (ticks == nrf_rtc_counter_get(audio_sync_lf_timer_instance.p_reg) &&
		     anchor == nrf_timer_cc_get(NRF_TIMER1, 2) &&
		     overflows == num_rtc_overflows + nrf_rtc_event_check(
			 audio_sync_lf_timer_instance.p_reg, NRF_RTC_EVENT_OVERFLOW) &&
		     age >= 2 && age <= 28) {
			 stable = true;
			 break;
		 }
		 k_busy_wait(1);
	 }
	 uint32_t result = audio_sync_clock_from_anchor(ticks, overflows, anchor, captured_us);
	 irq_unlock(key);
	 if (!stable) {
		 /* Bounded even before the controller starts TIMER1. */
		 LOG_WRN("Audio timer reference unavailable");
	 }
	 return result;
 }

 uint32_t audio_sync_timer_capture(void)
 {
	 unsigned int key = irq_lock();
	 nrf_egu_task_trigger(NRF_EGU0, NRF_EGU_TASK_TRIGGER0);
	 /* Let the EGU/DPPI capture complete, even if called twice in one us. */
	 k_busy_wait(1);
	 uint32_t captured = nrf_timer_cc_get(NRF_TIMER1,
		 AUDIO_SYNC_HF_TIMER_CURR_TIME_CAPTURE_CHANNEL);
	 uint32_t result = timestamp_from_anchor_get(captured);
	 irq_unlock(key);
	 return result;
 }

 uint32_t audio_sync_timer_capture_get(void)
 {
	 unsigned int key = irq_lock();
	 static uint32_t previous;
	 uint32_t captured;
	 unsigned int attempts = 0;
	 /* NEXT_BUFFERS_NEEDED can precede FRAMESTART. A free-running capture
	  * changes on every 1 ms block, unlike the former RTC-tick remainder.
	  */
	 do {
		 captured = nrf_timer_cc_get(NRF_TIMER1,
			 AUDIO_SYNC_HF_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL);
		 if (captured != previous) break;
		 k_busy_wait(1);
	 } while (++attempts < CC_GET_CALLS_MAX);
	 if (captured == previous) LOG_WRN("Unable to get new I2S capture");
	 previous = captured;
	 uint32_t result = timestamp_from_anchor_get(captured);
	 irq_unlock(key);
	 return result;
 }

 static void unused_timer_isr_handler(nrf_timer_event_t event_type, void *ctx)
 {
	 ARG_UNUSED(event_type);
	 ARG_UNUSED(ctx);
 }
 
 static void rtc_isr_handler(nrfx_rtc_int_type_t int_type)
 {
	 if (int_type == NRFX_RTC_INT_OVERFLOW) {
		 num_rtc_overflows++;
	 }
 }
 
 /* Keep overflow event acknowledgement and epoch update indivisible to the
  * higher-priority I2S reader. Otherwise it could see an already cleared event
  * with an epoch that has not yet been incremented. Runs once per 512 seconds.
  */
 static void rtc_irq_handler(const void *unused)
 {
	 ARG_UNUSED(unused);
	 unsigned int key = irq_lock();
	 nrfx_rtc_0_irq_handler();
	 irq_unlock(key);
 }

 /**
  * @brief Initialize audio sync timer
  *
  * @note The audio sync timers is replicating the controller's clock.
  * The controller starts or clears the sync timer using a PPI signal
  * sent from the controller. This makes the two clocks synchronized.
  *
  * @return 0 if successful, error otherwise
  */
 static int audio_sync_timer_init(void)
 {
	 nrfx_err_t ret;
	 nrfx_dppi_t dppi = NRFX_DPPI_INSTANCE(0);
 
	 ret = nrfx_timer_init(&audio_sync_hf_timer_instance, &cfg, unused_timer_isr_handler);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx timer init error: %d", ret);
		 return -ENODEV;
	 }
 
	 ret = nrfx_rtc_init(&audio_sync_lf_timer_instance, &rtc_cfg, rtc_isr_handler);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx rtc init error: %d", ret);
		 return -ENODEV;
	 }
 
	 IRQ_CONNECT(RTC0_IRQn, IRQ_PRIO_LOWEST, rtc_irq_handler, NULL, 0);
	 nrfx_rtc_overflow_enable(&audio_sync_lf_timer_instance, true);
 
	 /* Initialize capturing of I2S frame start event timestamps */
	 ret = nrfx_dppi_channel_alloc(&dppi, &dppi_channel_i2s_frame_start);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel alloc error (I2S frame start): %d", ret);
		 return -ENOMEM;
	 }
 
	 nrf_timer_subscribe_set(audio_sync_hf_timer_instance.p_reg,
				 AUDIO_SYNC_HF_TIMER_I2S_FRAME_START_EVT_CAPTURE,
				 dppi_channel_i2s_frame_start);
 
	 /* Initialize capturing of I2S frame start event timestamps at the RTC as well. */
	 nrf_rtc_subscribe_set(audio_sync_lf_timer_instance.p_reg,
				   AUDIO_SYNC_LF_TIMER_I2S_FRAME_START_EVT_CAPTURE,
				   dppi_channel_i2s_frame_start);
 
	 nrf_i2s_publish_set(NRF_I2S0, NRF_I2S_EVENT_FRAMESTART, dppi_channel_i2s_frame_start);
#if defined(CONFIG_AUDIO_SYNC_DIAGNOSTICS)
	 /* Independent free-running clock: a discontinuity in the RTC/TIMER1
	  * reconstruction must not be mistaken for a physical I2S timing jump.
	  */
	 nrfx_timer_config_t probe_cfg = cfg;
	 probe_cfg.frequency = NRFX_MHZ_TO_HZ(16UL);
	 ret = nrfx_timer_init(&audio_sync_probe_timer, &probe_cfg, unused_timer_isr_handler);
	 if (ret != NRFX_SUCCESS) return -ENODEV;
	 nrf_timer_subscribe_set(audio_sync_probe_timer.p_reg, NRF_TIMER_TASK_CAPTURE0,
				dppi_channel_i2s_frame_start);
	 nrfx_timer_enable(&audio_sync_probe_timer);
#endif
	 ret = nrfx_dppi_channel_enable(&dppi, dppi_channel_i2s_frame_start);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel enable error (I2S frame start): %d", ret);
		 return -EIO;
	 }
 
	 /* Initialize capturing of current timestamps */
	 ret = nrfx_dppi_channel_alloc(&dppi, &dppi_channel_curr_time_capture);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel alloc error (I2S frame start) - Return value: %d", ret);
		 return -ENOMEM;
	 }
 
	 nrf_rtc_subscribe_set(audio_sync_lf_timer_instance.p_reg,
				   AUDIO_SYNC_LF_TIMER_CURR_TIME_CAPTURE,
				   dppi_channel_curr_time_capture);
 
	 nrf_timer_subscribe_set(audio_sync_hf_timer_instance.p_reg,
				 AUDIO_SYNC_HF_TIMER_CURR_TIME_CAPTURE,
				 dppi_channel_curr_time_capture);
 
	 nrf_egu_publish_set(NRF_EGU0, NRF_EGU_EVENT_TRIGGERED0, dppi_channel_curr_time_capture);
 
	 ret = nrfx_dppi_channel_enable(&dppi, dppi_channel_curr_time_capture);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel enable error (I2S frame start) - Return value: %d", ret);
		 return -EIO;
	 }
 
	 /* Initialize functionality for synchronization between APP and NET core */
	 ret = nrfx_dppi_channel_alloc(&dppi, &dppi_channel_rtc_start);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel alloc error (timer clear): %d", ret);
		 return -ENOMEM;
	 }
 
	 nrf_rtc_subscribe_set(audio_sync_lf_timer_instance.p_reg, NRF_RTC_TASK_CLEAR,
				   dppi_channel_rtc_start);
	 nrf_timer_subscribe_set(audio_sync_hf_timer_instance.p_reg, NRF_TIMER_TASK_START,
				 dppi_channel_rtc_start);
 
	 nrf_ipc_receive_config_set(NRF_IPC, AUDIO_SYNC_TIMER_NET_APP_IPC_EVT_CHANNEL,
					NRF_IPC_CHANNEL_4);
	 nrf_ipc_publish_set(NRF_IPC, AUDIO_SYNC_TIMER_NET_APP_IPC_EVT, dppi_channel_rtc_start);
 
	 ret = nrfx_dppi_channel_enable(&dppi, dppi_channel_rtc_start);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel enable error (timer clear): %d", ret);
		 return -EIO;
	 }
 
	 /* Initialize functionality for synchronization between RTC and TIMER */
	 ret = nrfx_dppi_channel_alloc(&dppi, &dppi_channel_timer_sync_with_rtc);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel alloc error (timer clear): %d", ret);
		 return -ENOMEM;
	 }
 
	 nrf_rtc_publish_set(audio_sync_lf_timer_instance.p_reg, NRF_RTC_EVENT_TICK,
				 dppi_channel_timer_sync_with_rtc);
	 nrf_timer_subscribe_set(audio_sync_hf_timer_instance.p_reg, NRF_TIMER_TASK_CAPTURE2,
				 dppi_channel_timer_sync_with_rtc);
 
	 nrfx_rtc_tick_enable(&audio_sync_lf_timer_instance, false);
 
	 ret = nrfx_dppi_channel_enable(&dppi, dppi_channel_timer_sync_with_rtc);
	 if (ret - NRFX_ERROR_BASE_NUM) {
		 LOG_ERR("nrfx DPPI channel enable error (timer clear): %d", ret);
		 return -EIO;
	 }
 
	 nrfx_rtc_enable(&audio_sync_lf_timer_instance);
 
	 LOG_DBG("Audio sync timer initialized");
 
	 return 0;
 }
 
 SYS_INIT(audio_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
