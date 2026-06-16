/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fatal.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>

/* Print everything from the error handler */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(error_handler, CONFIG_ERROR_HANDLER_LOG_LEVEL);

#if (defined(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP) && (CONFIG_DEBUG))
/* nRF5340 Audio DK center RGB LED */
static const struct gpio_dt_spec center_led_r = GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_red), gpios);
static const struct gpio_dt_spec center_led_g = GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_green), gpios);
static const struct gpio_dt_spec center_led_b = GPIO_DT_SPEC_GET(DT_NODELABEL(rgb1_blue), gpios);
#endif /* (defined(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP) && (CONFIG_DEBUG)) */

void error_handler(unsigned int reason, const struct arch_esf *esf)
{
#if (CONFIG_DEBUG)
	LOG_ERR("Caught system error -- reason %d. Entering infinite loop", reason);
	LOG_PANIC();
#if defined(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP)
	(void)gpio_pin_configure_dt(&center_led_r, GPIO_OUTPUT_ACTIVE);
	(void)gpio_pin_configure_dt(&center_led_g, GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_configure_dt(&center_led_b, GPIO_OUTPUT_INACTIVE);

	irq_lock();
	while (1) {
		__asm__ volatile("nop");
	}
#endif /* defined(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP) */
#if CONFIG_BOARD_OPENEARABLE_V2_NRF5340_CPUAPP
	irq_lock();
	const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(led_error), gpios, {0});

	// Blink error LED to alert user of the crash
	for (int i = 0; i < 250; i++) {
		gpio_pin_set_dt(&error_led, 1);
		k_busy_wait(100000);
		gpio_pin_set_dt(&error_led, 0);
		k_busy_wait(300000);
	}

    // Bit-bang peripheral load switches to LOW to prevent power drain during System OFF.
    // This is safe to call from fault handlers (irq_lock held) unlike zephyr PM APIs.
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 11)); // ls_1_8
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 14)); // ls_3_3
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 12)); // ls_sd
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 6));  // ppg_ldo
    
    // Drive PMIC CD (Chip Disable) LOW to enter high-impedance mode
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 17)); // pmic_cd

    // Force network core off so that the main core can enter System OFF
    NRF_RESET->NETWORK.FORCEOFF = 1;

    sys_poweroff();

	// Fallback in case sys_poweroff() returns
	while (1) {
		k_busy_wait(1000000);
	}
#endif
#else
	LOG_ERR("Caught system error -- reason %d. Cold rebooting.", reason);
#if (CONFIG_LOG)
	LOG_PANIC();
#endif /* (CONFIG_LOG) */
	sys_reboot(SYS_REBOOT_COLD);
#endif /* (CONFIG_DEBUG) */
	CODE_UNREACHABLE;
}

void bt_ctlr_assert_handle(char *c, int code)
{
	LOG_ERR("BT controller assert: %s, code: 0x%x", c, code);
	error_handler(code, NULL);
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	error_handler(reason, esf);
}

void assert_post_action(const char *file, unsigned int line)
{
	LOG_ERR("Assert post action: file: %s, line %d", file, line);
	error_handler(0, NULL);
}
