/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bank_controller.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>

#include "zbus_common.h"
#include "button_assignments.h"
#include "macros_common.h"

#include "hw_codec.h"

LOG_MODULE_REGISTER(bank_controller, CONFIG_MAIN_LOG_LEVEL);

ZBUS_SUBSCRIBER_DEFINE(bank_button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(button_chan);

static struct k_thread bank_controller_thread_data;
static k_tid_t bank_controller_thread_id;

K_THREAD_STACK_DEFINE(bank_controller_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);

// Current filter bank state
static bool filter_bank_a_active = true;

/**
 * @brief Get current filter bank state
 * @return true if bank A is active, false if bank B is active
 */
bool bank_controller_is_bank_a_active(void)
{
    return filter_bank_a_active;
}

/**
 * @brief Switch to the other filter bank
 */
static void bank_controller_switch_bank(void)
{
    if (hw_codec_get_audio_mode() == AUDIO_MODE_NORMAL) {
        hw_codec_set_audio_mode(AUDIO_MODE_TRANSPARENCY);
    } else {
        hw_codec_set_audio_mode(AUDIO_MODE_NORMAL);
    }
}

/**
 * @brief Handle button events for bank switching
 */
static void bank_controller_thread(void)
{
    int ret;
    const struct zbus_channel *chan;

    LOG_INF("Bank controller thread started");

    while (1) {
        ret = zbus_sub_wait(&bank_button_evt_sub, &chan, K_FOREVER);
        ERR_CHK(ret);

        struct button_msg msg;

        ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
        ERR_CHK(ret);

        LOG_DBG("Bank controller got btn evt - id = %d, action = %d", 
                msg.button_pin, msg.button_action);

        // Only react to button released events
        if (msg.button_action != BUTTON_RELEASED) {
            continue;
        }

        // Switch banks on any button release
        bank_controller_switch_bank();

        STACK_USAGE_PRINT("bank_controller_thread", &bank_controller_thread_data);
    }
}

/**
 * @brief Initialize the bank controller
 * @return 0 for success, error otherwise
 */
int bank_controller_init(void)
{
    int ret;

    LOG_INF("Initializing bank controller");

    // Create the bank controller thread
    bank_controller_thread_id = k_thread_create(
        &bank_controller_thread_data, bank_controller_thread_stack,
        CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)bank_controller_thread, 
        NULL, NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
    
    ret = k_thread_name_set(bank_controller_thread_id, "BANK_CONTROLLER");
    if (ret) {
        LOG_ERR("Failed to create bank controller thread");
        return ret;
    }

    // Subscribe to button events
    ret = zbus_chan_add_obs(&button_chan, &bank_button_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
    if (ret) {
        LOG_ERR("Failed to add bank controller button subscriber: %d", ret);
        return ret;
    }

    return 0;
}
