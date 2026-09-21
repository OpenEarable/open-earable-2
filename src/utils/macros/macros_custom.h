#ifndef _MACROS_2_H_
#define _MACROS_2_H_

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEFINE_GATT_SERVICE(name, on_message, msg) \
static struct k_thread thread_data; \
static k_tid_t thread_id; \
ZBUS_SUBSCRIBER_DEFINE(name ## _sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE); \
ZBUS_CHAN_DECLARE(name ## _chan); \
static K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE); \
\
static void write_ ## name ## _gatt(void)\ 
{\
	int ret;\
	const struct zbus_channel *chan;\
\
	while (1) {\
		ret = zbus_sub_wait(&name ## _sub, &chan, K_FOREVER);\
		ERR_CHK(ret);\
\
		ret = zbus_chan_read(chan, msg, ZBUS_READ_TIMEOUT_MS);\
		ERR_CHK(ret);\
\
        on_message(msg);\
\
		STACK_USAGE_PRINT(" ## name ## _msg_thread", &thread_data);\
	}\
}\
\
int init_ ## name ## _service() {\
    int ret;\
\
	thread_id = k_thread_create(\
		&thread_data, thread_stack,\
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)write_ ## name ## _gatt, NULL,\
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);\
\
	ret = k_thread_name_set(thread_id, " ## name ## _gatt_sub");\
	if (ret) {\
		LOG_ERR("Failed to create  ## name ##_msg thread");\
		return ret;\
	}\
\
    ret = zbus_chan_add_obs(&name ## _chan, &name ## _sub, ZBUS_ADD_OBS_TIMEOUT_MS);\
	if (ret) {\
		LOG_ERR("Failed to add ## name ## sub");\
		return ret;\
	}\
\
    return 0;\
}

#ifdef __cplusplus
}
#endif

#endif