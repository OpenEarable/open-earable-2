/* SPDX-License-Identifier: Apache-2.0 */
#ifndef DEVICE_CONTROL_SERVICE_H_
#define DEVICE_CONTROL_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Register after all existing services and settings_load(), before advertising.
 * Appending dynamically preserves the handles cached by bonded phones and lets
 * Zephyr announce the added range through Service Changed.
 */
int device_control_service_register(void);

#ifdef __cplusplus
}
#endif

#endif
