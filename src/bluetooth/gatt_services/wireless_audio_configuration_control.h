#ifndef OPENEARABLE_WIRELESS_AUDIO_CONFIGURATION_CONTROL_H_
#define OPENEARABLE_WIRELESS_AUDIO_CONFIGURATION_CONTROL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize policy defaults, persistence hooks, and asynchronous service work. */
int init_wireless_audio_configuration_service(void);

/** Record the cumulative number of audio underruns observed by the datapath. */
void wireless_audio_configuration_underrun_observed(uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* OPENEARABLE_WIRELESS_AUDIO_CONFIGURATION_CONTROL_H_ */
