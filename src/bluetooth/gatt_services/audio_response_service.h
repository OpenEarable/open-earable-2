#ifndef _AUDIO_RESPONSE_SERVICE_H_
#define _AUDIO_RESPONSE_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the audio response service's asynchronous work items.
 *
 * @return 0 on success.
 */
int init_audio_response_service(void);

/**
 * Signal that the microphone capture for an audio response measurement completed.
 */
void audio_response_capture_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_RESPONSE_SERVICE_H_ */
