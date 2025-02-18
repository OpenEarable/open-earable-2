#ifndef OPEN_EARABLE_EQUALIZER_H
#define OPEN_EARABLE_EQUALIZER_H

#include <zephyr/kernel.h>

#define EQ_ORDER 9

#ifdef __cplusplus
extern "C" {
#endif

void reset_eq();
void equalize(int16_t * data, int length);

#ifdef __cplusplus
}
#endif

#endif //OPEN_EARABLE_AUDIO_PLAYER_H
