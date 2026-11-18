#include <stdbool.h>
#include <string.h>
#include <sw_codec_lc3.h>

/* Keep channel selection observable without the hardware LC3 library. */
unsigned int encode_calls;

int sw_codec_lc3_enc_run(void const *const pcm_data, uint32_t pcm_data_size,
                       uint32_t enc_bitrate, uint8_t audio_ch, uint16_t lc3_data_buf_size,
                       uint8_t *const lc3_data, uint16_t *const lc3_data_wr_size)
{
    ++encode_calls;
    memcpy(lc3_data, pcm_data, sizeof(uint16_t));
    *lc3_data_wr_size = sizeof(uint16_t);
    return 0;
}

int sw_codec_lc3_single_rate_init(uint16_t encoder_sample_rate, uint16_t decoder_sample_rate,
                                uint8_t *buffer, uint32_t *buffer_size, uint16_t framesize_us)
{
    return 0;
}

int sw_codec_lc3_enc_init(uint16_t pcm_sample_rate, uint8_t pcm_bit_depth, uint16_t framesize_us,
                        uint32_t enc_bitrate, uint8_t num_channels, uint16_t *const pcm_bytes_req)
{
    *pcm_bytes_req = 4;
    return 0;
}

int sw_codec_lc3_dec_init(uint16_t pcm_sample_rate, uint8_t pcm_bit_depth, uint16_t framesize_us,
                        uint8_t num_channels)
{
    return 0;
}

int sw_codec_lc3_enc_uninit_all(void) { return 0; }
int sw_codec_lc3_dec_uninit_all(void) { return 0; }
int sw_codec_lc3_uninit(void) { return 0; }
