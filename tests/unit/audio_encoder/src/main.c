#include <unity.h>
#include <string.h>
#include "sw_codec_select.h"

#define LEFT BT_AUDIO_LOCATION_FRONT_LEFT
#define RIGHT BT_AUDIO_LOCATION_FRONT_RIGHT
#define MONO BT_AUDIO_LOCATION_MONO_AUDIO

NET_BUF_POOL_FIXED_DEFINE(frames, 2, 32, sizeof(struct audio_metadata), NULL);
static struct net_buf *input;
static struct net_buf *output;
static const struct sw_codec_config config = {
    .sw_codec = SW_CODEC_LC3,
    .encoder = {.enabled = true, .num_ch = 2, .sample_rate_hz = 48000, .bitrate = 96000},
};
extern unsigned int encode_calls;

void setUp(void)
{
    TEST_ASSERT_EQUAL_INT(0, sw_codec_init(config));
    encode_calls = 0;
    input = net_buf_alloc(&frames, K_NO_WAIT);
    output = net_buf_alloc(&frames, K_NO_WAIT);
    TEST_ASSERT_NOT_NULL(input);
    TEST_ASSERT_NOT_NULL(output);
    memset(output->data, 0xa5, output->size);
}

void tearDown(void)
{
    net_buf_unref(input);
    net_buf_unref(output);
    TEST_ASSERT_EQUAL_INT(0, sw_codec_uninit(config));
}

static void prepare(uint32_t in_locations, uint32_t out_locations, bool interleaved)
{
    struct audio_metadata *in = net_buf_user_data(input);
    struct audio_metadata *out = net_buf_user_data(output);
    *in = (struct audio_metadata){
        .data_coding = PCM, .sample_rate_hz = 48000, .carried_bits_per_sample = 16,
        .bits_per_sample = 16, .bytes_per_location = 4, .locations = in_locations,
        .interleaved = interleaved,
    };
    *out = *in;
    out->data_coding = LC3;
    out->bytes_per_location = 2;
    out->locations = out_locations;
    const uint16_t planar[] = {0x1111, 0x1112, 0x2221, 0x2222};
    const uint16_t stereo[] = {0x1111, 0x2221, 0x1112, 0x2222};
    net_buf_add_mem(input, interleaved ? stereo : planar,
                    4 * audio_metadata_num_loc_get(in));
}

void test_disjoint_channels_reject_without_mutating_output(void)
{
    prepare(LEFT, RIGHT, false);
    struct audio_metadata before = *(struct audio_metadata *)net_buf_user_data(output);
    TEST_ASSERT_EQUAL_INT(-EINVAL, sw_codec_encode(input, output));
    TEST_ASSERT_EQUAL_UINT(0, encode_calls);
    TEST_ASSERT_EQUAL_UINT(0, output->len);
    TEST_ASSERT_EQUAL_MEMORY(&before, net_buf_user_data(output), sizeof(before));
    for (unsigned int i = 0; i < output->size; ++i) {
        TEST_ASSERT_EQUAL_HEX8(0xa5, output->data[i]);
    }
}

void test_matching_right_channel_encodes_one_frame(void)
{
    prepare(RIGHT, RIGHT, false);
    TEST_ASSERT_EQUAL_INT(0, sw_codec_encode(input, output));
    TEST_ASSERT_EQUAL_UINT(1, encode_calls);
    TEST_ASSERT_EQUAL_UINT(2, output->len);
    TEST_ASSERT_EQUAL_UINT32(RIGHT,
        ((struct audio_metadata *)net_buf_user_data(output))->locations);
}

void test_stereo_encodes_both_channels(void)
{
    prepare(LEFT | RIGHT, LEFT | RIGHT, true);
    const uint16_t expected[] = {0x1111, 0x2221};
    TEST_ASSERT_EQUAL_INT(0, sw_codec_encode(input, output));
    TEST_ASSERT_EQUAL_UINT(2, encode_calls);
    TEST_ASSERT_EQUAL_UINT(sizeof(expected), output->len);
    TEST_ASSERT_EQUAL_MEMORY(expected, output->data, sizeof(expected));
}

static void check_right_selection(bool interleaved)
{
    prepare(LEFT | RIGHT, RIGHT, interleaved);
    const uint16_t expected = 0x2221;
    TEST_ASSERT_EQUAL_INT(0, sw_codec_encode(input, output));
    TEST_ASSERT_EQUAL_UINT(1, encode_calls);
    TEST_ASSERT_EQUAL_UINT(sizeof(expected), output->len);
    TEST_ASSERT_EQUAL_MEMORY(&expected, output->data, sizeof(expected));
}

void test_select_right_from_interleaved_stereo(void) { check_right_selection(true); }
void test_select_right_from_planar_stereo(void) { check_right_selection(false); }

void test_mono_input_to_mono_output_is_valid(void)
{
    prepare(MONO, MONO, false);
    TEST_ASSERT_EQUAL_INT(0, sw_codec_encode(input, output));
    TEST_ASSERT_EQUAL_UINT(1, encode_calls);
    TEST_ASSERT_EQUAL_UINT(2, output->len);
    TEST_ASSERT_EQUAL_UINT32(MONO,
        ((struct audio_metadata *)net_buf_user_data(output))->locations);
}

void test_stereo_to_mono_selects_first_channel(void)
{
    prepare(LEFT | RIGHT, MONO, true);
    const uint16_t expected = 0x1111;
    TEST_ASSERT_EQUAL_INT(0, sw_codec_encode(input, output));
    TEST_ASSERT_EQUAL_UINT(1, encode_calls);
    TEST_ASSERT_EQUAL_UINT(sizeof(expected), output->len);
    TEST_ASSERT_EQUAL_MEMORY(&expected, output->data, sizeof(expected));
}

extern int unity_main(void);
int main(void) { return unity_main(); }
