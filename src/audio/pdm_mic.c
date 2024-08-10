#include "pdm_mic.h"
#include "audio_i2s.h"
#include "nrfx_pdm.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>

#include <hal/nrf_gpio.h>

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pdm_mic, CONFIG_MAIN_LOG_LEVEL);

#define BYTES_PER_SAMPLE sizeof(int16_t)
#define PDM_GAIN 0x38

#define NRF_PDM_FREQ_3072K_ACL 0x40000000

#define PDM0 DT_NODELABEL(dmic_dev)
PINCTRL_DT_DEFINE(PDM0);
static const struct pinctrl_dev_config* p_pinctrl_dev_cfg = PINCTRL_DT_DEV_CONFIG_GET(PDM0);

struct data_fifo *fifo_pdm;

void pdm_mic_start(void) {
    int ret;
    nrfx_err_t err;

    bool initialized = nrfx_pdm_init_check();

    if (!initialized) {
        LOG_ERR("PDM not initialized.");
        return;
    }

	err = nrfx_pdm_start();
	if (err != NRFX_SUCCESS) {
		LOG_ERR("START trigger failed: %d", err);
		return;
	}
}

void pdm_handler(nrfx_pdm_evt_t const *evt) {
    int ret;

    static int16_t *buffer;
    static int prev_ret;

    /* Lock last filled buffer into message queue */
    if (evt->buffer_released != NULL) {
        ret = data_fifo_block_lock(fifo_pdm, (void **)&(evt->buffer_released),
                        BLOCK_SIZE_BYTES);
        ERR_CHK_MSG(ret, "Unable to lock block RX");
    }

    if (evt->buffer_requested) {
        /* Get new empty buffer to send to PDM */
        ret = data_fifo_pointer_first_vacant_get(fifo_pdm, (void **)&buffer,
                                K_NO_WAIT);
        if (ret == 0 && prev_ret == -ENOMEM) {
            LOG_WRN("I2S RX continuing stream");
            prev_ret = ret;
        }

        /* If RX FIFO is filled up */
        if (ret == -ENOMEM) {
            void *data;
            size_t size;

            if (ret != prev_ret) {
                LOG_WRN("I2S RX overrun. Single msg");
                prev_ret = ret;
            }

            ret = data_fifo_pointer_last_filled_get(fifo_pdm, &data, &size,
                                K_NO_WAIT);
            ERR_CHK(ret);

            data_fifo_block_free(fifo_pdm, data);

            ret = data_fifo_pointer_first_vacant_get(fifo_pdm, (void **)&buffer,
                                    K_NO_WAIT);
        }

        ERR_CHK_MSG(ret, "RX failed to get block");

        nrfx_pdm_buffer_set(buffer, BLOCK_SIZE_BYTES / BYTES_PER_SAMPLE);
    }
}

void pdm_mic_stop(void) {
    nrfx_err_t err = nrfx_pdm_stop();
	if (err != NRFX_SUCCESS) {
		LOG_ERR("STOP trigger failed: %d", err);
	}
}

int pdm_mic_init(void) {
    nrfx_pdm_config_t pdm_config = {
        .mode = NRF_PDM_MODE_STEREO,
        .edge = NRF_PDM_EDGE_LEFTFALLING,
        .clk_pin = NRF_GPIO_PIN_MAP(1, 14),
        .din_pin = NRF_GPIO_PIN_MAP(0, 31),
        .gain_l = PDM_GAIN,
        .gain_r = PDM_GAIN,
        .interrupt_priority = NRFX_PDM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .clock_freq = NRF_PDM_FREQ_3072K_ACL,
        .ratio = NRF_PDM_RATIO_64X,
        .mclksrc = NRF_PDM_MCLKSRC_ACLK,
    };

    const struct pinctrl_state *state = &(p_pinctrl_dev_cfg->states[0]);

    for (unsigned i = 0; i < state->pin_cnt; ++i)
    {
        switch (NRF_GET_FUN(state->pins[i]))
        {
        case NRF_FUN_PDM_CLK:
            pdm_config.clk_pin = NRF_GET_PIN(state->pins[i]);
            LOG_DBG("pdm_config.clk_pin = %d\n", pdm_config.clk_pin);
            break;
        case NRF_FUN_PDM_DIN:
            pdm_config.din_pin = NRF_GET_PIN(state->pins[i]);
            LOG_DBG("pdm_config.din_pin = %d\n", pdm_config.din_pin);
            break;
        default:
            __ASSERT(0,"Unknown pin function\n");
        }
    }

    IRQ_DIRECT_CONNECT(PDM0_IRQn, 0, nrfx_pdm_irq_handler, 0);
    nrfx_err_t err = nrfx_pdm_init(&pdm_config, pdm_handler);
	if (err != NRFX_SUCCESS && err != NRFX_ERROR_ALREADY) {
		LOG_ERR("Failed setting up PDM: %d", err);
		return err;
	}
    
    return 0;
}

int pdm_datapath_start(struct data_fifo *fifo_rx)
{
    int ret;

	__ASSERT_NO_MSG(fifo_rx != NULL);

    fifo_pdm = fifo_rx;

    return 0;
}