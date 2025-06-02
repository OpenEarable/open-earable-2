/*!
 * @brief     Lark Device Level APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */

/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_device_get_id(adi_lark_device_t *device, uint8_t *vid, uint16_t *did, uint8_t *drev)
{
    int32_t  err;
    uint32_t id;

    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(vid);
    LARK_NULL_POINTER_RETURN(did);
    LARK_NULL_POINTER_RETURN(drev);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_read(device, BF_VENDOR_INFO, &id);
    LARK_ERROR_RETURN(err);
    *vid = (uint8_t)id;

    err = adi_lark_hal_bf_read(device, BF_DEVICE1_INFO, &id);
    LARK_ERROR_RETURN(err);
    *did = (uint16_t)id;
    err = adi_lark_hal_bf_read(device, BF_DEVICE2_INFO, &id);
    LARK_ERROR_RETURN(err);
    *did = (uint16_t)((id << 8) + (*did));

    err = adi_lark_hal_bf_read(device, BF_REV_INFO, &id);
    LARK_ERROR_RETURN(err);
    *drev = (uint8_t)id;

    #ifdef LARK_LITE_SDK
    if (*did != 0x1850)
        LARK_LOG_WARN("silicon is not matched to SDK (ADAU1850).");
    #endif
    #ifdef LARK_SDK
    if (*did != 0x1860)
        LARK_LOG_WARN("silicon is not matched to SDK (ADAU1860).");
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_get_sdk_rev(adi_lark_device_t *device, uint32_t *rev)
{
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(rev);
    LARK_LOG_FUNC();

    *rev = ADI_LARK_SDK_VER;
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_init(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t mode;
    uint8_t vid;
    uint16_t did;
    uint8_t drev;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_get_id(device, &vid, &did, &drev);
    LARK_ERROR_RETURN(err);
    if (did != 0x1860)
    {
        return API_LARK_ERROR_NOT_SUPPORTED;
    }
    
    err = adi_lark_hal_bf_write(device, BF_FDSP_MODE_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_read(device, BF_FDSP_MODE_INFO, &mode);
    LARK_ERROR_RETURN(err);
    if (mode != 1)
    {
        return API_LARK_ERROR_NOT_SUPPORTED;
    }
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_reset_device(adi_lark_device_t *device, bool full_reset)
{
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    if (full_reset)
        err = adi_lark_hal_bf_write(device, BF_SOFT_FULL_RESET_INFO, 1);
    else
        err = adi_lark_hal_bf_write(device, BF_SOFT_RESET_INFO, 1);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_power_off_adp(adi_lark_device_t *device)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    /* adc & dac */
    err = adi_lark_adc_enable_power_on(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_enable_power_on(device, 1, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_enable_power_on(device, 2, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, false);
    LARK_ERROR_RETURN(err);

    /* pdm */
    err = adi_lark_pdm_enable_power_on(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_enable_power_on(device, 1, false);
    LARK_ERROR_RETURN(err);

    /* fdsp */
    err = adi_lark_fdsp_enable_power_on(device, false);
    LARK_ERROR_RETURN(err);

    /* asrc */
    err = adi_lark_asrc_enable_multi_chnls_power_on(device, 0xff, false);
    LARK_ERROR_RETURN(err);

    /* fint */
    err = adi_lark_fint_enable_multi_chnls_power_on(device, 0xff, false);
    LARK_ERROR_RETURN(err);

    /* fdec */
    err = adi_lark_fdec_enable_multi_chnls_power_on(device, 0xff, false);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    /* dmic */
    err = adi_lark_dmic_enable_clock_power_on(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_multi_chnls_power_on(device, 0x0f, false);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_set_pin_drive_slew_pull(adi_lark_device_t *device, adi_lark_pin_name_e pin,
    adi_lark_pin_drive_e drive, adi_lark_pin_slew_e slew, adi_lark_pin_pull_e pull)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    #ifdef LARK_SDK
    err = adi_lark_hal_bf_write(device, pin + BF_DMIC_CLK_DRIVE_INFO, drive);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, pin + BF_DMIC_CLK_SLEW_INFO, slew);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, pin + BF_DMIC_CLK_PULL_EN_INFO, pull != API_LARK_PIN_PULL_NONE ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, pin + BF_DMIC_CLK_PULL_SEL_INFO, pull);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_hal_bf_write(device, pin + BF_FSYNC0_DRIVE_INFO, drive);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, pin + BF_FSYNC0_SLEW_INFO, slew);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, pin + BF_FSYNC0_PULL_EN_INFO, pull != API_LARK_PIN_PULL_NONE ? 1 : 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, pin + BF_FSYNC0_PULL_SEL_INFO, pull);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
#include <xtensa/tie/hifi3z_lark.h>
#if LARK_SDK_XOS_TEST_CODE
#include <xtensa/xos.h>
#else
#include <xtensa/xtruntime.h>
#endif

typedef struct
{
    adi_lark_device_t *device;
    volatile uint32_t *c0_pri_cfg_ptr;
    uint32_t c0_pri_cfg_val;
    volatile uint32_t *c0_alt_cfg_ptr;
    uint32_t c0_alt_cfg_val;
    volatile uint32_t *c1_pri_cfg_ptr;
    uint32_t c1_pri_cfg_val;
    volatile uint32_t *c1_alt_cfg_ptr;
    uint32_t c1_alt_cfg_val;
} adi_lark_device_ds_dma_int_arg_t;

#define API_LARK_RW_TEST_BUF_SIZE 128 /*!< 128*4bytes buffer for read/write test */
extern adi_lark_dma_chnl_ctrl_t dma_ctrl_data[];
uint32_t __attribute__((section(".dram0.data"))) mem_dram0[API_LARK_RW_TEST_BUF_SIZE];
uint32_t __attribute__((section(".dram1.data"))) mem_dram1[API_LARK_RW_TEST_BUF_SIZE];
uint32_t __attribute__((section(".sram.data")))  mem_sramx[API_LARK_RW_TEST_BUF_SIZE];

void adi_lark_device_ds_tie_handler(void *arg);
void adi_lark_device_ds_dma_handler(void *arg);
#endif

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
int32_t adi_lark_device_memory_test(adi_lark_device_t *device)
{
    int32_t err;
    int32_t i;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = API_LARK_ERROR_OK;
    LARK_LOG_MSG("memory r/w test start...");

    /* step1: ramp data test */
    {
        for (i = 0; i < API_LARK_RW_TEST_BUF_SIZE; i ++)
        {
            mem_dram0[i] = mem_dram1[i] = mem_sramx[i] = i;
        }

        /* write sram back if needed */
        xthal_dcache_region_writeback((void *)mem_sramx, API_LARK_RW_TEST_BUF_SIZE << 2);

        for (i = 0; i < API_LARK_RW_TEST_BUF_SIZE; i ++)
        {
            if (mem_dram0[i] != i || mem_dram1[i] != i || mem_sramx[i] != i)
            {
                LARK_LOG_VAR(LARK_LOG_ERR_MSG, "memory r/w failed: %x, %x, %x", mem_dram0[i], mem_dram1[i], mem_sramx[i]);
                err = API_LARK_ERROR_ERROR;
                break;
            }
        }
        LARK_ERROR_RETURN(err);
    }

    /* step2: 0x55aa55aa as data */
    {
        for (i = 0; i < API_LARK_RW_TEST_BUF_SIZE; i ++)
        {
            mem_dram0[i] = mem_dram1[i] = mem_sramx[i] = 0x55aa55aa;
        }

        /* write sram back if needed */
        xthal_dcache_region_writeback((void *)mem_sramx, API_LARK_RW_TEST_BUF_SIZE << 2);

        for (i = 0; i < API_LARK_RW_TEST_BUF_SIZE; i ++)
        {
            if (mem_dram0[i] != 0x55aa55aa || mem_dram1[i] != 0x55aa55aa || mem_sramx[i] != 0x55aa55aa)
            {
                LARK_LOG_VAR(LARK_LOG_ERR_MSG, "memory r/w failed: %x, %x, %x", mem_dram0[i], mem_dram1[i], mem_sramx[i]);
                err = API_LARK_ERROR_ERROR;
                break;
            }
        }
        LARK_ERROR_RETURN(err);
    }

    /* step3: 0xaa55aa55 as data */
    {
        for (i = 0; i < API_LARK_RW_TEST_BUF_SIZE; i ++)
        {
            mem_dram0[i] = mem_dram1[i] = mem_sramx[i] = 0xaa55aa55;
        }

        /* write sram back if needed */
        xthal_dcache_region_writeback((void *)mem_sramx, API_LARK_RW_TEST_BUF_SIZE << 2);

        for (i = 0; i < API_LARK_RW_TEST_BUF_SIZE; i ++)
        {
            if (mem_dram0[i] != 0xaa55aa55 || mem_dram1[i] != 0xaa55aa55 || mem_sramx[i] != 0xaa55aa55)
            {
                LARK_LOG_VAR(LARK_LOG_ERR_MSG, "memory r/w failed: %x, %x, %x", mem_dram0[i], mem_dram1[i], mem_sramx[i]);
                err = API_LARK_ERROR_ERROR;
                break;
            }
        }
        LARK_ERROR_RETURN(err);
    }

    LARK_LOG_MSG("memory r/w test passed.\n");
    return API_LARK_ERROR_OK;
}
#endif

#if LARK_SDK_EN_TEST_CODE
int32_t adi_lark_device_init_adp_for_test(adi_lark_device_t *device)
{
    /* initialize adp (audio data path) settings to default for running new test case only.
     * customer doesn't need to call this in real application.
     */

    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    /* power on all modules */
    err = adi_lark_pmu_enable_master_block(device, true);
    LARK_ERROR_RETURN(err);

    /* power off adp module */
    err = adi_lark_device_power_off_adp(device);
    LARK_ERROR_RETURN(err);

    /* disable i2s in & out */
    err = adi_lark_sap_enable_chnl_input(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 0, false);
    LARK_ERROR_RETURN(err);
    #ifdef LARK_SDK
    err = adi_lark_sap_enable_chnl_input(device, 1, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, false);
    LARK_ERROR_RETURN(err);
    #endif

    #ifdef LARK_SDK
    /* disable ds int req */
    err = adi_lark_ds_enable_multi_chnls_int(device, 0x3ffff, false);
    LARK_ERROR_RETURN(err);
    /* clear ds int status */
    err = adi_lark_ds_clear_int_status(device, 0x3ffff);
    LARK_ERROR_RETURN(err);
    /* disable ds fifo channel r/w */
    err = adi_lark_ds_enable_multi_fifo_chnls_read(device, 0x0fff, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_fifo_chnls_write(device, 0x0fff, false);
    LARK_ERROR_RETURN(err);
    /* disable ds dma req */
    err = adi_lark_ds_enable_multi_chnls_dmareq(device, 0xffff, false);
    LARK_ERROR_RETURN(err);
    /* disable ds tie */
    err = adi_lark_ds_enable_tie_ltif(device, false);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_clock_rate_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq)
{
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    LARK_LOG_MSG("Clock Rate Changing on the fly test start ... ");

    LARK_LOG_MSG("Select the correct MCLK ... ");
    err = adi_lark_clk_set_mclk_freq(device, mclk_freq, true);
    LARK_ERROR_RETURN(err);
    LARK_LOG_MSG("Done");

    LARK_LOG_MSG("Switch to 1/2 clock rate ... ");
    err = adi_lark_clk_set_tdsp_bus_rate(device, API_LARK_TDSP_CLK_RATE_MCLK_OVER_2, API_LARK_BUS_CLK_RATE_MCLK_OVER_2);
    LARK_ERROR_RETURN(err);
    LARK_LOG_MSG("Done");

    if (mclk_freq > API_LARK_MCLK_FREQ_24P576)
    {
        LARK_LOG_MSG("Switch to 1/3 clock rate ... ");
        err = adi_lark_clk_set_tdsp_bus_rate(device, API_LARK_TDSP_CLK_RATE_MCLK_OVER_3, API_LARK_BUS_CLK_RATE_MCLK_OVER_3);
        LARK_ERROR_RETURN(err);
        LARK_LOG_MSG("Done");

        LARK_LOG_MSG("Switch to 1/4 clock rate ... ");
        err = adi_lark_clk_set_tdsp_bus_rate(device, API_LARK_TDSP_CLK_RATE_MCLK_OVER_4, API_LARK_BUS_CLK_RATE_MCLK_OVER_4);
        LARK_ERROR_RETURN(err);
        LARK_LOG_MSG("Done");
    }

    /* reset to /1 */
    err = adi_lark_clk_set_tdsp_bus_rate(device, API_LARK_TDSP_CLK_RATE_MCLK_OVER_1, API_LARK_BUS_CLK_RATE_MCLK_OVER_1);
    LARK_ERROR_RETURN(err);

    LARK_LOG_MSG("Clock Rate Changing on the fly test passed. \n");

    return API_LARK_ERROR_OK;
}
#endif

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
int32_t adi_lark_device_ds_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq)
{
    int32_t err, i;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_clk_set_mclk_freq(device, mclk_freq, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pmu_enable_master_block(device, true);
    LARK_ERROR_RETURN(err);
    for(i = 0; i < 10000; i++)
        ; /* delay. Check reg status in the future */

    LARK_LOG_MSG("DS/SAP DMA loopback test start ... ");
    err = adi_lark_device_ds_i2s_dma_loopback_test(device);
    if (err != API_LARK_ERROR_OK)
        LARK_LOG_MSG("DS/SAP DMA loopback test failed.\n");
    else
        LARK_LOG_MSG("DS/SAP DMA loopback test ended. Did you hear music?\n");

    LARK_LOG_MSG("DS/SAP FIFO loopback test start ... ");
    err = adi_lark_device_ds_i2s_fifo_loopback_test(device);
    if (err != API_LARK_ERROR_OK)
        LARK_LOG_MSG("DS/SAP FIFO loopback test failed.\n");
    else
        LARK_LOG_MSG("DS/SAP FIFO loopback test ended. Did you hear music?\n");
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_ds_i2s_tie_loopback_test(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_ds_i2s_dma_loopback_test(adi_lark_device_t *device)
{
    int32_t err;
    volatile int32_t loop_count = 0x08000000;
    adi_lark_sap_config_t sap_config;
    adi_lark_dma_chnl_cfg_t *dma_config;
    adi_lark_dma_chnl_ctrl_t *dma_ctrl;
    static adi_lark_device_ds_dma_int_arg_t arg;
    adi_lark_device_ds_dma_int_arg_t* dma_int_arg = &arg;
    uint32_t dma_alt_base, irq_id;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_TDSP0);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_select_rdy2dma_chnl(device, 0, API_LARK_DS_INPUT_SPT0_SLOT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2dma_chnl(device, 1, API_LARK_DS_INPUT_SPT0_SLOT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_SPT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_clear_chnl_int_status(device, 5);                     /* Clear DS int chan from SPT0 */
    LARK_ERROR_RETURN(err);

    /*start DMA config*/
    dma_ctrl = &dma_ctrl_data[0];
    dma_ctrl->dma_src_end = LARK_ADP2SOC_BUF_START_ADDR + API_LARK_DS_INPUT_SPT0_SLOT0 * 4 + 3;
    dma_ctrl->dma_dst_end = (uint32_t)mem_dram0 + API_LARK_RW_TEST_BUF_SIZE * 4 - 1;
    dma_config = (adi_lark_dma_chnl_cfg_t*)&dma_ctrl->dma_chnl_cfg;
    dma_config->src_inc = API_LARK_DMA_INCR_NONE;
    dma_config->dst_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->src_size = API_LARK_DMA_WIDTH_4_BYTE;
    dma_config->r_power = 0;
    dma_config->n_minus_1 = API_LARK_RW_TEST_BUF_SIZE - 1;
    dma_config->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_PING_PONG;

    dma_int_arg->c0_pri_cfg_ptr = &dma_ctrl->dma_chnl_cfg;
    dma_int_arg->c0_pri_cfg_val = dma_ctrl->dma_chnl_cfg;

    dma_ctrl = &dma_ctrl_data[1];
    dma_ctrl->dma_src_end = (uint32_t)mem_dram1 + API_LARK_RW_TEST_BUF_SIZE * 4 - 1;
    dma_ctrl->dma_dst_end = LARK_SOC2ADP_BUF_START_ADDR + 3;                /* some dummy data will be moved from dram1 to TDSP0*/
    dma_config = (adi_lark_dma_chnl_cfg_t*)&dma_ctrl->dma_chnl_cfg;
    dma_config->src_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->dst_inc = API_LARK_DMA_INCR_NONE;
    dma_config->src_size = API_LARK_DMA_WIDTH_4_BYTE;
    dma_config->r_power = 0;
    dma_config->n_minus_1 = API_LARK_RW_TEST_BUF_SIZE - 1;
    dma_config->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_PING_PONG;

    dma_int_arg->c1_pri_cfg_ptr = &dma_ctrl->dma_chnl_cfg;
    dma_int_arg->c1_pri_cfg_val = dma_ctrl->dma_chnl_cfg;

    /* make sure dma ctrl data is saved to ram */
    if((uint32_t)dma_ctrl_data >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(dma_ctrl_data, sizeof(adi_lark_dma_chnl_ctrl_t) * 64);

    err = adi_lark_dma_init(device, (uint32_t)dma_ctrl_data);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_get_ctrl_base_reg(device, 1, &dma_alt_base);
    LARK_ERROR_RETURN(err);

    dma_ctrl = (adi_lark_dma_chnl_ctrl_t*)dma_alt_base;
    dma_ctrl->dma_src_end = LARK_ADP2SOC_BUF_START_ADDR + API_LARK_DS_INPUT_SPT0_SLOT0 * 4 + 3;
    dma_ctrl->dma_dst_end = (uint32_t)mem_dram1 + API_LARK_RW_TEST_BUF_SIZE * 4 - 1;
    dma_config = (adi_lark_dma_chnl_cfg_t*)&dma_ctrl->dma_chnl_cfg;
    dma_config->src_inc = API_LARK_DMA_INCR_NONE;
    dma_config->dst_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->src_size = API_LARK_DMA_WIDTH_4_BYTE;
    dma_config->r_power = 0;
    dma_config->n_minus_1 = API_LARK_RW_TEST_BUF_SIZE - 1;
    dma_config->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_PING_PONG;

    dma_int_arg->c0_alt_cfg_ptr = &dma_ctrl->dma_chnl_cfg;
    dma_int_arg->c0_alt_cfg_val = dma_ctrl->dma_chnl_cfg;

    dma_ctrl++;
    dma_ctrl->dma_src_end = (uint32_t)mem_dram0 + API_LARK_RW_TEST_BUF_SIZE * 4 - 1;
    dma_ctrl->dma_dst_end = LARK_SOC2ADP_BUF_START_ADDR + 3;                /* data moved from dram0 to TDSP0*/
    dma_config = (adi_lark_dma_chnl_cfg_t*)&dma_ctrl->dma_chnl_cfg;
    dma_config->src_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->dst_inc = API_LARK_DMA_INCR_NONE;
    dma_config->src_size = API_LARK_DMA_WIDTH_4_BYTE;
    dma_config->r_power = 0;
    dma_config->n_minus_1 = API_LARK_RW_TEST_BUF_SIZE - 1;
    dma_config->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_PING_PONG;

    dma_int_arg->c1_alt_cfg_ptr = &dma_ctrl->dma_chnl_cfg;
    dma_int_arg->c1_alt_cfg_val = dma_ctrl->dma_chnl_cfg;
    /* make sure dma ctrl data is saved to ram */
    if((uint32_t)dma_ctrl_data >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(dma_ctrl_data, sizeof(adi_lark_dma_chnl_ctrl_t) * 64);
    /* register dma interrupt and enable channel xfer */
    err = adi_lark_int_get_soc_irq_id(device, API_LARK_SOC_IRQ_DMA, &irq_id);
    LARK_ERROR_RETURN(err);
    dma_int_arg->device = device;
    #if LARK_SDK_XOS_TEST_CODE
    err = xos_register_interrupt_handler(irq_id, adi_lark_device_ds_dma_handler, (void*)dma_int_arg);
    LARK_ERROR_RETURN(err != 0 ? API_LARK_ERROR_REGISTER_ISR : API_LARK_ERROR_OK);
    xos_interrupt_enable(irq_id);
    #else
    _xtos_set_interrupt_handler_arg(irq_id, adi_lark_device_ds_dma_handler, (void*)dma_int_arg);
    _xtos_interrupt_enable(irq_id);
    #endif
    err = adi_lark_dma_enable_chnl_high_priority(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_high_priority(device, 1, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_dma_done_int(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_dma_done_int(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_dma_err_int(device, 3);
    LARK_ERROR_RETURN(err);

    /* turn on hardware */
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_multi_chnls(device, ( 1 << 0 ) | (1 << 1), true, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_chnls_dmareq(device, ( 1 << 0 ) | (1 << 1), true);
    LARK_ERROR_RETURN(err);

    while(loop_count);
    /* The code below will not be executed, just to demonstate how to disable the modules */
    err = adi_lark_dma_enable_chnl(device, 0, false, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl(device, 1, false, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_chnl_dmareq(device, 0, false);                 /* Disable data sync DMA chan 0 */
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_chnl_dmareq(device, 1, false);                 /* Disable data sync DMA chan 1 */
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_dma_done_int(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_dma_done_int(device, 1, false);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

void adi_lark_device_ds_dma_handler(void *arg)
{
    adi_lark_device_t *device;
    adi_lark_log_write log_func;
    adi_lark_device_ds_dma_int_arg_t* dma_int_arg;
    uint32_t int_stat, bus_stat, invalid_desc_stat;
    static uint32_t dc0_int_count = 0, dc1_int_count = 0, invalid_count = 0;

    if(arg == NULL)
       return;
    dma_int_arg = (adi_lark_device_ds_dma_int_arg_t*)arg;
    invalid_desc_stat = *(uint32_t*)REG_DMA_INVDESC_IDX_CLR_ADDR;
    bus_stat = *(uint32_t*)REG_DMA_BUSERR_IDX_CLR_ADDR;
    if(invalid_desc_stat > 0)
    {
        *(uint32_t*)REG_DMA_INVDESC_IDX_CLR_ADDR = invalid_desc_stat;
        invalid_count++;
    }
    if(bus_stat > 0)
    {
        *(uint32_t*)REG_DMA_BUSERR_IDX_CLR_ADDR = bus_stat;
    }
    int_stat = *(uint32_t*)REG_DMA_INT_STATUS_CLR_ADDR;
    if(int_stat & 0x0001)
    {
        dc0_int_count++;
        if(dc0_int_count%2)
        {
            *dma_int_arg->c0_pri_cfg_ptr = dma_int_arg->c0_pri_cfg_val;
        }
        else
        {
            *dma_int_arg->c0_alt_cfg_ptr = dma_int_arg->c0_alt_cfg_val;
        }
    }
    if(int_stat & 0x0002)
    {
        dc1_int_count++;
        if(dc1_int_count%2)
        {
            *dma_int_arg->c1_pri_cfg_ptr = dma_int_arg->c1_pri_cfg_val;
        }
        else
        {
            *dma_int_arg->c1_alt_cfg_ptr = dma_int_arg->c1_alt_cfg_val;
        }
    }
    xthal_dcache_region_writeback(dma_ctrl_data, sizeof(adi_lark_dma_chnl_ctrl_t) * 64);
    *(uint32_t*)REG_DMA_INT_STATUS_CLR_ADDR = int_stat;
    return;
}

int32_t adi_lark_device_ds_i2s_fifo_loopback_test(adi_lark_device_t *device)
{
    #define API_LARK_DS_FIFO_USE_DLINE 0
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_TDSP1);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, 5);                        /* Clear DS int chan from SPT0 */
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_clear_chnl_int_status(device, 6);                        /* Clear DS int chan from SPT1 */
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_set_dline_len(device, &fifo0_info, 64);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_set_dline_len(device, &fifo1_info, 64);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_SPT0_SLOT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_SPT0_SLOT1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_SPT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_SPT0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    #if !API_LARK_DS_FIFO_USE_DLINE
    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    #else
    err = adi_lark_ffsram_enable_delay_line(device, &fifo0_info, true);
    #endif
    LARK_ERROR_RETURN(err);
    #if !API_LARK_DS_FIFO_USE_DLINE
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    #else
    err = adi_lark_ffsram_enable_delay_line(device, &fifo1_info, true);
    #endif
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_ds_i2s_tie_loopback_test(adi_lark_device_t *device)
{
    #define API_LARK_DS_FIFO_USE_DLINE 0
    int32_t  err;
    volatile int32_t loop_count = 0x08000000;
    uint32_t i, irq_id;
    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_TDSP1);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_SPT0);         /* Clear DS int chan from SPT0 */
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_SPT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_SPT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_autoclear_chnl_int_status(device, API_LARK_DS_INT_SPT0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_int_get_soc_irq_id(device, API_LARK_SOC_IRQ_DTSYNC, &irq_id);
    LARK_ERROR_RETURN(err);
    #if LARK_SDK_XOS_TEST_CODE
    err = xos_register_interrupt_handler(irq_id, adi_lark_device_ds_tie_handler, NULL);
    LARK_ERROR_RETURN(err != 0 ? API_LARK_ERROR_REGISTER_ISR : API_LARK_ERROR_OK);
    xos_interrupt_enable(irq_id);
    #else
    _xtos_set_interrupt_handler_arg(irq_id, adi_lark_device_ds_tie_handler, NULL);
    _xtos_interrupt_enable(irq_id);
    #endif
    err = adi_lark_ds_enable_tie_ltif(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_chnl_int(device, API_LARK_DS_INT_SPT0, true);          /* Enable DS Interrupt at last */
    LARK_ERROR_RETURN(err);
    while(loop_count);
    /* The code below will not be executed, just to demonstate how to disable the modules */
    err = adi_lark_ds_enable_chnl_int(device, API_LARK_DS_INT_SPT0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_tie_ltif(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, false);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

void adi_lark_device_ds_tie_handler(void *arg)
{
    uint32_t data;
    data = LARK_LKUPMEMLD(API_LARK_DS_INPUT_SPT0_SLOT0);
    LARK_LKUPMEMST(0, data);
    data = LARK_LKUPMEMLD(API_LARK_DS_INPUT_SPT0_SLOT1);
    LARK_LKUPMEMST(1, data);
}
#endif

#if LARK_SDK_EN_TEST_CODE
int32_t adi_lark_device_datapath_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_mp_set_pin_mode(device, 16, API_LARK_MP_MODE_MCLKO);         /* set MP16 as MCLK out for debug */
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_set_mclk_freq(device, mclk_freq, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_set_sap_mclk_out_rate(device, API_LARK_SAP_MCLK_24576K); /* set MCLKO to 24MHz for debug */
    LARK_ERROR_RETURN(err);

    err = adi_lark_device_sap0_asrci0_fdec0_asrco0_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci0_fint0_fdsp0_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci0_fint0_fdsp0_fdec0_asrco0_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci01_fint01_dsfifo_fdec01_asrco01_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci01_fint01_dsfifo_fdec01_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci01_fint01_dsfifo_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci01_fint01_dsfifo_asrco01_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci01_dsfifo_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci01_dsfifo_asrco01_sap1_test(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_device_sap0_asrci0_fdec0_dsfifo_fint0_fdsp0_eq_asrco01_sap1_test(device);
    LARK_ERROR_RETURN(err);
    #endif   

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_fdec0_asrco0_sap1_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_fdec0_asrco0_sap1 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_96K, API_LARK_FDEC_OUT_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);
    #endif

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_FDEC0);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_sap_select_out_route(device, 0, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 0, 1, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    #endif

    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO1, true);
    LARK_ERROR_RETURN(err);
    #endif

    #ifdef LARK_SDK
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_sap_enable_chnl_output(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_fint0_fdsp0_sap1_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_fint0_fdsp0_sap1 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    sap_config.bclk_pol = API_LARK_SAP_CTRL_BCLK_POL_NEG;
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_power_on(device, true);                                          /* Must be powered on before write parameters*/
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_FINT0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x80000000);                       /* FDSP route from FINT0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);

    #ifdef LARK_SDK
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDSP0);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_sap_select_out_route(device, 0, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDSP0);
    LARK_ERROR_RETURN(err);
    #endif

    err = adi_lark_fdsp_enable_run(device, true);                                              /* FDSP_RUN */
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_sap_enable_chnl_output(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_fint0_fdsp0_fdec0_asrco0_sap1_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_fint0_fdsp0_fdec0_asrco0_sap1 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    sap_config.bclk_pol = API_LARK_SAP_CTRL_BCLK_POL_NEG;
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_FINT0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x80000000);                       /* FDSP route from FINT0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);

    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_96K, API_LARK_FDEC_OUT_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_FDSP0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_FDEC0);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDEC0);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_sap_select_out_route(device, 0, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDEC0);
    LARK_ERROR_RETURN(err);
    #endif

    err = adi_lark_fdsp_enable_run(device, true);                                              /* FDSP_RUN */
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);

    #ifdef LARK_SDK
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    #endif
    #ifdef LARK_LITE_SDK
    err = adi_lark_sap_enable_chnl_output(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_fdec01_asrco01_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_96K, API_LARK_FINT_OUT_FS_192K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 1, API_LARK_FINT_ROUTE_ASRCI1);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_FINT01);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_FINT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_FINT1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_192K, API_LARK_FDEC_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 1, API_LARK_FDEC_ROUTE_TDSP1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_FDEC0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 1, API_LARK_ASRCO_ROUTE_FDEC1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_fdec01_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_96K, API_LARK_FINT_OUT_FS_192K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 1, API_LARK_FINT_ROUTE_ASRCI1);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_FINT01);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_FINT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_FINT1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_192K, API_LARK_FDEC_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 1, API_LARK_FDEC_ROUTE_TDSP1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDEC0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_FDEC1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_96K, API_LARK_FINT_OUT_FS_192K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 1, API_LARK_FINT_ROUTE_ASRCI1);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_FINT01);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_FINT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_FINT1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_TDSP1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci01_dsfifo_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_ASCRI);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_ASRCI0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_ASRCI1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_ASCRI);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_ASCRI);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_TDSP1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_asrco01_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_96K, API_LARK_FINT_OUT_FS_192K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 1, API_LARK_FINT_ROUTE_ASRCI1);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_FINT01);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_FINT0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_FINT1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_FINT01);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_192K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 1, API_LARK_ASRCO_ROUTE_TDSP1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci01_dsfifo_asrco01_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info, fifo1_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_ASCRI);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    fifo1_info.base_offset_of_256B = 8;
    fifo1_info.block_id = 2;
    fifo1_info.channel_id = 1;
    fifo1_info.len_of_256B = 8;
    fifo1_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo1_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_ASRCI0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_src(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, API_LARK_DS_INPUT_ASRCI1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, (1 << 0) | (1 << 1), true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_ASCRI);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 1, API_LARK_DS_RDY2OUT_ASCRI);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_TDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 1, API_LARK_ASRCO_ROUTE_TDSP1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo1_info.block_id * 4 + fifo1_info.channel_id, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo1_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_fdec0_dsfifo_fint0_fdsp0_eq_asrco01_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    adi_lark_ffsram_fifo_info_t fifo0_info;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /*Start SAP config*/
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_96K, API_LARK_FDEC_OUT_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);

    /*start data sync config*/
    err = adi_lark_ds_clear_chnl_int_status(device, API_LARK_DS_INT_FDEC01);
    LARK_ERROR_RETURN(err);

    fifo0_info.base_offset_of_256B = 0;
    fifo0_info.block_id = 2;
    fifo0_info.channel_id = 0;
    fifo0_info.len_of_256B = 8;
    fifo0_info.width = 32;
    err = adi_lark_ffsram_init(device, &fifo0_info);
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_select_fifo_chnl_src(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, API_LARK_DS_INPUT_FDEC0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_fifo_chnl_dst(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_multi_out_chnls_resync(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_select_rdy2out_chnl(device, 0, API_LARK_DS_RDY2OUT_FDEC01);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_TDSP0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_FINT0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x80000000);                       /* FDSP route from FINT0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);

    err = adi_lark_eq_select_route(device, API_LARK_EQ_ROUTE_FDSP_0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_sel_param_ram(device, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_clear_param_ram(device);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE, 0x00010000);                       /* ACC = 0            // 0b01_0000_0000_0000_0000 0x0001_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x04, 0x00024000);                /* Load Din & AAA     // 0b10_0100_0000_0000_0000 0x0002_4000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x08, 0x00008000);                /* ACC = Din * AAA    // 0b00_1000_0000_0000_0000 0x0000_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x0C, 0x00028000);                /* Send ACC to Dout   // 0b10_1000_0000_0000_0000 0x0002_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x10, 0x00038000);                /* Enter sleep        // 0b11_1000_0000_0000_0000 0x0003_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x14, 0x00000000);                /* NOP                // 0b00_0000_0000_0000_0000 0x0000_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE, 0x01000000);                        /* EQ parameter AAA = 0, X00 = 0, p1p0 = 0x0100_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x04, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x08, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x0C, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x10, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x14, 0x01000000);

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_EQ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 1, API_LARK_ASRCO_ROUTE_FDSP0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 1, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_eq_enable_run(device, true);                                                 /* EQ Run on */
    err = adi_lark_fdsp_enable_run(device, true);                                               /* FDSP_RUN */
    LARK_ERROR_RETURN(err);

    err = adi_lark_ds_enable_fifo_chnl_read(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ds_enable_fifo_chnl_write(device, fifo0_info.block_id * 4 + fifo0_info.channel_id, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_ffsram_enable_fifo(device, &fifo0_info, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_adc0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("adc0_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* ADC0: 96KHz, single input */
    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* DAC: 96KHz, from ADC0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_ADC0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on ADC0 and DAC */
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_adc0_fint0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("adc0_fint0_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* ADC0: 48KHz, diff_input */
    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_48KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* FINT0: 48KHz -> 96KHz, from ADC0 */
    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ADC0);
    LARK_ERROR_RETURN(err);

    /* DAC: 96KHz, from FINT0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_FINT0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on ADC0 and DAC */
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_adc0_fdsp0_fint0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("adc0_fdsp0_fint0_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* ADC0: 48KHz, diff_input */
    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_48KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* FDSP0: from ADC0 */
    err = adi_lark_fdsp_enable_power_on(device, true);                                          /* Must be powered on before write parameters*/
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_ADC0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x00);                             /* FDSP route from ADC0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);

    /* FINT0: 48KHz -> 96KHz, from FDSP */
    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_FDSP0);
    LARK_ERROR_RETURN(err);

    /* DAC: 96KHz, from FINT0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_FINT0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on ADC0, FINT0 and DAC, run FDSP */
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_adc0_fint0_fdsp0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("adc0_fint0_fdsp_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* ADC0: 48KHz, diff_input */
    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_48KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* FINT0: 48KHz -> 96KHz, from ADC0 */
    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ADC0);
    LARK_ERROR_RETURN(err);

    /* FDSP0: from FINT0 */
    err = adi_lark_fdsp_enable_power_on(device, true);                                          /* Must be powered on before write parameters*/
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_FINT0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x80000000);                       /* FDSP route from FINT0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);

    /* DAC: 96KHz, from FDSP0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_FDSP0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on ADC0, FINT0 and DAC, run FDSP */
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_eq_dac0_and_adc0_fint0_fdsp0_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    
    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_eq_dac0 and adc0_fint0_fdsp0_sap1 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* SAP0: STEREO, I2S */
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    /* ASRCI0: 96KHz, from SAP0, channel 0 */
    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    /* EQ: from ASRCI0 */
    err = adi_lark_eq_select_route(device, API_LARK_EQ_ROUTE_ASRCI_0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_sel_param_ram(device, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_clear_param_ram(device);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE, 0x00010000);                       /* ACC = 0            // 0b01_0000_0000_0000_0000 0x0001_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x04, 0x00024000);                /* Load Din & AAA     // 0b10_0100_0000_0000_0000 0x0002_4000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x08, 0x00008000);                /* ACC = Din * AAA    // 0b00_1000_0000_0000_0000 0x0000_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x0C, 0x00028000);                /* Send ACC to Dout   // 0b10_1000_0000_0000_0000 0x0002_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x10, 0x00038000);                /* Enter sleep        // 0b11_1000_0000_0000_0000 0x0003_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x14, 0x00000000);                /* NOP                // 0b00_0000_0000_0000_0000 0x0000_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE, 0x01000000);                        /* EQ parameter AAA = 0, X00 = 0, p1p0 = 0x0100_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x04, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x08, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x0C, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x10, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x14, 0x01000000);

    /* DAC: 96KHz, from EQ */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_EQ);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* ADC0: 48KHz, diff_input */
    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_48KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* FINT0: 48KHz -> 96KHz, from ADC0 */
    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ADC0);
    LARK_ERROR_RETURN(err);

    /* FDSP0: from FINT0 */
    err = adi_lark_fdsp_enable_power_on(device, true);                                          /* Must be powered on before write parameters*/
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_FINT0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x80000000);                       /* FDSP route from FINT0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);
    
    /* SAP1: From FDSP0 */
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDSP0);
    LARK_ERROR_RETURN(err);
    
    /* Start Run EQ */
    err = adi_lark_eq_enable_run(device, true);
    LARK_ERROR_RETURN(err);

    /* Power on SAP0, ASRCI0, and DAC */
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    /* Power on ADC0, FINT0, run FDSP */
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, true);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* SAP0: STEREO, I2S */
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    /* ASRCI0: 96KHz, from SAP0, channel 0 */
    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    #ifdef LARK_SDK
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);
    #endif

    /* DAC: 96KHz, from ASRCI0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on SAP0, ASRCI0, and DAC */
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_eq_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_eq_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* SAP0: STEREO, I2S */
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    /* ASRCI0: 96KHz, from SAP0, channel 0 */
    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    #ifdef LARK_SDK
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);
    #endif

    /* EQ: from ASRCI0 */
    err = adi_lark_eq_select_route(device, API_LARK_EQ_ROUTE_ASRCI_0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_sel_param_ram(device, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_clear_param_ram(device);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE, 0x00010000);                       /* ACC = 0            // 0b01_0000_0000_0000_0000 0x0001_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x04, 0x00024000);                /* Load Din & AAA     // 0b10_0100_0000_0000_0000 0x0002_4000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x08, 0x00008000);                /* ACC = Din * AAA    // 0b00_1000_0000_0000_0000 0x0000_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x0C, 0x00028000);                /* Send ACC to Dout   // 0b10_1000_0000_0000_0000 0x0002_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x10, 0x00038000);                /* Enter sleep        // 0b11_1000_0000_0000_0000 0x0003_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x14, 0x00000000);                /* NOP                // 0b00_0000_0000_0000_0000 0x0000_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE, 0x01000000);                        /* EQ parameter AAA = 0, X00 = 0, p1p0 = 0x0100_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x04, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x08, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x0C, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x10, 0x01000000);
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x14, 0x01000000);

    /* DAC: 96KHz, from EQ */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_EQ);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Start Run EQ */
    err = adi_lark_eq_enable_run(device, true);
    LARK_ERROR_RETURN(err);

    /* Power on SAP0, ASRCI0, and DAC */
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_fint0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("sap0_asrci0_fint0_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    /* SAP0: STEREO, I2S */
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    /* ASRCI0: 96KHz, from SAP0, channel 0 */
    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    #ifdef LARK_SDK
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);
    #endif

    /* FINT0: 48KHz -> 96KHz, from ASRCI0 */
    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_ASRCI0);
    LARK_ERROR_RETURN(err);

    /* DAC: 96KHz, from FDSP */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_FINT0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on SAP0, ASRCI0, FINT0 and DAC */
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_dmic0_fdec0_asrco0_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("dmic0_fdec0_asrco0_sap1 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);

    /* DMIC0: 6144KHz, 96KHz */
    err = adi_lark_clk_set_dmic_clk_rate(device, API_LARK_DMIC_CLK_RATE_6144KHZ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_out_sample_rate(device, 0, API_LARK_DMIC_OUT_SAMPLE_RATE_96KHZ);
    LARK_ERROR_RETURN(err);

    /* FDEC: 96KHz to 48KHz, from DMIC0 */
    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_96K, API_LARK_FDEC_OUT_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_DMIC0);
    LARK_ERROR_RETURN(err);

    /* ASRCO0: 48KHz, from FDEC0 */
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_FDEC0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_sai(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_48K);
    LARK_ERROR_RETURN(err);

    /* SAP1: STEREO, I2S, from ASRCO0 */
    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);

    /* Power on DMIC, FDEC, ASRCO and SAP1 */
    err = adi_lark_dmic_enable_channel_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_clock_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_dmic1_fint0_dac_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("dmic1_fint0_dac test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);

    /* DMIC1: 6144KHz, 96KHz, MP24 */
    err = adi_lark_clk_set_dmic_clk_rate(device, API_LARK_DMIC_CLK_RATE_6144KHZ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_out_sample_rate(device, 1, API_LARK_DMIC_OUT_SAMPLE_RATE_96KHZ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_mp_set_pin_mode(device, 24, API_LARK_MP_MODE_DMIC_DATA1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_dmic1_from_mp24(device, true);
    LARK_ERROR_RETURN(err);
    
    /* FINT: 96KHz to 196KHz, from DMIC1 */
    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_96K, API_LARK_FINT_OUT_FS_192K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_DMIC1);
    LARK_ERROR_RETURN(err);

    /* DAC: 96KHz, from ADC0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_192KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_FINT0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);
    
    /* Power on DMIC, FDEC, ASRCO and SAP1 */
    err = adi_lark_dmic_enable_channel_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_clock_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_dmic0_pdm_sap0_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    adi_lark_sap_config_t sap_config =
    {
        .sai_mode = 0,
        .data_format = 2,
        .slot_width = 0,
        .tri_state = 1,
        .data_width = 1,
        .bclk_src = 1,
        .bclk_pol =0,
        .lrclk_src =1,
        .lrclk_pol =0
    };

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_pmu_set_chip_power_mode(device, API_LARK_PWR_MODE_ACTIVE);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pmu_enable_master_block(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pmu_enable_cm_pin_fast_charge(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_clk_enable_xtal_power_on(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_config_pll(device, API_LARK_CLK_PLL_SOURCE_MCLKIN, API_LARK_CLK_PLL_TYPE_INTEGER, API_LARK_CLK_SYNC_SOURCE_INTERNAL, 0x00, 0x02, 0x00, 0x00);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_update_pll(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_power_on(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_clk_set_mclk_freq(device, API_LARK_MCLK_FREQ_49P152, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_set_sap_mclk_out_rate(device, API_LARK_SAP_MCLK_24576K);
    LARK_ERROR_RETURN(err);

    err = adi_lark_mp_set_pin_mode(device, 16, API_LARK_MP_MODE_MCLKO);
    LARK_ERROR_RETURN(err);
    err = adi_lark_mp_set_pin_mode(device, 17, API_LARK_MP_MODE_PDM_CLK);
    LARK_ERROR_RETURN(err);
    err = adi_lark_mp_set_pin_mode(device, 18, API_LARK_MP_MODE_PDM_DATA);
    LARK_ERROR_RETURN(err);

    err = adi_lark_pdm_set_sample_rate(device, API_LARK_PDM_SAMPLE_RATE_48KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_set_output_rate(device, API_LARK_PDM_OUTPUT_RATE_6144KHz);
    LARK_ERROR_RETURN(err);

    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_enable_mute(device, 0, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_enable_mute(device, 1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_select_input_route(device, 0, API_LARK_PDM_ROUTE_DMIC0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_select_input_route(device, 1, API_LARK_PDM_ROUTE_DMIC0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_pdm_enable_power_on(device, 0, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pdm_enable_power_on(device, 1, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 0, 0, API_LARK_SAP_OUT_ROUTE_FROM_DMIC0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_clk_set_dmic_clk_rate(device, API_LARK_DMIC_CLK_RATE_6144KHZ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_out_sample_rate(device, 0, API_LARK_DMIC_OUT_SAMPLE_RATE_48KHZ);
    LARK_ERROR_RETURN(err);

    err = adi_lark_dmic_enable_channel_power_on(device, 0, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_clock_power_on(device, 0);
    LARK_ERROR_RETURN(err);

    err =adi_lark_sap_enable_chnl_output(device, 0, true);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_dmic0_fint0_fdsp0_sap1_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("dmic0_fint0_fdsp0_sap1 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    
    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    sap_config.bclk_pol = API_LARK_SAP_CTRL_BCLK_POL_NEG;

    err = adi_lark_clk_set_dmic_clk_rate(device, API_LARK_DMIC_CLK_RATE_6144KHZ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_out_sample_rate(device, 0, API_LARK_DMIC_OUT_SAMPLE_RATE_96KHZ);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_out_sample_rate(device, 1, API_LARK_DMIC_OUT_SAMPLE_RATE_96KHZ);
    LARK_ERROR_RETURN(err);

    err = adi_lark_dmic_set_channel_filter(device, 0, API_LARK_DMIC_DEC_ORDER_4TH, API_LARK_DMIC_HPF_CUTOFF_FREQ_OFF, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_filter(device, 1, API_LARK_DMIC_DEC_ORDER_4TH, API_LARK_DMIC_HPF_CUTOFF_FREQ_OFF, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_channel_mute(device, 0, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_channel_mute(device, 1, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_volume(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_set_channel_volume(device, 1, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_DMIC0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_power_on(device, true);                                          /* Must be powered on before write parameters*/
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_FINT0_1, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move to channel 0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x80000000);                       /* FDSP route from FINT0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00);
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00);

    err = adi_lark_sap_config_all(device, 1, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_select_out_route(device, 1, 0, API_LARK_SAP_OUT_ROUTE_FROM_FDSP0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_run(device, true);                                              /* FDSP_RUN */
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_output(device, 1, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_dmic_enable_channel_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_channel_power_on(device, 1, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dmic_enable_clock_power_on(device, true);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_sap0_asrci0_fdsp0_fint0_dac0_adc0_eq_fdec0_asrco0_sap0_test(adi_lark_device_t *device)
{
    int32_t err;

    adi_lark_sap_config_t sap_config;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_init_config_as_i2sdefault(device, &sap_config);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_config_all(device, 0, &sap_config);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrci_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_route(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrci_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_set_rate(device, API_LARK_FDSP_RATE_SRC_ASRCI, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE, 0x04800000);                     /* move (opcode d18) to channel 0 */
                                                                                                /* bit 22~26 for opcode, bit 9~14 for dest chnl*/
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x04, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PROGRAM_MEM_BASE + 0x08, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE, 0x18000000);                       /* FDSP route from ASCRI0 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x04, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + 0x08, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN, 0x00);      /* FDSP Bank A Para1 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x04, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN + 0x08, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 , 0x00); /* FDSP Bank A Para2 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x04, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 2 + 0x08, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3, 0x00);  /* FDSP Bank A Para3 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x04, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 3 + 0x08, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4, 0x00);  /* FDSP Bank A Para4 */
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x04, 0x00); 
    adi_lark_hal_reg_write(device, LARK_FDSP_PARAM_MEM_BASE + LARK_FDSP_PARAM_SPAN * 4 + 0x08, 0x00); 

    err = adi_lark_fint_select_pair_chnls_fs(device, API_LARK_FINT_PAIR_CHNL_0_1, API_LARK_FINT_IN_FS_48K, API_LARK_FINT_OUT_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fint_select_chnl_route(device, 0, API_LARK_FINT_ROUTE_FDSP0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_FINT0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_eq_select_route(device, API_LARK_EQ_ROUTE_ADC_0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_enable_run(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_sel_param_ram(device, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_eq_clear_param_ram(device);
    LARK_ERROR_RETURN(err);
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE, 0x00010000);                       /* ACC = 0            // 0b01_0000_0000_0000_0000 0x0001_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x04, 0x00024000);                /* Load Din & AAA     // 0b10_0100_0000_0000_0000 0x0002_4000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x08, 0x00008000);                /* ACC = Din * AAA    // 0b00_1000_0000_0000_0000 0x0000_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x0C, 0x00028000);                /* Send ACC to Dout   // 0b10_1000_0000_0000_0000 0x0002_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x10, 0x00038000);                /* Enter sleep        // 0b11_1000_0000_0000_0000 0x0003_8000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PROGRAM_MEM_BASE + 0x14, 0x00000000);                /* NOP                // 0b00_0000_0000_0000_0000 0x0000_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE, 0x01000000);                        /* EQ parameter AAA = 0, X00 = 0, p1p0 = 0x0100_0000 */
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x04, 0x01000000); 
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x08, 0x01000000); 
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x0C, 0x01000000); 
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x10, 0x01000000); 
    adi_lark_hal_reg_write(device, LARK_EQ_PARAM0_MEM_BASE + 0x14, 0x01000000); 

    err = adi_lark_fdec_select_pair_chnls_fs(device, API_LARK_FDEC_PAIR_CHNL_0_1, API_LARK_FDEC_IN_FS_96K, API_LARK_FDEC_OUT_FS_48K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_select_chnl_route(device, 0, API_LARK_FDEC_ROUTE_EQ);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_select_asrco_fs(device, API_LARK_ASRC_FS_96K);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_route(device, 0, API_LARK_ASRCO_ROUTE_FDEC0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_select_asrco_sai(device, 0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_select_out_route(device, 0, 0, API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0);
    LARK_ERROR_RETURN(err);

    err = adi_lark_eq_enable_run(device, true);                                                 /* EQ Run on */
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, true);                                               /* FDSP_RUN */
    LARK_ERROR_RETURN(err);

    err = adi_lark_fint_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdec_enable_chnl_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCI0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_asrc_enable_chnl_power_on(device, API_LARK_ASRC_CHNL_ASRCO0, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_sap_enable_chnl_output(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_sap_enable_chnl_input(device, 0, true);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_pllen_adc0_dac0_test(adi_lark_device_t *device)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_LOG_MSG("adc0_dac0 test start ... ");

    err = adi_lark_device_init_adp_for_test(device);
    LARK_ERROR_RETURN(err);
    /*
     * Steps of enable PLL
     * 1.    Ensure that POWER_EN=1 (Suppose it has been enabled outside this function)
     * 2.    Ensure that PLL_EN =0.
     * 3.    Set the PLL control registers correctly.
     * 4.    Write 1 to PLL_UPDATE bit to propagate the PLL settings.
     * 5.    Enable the PLL using the PLL_EN bit.
     */
    err = adi_lark_clk_enable_pll_power_on(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_bypass(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_config_pll(device, API_LARK_CLK_PLL_SOURCE_MCLKIN, API_LARK_CLK_PLL_TYPE_INTEGER, API_LARK_CLK_SYNC_SOURCE_INTERNAL, 0x01, 0x02, 0x00, 0x00);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_update_pll(device);
    LARK_ERROR_RETURN(err);

    /* ADC0: 96KHz, single input */
    err = adi_lark_adc_set_sample_rate(device, 0, API_LARK_ADC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_adc_set_input_mode(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* DAC: 96KHz, from ADC0 */
    err = adi_lark_dac_set_sample_rate(device, API_LARK_DAC_SAMPLE_RATE_96KHz);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_select_input_route(device, API_LARK_DAC_ROUTE_ADC0);
    LARK_ERROR_RETURN(err);

    /* HPAMP: headphone mode*/
    err = adi_lark_hpamp_set_output_mode(device, API_LARK_HPAMP_HEADPHONE);
    LARK_ERROR_RETURN(err);

    /* Power on ADC0 and DAC */
    err = adi_lark_adc_enable_power_on(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_enable_mute(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dac_set_volume(device, 0x34);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_device_cp_test(adi_lark_device_t *device)
{
    int32_t  err;
    uint32_t test_addr, test_addr_start, test_addr_stop;
    uint32_t i, j, reg_rd_data;
    uint8_t  mem_rd_data[16], mem_wr_data[16];

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    LARK_LOG_MSG("Control port write/read memery test start ... ");
    err = adi_lark_pmu_set_chip_power_mode(device, API_LARK_PWR_MODE_ACTIVE);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pmu_enable_master_block(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_pmu_enable_cm_pin_fast_charge(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_clk_enable_xtal_power_on(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_config_pll(device, API_LARK_CLK_PLL_SOURCE_MCLKIN, API_LARK_CLK_PLL_TYPE_INTEGER, API_LARK_CLK_SYNC_SOURCE_INTERNAL, 0x00, 0x02, 0x00, 0x00);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_update_pll(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_power_on(device, 1);
    LARK_ERROR_RETURN(err);

    err = adi_lark_fdsp_enable_power_on(device, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_fdsp_enable_run(device, 0);
    LARK_ERROR_RETURN(err);

    for (i = 0; i < 3; i ++)
    {
        switch(i)
        {
            case 0:
            {
                test_addr_start = LARK_FDSP_PROGRAM_MEM_BASE;
                test_addr_stop = LARK_FDSP_PARAM_MEM_BASE - 1;
                break;
            }
            case 1:
            {
                test_addr_start = LARK_FDSP_PARAM_MEM_BASE;
                test_addr_stop = LARK_8BIT_REG_ADDR_START - 1;
                break;
            }
            case 2:
            {
                test_addr_start = LARK_FDSP_STATE_MEM_BASE;
                test_addr_stop = LARK_FDSP_STATE_MEM_BASE + 0x3FF;
                break;
            }
        }

        /* r/w 4 bytes a time by reg r/w hal api */
        for (test_addr = test_addr_start; test_addr < test_addr_stop; test_addr += 4)
        {
            err = adi_lark_hal_reg_write(device, test_addr, 0xaa55aa55);
            LARK_ERROR_RETURN(err);
        }
        for (test_addr = test_addr_start; test_addr < test_addr_stop; test_addr += 4)
        {
            err = adi_lark_hal_reg_read(device, test_addr, &reg_rd_data);
            LARK_ERROR_RETURN(err);
            if(reg_rd_data != 0xaa55aa55)
            {
                LARK_LOG_ERR("control port write/read error\n");
                return API_LARK_ERROR_ERROR;
            }
        }

        /* r/w 16bytes buffer by mem r/w hal api */
        for(test_addr = test_addr_start; test_addr < test_addr_stop; test_addr_stop += 16)
        {
            for (j = 0; j < 16; j ++)
            {
                mem_rd_data[j] = 0x00;
                mem_wr_data[j] = 0xa5;
            }
            err = adi_lark_hal_mem_write(device, test_addr, mem_wr_data, 16);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_mem_read(device, test_addr, mem_rd_data, 16);
            LARK_ERROR_RETURN(err);
            for (j = 0; j < 16; j ++)
            {
                if ( mem_rd_data[j] != mem_wr_data[j])
                {
                    LARK_LOG_ERR("control port write/read error\n");
                    return API_LARK_ERROR_ERROR;
                }
            }
        }
    }

    return API_LARK_ERROR_OK;
}
#endif

/*! @} */
