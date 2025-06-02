/*!
 * @brief     Clock APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */

/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E C L A R E S ============*/
uint32_t gcd(uint32_t m, uint32_t n);

/*============= C O D E ====================*/
int32_t adi_lark_clk_enable_xtal_power_on(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_XTAL_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_select_xtal_mode(adi_lark_device_t *device, adi_lark_clk_xtal_mode_e mode)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_CLK_XTAL_MODE_XTAL);

    err = adi_lark_hal_bf_write(device, 0x4000C010, 0x00000103, mode);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_enable_pll_power_on(adi_lark_device_t *device, bool enable)
{
    int32_t  err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_PLL_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_config_pll(adi_lark_device_t *device, adi_lark_clk_pll_source_e pll_src, adi_lark_clk_pll_type_e type, 
    adi_lark_clk_sync_source_e sync_src, uint8_t prescaler, uint16_t multiplier, uint16_t numerator, uint16_t denominator)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    #ifdef LARK_SDK
    LARK_INVALID_PARAM_RETURN(pll_src > API_LARK_CLK_PLL_SOURCE_BCLK1);
    #endif
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(pll_src > API_LARK_CLK_PLL_SOURCE_RCOSC);
    #endif
    LARK_INVALID_PARAM_RETURN(type      > API_LARK_CLK_PLL_TYPE_FRAC);
    LARK_INVALID_PARAM_RETURN(sync_src  > API_LARK_CLK_SYNC_SOURCE_INTERNAL);
    LARK_INVALID_PARAM_RETURN(prescaler > 7);

    err = adi_lark_hal_bf_write(device, BF_PLL_SOURCE_INFO, pll_src);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_TYPE_INFO, type);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_SYNC_SOURCE_INFO, sync_src);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_INPUT_PRESCALER_INFO, prescaler);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_INTEGER_DIVIDER_INFO, multiplier);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_NUMERATOR_INFO, numerator);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_DENOMINATOR_INFO, denominator);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_update_pll(adi_lark_device_t *device)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_PLL_UPDATE_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_UPDATE_INFO, 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_get_pll_locked_status(adi_lark_device_t *device, uint8_t *locked)
{
    int32_t  err;
    uint32_t pll_locked;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(locked);

    err = adi_lark_hal_bf_read(device, BF_PLL_LOCK_INFO, &pll_locked);
    LARK_ERROR_RETURN(err);
    *locked = pll_locked & 0x01;

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_enable_2x_output(adi_lark_device_t *device, bool enable)
{
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_FREQ_MULT_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_get_2x_locked_status(adi_lark_device_t *device, uint8_t *locked)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t pll_locked;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(locked);

    err = adi_lark_hal_bf_read(device, BF_FM_CLK_READY_INFO, &pll_locked);
    LARK_ERROR_RETURN(err);
    *locked = pll_locked & 0x01;
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_get_asrc_locked_status(adi_lark_device_t *device, uint8_t *asrci_locked, uint8_t *asrco_locked)
{
    int32_t  err;
    uint32_t pll_locked;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(asrci_locked);
    LARK_NULL_POINTER_RETURN(asrco_locked);

    err = adi_lark_hal_bf_read(device, BF_ASRCI_LOCK_INFO, &pll_locked);
    LARK_ERROR_RETURN(err);
    *asrci_locked = pll_locked & 0x01;
    err = adi_lark_hal_bf_read(device, BF_ASRCO_LOCK_INFO, &pll_locked);
    LARK_ERROR_RETURN(err);
    *asrco_locked = pll_locked & 0x01;

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_tdsp_bus_rate(adi_lark_device_t *device, adi_lark_tdsp_clk_rate_e tdsp_rate, adi_lark_bus_clk_rate_e bus_rate)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, REG_CLK_CTRL10_ADDR, 0x00000600, (bus_rate << 3) + tdsp_rate);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_mclk_freq(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, bool bypass_pll)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t ctrl13_val;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mclk_freq > API_LARK_MCLK_FREQ_98P304);

    err = adi_lark_hal_reg_read(device, REG_CLK_CTRL13_ADDR, &ctrl13_val);
    LARK_ERROR_RETURN(err);
    ctrl13_val = ctrl13_val & 0x7C;
    ctrl13_val = ctrl13_val | (mclk_freq | ((bypass_pll ? 1 : 0) << 7));
    ctrl13_val = ctrl13_val | 0x40;
    err = adi_lark_hal_reg_write(device, REG_CLK_CTRL13_ADDR, ctrl13_val);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_enable_pll_bypass(adi_lark_device_t *device, bool bypass_pll)
{
    #ifdef LARK_LITE_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    err = adi_lark_hal_bf_write(device, BF_PLL_FM_BYPASS_INFO, bypass_pll ? 1 : 0) ;
    LARK_ERROR_RETURN(err);
    #else
    (void)device;
    (void)bypass_pll;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_ffsram_clk_rate(adi_lark_device_t *device, adi_lark_ffsram_clk_rate_e clk_rate)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(clk_rate > 3);

    err = adi_lark_hal_bf_write(device, BF_FFSRAM_RATE_INFO, clk_rate);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_uart_clk_rate(adi_lark_device_t *device, adi_lark_uart_clk_rate_e clk_rate)
{
    #ifdef LARK_SDK
    int32_t err;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_UART_CTRL_CLK_RATE_INFO, clk_rate);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_enable_adp_freq_div3(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_ADC_FREQ_INFO, enable ? 0 : 1);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_enable_adp_blocks_clk(adi_lark_device_t *device, uint16_t mask, bool enable)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t current_mask;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_read(device, BF_ADP_CLOCK_EN_INFO, &current_mask);
    LARK_ERROR_RETURN(err);
    current_mask &= ~mask;
    current_mask |= (enable) ? mask : 0x0000;
    err = adi_lark_hal_bf_write(device, BF_ADP_CLOCK_EN_INFO, current_mask);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_qspi_clk_rate(adi_lark_device_t *device, uint16_t clk_rate)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_CLK_RATIO_INFO, clk_rate); /* spi clock = sclk / (rate + 1) */
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_dmic_clk_rate(adi_lark_device_t *device, adi_lark_dmic_clk_rate_e clk_rate)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(clk_rate > API_LARK_DMIC_CLK_RATE_6144KHZ);
 
    err = adi_lark_hal_bf_write(device, BF_DMIC_CLK_RATE_INFO, clk_rate);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_dmic_clk1_rate(adi_lark_device_t *device, adi_lark_dmic_clk_rate_e clk1_rate)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(clk1_rate > API_LARK_DMIC_CLK_RATE_6144KHZ);
 
    err = adi_lark_hal_bf_write(device, BF_DMIC_CLK1_RATE_INFO, clk1_rate);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_set_sap_mclk_out_rate(adi_lark_device_t *device, adi_lark_sap_mclk_rate_e rate)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(rate >= API_LARK_SAP_MCLK_CNT);
    
    err = adi_lark_hal_bf_write(device, BF_MCLKO_RATE_INFO, rate);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}

uint32_t gcd(uint32_t m, uint32_t n)
{
    uint32_t c;
    while(m > 0)
    {
        c = n % m;
        n = m;
        m = c;
    }
    return n;
}

int32_t adi_lark_clk_startup_pll(adi_lark_device_t *device, adi_lark_clk_xtal_mode_e xtal_mode, adi_lark_clk_pll_source_e pll_src,
    adi_lark_clk_sync_source_e sync_src, uint32_t input_freq_hz, uint32_t output_freq_hz)
{
    #ifdef  LARK_SDK
    #define LARK_MAX_PLL_INPUT (36000000)
    #endif
    #ifdef  LARK_LITE_SDK
    #define LARK_MAX_PLL_INPUT (27000000)
    #endif
    #define LARK_MIN_PLL_INPUT (30000)
    #define LARK_MAX_PLL_DIV   (3072)

    int32_t  err;
    uint16_t multiplier = 1, best_mul = 1, numerator = 0, denominator = 65535;
    uint32_t diff, least_diff = output_freq_hz, gcd_val, best_den = 65535, best_num = 0, product_freq_hz;
    uint8_t  prescaler = 1, best_pre = 1;
    adi_lark_clk_pll_type_e pll_type = API_LARK_CLK_PLL_TYPE_INTEGER, best_type = API_LARK_CLK_PLL_TYPE_INTEGER;
    double ratio, rate, fract;

    #ifdef  LARK_SDK
    LARK_INVALID_PARAM_RETURN(output_freq_hz != 24576000 && output_freq_hz != 49152000 && output_freq_hz != 73728000 && output_freq_hz != 98304000);
    #endif
    #ifdef  LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(output_freq_hz != 24576000);
    #endif

    for(prescaler = 1; prescaler < 8; prescaler++)
    {
        multiplier = output_freq_hz * prescaler / input_freq_hz;
        if((prescaler * output_freq_hz == input_freq_hz * multiplier) && 
           (prescaler * LARK_MAX_PLL_INPUT >= input_freq_hz) && 
           (prescaler * LARK_MIN_PLL_INPUT <= input_freq_hz) && 
           (multiplier > 1) && (multiplier <= LARK_MAX_PLL_DIV))
        {
            pll_type = API_LARK_CLK_PLL_TYPE_INTEGER;
            least_diff = 0;
            break;
        }
    }
    if(prescaler == 8)
    {
        ratio = (double)output_freq_hz/input_freq_hz;
        for(best_pre = 1; best_pre < 8; best_pre++)
        {
            rate = ratio * best_pre;
            best_mul = (uint32_t)rate;
            if((best_pre * LARK_MAX_PLL_INPUT >= input_freq_hz) &&
            (best_pre * LARK_MIN_PLL_INPUT <= input_freq_hz) &&
            (best_mul > 1) && (best_mul <= LARK_MAX_PLL_DIV))
            {
                fract = rate - best_mul;
    
                if(fract < 0.1)
                {
                    best_type = API_LARK_CLK_PLL_TYPE_INTEGER;
                    best_num = 0;
                }
                else if(fract > 0.9)
                {
                    best_type = API_LARK_CLK_PLL_TYPE_INTEGER;
                    best_num = 0;
                    best_mul += 1;
                }
                else
                {
                    best_type = API_LARK_CLK_PLL_TYPE_FRAC;
                    best_den = best_pre * input_freq_hz;
                    best_num = best_den * fract;
                }
                product_freq_hz = best_mul * input_freq_hz / best_pre;
                product_freq_hz += (uint32_t)((double)best_num / best_den * input_freq_hz / best_pre);
                if(product_freq_hz > output_freq_hz)
                    diff = product_freq_hz - output_freq_hz;
                else
                    diff = output_freq_hz - product_freq_hz;
                if(diff < least_diff)
                {
                    least_diff = diff;
                    pll_type = best_type;
                    gcd_val = gcd(best_num, best_den);
                    best_num /= gcd_val;
                    best_den /= gcd_val;
                    if(best_den > 65535)
                    {
                        rate = (double)65535/best_den;
                        denominator = 65535;
                        numerator = (uint16_t)(rate * best_num);
                    }
                    else
                    {
                        numerator = (uint16_t)best_num;
                        denominator = (uint16_t)best_den;
                    }
                    multiplier = (uint16_t)best_mul;
                    prescaler = best_pre;
                }
            }
        }

    }
    if(least_diff == output_freq_hz)
    {
        LARK_LOG_ERR("unable to find valid configuration for specified input and output freq of PLL.");
        return API_LARK_ERROR_INVALID_PARAM;
    }
    err = adi_lark_clk_set_mclk_freq(device, API_LARK_MCLK_FREQ_98P304, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_power_on(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_xtal_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_select_xtal_mode(device, xtal_mode);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_config_pll(device, pll_src, pll_type, sync_src, prescaler, multiplier, numerator, denominator);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_power_on(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_update_pll(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_sel_ext_osc(adi_lark_device_t *device)
{
    #ifdef  LARK_LITE_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_RCOSC_PD_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_RCOSC_EN_INFO, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_MCLKIN_EN_INFO, 1);
    LARK_ERROR_RETURN(err);
    #else
    (void)device;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_sel_int_osc(adi_lark_device_t *device)
{
    #ifdef  LARK_LITE_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_RCOSC_PD_INFO, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_RCOSC_EN_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_PLL_SOURCE_INFO, API_LARK_CLK_PLL_SOURCE_RCOSC);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_MCLKIN_EN_INFO, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_enable_pll_bypass(device, true);
    LARK_ERROR_RETURN(err);
    #else
    (void)device;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_start_osc_cal(adi_lark_device_t *device, adi_lark_clk_osc_cal_ref_sel_e refclk_src, uint8_t tasks, uint8_t jobs)
{
    #ifdef  LARK_LITE_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(refclk_src > API_LARK_CLK_OSC_CAL_REF_16K);
    
    err = adi_lark_hal_bf_write(device, BF_RCOSC_TASK_TH_INFO, tasks);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_RCOSC_JOB_TH_INFO, jobs);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_RCOSC_DISTRIM_INFO, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_ASRCI_OSCCAL_LOCK_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_ASRCO_OSCCAL_LOCK_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_RCOSC_REFSEL_INFO, refclk_src);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_RCOSC_STARTCAL_INFO, 1);
    LARK_ERROR_RETURN(err);
    #else
    (void)device;
    (void)refclk_src;
    (void)tasks;
    (void)jobs;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_clk_get_osc_cal_freq_err(adi_lark_device_t *device, int32_t *freq_err)
{
    #ifdef  LARK_LITE_SDK
    int32_t err;
    uint32_t rcosc_status = 1;
    uint32_t timeout = 0x1000;
    uint32_t data;
    int32_t freq;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    while ((rcosc_status > 0) && (timeout > 0))
    {  
        err = adi_lark_hal_bf_read(device, BF_RCOSC_STATUS_INFO, &rcosc_status);
        LARK_ERROR_RETURN(err);
        timeout--;
    }
    if (timeout == 0)
    {
        return API_LARK_ERROR_OSC_CAL_NOT_DONE;
    }
    err = adi_lark_hal_bf_read(device, BF_RCOSC_FERR_INFO, &data);
    LARK_ERROR_RETURN(err);
    freq = ((data & 0x2000) > 0) ? (data - 0x4000) : data;
    freq = (freq * 24576000) / 8192;
    *freq_err = freq;
    #else
    (void)device;
    (void)freq_err;
    #endif
    return API_LARK_ERROR_OK;
}

/*! @} */
