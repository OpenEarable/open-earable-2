/*!
 * @brief     UART APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E C L A R E S ============*/
int32_t adi_lark_uart_select_parity_type(adi_lark_device_t *device, adi_lark_uart_parity_type_e parity);
int32_t adi_lark_uart_set_baudrate(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, uint32_t baudrate);
int32_t adi_lark_uart_get_tx_buff_empty_status(adi_lark_device_t *device, uint8_t *empty);
int32_t adi_lark_uart_get_rx_buff_full_status(adi_lark_device_t *device, uint8_t *full);

/*============= D E F I N E S ==============*/
#ifdef  LARK_SDK
#define LARK_FIXED_INT2Q(x, fb)                ((uint32_t)((x) << (fb)))
#define LARK_FIXED_QMUL(x, y, xFb, yFb, resFb) ((uint32_t)(((uint64_t)(x) * (uint64_t)(y))>>((xFb) + (yFb)- (resFb))))
#define LARK_FIXED_QDIV(x, y, xFb, yFb, resFb) ((uint32_t)((((uint64_t)(x)) << ((resFb) + (yFb) - (xFb))) / (y)))
#endif

/*============= C O D E ====================*/
int32_t adi_lark_uart_select_parity_type(adi_lark_device_t *device, adi_lark_uart_parity_type_e parity)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);

    switch (parity)
    {
        case API_LARK_UART_PARITY_NONE:
            err = adi_lark_hal_bf_write(device, BF_UART_PEN_INFO, 0);
            LARK_ERROR_RETURN(err);
            break;
        case API_LARK_UART_PARITY_ODD:
            err = adi_lark_hal_bf_write(device, BF_UART_PEN_INFO, 1);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_bf_write(device, BF_UART_EPS_INFO, 0);
            LARK_ERROR_RETURN(err);
            break;
        case API_LARK_UART_PARITY_EVEN:
            err = adi_lark_hal_bf_write(device, BF_UART_PEN_INFO, 1);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_bf_write(device, BF_UART_EPS_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        default:
            err = API_LARK_ERROR_INVALID_PARAM;
    }
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_set_baudrate(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, uint32_t baudrate)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t pclk, clk_ctrl10;
    uint8_t apb_div;
    uint8_t apb_div_lookup[8] = {1, 2, 3, 4, 6, 8, 12, 16};
    uint8_t osr_lookup[4] = {4, 8, 16, 32};
    uint8_t osr = 0, div_m = 0, div_m_index, get_golden_param_flag;
    uint16_t div = 0, div_n = 0, div_index, div_n_index;
    uint32_t m_q8, n_q8, divider_q8, baudrate_q8, cal_baudrate_q8, error_q8, error_min_q8;
    int8_t osr_index;
    LARK_NULL_POINTER_RETURN(device);
    
    err = adi_lark_hal_reg_read(device, REG_CLK_CTRL10_ADDR, &clk_ctrl10);
    LARK_ERROR_RETURN(err);
    apb_div = apb_div_lookup[(clk_ctrl10 >> 3) & 0x07];
    pclk = (24576000 * (mclk_freq + 1)) / apb_div;
    
    baudrate_q8 = LARK_FIXED_INT2Q(baudrate, 8);
    error_min_q8 = 0xFFFFFFFF;
    get_golden_param_flag = 0;

    for (osr_index = 0; osr_index <= 3; osr_index++)
    {
        div_index = pclk / (osr_lookup[osr_index] * baudrate);
        for (; div_index >= 1; div_index--)
        {
            for (div_m_index = 1; div_m_index <= 3; div_m_index++)
            {
                for (div_n_index = 0; div_n_index <= 2047; div_n_index++)
                {
                    divider_q8 = LARK_FIXED_INT2Q(osr_lookup[osr_index] * div_index, 8);
                    m_q8 = LARK_FIXED_INT2Q(div_m_index, 8);
                    n_q8 = LARK_FIXED_QDIV(div_n_index, 2048, 0, 0, 8);
                    divider_q8 = LARK_FIXED_QMUL(m_q8 + n_q8, divider_q8, 8, 8, 8);
                    
                    cal_baudrate_q8 = LARK_FIXED_QDIV(pclk, divider_q8, 0, 8, 8);
                    error_q8 = (cal_baudrate_q8 > baudrate_q8) ? cal_baudrate_q8 - baudrate_q8 : baudrate_q8 - cal_baudrate_q8;
    
                    if (error_q8 < error_min_q8)
                    {
                        osr = osr_index;
                        div = div_index;
                        div_m = div_m_index;
                        div_n = div_n_index;
                        error_min_q8 = error_q8;
                        /* if error of bandrate is lower than 0.01%, quit calculation to save time*/
                        if (error_q8 <= LARK_FIXED_QDIV(baudrate_q8, 1000, 8, 0, 8))
                        {
                            get_golden_param_flag = 1;
                            break;
                        }
                    }
                }
                if (get_golden_param_flag == 1)
                {
                    break;
                }
            }
            if (get_golden_param_flag == 1)
            {
                break;
            }
        }
        if (get_golden_param_flag == 1)
        {
            break;
        }
    }

    err = adi_lark_hal_bf_write(device, BF_UART_FBEN_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_UART_OSR_INFO, osr);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_UART_DIV_INFO, div);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_UART_DIVM_INFO, div_m);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_UART_DIVN_INFO, div_n);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_config(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, uint32_t baudrate, adi_lark_uart_word_len_e len, 
    uint8_t stop_bit, adi_lark_uart_parity_type_e parity)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_INVALID_PARAM_RETURN(mclk_freq > API_LARK_MCLK_FREQ_98P304);
    LARK_INVALID_PARAM_RETURN(baudrate < 9600);
    LARK_INVALID_PARAM_RETURN(baudrate > 10000000);
    LARK_INVALID_PARAM_RETURN(len > API_LARK_UART_WORDLEN_8BITS);
    LARK_INVALID_PARAM_RETURN(stop_bit > 1);
    LARK_INVALID_PARAM_RETURN(parity > API_LARK_UART_PARITY_EVEN);
    
    err = adi_lark_uart_set_baudrate(device, mclk_freq, baudrate);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_UART_WLS_INFO, len);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_UART_STOPPED_INFO, stop_bit);
    LARK_ERROR_RETURN(err);
    err = adi_lark_uart_select_parity_type(device, parity);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_get_tx_buff_empty_status(adi_lark_device_t *device, uint8_t *empty)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t data;
    LARK_NULL_POINTER_RETURN(device);

    err = adi_lark_hal_bf_read(device, BF_UART_TEMT_INFO, &data);
    LARK_ERROR_RETURN(err);
    *empty = data & 0x01;
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_get_rx_buff_full_status(adi_lark_device_t *device, uint8_t *full)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t data;
    LARK_NULL_POINTER_RETURN(device);

    err = adi_lark_hal_bf_read(device, BF_UART_DR_INFO, &data);
    LARK_ERROR_RETURN(err);
    *full = data & 0x01;
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_read_byte(adi_lark_device_t *device, uint8_t *data)
{
    #ifdef LARK_SDK
    int32_t err;
    uint8_t full = 0;
    uint32_t rx_data;
    LARK_NULL_POINTER_RETURN(device);

    while (!full)
    {
        err = adi_lark_uart_get_rx_buff_full_status(device, &full);
        LARK_ERROR_RETURN(err);
    }
    
    err = adi_lark_hal_bf_read(device, BF_UART_RBR_INFO, &rx_data);
    LARK_ERROR_RETURN(err);
    *data = rx_data & 0xff;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_send_byte(adi_lark_device_t *device, uint8_t data)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t timeout = 0xFFF;
    uint8_t empty = 0;
    LARK_NULL_POINTER_RETURN(device);

    timeout = 0xFFF;
    while ((--timeout) && (!empty))
    {
        err = adi_lark_uart_get_tx_buff_empty_status(device, &empty);
        LARK_ERROR_RETURN(err);
    }
    if(timeout == 0)
    {
        return API_LARK_ERROR_ERROR;
    }
    
    err = adi_lark_hal_bf_write(device, BF_UART_RBR_INFO, data);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_rx_string(adi_lark_device_t *device, char *string)
{
    #ifdef LARK_SDK
    int32_t err;
    uint8_t data = 0;
    LARK_NULL_POINTER_RETURN(device);

    do
    {
        err = adi_lark_uart_read_byte(device, &data);
        LARK_ERROR_RETURN(err);
        *string++ = data;
    } while ((data != '\n') && (data != '\r'));
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_uart_tx_string(adi_lark_device_t *device, char const *string)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);

    while (*string)
    {
        err = adi_lark_uart_send_byte(device, *string++);
        LARK_ERROR_RETURN(err);
    }
    #endif
    
    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
int32_t adi_lark_device_uart_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq)
{
    int32_t err;
    char string[20];
    err = adi_lark_mp_set_pin_mode(device, 17, API_LARK_MP_MODE_NORMAL);
    LARK_ERROR_RETURN(err);
    err = adi_lark_mp_set_pin_mode(device, 18, API_LARK_MP_MODE_NORMAL);
    LARK_ERROR_RETURN(err);
    err = adi_lark_clk_set_tdsp_bus_rate(device, API_LARK_TDSP_CLK_RATE_MCLK_OVER_2, API_LARK_BUS_CLK_RATE_MCLK_OVER_2);
    LARK_ERROR_RETURN(err);
    err = adi_lark_uart_config(device, mclk_freq, 115200, API_LARK_UART_WORDLEN_8BITS, 0, API_LARK_UART_PARITY_NONE);
    LARK_ERROR_RETURN(err);
    
    while (1)
    {
        err = adi_lark_uart_rx_string(device, string);
        LARK_ERROR_RETURN(err);
        err = adi_lark_uart_tx_string(device, string);
        LARK_ERROR_RETURN(err);
    }
    
    return API_LARK_ERROR_OK;
}
#endif

/*! @} */
