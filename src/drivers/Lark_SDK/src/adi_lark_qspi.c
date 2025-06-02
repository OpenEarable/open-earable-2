/*!
 * @brief     QSPI Controller APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E C L A R E S ============*/
int32_t adi_lark_qspi_get_rx_empty_status(adi_lark_device_t *device, uint8_t *rx_empty);
int32_t adi_lark_qspi_get_tx_full_status(adi_lark_device_t *device, uint8_t *tx_full);
int32_t adi_lark_qspi_wait_tx_finished(adi_lark_device_t *device);

/*============= C O D E ====================*/
int32_t adi_lark_qspi_config_clk(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, uint32_t clk_freq, uint8_t clk_pha, uint8_t clk_pol )
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t pclk, clk_ctrl10, qspi_div;
    uint8_t apb_div;
    uint8_t apb_div_lookup[8] = {1, 2, 3, 4, 6, 8, 12, 16};
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_reg_read(device, REG_CLK_CTRL10_ADDR, &clk_ctrl10);
    LARK_ERROR_RETURN(err);
    apb_div = apb_div_lookup[(clk_ctrl10 >> 3) & 0x07];
    pclk = (24576000 * (mclk_freq + 1)) / apb_div;
    qspi_div = (pclk / (clk_freq)) - 1;
    if (qspi_div > 65535)
    {
        return API_LARK_ERROR_INVALID_PARAM;
    }
    
    if ((qspi_div != 0) && ((qspi_div & 0x01) == 0x00))
    {
        qspi_div++;
    }
    err = adi_lark_clk_set_qspi_clk_rate(device, qspi_div);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_QSPI_CPHA_INFO, clk_pha);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_QSPI_CPOL_INFO, clk_pol);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_write(device, BF_QSPI_MASTER_INFO, 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_QSPI_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_lsb_first(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_LSB_FIRST_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_fast_mode(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_FAST_MODE_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_mem_mapped(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_MEMMAP_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_receive(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_RX_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_receive_counter(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_RX_WCNT_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_transmit(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_TX_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_transmit_counter(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_TX_WCNT_EN_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_hw_slave_select(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_CS_CTRL_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_set_hw_slave_select_level(adi_lark_device_t *device, uint8_t level)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_QSPI_CS_POL_IN_TRAN_INFO, (level == 1) ? 0 : 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_slave_select_pin(adi_lark_device_t *device, uint8_t index, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_QSPI_SSEL1_EN_INFO + index, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_set_slave_select_pin_level(adi_lark_device_t *device, uint8_t index, uint8_t level)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_bf_write(device, BF_QSPI_SSEL1_INFO + index, level);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_set_transfer_size(adi_lark_device_t *device, adi_lark_qspi_transfer_size_e size)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(size > API_LARK_QSPI_TRANSFER_32_BIT);

    err = adi_lark_hal_bf_write(device, BF_QSPI_SIZE_INFO, size);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_set_multi_io_mode(adi_lark_device_t *device, adi_lark_qspi_multi_io_mode_e mode)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(mode > API_LARK_QSPI_QUAD);

    err = adi_lark_hal_bf_write(device, BF_QSPI_MULTI_IO_INFO, mode);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_enable_start_mosi(adi_lark_device_t *device, bool enable)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_QSPI_MULTI_IO_ORDER_INFO, enable ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_get_status(adi_lark_device_t *device, uint32_t *status)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_reg_read(device, REG_QSPI_STATUS_ADDR, status);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_get_rx_empty_status(adi_lark_device_t *device, uint8_t *rx_empty)
{
    #ifdef LARK_SDK
    int32_t err;    
    uint32_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(rx_empty);
    
    err = adi_lark_hal_bf_read(device, BF_QSPI_RX_FF_EMPTY_INFO, &status);
    LARK_ERROR_RETURN(err);
    *rx_empty = status & 0x01;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_get_tx_full_status(adi_lark_device_t *device, uint8_t *tx_full)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(tx_full);
    
    err = adi_lark_hal_bf_read(device, BF_QSPI_TX_FF_FULL_INFO, &status);
    LARK_ERROR_RETURN(err);
    *tx_full = status & 0x01;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_wait_tx_finished(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    int32_t timeout = 0xFFF;
    uint32_t tx_finished = 0;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
     while((--timeout > 0) && (tx_finished == 0))
    {
        err = adi_lark_hal_bf_read(device, BF_QSPI_TX_DONE_INFO, &tx_finished);
        LARK_ERROR_RETURN(err);
    }
    err = adi_lark_hal_bf_write(device, BF_QSPI_TX_DONE_INFO, 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_tx_data(adi_lark_device_t *device, uint8_t *tx_buf, uint32_t count)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t i, data, tran_word_count;
    #if LARK_SDK_RUN_ON_HIFI3Z
    int32_t timeout = 0xFFF;
    uint8_t fifo_full = 1;
    #endif
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    while (count > 0)
    {
        tran_word_count = (count < 0xffff) ? count : 0xffff;
        err = adi_lark_hal_bf_write(device, BF_QSPI_TX_CNT_INFO, tran_word_count);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_bf_write(device, BF_QSPI_TX_INIT_INFO, 1);
        LARK_ERROR_RETURN(err);
        err = adi_lark_qspi_enable_transmit_counter(device, true);
        LARK_ERROR_RETURN(err);
        err = adi_lark_qspi_enable_transmit(device, true);
        LARK_ERROR_RETURN(err);
        
        #if LARK_SDK_RUN_ON_HIFI3Z
        timeout = 0xFFF;
        #endif
        for (i = 0; i < tran_word_count; i++)
        {
        #if LARK_SDK_RUN_ON_HIFI3Z
        	timeout = 0xFFF;
			fifo_full = 1;
            while((--timeout > 0) && (fifo_full == 1))
            {
                err = adi_lark_qspi_get_tx_full_status(device, &fifo_full);
                LARK_ERROR_RETURN(err);
            }
            if (timeout < 0) 
            {
                break;
            }
        #endif
                
            data = tx_buf[i];
            err = adi_lark_hal_reg_write(device, REG_QSPI_TX_FIFO_ADDR, data);
            LARK_ERROR_RETURN(err);
        }
        err = adi_lark_qspi_wait_tx_finished(device);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_reg_write(device, REG_QSPI_TX_CTRL_ADDR, 0);
        LARK_ERROR_RETURN(err);
        count -= tran_word_count;
        tx_buf += tran_word_count;
    }
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_rx_data(adi_lark_device_t *device, uint8_t *rx_buf, uint32_t count)
{
    #ifdef LARK_SDK
    int32_t err;
    int32_t i;
    uint32_t data, recv_word_count;
    #if LARK_SDK_RUN_ON_HIFI3Z
    int32_t timeout = 0xFFF;
    uint8_t fifo_empty = 1;
    #endif
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();


    while (count > 0)
	{
		recv_word_count = (count < 0xffff) ? count : 0xffff;
		err = adi_lark_hal_bf_write(device, BF_QSPI_RX_CNT_INFO, recv_word_count);
		LARK_ERROR_RETURN(err);
		err = adi_lark_hal_bf_write(device, BF_QSPI_RX_INIT_INFO, 1);
		LARK_ERROR_RETURN(err);
		err = adi_lark_qspi_enable_receive_counter(device, true);
		LARK_ERROR_RETURN(err);
		err = adi_lark_qspi_enable_receive(device, true);
		LARK_ERROR_RETURN(err);

		for (i = 0; i < recv_word_count; i++)
		{
		#if LARK_SDK_RUN_ON_HIFI3Z
			timeout = 0xFFF;
			fifo_empty = 1;
			while((--timeout > 0) && (fifo_empty == 1))
			{
				err = adi_lark_qspi_get_rx_empty_status(device, &fifo_empty);
				LARK_ERROR_RETURN(err);
			}
			if (timeout < 0)
			{
				break;
			}
		#endif

			err = adi_lark_hal_bf_read(device, REG_QSPI_RX_FIFO_ADDR, 0x00000800, &data);
			LARK_ERROR_RETURN(err);
			rx_buf[i] = data & 0xff;
		}
		count -= recv_word_count;
		rx_buf += recv_word_count;
	}


    err = adi_lark_hal_reg_write(device, REG_QSPI_RX_CTRL_ADDR, 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE
typedef enum {
    ERASE_4KB,
    ERASE_32KB,
    ERASE_64KB,
    ERASE_CHIP,
} erase_type_t;

#define W25Q64FW                                    0xef

#define WRITE_ENABLE                                0x06
#define WRITE_ENABLE_FOR_VOLATILE_STATUS_REGISTER   0x50
#define WRITE_DISABLE                               0x04
#define READ_STATUS_REGISTER_1                      0x05
#define READ_STATUS_REGISTER_2                      0x35
#define READ_STATUS_REGISTER_3                      0x15
#define WRITE_STATUS_REGISTER_1                     0x01
#define WRITE_STATUS_REGISTER_2                     0x31
#define WRITE_STATUS_REGISTER_3                     0x11

#define PAGE_PROGRAM                                0x02
#define QUAD_PAGE_PROGRAM                           0x32
#define SECTOR_ERASE                                0x20 /* 4KB */
#define BLOCK_ERASE_32KB                            0x52
#define BLOCK_ERASE_64KB                            0xd8
#define CHIP_ERASE                                  0xc7 /* or 0x60 */
#define ERASE_PROGRAM_SUSPEND                       0x75
#define ERASE_PROGRAM_RESUME                        0x7a
#define POWER_DOWN                                  0xb9

/* Instruction Set Table 2 (Read Instructions) */
#define READ_DATA                                   0x03
#define FAST_READ                                   0x0b
#define FAST_READ_DUAL_OUTPUT                       0x3b
#define FAST_READ_QUAD_OUTPUT                       0x6b
#define FAST_READ_DUAL_IO                           0xbb
#define FAST_READ_QUAD_IO                           0xeb
#define WORD_READ_QUAD_IO                           0xe7
#define OCTAL_WORD_READ_QUAD_IO                     0xe3
#define SET_BURST_WITH_WRAP                         0x77

/* Instruction Set Table 3 (ID, Security Instructions) */
#define RELEASE_POWER_DOWN_DEVICE_ID                0xab
#define MANUFACTURER_ID_DEVICE_ID                   0x90
#define MANUFACTURER_ID_DEVICE_ID_BY_DUAL_IO        0x92
#define MANUFACTURER_ID_DEVICE_ID_BY_QUAD_IO        0x94
#define JEDEC_ID                                    0x9f
#define READ_UNIQUE_ID                              0x4b
#define READ_SFDP_REGISTER                          0x5a
#define ERASE_SECURITY_REGISTERS                    0x44
#define PROGRAM_SECURITY_REGISTERS                  0x42
#define READ_SECURITY_REGISTERS                     0x48

/* CONTINUOUS_READ_MODE */
#define CONTINUOUS_READ_MODE_OFF                    0xff
#define CONTINUOUS_READ_MODE_ON                     0x10

/* STATUS bits */
#define STATUS1_BUSY                                0x01
#define STATUS1_WEL                                 0x02
#define STATUS2_QE                                  0x02

/* Various sizes */
#define PAGE_SIZE                                   0x100
#define SECTOR_SIZE                                 0x1000
#define BLOCK_32KB_SIZE                             0x8000
#define BLOCK_64KB_SIZE                             0x10000

int32_t adi_lark_qspi_select_w25q64(adi_lark_device_t *device)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_qspi_set_slave_select_pin_level(device, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable(device, true);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_unselect_w25q64(adi_lark_device_t *device)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_qspi_set_slave_select_pin_level(device, 0, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable(device, false);
    LARK_ERROR_RETURN(err);
    return API_LARK_ERROR_OK;
}


int32_t adi_lark_qspi_enable_w25q64_write(adi_lark_device_t *device, bool enable)
{
    int32_t err;
    uint8_t tx_buff[1];
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    tx_buff[0] = enable ? WRITE_ENABLE : WRITE_DISABLE;

    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_read_w25q64_status(adi_lark_device_t *device, uint8_t index, uint8_t *status)
{
    int32_t err;
    uint8_t tx_buff[1];
    uint8_t rx_buff[1];
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    switch (index)
    {
    case 0:
        tx_buff[0] = READ_STATUS_REGISTER_1;
        break;
    case 1:
        tx_buff[0] = READ_STATUS_REGISTER_2;
        break;
    case 2:
        tx_buff[0] = READ_STATUS_REGISTER_3;
        break;
    default: 
        return -1;
    }
    
    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_rx_data(device, rx_buff, sizeof(rx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);
    
    *status = rx_buff[0];
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_write_w25q64_status(adi_lark_device_t *device, uint8_t index, uint8_t status)
{
    int32_t err;
    uint8_t tx_buff[2];
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    switch (index)
    {
    case 0:
        tx_buff[0] = WRITE_STATUS_REGISTER_1;
        break;
    case 1:
        tx_buff[0] = WRITE_STATUS_REGISTER_2;
        break;
    case 2:
        tx_buff[0] = WRITE_STATUS_REGISTER_3;
        break;
    default:
        return -1;
    }
    tx_buff[1] = status;
    
    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);
    
    return API_LARK_ERROR_OK;
}


int32_t adi_lark_qspi_read_w25q64_mid_did(adi_lark_device_t *device, uint8_t *mid, uint8_t *did)
{
    int32_t err;
    uint8_t tx_buff[4] = { MANUFACTURER_ID_DEVICE_ID, 0, 0, 0};
    uint8_t rx_buff[2];
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_rx_data(device, rx_buff, sizeof(rx_buff));
    LARK_ERROR_RETURN(err);
    adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);

    *mid = rx_buff[0];
    *did = rx_buff[1];

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_read_w25q64_uid(adi_lark_device_t *device, uint64_t *uid)
{
    int32_t err;
    uint8_t tx_buff[5] = { READ_UNIQUE_ID, 0, 0, 0, 0 };
    uint8_t rx_buff[8];
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_rx_data(device, rx_buff, sizeof(rx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);

    *uid = ((uint64_t)rx_buff[0]) << 56;
    *uid |= ((uint64_t)rx_buff[1]) << 48;
    *uid |= ((uint64_t)rx_buff[2]) << 40;
    *uid |= ((uint64_t)rx_buff[3]) << 32;
    *uid |= rx_buff[4] << 24;
    *uid |= rx_buff[5] << 16;
    *uid |= rx_buff[6] << 8;
    *uid |= rx_buff[7];

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_wait_w25q64_ready(adi_lark_device_t *device)
{
    int32_t err;
    uint8_t status;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_read_w25q64_status(device, 0, &status);
    LARK_ERROR_RETURN(err);

    while ((status & STATUS1_BUSY))
    {
        err = adi_lark_qspi_read_w25q64_status(device, 0, &status);
        LARK_ERROR_RETURN(err);
    }
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_erase_w25q64(adi_lark_device_t *device, uint32_t addr, erase_type_t erase_type)
{
    int32_t err;
    uint8_t tx_buff[4];
    uint32_t start_addr = 0;
    uint8_t status;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    switch (erase_type)
    {
    case ERASE_4KB:
        tx_buff[0] = SECTOR_ERASE;
        start_addr = addr & 0xfffff000;
        break;

    case ERASE_32KB:
        tx_buff[0] = BLOCK_ERASE_32KB;
        start_addr = addr & 0xffff8000;
        break;

    case ERASE_64KB:
        tx_buff[0] = BLOCK_ERASE_64KB;
        start_addr = addr & 0xffff0000;
        break;

    case ERASE_CHIP:
        tx_buff[0] = CHIP_ERASE;
        break;

    default:
        return API_LARK_ERROR_ERROR;
    }

    tx_buff[1] = (start_addr >> 16) & 0xff;
    tx_buff[2] = (start_addr >> 8) & 0xff;
    tx_buff[3] = start_addr & 0xff;

    err = adi_lark_qspi_enable_w25q64_write(device, true);
    LARK_ERROR_RETURN(err);

    err = adi_lark_qspi_read_w25q64_status(device, 0, &status);
    LARK_ERROR_RETURN(err);
    if (!(status & STATUS1_WEL))
        return API_LARK_ERROR_ERROR;

    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);

    if (tx_buff[0] == CHIP_ERASE)
    {    
        err = adi_lark_qspi_tx_data(device, tx_buff, 1);
        LARK_ERROR_RETURN(err);
    }
    else
    {
        err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
        LARK_ERROR_RETURN(err);
    }
    
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_wait_w25q64_ready(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_w25q64_read(adi_lark_device_t *device, uint32_t addr, uint8_t *buf, int size)
{
    int32_t err;
    uint8_t tx_buff[4];
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    tx_buff[0] = READ_DATA;
    tx_buff[1] = (addr >> 16) & 0xff;
    tx_buff[2] = (addr >> 8) & 0xff;
    tx_buff[3] = addr & 0xff;

    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_rx_data(device, buf, size);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_w25q64_page_program(adi_lark_device_t *device, uint32_t addr, uint8_t *buf, int size)
{
    int32_t err;
    uint8_t tx_buff[4];
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    tx_buff[0] = PAGE_PROGRAM;
    tx_buff[1] = (addr >> 16) & 0xff;
    tx_buff[2] = (addr >> 8) & 0xff;
    tx_buff[3] = addr & 0xff;
    
    err = adi_lark_qspi_enable_w25q64_write(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_select_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, tx_buff, sizeof(tx_buff));
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_tx_data(device, buf, size);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_unselect_w25q64(device);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_wait_w25q64_ready(device);
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_qspi_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq)
{
    int32_t err;
    uint8_t status, mid, did;
    uint64_t uid;

    /* Config QSPI */
    #ifdef LARK_SDK
    err = adi_lark_hal_reg_write(device, REG_QSPI_CTRL_ADDR, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_QSPI_RX_CTRL_ADDR, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_QSPI_TX_CTRL_ADDR, 0);
    LARK_ERROR_RETURN(err);
    #endif
    err = adi_lark_qspi_enable(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_config_clk(device, mclk_freq, 1000000, 0, 0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable_lsb_first(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable_hw_slave_select(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_set_transfer_size(device, API_LARK_QSPI_TRANSFER_8_BIT);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_set_multi_io_mode(device, API_LARK_QSPI_STANDARD);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable_start_mosi(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_enable_slave_select_pin(device, 0, true);
    LARK_ERROR_RETURN(err);

    /* Test write enable */
    err = adi_lark_qspi_enable_w25q64_write(device, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_read_w25q64_status(device, 0, &status);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_read_w25q64_status(device, 1, &status);
    LARK_ERROR_RETURN(err);

    /* Read UID */    
    err = adi_lark_qspi_read_w25q64_uid(device, &uid);
    LARK_ERROR_RETURN(err);

    /* Read MID & DID*/
    err = adi_lark_qspi_read_w25q64_mid_did(device, &mid, &did);
    LARK_ERROR_RETURN(err);
    if (mid != W25Q64FW)
    {
        LARK_LOG_MSG("FLASH manufacturer ID is not Winbond FLASH!");
        return API_LARK_ERROR_ERROR;
    }

    /* Erase Flash */
    err = adi_lark_qspi_erase_w25q64(device, 0, ERASE_CHIP);
    LARK_ERROR_RETURN(err);
    LARK_LOG_MSG("FLASH Erased!!");

    /* Program data test */
    uint8_t buf_write[256] = { 0x12, 0x34, 0x56, 0x78, 0x55, 0x78, 0x77, 0x88 };
    uint8_t buf_read[16] = { 0 };

    err = adi_lark_qspi_w25q64_read(device, 0x700000, buf_read, 16);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_w25q64_page_program(device, 0x700000, buf_write, 256);
    LARK_ERROR_RETURN(err);
    err = adi_lark_qspi_w25q64_read(device, 0x700000, buf_read, 16);
    LARK_ERROR_RETURN(err);

    if (buf_read[5] == buf_write[5])
    {
        LARK_LOG_MSG("Test FLASH erase function successfully");
    }
    else
    {
        LARK_LOG_MSG("Test FLASH erase function failed");
    }
    return API_LARK_ERROR_OK;
}
#endif

/*! @} */
