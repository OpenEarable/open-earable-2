/*! 
 * @brief     adi lark header file
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*! 
 * @addtogroup adi_lark_sdk
 * @{
 */

#ifndef __ADI_LARK_H__
#define __ADI_LARK_H__

/*============= I N C L U D E S ============*/
#include "adi_lark_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#if LARK_REPORT_VERBOSE > LARK_LOG_NONE_MSG
#include <stdarg.h>
#include <stdio.h>
#endif
#ifdef LARK_LITE_SDK
#include "adi_lark_lite_bf_lark_lite_yoda.h"
#endif
#ifdef LARK_SDK
#include "adi_lark_bf_lark_yoda.h"
#include "adi_lark_bf_uart.h"
#include "adi_lark_bf_wdt.h"
#include "adi_lark_bf_gpt.h"
#include "adi_lark_bf_dma.h"
#include "adi_lark_bf_data_sync.h"
#include "adi_lark_bf_soc.h"
#include "adi_lark_bf_fifo_sram.h"
#include "adi_lark_bf_qspi.h"
#endif

/*============= D E F I N E S ==============*/
/*!
 * @brief SDK version macro (major.minor.build, one byte each)
 */
#define ADI_LARK_SDK_VER            (0x00030301)

#ifdef __XTENSA__
#define __USE_ANONYMOUSE_UNION__    1
#else   // __XTENSA__
#ifdef __STDC_VERSION__
#if __STDC_VERSION > 199901
#define __USE_ANONYMOUSE_UNION__    1
#else   // __STDC_VERSION > 199901
#define __USE_ANONYMOUSE_UNION__    0
#endif  // __STDC_VERSION > 199901
#else   // __STDC_VERSION__
#define __USE_ANONYMOUSE_UNION__    0
#endif  // __STDC_VERSION__
#endif  // __XTENSA__


/*!
 * @brief Address macros
 */
#ifdef  LARK_LITE_SDK
#define LARK_EQ_PROGRAM_MEM_BASE     (0xA000)                /*!< start address of eq program memory */
#define LARK_EQ_PARAM_MEM_BASE       (0xA200)                /*!< start address of eq param   memory */
#define LARK_EQ_PARAM0_MEM_BASE      (0xA200)                /*!< start address of eq param0  memory */
#define LARK_EQ_PARAM1_MEM_BASE      (0xA400)                /*!< start address of eq param1  memory */
#define LARK_8BIT_REG_ADDR_START     (0xC000)                /*!< start address of 8-bit reg space   */
#define LARK_8BIT_REG_ADDR_END       (0xCFFF)                /*!< end   address of 8-bit reg space   */
#define LARK_FDSP_PROGRAM_MEM_BASE   (0xD000)                /*!< start address of fdsp code  memory */
#define LARK_FDSP_PARAM_MEM_BASE     (0xD100)                /*!< start adddres of fdsp param memory */
#define LARK_FDSP_PARAM_SPAN         (0x0100)    
#define LARK_FDSP_PARAM_BANKA_BASE   (0xD100)                /*!< start adddres of fdsp param bank a */
#define LARK_FDSP_PARAM_BANKB_BASE   (0xD600)                /*!< start adddres of fdsp param bank b */
#define LARK_FDSP_PARAM_BANKC_BASE   (0xDB00)                /*!< start adddres of fdsp param bank c */
#define LARK_FDSP_STATE_MEM_BASE     (0xE000)                /*!< start address of fdsp state memory */
#endif                               
#ifdef  LARK_SDK                     
#define LARK_FDSP_PROGRAM_MEM_BASE   (0x40008000)            /*!< start address of fdsp code  memory 0~63 */
#define LARK_FDSP_PARAM_MEM_BASE     (0x40008100)            /*!< start adddres of fdsp param memory 0~63 */
#define LARK_FDSP_PARAM_SPAN         (0x00000100)
#define LARK_FDSP_PARAM_BANKA_BASE   (0x40008100)            /*!< start adddres of fdsp param bank a 0~63 */
#define LARK_FDSP_PARAM_BANKB_BASE   (0x40008600)            /*!< start adddres of fdsp param bank b 0~63 */
#define LARK_FDSP_PARAM_BANKC_BASE   (0x40008B00)            /*!< start adddres of fdsp param bank c 0~63 */
#define LARK_FDSP_STATE_MEM_BASE     (0x40009000)            /*!< start address of fdsp state memory */
#define LARK_EQ_PROGRAM_MEM_BASE     (0x4000A000)            /*!< start address of eq program memory */
#define LARK_EQ_PARAM_MEM_BASE       (0x4000A200)            /*!< start address of eq param   memory */
#define LARK_EQ_PARAM0_MEM_BASE      (0x4000A200)            /*!< start address of eq param0  memory */
#define LARK_EQ_PARAM1_MEM_BASE      (0x4000A400)            /*!< start address of eq param1  memory */
#define LARK_FDSP_PROGRAM_T_MEM_BASE (0x4000A800)            /*!< start address of fdsp code  memory 64~95 */
#define LARK_FDSP_PARAM_T_MEM_BASE   (0x4000A900)            /*!< start adddres of fdsp param memory 64~95 */
#define LARK_FDSP_PARAM_T_BANKA_BASE (0x4000A900)            /*!< start adddres of fdsp param bank a 64~95 */
#define LARK_FDSP_PARAM_T_BANKB_BASE (0x4000AE00)            /*!< start adddres of fdsp param bank b 64~95 */
#define LARK_FDSP_PARAM_T_BANKC_BASE (0x4000B300)            /*!< start adddres of fdsp param bank c 64~95 */
#define LARK_FDSP_STATE_T_MEM_BASE   (0x4000B800)            /*!< start address of fdsp state memory 64~95 */
#define LARK_8BIT_REG_ADDR_START     (0x4000C000)            /*!< start address of 8-bit reg space   */
#define LARK_8BIT_REG_ADDR_END       (0x4000CFFF)            /*!< end   address of 8-bit reg space   */
#define LARK_ADP2SOC_BUF_START_ADDR  (0x40040000)            /*!< start address of adp2soc data buf  */
#define LARK_ADP2SOC_BUF_BYTE_LEN    (0x00000140)
#define LARK_SOC2ADP_BUF_START_ADDR  (0x40040200)            /*!< start address of soc2adp data buf  */
#define LARK_SOC2ADP_BUF_BYTE_LEN    (0x00000040)
#define LARK_ROM_START_ADDR          (0x50000000)            /*!< start address of rom space         */
#define LARK_TDSP_DRAM0_START_ADDR   (0x5FFF0000)            /*!< start address of l1 dram0          */
#define LARK_TDSP_DRAM1_START_ADDR   (0x5FFF4000)            /*!< start address of l1 dram1          */
#define LARK_TDSP_IRAM0_START_ADDR   (0x5FFF8000)            /*!< start address of l1 iram0          */
#define LARK_SYSRAM_START_ADDR       (0x60000000)            /*!< start address of system sram space */
#define LARK_FIFOSRAM_START_ADDR     (0x60010000)            /*!< start address of fifo   sram space */
#define LARK_FIFOSRAM0_START_ADDR    (0x60010000)            /*!< start address of fifo0  sram space */
#define LARK_FIFOSRAM1_START_ADDR    (0x60020000)            /*!< start address of fifo1  sram space */
#define LARK_FIFOSRAM2_START_ADDR    (0x60030000)            /*!< start address of fifo2  sram space */
#define LARK_QSPIFLASH_START_ADDR    (0x68000000)            /*!< start address of qspi  flash space */
#define LARK_DS_TIE_SRC_BASE_ADDR    (0x00)                  /*!< start address of tie source        */
#define LARK_DS_TIE_DST_BASE_ADDR    (0x80)                  /*!< start address of tie destination   */
#define LARK_LKUPMEMLD(chnl)         LKUPMEMLD(LARK_DS_TIE_SRC_BASE_ADDR + chnl)       /*! chnl: @see adi_lark_ds_rdy2dma_chnl_e */
#define LARK_LKUPMEMST(chnl, data)   LKUPMEMST(LARK_DS_TIE_DST_BASE_ADDR + chnl, data) /*! chnl: 0 ~ 15 for TDSP output 0 ~ 15 */
#endif

/*!
 * @brief SDK running on hifi3z processor or not
 */
#ifdef __XTENSA__
#undef  LARK_SDK_RUN_ON_HIFI3Z
#define LARK_SDK_RUN_ON_HIFI3Z      1                       /*!< 1 to run sdk on hifi3z */
#else
#undef  LARK_SDK_RUN_ON_HIFI3Z
#define LARK_SDK_RUN_ON_HIFI3Z      0                       /*!< 0 to run sdk on hifi3z */
#endif

/*!
 * @brief SDK message report macro
 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
    #define __FUNCTION_NAME__ __func__
#else
    #define __FUNCTION_NAME__ __FUNCTION__
#endif

#if LARK_REPORT_VERBOSE > LARK_LOG_NONE_MSG
#define LARK_MSG_REPORT(var, comment) \
    adi_lark_hal_error_report(device, LARK_LOG_VAR_MSG | LARK_LOG_INFO_MSG, __FILE__, __FUNCTION_NAME__, __LINE__, #var, comment)
#define LARK_WARN_REPORT(var, comment) \
    adi_lark_hal_error_report(device, LARK_LOG_VAR_MSG | LARK_LOG_WARN_MSG, __FILE__, __FUNCTION_NAME__, __LINE__, #var, comment)
#define LARK_ERROR_REPORT(var, comment) \
    adi_lark_hal_error_report(device, LARK_LOG_VAR_MSG | LARK_LOG_ERR_MSG,  __FILE__, __FUNCTION_NAME__, __LINE__, #var, comment)
#define LARK_LOG_MSG(msg) \
    adi_lark_hal_log_write(device, LARK_LOG_MISC_MSG | LARK_LOG_INFO_MSG, msg)
#define LARK_LOG_WARN(msg) \
    adi_lark_hal_log_write(device, LARK_LOG_MISC_MSG | LARK_LOG_WARN_MSG, msg)
#define LARK_LOG_ERR(msg) \
    adi_lark_hal_log_write(device, LARK_LOG_MISC_MSG | LARK_LOG_ERR_MSG,  msg)
#define LARK_LOG_FUNC() \
    adi_lark_hal_log_write(device, LARK_LOG_FUNC_MSG | LARK_LOG_INFO_MSG, "%s(...)", __FUNCTION_NAME__)
#define LARK_LOG_REG(msg, ...) \
    adi_lark_hal_log_write(device, LARK_LOG_REG_MSG  | LARK_LOG_INFO_MSG, msg, ##__VA_ARGS__)
#define LARK_LOG_VAR(type, msg, ...) \
    adi_lark_hal_log_write(device, LARK_LOG_VAR_MSG  | type, msg, ##__VA_ARGS__)
#else
#define LARK_MSG_REPORT(var, comment)
#define LARK_WARN_REPORT(var, comment)
#define LARK_ERROR_REPORT(var, comment)
#define LARK_LOG_MSG(msg)
#define LARK_LOG_WARN(msg)
#define LARK_LOG_ERR(msg)
#define LARK_LOG_FUNC()
#define LARK_LOG_VAR(type, msg, ...)
#define LARK_LOG_REG(msg, ...)
#endif

/*!
 * @brief error code check macro, return error code if it is not API_LARK_ERROR_OK
 */
#define LARK_ERROR_RETURN(r) \
{ \
    if ((r) != API_LARK_ERROR_OK) { \
        return r; \
    } \
}

/*!
 * @brief param check macro, return API_LARK_ERROR_NULL_PARAM if param is NULL
 */
#define LARK_NULL_POINTER_RETURN(p) \
{ \
    if((p) == NULL) { \
        LARK_ERROR_REPORT(p, "Null pointer passed."); \
        return API_LARK_ERROR_NULL_PARAM; \
    } \
}

/*!
 * @brief param check macro, return API_LARK_ERROR_INVALID_PARAM if equation is true
 */
#define LARK_INVALID_PARAM_RETURN(r) \
{ \
    if(r) { \
        LARK_ERROR_REPORT(r, "Invalid param passed."); \
        return API_LARK_ERROR_INVALID_PARAM; \
    } \
}

/*!
 * @brief param check macro, log warning message if equation is true
 */
#define LARK_INVALID_PARAM_WARN(r) \
{ \
    if(r) { \
        LARK_WARN_REPORT(r, "Invalid param passed."); \
    } \
}

/**
 * @brief  Platform dependent control port read function.
 *
 * @param  user_data    Pointer to customer data if needed, usually handle to a specific spi/i2c/uart
 * @param  rd_buf       Pointer to data returned from read operation
 * @param  rd_len       Length to read, in bytes
 * @param  wr_buf       Pointer to write data required during read operation
 * @param  wr_len       Length to write, in bytes
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
typedef int32_t (*adi_lark_read)(void* user_data, uint8_t *rd_buf, uint32_t rd_len, uint8_t *wr_buf, uint32_t wr_len);

/**
 * @brief  Platform dependent control port write function.
 *
 * @param  user_data    Pointer to customer data if needed, usually handle to a specific spi/i2c/uart device
 * @param  wr_buf       Pointer to write data required during read operation
 * @param  len          Length to write, in bytes
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
typedef int32_t (*adi_lark_write)(void* user_data, uint8_t *wr_buf, uint32_t len);

/**
 * @brief  Platform dependent log write function.
 *         
 * @param  user_data  Pointer to customer data if needed
 * @param  string     Messsage to log
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
typedef int32_t (*adi_lark_log_write)(void* user_data, char *string);

/*!
 * @brief  control port mode
 */
typedef enum {
    API_LARK_CP_I2C    = 0,                                 /*!< i2c mode */
    API_LARK_CP_SPI    = 1,                                 /*!< spi mode */
    API_LARK_CP_UART   = 2,                                 /*!< uart mode */
    API_LARK_CP_NATIVE = 3                                  /*!< native mode (for hifi3z only */
} adi_lark_cp_e;

/*!
 * @brief  adi lark device structure
 */
typedef struct
{
    void*              user_data;
    adi_lark_cp_e      cp_mode;                             /*!< @see adi_lark_cp_e */
    adi_lark_read      read;
    adi_lark_write     write;
    adi_lark_log_write log_write;
} adi_lark_device_t;

/*!
 * @brief  return value for adi lark api
 */
typedef enum {
    API_LARK_ERROR_OK                       =  0,           /*!< No Error */
    API_LARK_ERROR_ERROR                    = -1,           /*!< General Error  */
    API_LARK_ERROR_NULL_PARAM               = -2,           /*!< Null parameter */
    API_LARK_ERROR_INVALID_PARAM            = -3,           /*!< Invalid parameter passed */
    API_LARK_ERROR_NOT_SUPPORTED            = -4,           /*!< Not supported */
    API_LARK_ERROR_PLL_NOT_LOCKED           = -5,           /*!< PLL is not locked */
    API_LARK_ERROR_LOG_WRITE                = -6,           /*!< Log write error */
    API_LARK_ERROR_OSC_CAL_NOT_DONE         = -7,           /*!< RCOSC calibration is not done */
    API_LARK_ERROR_DMA_INIT                 = -100,         /*!< DMA init error */
    API_LARK_ERROR_DMA_CHAN                 = -101,         /*!< DMA channel number out of range */
    API_LARK_ERROR_DMA_DATABASE_NOT_SET     = -102,         /*!< DMA control data base register is not set */
    API_LARK_ERROR_DMA_DATA_RESULT          = -103,         /*!< DMA data dst are not equal to data src */
    API_LARK_ERROR_CP_UART_HEAD             = -200,         /*!< UART control port wrong HEAD */
    API_LARK_ERROR_CP_UART_ID               = -201,         /*!< UART control port wrong ID */
    API_LARK_ERROR_CP_UART_CRC              = -202,         /*!< UART control port wrong CRC */
    API_LARK_ERROR_CP_UART_STAT             = -203,         /*!< UART control port wrong STAT */
    API_LARK_ERROR_CP_UART_TAIL             = -204,         /*!< UART control port wrong TAIL */
    API_LARK_ERROR_CP_UART_WRITE_STATE      = -205,         /*!< UART control port wrong state of reading status of write command*/
    API_LARK_ERROR_FFSRAM_FIFO_SIZE         = -300,         /*!< FFSRAM FIFO size error */
    API_LARK_ERROR_FFSRAM_FIFO_WIDTH        = -301,         /*!< FIFO r/w width doesn't match register setting */
    API_LARK_ERROR_FFSRAM_FIFO_ALIGN        = -302,         /*!< FIFO r/w buffer address doesn't align with bit-width */
    API_LARK_ERROR_FFSRAM_FIFO_DLINE_EN     = -303,         /*!< FIFO mode and Delay line are both enabled */
    API_LARK_ERROR_FFSRAM_FIFO_DLINE_DIS    = -304,         /*!< FIFO mode and Delay line are both disabled */
    API_LARK_ERROR_REGISTER_ISR             = -400,         /*!< Register interrupt fail */
} adi_lark_error_e;

/*!
 * @brief  lark dma channels enumuration, for lark only.
 */
typedef enum {
    API_LARK_DMA_CHAN_DATA_SYNC0,                           /*!< Channel 0: Data Sync 0 */
    API_LARK_DMA_CHAN_DATA_SYNC1,                           /*!< Channel 1: Data Sync 1 */
    API_LARK_DMA_CHAN_DATA_SYNC2,                           /*!< Channel 2: Data Sync 2 */
    API_LARK_DMA_CHAN_DATA_SYNC3,                           /*!< Channel 3: Data Sync 3 */
    API_LARK_DMA_CHAN_DATA_SYNC4,                           /*!< Channel 4: Data Sync 4 */
    API_LARK_DMA_CHAN_DATA_SYNC5,                           /*!< Channel 5: Data Sync 5 */
    API_LARK_DMA_CHAN_DATA_SYNC6,                           /*!< Channel 6: Data Sync 6 */
    API_LARK_DMA_CHAN_DATA_SYNC7,                           /*!< Channel 7: Data Sync 7 */
    API_LARK_DMA_CHAN_DATA_SYNC8,                           /*!< Channel 8: Data Sync 8 */
    API_LARK_DMA_CHAN_DATA_SYNC9,                           /*!< Channel 9: Data Sync 9 */
    API_LARK_DMA_CHAN_DATA_SYNC10,                          /*!< Channel 10: Data Sync 10 */
    API_LARK_DMA_CHAN_DATA_SYNC11,                          /*!< Channel 11: Data Sync 11 */
    API_LARK_DMA_CHAN_DATA_SYNC12,                          /*!< Channel 12: Data Sync 12 */
    API_LARK_DMA_CHAN_DATA_SYNC13,                          /*!< Channel 13: Data Sync 13 */
    API_LARK_DMA_CHAN_DATA_SYNC14,                          /*!< Channel 14: Data Sync 14 */
    API_LARK_DMA_CHAN_DATA_SYNC15,                          /*!< Channel 15: Data Sync 15 */
    API_LARK_DMA_CHAN_SPI_TX,                               /*!< Channel 16: SPI TX */
    API_LARK_DMA_CHAN_SPI_RX,                               /*!< Channel 17: SPI RX */
    API_LARK_DMA_CHAN_UART_TX,                              /*!< Channel 18: UART TX */
    API_LARK_DMA_CHAN_UART_RX,                              /*!< Channel 19: UART RX */
    API_LARK_NUM_DMA_CHANNEL                                /*!< Total channel number */
} adi_lark_dma_chno_e;

/*!
 * @brief  struct for DMA channel control, for lark only.
 */
typedef struct {
    volatile uint32_t dma_src_end;                          /*!< Source End Pointer         */
    volatile uint32_t dma_dst_end;                          /*!< Destination End Pointer    */
    volatile uint32_t dma_chnl_cfg;                         /*!< Channel Data Configuration, @see adi_lark_dma_chnl_cfg_t */
    uint32_t reserved;                                      /*!< Reserved for address gap   */ 
} adi_lark_dma_chnl_ctrl_t;

/*!
 * @brief  struct for DMA channel config, for lark only.
 */
typedef struct
{
    union
    {
        struct
        {
            uint32_t cycle_ctrl             : 3;            /*!< The operating mode of the DMA cycle, bit 0:2 */
            uint32_t reserved3_3            : 1;            /*!< reserved, bit 3:3 */
            uint32_t n_minus_1              : 10;           /*!< Total number of transfers in the current DMA cycle - 1, bit 4:13 */
            uint32_t r_power                : 4;            /*!< arbitrate after 2^r_power DMA transfers, bit 14:17, must be 2 for primary scatter-gathering DMA */
            uint32_t reserved18_23          : 6;            /*!< reserved, bit 18:23 */
            uint32_t src_size               : 2;            /*!< Size of the source data, bit 24:25  @see adi_lark_dma_width_type_e */
            uint32_t src_inc                : 2;            /*!< Source address increment, bit 26:27  @see adi_lark_dma_incr_type_e */
            uint32_t reserved28_29          : 2;            /*!< reserved, bit 28:29 */
            uint32_t dst_inc                : 2;            /*!< Destination address increment, bit 30:31 @see adi_lark_dma_incr_type_e */
        }
#if !__USE_ANONYMOUSE_UNION__
        bit_field
#endif
        ;
        uint32_t VALUE32;
    }
#if !__USE_ANONYMOUSE_UNION__
        cfg_word
#endif
    ;
} adi_lark_dma_chnl_cfg_t;

/*!
 * @brief  enum for DMA increment type, for lark only. 
 *         @see adi_lark_dma_enable_chnl_src_decrement or @see adi_lark_dma_enable_chnl_dst_decrement for decrement.
 *         When decrement is enabled, the negative values are applied. 
 */
typedef enum
{
    API_LARK_DMA_INCR_1_BYTE                = 0x00u,        /*!< Byte increment */
    API_LARK_DMA_INCR_2_BYTE                = 0x01u,        /*!< Half word increment */
    API_LARK_DMA_INCR_4_BYTE                = 0x02u,        /*!< Word increment */
    API_LARK_DMA_INCR_NONE                  = 0x03u,        /*!< No increment */
} adi_lark_dma_incr_type_e;

/*!
 * @brief  enum for DMA width type
 */
typedef enum
{
    API_LARK_DMA_WIDTH_1_BYTE               = 0x0,          /*!<  8-bit */
    API_LARK_DMA_WIDTH_2_BYTE               = 0x1,          /*!< 16-bit */
    API_LARK_DMA_WIDTH_4_BYTE               = 0x2           /*!< 32-bit */
} adi_lark_dma_width_type_e;

/*!
 * @brief  enum for the DMA Descriptor Cycle Control, for lark only.
 */
typedef enum
{
    API_LARK_DMA_CTL_CYCLE_CTL_INVALID      = 0u,
    API_LARK_DMA_CTL_CYCLE_CTL_BASIC        = 1u,
    API_LARK_DMA_CTL_CYCLE_CTL_AUTO_REQ     = 2u,
    API_LARK_DMA_CTL_CYCLE_CTL_PING_PONG    = 3u,
    API_LARK_DMA_CTL_CYCLE_CTL_MSG_PRI      = 4u,
    API_LARK_DMA_CTL_CYCLE_CTL_MSG_ALT      = 5u,
    API_LARK_DMA_CTL_CYCLE_CTL_PSG_PRI      = 6u,
    API_LARK_DMA_CTL_CYCLE_CTL_PSG_ALT      = 7u
} adi_lark_dma_cycle_type_e;

/*!
 * @brief  enum for data sync input channels. Used for rdy2dma select and buffer offset, for lark only.
 */
typedef enum
{
    API_LARK_DS_INPUT_ADC0          = 0,
    API_LARK_DS_INPUT_ADC1          = 1,
    API_LARK_DS_INPUT_ADC2          = 2,
    API_LARK_DS_INPUT_DMIC4         = 3,
    API_LARK_DS_INPUT_DMIC0         = 4,
    API_LARK_DS_INPUT_DMIC1         = 5,
    API_LARK_DS_INPUT_DMIC2         = 6,
    API_LARK_DS_INPUT_DMIC3         = 7,
    API_LARK_DS_INPUT_ASRCI0        = 8,
    API_LARK_DS_INPUT_ASRCI1        = 9,
    API_LARK_DS_INPUT_ASRCI2        = 10,
    API_LARK_DS_INPUT_ASRCI3        = 11,
    API_LARK_DS_INPUT_EQ            = 12,
    API_LARK_DS_INPUT_DMIC5         = 13,
    API_LARK_DS_INPUT_DMIC6         = 14,
    API_LARK_DS_INPUT_DMIC7         = 15,
    API_LARK_DS_INPUT_SPT0_SLOT0    = 16,
    API_LARK_DS_INPUT_SPT0_SLOT1    = 17,
    API_LARK_DS_INPUT_SPT0_SLOT2    = 18,
    API_LARK_DS_INPUT_SPT0_SLOT3    = 19,
    API_LARK_DS_INPUT_SPT0_SLOT4    = 20,
    API_LARK_DS_INPUT_SPT0_SLOT5    = 21,
    API_LARK_DS_INPUT_SPT0_SLOT6    = 22,
    API_LARK_DS_INPUT_SPT0_SLOT7    = 23,
    API_LARK_DS_INPUT_SPT0_SLOT8    = 24,
    API_LARK_DS_INPUT_SPT0_SLOT9    = 25,
    API_LARK_DS_INPUT_SPT0_SLOT10   = 26,
    API_LARK_DS_INPUT_SPT0_SLOT11   = 27,
    API_LARK_DS_INPUT_SPT0_SLOT12   = 28,
    API_LARK_DS_INPUT_SPT0_SLOT13   = 29,
    API_LARK_DS_INPUT_SPT0_SLOT14   = 30,
    API_LARK_DS_INPUT_SPT0_SLOT15   = 31,
    API_LARK_DS_INPUT_SPT1_SLOT0    = 32,
    API_LARK_DS_INPUT_SPT1_SLOT1    = 33,
    API_LARK_DS_INPUT_SPT1_SLOT2    = 34,
    API_LARK_DS_INPUT_SPT1_SLOT3    = 35,
    API_LARK_DS_INPUT_SPT1_SLOT4    = 36,
    API_LARK_DS_INPUT_SPT1_SLOT5    = 37,
    API_LARK_DS_INPUT_SPT1_SLOT6    = 38,
    API_LARK_DS_INPUT_SPT1_SLOT7    = 39,
    API_LARK_DS_INPUT_SPT1_SLOT8    = 40,
    API_LARK_DS_INPUT_SPT1_SLOT9    = 41,
    API_LARK_DS_INPUT_SPT1_SLOT10   = 42,
    API_LARK_DS_INPUT_SPT1_SLOT11   = 43,
    API_LARK_DS_INPUT_SPT1_SLOT12   = 44,
    API_LARK_DS_INPUT_SPT1_SLOT13   = 45,
    API_LARK_DS_INPUT_SPT1_SLOT14   = 46,
    API_LARK_DS_INPUT_SPT1_SLOT15   = 47,
    API_LARK_DS_INPUT_FDEC0         = 48,
    API_LARK_DS_INPUT_FDEC1         = 49,
    API_LARK_DS_INPUT_FDEC2         = 50,
    API_LARK_DS_INPUT_FDEC3         = 51,
    API_LARK_DS_INPUT_FDEC4         = 52,
    API_LARK_DS_INPUT_FDEC5         = 53,
    API_LARK_DS_INPUT_FDEC6         = 54,
    API_LARK_DS_INPUT_FDEC7         = 55,
    API_LARK_DS_INPUT_FINT0         = 56,
    API_LARK_DS_INPUT_FINT1         = 57,
    API_LARK_DS_INPUT_FINT2         = 58,
    API_LARK_DS_INPUT_FINT3         = 59,
    API_LARK_DS_INPUT_FINT4         = 60,
    API_LARK_DS_INPUT_FINT5         = 61,
    API_LARK_DS_INPUT_FINT6         = 62,
    API_LARK_DS_INPUT_FINT7         = 63,
    API_LARK_DS_INPUT_FDSP0         = 64,
    API_LARK_DS_INPUT_FDSP1         = 65,
    API_LARK_DS_INPUT_FDSP2         = 66,
    API_LARK_DS_INPUT_FDSP3         = 67,
    API_LARK_DS_INPUT_FDSP4         = 68,
    API_LARK_DS_INPUT_FDSP5         = 69,
    API_LARK_DS_INPUT_FDSP6         = 70,
    API_LARK_DS_INPUT_FDSP7         = 71,
    API_LARK_DS_INPUT_FDSP8         = 72,
    API_LARK_DS_INPUT_FDSP9         = 73,
    API_LARK_DS_INPUT_FDSP10        = 74,
    API_LARK_DS_INPUT_FDSP11        = 75,
    API_LARK_DS_INPUT_FDSP12        = 76,
    API_LARK_DS_INPUT_FDSP13        = 77,
    API_LARK_DS_INPUT_FDSP14        = 78,
    API_LARK_DS_INPUT_FDSP15        = 79,
    API_LARK_DS_INPUT_COUNT
} adi_lark_ds_rdy2dma_chnl_e;

/*!
 * @brief  macro for enum of TIE source channel, same as @see adi_lark_ds_rdy2dma_chnl_e
 */
#define adi_lark_ds_tie_src_chnl_e adi_lark_ds_rdy2dma_chnl_e

/*!
 * @brief  macro for enum of FIFO source channel, same as @see adi_lark_ds_rdy2dma_chnl_e
 */
#define adi_lark_ds_fifo_src_chnl_e adi_lark_ds_rdy2dma_chnl_e

/*!
 * @brief  enum for rdy2out channels, for lark only.
 */
typedef enum
{
    API_LARK_DS_RDY2OUT_ADC01       = 0,
    API_LARK_DS_RDY2OUT_ADC23       = 1,
    API_LARK_DS_RDY2OUT_DMIC01      = 2,
    API_LARK_DS_RDY2OUT_DMIC23      = 3,
    API_LARK_DS_RDY2OUT_ASCRI       = 4,
    API_LARK_DS_RDY2OUT_SPT0        = 5,
    API_LARK_DS_RDY2OUT_SPT1        = 6,
    API_LARK_DS_RDY2OUT_FDEC01      = 7,
    API_LARK_DS_RDY2OUT_FDEC23      = 8,
    API_LARK_DS_RDY2OUT_FDEC45      = 9,
    API_LARK_DS_RDY2OUT_FDEC67      = 10,
    API_LARK_DS_RDY2OUT_FINT01      = 11,
    API_LARK_DS_RDY2OUT_FINT23      = 12,
    API_LARK_DS_RDY2OUT_FINT45      = 13,
    API_LARK_DS_RDY2OUT_FINT67      = 14,
    API_LARK_DS_RDY2OUT_FDSP        = 15,
    API_LARK_DS_RDY2OUT_EQ          = 16,
    API_LARK_DS_RDY2OUT_FIXED       = 17,
    API_LARK_DS_RDY2OUT_8K          = 18,
    API_LARK_DS_RDY2OUT_12K         = 19,
    API_LARK_DS_RDY2OUT_16K         = 20,
    API_LARK_DS_RDY2OUT_24K         = 21,
    API_LARK_DS_RDY2OUT_48K         = 22,
    API_LARK_DS_RDY2OUT_96K         = 23,
    API_LARK_DS_RDY2OUT_192K        = 24,
    API_LARK_DS_RDY2OUT_384K        = 25,
    API_LARK_DS_RDY2OUT_768K        = 26,
    API_LARK_DS_RDY2OUT_COUNT
} adi_lark_ds_rdy2out_chnl_e;

/*!
 * @brief  enum for data sync interrupt channels, for lark only.
 */
typedef enum
{
    API_LARK_DS_INT_ADC01           = 0,
    API_LARK_DS_INT_ADC23           = 1,
    API_LARK_DS_INT_DMIC01          = 2,
    API_LARK_DS_INT_DMIC23          = 3,
    API_LARK_DS_INT_ASCRI           = 4,
    API_LARK_DS_INT_SPT0            = 5,
    API_LARK_DS_INT_SPT1            = 6,
    API_LARK_DS_INT_FDEC01          = 7,
    API_LARK_DS_INT_FDEC23          = 8,
    API_LARK_DS_INT_FDEC45          = 9,
    API_LARK_DS_INT_FDEC67          = 10,
    API_LARK_DS_INT_FINT01          = 11,
    API_LARK_DS_INT_FINT23          = 12,
    API_LARK_DS_INT_FINT45          = 13,
    API_LARK_DS_INT_FINT67          = 14,
    API_LARK_DS_INT_FDSP            = 15,
    API_LARK_DS_INT_EQ              = 16,
    API_LARK_DS_INT_FIXED           = 17,
    API_LARK_DS_INT_COUNT
} adi_lark_ds_int_chnl_e;

/*!
 * @brief  enum for SAP control's SAI mode (SPT_SAI_MODE)
 */
typedef enum
{
    API_LARK_SAP_CTRL_SAI_MODE_STEREO       = 0,            /*!< Sets the serial port in STEREO mode, assume 32-bit words per channel */
    API_LARK_SAP_CTRL_SAI_MODE_TDM          = 1,            /*!< Sets the serial port in TDM mode  */
} adi_lark_sap_sai_mode_e;

/*!
 * @brief  enum for SAP control's data format (SPT_DATA_FORMAT)
 */
typedef enum
{
    API_LARK_SAP_CTRL_DATA_FORMAT_LJ        = 1,            /*!< Left Justified,delay by 0 */
    API_LARK_SAP_CTRL_DATA_FORMAT_I2S       = 0,            /*!< Typical I2S mode, delay by 1 */
    API_LARK_SAP_CTRL_DATA_FORMAT_DLY8      = 2,            /*!< Delay by 8 */
    API_LARK_SAP_CTRL_DATA_FORMAT_DLY12     = 3,            /*!< Delay by 12 */
    API_LARK_SAP_CTRL_DATA_FORMAT_DLY16     = 4             /*!< Delay by 16 */
} adi_lark_sap_data_format_e;

/*!
 * @brief  enum for SAP control's slot width (SPT_SLOT_WIDTH)
 */
typedef enum
{
    API_LARK_SAP_CTRL_SLOT_WIDTH_32         = 0,
    API_LARK_SAP_CTRL_SLOT_WIDTH_16         = 1,
    API_LARK_SAP_CTRL_SLOT_WIDTH_24         = 2
} adi_lark_sap_slot_width_e;

/*!
 * @brief  enum for SAP control's tri-state enable (SPT_TRI_STATE)
 */
typedef enum
{
    API_LARK_SAP_CTRL_TRI_STATE_ENABLE      = 1,            /*!< tri-state enabled  */
    API_LARK_SAP_CTRL_TRI_STATE_DISABLE     = 0             /*!< tri-state disabled */
} adi_lark_sap_tristate_e;

/*!
 * @brief  enum for SAP control's data width (SPT_DATA_WIDTH)
 */
typedef enum
{
    API_LARK_SAP_CTRL_DATA_WIDTH_24         = 0,
    API_LARK_SAP_CTRL_DATA_WIDTH_16         = 1
} adi_lark_sap_data_width_e;

/*!
 * @brief  enum for sap mclk output rates, for both lark and lark-lite
 */
typedef enum{
    API_LARK_SAP_MCLK_24576K = 0,
    API_LARK_SAP_MCLK_12288K = 1,
    API_LARK_SAP_MCLK_6144K  = 2,
    API_LARK_SAP_MCLK_3072K  = 3,
    API_LARK_SAP_MCLK_1536K  = 4,
    API_LARK_SAP_MCLK_768K   = 5,
    API_LARK_SAP_MCLK_384K   = 6,
    API_LARK_SAP_MCLK_192K   = 7,
    API_LARK_SAP_MCLK_CNT    = 8,
} adi_lark_sap_mclk_rate_e;

/*!
 * @brief  enum for SAP control's bclk src (SPT_BCLK_SRC)
 */
typedef enum
{
    API_LARK_SAP_CTRL_BCLK_SRC_SLAVE        = 0,
    API_LARK_SAP_CTRL_BCLK_SRC_3072K        = 1,
    API_LARK_SAP_CTRL_BCLK_SRC_6144K        = 2,
    API_LARK_SAP_CTRL_BCLK_SRC_12288K       = 3,
    API_LARK_SAP_CTRL_BCLK_SRC_24576K       = 4
} adi_lark_sap_bclk_src_e;

/*!
 * @brief  enum for SAP control's bclk polarity (SPT_BCLK_POL)
 */
typedef enum
{
    API_LARK_SAP_CTRL_BCLK_POL_NEG          = 0,            /*!< Capture on rising edge  */
    API_LARK_SAP_CTRL_BCLK_POL_POS          = 1             /*!< Capture on falling egde */
} adi_lark_sap_bclk_polarity_e;

/*!
 * @brief  enum for SAP control's lrclk src (SPT_LRCLK_SRC)
 */
typedef enum
{
    API_LARK_SAP_CTRL_LRCLK_SRC_SLAVE       = 0,
    API_LARK_SAP_CTRL_LRCLK_SRC_48K         = 1,
    API_LARK_SAP_CTRL_LRCLK_SRC_96K         = 2,
    API_LARK_SAP_CTRL_LRCLK_SRC_192K        = 3,
    API_LARK_SAP_CTRL_LRCLK_SRC_12K         = 4,
    API_LARK_SAP_CTRL_LRCLK_SRC_24K         = 5,
    API_LARK_SAP_CTRL_LRCLK_SRC_384K        = 6,
    API_LARK_SAP_CTRL_LRCLK_SRC_768K        = 7,
    API_LARK_SAP_CTRL_LRCLK_SRC_8K          = 8,
    API_LARK_SAP_CTRL_LRCLK_SRC_16K         = 9
} adi_lark_sap_lrclk_src_e;

/*!
 * @brief  enum for SAP control's lrclk polarity (SPT_LRCLK_POL)
 */
typedef enum
{
    API_LARK_SAP_CTRL_LRCLK_POL_NORM        = 0,            /*!< Normal Polarity  */
    API_LARK_SAP_CTRL_LRCLK_POL_INVT        = 1             /*!< Inverted Polarity */
} adi_lark_sap_lrclk_polarity_e;

/*!
 * @brief  enum for SAP output data source
 */
typedef enum
{
    #ifdef LARK_SDK
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP0,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP1,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP2,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP3,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP4,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP5,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP6,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP7,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP8,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP9,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP10,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP11,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP12,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP13,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP14,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP15,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP0,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP1,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP2,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP3,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP4,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP5,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP6,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP7,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP8,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP9,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP10,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP11,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP12,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP13,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP14,
    API_LARK_SAP_OUT_ROUTE_FROM_TDSP15,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO2,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO3,
    API_LARK_SAP_OUT_ROUTE_FROM_ADC0,
    API_LARK_SAP_OUT_ROUTE_FROM_ADC1,
    API_LARK_SAP_OUT_ROUTE_FROM_ADC2,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC0,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC1,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC2,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC3,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC0,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC1,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC2,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC3,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC4,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC5,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC6,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC7,
    API_LARK_SAP_OUT_ROUTE_FROM_EQ,
    API_LARK_SAP_OUT_ROUTE_FROM_EQ1,
    API_LARK_SAP_OUT_ROUTE_FROM_EQ2,
    API_LARK_SAP_OUT_ROUTE_FROM_EQ3,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC4,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC5,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC6,
    API_LARK_SAP_OUT_ROUTE_FROM_DMIC7,
    API_LARK_SAP_OUT_ROUTE_NOT_USED  = 63,
    API_LARK_SAP_OUT_ROUTE_COUNT,
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_SAP_OUT_ROUTE_FROM_ADC0,
    API_LARK_SAP_OUT_ROUTE_FROM_ADC1,
    API_LARK_SAP_OUT_ROUTE_FROM_ADC2,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO0,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO1,
    API_LARK_SAP_OUT_ROUTE_FROM_ASRCO2,
    API_LARK_SAP_OUT_ROUTE_FROM_EQ,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC0,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC1,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC2,
    API_LARK_SAP_OUT_ROUTE_FROM_FDEC3,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP0,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP1,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP2,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP3,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP4,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP5,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP6,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP7,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP8,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP9,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP10,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP11,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP12,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP13,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP14,
    API_LARK_SAP_OUT_ROUTE_FROM_FDSP15,
    API_LARK_SAP_OUT_ROUTE_COUNT,
    #endif
} adi_lark_sap_output_route_from_e;

/*!
 * @brief  struct for serial audio port configuration
 */
typedef struct
{
    adi_lark_sap_sai_mode_e       sai_mode;
    adi_lark_sap_data_format_e    data_format;
    adi_lark_sap_slot_width_e     slot_width;
    adi_lark_sap_tristate_e       tri_state;
    adi_lark_sap_data_width_e     data_width;
    adi_lark_sap_bclk_src_e       bclk_src;
    adi_lark_sap_bclk_polarity_e  bclk_pol;
    adi_lark_sap_lrclk_src_e      lrclk_src;
    adi_lark_sap_lrclk_polarity_e lrclk_pol;
} adi_lark_sap_config_t;

/*!
 * @brief  struct for SAP SPT_CTRL1
 */
typedef struct
{
    union
    {
        struct
        {
            uint8_t sai_mode                    :  1;           /*!< SPT_SAI_MODE, bit 0:0 */
            uint8_t data_format                 :  3;           /*!< SPT_DATA_FORMAT, bit 1:3 */
            uint8_t slot_width                  :  2;           /*!< SPT_SLOT_WIDTH, bit 4:5 */
            uint8_t tristate                    :  1;           /*!< R_Power arbitrates after x DMA transfers, bit 6:6 */
            uint8_t data_width                  :  1;           /*!< reserved, bit 7:7 */
        }
#if !__USE_ANONYMOUSE_UNION__
        bit_field
#endif
        ;
        uint8_t VALUE8;
    }
#if !__USE_ANONYMOUSE_UNION__
	ctrl_byte
#endif
    ;
} adi_lark_sap_ctrl1_t;

/*!
 * @brief  struct for SAP SPT_CTRL2
 */
typedef struct
{
    union
    {
        struct
        {
            uint8_t bclk_src                    :  3;           /*!< SPT_BCLK_SRC, bit 0:2 */
            uint8_t bclk_pol                    :  1;           /*!< SPT_BCLK_POL, bit 3:3 */
        }
#if !__USE_ANONYMOUSE_UNION__
        bit_field
#endif
        ;
        uint8_t VALUE8;
    }
#if !__USE_ANONYMOUSE_UNION__
	ctrl_byte
#endif
    ;
} adi_lark_sap_ctrl2_t;

/*!<
 * @brief  struct for SAP SPT_CTRL3
 */
typedef struct
{
    union
    {
        struct
        {
            uint8_t lrclk_src                    :  4;           /*!< SPT_BCLK_SRC, bit 0:3 */
            uint8_t lrclk_pol                    :  1;           /*!< SPT_BCLK_POL, bit 4:4 */
        }
#if !__USE_ANONYMOUSE_UNION__
        bit_field
#endif
        ;
        uint8_t VALUE8;
    }
#if !__USE_ANONYMOUSE_UNION__
	ctrl_byte
#endif
    ;
} adi_lark_sap_ctrl3_t;

/*!
 * @brief  enum for lark internal interrupt (connected to IRQ1/2/3/4 pin)
 */
typedef enum
{
    API_LARK_IRQ_DAC0_CLIP        = 0x00000001,
    API_LARK_IRQ_ADC0_CLIP        = 0x00000002,
    API_LARK_IRQ_ADC1_CLIP        = 0x00000004,
    API_LARK_IRQ_ADC2_CLIP        = 0x00000008,
    API_LARK_IRQ_PLL_LOCKED       = 0x00000010,
    API_LARK_IRQ_PLL_UNLOCKED     = 0x00000020,
    API_LARK_IRQ_AVDD_UVW         = 0x00000040,
    API_LARK_IRQ_PRAMP            = 0x00000080,
    API_LARK_IRQ_FDSP             = 0x00000100,
    API_LARK_IRQ_EQ_CLR_DONE      = 0x00000200,
    API_LARK_IRQ_ASRCI_LOCKED     = 0x00000400,
    API_LARK_IRQ_ASRCI_UNLOCKED   = 0x00000800,
    API_LARK_IRQ_ASRCO_LOCKED     = 0x00001000,
    API_LARK_IRQ_ASRCO_UNLOCKED   = 0x00002000,
    API_LARK_IRQ_SYNC_LOCKED      = 0x00004000,
    API_LARK_IRQ_SYNC_UNLOCKED    = 0x00008000,
    API_LARK_IRQ_SPT0_UNLOCKED    = 0x00010000,
    #ifdef LARK_SDK
    API_LARK_IRQ_SPI1_UNLOCKED    = 0x00020000,
    #endif
    API_LARK_IRQ_POWERUP_COMPLETE = 0x00040000,
    #ifdef LARK_SDK
    API_LARK_IRQ_DLDO_DVS_DONE    = 0x00080000,
    API_LARK_IRQ_SW_INT0          = 0x00100000,
    API_LARK_IRQ_SW_INT1          = 0x00200000,
    API_LARK_IRQ_SW_INT2          = 0x00400000,
    API_LARK_IRQ_SW_INT3          = 0x00800000,
    API_LARK_IRQ_DATA_SYNC        = 0x01000000,
    API_LARK_IRQ_FFSRAM2          = 0x02000000,
    API_LARK_IRQ_TDSP_ERR         = 0x04000000,
    API_LARK_IRQ_AHB_ERR          = 0x08000000,
    API_LARK_IRQ_AHB_BUSY         = 0x10000000,
    API_LARK_IRQ_WDT              = 0x20000000,
    #endif
} adi_lark_irq_src_e;

/*!
 * @brief  enum for lark soc interrupt (connected to TDSP), for lark only.
 */
typedef enum
{
    API_LARK_SOC_IRQ_FFSRAM0    = 0x00000001,
    API_LARK_SOC_IRQ_FFSRAM1    = 0x00000002,
    API_LARK_SOC_IRQ_FFSRAM2    = 0x00000004,
    API_LARK_SOC_IRQ_QSPI       = 0x00000008,
    API_LARK_SOC_IRQ_DTSYNC     = 0x00000010,
    API_LARK_SOC_IRQ_DMA        = 0x00000020,
    API_LARK_SOC_IRQ_IRQ1       = 0x00000040,
    API_LARK_SOC_IRQ_IRQ2       = 0x00000080,
    API_LARK_SOC_IRQ_IRQ3       = 0x00000100,
    API_LARK_SOC_IRQ_IRQ4       = 0x00000200,
    API_LARK_SOC_IRQ_MP_IN_IRQ1 = 0x00000400,
    API_LARK_SOC_IRQ_MP_IN_IRQ2 = 0x00000800,
    API_LARK_SOC_IRQ_MP_IN_IRQ3 = 0x00001000,
    API_LARK_SOC_IRQ_UART       = 0x00002000,
    API_LARK_SOC_IRQ_SPI        = 0x00004000,
    API_LARK_SOC_IRQ_GPT        = 0x01000000,
    API_LARK_SOC_IRQ_WDT        = 0x02000000,
} adi_lark_soc_irq_src_e;

/*!
 * @brief  enum for lark soc interrupt priority, for lark only.
 */
typedef enum
{
    API_LARK_SOC_IRQ_PRI_LOW  = 0,
    API_LARK_SOC_IRQ_PRI_HIGH = 1
} adi_lark_soc_irq_pri_e;

/*!
 * @brief  enum for FastDSP mode
 */
typedef enum{
    API_LARK_FDSP_NORMAL_MODE = 0,
    API_LARK_FDSP_TURBO_MODE =  1,
} adi_lark_fdsp_mode_e;

/*!
 * @brief  enum for FastDSP parameter bank
 */
typedef enum{
    API_LARK_FDSP_PARAM_BANK_A = 0,
    API_LARK_FDSP_PARAM_BANK_B = 1,
    API_LARK_FDSP_PARAM_BANK_C = 2
} adi_lark_fdsp_param_bank_e;

/*!
 * @brief  enum for FastDSP parameter bank switch mode
 */
typedef enum{
    API_LARK_FDSP_PARAM_SWICTH_RAMP    = 0,
    API_LARK_FDSP_PARAM_SWICTH_INSTANT = 1
} adi_lark_fdsp_param_switch_mode_e;

/*!
 * @brief  enum for FastDSP ramp rate
 */
typedef enum{
    API_LARK_FDSP_RAMP_RATE_0P02SEC   = 0,                      /*!< 0.02s ramp */
    API_LARK_FDSP_RAMP_RATE_0P04SEC   = 1,                      /*!< 0.04s ramp */
    API_LARK_FDSP_RAMP_RATE_0P06SEC   = 2,                      /*!< 0.06s ramp */
    API_LARK_FDSP_RAMP_RATE_0P08SEC   = 3,                      /*!< 0.08s ramp */
    API_LARK_FDSP_RAMP_RATE_0P1SEC    = 4,                      /*!< 0.1s  ramp */
    API_LARK_FDSP_RAMP_RATE_0P15SEC   = 5,                      /*!< 0.15s ramp */
    API_LARK_FDSP_RAMP_RATE_0P2SEC    = 6,                      /*!< 0.2s  ramp */
    API_LARK_FDSP_RAMP_RATE_0P25SEC   = 7,                      /*!< 0.25s ramp */
    API_LARK_FDSP_RAMP_RATE_0P3SEC    = 8,                      /*!< 0.3s  ramp */
    API_LARK_FDSP_RAMP_RATE_0P5SEC    = 9,                      /*!< 0.5s  ramp */
    API_LARK_FDSP_RAMP_RATE_0P75SEC   = 10,                     /*!< 0.75s ramp */
    API_LARK_FDSP_RAMP_RATE_1SEC      = 11,                     /*!< 1s    ramp */
    API_LARK_FDSP_RAMP_RATE_1P25SEC   = 12,                     /*!< 1.25s ramp */
    API_LARK_FDSP_RAMP_RATE_1P5SEC    = 13,                     /*!< 1.5s  ramp */
    API_LARK_FDSP_RAMP_RATE_1P75SEC   = 14,                     /*!< 1.75s ramp */
    API_LARK_FDSP_RAMP_RATE_2SEC      = 15                      /*!< 2s    ramp */
} adi_lark_fdsp_ramp_rate_e;

/*!
 * @brief  enum for FastDSP copy parameter bank
 */
typedef enum{
    API_LARK_FDSP_COPY_PARAM_BANK_A2B = 0,                      /*!< Copy Parameter Bank A to Bank B */
    API_LARK_FDSP_COPY_PARAM_BANK_A2C = 1,                      /*!< Copy Parameter Bank A to Bank C */
    API_LARK_FDSP_COPY_PARAM_BANK_B2A = 2,                      /*!< Copy Parameter Bank B to Bank A */
    API_LARK_FDSP_COPY_PARAM_BANK_B2C = 3,                      /*!< Copy Parameter Bank B to Bank C */
    API_LARK_FDSP_COPY_PARAM_BANK_C2A = 4,                      /*!< Copy Parameter Bank C to Bank A */
    API_LARK_FDSP_COPY_PARAM_BANK_C2B = 5                       /*!< Copy Parameter Bank C to Bank B */
} adi_lark_fdsp_copy_param_bank_e;

/*!
 * @brief  enum for FastDSP rate source
 */
typedef enum{
    API_LARK_FDSP_RATE_SRC_ADC0_1     = 0,
    API_LARK_FDSP_RATE_SRC_ADC2       = 1,
    #ifdef LARK_SDK
    API_LARK_FDSP_RATE_SRC_DMIC0_1    = 2,
    API_LARK_FDSP_RATE_SRC_DMIC2_3    = 3,
    API_LARK_FDSP_RATE_SRC_DMIC4_5    = 4,
    API_LARK_FDSP_RATE_SRC_DMIC6_7    = 5,
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_FDSP_RATE_SRC_EQ         = 5,
    #endif
    API_LARK_FDSP_RATE_SRC_SAI0       = 6,
    #ifdef LARK_SDK
    API_LARK_FDSP_RATE_SRC_SAI1       = 7,
    #endif
    API_LARK_FDSP_RATE_SRC_FINT0_1    = 10,
    #ifdef LARK_SDK
    API_LARK_FDSP_RATE_SRC_FINT2_3    = 11,
    API_LARK_FDSP_RATE_SRC_FINT4_5    = 12,
    API_LARK_FDSP_RATE_SRC_FINT6_7    = 13,
    #endif
    API_LARK_FDSP_RATE_SRC_ASRCI      = 14,
    API_LARK_FDSP_RATE_SRC_FIXED      = 15
} adi_lark_fdsp_rate_src_e;

/*!
 * @brief  enum for multi-purpose pin modes, for both lark and lark-lite
 */
typedef enum{
    API_LARK_MP_MODE_NORMAL     = 0,                            /*!< Normal(default function */
    #ifdef LARK_SDK
    API_LARK_MP_MODE_GPI        = 1,                            /*!< General purpose input */
    API_LARK_MP_MODE_GPO_REG    = 2,                            /*!< General purpose output from register */
    API_LARK_MP_MODE_GPO_TDSP   = 3,                            /*!< General purpose output from TDSP */
    API_LARK_MP_MODE_MCLKO      = 4,                            /*!< MCLK output */
    API_LARK_MP_MODE_IRQ1       = 5,                            /*!< IRQ1 output */
    API_LARK_MP_MODE_IRQ2       = 6,                            /*!< IRQ2 output */
    API_LARK_MP_MODE_IRQ3       = 7,                            /*!< IRQ3 output */
    API_LARK_MP_MODE_IRQ4       = 8,                            /*!< IRQ4 output */
    API_LARK_MP_MODE_IRQ_IN     = 9,                            /*!< Interrupt input */
    API_LARK_MP_MODE_PDM_CLK    = 10,                           /*!< PDM clock output */
    API_LARK_MP_MODE_PDM_DATA   = 11,                           /*!< PDM data output */
    API_LARK_MP_MODE_DMIC45     = 12,                           /*!< DMIC45 */
    API_LARK_MP_MODE_DMIC67     = 13,                           /*!< DMIC67 */
    API_LARK_MP_MODE_DMIC_CLK1  = 14,                           /*!< DMIC CLK1 */
    API_LARK_MP_MODE_TEST4      = 15,                           /*!< TEST4 */
    API_LARK_MP_MODE_DMIC_CLK0  = 16,                           /*!< DMIC CLK0 */
    API_LARK_MP_MODE_DMIC_DATA1 = 17,                           /*!< DMIC DATA1 */
    API_LARK_MP_MODE_DMIC_DATA3 = 18,                           /*!< DMIC DATA3 */
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_MP_MODE_GPI        = 3,                            /*!< General purpose input */
    API_LARK_MP_MODE_GPO_REG    = 4,                            /*!< General purpose output from register */
    API_LARK_MP_MODE_MCLKO      = 6,                            /*!< MCLK output */
    API_LARK_MP_MODE_IRQ1       = 7,                            /*!< IRQ1 output */
    API_LARK_MP_MODE_IRQ2       = 8,                            /*!< IRQ2 output */
    API_LARK_MP_MODE_PDM_CLK    = 9,                            /*!< PDM clock output */
    API_LARK_MP_MODE_PDM_DATA   = 10,                           /*!< PDM data output */
    #endif
} adi_lark_mp_pin_mode_e;

/*!
 * @brief  enum for multi-purpose pin's input debounce settings, for both lark and lark-lite
 */
typedef enum{
    API_LARK_MP_DEBOUNCE_NONE   = 0,                            /*!< No debounce */
    API_LARK_MP_DEBOUNCE_300US  = 1,                            /*!< 300uS (0.3mS) */
    API_LARK_MP_DEBOUNCE_600US  = 2,                            /*!< 600uS (0.6mS) */
    API_LARK_MP_DEBOUNCE_900US  = 3,                            /*!< 900uS (0.9mS) */
    API_LARK_MP_DEBOUNCE_5MS    = 4,                            /*!< 5mS */
    API_LARK_MP_DEBOUNCE_10MS   = 5,                            /*!< 10mS */
    API_LARK_MP_DEBOUNCE_20MS   = 6,                            /*!< 20mS */
} adi_lark_mp_input_debounce_e;

/*!
 * @brief  enum for EQ audio source route, for both lark and lark-lite
 */
typedef enum{
    API_LARK_EQ_ROUTE_SAI0_0      = 0,                          /*!< SAP0 channel0 */
    API_LARK_EQ_ROUTE_SAI0_1      = 1,                          /*!< SAP0 channel1 */
    API_LARK_EQ_ROUTE_SAI0_2      = 2,                          /*!< SAP0 channel2 */
    API_LARK_EQ_ROUTE_SAI0_3      = 3,                          /*!< SAP0 channel3 */
    API_LARK_EQ_ROUTE_SAI0_4      = 4,                          /*!< SAP0 channel4 */
    API_LARK_EQ_ROUTE_SAI0_5      = 5,                          /*!< SAP0 channel5 */
    API_LARK_EQ_ROUTE_SAI0_6      = 6,                          /*!< SAP0 channel6 */
    API_LARK_EQ_ROUTE_SAI0_7      = 7,                          /*!< SAP0 channel7 */
    API_LARK_EQ_ROUTE_SAI0_8      = 8,                          /*!< SAP0 channel8 */
    API_LARK_EQ_ROUTE_SAI0_9      = 9,                          /*!< SAP0 channel9 */
    API_LARK_EQ_ROUTE_SAI0_10     = 10,                         /*!< SAP0 channel10 */
    API_LARK_EQ_ROUTE_SAI0_11     = 11,                         /*!< SAP0 channel11 */
    API_LARK_EQ_ROUTE_SAI0_12     = 12,                         /*!< SAP0 channel12 */
    API_LARK_EQ_ROUTE_SAI0_13     = 13,                         /*!< SAP0 channel13 */
    API_LARK_EQ_ROUTE_SAI0_14     = 14,                         /*!< SAP0 channel14 */
    API_LARK_EQ_ROUTE_SAI0_15     = 15,                         /*!< SAP0 channel15 */
    #ifdef LARK_SDK
    API_LARK_EQ_ROUTE_SAI1_0      = 16,                         /*!< SAP1 channel0 */
    API_LARK_EQ_ROUTE_SAI1_1      = 17,                         /*!< SAP1 channel1 */
    API_LARK_EQ_ROUTE_SAI1_2      = 18,                         /*!< SAP1 channel2 */
    API_LARK_EQ_ROUTE_SAI1_3      = 19,                         /*!< SAP1 channel3 */
    API_LARK_EQ_ROUTE_SAI1_4      = 20,                         /*!< SAP1 channel4 */
    API_LARK_EQ_ROUTE_SAI1_5      = 21,                         /*!< SAP1 channel5 */
    API_LARK_EQ_ROUTE_SAI1_6      = 22,                         /*!< SAP1 channel6 */
    API_LARK_EQ_ROUTE_SAI1_7      = 23,                         /*!< SAP1 channel7 */
    API_LARK_EQ_ROUTE_SAI1_8      = 24,                         /*!< SAP1 channel8 */
    API_LARK_EQ_ROUTE_SAI1_9      = 25,                         /*!< SAP1 channel9 */
    API_LARK_EQ_ROUTE_SAI1_10     = 26,                         /*!< SAP1 channel10 */
    API_LARK_EQ_ROUTE_SAI1_11     = 27,                         /*!< SAP1 channel11 */
    API_LARK_EQ_ROUTE_SAI1_12     = 28,                         /*!< SAP1 channel12 */
    API_LARK_EQ_ROUTE_SAI1_13     = 29,                         /*!< SAP1 channel13 */
    API_LARK_EQ_ROUTE_SAI1_14     = 30,                         /*!< SAP1 channel14 */
    API_LARK_EQ_ROUTE_SAI1_15     = 31,                         /*!< SAP1 channel15 */
    API_LARK_EQ_ROUTE_FDSP_0      = 32,                         /*!< FDSP channel0 */
    API_LARK_EQ_ROUTE_FDSP_1      = 33,                         /*!< FDSP channel1 */
    API_LARK_EQ_ROUTE_FDSP_2      = 34,                         /*!< FDSP channel2 */
    API_LARK_EQ_ROUTE_FDSP_3      = 35,                         /*!< FDSP channel3 */
    API_LARK_EQ_ROUTE_FDSP_4      = 36,                         /*!< FDSP channel4 */
    API_LARK_EQ_ROUTE_FDSP_5      = 37,                         /*!< FDSP channel5 */
    API_LARK_EQ_ROUTE_FDSP_6      = 38,                         /*!< FDSP channel6 */
    API_LARK_EQ_ROUTE_FDSP_7      = 39,                         /*!< FDSP channel7 */
    API_LARK_EQ_ROUTE_FDSP_8      = 40,                         /*!< FDSP channel8 */
    API_LARK_EQ_ROUTE_FDSP_9      = 41,                         /*!< FDSP channel9 */
    API_LARK_EQ_ROUTE_FDSP_10     = 42,                         /*!< FDSP channel10 */
    API_LARK_EQ_ROUTE_FDSP_11     = 43,                         /*!< FDSP channel11 */
    API_LARK_EQ_ROUTE_FDSP_12     = 44,                         /*!< FDSP channel12 */
    API_LARK_EQ_ROUTE_FDSP_13     = 45,                         /*!< FDSP channel13 */
    API_LARK_EQ_ROUTE_FDSP_14     = 46,                         /*!< FDSP channel14 */
    API_LARK_EQ_ROUTE_FDSP_15     = 47,                         /*!< FDSP channel15 */
    API_LARK_EQ_ROUTE_TDSP_0      = 48,                         /*!< TDSP channel0 */
    API_LARK_EQ_ROUTE_TDSP_1      = 49,                         /*!< TDSP channel1 */
    API_LARK_EQ_ROUTE_TDSP_2      = 50,                         /*!< TDSP channel2 */
    API_LARK_EQ_ROUTE_TDSP_3      = 51,                         /*!< TDSP channel3 */
    API_LARK_EQ_ROUTE_TDSP_4      = 52,                         /*!< TDSP channel4 */
    API_LARK_EQ_ROUTE_TDSP_5      = 53,                         /*!< TDSP channel5 */
    API_LARK_EQ_ROUTE_TDSP_6      = 54,                         /*!< TDSP channel6 */
    API_LARK_EQ_ROUTE_TDSP_7      = 55,                         /*!< TDSP channel7 */
    API_LARK_EQ_ROUTE_TDSP_8      = 56,                         /*!< TDSP channel8 */
    API_LARK_EQ_ROUTE_TDSP_9      = 57,                         /*!< TDSP channel9 */
    API_LARK_EQ_ROUTE_TDSP_10     = 58,                         /*!< TDSP channel10 */
    API_LARK_EQ_ROUTE_TDSP_11     = 59,                         /*!< TDSP channel11 */
    API_LARK_EQ_ROUTE_TDSP_12     = 60,                         /*!< TDSP channel12 */
    API_LARK_EQ_ROUTE_TDSP_13     = 61,                         /*!< TDSP channel13 */
    API_LARK_EQ_ROUTE_TDSP_14     = 62,                         /*!< TDSP channel14 */
    API_LARK_EQ_ROUTE_TDSP_15     = 63,                         /*!< TDSP channel15 */
    API_LARK_EQ_ROUTE_ASRCI_0     = 64,                         /*!< ASRCI channel0 */
    API_LARK_EQ_ROUTE_ASRCI_1     = 65,                         /*!< ASRCI channel1 */
    API_LARK_EQ_ROUTE_ASRCI_2     = 66,                         /*!< ASRCI channel2 */
    API_LARK_EQ_ROUTE_ASRCI_3     = 67,                         /*!< ASRCI channel3 */
    API_LARK_EQ_ROUTE_ADC_0       = 68,                         /*!< ADC channel0 */
    API_LARK_EQ_ROUTE_ADC_1       = 69,                         /*!< ADC channel1 */
    API_LARK_EQ_ROUTE_ADC_2       = 70,                         /*!< ADC channel2 */
    API_LARK_EQ_ROUTE_DMIC_0      = 71,                         /*!< DMIC channel0 */
    API_LARK_EQ_ROUTE_DMIC_1      = 72,                         /*!< DMIC channel1 */
    API_LARK_EQ_ROUTE_DMIC_2      = 73,                         /*!< DMIC channel2 */
    API_LARK_EQ_ROUTE_DMIC_3      = 74,                         /*!< DMIC channel3 */
    API_LARK_EQ_ROUTE_DMIC_4      = 75,                         /*!< DMIC channel4 */
    API_LARK_EQ_ROUTE_DMIC_5      = 76,                         /*!< DMIC channel5 */
    API_LARK_EQ_ROUTE_DMIC_6      = 77,                         /*!< DMIC channel6 */
    API_LARK_EQ_ROUTE_DMIC_7      = 78,                         /*!< DMIC channel7 */
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_EQ_ROUTE_FDSP_0      = 16,                         /*!< FDSP channel0 */
    API_LARK_EQ_ROUTE_FDSP_1      = 17,                         /*!< FDSP channel1 */
    API_LARK_EQ_ROUTE_FDSP_2      = 18,                         /*!< FDSP channel2 */
    API_LARK_EQ_ROUTE_FDSP_3      = 19,                         /*!< FDSP channel3 */
    API_LARK_EQ_ROUTE_FDSP_4      = 20,                         /*!< FDSP channel4 */
    API_LARK_EQ_ROUTE_FDSP_5      = 21,                         /*!< FDSP channel5 */
    API_LARK_EQ_ROUTE_FDSP_6      = 22,                         /*!< FDSP channel6 */
    API_LARK_EQ_ROUTE_FDSP_7      = 23,                         /*!< FDSP channel7 */
    API_LARK_EQ_ROUTE_FDSP_8      = 24,                         /*!< FDSP channel8 */
    API_LARK_EQ_ROUTE_FDSP_9      = 25,                         /*!< FDSP channel9 */
    API_LARK_EQ_ROUTE_FDSP_10     = 26,                         /*!< FDSP channel10 */
    API_LARK_EQ_ROUTE_FDSP_11     = 27,                         /*!< FDSP channel11 */
    API_LARK_EQ_ROUTE_FDSP_12     = 28,                         /*!< FDSP channel12 */
    API_LARK_EQ_ROUTE_FDSP_13     = 29,                         /*!< FDSP channel13 */
    API_LARK_EQ_ROUTE_FDSP_14     = 30,                         /*!< FDSP channel14 */
    API_LARK_EQ_ROUTE_FDSP_15     = 31,                         /*!< FDSP channel15 */
    API_LARK_EQ_ROUTE_ASRCI_0     = 32,                         /*!< ASRCI channel0 */
    API_LARK_EQ_ROUTE_ADC_0       = 33,                         /*!< ADC channel0 */
    API_LARK_EQ_ROUTE_ADC_1       = 34,                         /*!< ADC channel1 */
    API_LARK_EQ_ROUTE_ADC_2       = 35,                         /*!< ADC channel2 */
    #endif
} adi_lark_eq_route_e;

/*!
 * @brief  enum for pin names that use pin control settings
 */
typedef enum{
    #ifdef LARK_SDK
    API_LARK_PIN_DMIC_CLK       = 0,
    API_LARK_PIN_DMIC01         = 1,
    API_LARK_PIN_DMIC23         = 2,
    API_LARK_PIN_BCLK0          = 3,
    API_LARK_PIN_FSYNC0         = 4,
    API_LARK_PIN_SDATAO0        = 5,
    API_LARK_PIN_SDATAI0        = 6,
    API_LARK_PIN_BCLK1          = 7,
    API_LARK_PIN_FSYNC1         = 8,
    API_LARK_PIN_SDATAO1        = 9,
    API_LARK_PIN_SDATAI1        = 10,
    API_LARK_PIN_QSPIM_CLK      = 11,
    API_LARK_PIN_QSPIM_CS       = 12,
    API_LARK_PIN_QSPIM_SDIO0    = 13,
    API_LARK_PIN_QSPIM_SDIO1    = 14,
    API_LARK_PIN_QSPIM_SDIO2    = 15,
    API_LARK_PIN_QSPIM_SDIO4    = 16,
    API_LARK_PIN_UART_TX        = 17,
    API_LARK_PIN_UART_RX        = 18,
    API_LARK_PIN_SELFBOOT       = 19,
    API_LARK_PIN_IRQ            = 20,
    API_LARK_PIN_ROM_BOOT_MODE  = 21,
    API_LARK_PIN_TCK            = 22,
    API_LARK_PIN_TMS            = 23,
    API_LARK_PIN_TDO            = 24,
    API_LARK_PIN_TDI            = 25,
    API_LARK_PIN_I2C_SPI        = 26,
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_PIN_FSYNC0         = 0,
    API_LARK_PIN_BCLK0          = 1,
    API_LARK_PIN_SDATAO0        = 2,
    API_LARK_PIN_SDATAI0        = 3,
    API_LARK_PIN_IRQ            = 4,
    API_LARK_PIN_MCLKO          = 5,
    API_LARK_PIN_I2C_SPI        = 12,
    #endif
} adi_lark_pin_name_e;

/*!
 * @brief  enum for pin drive settings
 */
typedef enum{
    API_LARK_PIN_DR2MA          = 0,
    API_LARK_PIN_DR4MA          = 1,
    API_LARK_PIN_DR8MA          = 2,
    API_LARK_PIN_DR12MA         = 3,
} adi_lark_pin_drive_e;

/*!
 * @brief  enum for pin slew settings
 */
typedef enum{
    API_LARK_PIN_SLEW_FAST      = 0,
    API_LARK_PIN_SLEW_SLOW      = 1,
} adi_lark_pin_slew_e;

/*!
 * @brief  enum for pin pull settings
 */
typedef enum{
    API_LARK_PIN_PULL_DOWN      = 0,
    API_LARK_PIN_PULL_UP        = 1,
    API_LARK_PIN_PULL_NONE      = 3
} adi_lark_pin_pull_e;

/*!
 * @brief  struct for fifo information, for lark only.
 */
typedef struct
{
    uint8_t  block_id;                                          /*!< FIFO SRAM block */
                                                                /*!< id 0: block from 0x6001_0000 to 0x6001_FFFF */
                                                                /*!< id 1: block from 0x6002_0000 to 0x6002_FFFF */
                                                                /*!< id 2: block from 0x6003_0000 to 0x6003_7FFF */
    uint8_t  channel_id;                                        /*!< FIFO channel id. 4 channel for each block */
    uint8_t  base_offset_of_256B;                               /*!< FIFO channel start address offset of 256 bytes */
                                                                /*!< The FIFO start address of current channel = (block start addr + base_offset_of_256B * 256) */
    uint16_t len_of_256B;                                       /*!< FIFO channel length of 256 bytes. The total byte length is (len_of_256B * 256 bytes) */
    uint8_t  width;                                             /*!< FIFO channel bit width: 16 or 32. 16 for 16-bit access width, 32 for 32-bit access width */
} adi_lark_ffsram_fifo_info_t;

/*!
 * @brief  enum for ffsram interrupt source, for lark only.
 */
typedef enum{
    API_LARK_FFSRAM_INT_FIFO_EMPTY      = 0,                    /*!< FIFO is empty */
    API_LARK_FFSRAM_INT_FIFO_FULL       = 1,                    /*!< FIFO is full */
    API_LARK_FFSRAM_INT_FIFO_ALME       = 2,                    /*!< FIFO is almost empty, meeting almost empty threshold */
    API_LARK_FFSRAM_INT_FIFO_ALMF       = 3,                    /*!< FIFO is almost full, meeting almost full threshold */
    API_LARK_FFSRAM_INT_FIFO_UDFLOW     = 4,                    /*!< FIFO is underflow, read pointer exceeds write pointer */
    API_LARK_FFSRAM_INT_FIFO_OVFLOW     = 5,                    /*!< FIFO is overflow, write pointer exceeds read pointer */
    API_LARK_FFSRAM_INT_FIFO_COUNT      = 6                     /*!< FIFO interrupt source count */
} adi_lark_ffsram_int_src_e;

/*!
 * @brief  enum for fdec channel pairs
 */
typedef enum{
    API_LARK_FDEC_PAIR_CHNL_0_1 = 0,
    API_LARK_FDEC_PAIR_CHNL_2_3 = 1,
    #ifdef LARK_SDK
    API_LARK_FDEC_PAIR_CHNL_4_5 = 2,
    API_LARK_FDEC_PAIR_CHNL_6_7 = 3,
    #endif
} adi_lark_fdec_pair_chnl_e;

/*!
 * @brief  enum for fdec input sample rate
 */
typedef enum{
    API_LARK_FDEC_IN_FS_24K     = 1,                            /*!< input sample rate 24K, start from 1 */
    API_LARK_FDEC_IN_FS_48K     = 2,
    API_LARK_FDEC_IN_FS_96K     = 3,
    API_LARK_FDEC_IN_FS_192K    = 4,
    API_LARK_FDEC_IN_FS_384K    = 5,
    API_LARK_FDEC_IN_FS_768K    = 6,
} adi_lark_fdec_in_fs_e;

/*!
 * @brief  enum for fdec output sample rate
 */
typedef enum{
    API_LARK_FDEC_OUT_FS_12K    = 0,
    API_LARK_FDEC_OUT_FS_24K    = 1,
    API_LARK_FDEC_OUT_FS_48K    = 2,
    API_LARK_FDEC_OUT_FS_96K    = 3,
    API_LARK_FDEC_OUT_FS_192K   = 4,
    API_LARK_FDEC_OUT_FS_384K   = 5,

    API_LARK_FDEC_OUT_FS_8K     = 8,
    API_LARK_FDEC_OUT_FS_16K    = 9,
} adi_lark_fdec_out_fs_e;

/*!
 * @brief  enum for fdec route channel
 */
typedef enum{
    #ifdef LARK_SDK
    API_LARK_FDEC_ROUTE_FDSP0   = 0,
    API_LARK_FDEC_ROUTE_FDSP1   = 1,
    API_LARK_FDEC_ROUTE_FDSP2   = 2,
    API_LARK_FDEC_ROUTE_FDSP3   = 3,
    API_LARK_FDEC_ROUTE_FDSP4   = 4,
    API_LARK_FDEC_ROUTE_FDSP5   = 5,
    API_LARK_FDEC_ROUTE_FDSP6   = 6,
    API_LARK_FDEC_ROUTE_FDSP7   = 7,
    API_LARK_FDEC_ROUTE_FDSP8   = 8,
    API_LARK_FDEC_ROUTE_FDSP9   = 9,
    API_LARK_FDEC_ROUTE_FDSP10  = 10,
    API_LARK_FDEC_ROUTE_FDSP11  = 11,
    API_LARK_FDEC_ROUTE_FDSP12  = 12,
    API_LARK_FDEC_ROUTE_FDSP13  = 13,
    API_LARK_FDEC_ROUTE_FDSP14  = 14,
    API_LARK_FDEC_ROUTE_FDSP15  = 15,
    API_LARK_FDEC_ROUTE_TDSP0   = 16,
    API_LARK_FDEC_ROUTE_TDSP1   = 17,
    API_LARK_FDEC_ROUTE_TDSP2   = 18,
    API_LARK_FDEC_ROUTE_TDSP3   = 19,
    API_LARK_FDEC_ROUTE_TDSP4   = 20,
    API_LARK_FDEC_ROUTE_TDSP5   = 21,
    API_LARK_FDEC_ROUTE_TDSP6   = 22,
    API_LARK_FDEC_ROUTE_TDSP7   = 23,
    API_LARK_FDEC_ROUTE_TDSP8   = 24,
    API_LARK_FDEC_ROUTE_TDSP9   = 25,
    API_LARK_FDEC_ROUTE_TDSP10  = 26,
    API_LARK_FDEC_ROUTE_TDSP11  = 27,
    API_LARK_FDEC_ROUTE_TDSP12  = 28,
    API_LARK_FDEC_ROUTE_TDSP13  = 29,
    API_LARK_FDEC_ROUTE_TDSP14  = 30,
    API_LARK_FDEC_ROUTE_TDSP15  = 31,
    API_LARK_FDEC_ROUTE_ASRCI0  = 32,
    API_LARK_FDEC_ROUTE_ASRCI1  = 33,
    API_LARK_FDEC_ROUTE_ASRCI2  = 34,
    API_LARK_FDEC_ROUTE_ASRCI3  = 35,
    API_LARK_FDEC_ROUTE_ADC0    = 36,
    API_LARK_FDEC_ROUTE_ADC1    = 37,
    API_LARK_FDEC_ROUTE_ADC2    = 38,
    API_LARK_FDEC_ROUTE_DMIC0   = 39,
    API_LARK_FDEC_ROUTE_DMIC1   = 40,
    API_LARK_FDEC_ROUTE_DMIC2   = 41,
    API_LARK_FDEC_ROUTE_DMIC3   = 42,
    API_LARK_FDEC_ROUTE_EQ      = 43,
    API_LARK_FDEC_ROUTE_RSV1    = 44,
    API_LARK_FDEC_ROUTE_RSV2    = 45,
    API_LARK_FDEC_ROUTE_RSV3    = 46,
    API_LARK_FDEC_ROUTE_DMIC4   = 47,
    API_LARK_FDEC_ROUTE_DMIC5   = 48,
    API_LARK_FDEC_ROUTE_DMIC6   = 49,
    API_LARK_FDEC_ROUTE_DMIC7   = 50,
    API_LARK_FDEC_ROUTE_COUNT   = 51,
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_FDEC_ROUTE_ADC0    = 0,
    API_LARK_FDEC_ROUTE_ADC1    = 1,
    API_LARK_FDEC_ROUTE_ADC2    = 2,
    API_LARK_FDEC_ROUTE_ASRCI0  = 3,
    API_LARK_FDEC_ROUTE_EQ      = 4,
    API_LARK_FDEC_ROUTE_FDSP0   = 5,
    API_LARK_FDEC_ROUTE_FDSP1   = 6,
    API_LARK_FDEC_ROUTE_FDSP2   = 7,
    API_LARK_FDEC_ROUTE_FDSP3   = 8,
    API_LARK_FDEC_ROUTE_FDSP4   = 9,
    API_LARK_FDEC_ROUTE_FDSP5   = 10,
    API_LARK_FDEC_ROUTE_FDSP6   = 11,
    API_LARK_FDEC_ROUTE_FDSP7   = 12,
    API_LARK_FDEC_ROUTE_FDSP8   = 13,
    API_LARK_FDEC_ROUTE_FDSP9   = 14,
    API_LARK_FDEC_ROUTE_FDSP10  = 15,
    API_LARK_FDEC_ROUTE_FDSP11  = 16,
    API_LARK_FDEC_ROUTE_FDSP12  = 17,
    API_LARK_FDEC_ROUTE_FDSP13  = 18,
    API_LARK_FDEC_ROUTE_FDSP14  = 19,
    API_LARK_FDEC_ROUTE_FDSP15  = 20,
    API_LARK_FDEC_ROUTE_COUNT   = 21,
    #endif
} adi_lark_fdec_route_chnl_e;


/*!
 * @brief  enum for fint channel pairs
 */
typedef enum{
    API_LARK_FINT_PAIR_CHNL_0_1 = 0,
    #ifdef LARK_SDK
    API_LARK_FINT_PAIR_CHNL_2_3 = 1,
    API_LARK_FINT_PAIR_CHNL_4_5 = 2,
    API_LARK_FINT_PAIR_CHNL_6_7 = 3,
    #endif
} adi_lark_fint_pair_chnl_e;

/*!
 * @brief  enum for fint output sample rate
 */
typedef enum{
    API_LARK_FINT_OUT_FS_24K    = 1,                            /*!< input sample rate 24K, start from 1 */
    API_LARK_FINT_OUT_FS_48K    = 2,
    API_LARK_FINT_OUT_FS_96K    = 3,
    API_LARK_FINT_OUT_FS_192K   = 4,
    API_LARK_FINT_OUT_FS_384K   = 5,
    API_LARK_FINT_OUT_FS_768K   = 6,
} adi_lark_fint_out_fs_e;

/*!
 * @brief  enum for fint input sample rate
 */
typedef enum{
    API_LARK_FINT_IN_FS_12K     = 0,
    API_LARK_FINT_IN_FS_24K     = 1,
    API_LARK_FINT_IN_FS_48K     = 2,
    API_LARK_FINT_IN_FS_96K     = 3,
    API_LARK_FINT_IN_FS_192K    = 4,
    API_LARK_FINT_IN_FS_384K    = 5,
    API_LARK_FINT_IN_FS_8K      = 8,
    API_LARK_FINT_IN_FS_16K     = 9,
} adi_lark_fint_in_fs_e;

/*!
 * @brief  enum for fint route channel
 */
typedef enum{
    #ifdef LARK_SDK
    API_LARK_FINT_ROUTE_SAI0_0  = 0,
    API_LARK_FINT_ROUTE_SAI0_1  = 1,
    API_LARK_FINT_ROUTE_SAI0_2  = 2,
    API_LARK_FINT_ROUTE_SAI0_3  = 3,
    API_LARK_FINT_ROUTE_SAI0_4  = 4,
    API_LARK_FINT_ROUTE_SAI0_5  = 5,
    API_LARK_FINT_ROUTE_SAI0_6  = 6,
    API_LARK_FINT_ROUTE_SAI0_7  = 7,
    API_LARK_FINT_ROUTE_SAI0_8  = 8,
    API_LARK_FINT_ROUTE_SAI0_9  = 9,
    API_LARK_FINT_ROUTE_SAI0_10 = 10,
    API_LARK_FINT_ROUTE_SAI0_11 = 11,
    API_LARK_FINT_ROUTE_SAI0_12 = 12,
    API_LARK_FINT_ROUTE_SAI0_13 = 13,
    API_LARK_FINT_ROUTE_SAI0_14 = 14,
    API_LARK_FINT_ROUTE_SAI0_15 = 15,
    API_LARK_FINT_ROUTE_SAI1_0  = 16,
    API_LARK_FINT_ROUTE_SAI1_1  = 17,
    API_LARK_FINT_ROUTE_SAI1_2  = 18,
    API_LARK_FINT_ROUTE_SAI1_3  = 19,
    API_LARK_FINT_ROUTE_SAI1_4  = 20,
    API_LARK_FINT_ROUTE_SAI1_5  = 21,
    API_LARK_FINT_ROUTE_SAI1_6  = 22,
    API_LARK_FINT_ROUTE_SAI1_7  = 23,
    API_LARK_FINT_ROUTE_SAI1_8  = 24,
    API_LARK_FINT_ROUTE_SAI1_9  = 25,
    API_LARK_FINT_ROUTE_SAI1_10 = 26,
    API_LARK_FINT_ROUTE_SAI1_11 = 27,
    API_LARK_FINT_ROUTE_SAI1_12 = 28,
    API_LARK_FINT_ROUTE_SAI1_13 = 29,
    API_LARK_FINT_ROUTE_SAI1_14 = 30,
    API_LARK_FINT_ROUTE_SAI1_15 = 31,
    API_LARK_FINT_ROUTE_FDSP0   = 32,
    API_LARK_FINT_ROUTE_FDSP1   = 33,
    API_LARK_FINT_ROUTE_FDSP2   = 34,
    API_LARK_FINT_ROUTE_FDSP3   = 35,
    API_LARK_FINT_ROUTE_FDSP4   = 36,
    API_LARK_FINT_ROUTE_FDSP5   = 37,
    API_LARK_FINT_ROUTE_FDSP6   = 38,
    API_LARK_FINT_ROUTE_FDSP7   = 39,
    API_LARK_FINT_ROUTE_FDSP8   = 40,
    API_LARK_FINT_ROUTE_FDSP9   = 41,
    API_LARK_FINT_ROUTE_FDSP10  = 42,
    API_LARK_FINT_ROUTE_FDSP11  = 43,
    API_LARK_FINT_ROUTE_FDSP12  = 44,
    API_LARK_FINT_ROUTE_FDSP13  = 45,
    API_LARK_FINT_ROUTE_FDSP14  = 46,
    API_LARK_FINT_ROUTE_FDSP15  = 47,
    API_LARK_FINT_ROUTE_TDSP0   = 48,
    API_LARK_FINT_ROUTE_TDSP1   = 49,
    API_LARK_FINT_ROUTE_TDSP2   = 50,
    API_LARK_FINT_ROUTE_TDSP3   = 51,
    API_LARK_FINT_ROUTE_TDSP4   = 52,
    API_LARK_FINT_ROUTE_TDSP5   = 53,
    API_LARK_FINT_ROUTE_TDSP6   = 54,
    API_LARK_FINT_ROUTE_TDSP7   = 55,
    API_LARK_FINT_ROUTE_TDSP8   = 56,
    API_LARK_FINT_ROUTE_TDSP9   = 57,
    API_LARK_FINT_ROUTE_TDSP10  = 58,
    API_LARK_FINT_ROUTE_TDSP11  = 59,
    API_LARK_FINT_ROUTE_TDSP12  = 60,
    API_LARK_FINT_ROUTE_TDSP13  = 61,
    API_LARK_FINT_ROUTE_TDSP14  = 62,
    API_LARK_FINT_ROUTE_TDSP15  = 63,
    API_LARK_FINT_ROUTE_ASRCI0  = 64,
    API_LARK_FINT_ROUTE_ASRCI1  = 65,
    API_LARK_FINT_ROUTE_ASRCI2  = 66,
    API_LARK_FINT_ROUTE_ASRCI3  = 67,
    API_LARK_FINT_ROUTE_ADC0    = 68,
    API_LARK_FINT_ROUTE_ADC1    = 69,
    API_LARK_FINT_ROUTE_ADC2    = 70,
    API_LARK_FINT_ROUTE_DMIC0   = 71,
    API_LARK_FINT_ROUTE_DMIC1   = 72,
    API_LARK_FINT_ROUTE_DMIC2   = 73,
    API_LARK_FINT_ROUTE_DMIC3   = 74,
    API_LARK_FINT_ROUTE_EQ      = 75,
    API_LARK_FINT_ROUTE_RSV1    = 76,
    API_LARK_FINT_ROUTE_RSV2    = 77,
    API_LARK_FINT_ROUTE_RSV3    = 78,
    API_LARK_FINT_ROUTE_DMIC4   = 79,
    API_LARK_FINT_ROUTE_DMIC5   = 80,
    API_LARK_FINT_ROUTE_DMIC6   = 81,
    API_LARK_FINT_ROUTE_DMIC7   = 82,
    API_LARK_FINT_ROUTE_COUNT   = 83,
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_FINT_ROUTE_ADC0    = 0,
    API_LARK_FINT_ROUTE_ADC1    = 1,
    API_LARK_FINT_ROUTE_ADC2    = 2,
    API_LARK_FINT_ROUTE_ASRCI0  = 3,
    API_LARK_FINT_ROUTE_EQ      = 4,
    API_LARK_FINT_ROUTE_FDSP0   = 5,
    API_LARK_FINT_ROUTE_FDSP1   = 6,
    API_LARK_FINT_ROUTE_FDSP2   = 7,
    API_LARK_FINT_ROUTE_FDSP3   = 8,
    API_LARK_FINT_ROUTE_FDSP4   = 9,
    API_LARK_FINT_ROUTE_FDSP5   = 10,
    API_LARK_FINT_ROUTE_FDSP6   = 11,
    API_LARK_FINT_ROUTE_FDSP7   = 12,
    API_LARK_FINT_ROUTE_FDSP8   = 13,
    API_LARK_FINT_ROUTE_FDSP9   = 14,
    API_LARK_FINT_ROUTE_FDSP10  = 15,
    API_LARK_FINT_ROUTE_FDSP11  = 16,
    API_LARK_FINT_ROUTE_FDSP12  = 17,
    API_LARK_FINT_ROUTE_FDSP13  = 18,
    API_LARK_FINT_ROUTE_FDSP14  = 19,
    API_LARK_FINT_ROUTE_FDSP15  = 20,
    API_LARK_FINT_ROUTE_SAI0_0  = 21,
    API_LARK_FINT_ROUTE_SAI0_1  = 22,
    API_LARK_FINT_ROUTE_SAI0_2  = 23,
    API_LARK_FINT_ROUTE_SAI0_3  = 24,
    API_LARK_FINT_ROUTE_SAI0_4  = 25,
    API_LARK_FINT_ROUTE_SAI0_5  = 26,
    API_LARK_FINT_ROUTE_SAI0_6  = 27,
    API_LARK_FINT_ROUTE_SAI0_7  = 28,
    API_LARK_FINT_ROUTE_SAI0_8  = 29,
    API_LARK_FINT_ROUTE_SAI0_9  = 30,
    API_LARK_FINT_ROUTE_SAI0_10 = 31,
    API_LARK_FINT_ROUTE_SAI0_11 = 32,
    API_LARK_FINT_ROUTE_SAI0_12 = 33,
    API_LARK_FINT_ROUTE_SAI0_13 = 34,
    API_LARK_FINT_ROUTE_SAI0_14 = 35,
    API_LARK_FINT_ROUTE_SAI0_15 = 36,
    API_LARK_FINT_ROUTE_COUNT   = 37,
    #endif
} adi_lark_fint_route_chnl_e;

/*!
 * @brief  enum for asrc channel
 */
typedef enum{
    API_LARK_ASRC_CHNL_ASRCI0   = 0, 
    #ifdef LARK_SDK
    API_LARK_ASRC_CHNL_ASRCI1   = 1,
    API_LARK_ASRC_CHNL_ASRCI2   = 2,
    API_LARK_ASRC_CHNL_ASRCI3   = 3,
    #endif
    API_LARK_ASRC_CHNL_ASRCO0   = 4,
    API_LARK_ASRC_CHNL_ASRCO1   = 5,
    API_LARK_ASRC_CHNL_ASRCO2   = 6,
    #ifdef LARK_SDK
    API_LARK_ASRC_CHNL_ASRCO3   = 7,
    #endif
    API_LARK_ASRC_CHNL_COUNT
} adi_lark_asrc_chnl_e;

/*!
 * @brief  enum for asrci output and asrco input frequency
 */
typedef enum{
    API_LARK_ASRC_FS_12K    = 0,
    API_LARK_ASRC_FS_24K    = 1,
    API_LARK_ASRC_FS_48K    = 2,
    API_LARK_ASRC_FS_96K    = 3,
    API_LARK_ASRC_FS_192K   = 4,
} adi_lark_asrc_fs_e;

/*!
 * @brief  enum for asrco route channel
 */
typedef enum{
    #ifdef LARK_SDK
    API_LARK_ASRCO_ROUTE_FDSP0  = 0,
    API_LARK_ASRCO_ROUTE_FDSP1  = 1,
    API_LARK_ASRCO_ROUTE_FDSP2  = 2,
    API_LARK_ASRCO_ROUTE_FDSP3  = 3,
    API_LARK_ASRCO_ROUTE_FDSP4  = 4,
    API_LARK_ASRCO_ROUTE_FDSP5  = 5,
    API_LARK_ASRCO_ROUTE_FDSP6  = 6,
    API_LARK_ASRCO_ROUTE_FDSP7  = 7,
    API_LARK_ASRCO_ROUTE_FDSP8  = 8,
    API_LARK_ASRCO_ROUTE_FDSP9  = 9,
    API_LARK_ASRCO_ROUTE_FDSP10 = 10,
    API_LARK_ASRCO_ROUTE_FDSP11 = 11,
    API_LARK_ASRCO_ROUTE_FDSP12 = 12,
    API_LARK_ASRCO_ROUTE_FDSP13 = 13,
    API_LARK_ASRCO_ROUTE_FDSP14 = 14,
    API_LARK_ASRCO_ROUTE_FDSP15 = 15,
    API_LARK_ASRCO_ROUTE_TDSP0  = 16,
    API_LARK_ASRCO_ROUTE_TDSP1  = 17,
    API_LARK_ASRCO_ROUTE_TDSP2  = 18,
    API_LARK_ASRCO_ROUTE_TDSP3  = 19,
    API_LARK_ASRCO_ROUTE_TDSP4  = 20,
    API_LARK_ASRCO_ROUTE_TDSP5  = 21,
    API_LARK_ASRCO_ROUTE_TDSP6  = 22,
    API_LARK_ASRCO_ROUTE_TDSP7  = 23,
    API_LARK_ASRCO_ROUTE_TDSP8  = 24,
    API_LARK_ASRCO_ROUTE_TDSP9  = 25,
    API_LARK_ASRCO_ROUTE_TDSP10 = 26,
    API_LARK_ASRCO_ROUTE_TDSP11 = 27,
    API_LARK_ASRCO_ROUTE_TDSP12 = 28,
    API_LARK_ASRCO_ROUTE_TDSP13 = 29,
    API_LARK_ASRCO_ROUTE_TDSP14 = 30,
    API_LARK_ASRCO_ROUTE_TDSP15 = 31,
    API_LARK_ASRCO_ROUTE_ADC0   = 32,
    API_LARK_ASRCO_ROUTE_ADC1   = 33,
    API_LARK_ASRCO_ROUTE_ADC2   = 34,
    API_LARK_ASRCO_ROUTE_DMIC0  = 35,
    API_LARK_ASRCO_ROUTE_DMIC1  = 36,
    API_LARK_ASRCO_ROUTE_DMIC2  = 37,
    API_LARK_ASRCO_ROUTE_DMIC3  = 38,
    API_LARK_ASRCO_ROUTE_FDEC0  = 39,
    API_LARK_ASRCO_ROUTE_FDEC1  = 40,
    API_LARK_ASRCO_ROUTE_FDEC2  = 41,
    API_LARK_ASRCO_ROUTE_FDEC3  = 42,
    API_LARK_ASRCO_ROUTE_FDEC4  = 43,
    API_LARK_ASRCO_ROUTE_FDEC5  = 44,
    API_LARK_ASRCO_ROUTE_FDEC6  = 45,
    API_LARK_ASRCO_ROUTE_FDEC7  = 46,
    API_LARK_ASRCO_ROUTE_EQ     = 47,
    API_LARK_ASRCO_ROUTE_RSV1   = 48,
    API_LARK_ASRCO_ROUTE_RSV2   = 49,
    API_LARK_ASRCO_ROUTE_RSV3   = 50,
    API_LARK_ASRCO_ROUTE_DMIC4  = 51,
    API_LARK_ASRCO_ROUTE_DMIC5  = 52,
    API_LARK_ASRCO_ROUTE_DMIC6  = 53,
    API_LARK_ASRCO_ROUTE_DMIC7  = 54,
    API_LARK_ASRCO_ROUTE_COUNT  = 55,
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_ASRCO_ROUTE_ADC0   = 0,
    API_LARK_ASRCO_ROUTE_ADC1   = 1,
    API_LARK_ASRCO_ROUTE_ADC2   = 2,
    API_LARK_ASRCO_ROUTE_EQ     = 3,
    API_LARK_ASRCO_ROUTE_FDEC0  = 4,
    API_LARK_ASRCO_ROUTE_FDEC1  = 5,
    API_LARK_ASRCO_ROUTE_FDEC2  = 6,
    API_LARK_ASRCO_ROUTE_FDEC3  = 7,
    API_LARK_ASRCO_ROUTE_FDSP0  = 8,
    API_LARK_ASRCO_ROUTE_FDSP1  = 9,
    API_LARK_ASRCO_ROUTE_FDSP2  = 10,
    API_LARK_ASRCO_ROUTE_FDSP3  = 11,
    API_LARK_ASRCO_ROUTE_FDSP4  = 12,
    API_LARK_ASRCO_ROUTE_FDSP5  = 13,
    API_LARK_ASRCO_ROUTE_FDSP6  = 14,
    API_LARK_ASRCO_ROUTE_FDSP7  = 15,
    API_LARK_ASRCO_ROUTE_FDSP8  = 16,
    API_LARK_ASRCO_ROUTE_FDSP9  = 17,
    API_LARK_ASRCO_ROUTE_FDSP10 = 18,
    API_LARK_ASRCO_ROUTE_FDSP11 = 19,
    API_LARK_ASRCO_ROUTE_FDSP12 = 20,
    API_LARK_ASRCO_ROUTE_FDSP13 = 21,
    API_LARK_ASRCO_ROUTE_FDSP14 = 22,
    API_LARK_ASRCO_ROUTE_FDSP15 = 23,
    API_LARK_ASRCO_ROUTE_COUNT  = 24,
    #endif
} adi_lark_ascro_route_chnl_e;

/*!
 * @brief  enum for adc clock rate
 */
typedef enum{
    API_LARK_ADC_CLK_RATE_2048KHz  = 0,
    API_LARK_ADC_CLK_RATE_6144KHz  = 1,
} adi_lark_adc_clk_rate_e;

/*!
 * @brief  enum for adc sample rate
 */
typedef enum{
    API_LARK_ADC_SAMPLE_RATE_12KHz  = 0,
    API_LARK_ADC_SAMPLE_RATE_24KHz  = 1,
    API_LARK_ADC_SAMPLE_RATE_48KHz  = 2,
    API_LARK_ADC_SAMPLE_RATE_96KHz  = 3,
    API_LARK_ADC_SAMPLE_RATE_192KHz = 4,
    API_LARK_ADC_SAMPLE_RATE_384KHz = 5,
    API_LARK_ADC_SAMPLE_RATE_768KHz = 6
} adi_lark_adc_sample_rate_e;

/*!
 * @brief  enum for adc ibias
 */
typedef enum{
    API_LARK_ADC_IBIAS_NORMAL       = 0,                         /*!< Normal operation (default) */
    API_LARK_ADC_IBIAS_HIGHEST      = 1,                         /*!< Highest power consumption mode */
    API_LARK_ADC_IBIAS_HIGH_PERF    = 2,                         /*!< Enhanced performance */
    API_LARK_ADC_IBIAS_LOWEST       = 3,                         /*!< Extreme power saving */
    API_LARK_ADC_IBIAS_TBD1         = 4,                         /*!< Lowest Power */
    API_LARK_ADC_IBIAS_TBD2         = 5,                         /*!< Lowest Power */
    API_LARK_ADC_IBIAS_TBD3         = 6,                         /*!< Lowest Power */
    API_LARK_ADC_IBIAS_BYPASS       = 7                          /*!< Bypass the lookup table and use test mode redgsters */
} adi_lark_adc_ibias_e;

/*!
 * @brief  enum for adc high pass filter
 */
typedef enum{
    API_LARK_ADC_HPF_DISABLE        = 0,                         /*!< ADC High Pass Filter Off */
    API_LARK_ADC_HPF_CUTOFF_1HZ     = 1,                         /*!< ADC High Pass Filter, Cut Off Freq. 1Hz */
    API_LARK_ADC_HPF_CUTOFF_4HZ     = 2,                         /*!< ADC High Pass Filter, Cut Off Freq. 4Hz */
    API_LARK_ADC_HPF_CUTOFF_8HZ     = 3                          /*!< ADC High Pass Filter, Cut Off Freq. 5Hz */
} adi_lark_adc_hpf_e;

/*!
 * @brief  enum for adc input precharge
 */
typedef enum{
    API_LARK_ADC_PRECHARGE_DISABLE  = 0,
    API_LARK_ADC_PRECHARGE_5MS      = 1,
    API_LARK_ADC_PRECHARGE_10MS     = 2,
    API_LARK_ADC_PRECHARGE_20MS     = 3,
    API_LARK_ADC_PRECHARGE_30MS     = 4,
    API_LARK_ADC_PRECHARGE_40MS     = 5,
    API_LARK_ADC_PRECHARGE_50MS     = 6,
    API_LARK_ADC_PRECHARGE_60MS     = 7,
    API_LARK_ADC_PRECHARGE_80MS     = 8,
    API_LARK_ADC_PRECHARGE_100MS    = 9,
    API_LARK_ADC_PRECHARGE_125MS    = 10,
    API_LARK_ADC_PRECHARGE_150MS    = 11,
    API_LARK_ADC_PRECHARGE_200MS    = 12,
    API_LARK_ADC_PRECHARGE_250MS    = 13,
    API_LARK_ADC_PRECHARGE_300MS    = 14,
    API_LARK_ADC_PRECHARGE_400MS    = 15
} adi_lark_adc_precharge_time_e;

/*!
 * @brief  enum for adc dither level
 */
typedef enum{
    API_LARK_ADC_DITHER_DISABLE    = 0,                          /*!< Dither Off */
    API_LARK_ADC_DITHER_QUARTE_LSB = 1,                          /*!< 1/4 lsb */
    API_LARK_ADC_DITHER_HALF_LSB   = 2                           /*!< 1/2 lsb */
} adi_lark_adc_dither_level_e;

/*!
 * @brief  enum for DAC sample rate
 */
typedef enum{
    API_LARK_DAC_SAMPLE_RATE_12KHz  = 0,
    API_LARK_DAC_SAMPLE_RATE_24KHz  = 1,
    API_LARK_DAC_SAMPLE_RATE_48KHz  = 2,
    API_LARK_DAC_SAMPLE_RATE_96KHz  = 3,
    API_LARK_DAC_SAMPLE_RATE_192KHz = 4,
    API_LARK_DAC_SAMPLE_RATE_384KHz = 5,
    API_LARK_DAC_SAMPLE_RATE_768KHz = 6
} adi_lark_dac_sample_rate_e;

/*!
 * @brief  enum for dac ibias
 */
typedef enum{
    API_LARK_DAC_IBIAS_NORMAL       = 0,                         /*!< Normal operation (default) */
    API_LARK_DAC_IBIAS_LOWEST       = 1,                         /*!< Extreme power saving */
    API_LARK_DAC_IBIAS_HIGH         = 2,                         /*!< Enhanced performance */
    API_LARK_DAC_IBIAS_LOW          = 3                          /*!< Power saving */
} adi_lark_dac_ibias_e;

/*!
 * @brief  enum for DAC input routing
 */
typedef enum{
    #ifdef LARK_SDK
    API_LARK_DAC_ROUTE_SAI0_00      = 0,
    API_LARK_DAC_ROUTE_SAI0_01      = 1,
    API_LARK_DAC_ROUTE_SAI0_02      = 2,
    API_LARK_DAC_ROUTE_SAI0_03      = 3,
    API_LARK_DAC_ROUTE_SAI0_04      = 4,
    API_LARK_DAC_ROUTE_SAI0_05      = 5,
    API_LARK_DAC_ROUTE_SAI0_06      = 6,
    API_LARK_DAC_ROUTE_SAI0_07      = 7,
    API_LARK_DAC_ROUTE_SAI0_08      = 8,
    API_LARK_DAC_ROUTE_SAI0_09      = 9,
    API_LARK_DAC_ROUTE_SAI0_10      = 10,
    API_LARK_DAC_ROUTE_SAI0_11      = 11,
    API_LARK_DAC_ROUTE_SAI0_12      = 12,
    API_LARK_DAC_ROUTE_SAI0_13      = 13,
    API_LARK_DAC_ROUTE_SAI0_14      = 14,
    API_LARK_DAC_ROUTE_SAI0_15      = 15,
    API_LARK_DAC_ROUTE_SAI1_00      = 16,
    API_LARK_DAC_ROUTE_SAI1_01      = 17,
    API_LARK_DAC_ROUTE_SAI1_02      = 18,
    API_LARK_DAC_ROUTE_SAI1_03      = 19,
    API_LARK_DAC_ROUTE_SAI1_04      = 20,
    API_LARK_DAC_ROUTE_SAI1_05      = 21,
    API_LARK_DAC_ROUTE_SAI1_06      = 22,
    API_LARK_DAC_ROUTE_SAI1_07      = 23,
    API_LARK_DAC_ROUTE_SAI1_08      = 24,
    API_LARK_DAC_ROUTE_SAI1_09      = 25,
    API_LARK_DAC_ROUTE_SAI1_10      = 26,
    API_LARK_DAC_ROUTE_SAI1_11      = 27,
    API_LARK_DAC_ROUTE_SAI1_12      = 28,
    API_LARK_DAC_ROUTE_SAI1_13      = 29,
    API_LARK_DAC_ROUTE_SAI1_14      = 30,
    API_LARK_DAC_ROUTE_SAI1_15      = 31,
    API_LARK_DAC_ROUTE_FDSP0        = 32,
    API_LARK_DAC_ROUTE_FDSP1        = 33,
    API_LARK_DAC_ROUTE_FDSP2        = 34,
    API_LARK_DAC_ROUTE_FDSP3        = 35,
    API_LARK_DAC_ROUTE_FDSP4        = 36,
    API_LARK_DAC_ROUTE_FDSP5        = 37,
    API_LARK_DAC_ROUTE_FDSP6        = 38,
    API_LARK_DAC_ROUTE_FDSP7        = 39,
    API_LARK_DAC_ROUTE_FDSP8        = 40,
    API_LARK_DAC_ROUTE_FDSP9        = 41,
    API_LARK_DAC_ROUTE_FDSP10       = 42,
    API_LARK_DAC_ROUTE_FDSP11       = 43,
    API_LARK_DAC_ROUTE_FDSP12       = 44,
    API_LARK_DAC_ROUTE_FDSP13       = 45,
    API_LARK_DAC_ROUTE_FDSP14       = 46,
    API_LARK_DAC_ROUTE_FDSP15       = 47,
    API_LARK_DAC_ROUTE_TDSP0        = 48,
    API_LARK_DAC_ROUTE_TDSP1        = 49,
    API_LARK_DAC_ROUTE_TDSP2        = 50,
    API_LARK_DAC_ROUTE_TDSP3        = 51,
    API_LARK_DAC_ROUTE_TDSP4        = 52,
    API_LARK_DAC_ROUTE_TDSP5        = 53,
    API_LARK_DAC_ROUTE_TDSP6        = 54,
    API_LARK_DAC_ROUTE_TDSP7        = 55,
    API_LARK_DAC_ROUTE_TDSP8        = 56,
    API_LARK_DAC_ROUTE_TDSP9        = 57,
    API_LARK_DAC_ROUTE_TDSP10       = 58,
    API_LARK_DAC_ROUTE_TDSP11       = 59,
    API_LARK_DAC_ROUTE_TDSP12       = 60,
    API_LARK_DAC_ROUTE_TDSP13       = 61,
    API_LARK_DAC_ROUTE_TDSP14       = 62,
    API_LARK_DAC_ROUTE_TDSP15       = 63,
    API_LARK_DAC_ROUTE_ASRCI0       = 64,
    API_LARK_DAC_ROUTE_ASRCI1       = 65,
    API_LARK_DAC_ROUTE_ASRCI2       = 66,
    API_LARK_DAC_ROUTE_ASRCI3       = 67,
    API_LARK_DAC_ROUTE_ADC0         = 68,
    API_LARK_DAC_ROUTE_ADC1         = 69,
    API_LARK_DAC_ROUTE_ADC2         = 70,
    API_LARK_DAC_ROUTE_DMIC0        = 71,
    API_LARK_DAC_ROUTE_DMIC1        = 72,
    API_LARK_DAC_ROUTE_DMIC2        = 73,
    API_LARK_DAC_ROUTE_DMIC3        = 74,
    API_LARK_DAC_ROUTE_EQ           = 75,
    API_LARK_DAC_ROUTE_FINT0        = 76,
    API_LARK_DAC_ROUTE_FINT1        = 77,
    API_LARK_DAC_ROUTE_FINT2        = 78,
    API_LARK_DAC_ROUTE_FINT3        = 79,
    API_LARK_DAC_ROUTE_FINT4        = 80,
    API_LARK_DAC_ROUTE_FINT5        = 81,
    API_LARK_DAC_ROUTE_FINT6        = 82,
    API_LARK_DAC_ROUTE_FINT7        = 83,
    API_LARK_DAC_ROUTE_RSV1         = 84,
    API_LARK_DAC_ROUTE_RSV2         = 85,
    API_LARK_DAC_ROUTE_RSV3         = 86,
    API_LARK_DAC_ROUTE_DMIC4        = 87,
    API_LARK_DAC_ROUTE_DMIC5        = 88,
    API_LARK_DAC_ROUTE_DMIC6        = 89,
    API_LARK_DAC_ROUTE_DMIC7        = 90,
    API_LARK_DAC_ROUTE_COUNT
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_DAC_ROUTE_ADC0         = 0,
    API_LARK_DAC_ROUTE_ADC1         = 1,
    API_LARK_DAC_ROUTE_ADC2         = 2,
    API_LARK_DAC_ROUTE_ASRCI0       = 3,
    API_LARK_DAC_ROUTE_EQ           = 4,
    API_LARK_DAC_ROUTE_FDSP0        = 5,
    API_LARK_DAC_ROUTE_FDSP1        = 6,
    API_LARK_DAC_ROUTE_FDSP2        = 7,
    API_LARK_DAC_ROUTE_FDSP3        = 8,
    API_LARK_DAC_ROUTE_FDSP4        = 9,
    API_LARK_DAC_ROUTE_FDSP5        = 10,
    API_LARK_DAC_ROUTE_FDSP6        = 11,
    API_LARK_DAC_ROUTE_FDSP7        = 12,
    API_LARK_DAC_ROUTE_FDSP8        = 13,
    API_LARK_DAC_ROUTE_FDSP9        = 14,
    API_LARK_DAC_ROUTE_FDSP10       = 15,
    API_LARK_DAC_ROUTE_FDSP11       = 16,
    API_LARK_DAC_ROUTE_FDSP12       = 17,
    API_LARK_DAC_ROUTE_FDSP13       = 18,
    API_LARK_DAC_ROUTE_FDSP14       = 19,
    API_LARK_DAC_ROUTE_FDSP15       = 20,
    API_LARK_DAC_ROUTE_SAI0_00      = 21,
    API_LARK_DAC_ROUTE_SAI0_01      = 22,
    API_LARK_DAC_ROUTE_SAI0_02      = 23,
    API_LARK_DAC_ROUTE_SAI0_03      = 24,
    API_LARK_DAC_ROUTE_SAI0_04      = 25,
    API_LARK_DAC_ROUTE_SAI0_05      = 26,
    API_LARK_DAC_ROUTE_SAI0_06      = 27,
    API_LARK_DAC_ROUTE_SAI0_07      = 28,
    API_LARK_DAC_ROUTE_SAI0_08      = 29,
    API_LARK_DAC_ROUTE_SAI0_09      = 30,
    API_LARK_DAC_ROUTE_SAI0_10      = 31,
    API_LARK_DAC_ROUTE_SAI0_11      = 32,
    API_LARK_DAC_ROUTE_SAI0_12      = 33,
    API_LARK_DAC_ROUTE_SAI0_13      = 34,
    API_LARK_DAC_ROUTE_SAI0_14      = 35,
    API_LARK_DAC_ROUTE_SAI0_15      = 36,
    API_LARK_DAC_ROUTE_FINT0        = 37,
    API_LARK_DAC_ROUTE_FINT1        = 38
    #endif
} adi_lark_dac_input_route_e;

/*!
 * @brief  enum for PDM sample rate
 */
typedef enum{
    API_LARK_PDM_SAMPLE_RATE_12KHz  = 0,
    API_LARK_PDM_SAMPLE_RATE_24KHz  = 1,
    API_LARK_PDM_SAMPLE_RATE_48KHz  = 2,
    API_LARK_PDM_SAMPLE_RATE_96KHz  = 3,
    API_LARK_PDM_SAMPLE_RATE_192KHz = 4,
    API_LARK_PDM_SAMPLE_RATE_384KHz = 5,
    API_LARK_PDM_SAMPLE_RATE_768KHz = 6
} adi_lark_pdm_sample_rate_e;

/*!
 * @brief  enum for PDM output rate
 */
typedef enum{
    API_LARK_PDM_OUTPUT_RATE_6144KHz  = 0,
    API_LARK_PDM_OUTPUT_RATE_3072KHz  = 1,
} adi_lark_pdm_output_rate_e;

/*!
 * @brief  enum for PDM input routing
 */
typedef enum{
    #ifdef LARK_SDK
    API_LARK_PDM_ROUTE_SAI0_00      = 0,
    API_LARK_PDM_ROUTE_SAI0_01      = 1,
    API_LARK_PDM_ROUTE_SAI0_02      = 2,
    API_LARK_PDM_ROUTE_SAI0_03      = 3,
    API_LARK_PDM_ROUTE_SAI0_04      = 4,
    API_LARK_PDM_ROUTE_SAI0_05      = 5,
    API_LARK_PDM_ROUTE_SAI0_06      = 6,
    API_LARK_PDM_ROUTE_SAI0_07      = 7,
    API_LARK_PDM_ROUTE_SAI0_08      = 8,
    API_LARK_PDM_ROUTE_SAI0_09      = 9,
    API_LARK_PDM_ROUTE_SAI0_10      = 10,
    API_LARK_PDM_ROUTE_SAI0_11      = 11,
    API_LARK_PDM_ROUTE_SAI0_12      = 12,
    API_LARK_PDM_ROUTE_SAI0_13      = 13,
    API_LARK_PDM_ROUTE_SAI0_14      = 14,
    API_LARK_PDM_ROUTE_SAI0_15      = 15,
    API_LARK_PDM_ROUTE_SAI1_00      = 16,
    API_LARK_PDM_ROUTE_SAI1_01      = 17,
    API_LARK_PDM_ROUTE_SAI1_02      = 18,
    API_LARK_PDM_ROUTE_SAI1_03      = 19,
    API_LARK_PDM_ROUTE_SAI1_04      = 20,
    API_LARK_PDM_ROUTE_SAI1_05      = 21,
    API_LARK_PDM_ROUTE_SAI1_06      = 22,
    API_LARK_PDM_ROUTE_SAI1_07      = 23,
    API_LARK_PDM_ROUTE_SAI1_08      = 24,
    API_LARK_PDM_ROUTE_SAI1_09      = 25,
    API_LARK_PDM_ROUTE_SAI1_10      = 26,
    API_LARK_PDM_ROUTE_SAI1_11      = 27,
    API_LARK_PDM_ROUTE_SAI1_12      = 28,
    API_LARK_PDM_ROUTE_SAI1_13      = 29,
    API_LARK_PDM_ROUTE_SAI1_14      = 30,
    API_LARK_PDM_ROUTE_SAI1_15      = 31,
    API_LARK_PDM_ROUTE_FDSP0        = 32,
    API_LARK_PDM_ROUTE_FDSP1        = 33,
    API_LARK_PDM_ROUTE_FDSP2        = 34,
    API_LARK_PDM_ROUTE_FDSP3        = 35,
    API_LARK_PDM_ROUTE_FDSP4        = 36,
    API_LARK_PDM_ROUTE_FDSP5        = 37,
    API_LARK_PDM_ROUTE_FDSP6        = 38,
    API_LARK_PDM_ROUTE_FDSP7        = 39,
    API_LARK_PDM_ROUTE_FDSP8        = 40,
    API_LARK_PDM_ROUTE_FDSP9        = 41,
    API_LARK_PDM_ROUTE_FDSP10       = 42,
    API_LARK_PDM_ROUTE_FDSP11       = 43,
    API_LARK_PDM_ROUTE_FDSP12       = 44,
    API_LARK_PDM_ROUTE_FDSP13       = 45,
    API_LARK_PDM_ROUTE_FDSP14       = 46,
    API_LARK_PDM_ROUTE_FDSP15       = 47,
    API_LARK_PDM_ROUTE_TDSP0        = 48,
    API_LARK_PDM_ROUTE_TDSP1        = 49,
    API_LARK_PDM_ROUTE_TDSP2        = 50,
    API_LARK_PDM_ROUTE_TDSP3        = 51,
    API_LARK_PDM_ROUTE_TDSP4        = 52,
    API_LARK_PDM_ROUTE_TDSP5        = 53,
    API_LARK_PDM_ROUTE_TDSP6        = 54,
    API_LARK_PDM_ROUTE_TDSP7        = 55,
    API_LARK_PDM_ROUTE_TDSP8        = 56,
    API_LARK_PDM_ROUTE_TDSP9        = 57,
    API_LARK_PDM_ROUTE_TDSP10       = 58,
    API_LARK_PDM_ROUTE_TDSP11       = 59,
    API_LARK_PDM_ROUTE_TDSP12       = 60,
    API_LARK_PDM_ROUTE_TDSP13       = 61,
    API_LARK_PDM_ROUTE_TDSP14       = 62,
    API_LARK_PDM_ROUTE_TDSP15       = 63,
    API_LARK_PDM_ROUTE_ASRCI0       = 64,
    API_LARK_PDM_ROUTE_ASRCI1       = 65,
    API_LARK_PDM_ROUTE_ASRCI2       = 66,
    API_LARK_PDM_ROUTE_ASRCI3       = 67,
    API_LARK_PDM_ROUTE_ADC0         = 68,
    API_LARK_PDM_ROUTE_ADC1         = 69,
    API_LARK_PDM_ROUTE_ADC2         = 70,
    API_LARK_PDM_ROUTE_DMIC0        = 71,
    API_LARK_PDM_ROUTE_DMIC1        = 72,
    API_LARK_PDM_ROUTE_DMIC2        = 73,
    API_LARK_PDM_ROUTE_DMIC3        = 74,
    API_LARK_PDM_ROUTE_EQ           = 75,
    API_LARK_PDM_ROUTE_FINT0        = 76,
    API_LARK_PDM_ROUTE_FINT1        = 77,
    API_LARK_PDM_ROUTE_FINT2        = 78,
    API_LARK_PDM_ROUTE_FINT3        = 79,
    API_LARK_PDM_ROUTE_FINT4        = 80,
    API_LARK_PDM_ROUTE_FINT5        = 81,
    API_LARK_PDM_ROUTE_FINT6        = 82,
    API_LARK_PDM_ROUTE_FINT7        = 83,
    API_LARK_PDM_ROUTE_RSV1         = 84,
    API_LARK_PDM_ROUTE_RSV2         = 85,
    API_LARK_PDM_ROUTE_RSV3         = 86,
    API_LARK_PDM_ROUTE_DMIC4        = 87,
    API_LARK_PDM_ROUTE_DMIC5        = 88,
    API_LARK_PDM_ROUTE_DMIC6        = 89,
    API_LARK_PDM_ROUTE_DMIC7        = 90,
    API_LARK_PDM_ROUTE_COUNT
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_PDM_ROUTE_ADC0         = 0,
    API_LARK_PDM_ROUTE_ADC1         = 1,
    API_LARK_PDM_ROUTE_ADC2         = 2,
    API_LARK_PDM_ROUTE_ASRCI0       = 3,
    API_LARK_PDM_ROUTE_EQ           = 4,
    API_LARK_PDM_ROUTE_FDSP0        = 5,
    API_LARK_PDM_ROUTE_FDSP1        = 6,
    API_LARK_PDM_ROUTE_FDSP2        = 7,
    API_LARK_PDM_ROUTE_FDSP3        = 8,
    API_LARK_PDM_ROUTE_FDSP4        = 9,
    API_LARK_PDM_ROUTE_FDSP5        = 10,
    API_LARK_PDM_ROUTE_FDSP6        = 11,
    API_LARK_PDM_ROUTE_FDSP7        = 12,
    API_LARK_PDM_ROUTE_FDSP8        = 13,
    API_LARK_PDM_ROUTE_FDSP9        = 14,
    API_LARK_PDM_ROUTE_FDSP10       = 15,
    API_LARK_PDM_ROUTE_FDSP11       = 16,
    API_LARK_PDM_ROUTE_FDSP12       = 17,
    API_LARK_PDM_ROUTE_FDSP13       = 18,
    API_LARK_PDM_ROUTE_FDSP14       = 19,
    API_LARK_PDM_ROUTE_FDSP15       = 20,
    API_LARK_PDM_ROUTE_SAI0_00      = 21,
    API_LARK_PDM_ROUTE_SAI0_01      = 22,
    API_LARK_PDM_ROUTE_SAI0_02      = 23,
    API_LARK_PDM_ROUTE_SAI0_03      = 24,
    API_LARK_PDM_ROUTE_SAI0_04      = 25,
    API_LARK_PDM_ROUTE_SAI0_05      = 26,
    API_LARK_PDM_ROUTE_SAI0_06      = 27,
    API_LARK_PDM_ROUTE_SAI0_07      = 28,
    API_LARK_PDM_ROUTE_SAI0_08      = 29,
    API_LARK_PDM_ROUTE_SAI0_09      = 30,
    API_LARK_PDM_ROUTE_SAI0_10      = 31,
    API_LARK_PDM_ROUTE_SAI0_11      = 32,
    API_LARK_PDM_ROUTE_SAI0_12      = 33,
    API_LARK_PDM_ROUTE_SAI0_13      = 34,
    API_LARK_PDM_ROUTE_SAI0_14      = 35,
    API_LARK_PDM_ROUTE_SAI0_15      = 36,
    API_LARK_PDM_ROUTE_FINT0        = 37,
    API_LARK_PDM_ROUTE_FINT1        = 38
    #endif
} adi_lark_pdm_input_route_e;

/*!
 * @brief  enum for chip power mode
 */
typedef enum{
    API_LARK_PWR_MODE_HIBERNATE2     = 0,
    API_LARK_PWR_MODE_HIBERNATE1     = 1,
    API_LARK_PWR_MODE_HIBERNATE0     = 2,
    API_LARK_PWR_MODE_ACTIVE         = 3,
} adi_lark_pwr_mode_e;

/*!
 * @brief  enum for PGA auto slew rate
 */
typedef enum{
    API_LARK_PGA_SLEW_RATE_2P2_DB    = 0,                        /*!< 2.2 dB/ms */
    API_LARK_PGA_SLEW_RATE_1P1_DB    = 1,                        /*!< 1.1 dB/ms */
    API_LARK_PGA_SLEW_RATE_0P5_DB    = 2                         /*!< 0.5 dB/ms */
} adi_lark_pga_auto_slew_rate_e;

/*!
 * @brief  enum for gpt count mode, for lark only.
 */
typedef enum
{
    API_LARK_GPT_COUNT_MODE_DOWN     = 0,
    API_LARK_GPT_COUNT_MODE_UP       = 1
} adi_lark_gpt_count_mode_e;

/*!
 * @brief  enum for gpt mode, for lark only.
 */
typedef enum
{
    API_LARK_GPT_MODE_FREERUN        = 0,                        /*!< Timer Runs in Periodic Mode */
    API_LARK_GPT_MODE_PERIODIC       = 1                         /*!< Timer Runs in Free Running Mode */
} adi_lark_gpt_run_mode_e;

/*!
 * @brief  enum for gpt clock source, for lark only.
 */
typedef enum
{
    API_LARK_GPT_CLOCK_BUSCLK        = 0,                        /*!< APB bus clock */
    API_LARK_GPT_CLOCK_CLK32K        = 1,                        /*!< 32KHz clock generated from AON */
} adi_lark_gpt_clk_source_e;

/*!
 * @brief  enum for gpt pre scaler divider, for lark only.
 */
typedef enum
{
    API_LARK_GPT_PRE_SCALER_1        = 0,                        /*!< source clock / [1 or 4], If API_LARK_GPT_CLOCK_BUSCLK is selected then means divide by 4, else it means divide by 1 */
    API_LARK_GPT_PRE_SCALER_16       = 1,                        /*!< source clock / 16 */
    API_LARK_GPT_PRE_SCALER_256      = 2,                        /*!< source clock / 256 */
    API_LARK_GPT_PRE_SCALER_32768    = 3                         /*!< source clock / 32768 */
} adi_lark_gpt_pre_scaler_e;

/*!
 * @brief  enum for dmic input clock rate, for lark only.
 */
typedef enum
{
    API_LARK_DMIC_CLK_RATE_384KHZ  = 0,
    API_LARK_DMIC_CLK_RATE_768KHZ  = 1,
    API_LARK_DMIC_CLK_RATE_1536KHZ = 2,
    API_LARK_DMIC_CLK_RATE_3072KHZ = 3,
    API_LARK_DMIC_CLK_RATE_6144KHZ = 4,
} adi_lark_dmic_clk_rate_e;

/*!
 * @brief  enum for dmic output sample rate, for lark only.
 */
typedef enum
{
    API_LARK_DMIC_OUT_SAMPLE_RATE_12KHZ  = 0,
    API_LARK_DMIC_OUT_SAMPLE_RATE_24KHZ  = 1,
    API_LARK_DMIC_OUT_SAMPLE_RATE_48KHZ  = 2,
    API_LARK_DMIC_OUT_SAMPLE_RATE_96KHZ  = 3,
    API_LARK_DMIC_OUT_SAMPLE_RATE_192KHZ = 4,
    API_LARK_DMIC_OUT_SAMPLE_RATE_384KHZ = 5,
    API_LARK_DMIC_OUT_SAMPLE_RATE_768KHZ = 6,
} adi_lark_dmic_out_sample_rate_e;

/*!
 * @brief  enum for dmic high pass filter, for lark only.
 */
typedef enum
{
    API_LARK_DMIC_HPF_CUTOFF_FREQ_OFF = 0,
    API_LARK_DMIC_HPF_CUTOFF_FREQ_1HZ = 1,
    API_LARK_DMIC_HPF_CUTOFF_FREQ_4HZ = 2,
    API_LARK_DMIC_HPF_CUTOFF_FREQ_8HZ = 3,
} adi_lark_dmic_hpf_cutoff_freq_e;

/*!
 * @brief  enum for dmic dec order, for lark only.
 */
typedef enum
{
    API_LARK_DMIC_DEC_ORDER_4TH = 0,
    API_LARK_DMIC_DEC_ORDER_5TH = 1,
} adi_lark_dmic_dec_order_e;

/*!
 * @brief  enum for dmic clk1 pin for lark only.
 */
typedef enum
{
    API_LARK_DMIC_CLK1_PIN_MP7  = 0,
    API_LARK_DMIC_CLK1_PIN_MP11 = 1,
    API_LARK_DMIC_CLK1_PIN_MP17 = 2,
    API_LARK_DMIC_CLK1_PIN_MP22 = 3,
} adi_lark_dmic_clk1_pin_e;

/*!
 * @brief  enum for tdsp clk rate, for lark only.
 */
typedef enum
{
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_1  = 0,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_2  = 1,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_3  = 2,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_4  = 3,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_6  = 4,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_8  = 5,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_12 = 6,
    API_LARK_TDSP_CLK_RATE_MCLK_OVER_16 = 7,
    API_LARK_TDSP_CLK_RATE_COUNT        = 8
} adi_lark_tdsp_clk_rate_e;

/*!
 * @brief  enum for ahb bus clk rate, for lark only.
 */
typedef enum
{
    API_LARK_BUS_CLK_RATE_MCLK_OVER_1  = 0,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_2  = 1,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_3  = 2,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_4  = 3,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_6  = 4,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_8  = 5,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_12 = 6,
    API_LARK_BUS_CLK_RATE_MCLK_OVER_16 = 7,
    API_LARK_BUS_CLK_RATE_COUNT        = 8
} adi_lark_bus_clk_rate_e;

/*!
 * @brief  enum for ffsram clk rate, for lark only.
 */
typedef enum
{
    API_LARK_FFSRAM_CLK_RATE_BUSCLK_OVER_1  = 0,
    API_LARK_FFSRAM_CLK_RATE_BUSCLK_OVER_2  = 1,
    API_LARK_FFSRAM_CLK_RATE_BUSCLK_OVER_3  = 2,
    API_LARK_FFSRAM_CLK_RATE_BUSCLK_OVER_4  = 3,
    API_LARK_FFSRAM_CLK_RATE_COUNT          = 4
} adi_lark_ffsram_clk_rate_e;

/*!
 * @brief  enum for uart clk rate, for lark only.
 */
typedef enum
{
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_1  = 0,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_2  = 1,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_3  = 2,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_4  = 3,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_6  = 4,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_8  = 5,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_12 = 6,
    API_LARK_UART_CLK_RATE_BUSCLK_OVER_16 = 7,
    API_LARK_UART_CLK_RATE_COUNT          = 8
} adi_lark_uart_clk_rate_e;

/*!
 * @brief  enum for mclk freq, for lark only.
 */
typedef enum
{
    API_LARK_MCLK_FREQ_24P576 = 0,
    API_LARK_MCLK_FREQ_49P152 = 1,
    API_LARK_MCLK_FREQ_73P728 = 2,
    API_LARK_MCLK_FREQ_98P304 = 3
} adi_lark_mclk_freq_e;

/*!
 * @brief  enum for PLL source
 */
typedef enum
{
    API_LARK_CLK_PLL_SOURCE_MCLKIN = 0,
    API_LARK_CLK_PLL_SOURCE_FSYNC0 = 1,
    API_LARK_CLK_PLL_SOURCE_BCLK0  = 2,
    #ifdef LARK_SDK
    API_LARK_CLK_PLL_SOURCE_FSYNC1 = 3,
    API_LARK_CLK_PLL_SOURCE_BCLK1  = 4
    #endif
    #ifdef LARK_LITE_SDK
    API_LARK_CLK_PLL_SOURCE_RCOSC = 3,
    #endif
} adi_lark_clk_pll_source_e;

/*!
 * @brief  enum for XTAL mode, for lark only.
 */
typedef enum
{
    API_LARK_CLK_XTAL_MODE_MCLK = 0,
    API_LARK_CLK_XTAL_MODE_XTAL = 1
} adi_lark_clk_xtal_mode_e;

/*!
 * @brief  enum for PLL type, for lark only.
 */
typedef enum
{
    API_LARK_CLK_PLL_TYPE_INTEGER = 0,
    API_LARK_CLK_PLL_TYPE_FRAC    = 1
} adi_lark_clk_pll_type_e;

/*!
 * @brief  enum for sync_source, for lark only.
 */
typedef enum
{
    API_LARK_CLK_SYNC_SOURCE_LRCLK0   = 0,
    API_LARK_CLK_SYNC_SOURCE_LRCLK1   = 1,
    API_LARK_CLK_SYNC_SOURCE_ASRCI    = 2,
    API_LARK_CLK_SYNC_SOURCE_INTERNAL = 3
} adi_lark_clk_sync_source_e;

/*!
 * @brief  enum for interna OSC cal reference clock, for lark-lite only.
 */
typedef enum
{
    API_LARK_CLK_OSC_CAL_REF_12K  = 0,
    API_LARK_CLK_OSC_CAL_REF_24K  = 1,
    API_LARK_CLK_OSC_CAL_REF_48K  = 2,
    API_LARK_CLK_OSC_CAL_REF_96K  = 3,
    API_LARK_CLK_OSC_CAL_REF_192K = 4,
    API_LARK_CLK_OSC_CAL_REF_384K = 5,
    API_LARK_CLK_OSC_CAL_REF_768K = 6,
    API_LARK_CLK_OSC_CAL_REF_8K   = 8,
    API_LARK_CLK_OSC_CAL_REF_16K  = 9
} adi_lark_clk_osc_cal_ref_sel_e;

/*!
 * @brief  enum for HPAMP mode
 */
typedef enum
{
    API_LARK_HPAMP_HEADPHONE = 0,
    API_LARK_HPAMP_LINE_OUT  = 1
} adi_lark_hpamp_mode_e;

/*!
 * @brief  enum for HPAMP voltage mode
 */
typedef enum
{
    API_LARK_HPAMP_SWITCH_TO_NVM = 0,
    API_LARK_HPAMP_SWITCH_TO_LVM = 1
} adi_lark_hpamp_voltage_mode_e;

/*!
 * @brief  enum for HPAMP LDO vout
 */
typedef enum
{
    API_LARK_HPAMP_LDO_OUT_1P1  = 0,
    API_LARK_HPAMP_LDO_OUT_1P15 = 1,
    API_LARK_HPAMP_LDO_OUT_1P2  = 2,
    API_LARK_HPAMP_LDO_OUT_1P05 = 3
} adi_lark_hpamp_ldo_vout_e;

/*!
 * @brief  enum for HPAMP voltage switch mode
 */
typedef enum
{
    API_LARK_HPAMP_AUTO_SWITCH    = 0,
    API_LARK_HPAMP_MANNUAL_SWITCH = 1,
    API_LARK_HPAMP_HOT_SWITCH     = 2
} adi_lark_hpamp_voltage_switch_mode_e;

/*!
 * @brief  enum for memory voltage mode, for lark only.
 */
typedef enum {
    API_LARK_PMU_MEM_VOL_MODE_LP = 0, 
    API_LARK_PMU_MEM_VOL_MODE_MP = 1, 
    API_LARK_PMU_MEM_VOL_MODE_HP = 2 
} adi_lark_pmu_memory_voltage_mode_e;

/*!
 * @brief  enum for digital ldo output
 */
typedef enum
{
    API_LARK_PMU_DLDO_OUTPUT_0P85V = 0,
    API_LARK_PMU_DLDO_OUTPUT_0P9V  = 1,
    API_LARK_PMU_DLDO_OUTPUT_0P95V = 2,
    API_LARK_PMU_DLDO_OUTPUT_1P0V  = 3,
    API_LARK_PMU_DLDO_OUTPUT_1P05V = 4,
    API_LARK_PMU_DLDO_OUTPUT_1P1V  = 5
} adi_lark_pmu_dldo_output_e;

/*!
 * @brief  enum for memory retention control, for lark only.
 */
typedef enum
{
    API_LARK_PMU_RETAIN_FDSP_EQ = 0x01,
    API_LARK_PMU_RETAIN_FFSRAM2 = 0x02,
    API_LARK_PMU_RETAIN_IRAM0   = 0x04,
    API_LARK_PMU_RETAIN_DRAM0   = 0x08,
    API_LARK_PMU_RETAIN_DRAM1   = 0x10,
    API_LARK_PMU_RETAIN_SYSRAM  = 0x20,
    API_LARK_PMU_RETAIN_FFSRAM0 = 0x40,
    API_LARK_PMU_RETAIN_FFSRAM1 = 0x80,
    API_LARK_PMU_RETAIN_ALL     = 0xFF
} adi_lark_pmu_mem_retain_e;

/*!
 * @brief  enum for irq wakeup enable settings, for lark only.
 */
typedef enum
{
    API_LARK_PMU_WAKEUP_IRQ1    = 0x01,
    API_LARK_PMU_WAKEUP_IRQ2    = 0x02,
    API_LARK_PMU_WAKEUP_IRQ3    = 0x04,
    API_LARK_PMU_WAKEUP_IRQ4    = 0x08,
    API_LARK_PMU_WAKEUP_MP_IRQ1 = 0x10,
    API_LARK_PMU_WAKEUP_MP_IRQ2 = 0x20,
    API_LARK_PMU_WAKEUP_MP_IRQ3 = 0x40
} adi_lark_pmu_wakeup_irq_e;

/*!
 * @brief  enum for wdt prescaler factor, for lark only.
 */
typedef enum
{
    API_LARK_WDT_PRE_SCALER_FACTOR_1    = 0,
    API_LARK_WDT_PRE_SCALER_FACTOR_16   = 1,
    API_LARK_WDT_PRE_SCALER_FACTOR_256  = 2,
    API_LARK_WDT_PRE_SCALER_FACTOR_4096 = 3,
} adi_lark_wdt_pre_scaler_factor_e;

/*!
 * @brief  enum for wdt timer mode, for lark only.
 */
typedef enum
{
    API_LARK_WDT_MODE_FREE_RUNNING = 0,
    API_LARK_WDT_MODE_PERIODIC     = 1,
} adi_lark_wdt_mode_e;

/*!
 * @brief  enum for qspi transfer size, for lark only.
 */
typedef enum
{
    API_LARK_QSPI_TRANSFER_8_BIT  = 0,
    API_LARK_QSPI_TRANSFER_16_BIT = 1,
    API_LARK_QSPI_TRANSFER_32_BIT = 2
} adi_lark_qspi_transfer_size_e;

/*!
 * @brief  enum for qspi multiple I/O mode, for lark only.
 */
typedef enum
{
    API_LARK_QSPI_STANDARD = 0,
    API_LARK_QSPI_DUAL     = 1,
    API_LARK_QSPI_QUAD     = 2
} adi_lark_qspi_multi_io_mode_e;

/*!
 * @brief  enum for uart word length, for lark only.
 */
typedef enum
{
    API_LARK_UART_WORDLEN_5BITS = 0,
    API_LARK_UART_WORDLEN_6BITS = 1,
    API_LARK_UART_WORDLEN_7BITS = 2,
    API_LARK_UART_WORDLEN_8BITS = 3
} adi_lark_uart_word_len_e;

/*!
 * @brief  enum for uart parity type, for lark only.
 */
typedef enum
{
    API_LARK_UART_PARITY_NONE = 0,
    API_LARK_UART_PARITY_ODD  = 1,
    API_LARK_UART_PARITY_EVEN = 2
} adi_lark_uart_parity_type_e;

#ifdef __cplusplus
extern "C" {
#endif

/*============= E X P O R T S ==============*/
/**
 * @brief  Conrol port register read function by spi
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  reg_data   Pointer to save readback data
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_spi_reg_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t *reg_data);

/**
 * @brief  Conrol port register write function by spi
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to write
 * @param  reg_data   Register value to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_spi_reg_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t reg_data);

/**
 * @brief  Control port burst read via SPI
 *
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to read
 * @param  data         Pointer of data buffer
 * @param  byte_length  Number of bytes to read
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_spi_mem_read(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_length);

/**
 * @brief  Control port burst write via SPI
 *
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to write
 * @param  data         Pointer of data buffer
 * @param  byte_length  Number of bytes to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_spi_mem_write(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_length);

/**
 * @brief  Conrol port register read function by i2c
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  reg_data   Pointer to save readback data
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_i2c_reg_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t *reg_data);

/**
 * @brief  Conrol port register write function by i2c
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to write
 * @param  reg_data   Register value to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_i2c_reg_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t reg_data);

/**
 * @brief  Conrol port burst read function by i2c
 *         
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to read
 * @param  data         Pointer of data buffer
 * @param  byte_length  Number of bytes to read
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_i2c_mem_read(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_length);

/**
 * @brief  Conrol port burst write function by i2c
 *         
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to write
 * @param  data         Pointer of data buffer
 * @param  byte_length  Number of bytes to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_i2c_mem_write(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_length);

/**
 * @brief  Conrol port register read function by uart
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  reg_data   Pointer to save readback data
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_uart_reg_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t *reg_data);

/**
 * @brief  Conrol port register write function by uart
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to write
 * @param  reg_data   Register value to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_uart_reg_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t reg_data);

/**
 * @brief  Conrol port burst read function by uart
 *         
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to read
 * @param  data         Pointer of data buffer
 * @param  byte_len     Number of bytes to read
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_uart_mem_read(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_len);

/**
 * @brief  Conrol port burst write function by uart
 *         
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to write
 * @param  data         Pointer of data buffer
 * @param  byte_len     Number of bytes to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_uart_mem_write(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_len);

/**
 * @brief  Native register read function
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  reg_data   Pointer to save readback data
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_native_reg_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t *reg_data);

/**
 * @brief  Native register write function
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to write
 * @param  reg_data   Register value to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_native_reg_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t reg_data);

/**
 * @brief  Native memory read
 *         
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to read
 * @param  data         Pointer of data buffer
 * @param  byte_len     Number of bytes to read
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_native_mem_read(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_len);

/**
 * @brief  Native memory write
 *         
 * @param  device       Pointer to device structure
 * @param  addr         Starting address to write
 * @param  data         Pointer of data buffer
 * @param  byte_len     Number of bytes to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_cp_native_mem_write(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_len);

/**
 * @brief  HAL memory read function.
 *         
 * @param  device     Pointer to device structure
 * @param  addr       Start memory address to read
 * @param  data       Pointer of data buffer
 * @param  byte_len   byte length to read
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */ 
int32_t adi_lark_hal_mem_read(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_len);

/**
 * @brief  HAL memory write function.
 *         
 * @param  device     Pointer to device structure
 * @param  addr       Start memory address to write
 * @param  data       Pointer of data buffer
 * @param  byte_len   byte length to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_mem_write(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_len);

/**
 * @brief  HAL register read function.
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  reg_data   Pointer to save readback data
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_reg_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t *reg_data);

/**
 * @brief  HAL register write function.
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to write
 * @param  reg_data   Register value to write
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_reg_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t reg_data);

/**
 * @brief  HAL bit field read function.
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  bf_info    Bit field information, byte0 - start bit, byte1 - bits length
 * @param  bf_val     Pointer to bit field read back value
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_bf_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t bf_info, uint32_t *bf_val);

/**
 * @brief  HAL bit field read function.
 *         
 * @param  device     Pointer to device structure
 * @param  reg_addr   Register address to read
 * @param  bf_info    Bit field information, byte0 - start bit, byte1 - bits length
 * @param  bf_val     Pointer to bit field read back value
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_bf_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t bf_info, uint32_t bf_val);

/**
 * @brief  HAL error report function.
 *         
 * @param  device     Pointer to device structure
 * @param  log_type   Log type (LARK_LOG_NONE/MSG/WARN/ERROR/ALL)
 * @param  file_name  Source file name where error happens
 * @param  line_num   Source file line number where error happens
 * @param  func_name  Function name where error happens
 * @param  var_name   Variable name where error happens
 * @param  comment    Error message
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_error_report(adi_lark_device_t* device, uint32_t log_type,
    const char* file_name, const char* func_name, uint32_t line_num, const char* var_name, const char* comment);

/**
 * @brief  HAL log write function.
 *         
 * @param  device     Pointer to device structure
 * @param  log_type   Log type (LARK_LOG_NONE/MSG/WARN/ERROR/ALL)
 * @param  comment    Log message format
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_hal_log_write(adi_lark_device_t *device, uint32_t log_type, const char* comment, ...);

/**
 * @brief  Get device vendor id, device id and device revision
 *         
 * @param  device     Pointer to device structure
 * @param  vid        Pointer to vendor id value
 * @param  did        Pointer to device id value
 * @param  drev       Pointer to device revision value
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_get_id(adi_lark_device_t *device, uint8_t *vid, uint16_t *did, uint8_t *drev);

/**
 * @brief  Get this sdk revision
 *         
 * @param  device     Pointer to device structure
 * @param  rev        Pointer to revision value (major.minor.build, one byte each)
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_get_sdk_rev(adi_lark_device_t *device, uint32_t *rev);

/**
 * @brief  Check device ID and enable turbo mode.
 *         
 * @param  device     Pointer to device structure
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_init(adi_lark_device_t *device);

/**
 * @brief  Reset this device in soft way
 *         
 * @param  device     Pointer to device structure
 * @param  full_reset 1 for full soft reset, 0 for soft reset
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_reset_device(adi_lark_device_t *device, bool full_reset);

/**
 * @brief  Set pin drive & slew & pull property
 *         
 * @param  device     Pointer to device structure
 * @param  pin        @see adi_lark_pin_name_e
 * @param  drive      @see adi_lark_pin_drive_e
 * @param  slew       @see adi_lark_pin_slew_e
 * @param  pull       @see adi_lark_pin_pull_e
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_set_pin_drive_slew_pull(adi_lark_device_t *device, adi_lark_pin_name_e pin, 
    adi_lark_pin_drive_e drive, adi_lark_pin_slew_e slew, adi_lark_pin_pull_e pull);

/**
 * @brief  Power off all modules in audio data path
 *         
 * @param  device     Pointer to device structure
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_power_off_adp(adi_lark_device_t *device);

#if LARK_SDK_EN_TEST_CODE
/**
 * @brief  Initialize all adp modules to power off and disable state. Call before other code in test function.
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_init_adp_for_test(adi_lark_device_t *device);

/**
 * @brief  control port write/read memory test.
 * 
 * @param  device           Pointer to device structure
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_device_cp_test(adi_lark_device_t *device);

/**
 * @brief  Test function for clock rate changing, for lark only
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_clock_rate_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq);

/**
 * @brief  Obsolete test function for multiple audio data paths, for lark only.
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_datapath_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq);

/**
 * @brief  Test function to test audio data path sap0_asrci0_fdec0_asrco0_sap1 for lark
 *         and compatible with lark_lite using path sap0_asrci0_fdec0_asrco0_sap0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_fdec0_asrco0_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_fint0_fdsp0_sap1 for lark
 *         and compatible with lark_lite using path sap0_asrci0_fint0_fdsp0_sap0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_fint0_fdsp0_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_fint0_fdsp0_fdec0_asrco0_sap1 for lark
 *         and compatible with lark_lite using path sap0_asrci0_fint0_fdsp0_fdec0_asrco0_sap0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_fint0_fdsp0_fdec0_asrco0_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci01_fint01_dsfifo_fdec01_asrco01_sap1 for lark
 *         and compatible with lark_lite using path sap0_asrci01_fint01_dsfifo_fdec01_asrco01_sap0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_fdec01_asrco01_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci01_fint01_dsfifo_fdec01_sap1 for lark only.
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_fdec01_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci01_fint01_dsfifo_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci01_fint01_dsfifo_asrco01_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci01_fint01_dsfifo_asrco01_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci01_dsfifo_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci01_dsfifo_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci01_dsfifo_asrco01_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci01_dsfifo_asrco01_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_fdec0_dsfifo_fint0_fdsp0_eq_asrco01_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_fdec0_dsfifo_fint0_fdsp0_eq_asrco01_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path adc0_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_adc0_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path adc0_fint0_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_adc0_fint0_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path adc0_fdsp0_fint0_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_adc0_fdsp0_fint0_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path adc0_fint0_fdsp0_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_adc0_fint0_fdsp0_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_eq_dac0_and_adc0_fint0_fdsp0_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_eq_dac0_and_adc0_fint0_fdsp0_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_eq_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_eq_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_fint0_dac0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_fint0_dac0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path dmic0_fint0_fdsp0_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_dmic0_fint0_fdsp0_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path dmic0_fdec0_asrco0_sap1 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_dmic0_fdec0_asrco0_sap1_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path dmic0_pdm_sap0 for lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_dmic0_pdm_sap0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path sap0_asrci0_fdsp0_fint0_dac0_adc0_eq_fdec0_asrco0_sap0
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_sap0_asrci0_fdsp0_fint0_dac0_adc0_eq_fdec0_asrco0_sap0_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test audio data path adc0_dac0 with pll enabled
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_pllen_adc0_dac0_test(adi_lark_device_t *device);
#endif

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
/**
 * @brief  Test function to test memory read write, for code running on lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_memory_test(adi_lark_device_t *device);

/**
 * @brief  Obsolete test function for data sync paths, for code running on lark only.
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_ds_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq);

/**
 * @brief  Test function to test datasync transferring I2S in/out data using DMA mode, for code running on lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_ds_i2s_dma_loopback_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test datasync transferring I2S in/out data using FIFO mode, for code running on lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_ds_i2s_fifo_loopback_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test datasync transferring I2S in/out data using TIE mode, for code running on lark only
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_ds_i2s_tie_loopback_test(adi_lark_device_t *device);

/**
 * @brief  Test function to test using memory DMA to read FIFO sram read register.
 * @param  device           Pointer to device structure
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_device_fifo_mdma_test(adi_lark_device_t *device);
#endif

/**
 * @brief  Clear interrupt request
 *         
 * @param  device     Pointer to device structure
 * @param  index      1/2/3/4 for lark, 1/2 for lark-lite
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_clr_irq(adi_lark_device_t *device, uint8_t index);

/**
 * @brief  Set interrupt request polarity
 *         
 * @param  device     Pointer to device structure
 * @param  index      1/2/3/4 for lark, 1/2 for lark-lite
 * @param  pol        0: active low, 1: active high
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_set_irq_pol(adi_lark_device_t *device, uint8_t index, uint8_t pol);

/**
 * @brief  Enable interrupt request
 *         
 * @param  device     Pointer to device structure
 * @param  index      1/2/3/4 for lark, 1/2 for lark-lite
 * @param  irq_src    @see adi_lark_irq_src_e
 * @param  enable     0 to disable, 1 to enable
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_enable_irq(adi_lark_device_t *device, uint8_t index, adi_lark_irq_src_e irq_src, uint8_t enable);

/**
 * @brief  Get interrupt status
 *         
 * @param  device     Pointer to device structure
 * @param  index      1/2/3/4 for lark, 1/2 for lark-lite
 * @param  irq_src    @see adi_lark_irq_src_e
 * @param  status     0 - active, 1 - masked, will not cause IRQ
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_get_irq_status(adi_lark_device_t *device, uint8_t index, adi_lark_irq_src_e irq_src, uint8_t *status);

/**
 * @brief  Trigger TDSP software interrupt request, for lark only.
 *         
 * @param  device     Pointer to device structure
 * @param  index      1/2/3/4
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_trig_tdsp_sw_irq(adi_lark_device_t *device, uint8_t index);

/**
 * @brief  Clear muti-purpose input interrupt request, for lark only.
 *         
 * @param  device          Pointer to device structure
 * @param  mp_in_irq_index 1/2/3 for lark
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_clr_mp_input_irq(adi_lark_device_t *device, uint8_t mp_in_irq_index);

/**
 * @brief  Set muti-purpose input interrupt request working mode, for lark only.
 *         
 * @param  device          Pointer to device structure
 * @param  mp_in_irq_index 1/2/3 for lark
 * @param  mode            0: active low, 1: active high, 2: rising edge, 3: falling edge
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_set_mp_input_irq_mode(adi_lark_device_t* device, uint8_t mp_in_irq_index, uint8_t mode);

/**
 * @brief  Set muti-purpose input interrupt request enable, for lark only.
 *         
 * @param  device          Pointer to device structure
 * @param  mp_in_irq_index mp input irq1/2/3 for lark
 * @param  mp_in_irq       bit0: mp input0, bit1: mp input1, ..., bit25: mp input25 
 * @param  enable          0 to disable, 1 to enable
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_enable_mp_input_irq(adi_lark_device_t* device, uint8_t mp_in_irq_index, uint32_t mp_in_irq, uint8_t enable);

/**
 * @brief  Set muti-purpose input interrupt request enable, for lark only.
 *         
 * @param  device          Pointer to device structure
 * @param  mp_in_irq_index mp input irq1/2/3 for lark
 * @param  mp_in_bit_index 0 - IRQ1, 1 - IRQ1, 2 - IRQ3
 * @param  status          Pointer of status
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_get_mp_input_irq_status(adi_lark_device_t* device, uint8_t mp_in_irq_index, uint32_t mp_in_bit_index, uint8_t *status);

/**
 * @brief  Get SOC (TDSP) interrupt request id for xtos, for lark only.
 *         
 * @param  device          Pointer to device structure
 * @param  irq             @see adi_lark_soc_irq_src_e
 * @param  id              Pointer to TDSP interrupt id
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_get_soc_irq_id(adi_lark_device_t* device, adi_lark_soc_irq_src_e irq, uint32_t *id);

/**
 * @brief  Get SOC (TDSP) interrupt request priority, for lark only.
 *         
 * @param  device          Pointer to device structure
 * @param  irq             @see adi_lark_soc_irq_src_e
 * @param  pri             @see adi_lark_soc_irq_pri_e
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_int_set_soc_irq_priority(adi_lark_device_t* device, adi_lark_soc_irq_src_e irq, adi_lark_soc_irq_pri_e pri);

/**
 * @brief  Reset TDSP (hifi3z), for lark only.
 *         
 * @param  device     Pointer to device structure
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_tdsp_reset(adi_lark_device_t *device);

/**
 * @brief  Set TDSP (hifi3z) alternative vector enable and address, for lark only.
 *         
 * @param  device       Pointer to device structure
 * @param  enable       true to enable alternative vector
 * @param  alt_vec_addr alternative vector address
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_tdsp_set_alt_vec(adi_lark_device_t *device, bool enable, uint32_t alt_vec_addr);

/**
 * @brief  Enable TDSP (hifi3z) run, for lark only.
 *         
 * @param  device       Pointer to device structure
 * @param  enable       true to enable TDSP run, false to stall it.
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_tdsp_enable_run(adi_lark_device_t *device, bool enable);

/**
 * @brief  DMA init function, for lark only.
 *         
 * @param  device                   Pointer to device structure
 * @param  primary_ctrl_data_base   base address of primary control data structure base. 
 *                                  The control data structure array must be addressable from Lark.
 *                                  @see adi_lark_dma_chnl_ctrl_t. The host platform must define a
 *                                  structure array, for example:
 *                                  adi_lark_dma_chnl_ctrl_t control_data_array[max_dma_channel_supported],
 *                                  control_data_array must be in LARK's memory space, and provide the
 *                                  address of control_data_array as this input parameter.
 *                                  The base address must be 1024-byte aligned.
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_dma_init(adi_lark_device_t *device, uint32_t primary_ctrl_data_base);

/**
 * @brief  Update control data of a specified channel. Must be called after @see adi_lark_dma_set_ctrl_base_reg(), for lark only.
 * 
 * @param  device                   Pointer to device structure
 * @param  primary_alternate_flag   0 - using primary control data; 1 - using alternate control data
 * @param  dma_channel              DMA channel number
 * @param  ctrl_data                Pointer to the control data to to be updated
 *
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_update_chnl_ctrl_data(adi_lark_device_t *device, uint8_t primary_alternate_flag, 
    adi_lark_dma_chno_e dma_channel, adi_lark_dma_chnl_ctrl_t *ctrl_data);

/**
 * @brief  enable or disable DMA controler, for lark only.
 *
 * @param  device       Pointer to device structure
 * @param  enable_flag  true to enable, false to disable the controler
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Get DMA control data base register value, for lark only.
 * 
 * @param  device                   Pointer to device structure
 * @param  primary_alternate_flag   0 - using primary control data; 1 - using alternate control data
 * @param  reg_data                 Pointer to store register value read out. 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_ctrl_base_reg(adi_lark_device_t *device, uint8_t primary_alternate_flag, uint32_t *reg_data);

/**
 * @brief  Set DMA control data base register value, for lark only.
 * 
 * @param  device                   Pointer to device structure
 * @param  reg_data                 Base address of the control data structure. The control data must be in Lark memory space. 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_set_ctrl_base_reg(adi_lark_device_t *device, uint32_t reg_data);

/**
 * @brief  Enable or disable DMA controler to use alternate data for a specified DMA channel, for lark only.
 * @note   If alternate control data is used, alternate control data register must be set. @see adi_lark_dma_set_ctrl_base_reg
 *
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable channel alternate control data, false - disable it. 
 *
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl_alt_ctrl(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag);

/**
 * @brief  Enable or disable byte swap for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable, false - disable. 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl_byteswap(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag);

/**
 * @brief  Enable or disable destination pointer decrement for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable, false - disable. @see adi_lark_dma_incr_type_e
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl_dst_decrement(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag);

/**
 * @brief  Enable or disable source pointer decrement for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable, false - disable. 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl_src_decrement(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag);

/**
 * @brief  Enable or disable a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable, false - disable.
 * @param  mdma_flag    true - mdma, set channel mask, false - peripheral DMA, clr channel mask.  
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag, bool mdma_flag);

/**
 * @brief  Enable or disable the specified DMA channels, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channels Channels to be configured, channel N will be enable or disable if Bit N is 1.
 *                      If Bit N is 0, channel N will not be affected and the current status is kept.
 * @param  enable_flag  true - enable, false - disable.
 * @param  mdma_flag    true - mdma, set channel mask, false - peripheral DMA, clr channels masks.  
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_multi_chnls(adi_lark_device_t *device, uint32_t dma_channels, bool enable_flag, bool mdma_flag);

/**
 * @brief  Read DMA channel request mask set register, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  reg_data     Pointer to store the register data read out
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_req_mask(adi_lark_device_t *device, uint32_t *reg_data);
/**
 * @brief  Clear bus error for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_chnl_buserr_stat(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel);

/**
 * @brief  Clear bus error for the specified DMA channels, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channels Channels number to be clreared. If bit n is 1, channel n will be cleared.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_buserr_stats(adi_lark_device_t *device, uint32_t dma_channels);

/**
 * @brief  Read bus error status register, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  reg_data     Pointer to store the register data read out
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_buserr_stat(adi_lark_device_t *device, uint32_t *reg_data);

/**
 * @brief  Clear invalid descriptor error for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_chnl_invalid_desc_stat(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel);

/**
 * @brief  Clear invalid descriptor error for the specified DMA channels, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channels Channels number to be cleared. If bit n is 1, channel n will be cleared.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_invalid_desc_stats(adi_lark_device_t *device, uint32_t dma_channels);

/**
 * @brief  Read invalid descriptor status register, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  reg_data     Pointer to store the register data read out, TODO: more comments
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_invalid_desc_stat(adi_lark_device_t *device, uint32_t *reg_data);

/**
 * @brief  Enable or disable high priority for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable, false - disable. 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl_high_priority(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag);

/**
 * @brief  Start DMA software request for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_start_chnl_softdma(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel);

/**
 * @brief  Get DMA channel enable status, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  en_stat      Pointer to save DMA channel enable status 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_chnl_en_stat(adi_lark_device_t *device, uint32_t *en_stat);

/**
 * @brief  Read DMA status register, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  reg_data     Pointer to store the register data read out
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_stat(adi_lark_device_t *device, uint32_t *reg_data);

/**
 * @brief  Get how many DMA channels this DMA controller can support, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  reg_data     Pointer to store the register data read out
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_get_avail_channels(adi_lark_device_t *device, uint32_t *reg_data);

/**
 * @brief  Enable or disable DMA interrupt for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 * @param  enable_flag  true - enable, false - disable. 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_chnl_dma_done_int(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag);

/**
 * @brief  Enable or disable DMA error interrupt, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_err_type error type:
 *                      0 - all clear, 
 *                      1 - enable invalid descriptor interrupt
 *                      2 - enable bus error interrupt
 *                      3 - enable all
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_enable_dma_err_int(adi_lark_device_t *device, uint8_t dma_err_type);

/**
 * @brief  Clear DMA interrupt request for a specified DMA channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_channel  Channel number to be configured
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_chnl_int_stat(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel);

/**
 * @brief  Clear all the DMA interrupt request if the corresponding bit is 1. Added this function because all DMAs have only 1 interrupt, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  int_stat_set 
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_int_stat(adi_lark_device_t *device, uint32_t int_stat_set);

/**
 * @brief  clear DMA error interrupt stat, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  dma_err_type error type:
 *                      0 - clear none, 
 *                      1 - clear invalid descriptor interrupt stat
 *                      2 - clear bus error interrupt stat
 *                      3 - enable all
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dma_clr_err_int_stat(adi_lark_device_t *device, uint8_t dma_err_type);

/**
 * @brief  DMA test function, for lark only.
 *
 * @param  device       Pointer to device structure
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
int32_t adi_lark_dma_test(adi_lark_device_t *device);
#endif

/**
 * @brief  Enable or disable data sync DMA request of specified channel, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  ds_dma_chan  Channel number to be configured
 * @param  enable_flag  true - enable this channel to generate DMA request to DMA controller.
 *                      false - disable DMA reqeust
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_chnl_dmareq(adi_lark_device_t *device, uint32_t ds_dma_chan, bool enable_flag);

/**
 * @brief  Configure data sync DMA request register, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  ds_dma_chans Channel number to be configured. If Bit N is 1, Channel N's request will be affected.
 *                      If Bit N is 0, channel N's request will not be affected.
 * @param  enable_flag  true - Channels' reqeust whose bit==1 will be enabled. false - Channels' request whose bit==1 will be disabled  
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_multi_chnls_dmareq(adi_lark_device_t *device, uint32_t ds_dma_chans, bool enable_flag);

/**
 * @brief  Set data ready frequency, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  ready_div    Rate Division for Generating Ready Signal from 24M clock in Data Sync. Default is for 48K.
 *                      The frequency will 24MHz/(ready_div + 1)
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_set_rdy_rate(adi_lark_device_t *device, uint16_t ready_div);

/**
 * @brief  Set Data Ready Signal Width (Unit is Core Clock Period) for TIE Lookup table Reading, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  lt_rdy_width Let it = n, if n is in [1,255], the width is n * core clock period. 
 *                      if n = 0, data ready signal will be auto clear through reading dedicated address of lookup table.   
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
 
int32_t adi_lark_ds_set_lt_rdy_width(adi_lark_device_t *device, uint8_t lt_rdy_width);

/**
 * @brief  Enable or disable TIE lookup table interface, for lark only.
 * 
 * @param  device       Pointer to device structure
 * @param  enable_flag  true - enable, false - disable
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_tie_ltif(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Enable data sync output channels (TDSP0-15) to resync to other clock source. Enable only, source not selected. For lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  out_resync_chnls Channel number to be configured. If Bit N is 1, Output channel N will be affected.
 *                          If Bit N is 0, output channel N will not be affected.
 * @param  enable_flag      true - channels whose bit == 1 will be enabled to sync with selected source, false - disable  
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_multi_out_chnls_resync(adi_lark_device_t *device, uint16_t out_resync_chnls, bool enable_flag);

/**
 * @brief  Enable data sync output channel (TDSP0-15) to resync to other clock source, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  tdsp_out_chnl    Data Sync output channel (TDSP0-15) to be configured.
 * @param  enable_flag      true - enable, false - disable  
 * @param  rdy_chnl         output clock source channel.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_tdsp_out_chnl_resync(adi_lark_device_t *device, uint16_t tdsp_out_chnl, bool enable_flag, adi_lark_ds_rdy2out_chnl_e rdy_chnl);

/**
 * @brief  Configure data sync interrupt masks for data ready, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  chnls            Channel number to be configured. If Bit N is 1, channel N will be affected.
 *                          If Bit N is 0, channel N will not be affected. @see adi_lark_ds_int_chnl_e
 * @param  enable_flag      Flag of enable. true: enable (clear mask of the specific channel), false: disable
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_multi_chnls_int(adi_lark_device_t *device, adi_lark_ds_int_chnl_e chnls, bool enable_flag);

/**
 * @brief  Enable data sync channel interrupt, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  int_chnl         Channel number to be set. @see adi_lark_ds_int_chnl_e
 * @param  enable_flag      Flag of enable. true: enable (clear mask of the specific channel), false: disable
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_chnl_int(adi_lark_device_t *device, adi_lark_ds_int_chnl_e int_chnl, bool enable_flag);

/**
 * @brief  read data sync interrupt status, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  int_stat         Pointer to where store interrupt status.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_read_int_status(adi_lark_device_t *device, uint32_t *int_stat);

/**
 * @brief  clear data sync interrupt status of a specific channel, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  int_chnl         Channel to be cleared.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_clear_chnl_int_status(adi_lark_device_t *device, adi_lark_ds_int_chnl_e int_chnl);

/**
 * @brief  clear data sync interrupt status, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  int_chnls        Channels to be cleared. If bit n is set, channel n is cleared
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_clear_int_status(adi_lark_device_t *device, uint32_t int_chnls);

/**
 * @brief  enable auto clear interrupt status of a specific channel, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  int_chnl         Channel to be cleared. Range from 0 to 16.
 * @param  enable_flag      true - enable, false - disable.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_autoclear_chnl_int_status(adi_lark_device_t *device, adi_lark_ds_int_chnl_e int_chnl, bool enable_flag);

/**
 * @brief  Select DMA request source for a Data Sync DMA channel, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  dma_chnl         Data Sync DMA channel number (0~15) to be configured.
 * @param  rdy_chnl         DMA request source to trigger the DMA.
 *                          For example, if rdy_chnl is API_LARK_DS_INPUT_ADC0, every ADC0 frame will generate a DMA request.
 *                          The DMA data source or destination does not have to be ADC0 related. It can be flexible.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_select_rdy2dma_chnl(adi_lark_device_t *device, uint8_t dma_chnl, adi_lark_ds_rdy2dma_chnl_e rdy_chnl);

/**
 * @brief  Select ready signal for a output channel, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  out_chnl         output channel number to be configured.
 * @param  rdy_chnl         output clock source channel.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_select_rdy2out_chnl(adi_lark_device_t *device, uint8_t out_chnl, adi_lark_ds_rdy2out_chnl_e rdy_chnl);

/**
 * @brief  Select data sync fifo source, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_chnl        The fifo number to be configured.
 * @param  src_chnl         input source channel number.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_select_fifo_chnl_src(adi_lark_device_t *device, uint8_t fifo_chnl, adi_lark_ds_fifo_src_chnl_e src_chnl);

/**
 * @brief  Select data sync fifo destination, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_chnl        The fifo number to be configured.
 * @param  dst_chnl         output destination channel number.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_select_fifo_chnl_dst(adi_lark_device_t *device, uint8_t fifo_chnl, uint8_t dst_chnl);

/**
 * @brief  Enable data sync fifo read, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_chnl        The fifo number to be configured.
 * @param  enable_flag      true - enable, false - disable.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_fifo_chnl_read(adi_lark_device_t *device, uint8_t fifo_chnl, bool enable_flag);

/**
 * @brief  Enable or disable mutiple data sync fifo read, for lark only 
 * @param  device           Pointer to device structure
 * @param  chnl_bits_set    If the bit is set, the channel will be affected; if the bit is clear, the channel will not be affected 
 * @param  enable_flag      Enable or disable flag:
 *                          true - enable the channels whose bit is set, false - disable the channels whose bit is clear 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_multi_fifo_chnls_read(adi_lark_device_t *device, uint16_t chnl_bits_set, bool enable_flag);

/**
 * @brief  Enable data sync fifo write, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_chnl        The fifo number to be configured.
 * @param  enable_flag      true - enable, false - disable.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_fifo_chnl_write(adi_lark_device_t *device, uint8_t fifo_chnl, bool enable_flag);

/**
 * @brief  Enable or disable mutiple data sync fifo write, for lark only 
 * @param  device           Pointer to device structure
 * @param  chnl_bits_set    If the bit is set, the channel will be affected; if the bit is clear, the channel will not be affected 
 * @param  enable_flag      Enable or disable flag:
 *                          true - enable the channels whose bit is set, false - disable the channels whose bit is clear 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ds_enable_multi_fifo_chnls_write(adi_lark_device_t *device, uint16_t chnl_bits_set, bool enable_flag);

/**
 * @brief  Config adi_lark_sap_config_t for I2S.
 * 
 * @param  device           Pointer to device structure
 * @param  config_ptr       Pointer to @see adi_lark_sap_config_t to be constructed.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_init_config_as_i2sdefault(adi_lark_device_t *device, adi_lark_sap_config_t *config_ptr);

/**
 * @brief  Configure all the control features of SAP
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  config_ptr       Pointer to @see adi_lark_sap_config_t to be used.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_config_all(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_config_t *config_ptr);

/**
 * @brief  Set serial audio interface mode
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  sai_mode         sai mode @see adi_lark_sap_sai_mode_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_sai_mode(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_sai_mode_e sai_mode);

/**
 * @brief  Set SAP data format
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  data_format      data format @see adi_lark_sap_data_format_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_data_format(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_data_format_e data_format);

/**
 * @brief  Set SAP slot width
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  slot_width       slot width @see adi_lark_sap_slot_width_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_slot_width(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_slot_width_e slot_width);

/**
 * @brief  Set SAP tri-state
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  tri_state        tri-state @see adi_lark_sap_tristate_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_tri_state(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_tristate_e tri_state);

/**
 * @brief  Set SAP bit clock source
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  bclk_src         bit clock source @see adi_lark_sap_bclk_src_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_bclk_src(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_bclk_src_e bclk_src);

/**
 * @brief  Set SAP bit clock polarity
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  bclk_pol         bit clock polarity @see adi_lark_sap_bclk_polarity_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_bclk_pol(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_bclk_polarity_e bclk_pol);

/**
 * @brief  Set SAP frame (LR) clock source
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  lrclk_src        bit clock source @see adi_lark_sap_lrclk_src_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_lrclk_src(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_lrclk_src_e lrclk_src);

/**
 * @brief  Set SAP frame (LR) clock polarity
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  lrclk_pol        frame clock polarity @see adi_lark_sap_lrclk_polarity_e.
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_set_lrclk_pol(adi_lark_device_t *device, uint32_t port_number, adi_lark_sap_lrclk_polarity_e lrclk_pol);

/**
 * @brief  Select SAP SPT output slot data source
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  slot_id          The Slot id to be configured. Slot 0 to Slot 15
 * @param  route_from       Where the output data from. @see adi_lark_sap_output_route_from_e
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_select_out_route(adi_lark_device_t *device, uint32_t port_number, uint32_t slot_id, adi_lark_sap_output_route_from_e route_from);

/**
 * @brief  Turn on/off SAP input
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  enable_flag      1 - turn on input, 0 - turn off input
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_enable_chnl_input(adi_lark_device_t *device, uint32_t port_number, bool enable_flag);

/**
 * @brief  Turn on/off SAP output
 * 
 * @param  device           Pointer to device structure
 * @param  port_number      The SPT id to be configured. 0 - SPT0, 1 - SPT1
 * @param  enable_flag      1 - turn on output, 0 - turn off output
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_sap_enable_chnl_output(adi_lark_device_t *device, uint32_t port_number, bool enable_flag);

/**
 * @brief  Initialize FIFO sram, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation to be initialized. @see adi_lark_ffsram_fifo_info_t
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_init(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info);

/**
 * @brief  Get FIFO length of 256-byte, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  len_of_256B      Pointer to the varialble to store the lenghth.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_fifo_len(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *len_of_256B);

/**
 * @brief  Get FIFO base offset of 256-byte, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  offset_of_256B   Pointer to the varialble to store the base offset.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_fifo_base(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *offset_of_256B);

/**
 * @brief  Read FIFO, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  data_len         Pointer to read length of word width. Input: target length. Ouput: Length actually read
 * @param  data_buf         Pointer to read buffer. Pass (int32_t*)data_buf to this function if word width is 16-bit and the buffer is type of (int16_t*)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_read_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *data_len, int32_t *data_buf);

/**
 * @brief  Write FIFO, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  data_len         Pointer to write length of word width. Input: target length. Ouput: Length actually written
 * @param  data_buf         Pointer to write buffer. Pass (int32_t*)data_buf to this function if word width is 16-bit and the buffer is type of (int16_t*)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_write_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *data_len, int32_t *data_buf);

/**
 * @brief  Get FIFO available size (of bytes), for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  byte_len         Pointer to availabe byte length in the FIFO. Output.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_avail_size(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *byte_len);

/**
 * @brief  Get FIFO available size (of bytes), for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  byte_len         Pointer to free byte length in the FIFO. Output.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_free_size(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *byte_len);

/**
 * @brief  Get FIFO access bit width (of bits), for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  width            Pointer to FIFO width. Output.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_fifo_width(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *width);

/**
 * @brief  Get FIFO read pointer offset (of bytes), for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  rptr_offset      Pointer to FIFO read pointer offset of bytes. Output.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_read_ptr(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *rptr_offset);

/**
 * @brief  Get FIFO write pointer offset (of bytes), for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  wptr_offset      Pointer to FIFO write pointer offset of bytes. Output.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_write_ptr(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *wptr_offset);

/**
 * @brief  Enable or disable normal FIFO mode access, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  enable_flag      true: enable FIFO and disable delayline at the same time, false: disable FIFO.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_enable_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, bool enable_flag);

/**
 * @brief  Get fifo sram channel normal enable status, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  enable_state     to store the enable status which is read back.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_chnl_enable_state(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *enable_state);
/**
 * @brief  Enable or disable delay line mode FIFO access, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  enable_flag      true: disable normal FIFO mode and enable delayline at the same time, false: disable delayline.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_enable_delay_line(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, bool enable_flag);

/**
 * @brief  Flush fifo
 * @param  device           Pointer to device structure, for lark only.
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_flush_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info);

/**
 * @brief  Enable FIFO interrupt, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  int_src          Interrupt source @see adi_lark_ffsram_int_src_e
 * @param  enable_flag      Enable flag: true - enable, false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_enable_int(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, adi_lark_ffsram_int_src_e int_src, bool enable_flag);

/**
 * @brief  Get FIFO interupt status, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  int_stat_all     Pointer to data to store the status. If a bit is set, that condition is matched, 
 *                          @see adi_lark_ffsram_int_src_e for bit position.
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_fifo_int_stat(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *int_stat_all);

/**
 * @brief  Clear FIFO status, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  int_src          Interrupt source @see adi_lark_ffsram_int_src_e
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_clr_fifo_int_stat(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, adi_lark_ffsram_int_src_e int_src);

/**
 * @brief  Set FIFO almost-full threshold, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  almf_th          Threshold of almost-full. Unit: word (a word could be 32-bit or 16-bit)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_set_th_full(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t almf_th);

/**
 * @brief  Get FIFO almost-full threshold, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  almf_th          Pointer to threshold of almost-full. Unit: word (a word could be 32-bit or 16-bit)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_th_full(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *almf_th);

/**
 * @brief  Set FIFO almost-empty threshold, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  alme_th          Threshold of almost-empty. Unit: word (a word could be 32-bit or 16-bit)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_set_th_empty(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t alme_th);

/**
 * @brief  Get FIFO almost-empty threshold, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  alme_th          Pointer to threshold of almost-empty. Unit: word (a word could be 32-bit or 16-bit)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_th_empty(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *alme_th);

/**
 * @brief  Set FIFO delay line length, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  dline_len        Delay line length. Unit: word (a word could be 32-bit or 16-bit).
 *                          Max: 0x3FFF for 32KB SRAM, 0x7FFF for 64K SRAM. Min: 0.
 *                          Read Pointer has fixed relation with Write Pointer: ReadPointer = WritePointer - dline_len - 1
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_set_dline_len(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t dline_len);

/**
 * @brief  Get FIFO delay line length, for lark only.
 * @param  device           Pointer to device structure
 * @param  fifo_info        Pointer to FIFO infomation. @see adi_lark_ffsram_fifo_info_t
 * @param  dline_len        Pointer to Delay line length. Unit: word (a word could be 32-bit or 16-bit)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_ffsram_get_dline_len(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *dline_len);

/**
 * @brief  Enable or disable specific decimator channel.
 * @param  device           Pointer to device structure
 * @param  chnl_no          Channel to be enabled/disabled
 * @param  enable_flag      Enable or disable flag: true - enable, false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdec_enable_chnl_power_on(adi_lark_device_t *device, uint8_t chnl_no, bool enable_flag);

/**
 * @brief  Enable or disable multiple decimator channels 
 * @param  device           Pointer to device structure
 * @param  chnl_bits_set    If the bit is set, the channel will be affected; if the bit is clear, the channel will not be affected 
 * @param  enable_flag      Enable or disable flag:
 *                          true - enable the channels whose bit is set, false - disable the channels whose bit is clear 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdec_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable_flag);

/**
 * @brief  Select sample frequency of a pair of decimator channels. 
 * @param  device           Pointer to device structure
 * @param  pair_chnls       Channel pair to be set
 * @param  in_fs            Input sample rate
 * @param  out_fs           output sample rate
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdec_select_pair_chnls_fs(adi_lark_device_t *device, adi_lark_fdec_pair_chnl_e pair_chnls, adi_lark_fdec_in_fs_e in_fs, adi_lark_fdec_out_fs_e out_fs);

/**
 * @brief  Select the route source of a decimator channel. 
 * @param  device           Pointer to device structure
 * @param  dec_chnl         Decimator channel to be configured
 * @param  route_chnl       Input/Output channel for the decimator channel
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdec_select_chnl_route(adi_lark_device_t *device, uint8_t dec_chnl, adi_lark_fdec_route_chnl_e route_chnl);

/**
 * @brief  Enable or disable specific interpolator channel 
 * @param  device           Pointer to device structure
 * @param  chnl_no          Channel to be enabled/disabled
 * @param  enable_flag      Enable or disable flag: true - enable, false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fint_enable_chnl_power_on(adi_lark_device_t *device, uint8_t chnl_no, bool enable_flag);

/**
 * @brief  Enable or disable mutiple interpolator channels 
 * @param  device           Pointer to device structure
 * @param  chnl_bits_set    If the bit is set, the channel will be affected; if the bit is clear, the channel will not be affected 
 * @param  enable_flag      Enable or disable flag:
 *                          true - enable the channels whose bit is set, false - disable the channels whose bit is clear 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fint_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable_flag);

/**
 * @brief  Select sample frequency of a pair of interpolator channels. 
 * @param  device           Pointer to device structure
 * @param  pair_chnls       Channel pair to be set
 * @param  in_fs            Input sample rate
 * @param  out_fs           output sample rate
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fint_select_pair_chnls_fs(adi_lark_device_t *device, adi_lark_fint_pair_chnl_e pair_chnls, adi_lark_fint_in_fs_e in_fs, adi_lark_fint_out_fs_e out_fs);

/**
 * @brief  Select the route srouce of a interpolator channel. 
 * @param  device           Pointer to device structure
 * @param  dec_chnl         Decimator channel to be configured
 * @param  route_chnl       Input/Output channel for the decimator channel
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fint_select_chnl_route(adi_lark_device_t *device, uint8_t dec_chnl, adi_lark_fint_route_chnl_e route_chnl);

/**
 * @brief  Enable or disable ASRC channel. 
 * @param  device           Pointer to device structure
 * @param  chnl_no          ASRCI/O channel to be configured, @see adi_lark_asrc_chnl_e
 * @param  enable_flag      Enable flag: true - enable, false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_enable_chnl_power_on(adi_lark_device_t *device, adi_lark_asrc_chnl_e chnl_no, bool enable_flag);

/**
 * @brief  Enable or disable multiple ASRC channels. 
 * @param  device           Pointer to device structure
 * @param  chnl_bits_set    If the bit is set, the channel will be affected; if the bit is clear, the channel will not be affected 
 * @param  enable_flag      Enable or disable flag:
 *                          true - enable the channels whose bit is set, false - disable the channels whose bit is clear 
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable_flag);

/**
 * @brief  Select ASRCI input channel. 
 * @param  device           Pointer to device structure
 * @param  asrci_chnl       ASRCI channel to be configured
 * @param  route_from       ASRCI input source channel  
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_select_asrci_route(adi_lark_device_t *device, uint8_t asrci_chnl, uint8_t route_from);

/**
 * @brief  Select ASRCO input channel. 
 * @param  device           Pointer to device structure
 * @param  asrco_chnl       ASRCO channel to be configured
 * @param  route_from       ASRCO input source channel  
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_select_asrco_route(adi_lark_device_t *device, uint8_t asrco_chnl, adi_lark_ascro_route_chnl_e route_from);

/**
 * @brief  Select ASRCI output frequency. 
 * @param  device           Pointer to device structure
 * @param  out_fs           ASRCI output frequency, @see adi_lark_asrc_fs_e  
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_select_asrci_fs(adi_lark_device_t *device, adi_lark_asrc_fs_e out_fs);

/**
 * @brief  Select ASRCO input frequency. 
 * @param  device           Pointer to device structure
 * @param  in_fs            ASRCO input frequency, @see adi_lark_asrc_fs_e  
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_select_asrco_fs(adi_lark_device_t *device, adi_lark_asrc_fs_e in_fs);

/**
 * @brief  Select ASRCI input serial audio interface. 
 * @param  device           Pointer to device structure
 * @param  sai_port         Serial audio port number, 0 - SAI 0, 1 - SAI 1 (LARK only)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_select_asrci_sai(adi_lark_device_t *device, uint8_t sai_port);

/**
 * @brief  Select ASRCO output serial audio interface. 
 * @param  device           Pointer to device structure
 * @param  sai_port         Serial audio port number, 0 - SAI 0, 1 - SAI 1 (LARK only)
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_select_asrco_sai(adi_lark_device_t *device, uint8_t sai_port);

/**
 * @brief  Set ASRCI power level. 
 * @param  device           Pointer to device structure
 * @param  power_level      Power Level: 0 - lowest power, 1 - low power, others - high performance
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_set_asrci_power_level(adi_lark_device_t *device, uint8_t power_level);

/**
 * @brief  Set ASRCO power level. 
 * @param  device           Pointer to device structure
 * @param  power_level      Power Level: 0 - lowest power, 1 - low power, others - high performance
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_set_asrco_power_level(adi_lark_device_t *device, uint8_t power_level);

/**
 * @brief  Enable ASRCI voice band filter. 
 * @param  device           Pointer to device structure
 * @param  enable_flag      enable flag: true - enable , false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_enable_asrci_vfilter(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Enable ASRCI addtional voice filter. Setting additional voice filter is independent of voice filter.
 * @param  device           Pointer to device structure
 * @param  enable_flag      enable flag: true - enable , false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_enable_asrci_addi_vfilter(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Enable ASRCO voice filter. 
 * @param  device           Pointer to device structure
 * @param  enable_flag      enable flag: true - enable , false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_enable_asrco_vfilter(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Enable ASRCO addtional voice filter. Setting additional voice filter is independent of voice filter.
 * @param  device           Pointer to device structure
 * @param  enable_flag      enable flag: true - enable , false - disable
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_asrc_enable_asrco_addi_vfilter(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Get chip power mode status, for lark only.
 * @param  device           Pointer to device structure
 * @param  mode             Pointer to power mode storage
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_get_chip_power_mode(adi_lark_device_t *device, adi_lark_pwr_mode_e *mode);

/**
 * @brief  Set chip power mode.
 * @param  device           Pointer to device structure
 * @param  mode             Power mode @see adi_lark_pwr_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_set_chip_power_mode(adi_lark_device_t *device, adi_lark_pwr_mode_e mode);

/**
 * @brief  Enable or disable master block power. 
 * @param  device           Pointer to device structure
 * @param  enable_flag      Enable flag: true - enable, false - disable
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_enable_master_block(adi_lark_device_t *device, bool enable_flag);

/**
 * @brief  Set memory volatge mode, for lark only. 
 * @param  device           Pointer to device structure
 * @param  mode             Memory volatge mode, @see adi_lark_pmu_memory_voltage_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_set_memory_voltage_mode(adi_lark_device_t *device, adi_lark_pmu_memory_voltage_mode_e mode);

/**
 * @brief  Get memory volatge mode, for lark only. 
 * @param  device           Pointer to device structure
 * @param  mode             Memory volatge mode, @see adi_lark_pmu_memory_voltage_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_get_memory_voltage_mode(adi_lark_device_t *device, adi_lark_pmu_memory_voltage_mode_e *mode);

/**
 * @brief  Set retention control mask for ADP and SOC memories, for lark only. 
 * @param  device           Pointer to device structure
 * @param  memory_mask      Memory mask, @see adi_lark_pmu_mem_retain_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_retain_memory_power(adi_lark_device_t *device, adi_lark_pmu_mem_retain_e memory_mask);

/**
 * @brief  Enable/disable high power CM startup boost mode.
 * @param  device           Pointer to device structure
 * @param  enable           true - CM pin fast charge disabled, false - CM pin fast charge enabled. Setting to false can save some power.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_enable_cm_pin_fast_charge(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set dldo output voltage.
 * @param  device           Pointer to device structure
 * @param  output           output, @see adi_lark_pmu_dldo_output_e. Must set to 1.1v when running at 98MHz.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_set_dldo_output_voltage(adi_lark_device_t *device, adi_lark_pmu_dldo_output_e output);

/**
 * @brief  Enable/disable PMU irq wakeup, for lark only
 * @param  device           Pointer to device structure
 * @param  enable_mask      Enable Mask, @see adi_lark_pmu_wakeup_irq_e           
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_enable_irq_wakeup(adi_lark_device_t *device, adi_lark_pmu_wakeup_irq_e enable_mask);

/**
 * @brief  Get PMU irq wakeup enable bits, for lark only
 * @param  device           Pointer to device structure
 * @param  enable_mask      Enable bits           
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_get_irq_wakeup_en(adi_lark_device_t *device, uint32_t *enable_mask);

/**
 * @brief  Enable or disable PMU delay count bypass, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true: bypass delay count, false: not bypass. Set to true to get chip working when 35ms delay is reached in non-selfboot mode.      
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pmu_enable_dlycnt_byp(adi_lark_device_t *device, bool enable);

/**
 * @brief  Power on/off XTAL, for lark only. 
 * @param  device           Pointer to device structure
 * @param  enable           true - Power on, false - Power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_enable_xtal_power_on(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set xtal mode, for lark only
 *         
 * @param  device       Pointer to device structure
 * @param  mode         @see adi_lark_clk_xtal_mode_e
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_select_xtal_mode(adi_lark_device_t *device, adi_lark_clk_xtal_mode_e mode);

/**
 * @brief  Power on/off PLL. 
 * @param  device           Pointer to device structure
 * @param  enable           true - Power on, false - Power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_enable_pll_power_on(adi_lark_device_t *device, bool enable);

/**
 * @brief  Configure PLL, PLL output = (input_clk/prescaler)*multiplier for integer mode, = (input_clk/prescaler)*(multiplier + numerator/denominator) for fractional mode
 *         
 * @param  device       Pointer to device structure
 * @param  pll_src      @see adi_lark_clk_pll_source_e
 * @param  type         @see adi_lark_clk_pll_type_e
 * @param  sync_src     @see adi_lark_clk_sync_source_e
 * @param  prescaler    pll divider
 * @param  multiplier   pll multiplier
 * @param  numerator    pll numerator
 * @param  denominator  pll denominator
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_config_pll(adi_lark_device_t *device, adi_lark_clk_pll_source_e pll_src, adi_lark_clk_pll_type_e type, 
    adi_lark_clk_sync_source_e sync_src, uint8_t prescaler, uint16_t multiplier, uint16_t numerator, uint16_t denominator);

/**
 * @brief  Set and update PLL. Please call adi_lark_clk_set_mclk_freq to enable or disable PLL bypass after calling this function.
 *         
 * @param  device           Pointer to device structure
 * @param  xtal_mode        @see adi_lark_clk_xtal_mode_e
 * @param  pll_src          @see adi_lark_clk_pll_source_e
 * @param  sync_src         @see adi_lark_clk_sync_source_e
 * @param  input_freq_hz    input frequency Hz
 * @param  output_freq_hz   target frequecy Hz
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_startup_pll(adi_lark_device_t *device, adi_lark_clk_xtal_mode_e xtal_mode, adi_lark_clk_pll_source_e pll_src,
    adi_lark_clk_sync_source_e sync_src, uint32_t input_freq_hz, uint32_t output_freq_hz);

/**
 * @brief  Update all PLL configuration settings.
 *         
 * @param  device       Pointer to device structure
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_update_pll(adi_lark_device_t *device);

/**
 * @brief  Get pll locked status. 
 * @param  device           Pointer to device structure
 * @param  locked           0 - pll unlocked
 *                          1 - pll locked
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_get_pll_locked_status(adi_lark_device_t *device, uint8_t *locked);

/**
 * @brief  Enable/disable 2x clock output, for lark and lark-lite
 *         
 * @param  device       Pointer to device structure
 * @param  enable       false - normal pll output, true - double pll output.
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_enable_2x_output(adi_lark_device_t *device, bool enable);

/**
 * @brief  Get frequency multiplier (2x) output clock locked status, for lark only.
 * @param  device           Pointer to device structure
 * @param  locked           0 - pll unlocked
 *                          1 - pll locked
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_get_2x_locked_status(adi_lark_device_t *device, uint8_t *locked);

/**
 * @brief  Get input and output ASRCI lock status. 
 * @param  device           Pointer to device structure
 * @param  asrci_locked     0 - ASRC currently unlocked  1 - ASRC currently locked
 * @param  asrco_locked     0 - ASRC currently unlocked  1 - ASRC currently locked
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_get_asrc_locked_status(adi_lark_device_t *device, uint8_t *asrci_locked, uint8_t *asrco_locked);

/**
 * @brief  Set TDSP (hifi3z) and bus clock rate, for lark only.
 *         
 * @param  device       Pointer to device structure, tdsp rate and bus rate should be set at same time
 * @param  tdsp_rate    @see adi_lark_tdsp_clk_rate_e, = mclk freq/tdsp freq, TDSP freq must be 1x/2x/3x/4x bus freq.
 * @param  bus_rate     @see adi_lark_bus_clk_rate_e,  = mclk freq/ahb  freq.
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_tdsp_bus_rate(adi_lark_device_t *device, adi_lark_tdsp_clk_rate_e tdsp_rate, adi_lark_bus_clk_rate_e bus_rate);

/**
 * @brief  Set mclk freq, for lark only.
 *         
 * @param  device       Pointer to device structure
 * @param  mclk_freq    @see adi_lark_mclk_freq_e
 * @param  bypass_pll   Enable flag: true - bypass pll, false - use pll
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_mclk_freq(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, bool bypass_pll);

/**
 * @brief  Enable or disable pll bypass, for lark_lite only.
 *         
 * @param  device       Pointer to device structure
 * @param  bypass_pll   Enable flag: true - bypass pll, false - use pll
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_enable_pll_bypass(adi_lark_device_t *device, bool bypass_pll);

/**
 * @brief  Set ffsram clock rate, for lark only..
 *         
 * @param  device       Pointer to device structure
 * @param  clk_rate     @see adi_lark_ffsram_clk_rate_e, ffsram clk = AHB bus clk / (1 + clk_rate)
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_ffsram_clk_rate(adi_lark_device_t *device, adi_lark_ffsram_clk_rate_e clk_rate);

/**
 * @brief  Set uart clock rate, for lark only.
 *         
 * @param  device       Pointer to device structure
 * @param  clk_rate     @see adi_lark_uart_clk_rate_e
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_uart_clk_rate(adi_lark_device_t *device, adi_lark_uart_clk_rate_e clk_rate);

/**
 * @brief  Enable audio data path (ADP ) clock being divided by 3 or not.
 *         
 * @param  device       Pointer to device structure
 * @param  enable       true:  ADP clock is at 8.192MHz, ADC clock is at 2048KHz
 *                      false: ADP clock is at 24.576MHz, ADC clock is at 6144KHz.
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_enable_adp_freq_div3(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set ADP clock enable mask, at default all modules are enabled, for lark only.
 *         
 * @param  device       Pointer to device structure
 * @param  mask         mask, bit0~15: [0]adp_overall/[1]fdsp/[2]pdm/[3]eq/[4]pctrl/[5]adc/[6]dmic/[7]spt/[8]dgen/[9]asrci/[10]asrco/[11]fdec/[12]fint/[13]dac/[14]dbuf/[15]dsync, set bit value to 1 to select module
 * @param  enable       true to enable
 *
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_clk_enable_adp_blocks_clk(adi_lark_device_t *device, uint16_t mask, bool enable);

/**
 * @brief  Set band rate for QSPI data transfers, relating this rate to the SPI serial clock (SCK) and the system clock(SCLK), for lark only..
 * @param  device           Pointer to device structure
 * @param  rate             spi clock = sclk / (rate + 1)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_qspi_clk_rate(adi_lark_device_t *device, uint16_t rate);

/**
 * @brief  Select DMIC clock rate, for lark only.
 * @param  device           Pointer to device structure
 * @param  clk_rate         @see adi_lark_dmic_clk_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_dmic_clk_rate(adi_lark_device_t *device, adi_lark_dmic_clk_rate_e clk_rate);

/**
 * @brief  Select DMIC clock1 rate, for lark only.
 * @param  device           Pointer to device structure
 * @param  clk1_rate         @see adi_lark_dmic_clk_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_dmic_clk1_rate(adi_lark_device_t *device, adi_lark_dmic_clk_rate_e clk1_rate);

/**
 * @brief  Select SAP MCLK output rate
 * @param  device           Pointer to device structure
 * @param  rate             @see adi_lark_sap_mclk_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_set_sap_mclk_out_rate(adi_lark_device_t *device, adi_lark_sap_mclk_rate_e rate);

/**
 * @brief  Select external OSC, 
 * @param  device           Pointer to device structure
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_sel_ext_osc(adi_lark_device_t *device);

/**
 * @brief  Select internal OSC, and bypass PLL
 * @param  device           Pointer to device structure
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_sel_int_osc(adi_lark_device_t *device);

/**
 * @brief  Calibrate internal OSC
 * @param  device           Pointer to device structure
 * @param  lrclk_src        @see adi_lark_clk_osc_cal_ref_sel_e
 * @param  tasks            RC Oscillator Task Timer Threshold.  Numbers of ref_clk for one task.
 * @param  jobs             RC Oscillator Job Timer Threshold. Numbers of task in a job.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_start_osc_cal(adi_lark_device_t *device, adi_lark_clk_osc_cal_ref_sel_e refclk_src, uint8_t tasks, uint8_t jobs);

/**
 * @brief  Get calibrated OSC frequency error
 * @param  device           Pointer to device structure
 * @param  freq_err         Pointer to frequency
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_clk_get_osc_cal_freq_err(adi_lark_device_t *device, int32_t *freq_err);

/**
 * @brief  Power on/off fast DSP. 
 * @param  device           Pointer to device structure
 * @param  enable           Enable: true - power on, false - power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_enable_power_on(adi_lark_device_t *device, bool enable);

/**
 * @brief  Run/stop fast DSP. 
 * @param  device           Pointer to device structure
 * @param  enable           Enable: true - run, false - stop
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_enable_run(adi_lark_device_t *device, bool enable);

/**
 * @brief  Select parameter bank. 
 * @param  device           Pointer to device structure
 * @param  sel              Parameter bank index, @see adi_lark_fdsp_param_bank_e  
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_select_param_bank(adi_lark_device_t *device, adi_lark_fdsp_param_bank_e sel);

/**
 * @brief  Set parameter bank switch settings.
 * @param  device           Pointer to device structure
 * @param  mode             Ramp or instant, @see adi_lark_fdsp_param_switch_mode_e  
 * @param  rate             Ramp or instant, @see adi_lark_fdsp_ramp_rate_e  
 * @param  zero             Enable: true - zero state during bank switch, false - do not zero state during bank switc 
 * @param  stop_point       Parameter bank swicth ramp will stop at lamda/64 of full ramp
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_config_bank_switch_settings(adi_lark_device_t *device, adi_lark_fdsp_param_switch_mode_e mode, adi_lark_fdsp_ramp_rate_e rate, bool zero, uint8_t stop_point);

/**
 * @brief  Copy parameter bank. 
 * @param  device           Pointer to device structure
 * @param  mask             Copy mask @see adi_lark_fdsp_copy_param_bank_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_copy_param_bank(adi_lark_device_t *device, adi_lark_fdsp_copy_param_bank_e mask);

/**
 * @brief  Set rate source and divider. 
 * @param  device           Pointer to device structure
 * @param  source           Rate source @see adi_lark_fdsp_rate_src_e
 * @param  div              Divider, 16 bits
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_set_rate(adi_lark_device_t *device, adi_lark_fdsp_rate_src_e source,  uint16_t div);

/**
 * @brief  Set modulo N. 
 * @param  device           Pointer to device structure
 * @param  n                N, 6 bits
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_set_modulo_n(adi_lark_device_t *device, uint8_t n);

/**
 * @brief  Load active parameter bank. 
 * @param  device           Pointer to device structure
 * @param  inst_index       Instruction index, 0 ~ 95
 * @param  param            Parameter memory 0 ~ 4
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_safeload(adi_lark_device_t *device, uint8_t inst_index, uint32_t param[5]);

/**
 * @brief  Set FDSP mode, for lark only. 
 * @param  device           Pointer to device structure
 * @param  mode             @see adi_lark_fdsp_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_set_mode(adi_lark_device_t *device, adi_lark_fdsp_mode_e mode);

/**
 * @brief  Get FDSP mode, for lark only. 
 * @param  device           Pointer to device structure
 * @param  mode             0 - Normal mode, 1 - Turble mode
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_get_mode(adi_lark_device_t *device, uint8_t *mode);

/**
 * @brief  Get FDSP current lambda value, for lark only. 
 * @param  device           Pointer to device structure
 * @param  lambda           Current Lambda value. If lambda < stop_point - 1, it is still in parameter bank swtich 
 *                          stop_point @see adi_lark_fdsp_config_bank_switch_settings()
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_fdsp_get_cur_lambda(adi_lark_device_t *device, uint8_t *lambda);

/**
 * @brief  Set EQ input data source route 
 * @param  device           Pointer to device structure
 * @param  route            @see adi_lark_eq_route_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_eq_select_route(adi_lark_device_t *device, adi_lark_eq_route_e route);

/**
 * @brief  Enable/Disable EQ
 * @param  device           Pointer to device structure
 * @param  enable           true to run EQ, false to stop EQ
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_eq_enable_run(adi_lark_device_t *device, bool enable);

/**
 * @brief  Clear param ram
 * @param  device           Pointer to device structure
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_eq_clear_param_ram(adi_lark_device_t *device);

/**
 * @brief  Get clear status when clearing param ram
 * @param  device           Pointer to device structure
 * @param  clear_done       1 to mean clearing is done
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_eq_get_param_ram_clear_status(adi_lark_device_t *device, uint8_t *clear_done);

/**
 * @brief  Select param ram
 * @param  device           Pointer to device structure
 * @param  bank_id          Param bank id (0 or 1)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_eq_sel_param_ram(adi_lark_device_t *device, uint8_t bank_id);

/**
 * @brief  Power on/off ADC. 
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  enable           true - Power on, false - Power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_enable_power_on(adi_lark_device_t *device, uint8_t adc_channel, bool enable);

/**
 * @brief  Get ADC Power on/off status. 
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  power_state      to store the power state which is read back
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_get_power_state(adi_lark_device_t *device, uint8_t adc_channel, uint32_t *power_state);

/**
 * @brief  Set ADC sample rate 
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  rate             ADC sample rate, @see adi_lark_adc_sample_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_sample_rate(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_sample_rate_e rate);

/**
 * @brief  Get ADC sample rate 
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  rate             to store ADC sample rate which is read back, @see adi_lark_adc_sample_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_get_sample_rate(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_sample_rate_e *rate);

/**
 * @brief  Set ADC Filter
 * @param  device                Pointer to device structure
 * @param  adc_channel           ADC channel, 0 ~ 2
 * @param  filter                High pass filter cut off frequence, @see adi_lark_adc_hpf_e
 * @param  higher_order_enable   true - higher order, false - lower order
 * @param  high_freq_comp_enable true - High frequency response compensated, false - High frequency response not compensated
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_filter(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_hpf_e filter, bool higher_order_enable, bool high_freq_comp_enable);

/**
 * @brief  Set ADC volume control
 * @param  device           Pointer to device structure
 * @param  hard_vol_change  true - Hard/immediate volume change, false - Ramp volume change
 * @param  use_chan0_vol    true - All ADC channels use Channel 0 volume value 
 *                          false - Each ADC channel uses its respective volume value
 * @param  change_at_zc     true - Volume change only occurs at zero crossing
 *                          false - Volume change occurs at any time
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool use_chan0_vol, bool change_at_zc);

/**
 * @brief  Set ADC ibias 
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  ibias            ADC ibias, @see adi_lark_adc_ibias_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_ibias(adi_lark_device_t *device, uint8_t adc_channel, adi_lark_adc_ibias_e ibias);

/**
 * @brief  Set ADC precharge time 
 * @param  device           Pointer to device structure
 * @param  time             Precharge time, @see adi_lark_adc_precharge_time_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_input_precharge_time(adi_lark_device_t *device, adi_lark_adc_precharge_time_e time);

/**
 * @brief  Set ADC voice wakeup mode.
 * @param  device               Pointer to device structure
 * @param  voicewakeup_enable   true: enable voice wakeup; false: disable voice wakeup
 *                              The ADCx can work in voice wake up mode to further reduce power consumption.
 *                              During the voice wake up mode, the ADCx sample rate is 8kHz or 16kHz and
 *                              the ADCx data width is 16 bits (16 MSBs of the ADC output).
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_voice_wakeup_mode(adi_lark_device_t *device, bool voicewakeup_enable);

/**
 * @brief  Set adc input mode.
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  single_enable    ADC input mode, true - single input, false - diff input
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_input_mode(adi_lark_device_t *device, uint8_t adc_channel, bool single_enable);

/**
 * @brief  Enable/disable ADC mute.
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  enable           true - mute, false - unmute
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_enable_mute(adi_lark_device_t *device, uint8_t adc_channel, bool enable);

/**
 * @brief  Set ADC volume.
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  volume           Volume, output dB = 24 - 0.375 * volume, if volume is 0xff, mute ADC
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_volume(adi_lark_device_t *device, uint8_t adc_channel, uint8_t volume);

/**
 * @brief  Set ADC volume.
 * @param  device           Pointer to device structure
 * @param  adc_channel      ADC channel, 0 ~ 2
 * @param  vol_db           Volume, unit: dB
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_volume_db(adi_lark_device_t *device, uint8_t adc_channel, float vol_db);

/**
 * @brief  Enable dither and set dither offset size.
 * @param  device           Pointer to device structure
 * @param  dither           Dither level,  @see adi_lark_adc_dither_level_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_adc_set_dither_level(adi_lark_device_t *device, adi_lark_adc_dither_level_e dither);

/**
 * @brief  Power on/off DAC. 
 * @param  device           Pointer to device structure
 * @param  enable           true - Power on, false - Power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_enable_power_on(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set DAC input sample rate. 
 * @param  device           Pointer to device structure
 * @param  rate             @see adi_lark_dac_sample_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_set_sample_rate(adi_lark_device_t *device, adi_lark_dac_sample_rate_e rate);

/**
 * @brief  Enable/disable DAC high frequency response compensated.
 * @param  device                   Pointer to device structure
 * @param  hpf_enable               true - ADC High Pass Filter On, false - ADC High Pass Filter Off
 * @param  high_freq_comp_enable    true - High frequency response compensated, false - High frequency response not compensated
 * @param  interp_enable            true -  More interpolation filtering : Higher delay, false - Less interpolation filtering : Lower delay
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_set_filter(adi_lark_device_t *device, bool hpf_enable, bool high_freq_comp_enable, bool interp_enable);
/**
 * @brief  Set DAC input ibias. 
 * @param  device           Pointer to device structure
 * @param  ibias            DAC input ibias, @see adi_lark_dac_ibias_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_set_ibias(adi_lark_device_t *device, adi_lark_dac_ibias_e ibias);

/**
 * @brief  Enable/disable low power mode. 
 * @param  device           Pointer to device structure
 * @param  enable           true -  enable low power mode, output sample rate 3.072MHz
 *                          false - disable low power mode, output sample rate 6.144MHz
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_enable_low_power_mode(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set DAC volume change mode.
 * @param  device           Pointer to device structure
 * @param  hard_vol_change  true - Hard/immediate volume change
 *                          false - Ramp volume change
 * @param  change_at_zc     true - Volume change only occurs at zero crossing
 *                          false - Volume change occurs at any time
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool change_at_zc);

/**
 * @brief  Enable/disable DAC mute.
 * @param  device           Pointer to device structure
 * @param  enable           true - mute, false - unmute
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_enable_mute(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set DAC volume.
 * @param  device           Pointer to device structure
 * @param  volume           Volume, output dB = 24 - 0.375 * volume, if volume is 0xff, mute DAC
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_set_volume(adi_lark_device_t *device, uint8_t volume);

/**
 * @brief  Set DAC volume.
 * @param  device           Pointer to device structure
 * @param  vol_db           Volume, unit: dB
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_set_volume_db(adi_lark_device_t *device, float vol_db);

/**
 * @brief  Set DAC input route.
 * @param  device           Pointer to device structure
 * @param  route            @see adi_lark_dac_input_route_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dac_select_input_route(adi_lark_device_t *device, adi_lark_dac_input_route_e route);
 
/**
 * @brief  Power on/off PGA.
 * @param  device           Pointer to device structure
 * @param  pga_channel      PGA channel, 0 ~ 2
 * @param  enable           true - power on, false - power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pga_enable_power_on(adi_lark_device_t *device, uint8_t pga_channel, bool enable);

/**
 * @brief  Set PGA course gain.
 * @param  device           Pointer to device structure
 * @param  pga_channel      PGA channel, 0 ~ 2
 * @param  gain             Gain, output in dB is 0.375 * gain
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pga_set_gain(adi_lark_device_t *device, uint8_t pga_channel, uint8_t gain);

/**
 * @brief  Set auto slew rate
 * @param  device           Pointer to device structure
 * @param  rate             @see adi_lark_pga_auto_slew_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pga_set_auto_slew_rate(adi_lark_device_t *device, adi_lark_pga_auto_slew_rate_e rate);

/**
 * @brief  Enable/disable gain link.
 * @param  device           Pointer to device structure
 * @param  enable           true - All PGA channels use Channel 0 gain value
 *                          false - Each PGA channel uses its respective gain value
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pga_enable_gain_link(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set input resistance.
 * @param  device                Pointer to device structure
 * @param  pga_channel           PGA channel index
 * @param  low_resistance_enable true - PGA Channel has low input resistance
 *                               false - PGA Channel has high input resistance
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pga_set_input_resistance(adi_lark_device_t *device, uint8_t pga_channel, bool low_resistance_enable);

/**
 * @brief  Set PGA operation mode
 * @param  device           Pointer to device structure
 * @param  pga_channel      PGA channel index
 * @param  enhanced_enable  true - PGA Channel is in enhanced operation mode
 *                          false - PGA Channel is in normal  performance mode.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pga_set_power_mode(adi_lark_device_t *device, uint8_t pga_channel, bool enhanced_enable);

/**
 * @brief  Power on/off DMIC channel, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel index, 0 ~ 7
 * @param  enable           true to power on, false to power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_channel_power_on(adi_lark_device_t *device, uint8_t channel, bool enable);

/**
 * @brief  Power on/off DMIC channels, for lark only.
 * @param  device           Pointer to device structure
 * @param  chnl_bits_set    If the bit is set, the channel will be affected; if the bit is clear, the channel will not be affected 
 * @param  enable           true to power on, false to power off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_multi_chnls_power_on(adi_lark_device_t *device, uint8_t chnl_bits_set, bool enable);

/**
 * @brief  Power on/off DMIC clock, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Digital Mic Clock Powered On 
 *                          false - Digital Mic Clock Powered Off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_clock_power_on(adi_lark_device_t *device, bool enable);

/**
 * @brief  Power on/off DMIC clock1, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Digital Mic Clock1 Powered On 
 *                          false - Digital Mic Clock1 Powered Off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_clock1_power_on(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set DMIC output sample rate, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel, 0 ~ 7
 * @param  out_sample_rate  output sample rate, @see adi_lark_dmic_out_sample_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_set_channel_out_sample_rate(adi_lark_device_t *device, uint8_t channel, adi_lark_dmic_out_sample_rate_e out_sample_rate);

/**
 * @brief  Select DMIC decimation order of cic4/5 compensation filter, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel, 0 ~ 7
 * @param  dec_order        @see adi_lark_dmic_dec_order_e
 * @param  hpf_cutoff_freq  @see adi_lark_dmic_hpf_cutoff_freq_e
 * @param  high_freq_comp_enable true - High frequency response compensated (higher delay), false - High frequency response not compensated (lower delay)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_set_channel_filter(adi_lark_device_t *device, uint8_t channel, adi_lark_dmic_dec_order_e dec_order, adi_lark_dmic_hpf_cutoff_freq_e hpf_cutoff_freq, bool high_freq_comp_enable);

/**
 * @brief  Set DMIC volume control, for lark only.
 * @param  device           Pointer to device structure
 * @param  hard_vol_change  true - Hard/Immediate Volume Change, false - Soft Volume Ramping
 * @param  use_chan0_vol    true - All DMIC channels use Channel 0 volume value, false - Each DMIC channel uses its respective volume value
 * @param  change_at_zc     true - Volume change only occurs at zero crossing, false - Volume change occurs at any time
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool use_chan0_vol, bool change_at_zc);

/**
 * @brief  Set DMIC volume, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel, 0 ~ 7
 * @param  volume           volume, output dB = 24 - 0.375 * volume, if volume is 0xff, mute DMIC
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_set_channel_volume(adi_lark_device_t *device, uint8_t channel, int32_t volume);

/**
 * @brief  Set DMIC volume, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel, 0 ~ 7
 * @param  vol_db           Volume, unit: dB
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_set_channel_volume_db(adi_lark_device_t *device, uint8_t channel, float vol_db);

/**
 * @brief  Enable/disable DMIC mute, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel, 0 ~ 7
 * @param  enable           true - mute, false - unmute
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_channel_mute(adi_lark_device_t *device, uint8_t channel, bool enable);

/**
 * @brief  Enable/disable dmic1 data from MP24, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - enable, false - disable
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_dmic1_from_mp24(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable dmic3 data from MP25, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - enable, false - disable
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_enable_dmic3_from_mp25(adi_lark_device_t *device, bool enable);

/**
 * @brief  Select dmic clk, for lark only.
 * @param  device           Pointer to device structure
 * @param  channel          DMIC channel, 0 ~ 7
 * @param  src              0 - dmic clk, 1 - dmic clk1
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_sel_clk_source(adi_lark_device_t *device, uint8_t channel, uint8_t src);

/**
 * @brief  Selct dmic clk1 source, for lark only.
 * @param  device           Pointer to device structure
 * @param  pin             @see adi_lark_dmic_clk1_pin_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_dmic_sel_clk1_pin_src(adi_lark_device_t *device, adi_lark_dmic_clk1_pin_e pin);

/**
 * @brief  Enable/disable PDM output, for lark only.
 * @param  device           Pointer to device structure
 * @param  pdm_channel      PDM channel index
 * @param  enable           true - PDM output channel powered on
 *                          false - PDM output channel powered off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_enable_power_on(adi_lark_device_t *device, uint8_t pdm_channel, bool enable);

/**
 * @brief  Set PDM input sample rate, for lark only. 
 * @param  device           Pointer to device structure
 * @param  rate             @see adi_lark_pdm_sample_rate_e
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_set_sample_rate(adi_lark_device_t *device, adi_lark_pdm_sample_rate_e rate);

/**
 * @brief  Set output rate, for lark only. 
 * @param  device           Pointer to device structure
 * @param  rate             @see adi_lark_pdm_output_rate_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_set_output_rate(adi_lark_device_t *device, adi_lark_pdm_output_rate_e rate);

/**
 * @brief  Enable/disable high pass filter, for lark only.
 * @param  device           Pointer to device structure
 * @param  pdm_channel      PDM channel, 0 ~ 1
 * @param  enable           true - ADC High Pass Filter On, false - ADC High Pass Filter Off
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_enable_high_pass_filter(adi_lark_device_t *device, uint8_t pdm_channel, bool enable);

/**
 * @brief  Enable/disable PDM high frequency response compensated, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - High frequency response compensated
 *                          false - High frequency response not compensated
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_enable_high_freq_response_compensation(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable PDM more interpolation filtering, for lark only. 
 * @param  device           Pointer to device structure
 * @param  enable           true -  More interpolation filtering : Higher delay
 *                          false - Less interpolation filtering : Lower delay
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_enable_more_interp_filter(adi_lark_device_t *device, bool enable);

/**
 * @brief  All PDMs use the same volume, for lark only. 
 * @param  device           Pointer to device structure
 * @param  hard_vol_change  true - Hard/immediate volume change, false - Ramp volume change
 * @param  use_chan0_vol    true -  All PDM channels use Channel 0 volume value
 *                          false - Each PDM channel uses its respective volume value
 * @param  change_at_zc     true - Volume change only occurs at zero crossing
 *                          false - Volume change occurs at any time
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_set_volume_control(adi_lark_device_t *device, bool hard_vol_change, bool use_chan0_vol, bool change_at_zc);

/**
 * @brief  Enable/disable PDM mute, for lark only.
 * @param  device           Pointer to device structure
 * @param  pdm_channel      PDM channel, 0 ~ 1
 * @param  enable           true - mute, false - unmute
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_enable_mute(adi_lark_device_t *device, uint8_t pdm_channel, bool enable);

/**
 * @brief  Set PDM volume, for lark only.
 * @param  device           Pointer to device structure
 * @param  pdm_channel      PDM channel, 0 ~ 1
 * @param  volume           Volume, output dB = 24 - 0.375 * volume, if volume is 0xff, mute DAC
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_set_volume(adi_lark_device_t *device, uint8_t pdm_channel, uint8_t volume);

/**
 * @brief  Set PDM volume, for lark only.
 * @param  device           Pointer to device structure
 * @param  pdm_channel      PDM channel, 0 ~ 1
 * @param  vol_db           Volume, unit: dB
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_set_volume_db(adi_lark_device_t *device, uint8_t pdm_channel, float vol_db);

/**
 * @brief  Set PDM volume, for lark only.
 * @param  device           Pointer to device structure
 * @param  pdm_channel      PDM channel, 0 ~ 1
 * @param  route            @see adi_lark_pdm_input_route_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_pdm_select_input_route(adi_lark_device_t *device, uint8_t pdm_channel, adi_lark_pdm_input_route_e route);

/**
 * @brief  Set GPTimer mode, for lark only.
 * @param  device           Pointer to device structure
 * @param  run_mode         Freerun/Periodic, @see adi_lark_gpt_count_mode_e
 * @param  count_mode       Count up/down, @see adi_lark_gpt_count_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_set_mode(adi_lark_device_t *device, adi_lark_gpt_run_mode_e run_mode, adi_lark_gpt_count_mode_e count_mode);

/**
 * @brief  Enable GPT, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true to enable, false to disable
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_enable(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set timer load value, for lark only.
 * @param  device           Pointer to device structure
 * @param  value            Load value, the Up/Down counter is periodically loaded with this value
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_set_load_value(adi_lark_device_t *device, uint16_t value);

/**
 * @brief  Get  current timer value, for lark only.
 * @param  device           Pointer to device structure
 * @param  value            The current Up/Down counter value
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_get_current_value(adi_lark_device_t *device, uint16_t *value);

/**
 * @brief  Set GPTimer clock source, for lark only.
 * @param  device           Pointer to device structure
 * @param  source           Clock source, @see adi_lark_gpt_clk_source_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_select_clk_source(adi_lark_device_t *device, adi_lark_gpt_clk_source_e source);

/**
 * @brief  Set the prescaler division, for lark only.
 * @param  device           Pointer to device structure
 * @param  div              the prescaler division, @see adi_lark_gpt_pre_scaler_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_set_pre_scaler(adi_lark_device_t *device, adi_lark_gpt_pre_scaler_e div);

/**
 * @brief  Clear timeout interrupt, for lark only.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_clr_timeout_int(adi_lark_device_t *device);

/**
 * @brief  Check if timeout event is occured, for lark only.
 * @param  device           Pointer to device structure
 * @param  timeout          false - No timeout event has occurred
 *                          true  - A timeout event has occurred
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_get_timeout_status(adi_lark_device_t *device, bool *timeout);

/**
 * @brief  Check if timer is busy, for lark only.
 * @param  device           Pointer to device structure
 * @param  busy             false - Not busy, timer ready to receive commands to GPT_CTRL
 *                          true  - Busy, timer not ready to receive commands to GPT_CTRL
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_get_busy_status(adi_lark_device_t *device, bool *busy);

/**
 * @brief  Check if timeout interrupt is cleardone, for lark only.
 * @param  device           Pointer to device structure
 * @param  clrdone          false - The interrupt is cleared
 *                          true  - The interrupt is not cleared
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_gpt_get_clrdone_status(adi_lark_device_t *device, bool *clrdone);

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
/**
 * @brief  Test function is to test gpt in device module, for running sdk on lark only
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_gpt_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq);
#endif

/**
 * @brief  Set headphone output mode.
 * @param  device           Pointer to device structure
 * @param  mode             Headphone or line-out, @see adi_lark_hpamp_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_set_output_mode(adi_lark_device_t *device, adi_lark_hpamp_mode_e mode);

/**
 * @brief  Enable/disable low voltage mode.
 * @param  device           Pointer to device structure
 * @param  enable           true - enable, false - disable
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_enable_lvm_mode(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable lgenerate HPVDDL/2 VCM.
 * @param  device           Pointer to device structure
 * @param  enable           true - enable generate HPVDDL/2 VCM, false - disable generate HPVDDL/2 VCM
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_enable_lvm_mode_cm(adi_lark_device_t *device, bool enable);

/**
 * @brief  Set volatge swicth mode.
 * @param  device           Pointer to device structure
 * @param  mode             Voltage swicth mode, @see adi_lark_hpamp_voltage_switch_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_set_voltage_switch_mode(adi_lark_device_t *device, adi_lark_hpamp_voltage_switch_mode_e mode);

/**
 * @brief  Select voltage mode.
 * @param  device           Pointer to device structure
 * @param  mode             Normal volatge mode or low voltage mode, @see adi_lark_hpamp_voltage_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_select_voltage_mode(adi_lark_device_t *device, adi_lark_hpamp_voltage_mode_e mode);

/**
 * @brief  Switch to low voltage HP mode, auto counting mode delay timer.
 * @param  device           Pointer to device structure
 * @param  delay            delay time is pow(2 , delay) * 10us
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_set_lvmmode_delay(adi_lark_device_t *device, uint8_t delay);

/**
 * @brief  CM voltage ready wait time.
 * @param  device           Pointer to device structure
 * @param  delay            0 - 20us
 *                          1 - 40us
 *                          2 - 80us
 *                          3 - 100us
 *
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_set_lvmmode_cm_delay(adi_lark_device_t *device, uint8_t delay);

/**
 * @brief  Enable/disable headphone LDO.
 * @param  device           Pointer to device structure
 * @param  enable           true - enbale
 *                          false - disable
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_enable_ldo(adi_lark_device_t *device, bool enable);

/**
 * @brief  Bypass LDO or not bypass.
 * @param  device           Pointer to device structure
 * @param  bypass           true - bypass
 *                          false - no bypass
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_set_ldo_bypass(adi_lark_device_t *device, bool bypass);

/**
 * @brief  Select headphone amp LDO vout.
 * @param  device           Pointer to device structure
 * @param  vout             Vout, @see adi_lark_hpamp_ldo_vout_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_hpamp_set_ldo_vout(adi_lark_device_t *device, adi_lark_hpamp_ldo_vout_e vout);

/**
 * @brief  Get the current value of WDT, for lark only.
 * @param  device           Pointer to device structure
 * @param  cur_val          Pointer to current value
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
 int32_t adi_lark_wdt_get_current_value(adi_lark_device_t *device, uint16_t *cur_val);
 
/**
 * @brief  Enable WDT, for lark only.
 * @param  device           Pointer to device structure
 * @param  mode             Mode, @see adi_lark_wdt_mode_e
 * @param  ld_val           Load value, a timeout value for watchdog to decrement
 * @param  pre_scaler_fac   Prescaler factor, @see adi_lark_wdt_mode_e
 * @param  wdt_en           true - Timeout creates an interrupt
 *                          false - Timeout creates a reset
 * 
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_wdt_pre_scaler_factor_e
 */
int32_t adi_lark_wdt_enable(adi_lark_device_t *device, adi_lark_wdt_mode_e mode, uint32_t ld_val, adi_lark_wdt_pre_scaler_factor_e pre_scaler_fac, bool wdt_en);

/**
 * @brief  Refresh the timer, for lark only.
 * 
 * @param  device           Pointer to device structure
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_wdt_refresh(adi_lark_device_t *device);

/**
 * @brief  Get the status of interrupt, for lark only.
 * 
 * @param  device           Pointer to device structure
 * @param  irq              Pointer to status of interrupt
 *   
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_wdt_get_irq_status(adi_lark_device_t *device, uint8_t *irq);

/**
 * @brief  Refresh the timer, for lark only.
 *
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 * @param  clk_freq         QSPI clock frequency
 * @param  clk_pha          serial clock phase mode
 * @param  clk_pol          serial clock polarity mode
 *
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_config_clk(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, uint32_t clk_freq, uint8_t clk_pha, uint8_t clk_pol);

/**
 * @brief  Enable/disable QSPI, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable QSPI
 *                          false - Disable QSPI
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable least singnificant bit first, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - LSB send/received first
 *                          false - MSB send/received first
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_lsb_first(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable fast mode, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable fast mode
 *                          false - Disable fast mode
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_fast_mode(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable hardware automated access of memory mapped SPI memory, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable hardware automated access of memory mapped SPI memory.
 *                          false - Disable hardware automated access of memory mapped SPI memory.
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_mem_mapped(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable receive channel operation, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable receive channel operation
 *                          false - Disable receive channel operation
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_receive(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable receive word counter, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable receive word counter
 *                          false - Disable receive word counter
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_receive_counter(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable transmit channel operation, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable transmit channel operation
 *                          false - Disable transmit channel operation
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_transmit(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable transmit word counter, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Enable transmit word counter
 *                          false - Disable transmit word counter
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_transmit_counter(adi_lark_device_t *device, bool enable);

/**
 * @brief  Enable/disable hardware slave select control, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Hardware slave select control
 *                          false - Software slave select control
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_hw_slave_select(adi_lark_device_t *device, bool enable);

/**
 * @brief  Ste slave select polarity between transfers, for lark only.
 * @param  device           Pointer to device structure
 * @param  level            1 - Assert slave select (low)
 *                          0 - De-assert slave select (high)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_set_hw_slave_select_level(adi_lark_device_t *device, uint8_t level);

/**
 * @brief  Enable/disable the related SELn pin for input, for lark only.
 * @param  device           Pointer to device structure
 * @param  index            Sel index, 0~6
 * @param  enable           true - Enable the related SELn pin for input
 *                          false - Disable the related SELn pin for input
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_slave_select_pin(adi_lark_device_t *device, uint8_t index, bool enable);

/**
 * @brief  Indicates the value on the related SELn pin, for lark only.
 * @param  device           Pointer to device structure
 * @param  index            Sel index, 0~6
 * @param  level            0 - low
 *                          1 - high
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_set_slave_select_pin_level(adi_lark_device_t *device, uint8_t index, uint8_t level);

/**
 * @brief  Set word transfer size, for lark only.
 * @param  device           Pointer to device structure
 * @param  size             @see adi_lark_qspi_transfer_size_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_set_transfer_size(adi_lark_device_t *device, adi_lark_qspi_transfer_size_e size);

/**
 * @brief  Select SPI operation  in dual I/O mode or quad I/O mode, for lark only.
 * @param  device           Pointer to device structure
 * @param  mode             0 - standard, 1 - dual, 2 - quad, @see adi_lark_qspi_transfer_size_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_set_multi_io_mode(adi_lark_device_t *device, adi_lark_qspi_multi_io_mode_e mode);

/**
 * @brief  Selects the starting pin and the bit placement on pins for dual I/O mode or quad I/O mode, for lark only.
 * @param  device           Pointer to device structure
 * @param  enable           true - Start on MOSI
 *                          false - Start on MISO (DIOM) or start on SPIQ3 (QSPI)
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_enable_start_mosi(adi_lark_device_t *device, bool enable);

/**
 * @brief  Get QSPI status, for lark only.
 * @param  device           Pointer to device structure
 * @param  status           Refer to QSPI status bitfield
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_get_status(adi_lark_device_t *device, uint32_t *status);

/**
 * @brief  Send multiple bytes, for lark only.
 * @param  device           Pointer to device structure
 * @param  tx_buf           Tx buff pointer
 * @param  count            Size of tx data 
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_tx_data(adi_lark_device_t *device, uint8_t *tx_buf, uint32_t count);

/**
 * @brief  Receive multiple bytes, for lark only.
 * @param  device           Pointer to device structure
 * @param  rx_buf           Rx buff pointer
 * @param  count            Size of rx data 
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_qspi_rx_data(adi_lark_device_t *device, uint8_t *rx_buf, uint32_t count);

/**
 * @brief  QSPI test function, for lark only.
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
#if LARK_SDK_EN_TEST_CODE
int32_t adi_lark_qspi_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq);
#endif

/**
 * @brief  Config Uart all settings, call after adi_lark_clk_set_tdsp_bus_rate(), for lark only.
 * @param  device           Pointer to device structure
 * @param  mclk_freq        mclk_freq, @see adi_lark_mclk_freq_e
 * @param  baudrate         baudrate, unit: Hz
 * @param  len              len, @see adi_lark_uart_word_len_e
 * @param  stop_bit         0 - 1 bit, 1 - multi-bit
 * @param  parity           parity, @see adi_lark_uart_parity_type_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_uart_config(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq, uint32_t baudrate, adi_lark_uart_word_len_e len, 
    uint8_t stop_bit, adi_lark_uart_parity_type_e parity);

/**
 * @brief  Read a byte, blocked if there is no new receive data, for lark only.
 * @param  device           Pointer to device structure
 * @param  data             Data
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_uart_read_byte(adi_lark_device_t *device, uint8_t *data);

/**
 * @brief  Send a byte, for lark only.
 * @param  device           Pointer to device structure
 * @param  data             Data to send
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_uart_send_byte(adi_lark_device_t *device, uint8_t data);

/**
 * @brief  Receive a string, stop read when getting a "\n" or "\r", for lark only.
 * @param  device           Pointer to device structure
 * @param  string           String to read
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_uart_rx_string(adi_lark_device_t *device, char *string);

/**
 * @brief  Send a string, for lark only.
 * @param  device           Pointer to device structure
 * @param  string           String to send
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_uart_tx_string(adi_lark_device_t *device, char const *string);

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
/**
 * @brief  Test function is to test uart in device module, for running sdk on lark only
 * @param  device           Pointer to device structure
 * @param  mclk_freq        @see adi_lark_mclk_freq_e
 * @return API_LARK_ERROR_OK for success, @see adi_lark_error_e
 */
int32_t adi_lark_uart_test(adi_lark_device_t *device, adi_lark_mclk_freq_e mclk_freq);
#endif

/**
 * @brief  Set MP (multi-purpose) pin mode, for lark only
 * @param  device           Pointer to device structure
 * @param  pin_num          pin number id, 0~24 for lark
 * @param  mode             @see adi_lark_mp_pin_mode_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_mp_set_pin_mode(adi_lark_device_t *device, uint8_t pin_num, adi_lark_mp_pin_mode_e mode);

/**
 * @brief  Set MP (multi-purpose) pin output level, for lark only
 * @param  device           Pointer to device structure
 * @param  pin_num          pin number id, 0~24 for lark
 * @param  level            0 - low level, 1 - high level
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_mp_set_output_level(adi_lark_device_t *device, uint8_t pin_num, uint8_t level);

/**
 * @brief  Get MP (multi-purpose) pin input level, for lark only
 * @param  device           Pointer to device structure
 * @param  pin_num          pin number id, 0~24 for lark
 * @param  level            0 - low level, 1 - high level
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_mp_get_input_level(adi_lark_device_t *device, uint8_t pin_num, uint8_t *level);

/**
 * @brief  Set MP (multi-purpose) pin input debounce, for lark only
 * @param  device           Pointer to device structure
 * @param  debounce         @see adi_lark_mp_input_debounce_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_mp_set_gpi_debounce(adi_lark_device_t *device, adi_lark_mp_input_debounce_e debounce);

/**
 * @brief  Set MP (multi-purpose) pin irq input debounce, for lark only
 * @param  device           Pointer to device structure
 * @param  debounce         @see adi_lark_mp_input_debounce_e
 *  
 * @return API_LARK_ERROR_OK for success, error @see adi_lark_error_e
 */
int32_t adi_lark_mp_set_irq_input_debounce(adi_lark_device_t *device, adi_lark_mp_input_debounce_e debounce);

#ifdef __cplusplus
}
#endif

#endif  /*__ADI_LARK_H__*/
/*! @} */
