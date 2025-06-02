/*!
 * @brief     Data Sync APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#define API_LARK_DS_DMA_CHANS           16                          /*!< ds dma channels */
#define API_LARK_DS_OUT_CHANS           16                          /*!< ds output channels */
#define API_LARK_DS_FIFOS               12                          /*!< ds fifo number */
#define API_LARK_DS_INPUT_RESERVED      13, 14 ,15                  /*!< reserved ADP2SOC channel numbers among 80 */

/*============= C O D E ====================*/
bool adi_lark_ds_chnl_is_reserved(uint8_t chnl);

int32_t adi_lark_ds_enable_chnl_dmareq(adi_lark_device_t *device, uint32_t ds_dma_chan, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(ds_dma_chan >= API_LARK_DS_DMA_CHANS);

    err = adi_lark_hal_bf_write(device, REG_DS_DMA_CTRL_ADDR, 0x0100 | ds_dma_chan, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_multi_chnls_dmareq(adi_lark_device_t *device, uint32_t ds_dma_chnls, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t reg_val;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_read(device, REG_DS_DMA_CTRL_ADDR, &reg_val);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DS_DMA_CTRL_ADDR, enable_flag ? (reg_val | ds_dma_chnls) : (reg_val & (~ds_dma_chnls)));
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_set_rdy_rate(adi_lark_device_t *device, uint16_t rdy_div)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_RDY_RATE_DIV_INFO, rdy_div);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_set_lt_rdy_width(adi_lark_device_t *device, uint8_t lt_rdy_width)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_LT_RDY_WIDTH_INFO, lt_rdy_width);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_tie_ltif(adi_lark_device_t *device, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_LT_EN_INFO, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif 
  
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_multi_out_chnls_resync(adi_lark_device_t *device, uint16_t out_resync_chnls, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t reg_val;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_read(device, REG_DS_OUT_CTRL_ADDR, &reg_val);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DS_OUT_CTRL_ADDR, enable_flag ? (reg_val | out_resync_chnls) : (reg_val & (~out_resync_chnls)));
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_tdsp_out_chnl_resync(adi_lark_device_t *device, uint16_t tdsp_out_chnl, bool enable_flag, adi_lark_ds_rdy2out_chnl_e rdy_chnl)
{
    #ifdef LARK_SDK
    int32_t  err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(tdsp_out_chnl >= API_LARK_DS_OUT_CHANS);

    err = adi_lark_ds_enable_multi_out_chnls_resync(device, 1 << tdsp_out_chnl, enable_flag);
    LARK_ERROR_RETURN(err);    
    err = adi_lark_ds_select_rdy2out_chnl(device, tdsp_out_chnl, rdy_chnl);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_multi_chnls_int(adi_lark_device_t *device, adi_lark_ds_int_chnl_e chnls, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t reg_val;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_reg_read(device, REG_DS_INT_MASK_ADDR, &reg_val);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DS_INT_MASK_ADDR, enable_flag ? (reg_val & (~chnls)) : (reg_val | chnls));
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_chnl_int(adi_lark_device_t *device, adi_lark_ds_int_chnl_e int_chnl, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(int_chnl >= API_LARK_DS_INT_COUNT);

    err = adi_lark_hal_bf_write(device, BF_INT_MASK_ADC01_INFO + int_chnl, enable_flag ? 0 : 1);
    LARK_ERROR_RETURN(err);
    #endif 

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_read_int_status(adi_lark_device_t *device, uint32_t *int_stat)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    err = adi_lark_hal_reg_read(device, REG_DS_INT_STATUS_ADDR, int_stat);
    LARK_ERROR_RETURN(err);
    #endif 

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_clear_int_status(adi_lark_device_t *device, uint32_t int_chnls)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_write(device, REG_DS_INT_CLR_CFG_ADDR, int_chnls);
    LARK_ERROR_RETURN(err);
    #endif 
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_clear_chnl_int_status(adi_lark_device_t *device, adi_lark_ds_int_chnl_e int_chnl)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(int_chnl >= API_LARK_DS_INT_COUNT);

    err = adi_lark_hal_reg_write(device, REG_DS_INT_CLR_CFG_ADDR, 1 << int_chnl);     /* this register is W1C */
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_autoclear_chnl_int_status(adi_lark_device_t *device, adi_lark_ds_int_chnl_e int_chnl, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(int_chnl >= API_LARK_DS_INT_COUNT);

    err = adi_lark_hal_bf_write(device, BF_INT_ACLR_ADC01_INFO + int_chnl, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif 

    return API_LARK_ERROR_OK;
}

bool adi_lark_ds_chnl_is_reserved(uint8_t chnl_no)
{
    #ifdef LARK_SDK
    uint8_t rsvr_chnl[3] = {API_LARK_DS_INPUT_RESERVED};
    int32_t i;
    for(i = 0; i < 3; i++)
    {
        if(chnl_no == rsvr_chnl[i])
            return true;
    }
    #endif

    return false;
}

int32_t adi_lark_ds_select_rdy2dma_chnl(adi_lark_device_t *device, uint8_t dma_chnl, adi_lark_ds_rdy2dma_chnl_e rdy_chnl)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(rdy_chnl >= API_LARK_DS_INPUT_COUNT || dma_chnl > API_LARK_DS_DMA_CHANS || adi_lark_ds_chnl_is_reserved(rdy_chnl));
    
    /* It's not necessary to call _bf_write(), unless more cycles is expected */
    err = adi_lark_hal_reg_write(device, REG_DS_RDY2DMA_SEL0_ADDR + dma_chnl * 4, (uint32_t)rdy_chnl);
    LARK_ERROR_RETURN(err);
    #endif 

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_select_rdy2out_chnl(adi_lark_device_t *device, uint8_t out_chnl, adi_lark_ds_rdy2out_chnl_e rdy_chnl)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(rdy_chnl >= API_LARK_DS_RDY2OUT_COUNT || out_chnl >= API_LARK_DS_OUT_CHANS);
    
    /* It's not necessary to call _bf_write(), unless more cycles is expected */
    err = adi_lark_hal_reg_write(device, REG_DS_RDY2OUT_SEL0_ADDR + out_chnl * 4, (uint32_t)rdy_chnl);
    LARK_ERROR_RETURN(err);
    #endif 

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_select_fifo_chnl_src(adi_lark_device_t *device, uint8_t fifo_chnl, adi_lark_ds_fifo_src_chnl_e src_chnl)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(src_chnl >= API_LARK_DS_INPUT_COUNT || fifo_chnl >= API_LARK_DS_FIFOS || adi_lark_ds_chnl_is_reserved(src_chnl));
    
    err = adi_lark_hal_bf_write(device, fifo_chnl * 4 + BF_FIFO_CFG_SRC0_INFO, src_chnl);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_select_fifo_chnl_dst(adi_lark_device_t *device, uint8_t fifo_chnl, uint8_t dst_chnl)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dst_chnl >= API_LARK_DS_OUT_CHANS || fifo_chnl >= API_LARK_DS_FIFOS);
    
    err = adi_lark_hal_bf_write(device, fifo_chnl * 4 + BF_FIFO_CFG_DST0_INFO, dst_chnl);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_fifo_chnl_read(adi_lark_device_t *device, uint8_t fifo_chnl, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_chnl >= API_LARK_DS_FIFOS);
    
    err = adi_lark_hal_bf_write(device, fifo_chnl * 4 + BF_FIFO_CFG_REN0_INFO, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_multi_fifo_chnls_read(adi_lark_device_t *device, uint16_t chnl_bits_set, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    uint8_t fifo_chnl;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    for (fifo_chnl = 0; fifo_chnl < API_LARK_DS_FIFOS; fifo_chnl++)
    {
        if (((1 << fifo_chnl) & chnl_bits_set) > 0)
        {
            err = adi_lark_hal_bf_write(device, fifo_chnl * 4 + BF_FIFO_CFG_REN0_INFO, enable_flag ? 1 : 0);
            LARK_ERROR_RETURN(err);
        }
    }
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_fifo_chnl_write(adi_lark_device_t *device, uint8_t fifo_chnl, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_chnl >= API_LARK_DS_FIFOS);
    
    err = adi_lark_hal_bf_write(device, fifo_chnl * 4 + BF_FIFO_CFG_WEN0_INFO, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif 
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ds_enable_multi_fifo_chnls_write(adi_lark_device_t *device, uint16_t chnl_bits_set, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    uint8_t fifo_chnl;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    for (fifo_chnl = 0; fifo_chnl < API_LARK_DS_FIFOS; fifo_chnl++)
    {
        if (((1 << fifo_chnl) & chnl_bits_set) > 0)
        {
            err = adi_lark_hal_bf_write(device, fifo_chnl * 4 + BF_FIFO_CFG_WEN0_INFO, enable_flag ? 1 : 0);
            LARK_ERROR_RETURN(err);
        }
    }
    #endif

    return API_LARK_ERROR_OK;
}

/*! @} */
