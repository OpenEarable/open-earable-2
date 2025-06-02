/*!
 * @brief     DMA Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E C L A R E S ============*/
int32_t adi_lark_dma_get_int_stat(adi_lark_device_t *device, uint32_t *reg_data);

/*============= C O D E ====================*/
int32_t adi_lark_dma_init(adi_lark_device_t *device, uint32_t primary_ctrl_data_base)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t w1r_value = (uint32_t) ((1 << API_LARK_NUM_DMA_CHANNEL) - 1);
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_bf_write(device, BF_DMA_EN_INFO, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_PRI_BASE_PTR_ADDR, primary_ctrl_data_base);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, w1r_value);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_CLR_ADDR, w1r_value);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_PRI_ALT_CLR_ADDR, w1r_value);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_PRIORITY_CLR_ADDR, w1r_value);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_BUSERR_IDX_CLR_ADDR, w1r_value);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_INVDESC_IDX_CLR_ADDR, w1r_value);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_update_chnl_ctrl_data(adi_lark_device_t *device, uint8_t primary_alternate_flag, 
    adi_lark_dma_chno_e dma_channel, adi_lark_dma_chnl_ctrl_t *ctrl_data)
{
    #ifdef LARK_SDK
    int32_t err;
    adi_lark_dma_chnl_ctrl_t* ctrl_data_ptr;
    uint32_t reg_data;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(ctrl_data);
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL);
    
    err = adi_lark_dma_get_ctrl_base_reg(device, primary_alternate_flag, &reg_data);
    LARK_ERROR_RETURN(err);
    if(reg_data == 0)
        return API_LARK_ERROR_DMA_DATABASE_NOT_SET;
    ctrl_data_ptr = (adi_lark_dma_chnl_ctrl_t *)reg_data;
    ctrl_data_ptr += dma_channel;
    ctrl_data_ptr->dma_src_end  = ctrl_data->dma_src_end;
    ctrl_data_ptr->dma_dst_end  = ctrl_data->dma_dst_end;
    ctrl_data_ptr->dma_chnl_cfg = ctrl_data->dma_chnl_cfg;
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable(adi_lark_device_t *device, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    err = adi_lark_hal_bf_write(device, BF_DMA_EN_INFO, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_ctrl_base_reg(adi_lark_device_t *device, uint8_t primary_alternate_flag, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);
    if(primary_alternate_flag == 0)
        err = adi_lark_hal_reg_read(device, REG_DMA_PRI_BASE_PTR_ADDR, reg_data);
    else
        err = adi_lark_hal_reg_read(device, REG_DMA_ALT_BASE_PTR_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_set_ctrl_base_reg(adi_lark_device_t *device, uint32_t reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    err = adi_lark_hal_reg_write(device, REG_DMA_PRI_BASE_PTR_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl_alt_ctrl(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL);

    if(enable_flag)
        err = adi_lark_hal_reg_write(device, REG_DMA_PRI_ALT_SET_ADDR, 1 << dma_channel);
    else
        err = adi_lark_hal_reg_write(device, REG_DMA_PRI_ALT_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl_byteswap(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    if(enable_flag)
        err = adi_lark_hal_reg_write(device, REG_DMA_BYTE_SWAP_SET_ADDR, 1 << dma_channel);
    else
        err = adi_lark_hal_reg_write(device, REG_DMA_BYTE_SWAP_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl_dst_decrement(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    if(enable_flag)
        err = adi_lark_hal_reg_write(device, REG_DMA_DSTADR_DEC_SET_ADDR, 1 << dma_channel);
    else
        err = adi_lark_hal_reg_write(device, REG_DMA_DSTADR_DEC_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl_src_decrement(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    if(enable_flag)
        err = adi_lark_hal_reg_write(device, REG_DMA_SRCADR_DEC_SET_ADDR, 1 << dma_channel);
    else
        err = adi_lark_hal_reg_write(device, REG_DMA_SRCADR_DEC_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag, bool mdma_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    if(enable_flag)
    {
        err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_SET_ADDR, 1 << dma_channel);
        LARK_ERROR_RETURN(err);
        if(mdma_flag)
        {
            err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, 1 << dma_channel);
            LARK_ERROR_RETURN(err);
        }
        else
        {
            err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_CLR_ADDR, 1 << dma_channel);
            LARK_ERROR_RETURN(err);
        }
    }
    else
    {
        err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_CLR_ADDR, 1 << dma_channel);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, 1 << dma_channel);
        LARK_ERROR_RETURN(err);
    }
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_multi_chnls(adi_lark_device_t *device, uint32_t dma_channels, bool enable_flag, bool mdma_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t mask;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    mask = (uint32_t) ((1 << API_LARK_NUM_DMA_CHANNEL) - 1);
    dma_channels &= mask;
    if(enable_flag)
    {
        err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_SET_ADDR, dma_channels);
        LARK_ERROR_RETURN(err);
        if(mdma_flag)
        {
            err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, dma_channels);
            LARK_ERROR_RETURN(err);
        }
        else
        {
            err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_CLR_ADDR, dma_channels);
            LARK_ERROR_RETURN(err);
        }
    }
    else
    {
        err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_CLR_ADDR, dma_channels);
        LARK_ERROR_RETURN(err);
        err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, dma_channels);
        LARK_ERROR_RETURN(err);
    }
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_req_mask(adi_lark_device_t *device, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);
    err = adi_lark_hal_reg_read(device, REG_DMA_REQ_MASK_SET_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_chnl_en_stat(adi_lark_device_t *device, uint32_t *en_stat)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(en_stat);
    err = adi_lark_hal_reg_read(device, REG_DMA_CHNL_EN_SET_ADDR, en_stat);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_chnl_buserr_stat(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );
    err = adi_lark_hal_reg_write(device, REG_DMA_BUSERR_IDX_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_buserr_stats(adi_lark_device_t *device, uint32_t dma_channels)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_write(device, REG_DMA_BUSERR_IDX_CLR_ADDR, dma_channels);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_buserr_stat(adi_lark_device_t *device, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);
    err = adi_lark_hal_reg_read(device, REG_DMA_BUSERR_IDX_CLR_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_chnl_invalid_desc_stat(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );
    err = adi_lark_hal_reg_write(device, REG_DMA_INVDESC_IDX_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_invalid_desc_stats(adi_lark_device_t *device, uint32_t dma_channels)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_write(device, REG_DMA_INVDESC_IDX_CLR_ADDR, dma_channels);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_invalid_desc_stat(adi_lark_device_t *device, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);
    err = adi_lark_hal_reg_read(device, REG_DMA_INVDESC_IDX_CLR_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl_high_priority(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    if(enable_flag)
        err = adi_lark_hal_reg_write(device, REG_DMA_PRIORITY_SET_ADDR, 1 << dma_channel);
    else
        err = adi_lark_hal_reg_write(device, REG_DMA_PRIORITY_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_start_chnl_softdma(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel)
{
    #ifdef LARK_SDK
    uint32_t reg_info;
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    reg_info = 1 << 8;
    reg_info |= dma_channel;
    err = adi_lark_hal_bf_write(device, REG_DMA_SW_REQ_ADDR, reg_info, 1);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_stat(adi_lark_device_t *device, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);

    err = adi_lark_hal_reg_read(device, REG_DMA_STATUS_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_avail_channels(adi_lark_device_t *device, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);

    err = adi_lark_hal_bf_read(device, BF_DMA_STATUS_CHNL_NUM_INFO, reg_data);
    LARK_ERROR_RETURN(err);
    *reg_data += 1;
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_chnl_dma_done_int(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );
    
    if(enable_flag)
        err = adi_lark_hal_reg_write(device, REG_DMA_INT_EN_SET_ADDR, 1 << dma_channel);
    else
        err = adi_lark_hal_reg_write(device, REG_DMA_INT_EN_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_enable_dma_err_int(adi_lark_device_t *device, uint8_t dma_err_type)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t err_type_mask;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_err_type > 3);
    err_type_mask = ~dma_err_type & 3;
    err_type_mask <<= 0x1E;
    err = adi_lark_hal_reg_write(device, REG_DMA_INT_EN_CLR_ADDR, err_type_mask);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_write(device, REG_DMA_INT_EN_SET_ADDR, dma_err_type << 0x1E);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_chnl_int_stat(adi_lark_device_t *device, adi_lark_dma_chno_e dma_channel)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(dma_channel >= API_LARK_NUM_DMA_CHANNEL );

    err = adi_lark_hal_reg_write(device, REG_DMA_INT_STATUS_CLR_ADDR, 1 << dma_channel);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_err_int_stat(adi_lark_device_t *device, uint8_t dma_err_type)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN((dma_err_type > 3));

    err = adi_lark_hal_reg_write(device, REG_DMA_INT_STATUS_CLR_ADDR, dma_err_type << 0x1E);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_clr_int_stat(adi_lark_device_t *device, uint32_t int_stat_set)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_write(device, REG_DMA_INT_STATUS_CLR_ADDR, int_stat_set);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_get_int_stat(adi_lark_device_t *device, uint32_t *reg_data)
{
    #ifdef LARK_SDK
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(reg_data);
    
    err = adi_lark_hal_reg_read(device, REG_DMA_INT_STATUS_CLR_ADDR, reg_data);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE && LARK_SDK_RUN_ON_HIFI3Z
#if LARK_SDK_XOS_TEST_CODE
#include <xtensa/xos.h>
#else
#include <xtensa/xtruntime.h>
#endif

/*! DMA control config data (ccd) base addr must be 1024 byte aligned */
/*! DMA ccd primary and alternate are ajacent */
/*! Primary and alternate DMA ccd unit number must be pow(2, n) */
/*! 20 DMA channels need 32 ccd units for primary and alternate respectively */
adi_lark_dma_chnl_ctrl_t dma_ctrl_data[64] __attribute__((aligned(0x400)));

#define API_LARK_DMA_TEST_BUF_SIZE  32          /*!< dma buf size (32*4bytes), for test */
int32_t __attribute__((section(".dram0.data")))  mdma_src_L1[API_LARK_DMA_TEST_BUF_SIZE];
int32_t __attribute__((section(".dram0.data")))  mdma_dst_L1[API_LARK_DMA_TEST_BUF_SIZE];
int32_t mdma_src_L2[API_LARK_DMA_TEST_BUF_SIZE], mdma_dst_L2[API_LARK_DMA_TEST_BUF_SIZE];

typedef struct
{
	adi_lark_dma_chno_e chan_no;                /*!< Channel number to be tested */
	int32_t count;                              /*!< word count */
	int8_t *src_buf;                            /*!< source buffer address for memory DMA */
	int8_t *dst_buf;                            /*!< destination buffer address for memory DMA */
	adi_lark_dma_incr_type_e src_inc;           /*!< source increment */
	adi_lark_dma_incr_type_e dst_inc;           /*!< destination increment */
	adi_lark_dma_width_type_e width;            /*!< DMA word size */
	adi_lark_dma_chnl_ctrl_t * base_addr;       /*!< base address of DMA control data */
} adi_lark_dma_test_t;

typedef struct
{
	adi_lark_dma_chno_e chan_no;                /*!< Channel number to be tested */
	int32_t count;                              /*!< word count */
	int8_t *src_buf[3];                         /*!< source buffer address for memory DMA */
	int8_t *dst_buf[3];                         /*!< destination buffer address for memory DMA */
	adi_lark_dma_incr_type_e src_inc;           /*!< source increment */
	adi_lark_dma_incr_type_e dst_inc;           /*!< destination increment */
	adi_lark_dma_width_type_e width;            /*!< DMA word size */
	adi_lark_dma_chnl_ctrl_t * base_addr;       /*!< base address of DMA control data */
} adi_lark_dma_scatter_test_t;

typedef struct
{
	int32_t  err;                               /*!< error number */
	int32_t  count;                             /*!< count of transfer that source and dest values are same */
	uint32_t dma_bus_stat;                      /*!< bus err register value */
	uint32_t invalid_desc_stat;                 /*!< invalid descriptor register value */
} adi_lark_dma_test_result_t;

typedef struct
{
	adi_lark_device_t *device;
	uint32_t int_status;
	uint32_t dma_bus_stat;
	uint32_t invalid_desc_stat;
    bool     dma_done;
    bool     dma_err;
} adi_lark_dma_int_arg_t;

int32_t adi_lark_dma_basic_test(adi_lark_device_t *device, adi_lark_dma_test_t *test_cfg, adi_lark_dma_test_result_t *test_result);
int32_t adi_lark_dma_scatter_test(adi_lark_device_t *device, adi_lark_dma_scatter_test_t *test_cfg, adi_lark_dma_test_result_t *test_result);
int32_t adi_lark_dma_ping_pong_test(adi_lark_device_t *device);
int32_t adi_lark_dma_reg_write_read_test(adi_lark_device_t *device);
void    adi_lark_dma_scatter_handler(void *arg);
void    adi_lark_dma_ping_pong_handler(void *arg);

int32_t adi_lark_dma_basic_test(adi_lark_device_t *device, adi_lark_dma_test_t *test_cfg, adi_lark_dma_test_result_t *test_result)
{
    #ifdef LARK_SDK
    int32_t  err = API_LARK_ERROR_OK;
    int32_t  i;
    uint32_t running_stat;
    uint32_t loop_count = 0;
    uint32_t src_inc, dst_inc;
    adi_lark_dma_chnl_ctrl_t *chnl_ctrl;
    adi_lark_dma_chnl_cfg_t  *chnl_cfg;
    
    test_result->err = API_LARK_ERROR_ERROR;
    test_result->count = -1;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(test_cfg->base_addr);

    if(((int32_t)test_cfg->src_inc < (int32_t)test_cfg->width) || (int32_t)test_cfg->dst_inc < (int32_t)test_cfg->width)
        return API_LARK_ERROR_ERROR;
    
    switch(test_cfg->src_inc)
    {
        case API_LARK_DMA_INCR_1_BYTE:
            src_inc = 1;
            break;
        case API_LARK_DMA_INCR_2_BYTE:
            src_inc = 2;
            break;
        case API_LARK_DMA_INCR_4_BYTE:
            src_inc = 4;
            break;
        default:
             return API_LARK_ERROR_ERROR;
    }
    
    switch(test_cfg->dst_inc)
    {
        case API_LARK_DMA_INCR_1_BYTE:
            dst_inc = 1;
            break;
        case API_LARK_DMA_INCR_2_BYTE:
            dst_inc = 2;
            break;
        case API_LARK_DMA_INCR_4_BYTE:
            dst_inc = 4;
            break;
        default:
            return API_LARK_ERROR_ERROR;
    }

    /* generate test data */
    switch(test_cfg->width)
    {
        case API_LARK_DMA_WIDTH_1_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                *(test_cfg->src_buf + i*src_inc) = i;
                *(test_cfg->dst_buf + i*dst_inc) = -1;
            }
            break;
        case API_LARK_DMA_WIDTH_2_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                *((int16_t*)test_cfg->src_buf + i*src_inc/2) = i;
                *((int16_t*)test_cfg->dst_buf + i*dst_inc/2) = -1;
            }
            break;
        case API_LARK_DMA_WIDTH_4_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                *((int32_t*)test_cfg->src_buf + i) = i;
                *((int32_t*)test_cfg->dst_buf + i) = -1;
            }
            break;
        default:
            return API_LARK_ERROR_ERROR;
    }

    test_result->count = test_cfg->count;
    chnl_ctrl = test_cfg->base_addr + test_cfg->chan_no;
    chnl_ctrl->dma_src_end = (uint32_t)test_cfg->src_buf + test_cfg->count * src_inc - 1;
    chnl_ctrl->dma_dst_end = (uint32_t)test_cfg->dst_buf + test_cfg->count * dst_inc - 1;
    chnl_cfg = (adi_lark_dma_chnl_cfg_t *)&chnl_ctrl->dma_chnl_cfg;
    chnl_cfg->src_inc    = test_cfg->src_inc;
    chnl_cfg->dst_inc    = test_cfg->dst_inc;
    chnl_cfg->src_size   = test_cfg->width;
    chnl_cfg->r_power    = 5;
    chnl_cfg->n_minus_1  = test_cfg->count - 1;
    chnl_cfg->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_BASIC;

    /* clear cache for src buffer to make sure sram has latest test data */ 
    if((uint32_t)test_cfg->base_addr >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(test_cfg->base_addr, sizeof(adi_lark_dma_chnl_ctrl_t) * 64);
    if((uint32_t)test_cfg->src_buf >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(test_cfg->src_buf, src_inc * test_cfg->count);

    err = adi_lark_dma_enable(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_init(device, (uint32_t)test_cfg->base_addr);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_dst_decrement(device, test_cfg->chan_no, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_src_decrement(device, test_cfg->chan_no, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl(device, test_cfg->chan_no, true, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_alt_ctrl(device, test_cfg->chan_no, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_start_chnl_softdma(device, test_cfg->chan_no);
    LARK_ERROR_RETURN(err);
    do
    {
        err = adi_lark_dma_get_chnl_en_stat(device, &running_stat);
        LARK_ERROR_RETURN(err);
    } while ((running_stat & (1 << test_cfg->chan_no)) && (loop_count++ < 1000));
    
    /* clear cache for dma dst buffer area for following data check */ 
    xthal_dcache_region_invalidate(test_cfg->dst_buf, dst_inc * test_cfg->count);
    
    /* verify data */
    switch(test_cfg->width)
    {
        case API_LARK_DMA_WIDTH_1_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                if(*(test_cfg->src_buf + i*src_inc) != *(test_cfg->dst_buf + i*dst_inc))
                    break;
            }
            break;
        case API_LARK_DMA_WIDTH_2_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                if(*((int16_t*)test_cfg->src_buf + i*src_inc/2) != *((int16_t*)test_cfg->dst_buf + i*dst_inc/2))
                    break;
            }
            break;
        case API_LARK_DMA_WIDTH_4_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                if(*((int32_t*)test_cfg->src_buf + i) != *((int32_t*)test_cfg->dst_buf + i))
                    break;
            }
            break;
    }
    err = (i == test_cfg->count) ? API_LARK_ERROR_OK : API_LARK_ERROR_DMA_DATA_RESULT;
    test_result->count = i;
    test_result->err = err;
    test_result->dma_bus_stat = *(volatile uint32_t *)REG_DMA_BUSERR_IDX_CLR_ADDR;
    test_result->invalid_desc_stat = *(volatile uint32_t *)REG_DMA_INVDESC_IDX_CLR_ADDR;
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

static int32_t ping_pong_count = 0;
int32_t adi_lark_dma_ping_pong_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t  err, i;
    uint32_t src_inc, dst_inc;
    adi_lark_dma_chnl_cfg_t *dma_config;
    adi_lark_dma_chnl_ctrl_t *dma_ctrl;
    uint32_t dma_alt_base, irq_id;

    for(i = 0; i < API_LARK_DMA_TEST_BUF_SIZE; i++)
    {
        mdma_src_L1[i] = i + 0xF000;
    }
    dma_ctrl = &dma_ctrl_data[0];
    dma_ctrl->dma_src_end = (uint32_t)&mdma_src_L1[API_LARK_DMA_TEST_BUF_SIZE] - 1;
    dma_ctrl->dma_dst_end = (uint32_t)&mdma_dst_L1[API_LARK_DMA_TEST_BUF_SIZE] - 1;
    dma_config = (adi_lark_dma_chnl_cfg_t*)&dma_ctrl->dma_chnl_cfg;
    dma_config->src_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->dst_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->src_size = API_LARK_DMA_WIDTH_4_BYTE;
    dma_config->r_power = 5;
    dma_config->n_minus_1 = API_LARK_DMA_TEST_BUF_SIZE - 1;
    dma_config->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_PING_PONG;

    err = adi_lark_dma_init(device, (uint32_t)dma_ctrl_data);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_get_ctrl_base_reg(device, 1, &dma_alt_base);
    LARK_ERROR_RETURN(err);

    dma_ctrl = (adi_lark_dma_chnl_ctrl_t*)dma_alt_base;
    dma_ctrl->dma_src_end = (uint32_t)&mdma_dst_L1[API_LARK_DMA_TEST_BUF_SIZE] - 1;
    dma_ctrl->dma_dst_end = (uint32_t)&mdma_dst_L2[API_LARK_DMA_TEST_BUF_SIZE] - 1;
    dma_config = (adi_lark_dma_chnl_cfg_t*)&dma_ctrl->dma_chnl_cfg;
    dma_config->src_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->dst_inc = API_LARK_DMA_INCR_4_BYTE;
    dma_config->src_size = API_LARK_DMA_WIDTH_4_BYTE;
    dma_config->r_power = 5;
    dma_config->n_minus_1 = API_LARK_DMA_TEST_BUF_SIZE - 1;
    dma_config->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_BASIC;

    /* make sure dma ctrl data is saved to ram */
    if((uint32_t)dma_ctrl_data >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(dma_ctrl_data, sizeof(adi_lark_dma_chnl_ctrl_t) * 64);
    /* register dma interrupt and enable channel xfer */
    err = adi_lark_int_get_soc_irq_id(device, API_LARK_SOC_IRQ_DMA, &irq_id);
    LARK_ERROR_RETURN(err);

    #if LARK_SDK_XOS_TEST_CODE
    err = xos_register_interrupt_handler(irq_id, adi_lark_dma_ping_pong_handler, device);
    LARK_ERROR_RETURN(err != 0 ? API_LARK_ERROR_REGISTER_ISR : API_LARK_ERROR_OK);
    xos_interrupt_enable(irq_id);
    #else
    _xtos_set_interrupt_handler_arg(irq_id, adi_lark_dma_ping_pong_handler, device);
    _xtos_interrupt_enable(irq_id);
    #endif
    err = adi_lark_dma_enable_chnl_dma_done_int(device, 0, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_multi_chnls(device, 1 << 0 , true, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_dma_err_int(device, 3);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_start_chnl_softdma(device, 0);
    LARK_ERROR_RETURN(err);
    while(ping_pong_count < 2 && ping_pong_count > -1);
    xthal_dcache_region_invalidate(mdma_dst_L2, API_LARK_DMA_TEST_BUF_SIZE * 4);
    for(i = 0; i < API_LARK_DMA_TEST_BUF_SIZE; i++)
    {
        if(mdma_dst_L2[i] != mdma_src_L1[i])
            return API_LARK_ERROR_ERROR;
    }
    #endif
    return API_LARK_ERROR_OK;
}

void adi_lark_dma_ping_pong_handler(void *arg)
{
    adi_lark_device_t *device = (adi_lark_device_t *)arg;
    uint32_t int_stat;
    if(device == NULL)
    {
        ping_pong_count = -1;
        return;
    }
    ping_pong_count++;
    int_stat = *(uint32_t*)REG_DMA_INT_STATUS_CLR_ADDR;
    *(uint32_t*)REG_DMA_INT_STATUS_CLR_ADDR = int_stat;
    adi_lark_dma_start_chnl_softdma(device, 0);
    return;
}

int32_t adi_lark_dma_scatter_test(adi_lark_device_t *device, adi_lark_dma_scatter_test_t *test_cfg, adi_lark_dma_test_result_t *test_result)
{
    #ifdef LARK_SDK
    int32_t  err = API_LARK_ERROR_OK;
    int32_t  i;
    uint32_t running_stat, reg_data;
    uint32_t loop_count = 0;
    uint32_t src_inc, dst_inc;
    uint32_t irq_id;
    static adi_lark_dma_int_arg_t arg;
    adi_lark_dma_int_arg_t   *dma_int_arg = &arg;
    adi_lark_dma_chnl_ctrl_t *chnl_ctrl, scatter_ctrl_mem[3];
    adi_lark_dma_chnl_cfg_t  *chnl_cfg;
    
    test_result->err = API_LARK_ERROR_ERROR;
    test_result->count = -1;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_NULL_POINTER_RETURN(test_cfg->base_addr);

    if(((int32_t)test_cfg->src_inc < (int32_t)test_cfg->width) || (int32_t)test_cfg->dst_inc < (int32_t)test_cfg->width)
        return API_LARK_ERROR_ERROR;
    
    switch(test_cfg->src_inc)
    {
        case API_LARK_DMA_INCR_1_BYTE:
            src_inc = 1;
            break;
        case API_LARK_DMA_INCR_2_BYTE:
            src_inc = 2;
            break;
        case API_LARK_DMA_INCR_4_BYTE:
            src_inc = 4;
            break;
        default:
            return API_LARK_ERROR_ERROR;
    }
    
    switch(test_cfg->dst_inc)
    {
        case API_LARK_DMA_INCR_1_BYTE:
            dst_inc = 1;
            break;
        case API_LARK_DMA_INCR_2_BYTE:
            dst_inc = 2;
            break;
        case API_LARK_DMA_INCR_4_BYTE:
            dst_inc = 4;
            break;
        default:
            return API_LARK_ERROR_ERROR;
    }

    /* generate test data */
    switch(test_cfg->width)
    {
        case API_LARK_DMA_WIDTH_1_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                *(test_cfg->src_buf[0] + i*src_inc) = i;
                *(test_cfg->dst_buf[0] + i*dst_inc) = -1;
                *(test_cfg->dst_buf[1] + i*dst_inc) = -1;
                *(test_cfg->dst_buf[2] + i*dst_inc) = -1;
            }
            break;
        case API_LARK_DMA_WIDTH_2_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                *((int16_t*)test_cfg->src_buf[0] + i*src_inc/2) = 0xFF + i;
                *((int16_t*)test_cfg->dst_buf[0] + i*dst_inc/2) = -1;
                *((int16_t*)test_cfg->dst_buf[1] + i*dst_inc/2) = -1;
                *((int16_t*)test_cfg->dst_buf[2] + i*dst_inc/2) = -1;
            }
            break;
        case API_LARK_DMA_WIDTH_4_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                *((int32_t*)test_cfg->src_buf[0] + i) = 0xFFFF + i;
                *((int32_t*)test_cfg->dst_buf[0] + i) = -1;
                *((int32_t*)test_cfg->dst_buf[1] + i) = -1;
                *((int32_t*)test_cfg->dst_buf[2] + i) = -1;
            }
            break;
        default:
            return API_LARK_ERROR_ERROR;
    }

    /* Configure Alternate DMA control data for real transfer */
    for(i = 0; i < 3; i++)
    {
        scatter_ctrl_mem[i].dma_src_end = (uint32_t)test_cfg->src_buf[i] + 127;
        scatter_ctrl_mem[i].dma_dst_end = (uint32_t)test_cfg->dst_buf[i] + 127;
        chnl_cfg = (adi_lark_dma_chnl_cfg_t *)&scatter_ctrl_mem[i].dma_chnl_cfg;
        chnl_cfg->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_MSG_ALT;
        chnl_cfg->dst_inc = test_cfg->dst_inc;
        chnl_cfg->src_inc = test_cfg->src_inc;
        chnl_cfg->src_size = test_cfg->width;
        chnl_cfg->r_power = 5;
        chnl_cfg->n_minus_1 = test_cfg->count - 1;
    }
    chnl_cfg->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_BASIC;                        /* The scatter DMA will stop after a basic cycle met*/
    
    /* configure primary dma control data, for transfering alternate control data*/
    err = adi_lark_dma_enable(device, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_init(device, (uint32_t)test_cfg->base_addr);                 /* init first to set alternate control data */
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_get_ctrl_base_reg(device, 1, &reg_data);
    LARK_ERROR_RETURN(err);
    reg_data += test_cfg->chan_no * sizeof(adi_lark_dma_chnl_ctrl_t);

    test_result->count = test_cfg->count;
    chnl_ctrl = test_cfg->base_addr + test_cfg->chan_no;
    chnl_ctrl->dma_src_end = (uint32_t)scatter_ctrl_mem + sizeof(adi_lark_dma_chnl_ctrl_t)* 3 - 1u;
    chnl_ctrl->dma_dst_end = reg_data + sizeof(adi_lark_dma_chnl_ctrl_t) - 1u;

    chnl_cfg = (adi_lark_dma_chnl_cfg_t *)&chnl_ctrl->dma_chnl_cfg;
    /* The following settings are fixed for all the primary scatter DMA */
    chnl_cfg->src_inc    = API_LARK_DMA_INCR_4_BYTE;
    chnl_cfg->dst_inc    = API_LARK_DMA_INCR_4_BYTE;
    chnl_cfg->src_size   = API_LARK_DMA_WIDTH_4_BYTE;
    chnl_cfg->r_power    = 2;
    /* End of <The following settings are fixed for all the primary scatter DMA> */
    chnl_cfg->n_minus_1  = 3 * 4 - 1;
    chnl_cfg->cycle_ctrl = API_LARK_DMA_CTL_CYCLE_CTL_MSG_PRI;

    /* clear cache for src buffer to make sure sram has latest test data */
    if((uint32_t)test_cfg->base_addr >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(test_cfg->base_addr, sizeof(adi_lark_dma_chnl_ctrl_t) * 64);
    if((uint32_t)test_cfg->src_buf[0] >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(test_cfg->src_buf[0], src_inc * test_cfg->count);
    if((uint32_t)scatter_ctrl_mem >= LARK_SYSRAM_START_ADDR)
        xthal_dcache_region_writeback(scatter_ctrl_mem, sizeof(adi_lark_dma_chnl_ctrl_t)* 3);

    err = adi_lark_dma_enable_chnl_dst_decrement(device, test_cfg->chan_no, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_src_decrement(device, test_cfg->chan_no, false);
    LARK_ERROR_RETURN(err);
    
    /* register dma interrupt and enable channel xfer */
    err = adi_lark_int_get_soc_irq_id(device, API_LARK_SOC_IRQ_DMA, &irq_id);
    LARK_ERROR_RETURN(err);
    dma_int_arg->device = device;
    dma_int_arg->int_status = 0;
    dma_int_arg->dma_bus_stat = 0;
    dma_int_arg->invalid_desc_stat = 0;
    dma_int_arg->dma_done = false;
    dma_int_arg->dma_err = false;
    #if LARK_SDK_XOS_TEST_CODE
    err = xos_register_interrupt_handler(irq_id, adi_lark_dma_scatter_handler, (void*)dma_int_arg);
    LARK_ERROR_RETURN(err != 0 ? API_LARK_ERROR_REGISTER_ISR : API_LARK_ERROR_OK);
    xos_interrupt_enable(irq_id);
    #else
    _xtos_set_interrupt_handler_arg(irq_id, adi_lark_dma_scatter_handler, (void*)dma_int_arg);
    _xtos_interrupt_enable(irq_id);
    #endif
    err = adi_lark_dma_enable_chnl_dma_done_int(device, test_cfg->chan_no, true);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_dma_err_int(device, 3);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl(device, test_cfg->chan_no, true, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_enable_chnl_alt_ctrl(device, test_cfg->chan_no, false);
    LARK_ERROR_RETURN(err);
    err = adi_lark_dma_start_chnl_softdma(device, test_cfg->chan_no);
    LARK_ERROR_RETURN(err);

    /* wait for dma to finish */
    while(!dma_int_arg->dma_done && !dma_int_arg->dma_err)
    {
        ;
    }
    
    /* clear cache for dma dst buffer area for following data check */ 
    xthal_dcache_region_invalidate(test_cfg->dst_buf[0], dst_inc * test_cfg->count);
    xthal_dcache_region_invalidate(test_cfg->dst_buf[1], dst_inc * test_cfg->count);
    xthal_dcache_region_invalidate(test_cfg->dst_buf[2], dst_inc * test_cfg->count);
    
    /* verify data */
    switch(test_cfg->width)
    {
        case API_LARK_DMA_WIDTH_1_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                if(*(test_cfg->src_buf[0] + i*src_inc) != *(test_cfg->dst_buf[0] + i*dst_inc))
                    break;
                if(*(test_cfg->src_buf[1] + i*src_inc) != *(test_cfg->dst_buf[1] + i*dst_inc))
                    break;
                if(*(test_cfg->src_buf[2] + i*src_inc) != *(test_cfg->dst_buf[2] + i*dst_inc))
                    break;
            }
            break;
        case API_LARK_DMA_WIDTH_2_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                if(*((int16_t*)test_cfg->src_buf[0] + i*src_inc/2) != *((int16_t*)test_cfg->dst_buf[0] + i*dst_inc/2))
                    break;
                if(*((int16_t*)test_cfg->src_buf[1] + i*src_inc/2) != *((int16_t*)test_cfg->dst_buf[1] + i*dst_inc/2))
                    break;
                if(*((int16_t*)test_cfg->src_buf[2] + i*src_inc/2) != *((int16_t*)test_cfg->dst_buf[2] + i*dst_inc/2))
                    break;
            }
            break;
        case API_LARK_DMA_WIDTH_4_BYTE:
            for(i = 0; i < test_cfg->count; i++)
            {
                if(*((int32_t*)test_cfg->src_buf[0] + i) != *((int32_t*)test_cfg->dst_buf[0] + i))
                    break;
                if(*((int32_t*)test_cfg->src_buf[1] + i) != *((int32_t*)test_cfg->dst_buf[1] + i))
                    break;
                if(*((int32_t*)test_cfg->src_buf[2] + i) != *((int32_t*)test_cfg->dst_buf[2] + i))
                    break;
            }
            break;
    }
    err = (i == test_cfg->count) ? API_LARK_ERROR_OK : API_LARK_ERROR_DMA_DATA_RESULT;
    test_result->count = i;
    test_result->err = err;
    test_result->dma_bus_stat = *(volatile uint32_t *)REG_DMA_BUSERR_IDX_CLR_ADDR;
    test_result->invalid_desc_stat = *(volatile uint32_t *)REG_DMA_INVDESC_IDX_CLR_ADDR;
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err, i, j;
    adi_lark_dma_test_t         test_cfg;
    adi_lark_dma_scatter_test_t scatter_cfg;
    adi_lark_dma_test_result_t  test_result;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    LARK_LOG_MSG("dma register write/read test start ... ");
    err = adi_lark_dma_reg_write_read_test(device);
    if (err != API_LARK_ERROR_OK) 
    {
        LARK_LOG_MSG("dma register write/read test failed.\n");
        return err;
    }
    LARK_LOG_MSG("dma register write/read test passed.\n");

    LARK_LOG_MSG("dma basic test start ...");
    for(i = 0; i < 20; i++)      /* 20 channels, @see adi_lark_dma_chno_e for more info */
    {
        for(j = 0; j < 4; j++)   /* 4 test dma xfers for each channel */
        {
            test_cfg.base_addr = dma_ctrl_data;
            test_cfg.chan_no = i;
            test_cfg.width   = API_LARK_DMA_WIDTH_4_BYTE;
            test_cfg.src_inc = API_LARK_DMA_INCR_4_BYTE;
            test_cfg.dst_inc = API_LARK_DMA_INCR_4_BYTE;
            test_cfg.count   = API_LARK_DMA_TEST_BUF_SIZE;
            
            if (j == 0) {
                /* L1 -> L1*/
                test_cfg.src_buf = (int8_t*)mdma_src_L1;
                test_cfg.dst_buf = (int8_t*)mdma_dst_L1;
            }
            if (j == 1) {
                /* L1 -> L2*/
                test_cfg.src_buf = (int8_t*)mdma_src_L1;
                test_cfg.dst_buf = (int8_t*)mdma_dst_L2;
            }
            if (j == 2) {
                /* L2 -> L1*/
                test_cfg.src_buf = (int8_t*)mdma_src_L2;
                test_cfg.dst_buf = (int8_t*)mdma_dst_L1;
            }
            if (j == 3) {
                /* L2 -> L2 */
                test_cfg.src_buf = (int8_t*)mdma_src_L2;
                test_cfg.dst_buf = (int8_t*)mdma_dst_L2;
            }
            
            LARK_LOG_VAR(LARK_LOG_INFO_MSG, "dma channel: %d", test_cfg.chan_no);
            err = adi_lark_dma_basic_test(device, &test_cfg, &test_result);
            if(err != API_LARK_ERROR_OK)
            {
                LARK_LOG_VAR(LARK_LOG_INFO_MSG, "src->dst = %d (0:L1->L1, 1:L1->L2, 2:L2->L1, 3:L2->L2)", j);
                LARK_LOG_VAR(LARK_LOG_INFO_MSG, "error no = %d", test_result.err);
                LARK_LOG_VAR(LARK_LOG_INFO_MSG, "succeeded trans = %d/32", test_result.count);
                LARK_LOG_VAR(LARK_LOG_INFO_MSG, "REG_INV_DESC = %08x", test_result.invalid_desc_stat);
                LARK_LOG_VAR(LARK_LOG_INFO_MSG, "REG_BUS_STAT = %08x", test_result.dma_bus_stat);
                LARK_LOG_MSG("dma basic test failed.\n");
                return err;
            }
        }
    }
    LARK_LOG_MSG("dma basic test passed.\n");

    LARK_LOG_MSG("dma dma scatter-gather test start ... ");
    scatter_cfg.base_addr = dma_ctrl_data;
    scatter_cfg.chan_no = API_LARK_DMA_CHAN_DATA_SYNC15;
    scatter_cfg.dst_inc = API_LARK_DMA_INCR_4_BYTE;
    scatter_cfg.src_inc = API_LARK_DMA_INCR_4_BYTE;
    scatter_cfg.width   = API_LARK_DMA_WIDTH_4_BYTE;
    scatter_cfg.count   = API_LARK_DMA_TEST_BUF_SIZE;
    scatter_cfg.src_buf[0] = (int8_t*)mdma_src_L1;
    scatter_cfg.dst_buf[0] = (int8_t*)mdma_dst_L1; /* L1 -> L1 */
    scatter_cfg.src_buf[1] = (int8_t*)mdma_dst_L1;
    scatter_cfg.dst_buf[1] = (int8_t*)mdma_src_L2; /* L1 -> L2 */
    scatter_cfg.src_buf[2] = (int8_t*)mdma_src_L2;
    scatter_cfg.dst_buf[2] = (int8_t*)mdma_dst_L2; /* L2 -> L2 */
    err = adi_lark_dma_scatter_test(device, &scatter_cfg, &test_result);
    if (err != API_LARK_ERROR_OK)
    {
        LARK_LOG_MSG("dma scatter-gather test failed.\n");
        return err;
    }
    LARK_LOG_MSG("dma scatter-gather test passed.\n");
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_dma_reg_write_read_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t reg_value;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_hal_reg_write(device, REG_DMA_CTRL_ADDR, 1);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_STATUS_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if((reg_value & 1) != 1)
    {
        LARK_LOG_ERR("REG_DMA_CTRL_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }

    err = adi_lark_hal_reg_write(device, REG_DMA_PRI_BASE_PTR_ADDR, 0x60009000);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_PRI_BASE_PTR_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0x60009000)
    {
        LARK_LOG_ERR("REG_DMA_PRI_BASE_PTR_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }

    err = adi_lark_hal_reg_read(device, REG_DMA_ALT_BASE_PTR_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0x60009200)
    {
        LARK_LOG_ERR("REG_DMA_ALT_BASE_PTR_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }

    err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, 0xFFFFF);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_REQ_MASK_SET_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0xFFFFF)
    {
        LARK_LOG_ERR("REG_DMA_REQ_MASK_SET_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }
    err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_CLR_ADDR, 0xFFF00);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_REQ_MASK_SET_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0x000FF)
    {
        LARK_LOG_ERR("REG_DMA_REQ_MASK_CLR_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }
    err = adi_lark_hal_reg_write(device, REG_DMA_REQ_MASK_SET_ADDR, 0xFFFFF);
    LARK_ERROR_RETURN(err);

    err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_SET_ADDR, 0xFFFFF);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_CHNL_EN_SET_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0xFFFFF)
    {
        LARK_LOG_ERR("REG_DMA_CHNL_EN_SET_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }
    err = adi_lark_hal_reg_write(device, REG_DMA_CHNL_EN_CLR_ADDR, 0xFFFFF);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_CHNL_EN_SET_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0)
    {
        LARK_LOG_ERR("REG_DMA_CHNL_EN_CLR_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }
    
    err = adi_lark_hal_reg_write(device, REG_DMA_INT_EN_SET_ADDR, 0xFFFFF);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_INT_EN_SET_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0xFFFFF)
    {
        LARK_LOG_ERR("REG_DMA_INT_EN_SET_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }
    err = adi_lark_hal_reg_write(device, REG_DMA_INT_EN_CLR_ADDR, 0xFFFFF);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, REG_DMA_INT_EN_SET_ADDR, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0)
    {
        LARK_LOG_ERR("REG_DMA_INT_EN_CLR_ADDR write/read error\n");
        return API_LARK_ERROR_ERROR;
    }

    err = adi_lark_hal_reg_write(device, 0x40002020, 0xA0A0);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_reg_read(device, 0x40002020, &reg_value);
    LARK_ERROR_RETURN(err);
    if(reg_value != 0x4144)
    {
        LARK_LOG_ERR("0x40002020 ADI_ID write/read error\n");
    }
    #endif
    
    return API_LARK_ERROR_OK;
}

void adi_lark_dma_scatter_handler(void *arg)
{
    adi_lark_device_t *device;
    adi_lark_dma_int_arg_t *dma_int_arg;
    adi_lark_log_write log_func;
    if(arg == NULL)
        return;
    dma_int_arg = (adi_lark_dma_int_arg_t *)arg;
    device = dma_int_arg->device;
    if(device == NULL)
        return;
    log_func = device->log_write;
    device->log_write = NULL;                                       /* XOS does not allow fprintf in ISR */
    adi_lark_dma_get_int_stat(device, &dma_int_arg->int_status);
    adi_lark_dma_get_buserr_stat(device, &dma_int_arg->dma_bus_stat);
    adi_lark_dma_get_invalid_desc_stat(device, &dma_int_arg->invalid_desc_stat);
    if((dma_int_arg->invalid_desc_stat > 0) || (dma_int_arg->dma_bus_stat > 0))
        dma_int_arg->dma_err = true;
    else
        dma_int_arg->dma_done = true;
    adi_lark_dma_clr_int_stat(device,dma_int_arg->int_status);
    device->log_write = log_func;
}
#endif /*LARK_SDK_EN_TEST_CODE*/

/*! @} */
