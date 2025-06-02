/*!
 * @brief     FIFO SRAM APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#define FIFO_CHNL_BASE_ADDR(fifo)       (FFSRAM0_CFG + fifo->block_id * 0x10000 + fifo->channel_id * 0x20)
#define BF_FIFO_MODE_EN_INFO(fifo)      (REG_FF0_CSR_ADDR(FIFO_CHNL_BASE_ADDR(fifo))), 0x00000310

/*============= C O D E ====================*/
int32_t adi_lark_ffsram_init(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t fifo_init_data, reg_addr, fifo_end_section;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    LARK_INVALID_PARAM_RETURN(fifo_info->width != 16  && fifo_info->width != 32);

    fifo_end_section = fifo_info->base_offset_of_256B + fifo_info->len_of_256B - 1;
    if(fifo_info->block_id == 2)
    {
        LARK_INVALID_PARAM_RETURN(fifo_info->base_offset_of_256B > 0x7F);
        LARK_INVALID_PARAM_RETURN(fifo_info->len_of_256B - 1 > 0x7E);
        LARK_INVALID_PARAM_RETURN(fifo_end_section > 0x7F);
    }
    else
    {
        LARK_INVALID_PARAM_RETURN(fifo_info->len_of_256B - 1 > 0xFE);
        LARK_INVALID_PARAM_RETURN(fifo_end_section > 0xFF);
    }
    fifo_init_data = fifo_info->base_offset_of_256B + ((fifo_info->len_of_256B - 1) << 8);
    fifo_init_data = (fifo_info->width == 16) ? (fifo_init_data + (1 << 16)) : fifo_init_data;
    reg_addr = REG_FF0_INIT_ADDR(FIFO_CHNL_BASE_ADDR(fifo_info));
    err = adi_lark_hal_reg_write(device, reg_addr, fifo_init_data);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_fifo_len(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *len_of_256B)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_read(device, BF_FIFO0_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);
    LARK_ERROR_RETURN(err);
    *len_of_256B = (bf_val & 0xFF) + 1;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_fifo_base(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *offset_of_256B)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_read(device, BF_FIFO0_BASE_ADDR_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);
    LARK_ERROR_RETURN(err);
    *offset_of_256B = bf_val & 0xFF;
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_read_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *data_len, int32_t *data_buf)
{
    #ifdef LARK_SDK
    int32_t  err, i;
    uint32_t fifo_en_flag, dline_en_flag, reg_addr, reg_data;
    uint8_t  data_width;
    uint16_t *buf_ptr, avail_len;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_ffsram_get_fifo_width(device, fifo_info, &data_width);
    LARK_ERROR_RETURN(err);
    if(data_width != fifo_info->width)                          /* check bit width */
        return API_LARK_ERROR_FFSRAM_FIFO_WIDTH;
    if((uint32_t)data_buf%(data_width/8u) != 0)                 /* check buffer start address alignment */
        return API_LARK_ERROR_FFSRAM_FIFO_ALIGN;
        
    err = adi_lark_hal_bf_read(device, BF_FIFO0_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &fifo_en_flag);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_read(device, BF_FIFO0_DLINE_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &dline_en_flag);
    LARK_ERROR_RETURN(err);
    if(fifo_en_flag && dline_en_flag)
        return API_LARK_ERROR_FFSRAM_FIFO_DLINE_EN;
    if(!fifo_en_flag && !dline_en_flag)
        return API_LARK_ERROR_FFSRAM_FIFO_DLINE_DIS;
        
    err = adi_lark_ffsram_get_avail_size(device, fifo_info, &avail_len);
    LARK_ERROR_RETURN(err);
    avail_len /= data_width/8;                                  /* get available length of bit width*/
    if(avail_len < *data_len)
        *data_len =  avail_len;

    /* removed length limit to 1 for delay line */
    reg_addr = REG_FF0_PORT_ADDR(FIFO_CHNL_BASE_ADDR(fifo_info));
    if(data_width == 16)
    {
        buf_ptr = (uint16_t*)data_buf;
        for(i = 0; i < *data_len; i++)
        {
            err = adi_lark_hal_reg_read(device, reg_addr, &reg_data);
            *(buf_ptr + i) = reg_data & 0xFFFF;
            LARK_ERROR_RETURN(err);
        }
    } 
    else
    {
        for(i = 0; i < *data_len; i++)
        {
            err = adi_lark_hal_reg_read(device, reg_addr, (uint32_t*)(data_buf + i));
            LARK_ERROR_RETURN(err);
        }
    }
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_write_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *data_len, int32_t *data_buf)
{
    #ifdef LARK_SDK
    int32_t  err, i;
    uint32_t fifo_en_flag, dline_en_flag, reg_addr, free_len;
    uint8_t  data_width;
    uint16_t *buf_ptr;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_ffsram_get_fifo_width(device, fifo_info, &data_width);
    LARK_ERROR_RETURN(err);
    if(data_width != fifo_info->width)                          /* check bit width */
        return API_LARK_ERROR_FFSRAM_FIFO_WIDTH;
    if((uint32_t)data_buf%(data_width/8u) != 0)                 /* check buffer start address alignment */
        return API_LARK_ERROR_FFSRAM_FIFO_ALIGN;

    err = adi_lark_hal_bf_read(device, BF_FIFO0_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &fifo_en_flag);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_read(device, BF_FIFO0_DLINE_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &dline_en_flag);
    LARK_ERROR_RETURN(err);
    if(fifo_en_flag && dline_en_flag)
        return API_LARK_ERROR_FFSRAM_FIFO_DLINE_EN;
    if(!fifo_en_flag && !dline_en_flag)
        return API_LARK_ERROR_FFSRAM_FIFO_DLINE_DIS;
        
    err = adi_lark_ffsram_get_free_size(device, fifo_info, &free_len);
    LARK_ERROR_RETURN(err);
    free_len /= data_width/8;                                   /* get available length of bit width*/
    if(free_len < *data_len)
        *data_len =  free_len;

    /* removed length limit to 1 for delay line, no write limitation */
    reg_addr = REG_FF0_PORT_ADDR(FIFO_CHNL_BASE_ADDR(fifo_info));
    if(data_width == 16)
    {
        buf_ptr = (uint16_t*)data_buf;
        for(i = 0; i < *data_len; i++)
        {
            err = adi_lark_hal_reg_write(device, reg_addr, *(buf_ptr + i));
            LARK_ERROR_RETURN(err);
        }
    } 
    else
    {
        for(i = 0; i < *data_len; i++)
        {
            err = adi_lark_hal_reg_write(device, reg_addr, *(data_buf + i));
            LARK_ERROR_RETURN(err);
        }
    }
    #endif
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_avail_size(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *byte_len)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t bf_val;

    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_hal_bf_read(device, BF_FIFO0_USED_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);
    *byte_len = bf_val & 0xFFFF;
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_free_size(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *byte_len)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t fifo_size;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_hal_bf_read(device, BF_FIFO0_USED_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), byte_len);
    LARK_ERROR_RETURN(err);
    err = adi_lark_hal_bf_read(device, BF_FIFO0_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &fifo_size);
    LARK_ERROR_RETURN(err);
    fifo_size = (fifo_size + 1) * 256;
    if(fifo_size < *byte_len)
        return API_LARK_ERROR_FFSRAM_FIFO_SIZE;

    *byte_len = fifo_size - *byte_len;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_fifo_width(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *width)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t fifo_width;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_read(device, BF_FIFO0_WIDTH_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &fifo_width);
    LARK_ERROR_RETURN(err);
    if(fifo_width)
        *width = 16;
    else
        *width = 32;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_read_ptr(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *rptr_offset)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_hal_bf_read(device, BF_FIFO0_RD_PTR_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);
    LARK_ERROR_RETURN(err);
    *rptr_offset = bf_val & 0xFFFF;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_write_ptr(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *wptr_offset)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_hal_bf_read(device, BF_FIFO0_WR_PTR_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);
    LARK_ERROR_RETURN(err);
    *wptr_offset = bf_val & 0xFFFF;
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_enable_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    if(enable_flag)
    {
        /* delay_line_en and fifo_en should not be enabled at the sametime */
        err = adi_lark_hal_bf_write(device, BF_FIFO_MODE_EN_INFO(fifo_info), 3);
        LARK_ERROR_RETURN(err);
    }
    else
    {
        err = adi_lark_hal_bf_write(device, BF_FIFO0_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), 0);
        LARK_ERROR_RETURN(err);
    }
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_chnl_enable_state(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *enable_state)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_read(device, BF_FIFO0_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), enable_state);
    LARK_ERROR_RETURN(err);
    #endif

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_enable_delay_line(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    if(enable_flag)
    {
        /* delay_line_en and fifo_en should not be enabled at the sametime */
        err = adi_lark_hal_bf_write(device, BF_FIFO_MODE_EN_INFO(fifo_info), 5);
        LARK_ERROR_RETURN(err);
    }
    else
    {
        err = adi_lark_hal_bf_write(device, BF_FIFO0_DLINE_EN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), 0);
        LARK_ERROR_RETURN(err);
    }
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_flush_fifo(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_hal_bf_write(device, BF_FIFO0_FLUSH_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), 1);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_enable_int(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, adi_lark_ffsram_int_src_e int_src, bool enable_flag)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_info, reg_addr;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    LARK_INVALID_PARAM_RETURN(int_src > API_LARK_FFSRAM_INT_FIFO_OVFLOW);

    bf_info  = (1 << 8) + int_src + 0x08;
    reg_addr = REG_FF0_INT_ADDR(FIFO_CHNL_BASE_ADDR(fifo_info));
    err = adi_lark_hal_bf_write(device, reg_addr, bf_info, enable_flag ? 1 : 0);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK; 
}

int32_t adi_lark_ffsram_get_fifo_int_stat(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint32_t *int_stat_all)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_info, reg_addr;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    bf_info = (API_LARK_FFSRAM_INT_FIFO_COUNT << 8) + 0x00;
    reg_addr = REG_FF0_INT_ADDR(FIFO_CHNL_BASE_ADDR(fifo_info));
    err = adi_lark_hal_bf_read(device, reg_addr, bf_info, int_stat_all);
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK; 
}

int32_t adi_lark_ffsram_clr_fifo_int_stat(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, adi_lark_ffsram_int_src_e int_src)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_info, reg_addr;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    LARK_INVALID_PARAM_RETURN(int_src > API_LARK_FFSRAM_INT_FIFO_OVFLOW);
    
    bf_info = (1 << 8) + int_src + 0x0010;
    reg_addr = REG_FF0_INT_ADDR(FIFO_CHNL_BASE_ADDR(fifo_info));
    err = adi_lark_hal_bf_write(device, reg_addr, bf_info, 1);  /* write 1 to clear */           
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK; 
}
int32_t adi_lark_ffsram_set_th_full(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t almf_th)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_write(device, BF_FIFO0_ALMF_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), (uint32_t)almf_th);    
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK; 
}

int32_t adi_lark_ffsram_get_th_full(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *almf_th)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_read(device, BF_FIFO0_ALMF_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);    
    LARK_ERROR_RETURN(err);
    *almf_th = bf_val & 0xFF;
    #endif
    
    return API_LARK_ERROR_OK; 
}

int32_t adi_lark_ffsram_set_th_empty(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t alme_th)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_write(device, BF_FIFO0_ALME_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), (uint32_t)alme_th);    
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_th_empty(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint8_t *alme_th)
{
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);

    err = adi_lark_hal_bf_read(device, BF_FIFO0_ALME_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);    
    LARK_ERROR_RETURN(err);
    *alme_th = bf_val & 0xFF;
    #endif
    
    return API_LARK_ERROR_OK; 
}

int32_t adi_lark_ffsram_set_dline_len(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t dline_len)
{
    /* delay line length can be changed on the fly */
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    if(fifo_info->block_id == 2)
    {
        LARK_INVALID_PARAM_RETURN(dline_len > 0x3FFF);      /* 32K FIFO SRAM, */
    }
    else
    {
        LARK_INVALID_PARAM_RETURN(dline_len > 0x7FFF);      /* 64K FIFO SRAM, */
    }

    err = adi_lark_hal_bf_write(device, BF_FIFO0_DLINE_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), (uint32_t)dline_len);    
    LARK_ERROR_RETURN(err);
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_get_dline_len(adi_lark_device_t *device, adi_lark_ffsram_fifo_info_t *fifo_info, uint16_t *dline_len)
{
    /* delay line length can be changed on the fly */
    #ifdef LARK_SDK
    int32_t err;
    uint32_t bf_val;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(fifo_info->block_id > 2 || fifo_info->channel_id > 3);
    
    err = adi_lark_hal_bf_read(device, BF_FIFO0_DLINE_LEN_INFO(FIFO_CHNL_BASE_ADDR(fifo_info)), &bf_val);    
    LARK_ERROR_RETURN(err);
    *dline_len = bf_val & 0xFFFF;
    #endif
    
    return API_LARK_ERROR_OK;
}

#if LARK_SDK_EN_TEST_CODE
int32_t adi_lark_ffsram_test_max_values(adi_lark_device_t *device);

int32_t adi_lark_ffsram_test(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    err = adi_lark_pmu_enable_master_block(device, true);
    LARK_ERROR_RETURN(err);
    LARK_LOG_MSG("ffsram test start...\n");
    err = adi_lark_ffsram_test_max_values(device);
    LARK_ERROR_RETURN(err);
    #endif 
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_ffsram_test_max_values(adi_lark_device_t *device)
{
    #ifdef LARK_SDK
    int32_t  err, data, i;
    uint8_t  bf_val_8;
    uint16_t bf_val_16, j;
    uint32_t len, bf_val_32;
    adi_lark_ffsram_fifo_info_t fifo_info[12];
    adi_lark_log_write log_func;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    for(i = 0; i < 12; i++)
    {
        fifo_info[i].block_id = i/4;
        fifo_info[i].channel_id = i%4;
        fifo_info[i].len_of_256B = fifo_info[i].block_id == 2 ? 0x7E + 1 : 0xFE + 1;
        fifo_info[i].base_offset_of_256B = 0;
        fifo_info[i].width =32;
        err = adi_lark_ffsram_init(device, &fifo_info[i]);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_fifo_len(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM max len for inst %d channel %d = %02x\n",fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        if(bf_val_16 != fifo_info[i].len_of_256B)
        {
            LARK_LOG_VAR(LARK_LOG_ERR_MSG, "ffsram max len error: Inst %d, channel %d\n", fifo_info[i].block_id, fifo_info[i].channel_id);
        }
        
        err = adi_lark_ffsram_set_dline_len(device, &fifo_info[i], fifo_info[i].block_id == 2 ? 0x3FFF : 0x7FFF);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_dline_len(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d DLINE: 0x7FFF->%04x\n", fifo_info[i].block_id, bf_val_16);

        err = adi_lark_ffsram_set_th_empty(device, &fifo_info[i], 0xFE);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_th_empty(device, &fifo_info[i], &bf_val_8);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d ALME: 0xFF->%02x\n", fifo_info[i].block_id, bf_val_8);
        err = adi_lark_ffsram_set_th_empty(device, &fifo_info[i], 0x7E);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_th_empty(device, &fifo_info[i], &bf_val_8);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d ALME: 0x7F->%02x\n", fifo_info[i].block_id, bf_val_8);

        err = adi_lark_ffsram_set_th_full(device, &fifo_info[i], 0xFE);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_th_full(device, &fifo_info[i], &bf_val_8);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d ALMF: 0xFF->%02x\n", fifo_info[i].block_id, bf_val_8);
        err = adi_lark_ffsram_set_th_full(device, &fifo_info[i], 0x7E);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_th_full(device, &fifo_info[i], &bf_val_8);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d ALMF: 0x7F->%02x\n", fifo_info[i].block_id, bf_val_8);

        err = adi_lark_ffsram_enable_fifo(device, &fifo_info[i], true);
        LARK_ERROR_RETURN(err);
        
        err = adi_lark_ffsram_get_free_size(device, &fifo_info[i], &bf_val_32);
        LARK_ERROR_RETURN(err);
        bf_val_32 /= fifo_info[i].width/8;
        bf_val_32 -= 1;                                                         /* prevent write pointer wrap around */
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d Writing %04x words\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_32);
        log_func = device->log_write;
        device->log_write = NULL;
        for(j = 0; j < bf_val_32; j++)
        {
            data = j;
            len = 1;
            err = adi_lark_ffsram_write_fifo(device, &fifo_info[i], &len, &data);
            LARK_ERROR_RETURN(err);
        }
        device->log_write = log_func;

        err = adi_lark_ffsram_get_write_ptr(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d WR PTR: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        
        data = j;
        len = 1;
        err = adi_lark_ffsram_write_fifo(device, &fifo_info[i], &len, &data);      /* Full the fifo to get the max used size */
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_avail_size(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d max used size: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        err = adi_lark_ffsram_get_write_ptr(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d WR Full PTR: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        err = adi_lark_ffsram_write_fifo(device, &fifo_info[i], &len, &data);      /* overflow the fifo to get the max used size */
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_write_ptr(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d WR O/V PTR: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        err = adi_lark_ffsram_get_avail_size(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d max used size: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);

        bf_val_16 /= fifo_info[i].width/8;
        if(bf_val_16 == 0)
            bf_val_16 = fifo_info[i].block_id == 2 ? 8159 : 16319;
        else
            bf_val_16 -= 1;                                                         /* prevent read pointer wrap around */
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d reading %04x words\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        log_func = device->log_write;
        device->log_write = NULL;
        for(j = 0; j < bf_val_16; j++)
        {
            len = 1; 
            err = adi_lark_ffsram_read_fifo(device, &fifo_info[i], &len, &data);
            LARK_ERROR_RETURN(err);
            if(data != j)
                LARK_LOG_VAR(LARK_LOG_ERR_MSG, "FFSRAM %d CHN %d WR/RD error: %d != %d\n", fifo_info[i].block_id, fifo_info[i].channel_id, j, data);
        }
        device->log_write = log_func;

        err = adi_lark_ffsram_get_read_ptr(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d RD PTR: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        err = adi_lark_ffsram_read_fifo(device, &fifo_info[i], &len, &data);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_read_ptr(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d RD EPT PTR: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);
        err = adi_lark_ffsram_read_fifo(device, &fifo_info[i], &len, &data);
        LARK_ERROR_RETURN(err);
        err = adi_lark_ffsram_get_read_ptr(device, &fifo_info[i], &bf_val_16);
        LARK_ERROR_RETURN(err);
        LARK_LOG_VAR(LARK_LOG_INFO_MSG, "FFSRAM %d CHN %d RD U/F PTR: %04x\n", fifo_info[i].block_id, fifo_info[i].channel_id, bf_val_16);

    }

    #endif 
    return API_LARK_ERROR_OK;
}
#endif /* LARK_SDK_EN_TEST_CODE */

/*! @} */
