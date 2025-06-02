/*!
 * @brief     Hardare Abstract Layer Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= C O D E ====================*/
int32_t adi_lark_hal_mem_read(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_length)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(data);
    switch(device->cp_mode)
    {
        case API_LARK_CP_I2C:
            err = adi_lark_cp_i2c_mem_read(device, addr, data, byte_length);
            break;
        case API_LARK_CP_SPI:
            err = adi_lark_cp_spi_mem_read(device, addr, data, byte_length);
            break;
        case API_LARK_CP_UART:
            err = adi_lark_cp_uart_mem_read(device, addr, data, byte_length);
            break;
        case API_LARK_CP_NATIVE:
            err = adi_lark_cp_native_mem_read(device, addr, data, byte_length);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_mem_write(adi_lark_device_t *device, uint32_t addr, uint8_t *data, uint32_t byte_length)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(data);
    switch(device->cp_mode)
    {
        case API_LARK_CP_I2C:
            err = adi_lark_cp_i2c_mem_write(device, addr, data, byte_length);
            break;
        case API_LARK_CP_SPI:
            err = adi_lark_cp_spi_mem_write(device, addr, data, byte_length);
            break;
        case API_LARK_CP_UART:
            err = adi_lark_cp_uart_mem_write(device, addr, data, byte_length);
            break;
        case API_LARK_CP_NATIVE:
            err = adi_lark_cp_native_mem_write(device, addr, data, byte_length);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    LARK_ERROR_RETURN(err);

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_reg_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t *reg_data)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(reg_data);

    switch(device->cp_mode)
    {
        case API_LARK_CP_NATIVE:
            err = adi_lark_cp_native_reg_read(device, reg_addr, reg_data);
            break;
        case API_LARK_CP_I2C:
            err = adi_lark_cp_i2c_reg_read(device, reg_addr, reg_data);
            break;
        case API_LARK_CP_SPI:
            err = adi_lark_cp_spi_reg_read(device, reg_addr, reg_data);
            break;
        case API_LARK_CP_UART:
            err = adi_lark_cp_uart_reg_read(device, reg_addr, reg_data);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    LARK_ERROR_RETURN(err);
    LARK_LOG_REG("r@%.8x = %.8x", reg_addr, *reg_data);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_reg_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t reg_data)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);

    switch(device->cp_mode)
    {
        case API_LARK_CP_NATIVE:
            err = adi_lark_cp_native_reg_write(device, reg_addr, reg_data);
            break;
        case API_LARK_CP_I2C:
            err = adi_lark_cp_i2c_reg_write(device, reg_addr, reg_data);
            break;
        case API_LARK_CP_SPI:
            err = adi_lark_cp_spi_reg_write(device, reg_addr, reg_data);
            break;
        case API_LARK_CP_UART:
            err = adi_lark_cp_uart_reg_write(device, reg_addr, reg_data);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    LARK_ERROR_RETURN(err);
    LARK_LOG_REG("w@%.8x = %.8x", reg_addr, reg_data);
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_bf_read(adi_lark_device_t *device, uint32_t reg_addr, uint32_t bf_info, uint32_t *bf_val)
{
    uint32_t reg_value, reg_temp, reg_mask;
    int32_t  err, reg_count, i;
    uint8_t  bit_start = bf_info;
    uint8_t  bit_count = bf_info >> 8;
    LARK_NULL_POINTER_RETURN(device);
    LARK_NULL_POINTER_RETURN(bf_val);
    LARK_INVALID_PARAM_RETURN((bit_count > 32) || (bit_count == 0));

    if(reg_addr >= LARK_8BIT_REG_ADDR_START && reg_addr <= LARK_8BIT_REG_ADDR_END)
    {
        reg_value = 0;
        reg_count = ((bit_start + bit_count) >> 3) + ((((bit_start + bit_count) & 7) > 0) ? 1 : 0);
        for(i = 0; i < reg_count; i++)
        {
            err = adi_lark_hal_reg_read(device, reg_addr + i, &reg_temp);
            LARK_ERROR_RETURN(err);
            reg_value += reg_temp << (i * 8);
        }
        reg_mask = (bit_count == 32) ? 0xffffffff : ((1 << bit_count) - 1);
        *bf_val  = (reg_value >> bit_start) & reg_mask;
    }
    else
    {
        err = adi_lark_hal_reg_read(device, reg_addr, &reg_value);
        LARK_ERROR_RETURN(err);
        if((bit_count + bit_start) <= 32)
        {
            reg_mask = (bit_count == 32) ? 0xffffffff : ((1 << bit_count) - 1);
            *bf_val  = (reg_value >> bit_start) & reg_mask;
        }
        else
        {
            *bf_val = reg_value >> bit_start;
            err = adi_lark_hal_reg_read(device, reg_addr + 4, &reg_value);
            LARK_ERROR_RETURN(err);
            reg_mask = (1 << (bit_count - (32 - bit_start))) - 1;
            *bf_val += (reg_value & reg_mask) << (32 - bit_start);
        }
    }
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_bf_write(adi_lark_device_t *device, uint32_t reg_addr, uint32_t bf_info, uint32_t bf_val)
{
    uint32_t reg_value, reg_temp, reg_mask;
    int32_t  err, reg_count, i;
    uint8_t  bit_start = bf_info;
    uint8_t  bit_count = bf_info >> 8;    
    LARK_NULL_POINTER_RETURN(device);
    LARK_INVALID_PARAM_RETURN((bit_count > 32) || (bit_count == 0));

    if(reg_addr >= LARK_8BIT_REG_ADDR_START && reg_addr <= LARK_8BIT_REG_ADDR_END)
    {
        if ((bit_count == 8) && (bit_start == 0))
        {
            err = adi_lark_hal_reg_write(device, reg_addr, bf_val);
            LARK_ERROR_RETURN(err);
        }
        else
        {
            reg_value = 0;
            reg_count = ((bit_start + bit_count) >> 3) + ((((bit_start + bit_count) & 7) > 0) ? 1 : 0);
            for(i = 0; i < reg_count; i++)
            {
                err = adi_lark_hal_reg_read(device, reg_addr + i, &reg_temp);
                LARK_ERROR_RETURN(err);
                reg_value += reg_temp << (i * 8);
            }
            reg_mask = (bit_count == 32) ? 0xffffffff : ((1 << bit_count) - 1);
            reg_value &= ~(reg_mask << bit_start);
            reg_value |=  (bf_val   << bit_start);
            for(i = 0; i < reg_count; i++)
            {
                err = adi_lark_hal_reg_write(device, reg_addr + i, (reg_value >> (i * 8)) & 0xff);
                LARK_ERROR_RETURN(err);
            }
        }
    }
    else
    {
        if ((bit_count == 32) && (bit_start == 0))
        {
            err = adi_lark_hal_reg_write(device, reg_addr, bf_val);
            LARK_ERROR_RETURN(err);
        }
        else
        {
            err = adi_lark_hal_reg_read(device, reg_addr, &reg_value);
            LARK_ERROR_RETURN(err);
            if((bit_count + bit_start) <= 32)
            {
                reg_mask = (bit_count == 32) ? 0xffffffff : ((1 << bit_count) - 1);
                reg_value &= ~(reg_mask << bit_start);
                reg_value |=  (bf_val   << bit_start);
                err = adi_lark_hal_reg_write(device, reg_addr, reg_value);
                LARK_ERROR_RETURN(err);
            }
            else
            {
                reg_mask = 0xffffffff;
                reg_value &= ~(reg_mask << bit_start);
                reg_value |=  (bf_val   << bit_start);
                err = adi_lark_hal_reg_write(device, reg_addr, reg_value);
                LARK_ERROR_RETURN(err);
                err = adi_lark_hal_reg_read(device, reg_addr + 4, &reg_value);
                LARK_ERROR_RETURN(err);
                reg_mask = (1 << (bit_count - (32 - bit_start))) - 1;
                reg_value &= ~reg_mask;
                reg_value |= (bf_val >> (32 - bit_start));
                err = adi_lark_hal_reg_write(device, reg_addr, reg_value);
                LARK_ERROR_RETURN(err);
            }
        }
    }
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_error_report(adi_lark_device_t* device, uint32_t log_type, const char* file_name, const char* func_name, uint32_t line_num, const char* var_name, const char* comment)
{
    if (device == NULL)
        return API_LARK_ERROR_NULL_PARAM;
    if (file_name == NULL)
        return API_LARK_ERROR_NULL_PARAM;
    if (func_name == NULL)
        return API_LARK_ERROR_NULL_PARAM;
    if (var_name == NULL)
        return API_LARK_ERROR_NULL_PARAM;
    if (comment == NULL)
        return API_LARK_ERROR_NULL_PARAM;

    if (API_LARK_ERROR_OK != adi_lark_hal_log_write(device, log_type, "%s, \"%s\" in %s(...), line%d in %s", 
        comment, var_name, func_name, line_num, file_name))
        return API_LARK_ERROR_LOG_WRITE;

    return API_LARK_ERROR_OK;
}

int32_t adi_lark_hal_log_write(adi_lark_device_t *device, uint32_t log_type, const char* comment, ...)
{
    #if ((LARK_REPORT_VERBOSE & 0x0000ffff) > 0) && ((LARK_REPORT_VERBOSE & 0xffff0000) > 0)
    va_list argp;
    char *msg_header = NULL;
    static char logmsgbuf[100];
    if (device == NULL)
        return API_LARK_ERROR_NULL_PARAM;
    if (comment == NULL)
        return API_LARK_ERROR_NULL_PARAM;
    if ((device->log_write != NULL) && ((LARK_REPORT_VERBOSE & log_type & 0x0000ffff) > 0) && ((LARK_REPORT_VERBOSE & log_type & 0xffff0000) > 0))
    {
        va_start(argp, comment);
        if (((LARK_REPORT_VERBOSE & LARK_LOG_ERR_MSG)  > 0) && ((log_type & LARK_LOG_ERR_MSG)  > 0))
            msg_header = "ERR ";
        if (((LARK_REPORT_VERBOSE & LARK_LOG_WARN_MSG) > 0) && ((log_type & LARK_LOG_WARN_MSG) > 0))
            msg_header = "WARN";
        if (((LARK_REPORT_VERBOSE & LARK_LOG_INFO_MSG) > 0) && ((log_type & LARK_LOG_INFO_MSG) > 0))
            msg_header = "INFO";
        if (sprintf(logmsgbuf, "%s: ", msg_header) < 0)
            return API_LARK_ERROR_LOG_WRITE;
        if (vsprintf(logmsgbuf + 6, comment, argp) < 0)
            return API_LARK_ERROR_LOG_WRITE;
        if (API_LARK_ERROR_OK != device->log_write(device->user_data, logmsgbuf))
            return API_LARK_ERROR_LOG_WRITE;
        va_end(argp);
    }
    #else
    (void)device;
    (void)log_type;
    (void)comment;
    #endif

    return API_LARK_ERROR_OK;
}

/*! @} */
