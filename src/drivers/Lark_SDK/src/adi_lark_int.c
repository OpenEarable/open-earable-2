/*!
 * @brief     Interrupt APIs Implementation
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_lark.h"

/*============= D E F I N E S ==============*/
#define LEVEL_INT_MASK  0x00007fff
#define EDGE_INT_MASK   0x03000000
#define EDGE_INT_SHIFT  24

/*============= C O D E ====================*/
int32_t adi_lark_int_clr_irq(adi_lark_device_t *device, uint8_t index)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    switch(index)
    {
        case 1:
            err = adi_lark_hal_bf_write(device, BF_IRQ1_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 2:
            err = adi_lark_hal_bf_write(device, BF_IRQ2_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        #ifdef LARK_SDK 
        case 3:
            err = adi_lark_hal_bf_write(device, BF_IRQ3_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 4:
            err = adi_lark_hal_bf_write(device, BF_IRQ4_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        #endif
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_set_irq_pol(adi_lark_device_t *device, uint8_t index, uint8_t pol)
{
    int32_t err;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
 
    switch(index)
    {
        case 1:
            err = adi_lark_hal_bf_write(device, BF_IRQ1_PINOUT_POL_INFO, pol);
            LARK_ERROR_RETURN(err);
            break;
        case 2:
            err = adi_lark_hal_bf_write(device, BF_IRQ2_PINOUT_POL_INFO, pol);
            LARK_ERROR_RETURN(err);
            break;
        #ifdef LARK_SDK 
        case 3:
            err = adi_lark_hal_bf_write(device, BF_IRQ3_PINOUT_POL_INFO, pol);
            LARK_ERROR_RETURN(err);
            break;
        case 4:
            err = adi_lark_hal_bf_write(device, BF_IRQ4_PINOUT_POL_INFO, pol);
            LARK_ERROR_RETURN(err);
            break;
        #endif
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_enable_irq(adi_lark_device_t *device, uint8_t index, adi_lark_irq_src_e irq_src, uint8_t enable)
{
    int32_t  err;
    uint32_t mask = enable > 0 ? 0 : 1;
    uint32_t masks = 0;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    #ifdef LARK_SDK 
    LARK_INVALID_PARAM_RETURN(index == 0 || index > 4);
    if ((irq_src & 0x0000007f) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK1_ADDR + (index - 1) * 5, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_DAC0_CLIP) > 0)        ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ADC0_CLIP) > 0)        ? ((masks & 0xfd) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ADC1_CLIP) > 0)        ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ADC2_CLIP) > 0)        ? ((masks & 0xf7) | (mask << 3)) : masks;
        masks = ((irq_src & API_LARK_IRQ_PLL_LOCKED) > 0)       ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_PLL_UNLOCKED) > 0)     ? ((masks & 0xdf) | (mask << 5)) : masks;
        masks = ((irq_src & API_LARK_IRQ_AVDD_UVW) > 0)         ? ((masks & 0xbf) | (mask << 6)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK1_ADDR + (index - 1) * 5, masks);
        LARK_ERROR_RETURN(err);
    }
    if ((irq_src & 0x00003f80) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK2_ADDR + (index - 1) * 5, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_PRAMP) > 0)            ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_FDSP) > 0)             ? ((masks & 0xfd) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_EQ_CLR_DONE) > 0)      ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCI_LOCKED) > 0)     ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCI_UNLOCKED) > 0)   ? ((masks & 0xdf) | (mask << 5)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCO_LOCKED) > 0)     ? ((masks & 0xbf) | (mask << 6)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCO_UNLOCKED) > 0)   ? ((masks & 0x7f) | (mask << 7)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK2_ADDR + (index - 1) * 5, masks);
        LARK_ERROR_RETURN(err);
    }
    if ((irq_src & 0x000fc000) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK3_ADDR + (index - 1) * 5, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_SYNC_LOCKED) > 0)      ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SYNC_UNLOCKED) > 0)    ? ((masks & 0xfd) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SPT0_UNLOCKED) > 0)    ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SPI1_UNLOCKED) > 0)    ? ((masks & 0xf7) | (mask << 3)) : masks;
        masks = ((irq_src & API_LARK_IRQ_POWERUP_COMPLETE) > 0) ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_DLDO_DVS_DONE) > 0)    ? ((masks & 0xdf) | (mask << 5)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK3_ADDR + (index - 1) * 5, masks);
        LARK_ERROR_RETURN(err);
    }
    if ((irq_src & 0x03f00000) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK4_ADDR + (index - 1) * 5, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_SW_INT0) > 0)          ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SW_INT1) > 0)          ? ((masks & 0xfd) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SW_INT2) > 0)          ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SW_INT3) > 0)          ? ((masks & 0xf7) | (mask << 3)) : masks;
        masks = ((irq_src & API_LARK_IRQ_DATA_SYNC) > 0)        ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_FFSRAM2) > 0)          ? ((masks & 0xdf) | (mask << 5)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK4_ADDR + (index - 1) * 5, masks);
        LARK_ERROR_RETURN(err);
    }
    if ((irq_src & 0x3c000000) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK5_ADDR + (index - 1) * 5, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_TDSP_ERR) > 0)         ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_AHB_ERR) > 0)          ? ((masks & 0xfd) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_AHB_BUSY) > 0)         ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_WDT) > 0)              ? ((masks & 0xf7) | (mask << 3)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK5_ADDR + (index - 1) * 5, masks);
        LARK_ERROR_RETURN(err);
    }
    #endif

    #ifdef LARK_LITE_SDK 
    LARK_INVALID_PARAM_RETURN(index == 0 || index > 2);
    if ((irq_src & 0x0000000f) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK1_ADDR + (index - 1) * 3, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_DAC0_CLIP) > 0)        ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ADC0_CLIP) > 0)        ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ADC1_CLIP) > 0)        ? ((masks & 0xdf) | (mask << 5)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ADC2_CLIP) > 0)        ? ((masks & 0xbf) | (mask << 6)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK1_ADDR + (index - 1) * 3, masks);
        LARK_ERROR_RETURN(err);
    }
    if ((irq_src & 0x00003cf0) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK2_ADDR + (index - 1) * 3, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_PLL_LOCKED) > 0)       ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_PLL_UNLOCKED) > 0)     ? ((masks & 0xfe) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_AVDD_UVW) > 0)         ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_PRAMP) > 0)            ? ((masks & 0xf7) | (mask << 3)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCI_LOCKED) > 0)     ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCI_UNLOCKED) > 0)   ? ((masks & 0xdf) | (mask << 5)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCO_LOCKED) > 0)     ? ((masks & 0xbf) | (mask << 6)) : masks;
        masks = ((irq_src & API_LARK_IRQ_ASRCO_UNLOCKED) > 0)   ? ((masks & 0x7f) | (mask << 7)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK2_ADDR + (index - 1) * 3, masks);
        LARK_ERROR_RETURN(err);
    }
    if ((irq_src & 0x0005c300) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_IRQ1_MASK3_ADDR + (index - 1) * 3, &masks);
        LARK_ERROR_RETURN(err);
        masks = ((irq_src & API_LARK_IRQ_SPT0_UNLOCKED) > 0)    ? ((masks & 0xfe) | (mask << 0)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SYNC_LOCKED) > 0)      ? ((masks & 0xfd) | (mask << 1)) : masks;
        masks = ((irq_src & API_LARK_IRQ_SYNC_UNLOCKED) > 0)    ? ((masks & 0xfb) | (mask << 2)) : masks;
        masks = ((irq_src & API_LARK_IRQ_POWERUP_COMPLETE) > 0) ? ((masks & 0xef) | (mask << 4)) : masks;
        masks = ((irq_src & API_LARK_IRQ_EQ_CLR_DONE) > 0)      ? ((masks & 0xdf) | (mask << 5)) : masks;
        masks = ((irq_src & API_LARK_IRQ_FDSP) > 0)             ? ((masks & 0xbf) | (mask << 6)) : masks;
        err = adi_lark_hal_reg_write(device, REG_IRQ1_MASK3_ADDR + (index - 1) * 3, masks);
        LARK_ERROR_RETURN(err);
    }
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_get_irq_status(adi_lark_device_t *device, uint8_t index, adi_lark_irq_src_e irq_src, uint8_t *status)
{
    int32_t  err = 0;
    uint32_t irq_status = 0;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    #ifdef LARK_SDK 
    LARK_INVALID_PARAM_RETURN(index == 0 || index > 4);
    #endif
    #ifdef LARK_LITE_SDK
    LARK_INVALID_PARAM_RETURN(index == 0 || index > 2);
    #endif
    switch(irq_src)
    {
        case API_LARK_IRQ_DAC0_CLIP:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_DAC0_CLIP_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ADC0_CLIP:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ADC0_CLIP_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ADC1_CLIP:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ADC1_CLIP_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ADC2_CLIP:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ADC2_CLIP_INFO, &irq_status);
            break;
        case API_LARK_IRQ_PLL_LOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_PLL_LOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_PLL_UNLOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_PLL_UNLOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_AVDD_UVW:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_AVDD_UVW_INFO, &irq_status);
            break;
        case API_LARK_IRQ_PRAMP:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_PRAMP_INFO, &irq_status);
            break;
        case API_LARK_IRQ_FDSP:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_FDSP_MASK_ONZ_INFO, &irq_status);
            break;
        case API_LARK_IRQ_EQ_CLR_DONE:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_EQ_CLEAR_DONE_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ASRCI_LOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ASRCI_LOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ASRCI_UNLOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ASRCI_UNLOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ASRCO_LOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ASRCO_LOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_ASRCO_UNLOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_ASRCO_UNLOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SYNC_LOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SYNC_LOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SYNC_UNLOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SYNC_UNLOCKED_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SPT0_UNLOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SPT0_UNLOCKED_INFO, &irq_status);
            break;
        #ifdef LARK_SDK
        case API_LARK_IRQ_SPI1_UNLOCKED:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SPT1_UNLOCKED_INFO, &irq_status);
            break;
        #endif
        case API_LARK_IRQ_POWERUP_COMPLETE:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_POWER_UP_COMPLETE_INFO, &irq_status);
            break;
        #ifdef LARK_SDK
        case API_LARK_IRQ_DLDO_DVS_DONE:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_DLDO_DVS_DONE_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SW_INT0:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SW_INT0_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SW_INT1:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SW_INT1_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SW_INT2:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SW_INT2_INFO, &irq_status);
            break;
        case API_LARK_IRQ_SW_INT3:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_SW_INT3_INFO, &irq_status);
            break;
        case API_LARK_IRQ_DATA_SYNC:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_DATA_SYNC_IRQ_INFO, &irq_status);
            break;
        case API_LARK_IRQ_FFSRAM2:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_FFSRAM2_IRQ_INFO, &irq_status);
            break;
        case API_LARK_IRQ_TDSP_ERR:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_TDSP_ERR_INFO, &irq_status);
            break;
        case API_LARK_IRQ_AHB_ERR:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_AHB_ERR_INFO, &irq_status);
            break;
        case API_LARK_IRQ_AHB_BUSY:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_AHB_BUSY_INFO, &irq_status);
            break;
        case API_LARK_IRQ_WDT:
            err = adi_lark_hal_bf_read(device, BF_IRQ1_WDT_IRQ_INFO, &irq_status);
            break;
        #endif
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    *status = (uint8_t)irq_status;

    return err;
}

int32_t adi_lark_int_trig_tdsp_sw_irq(adi_lark_device_t *device, uint8_t index)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    switch(index)
    {
        case 1:
            err = adi_lark_hal_bf_write(device, BF_SW_INT0_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 2:
            err = adi_lark_hal_bf_write(device, BF_SW_INT1_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 3:
            err = adi_lark_hal_bf_write(device, BF_SW_INT2_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 4:
            err = adi_lark_hal_bf_write(device, BF_SW_INT3_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    #endif /*LARK_SDK*/
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_clr_mp_input_irq(adi_lark_device_t *device, uint8_t mp_in_irq_index)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    switch(mp_in_irq_index)
    {
        case 1:
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ1_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 2:
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ2_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        case 3:
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ3_CLEAR_INFO, 1);
            LARK_ERROR_RETURN(err);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    #endif /*#ifdef LARK_SDK*/
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_set_mp_input_irq_mode(adi_lark_device_t* device, uint8_t mp_in_irq_index, uint8_t mode)
{
    #ifdef LARK_SDK
    int32_t err;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    
    switch(mp_in_irq_index)
    {
        case 1:
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ1_MODE_INFO, mode);
            LARK_ERROR_RETURN(err);
            break;
        case 2:
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ2_MODE_INFO, mode);
            LARK_ERROR_RETURN(err);
            break;
        case 3:
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ3_MODE_INFO, mode);
            LARK_ERROR_RETURN(err);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    #endif /*#ifdef LARK_SDK*/
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_enable_mp_input_irq(adi_lark_device_t* device, uint8_t mp_in_irq_index, uint32_t mp_in_irq, uint8_t enable)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t masks = 0;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    switch(mp_in_irq_index)
    {
        case 1:
            err = adi_lark_hal_bf_read(device, BF_MP_INPUT_IRQ1_MASK_INFO, &masks);
            LARK_ERROR_RETURN(err);
            masks = (masks & ~(mp_in_irq)) | (enable > 0 ? 0x00000000 : (mp_in_irq));
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ1_MASK_INFO, masks);
            LARK_ERROR_RETURN(err);
            break;
        case 2:
            err = adi_lark_hal_bf_read(device, BF_MP_INPUT_IRQ2_MASK_INFO, &masks);
            LARK_ERROR_RETURN(err);
            masks = (masks & ~(mp_in_irq)) | (enable > 0 ? 0x00000000 : (mp_in_irq));
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ2_MASK_INFO, masks);
            LARK_ERROR_RETURN(err);
            break;
        case 3:
            err = adi_lark_hal_bf_read(device, BF_MP_INPUT_IRQ3_MASK_INFO, &masks);
            LARK_ERROR_RETURN(err);
            masks = (masks & ~(mp_in_irq)) | (enable > 0 ? 0x00000000 : (mp_in_irq));
            err = adi_lark_hal_bf_write(device, BF_MP_INPUT_IRQ3_MASK_INFO, masks);
            LARK_ERROR_RETURN(err);
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    #endif /*#ifdef LARK_SDK*/
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_get_mp_input_irq_status(adi_lark_device_t* device, uint8_t mp_in_irq_index, uint32_t mp_in_bit_index, uint8_t *status)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t mp_status;
    
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    switch(mp_in_irq_index)
    {
        case 1:
            err = adi_lark_hal_bf_read(device, BF_MP_INPUT_IRQ1_STATUS_INFO, &mp_status);
            LARK_ERROR_RETURN(err);
            *status = (mp_status >> mp_in_bit_index) & 0x01;
            break;
        case 2:
            err = adi_lark_hal_bf_read(device, BF_MP_INPUT_IRQ2_STATUS_INFO, &mp_status);
            LARK_ERROR_RETURN(err);
            *status = (mp_status >> mp_in_bit_index) & 0x01;
            break;
        case 3:
            err = adi_lark_hal_bf_read(device, BF_MP_INPUT_IRQ3_IRQ_STATUS_INFO, &mp_status);
            LARK_ERROR_RETURN(err);
            *status = (mp_status >> mp_in_bit_index) & 0x01;
            break;
        default:
            return API_LARK_ERROR_INVALID_PARAM;
    }
    #endif
    
    return API_LARK_ERROR_OK;
}

int32_t adi_lark_int_get_soc_irq_id(adi_lark_device_t* device, adi_lark_soc_irq_src_e irq, uint32_t *id)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t i, j, lo_pri_mask, hi_pri_mask;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();

    if ((irq & LEVEL_INT_MASK) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_SOC_LOPRI_LVLINT_CON_ADDR, &lo_pri_mask);
        LARK_ERROR_RETURN(err);
        for (i = 0; i <= 14; i+=1)
        {
            if ((irq & lo_pri_mask & (1 << i)) > 0)
            {
                *id = i;
                return API_LARK_ERROR_OK;
            }
        }
        err = adi_lark_hal_reg_read(device, REG_SOC_HIPRI_LVLINT_CON_ADDR, &hi_pri_mask);
        LARK_ERROR_RETURN(err);
        for (i = 0; i <= 28; i+=4)
        {
            j = (hi_pri_mask >> i) & 0xf;
            if ((irq & (1 << j)) > 0)   /* interrupt is in high priority list */
            {
                *id = 16 + (i >> 2);
                return API_LARK_ERROR_OK;
            }
        }
    }
    if ((irq & EDGE_INT_MASK) > 0)
    {
        err = adi_lark_hal_reg_read(device, REG_SOC_LOPRI_EDGINT_CON_ADDR, &lo_pri_mask);
        LARK_ERROR_RETURN(err);
        irq = irq >> EDGE_INT_SHIFT;
        lo_pri_mask = lo_pri_mask >> EDGE_INT_SHIFT;
        for (i = 0; i <= 1; i+=1)
        {
            if ((irq & lo_pri_mask & (1 << i)) > 0)
            {
                *id = 24 + i;
                return API_LARK_ERROR_OK;
            }
        }
        err = adi_lark_hal_reg_read(device, REG_SOC_HIPRI_EDGINT_CON_ADDR, &hi_pri_mask);
        LARK_ERROR_RETURN(err);
        for (i = 0; i <= 4; i+=4)
        {
            j = (hi_pri_mask >> i) & 0xf;
            if ((irq & (1 << j)) > 0)   /* interrupt is in high priority list */
            {
                *id = 26 + (i >> 2);
                return API_LARK_ERROR_OK;
            }
        }
    }
    #endif /*#ifdef LARK_SDK*/

    return API_LARK_ERROR_ERROR;
}

int32_t adi_lark_int_set_soc_irq_priority(adi_lark_device_t* device, adi_lark_soc_irq_src_e irq, adi_lark_soc_irq_pri_e pri)
{
    #ifdef LARK_SDK
    int32_t  err;
    uint32_t i, j, k, lo_pri_mask, hi_pri_mask;
    LARK_NULL_POINTER_RETURN(device);
    LARK_LOG_FUNC();
    LARK_INVALID_PARAM_RETURN(pri > API_LARK_SOC_IRQ_PRI_HIGH);
    
    if (pri == API_LARK_SOC_IRQ_PRI_LOW)
    {
        if ((irq & LEVEL_INT_MASK) > 0)
        {
            /* enable low priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_LOPRI_LVLINT_CON_ADDR, &lo_pri_mask);
            LARK_ERROR_RETURN(err);
            lo_pri_mask |= irq & LEVEL_INT_MASK;
            err = adi_lark_hal_reg_write(device, REG_SOC_LOPRI_LVLINT_CON_ADDR, lo_pri_mask);
            LARK_ERROR_RETURN(err);
            /* clear high priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_HIPRI_LVLINT_CON_ADDR, &hi_pri_mask);
            LARK_ERROR_RETURN(err);
            for (i = 0; i <= 28; i+=4)
            {
                j = (hi_pri_mask >> i) & 0xf; /* interrupt id in high priority list */
                hi_pri_mask = (irq & (1 << j)) > 0 ? hi_pri_mask | (0xf << i) : hi_pri_mask;
            }
            err = adi_lark_hal_reg_write(device, REG_SOC_HIPRI_LVLINT_CON_ADDR, hi_pri_mask);
            LARK_ERROR_RETURN(err);
        }
        if ((irq & EDGE_INT_MASK) > 0)
        {
            /* enable low priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_LOPRI_EDGINT_CON_ADDR, &lo_pri_mask);
            LARK_ERROR_RETURN(err);
            lo_pri_mask |= irq & EDGE_INT_MASK;
            err = adi_lark_hal_reg_write(device, REG_SOC_LOPRI_EDGINT_CON_ADDR, lo_pri_mask);
            LARK_ERROR_RETURN(err);
            /* clear high priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_HIPRI_EDGINT_CON_ADDR, &hi_pri_mask);
            LARK_ERROR_RETURN(err);
            irq = irq >> EDGE_INT_SHIFT;
            for (i = 0; i <= 4; i+=4)
            {
                hi_pri_mask = (irq & 0x1) > 0 ? hi_pri_mask | (0xf << i) : hi_pri_mask;
                irq = irq >> 1;
            }
            err = adi_lark_hal_reg_write(device, REG_SOC_HIPRI_EDGINT_CON_ADDR, hi_pri_mask);
            LARK_ERROR_RETURN(err);
        }
    }
    if (pri == API_LARK_SOC_IRQ_PRI_HIGH)
    {
        if ((irq & LEVEL_INT_MASK) > 0)
        {
            /* clear low priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_LOPRI_LVLINT_CON_ADDR, &lo_pri_mask);
            LARK_ERROR_RETURN(err);
            lo_pri_mask &= ~(irq & LEVEL_INT_MASK);
            /* set high priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_HIPRI_LVLINT_CON_ADDR, &hi_pri_mask);
            LARK_ERROR_RETURN(err);
            for (i = 0; i <= 28; i+=4)
            { 
                j = (hi_pri_mask >> i) & 0xf;                                                           /* interrupt id in high priority list */
                hi_pri_mask = (irq & (1 << j)) > 0 ? hi_pri_mask | (0xf << i) : hi_pri_mask;            /* clear bit if it's already in high-priority list */
            }
            for (i = 0; i <= 28; i+=4)                                                                  /* find empty high priority position */
            {
                if ((irq & LEVEL_INT_MASK) == 0)
                    break;
                j = (hi_pri_mask >> i) & 0xf;                                                           /* interrupt id in high priority list */
                if (j == 0xf)                                                                           /* empty slot */
                {
                    for (k = 0; k <= 14; k++) 
                    {
                        if ((irq & (1 << k)) > 0)                                                       /* find which irq should be set */
                        {
                            irq = irq & ~(1 << k);
                            break;
                        }
                    }
                    hi_pri_mask = (k <= 14) ? hi_pri_mask & ~(0xf << i) | (k << i) : hi_pri_mask;
                }
            }
            if ((irq & LEVEL_INT_MASK) > 0)
            {
                LARK_LOG_ERR("unable to change new priority to high as all high priority locations are full.");
                return API_LARK_ERROR_INVALID_PARAM;
            }
            err = adi_lark_hal_reg_write(device, REG_SOC_LOPRI_LVLINT_CON_ADDR, lo_pri_mask);
            LARK_ERROR_RETURN(err);
            err = adi_lark_hal_reg_write(device, REG_SOC_HIPRI_LVLINT_CON_ADDR, hi_pri_mask);
            LARK_ERROR_RETURN(err);
        }
        if ((irq & EDGE_INT_MASK) > 0)
        {
            /* clear low priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_LOPRI_EDGINT_CON_ADDR, &lo_pri_mask);
            LARK_ERROR_RETURN(err);
            lo_pri_mask &= ~(irq & EDGE_INT_MASK);
            err = adi_lark_hal_reg_write(device, REG_SOC_LOPRI_EDGINT_CON_ADDR, lo_pri_mask);
            LARK_ERROR_RETURN(err);
            /* set high priority bits */
            err = adi_lark_hal_reg_read(device, REG_SOC_HIPRI_EDGINT_CON_ADDR, &hi_pri_mask);
            LARK_ERROR_RETURN(err);
            irq = irq >> EDGE_INT_SHIFT;
            for (i = 0; i <= 4; i+=4)
            {
                hi_pri_mask = (irq & 0x1) > 0 ? hi_pri_mask & ~(0xf << i) | ((i>>2) << i) : hi_pri_mask;
                irq = irq >> 1;
            }
            err = adi_lark_hal_reg_write(device, REG_SOC_HIPRI_EDGINT_CON_ADDR, hi_pri_mask);
            LARK_ERROR_RETURN(err);
        }
    }
    #endif /*#ifdef LARK_SDK*/
    
    return API_LARK_ERROR_OK;
}

/*! @} */
