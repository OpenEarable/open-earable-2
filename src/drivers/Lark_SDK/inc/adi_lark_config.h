/*!
 * @brief     LARK/LARK-LITE Configuration Header
 * @copyright Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
 */
 
/*!
 * @addtogroup adi_lark_sdk
 * @{
 */

#ifndef __ADI_LARK_CONFIG_H__
#define __ADI_LARK_CONFIG_H__

/*============= D E F I N E S ==============*/
/*!< sdk control */
#if !defined(LARK_SDK) && !defined(LARK_LITE_SDK)
#define LARK_SDK
#endif

/*!< verbose log report control, bit0~15: level, bit16~31: content */
#define LARK_LOG_ERR_MSG       0x00000001       /*!< error message */
#define LARK_LOG_WARN_MSG      0x00000002       /*!< warning message */
#define LARK_LOG_INFO_MSG      0x00000004       /*!< info message */
#define LARK_LOG_MISC_MSG      0x00010000       /*!< miscellaneous message */ 
#define LARK_LOG_FUNC_MSG      0x00020000       /*!< function call message */
#define LARK_LOG_VAR_MSG       0x00040000       /*!< variable related message */
#define LARK_LOG_REG_MSG       0x00080000       /*!< register r/w  message */

#define LARK_LOG_NONE_MSG      0x00000000       /*!< all not selected */
#define LARK_LOG_ALL_MSG       0xffffffff       /*!< all selected */

#ifndef LARK_REPORT_VERBOSE
#define LARK_REPORT_VERBOSE    0                /*!< actual log control */
#endif

/*!< max buffer size that sdk is using internally for control port access */
#define LARK_SDK_MAX_BUFSIZE   16               /*!< buffer size sdk allocates for control port access */

/*!< enable test code or not */
#ifndef LARK_SDK_EN_TEST_CODE
#define LARK_SDK_EN_TEST_CODE  0                /*!< 1 to enable test code */
#endif

/*!< use hifi3z xos or xtos in sdk test code */
#ifndef LARK_SDK_XOS_TEST_CODE
#define LARK_SDK_XOS_TEST_CODE 0                /*!< 1 to use XOS, 0 to use XTOS */
#endif

#endif /* __ADI_LARK_CONFIG_H__ */

/*! @} */
