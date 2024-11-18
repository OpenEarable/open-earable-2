/**
* Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bma580_context.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA580_CONTEXT_H
#define _BMA580_CONTEXT_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/

#include "bma580.h"

/******************************************************************************/
/********************* Macros for context parameters **************************/
/******************************************************************************/
#define BMA580_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_S       UINT16_C(15)
#define BMA580_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_W       UINT16_C(13)
#define BMA580_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_H       UINT16_C(12)

#define BMA580_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_S        UINT16_C(3)
#define BMA580_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_W        UINT16_C(2)
#define BMA580_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_H        UINT16_C(4)

#define BMA580_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_S       UINT16_C(15)
#define BMA580_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_W       UINT16_C(12)
#define BMA580_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_H       UINT16_C(8)

#define BMA580_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_S        UINT16_C(3)
#define BMA580_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_W        UINT16_C(2)
#define BMA580_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_H        UINT16_C(1)

#define BMA580_TAP_DETECTOR_1_AXIS_SEL_S                    UINT16_C(2)
#define BMA580_TAP_DETECTOR_1_AXIS_SEL_W                    UINT16_C(2)
#define BMA580_TAP_DETECTOR_1_AXIS_SEL_H                    UINT16_C(2)

#define BMA580_TAP_DETECTOR_1_WAIT_FOR_TIMEOUT_S            UINT16_C(1)
#define BMA580_TAP_DETECTOR_1_WAIT_FOR_TIMEOUT_W            UINT16_C(1)
#define BMA580_TAP_DETECTOR_1_WAIT_FOR_TIMEOUT_H            UINT16_C(1)

#define BMA580_TAP_DETECTOR_1_MAX_PEAKS_FOR_TAP_S           UINT16_C(6)
#define BMA580_TAP_DETECTOR_1_MAX_PEAKS_FOR_TAP_W           UINT16_C(6)
#define BMA580_TAP_DETECTOR_1_MAX_PEAKS_FOR_TAP_H           UINT16_C(6)

#define BMA580_TAP_DETECTOR_1_MODE_S                        UINT16_C(1)
#define BMA580_TAP_DETECTOR_1_MODE_W                        UINT16_C(2)
#define BMA580_TAP_DETECTOR_1_MODE_H                        UINT16_C(1)

#define BMA580_TAP_DETECTOR_2_TAP_PEAK_THRES_S              UINT16_C(143)
#define BMA580_TAP_DETECTOR_2_TAP_PEAK_THRES_W              UINT16_C(250)
#define BMA580_TAP_DETECTOR_2_TAP_PEAK_THRES_H              UINT16_C(750)

#define BMA580_TAP_DETECTOR_2_MAX_GESTURE_DUR_S             UINT16_C(25)
#define BMA580_TAP_DETECTOR_2_MAX_GESTURE_DUR_W             UINT16_C(25)
#define BMA580_TAP_DETECTOR_2_MAX_GESTURE_DUR_H             UINT16_C(25)

#define BMA580_TAP_DETECTOR_3_MAX_DUR_BETWEEN_PEAKS_S       UINT16_C(4)
#define BMA580_TAP_DETECTOR_3_MAX_DUR_BETWEEN_PEAKS_W       UINT16_C(4)
#define BMA580_TAP_DETECTOR_3_MAX_DUR_BETWEEN_PEAKS_H       UINT16_C(4)

#define BMA580_TAP_DETECTOR_3_TAP_SHOCK_SETTLING_DUR_S      UINT16_C(6)
#define BMA580_TAP_DETECTOR_3_TAP_SHOCK_SETTLING_DUR_W      UINT16_C(6)
#define BMA580_TAP_DETECTOR_3_TAP_SHOCK_SETTLING_DUR_H      UINT16_C(6)

#define BMA580_TAP_DETECTOR_3_MIN_QUITE_DUR_BETWEEN_TAPS_S  UINT16_C(8)
#define BMA580_TAP_DETECTOR_3_MIN_QUITE_DUR_BETWEEN_TAPS_W  UINT16_C(8)
#define BMA580_TAP_DETECTOR_3_MIN_QUITE_DUR_BETWEEN_TAPS_H  UINT16_C(8)

#define BMA580_TAP_DETECTOR_3_QUITE_TIME_AFTER_GESTURE_S    UINT16_C(6)
#define BMA580_TAP_DETECTOR_3_QUITE_TIME_AFTER_GESTURE_W    UINT16_C(6)
#define BMA580_TAP_DETECTOR_3_QUITE_TIME_AFTER_GESTURE_H    UINT16_C(6)

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMA580_CONTEXT_H */
