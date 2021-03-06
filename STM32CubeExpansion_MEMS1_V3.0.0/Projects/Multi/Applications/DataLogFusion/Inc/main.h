/**
  *******************************************************************************
  * @file    Projects/Multi/Applications/DataLogFusion/Inc/main.h
  * @author  CL
  * @version V1.6.0
  * @date    8-November-2016
  * @brief   header for main.c.
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#ifdef USE_IKS01A2
#include "x_nucleo_iks01a2.h"
#include "x_nucleo_iks01a2_accelero.h"
#include "x_nucleo_iks01a2_gyro.h"
#include "x_nucleo_iks01a2_magneto.h"
#include "x_nucleo_iks01a2_pressure.h"
#include "x_nucleo_iks01a2_humidity.h"
#include "x_nucleo_iks01a2_temperature.h"
#elif USE_IKS01A1
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_temperature.h"
#endif


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/

/* Definition for TIMx clock resources */
#define TIMDataLog                           TIM2
#define TIMDataLog_CLK_ENABLE                __TIM2_CLK_ENABLE
#define TIMDataLog_CLK_DISABLE               __TIM2_CLK_DISABLE

/* Definition for TIMx's NVIC */
#define TIMDataLog_IRQn                      TIM2_IRQn
#define TIMDataLog_IRQHandler                TIM2_IRQHandler

// Enable sensor masks
#define PRESSURE_SENSOR                         0x00000001
#define TEMPERATURE_SENSOR                      0x00000002
#define HUMIDITY_SENSOR                         0x00000004
#define UV_SENSOR                               0x00000008  // for future use
#define ACCELEROMETER_SENSOR                    0x00000010
#define GYROSCOPE_SENSOR                        0x00000020
#define MAGNETIC_SENSOR                         0x00000040


/* Exported functions ------------------------------------------------------- */
uint32_t user_currentTimeGetTick(void);
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1);

void Error_Handler(void);
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);
void DataLogTimerInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

