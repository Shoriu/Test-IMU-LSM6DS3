/**
  *******************************************************************************
  * @file    Projects/Multi/Applications/DataLogFusion/Src/MotionFX_Manager.c
  * @author  CL
  * @version V1.6.0
  * @date    8-November-2016
  * @brief   This file includes sensor fusion interface functions
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
  * ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MotionFX_Manager.h"
#include "osx_license.h"

/** @addtogroup OSX_MOTION_FX_Applications
  * @{
  */

/** @addtogroup DATALOGFUSION
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define FROM_MG_TO_G    0.001f
#define FROM_G_TO_MG    1000.0f
#define FROM_MDPS_TO_DPS    0.001f
#define FROM_DPS_TO_MDPS    1000.0f
#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define SAMPLETODISCARD 15
#define MOTIONFX_ENGINE_DELTATIME 0.01f
#define GBIAS_ACC_TH_SC_6X (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_6X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X (2.0f*0.001500f)
#define GBIAS_ACC_TH_SC_9X (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9X (2.0f*0.001500f)

/* Private types -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern SensorAxes_t ACC_Value;
extern SensorAxes_t GYR_Value;
extern SensorAxes_t MAG_Value;
extern void *GYRO_handle;
extern osxMFX_calibFactor magOffset;

/* Private variables ---------------------------------------------------------*/
osxMFX_knobs iKnobs;
osxMFX_knobs* ipKnobs;
volatile osxMFX_output iDataOUT;
volatile osxMFX_input iDataIN;
float go[3];
volatile int sampleToDiscard = SAMPLETODISCARD;
float MotionFX_engine_deltatime = MOTIONFX_ENGINE_DELTATIME;
int discardedCount = 0;

/**
  * @brief  Run sensor fusion algorithm
  * @param  None
  * @retval None
  */
void MotionFX_manager_run(void)
{
  iDataIN.gyro[0] = GYR_Value.AXIS_X  * FROM_MDPS_TO_DPS;
  iDataIN.gyro[1] = GYR_Value.AXIS_Y  * FROM_MDPS_TO_DPS;
  iDataIN.gyro[2] = GYR_Value.AXIS_Z  * FROM_MDPS_TO_DPS;
  
  iDataIN.acc[0] = ACC_Value.AXIS_X * FROM_MG_TO_G;
  iDataIN.acc[1] = ACC_Value.AXIS_Y * FROM_MG_TO_G;
  iDataIN.acc[2] = ACC_Value.AXIS_Z * FROM_MG_TO_G;
  
  iDataIN.mag[0] = (MAG_Value.AXIS_X - magOffset.magOffX) * FROM_MGAUSS_TO_UT50;
  iDataIN.mag[1] = (MAG_Value.AXIS_Y - magOffset.magOffY) * FROM_MGAUSS_TO_UT50;
  iDataIN.mag[2] = (MAG_Value.AXIS_Z - magOffset.magOffZ) * FROM_MGAUSS_TO_UT50;
  
  if(discardedCount == sampleToDiscard)
  {
    osx_MotionFX_propagate((osxMFX_output*)&iDataOUT, (osxMFX_input*)&iDataIN, MotionFX_engine_deltatime);
    
    osx_MotionFX_update((osxMFX_output*)&iDataOUT, (osxMFX_input*)&iDataIN, MotionFX_engine_deltatime, NULL);
  }
  else
  {
    discardedCount++;
  }
}


/**
  * @brief  Initialize MotionFX engine
  * @param  None
  * @retval None
  */
void MotionFX_manager_init(void)
{
  uint8_t instance;
  
  magOffset.magOffX = 0;
  magOffset.magOffY = 0;
  magOffset.magOffZ = 0;
  
  //  ST MotionFX Engine Initializations
  ipKnobs = &iKnobs;
  
  osx_MotionFX_initialize();
  
  osx_MotionFX_compass_init();
  
  osx_MotionFX_getKnobs(ipKnobs);
  
  ipKnobs->gbias_acc_th_sc_6X = GBIAS_ACC_TH_SC_6X;
  ipKnobs->gbias_gyro_th_sc_6X = GBIAS_GYRO_TH_SC_6X;
  ipKnobs->gbias_mag_th_sc_6X = GBIAS_MAG_TH_SC_6X;
  
  ipKnobs->gbias_acc_th_sc_9X = GBIAS_ACC_TH_SC_9X;
  ipKnobs->gbias_gyro_th_sc_9X = GBIAS_GYRO_TH_SC_9X;
  ipKnobs->gbias_mag_th_sc_9X = GBIAS_MAG_TH_SC_9X;
  
  BSP_GYRO_Get_Instance( GYRO_handle, &instance );
  
  switch(instance)
  {
#ifdef USE_IKS01A2
    case LSM6DSL_G_0:
    default:
      ipKnobs->acc_orientation[0] = 'n';
      ipKnobs->acc_orientation[1] = 'w';
      ipKnobs->acc_orientation[2] = 'u';
      
      ipKnobs->gyro_orientation[0] = 'n';
      ipKnobs->gyro_orientation[1] = 'w';
      ipKnobs->gyro_orientation[2] = 'u';
      break;
#elif USE_IKS01A1
    case LSM6DS3_G_0: /*Instance LSM6DS3*/
      ipKnobs->acc_orientation[0] = 'n';
      ipKnobs->acc_orientation[1] = 'w';
      ipKnobs->acc_orientation[2] = 'u';
      
      ipKnobs->gyro_orientation[0] = 'n';
      ipKnobs->gyro_orientation[1] = 'w';
      ipKnobs->gyro_orientation[2] = 'u';
      break;
    case LSM6DS0_G_0: /*Instance LSM6DS0*/
    default:
      ipKnobs->acc_orientation[0] = 'e';
      ipKnobs->acc_orientation[1] = 'n';
      ipKnobs->acc_orientation[2] = 'u';
      
      ipKnobs->gyro_orientation[0] = 'e';
      ipKnobs->gyro_orientation[1] = 'n';
      ipKnobs->gyro_orientation[2] = 'u';
      break;
#endif
  }

#ifdef USE_IKS01A2
  ipKnobs->mag_orientation[0] = 'n';
  ipKnobs->mag_orientation[1] = 'e';
  ipKnobs->mag_orientation[2] = 'u';
#elif USE_IKS01A1
  ipKnobs->mag_orientation[0] = 's';
  ipKnobs->mag_orientation[1] = 'e';
  ipKnobs->mag_orientation[2] = 'u';
#endif
  
  ipKnobs->output_type = OSXMFX_ENGINE_OUTPUT_ENU;
  
  ipKnobs->LMode = 1;
  
  ipKnobs->modx = 1;
  
  osx_MotionFX_setKnobs(ipKnobs);
  
  osx_MotionFX_enable_6X(OSXMFX_ENGINE_DISABLE);
  
  osx_MotionFX_enable_9X(OSXMFX_ENGINE_DISABLE);
  
  /* Number of Sample to Discard */
  sampleToDiscard = SAMPLETODISCARD;
  discardedCount = 0;
}

/**
 * @brief  Start 6 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_start_6X(void)
{
  sampleToDiscard = SAMPLETODISCARD;
  osx_MotionFX_enable_6X(OSXMFX_ENGINE_ENABLE);
}


/**
 * @brief  Stop 6 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_stop_6X(void)
{
  osx_MotionFX_enable_6X(OSXMFX_ENGINE_DISABLE);
}


/**
 * @brief  Start 9 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_start_9X(void)
{
  sampleToDiscard = SAMPLETODISCARD;
  osx_MotionFX_enable_9X(OSXMFX_ENGINE_ENABLE);
  osx_MotionFX_setGbias(go);
}


/**
 * @brief  Stop 9 axes MotionFX engine
 * @param  None
 * @retval None
 */
void MotionFX_manager_stop_9X(void)
{
  osx_MotionFX_getGbias(go);
  osx_MotionFX_enable_9X(OSXMFX_ENGINE_DISABLE);
}


/**
 * @brief  Get MotionFX engine data OUT
 * @param  None
 * @retval osxMFX_output pointer to data output
 */
osxMFX_output* MotionFX_manager_getDataOUT(void)
{
  return (osxMFX_output*)&iDataOUT;
}


/**
 * @brief  Get MotionFX Engine data IN
 * @param  None
 * @retval osxMFX_input pointer to data input
 */
osxMFX_input* MotionFX_manager_getDataIN(void)
{
  return (osxMFX_input*)&iDataIN;
}

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/

