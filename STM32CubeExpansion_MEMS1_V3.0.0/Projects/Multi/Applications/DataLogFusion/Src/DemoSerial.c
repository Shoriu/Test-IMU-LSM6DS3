/**
  ******************************************************************************
  * @file    Projects/Multi/Applications/DataLogFusion/Src/DemoSerial.c
  * @author  CL
  * @version V1.6.0
  * @date    8-November-2016
  * @brief   Handler AST Serial Protocol
  ******************************************************************************
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
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "DemoSerial.h"
#include "MotionFX_Manager.h"
#include "main.h"
#include "com.h"

/** @addtogroup OSX_MOTION_FX_Applications
  * @{
  */

/** @addtogroup DATALOGFUSION
  * @{
  */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint32_t Sensors_Enabled;
extern volatile uint32_t DataTxPeriod;
extern volatile uint8_t SF_Active;
extern volatile uint8_t SF_change;
extern volatile uint8_t SF_6x_enabled;
extern TIM_HandleTypeDef DataLogTimHandle;
extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *HUMIDITY_handle;
extern void *TEMPERATURE_handle;
extern void *PRESSURE_handle;

/* Private variables ---------------------------------------------------------*/
volatile uint8_t DataLoggerActive;
volatile uint8_t SenderInterface = 0;
#ifdef USE_IKS01A2
uint8_t PresentationString[] = {"MEMS shield demo,4,1.6.0,1.0.7,IKS01A2"};
#elif USE_IKS01A1
uint8_t PresentationString[] = {"MEMS shield demo,4,1.6.0,1.0.7,IKS01A1"};
#endif
volatile uint8_t DataStreamingDest = 1;


/**
  * @brief  Build the reply header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
}

/**
  * @brief  Build the nack header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_NACK;
}

/**
  * @brief  Initialize the streaming header
  * @param  Msg the pointer to the header to be initialized
  * @retval None
  */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  Msg->Len = 3;
}

/**
  * @brief  Initialize the streaming message
  * @param  Msg the pointer to the message to be initialized
  * @retval None
  */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint8_t i;
  
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  for(i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }
  Msg->Len = 3;
  
}

/**
  * @brief  Handle a message
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int HandleMSG(TMsg *Msg)
//  DestAddr | SouceAddr | CMD | PAYLOAD
//      1          1        1       N
{
  uint32_t i;
  uint8_t instance;
  
  if (Msg->Len < 2) return 0;
  if (Msg->Data[0] != DEV_ADDR) return 0;
  switch (Msg->Data[2])   // CMD
  {
  
    case CMD_Ping:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      return 1;
      
    case CMD_Read_PresString:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      i = 0; //
      while (i < (sizeof(PresentationString) - 1))
      {
        Msg->Data[3 + i] = PresentationString[i];
        i++;
      }
      
      Msg->Len = 3 + i;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_CheckModeSupport:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Serialize_s32(&Msg->Data[3], DATALOG_FUSION_MODE, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_PRESSURE_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
        
      BSP_PRESSURE_Get_Instance( PRESSURE_handle, &instance );
        
      switch (instance)
      {
#ifdef USE_IKS01A2
        case LPS22HB_P_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;
#elif USE_IKS01A1
        case LPS25HB_P_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;
        case LPS25HB_P_1:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;
        case LPS22HB_P_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;
#endif		  
        default:
          break;
      }
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_HUMIDITY_TEMPERATURE_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg ); 
      Serialize_s32(&Msg->Data[3], 1, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_ACCELERO_GYRO_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
      
      /* We can check one between accelerometer instance and gyroscope instance */
      BSP_GYRO_Get_Instance( GYRO_handle, &instance );
          
      switch (instance)
      {
#ifdef USE_IKS01A2
        case LSM6DSL_G_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;
#elif USE_IKS01A1
        case LSM6DS0_G_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;
        case LSM6DS3_G_0:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;
#endif
        default:
          break;
      }

      UART_SendMsg(Msg);
      return 1;
      
    case CMD_MAGNETO_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
      
      /* We can check magnetometer instance */
      BSP_MAGNETO_Get_Instance( MAGNETO_handle, &instance );
          
      switch (instance)
      {
#ifdef USE_IKS01A2
        case LSM303AGR_M_0:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;
#elif USE_IKS01A1
        case LIS3MDL_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;
#endif
        default:
          break;
      }

      UART_SendMsg(Msg);
      return 1;
      
    case CMD_Start_Data_Streaming:
      if (Msg->Len < 3) return 0;
      Sensors_Enabled = Deserialize(&Msg->Data[3], 4);
      DataTxPeriod = Deserialize(&Msg->Data[7], 4);
      DataLoggerActive = 1;
      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_Stop_Data_Streaming:
      if (Msg->Len < 3) return 0;
      Sensors_Enabled = 0;
      DataLoggerActive = 0;
      SF_Active = 0;
      HAL_TIM_OC_Stop(&DataLogTimHandle, TIM_CHANNEL_1);
      
      if(SF_6x_enabled == 1)
      {
        MotionFX_manager_stop_6X();
        MotionFX_manager_start_9X();
        SF_6x_enabled = 0;
      }
      
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      return 1;
    case CMD_ChangeSF:
      if (Msg->Len < 3) return 0;
      
      SF_change = 1;
      
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      return 1;
    case CMD_Set_DateTime:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_TimeRegulate(Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_SF_Init:
      if (Msg->Len < 3) return 0;
      DataStreamingDest = Msg->Data[1];
      DataLogTimerInit();
	  BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      SF_Active = 1;
      return 1;
      
    default:
      return 0;
  }
}

/**
 * @}
 */

/**
 * @}
 */
