/**
  *******************************************************************************
  * @file    Projects/Multi/Examples/DataLog/Inc/sensor_commands.h
  * @author  MB
  * @version V3.0.0
  * @date    12-August-2016
  * @brief   Header for sensor_commands.h
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __SENSOR_COMMANDS__
#define __SENSOR_COMMANDS__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "serial_protocol.h"

/* Exported defines ----------------------------------------------------------*/
#define SC_ACCELEROMETER                                                0x01
#define SC_GYROSCOPE                                                    0x02
#define SC_MAGNETOMETER                                                 0x03
#define SC_TEMPERATURE                                                  0x04
#define SC_HUMIDITY                                                     0x05
#define SC_PRESSURE                                                     0x06
#define SC_UV                                                           0x07

#define SC_GET_SENSOR_NAME                                              0x01
#define SC_READ_REGISTER                                                0x02
#define SC_WRITE_REGISTER                                               0x03
#define SC_GET_FULL_SCALE_LIST                                          0x04
#define SC_SET_FULL_SCALE                                               0x05
#define SC_GET_ODR_LIST                                                 0x06
#define SC_SET_ODR                                                      0x07

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
int SC_Get_Sensor_Name(TMsg *Msg);
int SC_Read_Register(TMsg *Msg);
int SC_Write_Register(TMsg *Msg);
int SC_Get_Full_Scale_List(TMsg *Msg);
int SC_Set_Full_Scale(TMsg *Msg);
int SC_Get_ODR_List(TMsg *Msg);
int SC_Set_ODR(TMsg *Msg);

void Send_Sensor_Name(TMsg *Msg, uint8_t* sensor_name);
void Send_Sensor_FS_List(TMsg *Msg, uint32_t* fs_list);
void Send_Sensor_ODR_List(TMsg *Msg, float* odr_list);

void FloatToArray(uint8_t *Dest, float data);
void ArrayToFloat(uint8_t *Source, float *data);

/* Exported functions ------------------------------------------------------- */
int Handle_Sensor_command(TMsg *Msg);

#endif /* __SENSOR_COMMANDS__ */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
