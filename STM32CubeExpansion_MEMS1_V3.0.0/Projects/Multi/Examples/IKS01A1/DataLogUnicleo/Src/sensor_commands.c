/**
  ******************************************************************************
  * @file    Projects/Multi/Examples/IKS01A1/DataLogUnicleo/Src/sensor_commands.c
  * @author  MB
  * @version V3.0.0
  * @date    12-August-2016
  * @brief   Handle commands for sensor
  ******************************************************************************
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
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "sensor_commands.h"
#include "main.h"
#include "DemoSerial.h"
#include "com.h"

/** @addtogroup X_NUCLEO_IKS01A1_Examples
  * @{
  */

/** @addtogroup DATALOG_UNICLEO
  * @{
  */

/* Extern variables ----------------------------------------------------------*/
extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *HUMIDITY_handle;
extern void *TEMPERATURE_handle;
extern void *PRESSURE_handle;

/* Private variables ---------------------------------------------------------*/
uint8_t lsm6ds0[] = {"LSM6DS0"};
uint8_t lis3mdl[] = {"LIS3MDL"};
uint8_t hts221[] = {"HTS221"};
uint8_t lps25hb[] = {"LPS25HB"};
uint8_t lsm6ds3_dil[] = {"LSM6DS3 (DIL24)"};
uint8_t lis2dh12_dil[] = {"LIS2DH12 (DIL24)"};
uint8_t lps25hb_dil[] = {"LPS25HB (DIL24)"};
uint8_t lps22hb_dil[] = {"LPS22HB (DIL24)"};

uint32_t lsm6ds0_acc_fs[] = {4, 2, 4, 8, 16};      //g
uint32_t lsm6ds0_gyr_fs[] = {3, 245, 500, 2000};   //dps
float lsm6ds0_acc_odr[] = {6, 10, 50, 119, 238, 476, 952};
float lsm6ds0_gyr_odr[] = {6, 15, 60, 119, 238, 476, 952};

uint32_t lsm6ds3_acc_fs[] = {4, 2, 4, 8, 16};      //g
uint32_t lsm6ds3_gyr_fs[] = {4, 245, 500, 1000, 2000};   //dps
float lsm6ds3_acc_odr[] = {10, 13, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660};
float lsm6ds3_gyr_odr[] = {8, 13, 26, 52, 104, 208, 416, 833, 1660};

uint32_t lis3mdl_mag_fs[] = {4, 4, 8, 12, 16};     //Ga
float lis3mdl_mag_odr[] = {8, 0.625, 1.25, 2.5, 5, 10, 20, 40, 80};

uint32_t hts221_tem_fs[] = {0}; //C
uint32_t hts221_hum_fs[] = {0}; //%
float hts221_tem_odr[] = {3, 1, 7, 12.5};
float hts221_hum_odr[] = {3, 1, 7, 12.5};

uint32_t lps25hb_pre_fs[] = {0}; //Pa
uint32_t lps25hb_tem_fs[] = {0}; //C
float lps25hb_pre_odr[] = {4, 1, 7, 12.5, 25};
float lps25hb_tem_odr[] = {4, 1, 7, 12.5, 25};

uint32_t lis2dh12_acc_fs[] = {4, 2, 4, 8, 16};      //g
float lis2dh12_acc_odr[] = {7, 1, 10, 25, 50, 100, 200, 400};

uint32_t lps22hb_pre_fs[] = {0}; //Pa
uint32_t lps22hb_tem_fs[] = {0}; //C
float lps22hb_pre_odr[] = {5, 1, 10, 25, 50, 75};
float lps22hb_tem_odr[] = {5, 1, 10, 25, 50, 75};

/**
  * @brief  Handle Sensors command
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int Handle_Sensor_command(TMsg *Msg)
{
  /* Commands */
  switch (Msg->Data[3])
  {
    case SC_GET_SENSOR_NAME:
      return SC_Get_Sensor_Name(Msg);

    case SC_READ_REGISTER:
      return SC_Read_Register(Msg);

    case SC_WRITE_REGISTER:
      return SC_Write_Register(Msg);

    case SC_GET_FULL_SCALE_LIST:
      return SC_Get_Full_Scale_List(Msg);

    case SC_SET_FULL_SCALE:
      return SC_Set_Full_Scale(Msg);

    case SC_GET_ODR_LIST:
      return SC_Get_ODR_List(Msg);

    case SC_SET_ODR:
      return SC_Set_ODR(Msg);

    default:
      return 0;
  }
}

int SC_Get_Sensor_Name(TMsg *Msg)
{
  uint8_t instance;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Get_Instance( ACCELERO_handle, &instance );
      switch (instance)
      {
        case LSM6DS0_X_0:
          Send_Sensor_Name(Msg, lsm6ds0);
          return 1;

        case LSM6DS3_X_0:
          Send_Sensor_Name(Msg, lsm6ds3_dil);
          return 1;

        default:
          return 0;
      }

    case SC_GYROSCOPE:
      BSP_GYRO_Get_Instance( GYRO_handle, &instance );
      switch (instance)
      {
        case LSM6DS0_G_0:
          Send_Sensor_Name(Msg, lsm6ds0);
          return 1;

        case LSM6DS3_G_0:
          Send_Sensor_Name(Msg, lsm6ds3_dil);
          return 1;

        default:
          return 0;
      }

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Get_Instance( MAGNETO_handle, &instance );
      switch (instance)
      {
        case LIS3MDL_0:
          Send_Sensor_Name(Msg, lis3mdl);
          return 1;

        default:
          return 0;
      }

    case SC_TEMPERATURE:
      BSP_TEMPERATURE_Get_Instance( TEMPERATURE_handle, &instance );
      switch (instance)
      {
        case HTS221_T_0:
          Send_Sensor_Name(Msg, hts221);
          return 1;

        case LPS25HB_T_0:
          Send_Sensor_Name(Msg, lps25hb);
          return 1;

        case LPS25HB_T_1:
          Send_Sensor_Name(Msg, lps25hb_dil);
          return 1;

        case LPS22HB_T_0:
          Send_Sensor_Name(Msg, lps22hb_dil);
          return 1;

        default:
          return 0;
      }

    case SC_HUMIDITY:
      BSP_HUMIDITY_Get_Instance( HUMIDITY_handle, &instance );
      switch (instance)
      {
        case HTS221_H_0:
          Send_Sensor_Name(Msg, hts221);
          return 1;

        default:
          return 0;
      }

    case SC_PRESSURE:
      BSP_PRESSURE_Get_Instance( PRESSURE_handle, &instance );
      switch (instance)
      {
        case LPS25HB_P_0:
          Send_Sensor_Name(Msg, lps25hb);
          return 1;
        case LPS25HB_P_1:
          Send_Sensor_Name(Msg, lps25hb);
          return 1;
        case LPS22HB_P_0:
          Send_Sensor_Name(Msg, lps22hb_dil);
          return 1;
        default:
          return 0;
      }

    default:
      return 0;
  }
}

int SC_Read_Register(TMsg *Msg)
{
  uint8_t reg_value;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Read_Reg( ACCELERO_handle, Msg->Data[5], &reg_value );
      break;

    case SC_GYROSCOPE:
      BSP_GYRO_Read_Reg( GYRO_handle, Msg->Data[5], &reg_value );
      break;

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Read_Reg( MAGNETO_handle, Msg->Data[5],  &reg_value );
      break;

    case SC_TEMPERATURE:
      BSP_TEMPERATURE_Read_Reg( TEMPERATURE_handle, Msg->Data[5],  &reg_value );
      break;

    case SC_HUMIDITY:
      BSP_HUMIDITY_Read_Reg( HUMIDITY_handle, Msg->Data[5],  &reg_value );
      break;

    case SC_PRESSURE:
      BSP_PRESSURE_Read_Reg( PRESSURE_handle, Msg->Data[5],  &reg_value );
      break;

    default:
      return 0;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Data[6] = reg_value;
  Msg->Len = 7;
  UART_SendMsg(Msg);
  return 1;
}

int SC_Write_Register(TMsg *Msg)
{
  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Write_Reg( ACCELERO_handle, Msg->Data[5], Msg->Data[6] );
      break;

    case SC_GYROSCOPE:
      BSP_GYRO_Write_Reg( GYRO_handle, Msg->Data[5], Msg->Data[6] );
      break;

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Write_Reg( MAGNETO_handle, Msg->Data[5], Msg->Data[6] );
      break;

    case SC_TEMPERATURE:
      BSP_TEMPERATURE_Write_Reg( TEMPERATURE_handle, Msg->Data[5], Msg->Data[6] );
      break;

    case SC_HUMIDITY:
      BSP_HUMIDITY_Write_Reg( HUMIDITY_handle, Msg->Data[5], Msg->Data[6] );
      break;

    case SC_PRESSURE:
      BSP_PRESSURE_Write_Reg( PRESSURE_handle, Msg->Data[5], Msg->Data[6] );
      break;

    default:
      return 0;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 7;
  UART_SendMsg(Msg);
  return 1;
}

int SC_Get_Full_Scale_List(TMsg *Msg)
{
  uint8_t instance;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Get_Instance( ACCELERO_handle, &instance );
      switch (instance)
      {
        case LSM6DS0_X_0:
          Send_Sensor_FS_List(Msg, lsm6ds0_acc_fs);
          return 1;

        case LSM6DS3_X_0:
          Send_Sensor_FS_List(Msg, lsm6ds3_acc_fs);
          return 1;

        default:
          return 0;
      }

    case SC_GYROSCOPE:
      BSP_GYRO_Get_Instance( GYRO_handle, &instance );
      switch (instance)
      {
        case LSM6DS0_G_0:
          Send_Sensor_FS_List(Msg, lsm6ds0_gyr_fs);
          return 1;

        case LSM6DS3_G_0:
          Send_Sensor_FS_List(Msg, lsm6ds3_gyr_fs);
          return 1;

        default:
          return 0;
      }

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Get_Instance( MAGNETO_handle, &instance );
      switch (instance)
      {
        case LIS3MDL_0:
          Send_Sensor_FS_List(Msg, lis3mdl_mag_fs);
          return 1;

        default:
          return 0;
      }

    case SC_TEMPERATURE:
      BSP_TEMPERATURE_Get_Instance( TEMPERATURE_handle, &instance );
      switch (instance)
      {
        case HTS221_T_0:
          Send_Sensor_FS_List(Msg, hts221_tem_fs);
          return 1;

        case LPS25HB_T_0:
          Send_Sensor_FS_List(Msg, lps25hb_tem_fs);
          return 1;

        case LPS25HB_T_1:
          Send_Sensor_FS_List(Msg, lps25hb_tem_fs);
          return 1;

        case LPS22HB_T_0:
          Send_Sensor_FS_List(Msg, lps22hb_tem_fs);
          return 1;

        default:
          return 0;
      }

    case SC_HUMIDITY:
      BSP_HUMIDITY_Get_Instance( HUMIDITY_handle, &instance );
      switch (instance)
      {
        case HTS221_H_0:
          Send_Sensor_FS_List(Msg, hts221_hum_fs);
          return 1;

        default:
          return 0;
      }

    case SC_PRESSURE:
      BSP_PRESSURE_Get_Instance( PRESSURE_handle, &instance );
      switch (instance)
      {
        case LPS25HB_P_0:
          Send_Sensor_FS_List(Msg, lps25hb_pre_fs);
          return 1;
        case LPS25HB_P_1:
          Send_Sensor_FS_List(Msg, lps25hb_pre_fs);
          return 1;
        case LPS22HB_P_0:
          Send_Sensor_FS_List(Msg, lps22hb_pre_fs);
          return 1;
        default:
          return 0;
      }

    default:
      return 0;
  }
}

int SC_Set_Full_Scale(TMsg *Msg)
{
  uint32_t full_scale;

  full_scale = Deserialize(&Msg->Data[5], 4);

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Set_FS_Value( ACCELERO_handle, full_scale );
      break;

    case SC_GYROSCOPE:
      BSP_GYRO_Set_FS_Value( GYRO_handle, full_scale );
      break;

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Set_FS_Value( MAGNETO_handle, full_scale );
      break;

    case SC_TEMPERATURE:
      //BSP_TEMPERATURE_Set_FS_Value( TEMPERATURE_handle, full_scale );
      break;

    case SC_HUMIDITY:
      //BSP_HUMIDITY_Set_FS_Value( HUMIDITY_handle, full_scale );
      break;

    case SC_PRESSURE:
    //BSP_PRESSURE_Set_FS_Value( PRESSURE_handle, full_scale );

    default:
      return 0;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 9;
  UART_SendMsg(Msg);
  return 1;
}

int SC_Get_ODR_List(TMsg *Msg)
{
  uint8_t instance;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Get_Instance( ACCELERO_handle, &instance );
      switch (instance)
      {
        case LSM6DS0_X_0:
          Send_Sensor_ODR_List(Msg, lsm6ds0_acc_odr);
          return 1;

        case LSM6DS3_X_0:
          Send_Sensor_ODR_List(Msg, lsm6ds3_acc_odr);
          return 1;

        default:
          return 0;
      }

    case SC_GYROSCOPE:
      BSP_GYRO_Get_Instance( GYRO_handle, &instance );
      switch (instance)
      {
        case LSM6DS0_G_0:
          Send_Sensor_ODR_List(Msg, lsm6ds0_gyr_odr);
          return 1;

        case LSM6DS3_G_0:
          Send_Sensor_ODR_List(Msg, lsm6ds3_gyr_odr);
          return 1;

        default:
          return 0;
      }

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Get_Instance( MAGNETO_handle, &instance );
      switch (instance)
      {
        case LIS3MDL_0:
          Send_Sensor_ODR_List(Msg, lis3mdl_mag_odr);
          return 1;

        default:
          return 0;
      }

    case SC_TEMPERATURE:
      BSP_TEMPERATURE_Get_Instance( TEMPERATURE_handle, &instance );
      switch (instance)
      {
        case HTS221_T_0:
          Send_Sensor_ODR_List(Msg, hts221_tem_odr);
          return 1;

        case LPS25HB_T_0:
          Send_Sensor_ODR_List(Msg, lps25hb_tem_odr);
          return 1;

        case LPS25HB_T_1:
          Send_Sensor_ODR_List(Msg, lps25hb_tem_odr);
          return 1;

        case LPS22HB_T_0:
          Send_Sensor_ODR_List(Msg, lps22hb_tem_odr);
          return 1;

        default:
          return 0;
      }

    case SC_HUMIDITY:
      BSP_HUMIDITY_Get_Instance( HUMIDITY_handle, &instance );
      switch (instance)
      {
        case HTS221_H_0:
          Send_Sensor_ODR_List(Msg, hts221_hum_odr);
          return 1;

        default:
          return 0;
      }

    case SC_PRESSURE:
      BSP_PRESSURE_Get_Instance( PRESSURE_handle, &instance );
      switch (instance)
      {
        case LPS25HB_P_0:
          Send_Sensor_ODR_List(Msg, lps25hb_pre_odr);
          return 1;
        case LPS25HB_P_1:
          Send_Sensor_ODR_List(Msg, lps25hb_pre_odr);
          return 1;
        case LPS22HB_P_0:
          Send_Sensor_ODR_List(Msg, lps22hb_pre_odr);
          return 1;
        default:
          return 0;
      }

    default:
      return 0;
  }
}

int SC_Set_ODR(TMsg *Msg)
{
  float odr;

  ArrayToFloat(&Msg->Data[5], &odr);

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      BSP_ACCELERO_Set_ODR_Value( ACCELERO_handle, odr );
      break;

    case SC_GYROSCOPE:
      BSP_GYRO_Set_ODR_Value( GYRO_handle, odr );
      break;

    case SC_MAGNETOMETER:
      BSP_MAGNETO_Set_ODR_Value( MAGNETO_handle, odr );
      break;

    case SC_TEMPERATURE:
      BSP_TEMPERATURE_Set_ODR_Value( TEMPERATURE_handle, odr );
      break;

    case SC_HUMIDITY:
      BSP_HUMIDITY_Set_ODR_Value( HUMIDITY_handle, odr );
      break;

    case SC_PRESSURE:
      BSP_PRESSURE_Set_ODR_Value( PRESSURE_handle, odr );

    default:
      return 0;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 9;
  UART_SendMsg(Msg);
  return 1;
}

void Send_Sensor_Name(TMsg *Msg, uint8_t* sensor_name)
{
  int i = 0;
  BUILD_REPLY_HEADER(Msg);

  while (i < strlen((char const*)sensor_name))
  {
    Msg->Data[5 + i] = sensor_name[i];
    i++;
  }

  Msg->Len = 5 + i;
  UART_SendMsg(Msg);
}

void Send_Sensor_FS_List(TMsg *Msg, uint32_t* fs_list)
{
  int i = 0;
  BUILD_REPLY_HEADER(Msg);

  Serialize(&Msg->Data[5], fs_list[0], 4);

  for (i = 0; i < fs_list[0]; i++)
  {
    Serialize(&Msg->Data[9 + i * 4], fs_list[i + 1], 4);
  }

  Msg->Len = 9 + i * 4;
  UART_SendMsg(Msg);
}

void Send_Sensor_ODR_List(TMsg *Msg, float* odr_list)
{
  int i = 0;
  BUILD_REPLY_HEADER(Msg);

  Serialize(&Msg->Data[5], (int) odr_list[0], 4);

  for (i = 0; i < odr_list[0]; i++)
  {
    FloatToArray(&Msg->Data[9 + i * 4], odr_list[i + 1]);
  }

  Msg->Len = 9 + i * 4;
  UART_SendMsg(Msg);
}

void FloatToArray(uint8_t *Dest, float data)
{
  memcpy(Dest, (void *) &data, 4);
}

void ArrayToFloat(uint8_t *Source, float *data)
{
  memcpy((void *) data, Source, 4);
}

/**
 * @}
 */

/**
 * @}
 */
