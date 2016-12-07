/**
  ******************************************************************************
  * @file    Projects/Multi/Applications/DataLogFusion/Src/main.c
  * @author  CL
  * @version V1.6.0
  * @date    8-November-2016
  * @brief   Main program body
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

/**
  * @mainpage Documentation for OSX MotionFX package of X-CUBE-MEMS1 Software for X-NUCLEO-IKS01A1 or X-NUCLEO-IKS01A2 expansion board
  *
  * @image html st_logo.png
  *
  * <b>Introduction</b>
  *
  * OSX MotionFX software is an add-on for the X-CUBE-MEMS1 software and provides real-time motion-sensor data fusion.
  * The expansion is built on top of STM32Cube software technology that eases portability across different STM32 microcontrollers.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "com.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include <math.h>   // trunc
#include "DemoSerial.h"
#include "MotionFX_Manager.h"

/** @addtogroup OSX_MOTION_FX_Applications
  * @{
  */

/** @addtogroup DATALOGFUSION
  * @{
  */

/* Private define ------------------------------------------------------------*/
#define DATALOG_TIM_COUNTER_CLK        10000
#define DATALOG_TIM_PERIOD             10

#define OSXMOTIONFX_CHECK_CALIBRATION ((uint32_t)0x12345678)

#ifdef OSXMOTIONFX_STORE_CALIB_FLASH

#if (defined (USE_STM32F4XX_NUCLEO))
#define OSXMOTIONFX_FLASH_ADD ((uint32_t)0x08060000)
#define OSXMOTIONFX_FLASH_SECTOR FLASH_SECTOR_7
#endif

#if (defined (USE_STM32L4XX_NUCLEO))
#define OSXMOTIONFX_FLASH_ADD ((uint32_t)0x080FF800)
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
#endif

#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
#if defined (__IAR_SYSTEMS_ICC__)
__no_init uint32_t CalibrationStructureRAM[8];
#elif defined (__CC_ARM)
#if (defined (USE_STM32F4XX_NUCLEO))
uint32_t *CalibrationStructureRAM = (uint32_t *)0x20017FC0;
#endif

#if (defined (USE_STM32L4XX_NUCLEO))
uint32_t *CalibrationStructureRAM = (uint32_t *)0x10000000;
#endif
#elif defined (__GNUC__)
uint32_t CalibrationStructureRAM[8] __attribute__ ((section (".noinit")));
#else
#error "Toolchain not supported"
#endif
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive;
extern int use_LSI;

/* Private variables ---------------------------------------------------------*/
char dataOut[256];
RTC_HandleTypeDef RtcHandle;
int RTC_SYNCH_PREDIV;
volatile uint32_t Sensors_Enabled = 0;
volatile uint32_t DataTxPeriod = 1000;
SensorAxes_t ACC_Value;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;
osxMFX_calibFactor magOffset;
uint8_t calibIndex = 0;         // run calibration @ 25Hz
unsigned char isCal = 0;
float PRESSURE_Value;
float HUMIDITY_Value;
float TEMPERATURE_Value;
volatile uint8_t SF_Active = 0;
volatile uint8_t SF_6x_enabled = 0;
volatile uint8_t SF_change = 0;
TMsg Msg;
/* DataLog timer */
TIM_HandleTypeDef    DataLogTimHandle;
uint16_t timer_period = 1000 - 1;
uint16_t timer_pulse = (1000) / 2;
uint32_t sysclk, hclk, pclk1, pclk2;
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
void *MAGNETO_handle = NULL;
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);

static void initializeAllSensors(void);
static void enableAllSensors(void);
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
static void RTC_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Pressure_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);
static void SF_Handler(TMsg *Msg);
static unsigned char SaveCalibrationToMemory(void);
static unsigned char ResetCalibrationInMemory(void);
static unsigned char RecallCalibrationFromMemory(void);
void DataLogTimerInit(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use X_NUCLEO_IKS01A1 or X_NUCLEO_IKS01A2 expansion board to send sensor fusion data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on Unicleo, which is developed by STMicroelectronics and provided
 *         in binary with a separated package, or on Sensors_DataLog specific application, which is developed by STMicroelectronics
 *         and provided with X-CUBE-MEMS1 package.
 *
 *         After connection has been established with Unicleo or Sensors_DataLog application:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity and Pressure
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope and Magnetometer
 *         - the user can also visualize this data as graphs
 *         - the user can also visualize a cube animation that shows sensor fusion outputs
 *         - the user can also visualize MEMS data (Accelerometer, Gyroscope and Magnetometer) and sensor fusion data (quaternions and Euler angles) when sensor fusion is
 *           activated
 *         - the user can calibrate the Magnetometer when sensor fusion is activated pressing on the user button; the calibration data are stored in memory; this procedure
 *           is needed only when fusion 9X is used
 *         - the user can dynamically switch from fusion 9X to fusion 6X and vice-versa
 *         - the user can visualize the Magnetometer scatter plots in order to check the goodness of the calibration
 * @param  None
 * @retval Integer
 */
int main(void)
{
  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize LEDs */
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED2);
  
  /* Initialize Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  initializeAllSensors();
  enableAllSensors();
  
  MotionFX_manager_init();
  
  MotionFX_manager_start_9X();
  
  timer_period = (DATALOG_TIM_COUNTER_CLK / (1000 / DATALOG_TIM_PERIOD)) - 1;
  timer_pulse = timer_period / 2;
  
  /* Initialize UART */
  USARTConfig();
  
  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  
  sysclk = HAL_RCC_GetSysClockFreq();
  hclk = HAL_RCC_GetHCLKFreq();
  pclk1 = HAL_RCC_GetPCLK1Freq();
  pclk2 = HAL_RCC_GetPCLK2Freq();
  
  /* Check if the calibration is already available in memory */
  RecallCalibrationFromMemory();
  
  while(1)
  {
    /* Check if user button was pressed only when Sensor Fusion is active */
    if((BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET) && SF_Active)
    {
      while (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET);
      /* Reset the Compass Calibration */
      isCal = 0;
      osx_MotionFX_compass_forceReCalibration();
      
      ResetCalibrationInMemory();
      
      magOffset.magOffX = 0;
      magOffset.magOffY = 0;
      magOffset.magOffZ = 0;
      
      /* Switch off the LED */
      BSP_LED_Off(LED2);
    }
    
    if (UART_ReceivedMSG((TMsg*) &Msg))
    {
      if (Msg.Data[0] == DEV_ADDR)
      {
        HandleMSG((TMsg*) &Msg);
      }
    }
    
    if ( !SF_Active )
    {
      RTC_Handler(&Msg);
      Pressure_Sensor_Handler(&Msg);
      Humidity_Sensor_Handler(&Msg);
      Temperature_Sensor_Handler(&Msg);
      Accelero_Sensor_Handler(&Msg);
      Gyro_Sensor_Handler(&Msg);
      Magneto_Sensor_Handler(&Msg);
      
      if(DataLoggerActive)
      {
        INIT_STREAMING_HEADER(&Msg);
        Msg.Len = STREAMING_MSG_LENGTH;
        UART_SendMsg(&Msg);
        HAL_Delay(DataTxPeriod);
      }
    }
  }
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void)
{
#ifdef USE_IKS01A2
  /* Try to use automatic discovery. By default use LSM6DSL on board */
  BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
  /* Try to use automatic discovery. By default use LSM6DSL on board */
  BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle );
  /* Try to use automatic discovery. By default use LSM303AGR on board */
  BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &MAGNETO_handle );
  /* Try to use automatic discovery. By default use HTS221 on board */
  BSP_HUMIDITY_Init( HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle );
  /* Try to use automatic discovery. By default use HTS221 on board */
  BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle );
  /* Try to use automatic discovery. By default use LPS22HB on board */
  BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle );
#elif USE_IKS01A1
  /* Try to use LSM6DS3 DIL24 if present, otherwise use LSM6DS0 on board */
  BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
  /* Try to use LSM6DS3 if present, otherwise use LSM6DS0 */
  BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle );
  /* Force to use LIS3MDL */
  BSP_MAGNETO_Init( LIS3MDL_0, &MAGNETO_handle );
  /* Force to use HTS221 */
  BSP_HUMIDITY_Init( HTS221_H_0, &HUMIDITY_handle );
  /* Force to use HTS221 */
  BSP_TEMPERATURE_Init( HTS221_T_0, &TEMPERATURE_handle );
  /* Try to use LPS22HB DIL24 or LPS25HB DIL24 if present, otherwise use LPS25HB on board */
  BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle );
#endif
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors(void)
{
  BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
  BSP_GYRO_Sensor_Enable( GYRO_handle );
  BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
  BSP_HUMIDITY_Sensor_Enable( HUMIDITY_handle );
  BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
  BSP_PRESSURE_Sensor_Enable( PRESSURE_handle );
}


/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;
  
  if(DataLoggerActive)
  {
    HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
    subSec = ((((((int) RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV + 1)) & 0xff);
    Msg->Data[3] = (uint8_t)stimestructure.Hours;
    Msg->Data[4] = (uint8_t)stimestructure.Minutes;
    Msg->Data[5] = (uint8_t)stimestructure.Seconds;
    Msg->Data[6] = subSec;
  }
}

/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg ACCELERO part of the stream
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  
  if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & ACCELEROMETER_SENSOR)
      {
        BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
        Serialize_s32(&Msg->Data[15], ACC_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[19], ACC_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[23], ACC_Value.AXIS_Z, 4);
      }
    }
  }
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg GYRO part of the stream
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  
  if(BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & GYROSCOPE_SENSOR)
      {
        BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
        Serialize_s32(&Msg->Data[27], GYR_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[31], GYR_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[35], GYR_Value.AXIS_Z, 4);
      }
    }
  }
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg MAGNETO part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
  uint8_t status = 0;
  
  if(BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & MAGNETIC_SENSOR)
      {
        BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
        Serialize_s32(&Msg->Data[39], (int32_t)(MAG_Value.AXIS_X - magOffset.magOffX), 4);
        Serialize_s32(&Msg->Data[43], (int32_t)(MAG_Value.AXIS_Y - magOffset.magOffY), 4);
        Serialize_s32(&Msg->Data[47], (int32_t)(MAG_Value.AXIS_Z - magOffset.magOffZ), 4);
      }
    }
  }
}

/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg PRESSURE part of the stream
 * @retval None
 */
static void Pressure_Sensor_Handler(TMsg *Msg)
{
  int32_t d1, d2;
  uint8_t status = 0;
  
  if(BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & PRESSURE_SENSOR)
      {
        BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);
        floatToInt(PRESSURE_Value, &d1, &d2, 2);
        Serialize(&Msg->Data[7], d1, 2);
        Serialize(&Msg->Data[9], d2, 2);
      }
    }
  }
}

/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg HUMIDITY part of the stream
 * @retval None
 */
static void Humidity_Sensor_Handler(TMsg *Msg)
{
  int32_t d1, d2;
  uint8_t status = 0;
  
  if(BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & HUMIDITY_SENSOR)
      {
        BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
        floatToInt(HUMIDITY_Value, &d1, &d2, 2);
        Serialize(&Msg->Data[13], d1, 1);
        Serialize(&Msg->Data[14], d2, 1);
      }
    }
  }
}

/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg TEMPERATURE part of the stream
 * @retval None
 */
static void Temperature_Sensor_Handler(TMsg *Msg)
{
  int32_t d3, d4;
  uint8_t status = 0;
  
  if(BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
  {
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & TEMPERATURE_SENSOR)
      {
        BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
        floatToInt(TEMPERATURE_Value, &d3, &d4, 2);
        Serialize(&Msg->Data[11], d3, 1);
        Serialize(&Msg->Data[12], d4, 1);
      }
    }
  }
}

/**
 * @brief  Handles the Sensor Fusion
 * @param  Msg Sensor Fusion part of the stream
 * @retval None
 */
static void SF_Handler(TMsg *Msg)
{
  uint8_t status_acc = 0;
  uint8_t status_gyr = 0;
  uint8_t status_mag = 0;
  
  BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status_acc);
  BSP_GYRO_IsInitialized(GYRO_handle, &status_gyr);
  BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status_mag);
  
  if(status_acc && status_gyr && status_mag)
  {
    if ( SF_Active )
    {
      uint8_t subSec = 0;
      RTC_DateTypeDef sdatestructureget;
      RTC_TimeTypeDef stimestructure;
      
      if(SF_change == 1)
      {
        if(SF_6x_enabled == 0)
        {
          MotionFX_manager_stop_9X();
          MotionFX_manager_start_6X();
          SF_6x_enabled = 1;
        }
        else
        {
          MotionFX_manager_stop_6X();
          MotionFX_manager_start_9X();
          SF_6x_enabled = 0;
        }
        SF_change = 0;
      }
      
      BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
      BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
      BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
      
      INIT_STREAMING_HEADER(Msg);
      Msg->Data[2] = CMD_SF_Data;
      
      MotionFX_manager_run();
      
      /* Check if is calibrated */
      if(isCal != 0x01)
      {
        /* Run Compass Calibration @ 25Hz */
        calibIndex++;
        if (calibIndex == 4)
        {
          SensorAxes_t ACC_Loc, MAG_Loc;
          calibIndex = 0;
          ACC_Loc.AXIS_X = ACC_Value.AXIS_X;
          ACC_Loc.AXIS_Y = ACC_Value.AXIS_Y;
          ACC_Loc.AXIS_Z = ACC_Value.AXIS_Z;
          MAG_Loc.AXIS_X = MAG_Value.AXIS_X;
          MAG_Loc.AXIS_Y = MAG_Value.AXIS_Y;
          MAG_Loc.AXIS_Z = MAG_Value.AXIS_Z;
          osx_MotionFX_compass_saveAcc(ACC_Loc.AXIS_X, ACC_Loc.AXIS_Y, ACC_Loc.AXIS_Z); /* Accelerometer data ENU systems coordinate  */
          osx_MotionFX_compass_saveMag(MAG_Loc.AXIS_X, MAG_Loc.AXIS_Y, MAG_Loc.AXIS_Z); /* Magnetometer data ENU systems coordinate */
          osx_MotionFX_compass_run();
        }
        
        /* Check if is calibrated */
        isCal = osx_MotionFX_compass_isCalibrated();
        if(isCal == 0x01)
        {
          /* Get new magnetometer offset */
          osx_MotionFX_getCalibrationData(&magOffset);
          
          /* Save the calibration in Memory */
          SaveCalibrationToMemory();
          
          /* Switch on the Led */
          BSP_LED_On(LED2);
        }
      }
      
      osxMFX_output *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();
      
      if(SF_6x_enabled == 1)
      {
        memcpy(&Msg->Data[3], (uint8_t*)&MotionFX_Engine_Out->rotation_6X, 3 * sizeof(float));  // rot + Quat
        memcpy(&Msg->Data[3 + (3 * sizeof(float))], (uint8_t*)&MotionFX_Engine_Out->quaternion_6X, 4 * sizeof(float)); // rot + Quat
      }
      else
      {
        memcpy(&Msg->Data[3], (uint8_t*)&MotionFX_Engine_Out->rotation_9X, 3 * sizeof(float));  // rot + Quat
        memcpy(&Msg->Data[3 + (3 * sizeof(float))], (uint8_t*)&MotionFX_Engine_Out->quaternion_9X, 4 * sizeof(float)); // rot + Quat
      }
      
      HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
      HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
      subSec = ((((((int) RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV + 1)) & 0xff);
      Msg->Data[31] = (uint8_t)stimestructure.Hours;
      Msg->Data[32] = (uint8_t)stimestructure.Minutes;
      Msg->Data[33] = (uint8_t)stimestructure.Seconds;
      Msg->Data[34] = subSec;
      Serialize_s32(&Msg->Data[35], (int32_t)ACC_Value.AXIS_X, 4);
      Serialize_s32(&Msg->Data[39], (int32_t)ACC_Value.AXIS_Y, 4);
      Serialize_s32(&Msg->Data[43], (int32_t)ACC_Value.AXIS_Z, 4);
      Serialize_s32(&Msg->Data[47], (int32_t)GYR_Value.AXIS_X, 4);
      Serialize_s32(&Msg->Data[51], (int32_t)GYR_Value.AXIS_Y, 4);
      Serialize_s32(&Msg->Data[55], (int32_t)GYR_Value.AXIS_Z, 4);
      Serialize_s32(&Msg->Data[59], (int32_t)(MAG_Value.AXIS_X - magOffset.magOffX), 4);
      Serialize_s32(&Msg->Data[63], (int32_t)(MAG_Value.AXIS_Y - magOffset.magOffY), 4);
      Serialize_s32(&Msg->Data[67], (int32_t)(MAG_Value.AXIS_Z - magOffset.magOffZ), 4);
      
      Msg->Len = 3 + 4 * 7 + 4 + 4 * 3 + 4 * 3 + 4 * 3;
      UART_SendMsg(Msg);
    }
  }
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  
  /*##-1- Configue LSE as RTC clock soucre ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    use_LSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSI;
  } else
  {
    /* We use LSE */
    use_LSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSE;
  }
  RtcHandle.Instance = RTC;
  
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 12
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  
  /*##-3- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x14;
  sdatestructure.Month = RTC_MONTH_FEBRUARY;
  sdatestructure.Date = 0x18;
  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-4- Configure the Time #################################################*/
  /* Set Time: 08:10:00 */
  stimestructure.Hours = 0x08;
  stimestructure.Minutes = 0x10;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;
  
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Initialize DataLog timer
 * @param  None
 * @retval None
 */
void DataLogTimerInit(void)
{
  TIM_OC_InitTypeDef sConfig;
  uint16_t uhPrescalerValue;
  //uint16_t uhCapturedValue = 0;
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uhPrescalerValue = (uint16_t) ((SystemCoreClock) / DATALOG_TIM_COUNTER_CLK) - 1;
  
  /*##-1- Configure the TIM peripheral #######################################*/
  /* Set TIMx instance */
  DataLogTimHandle.Instance = TIMDataLog;
  
  /* Initialize TIM3 peripheral as follow:
  + Period = 65535
  + Prescaler = (SystemCoreClock/2)/60000
  + ClockDivision = 0
  + Counter direction = Up
  */
  
  DataLogTimHandle.Init.Period = timer_period;
  DataLogTimHandle.Init.Prescaler = uhPrescalerValue;
  DataLogTimHandle.Init.ClockDivision = 0;
  DataLogTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  DataLogTimHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_OC_Init(&DataLogTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration */
  sConfig.OCMode = TIM_OCMODE_TIMING;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;
  
  /* Set the pulse value for channel 1 */
  sConfig.Pulse = timer_pulse;
  
  if(HAL_TIM_OC_ConfigChannel(&DataLogTimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  
  /*##-4- Start the Output Compare mode in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_OC_Start_IT(&DataLogTimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
}


/**
* @brief  Conversion complete callback in non blocking mode
* @param  htim timer handler
* @retval None
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  SF_Handler(&Msg);
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  while(1)
  {
  }
}

/**
 * @brief  Get the current tick value in millisecond
 * @param  None
 * @retval The tick value
 */
uint32_t user_currentTimeGetTick(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Get the delta tick value in millisecond from Tick1 to the current tick
 * @param  Tick1 the reference tick used to compute the delta
 * @retval The delta tick value
 */
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1)
{
  volatile uint32_t Delta, Tick2;
  
  Tick2 = HAL_GetTick();
  
  /* Capture computation */
  Delta = Tick2 - Tick1;
  return Delta;
}

/**
 * @brief  Save the Magnetometer calibration values to memory
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
static unsigned char SaveCalibrationToMemory(void)
{
  unsigned char Success = 1;
  
  /* Reset Before The data in Memory */
  Success = ResetCalibrationInMemory();
  
  if(Success)
#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
  {
    /* Store in Flash Memory */
    uint32_t Address = OSXMOTIONFX_FLASH_ADD;
#if (defined (USE_STM32F4XX_NUCLEO))
    uint32_t WritemagOffset[8];
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    uint64_t WritemagOffset64[8];
#endif
    int32_t WriteIndex;
#if (defined (USE_STM32F4XX_NUCLEO))
    WritemagOffset[0] = OSXMOTIONFX_CHECK_CALIBRATION;
    WritemagOffset[1] = (uint32_t) magOffset.magOffX;
    WritemagOffset[2] = (uint32_t) magOffset.magOffY;
    WritemagOffset[3] = (uint32_t) magOffset.magOffZ;
    memcpy(&WritemagOffset[4], &(magOffset.magGainX), sizeof(float));
    memcpy(&WritemagOffset[5], &(magOffset.magGainY), sizeof(float));
    memcpy(&WritemagOffset[6], &(magOffset.magGainZ), sizeof(float));
    memcpy(&WritemagOffset[7], &(magOffset.expMagVect), sizeof(float));
#endif
    
#if (defined (USE_STM32L4XX_NUCLEO))
    WritemagOffset64[0] = (uint64_t)OSXMOTIONFX_CHECK_CALIBRATION;
    WritemagOffset64[1] = (uint64_t) magOffset.magOffX;
    WritemagOffset64[2] = (uint64_t) magOffset.magOffY;
    WritemagOffset64[3] = (uint64_t) magOffset.magOffZ;
    memcpy(&WritemagOffset64[4], &(magOffset.magGainX), sizeof(float));
    memcpy(&WritemagOffset64[5], &(magOffset.magGainY), sizeof(float));
    memcpy(&WritemagOffset64[6], &(magOffset.magGainZ), sizeof(float));
    memcpy(&WritemagOffset64[7], &(magOffset.expMagVect), sizeof(float));
#endif
    
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    
#if (defined (USE_STM32F4XX_NUCLEO))
    for(WriteIndex = 0; WriteIndex < 8; WriteIndex++)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    for(WriteIndex = 0; WriteIndex < 8; WriteIndex++)
#endif
    {
#if (defined (USE_STM32F4XX_NUCLEO))
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, WritemagOffset[WriteIndex]) == HAL_OK)
      {
        Address = Address + 4;
      }
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, WritemagOffset64[WriteIndex]) == HAL_OK)
      {
        Address = Address + 8;
      }
#endif
      else
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        /*
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
        */
        Error_Handler();
      }
    }
      
    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }
#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
  {
    /* Store in RAM */
    CalibrationStructureRAM[0] = OSXMOTIONFX_CHECK_CALIBRATION;
    CalibrationStructureRAM[1] = (uint32_t) magOffset.magOffX;
    CalibrationStructureRAM[2] = (uint32_t) magOffset.magOffY;
    CalibrationStructureRAM[3] = (uint32_t) magOffset.magOffZ;
    memcpy(&CalibrationStructureRAM[4], &(magOffset.magGainX), sizeof(float));
    memcpy(&CalibrationStructureRAM[5], &(magOffset.magGainY), sizeof(float));
    memcpy(&CalibrationStructureRAM[6], &(magOffset.magGainZ), sizeof(float));
    memcpy(&CalibrationStructureRAM[7], &(magOffset.expMagVect), sizeof(float));
  }
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */
  
  
  return Success;
}

/**
 * @brief  Reset the Magnetometer calibration values in memory
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
static unsigned char ResetCalibrationInMemory(void)
#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
{
  /* Reset Calibration Values in FLASH */
  unsigned char Success = 1;
  
  /* Erase First Flash sector */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  
#if (defined (USE_STM32F4XX_NUCLEO))
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = OSXMOTIONFX_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;
#endif
  
#if (defined (USE_STM32L4XX_NUCLEO))
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(OSXMOTIONFX_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(OSXMOTIONFX_FLASH_ADD);
  EraseInitStruct.NbPages     = 1;
#endif
  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    /*
      Error occurred while sector erase.
      User can add here some code to deal with this error.
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */
    /*
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
    */
    Error_Handler();
    Success = 0;
  }
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  
  return Success;
}
#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
{
  /* Reset Calibration Values in RAM */
  unsigned char Success = 1;
  int32_t Counter;
  
  for(Counter = 0; Counter < 8; Counter++)
    CalibrationStructureRAM[Counter] = 0xFFFFFFFF;
    
  return Success;
}
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

/**
 * @brief  Check if there are valid calibration values in memory and read them
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
static unsigned char RecallCalibrationFromMemory(void)
#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
{
  /* ReLoad the Calibration Values from FLASH */
  unsigned char Success = 1;
  uint32_t Address = OSXMOTIONFX_FLASH_ADD;
#if (defined (USE_STM32F4XX_NUCLEO))
  __IO uint32_t data = *(__IO uint32_t*) Address;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
  __IO uint64_t data = *(__IO uint64_t*) Address;
#endif
  if(data == OSXMOTIONFX_CHECK_CALIBRATION)
  {
    int32_t ReadIndex;
#if (defined (USE_STM32F4XX_NUCLEO))
    uint32_t ReadmagOffset[7];
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    uint64_t ReadmagOffset[7];
#endif
    
    for(ReadIndex = 0; ReadIndex < 7; ReadIndex++)
    {
#if (defined (USE_STM32F4XX_NUCLEO))
      Address += 4;
      data = *(__IO uint32_t*) Address;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
      Address += 8;
      data = *(__IO uint64_t*) Address;
#endif
      ReadmagOffset[ReadIndex] = data;
    }
    
    magOffset.magOffX    = (signed short) ReadmagOffset[0];
    magOffset.magOffY    = (signed short) ReadmagOffset[1];
    magOffset.magOffZ    = (signed short) ReadmagOffset[2];
    magOffset.magGainX = (float) ReadmagOffset[3];
    magOffset.magGainY = (float) ReadmagOffset[4];
    magOffset.magGainZ = (float) ReadmagOffset[5];
    magOffset.expMagVect = (float) ReadmagOffset[6];
    
    /* Set the Calibration Structure */
    osx_MotionFX_setCalibrationData(&magOffset);
    
    /* Control the calibration status */
    isCal = osx_MotionFX_compass_isCalibrated();
    
    if(isCal == 0x01)
    {
      /* Switch on the Led */
      BSP_LED_On(LED2);
    }
    else
    {
      /* Switch off the Led */
      BSP_LED_Off(LED2);
    }
  }
  else
  {
    /* Switch off the Led */
    BSP_LED_Off(LED2);
    isCal = 0;
  }
  
  return Success;
}
#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
{
  /* ReLoad the Calibration Values from RAM */
  unsigned char Success = 1;
  
  if(CalibrationStructureRAM[0] == OSXMOTIONFX_CHECK_CALIBRATION)
  {
    magOffset.magOffX    = (signed short) CalibrationStructureRAM[1];
    magOffset.magOffY    = (signed short) CalibrationStructureRAM[2];
    magOffset.magOffZ    = (signed short) CalibrationStructureRAM[3];
    memcpy(&magOffset.magGainX, &(CalibrationStructureRAM[4]), sizeof(uint32_t));
    memcpy(&magOffset.magGainY, &(CalibrationStructureRAM[5]), sizeof(uint32_t));
    memcpy(&magOffset.magGainZ, &(CalibrationStructureRAM[6]), sizeof(uint32_t));
    memcpy(&magOffset.expMagVect, &(CalibrationStructureRAM[7]), sizeof(uint32_t));
    
    /* Set the Calibration Structure */
    osx_MotionFX_setCalibrationData(&magOffset);
    
    /* Control the calibration status */
    isCal = osx_MotionFX_compass_isCalibrated();
    
    if(isCal == 0x01)
    {
      /* Switch on the Led */
      BSP_LED_On(LED2);
    }
    else
    {
      /* Switch off the Led */
      BSP_LED_Off(LED2);
    }
  }
  else
  {
    /* Switch off the Led */
    BSP_LED_Off(LED2);
    isCal = 0;
  }
  
  return Success;
}
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
#if (defined (USE_STM32L4XX_NUCLEO))
/**
  * @brief  Gets the page of a given address
  * @param  Addr Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}
#endif /* USE_STM32L4XX_NUCLEO */
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
