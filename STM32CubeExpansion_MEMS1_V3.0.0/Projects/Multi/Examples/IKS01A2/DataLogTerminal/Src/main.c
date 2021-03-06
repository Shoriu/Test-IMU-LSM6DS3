/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A2/DataLogTerminal/Src/main.c
 * @author  CL
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   Main program body
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

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A2_Examples
  * @{
  */

/** @addtogroup DATALOG_TERMINAL
  * @{
  */

extern int use_LSI;
int RTC_SYNCH_PREDIV;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t acquire_data_enable_request  = 1;
static volatile uint8_t acquire_data_disable_request = 0;

static uint8_t acquire_data_enabled = 0;
static uint8_t verbose              = 1;  /* Verbose output to UART terminal ON/OFF. */
static RTC_HandleTypeDef RtcHandle;
static char dataOut[256];

static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *HTS221_H_0_handle  = NULL;
static void *HTS221_T_0_handle  = NULL;
static void *LPS22HB_P_0_handle  = NULL;
static void *LPS22HB_T_0_handle  = NULL;



/* Private function prototypes -----------------------------------------------*/

static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void disableAllSensors( void );
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );
static void RTC_Handler( void );

static void Accelero_Sensor_Handler( void *handle );
static void Gyro_Sensor_Handler( void *handle );
static void Magneto_Sensor_Handler( void *handle );
static void Humidity_Sensor_Handler( void *handle );
static void Temperature_Sensor_Handler( void *handle );
static void Pressure_Sensor_Handler( void *handle );



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to use sensor expansion board to send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like TeraTerm.
 *         After connection has been established:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure.
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope, and Magnetometer.
 * @param  None
 * @retval Integer
 */
int main( void )
{

  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED */
  BSP_LED_Init( LED2 );

  /* Initialize button */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
#endif

  /* Initialize UART */
  USARTConfig();

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  initializeAllSensors();

  while (1)
  {

    if ( acquire_data_enable_request == 1 )
    {
      enableAllSensors();
      acquire_data_enabled = 1;
      acquire_data_enable_request = 0;
    }

    if ( acquire_data_disable_request == 1 )
    {
      disableAllSensors();
      acquire_data_enabled = 0;
      acquire_data_disable_request = 0;
    }

    if ( acquire_data_enabled == 1 )
    {

      /* Perform all handlers */
      RTC_Handler();
      Accelero_Sensor_Handler( LSM6DSL_X_0_handle );
      Gyro_Sensor_Handler( LSM6DSL_G_0_handle );
      Accelero_Sensor_Handler( LSM303AGR_X_0_handle );
      Magneto_Sensor_Handler( LSM303AGR_M_0_handle );
      Humidity_Sensor_Handler( HTS221_H_0_handle );
      Temperature_Sensor_Handler( HTS221_T_0_handle );
      Temperature_Sensor_Handler( LPS22HB_T_0_handle );
      Pressure_Sensor_Handler( LPS22HB_P_0_handle );
    }

    HAL_Delay( 1000 );
  }
}



/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
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
 * @param  None
 * @retval None
 */
static void RTC_Handler( void )
{

  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime( &RtcHandle, &stimestructure, FORMAT_BIN );
  HAL_RTC_GetDate( &RtcHandle, &sdatestructureget, FORMAT_BIN );
  subSec = (((((( int )RTC_SYNCH_PREDIV) - (( int )stimestructure.SubSeconds)) * 100) /
             ( RTC_SYNCH_PREDIV + 1 )) & 0xff );

  sprintf( dataOut, "\r\n\r\n\r\nTimeStamp: %02d:%02d:%02d.%02d\r\n", stimestructure.Hours, stimestructure.Minutes,
           stimestructure.Seconds, subSec );
  HAL_UART_Transmit( &UartHandle, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
}



/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Accelero_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t acceleration;
  uint8_t status;
  int32_t d1, d2;

  BSP_ACCELERO_Get_Instance( handle, &id );

  BSP_ACCELERO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
    {
      acceleration.AXIS_X = 0;
      acceleration.AXIS_Y = 0;
      acceleration.AXIS_Z = 0;
    }

    sprintf( dataOut, "\r\nACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n", (int)id, (int)acceleration.AXIS_X, (int)id,
             (int)acceleration.AXIS_Y, (int)id, (int)acceleration.AXIS_Z );

    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_ACCELERO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_ACCELERO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &d1, &d2, 3 );
        sprintf( dataOut, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_ACCELERO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "FS[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( fullScale, &d1, &d2, 3 );
        sprintf( dataOut, "FS[%d]: %d.%03d g\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the gyroscope axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Gyro_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t angular_velocity;
  uint8_t status;
  int32_t d1, d2;

  BSP_GYRO_Get_Instance( handle, &id );

  BSP_GYRO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }

    sprintf( dataOut, "\r\nGYR_X[%d]: %d, GYR_Y[%d]: %d, GYR_Z[%d]: %d\r\n", (int)id, (int)angular_velocity.AXIS_X, (int)id,
             (int)angular_velocity.AXIS_Y, (int)id, (int)angular_velocity.AXIS_Z );

    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_GYRO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_GYRO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &d1, &d2, 3 );
        sprintf( dataOut, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_GYRO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "FS[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( fullScale, &d1, &d2, 3 );
        sprintf( dataOut, "FS[%d]: %d.%03d dps\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the magneto axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Magneto_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t magnetic_field;
  uint8_t status;
  int32_t d1, d2;

  BSP_MAGNETO_Get_Instance( handle, &id );

  BSP_MAGNETO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }

    sprintf( dataOut, "\r\nMAG_X[%d]: %d, MAG_Y[%d]: %d, MAG_Z[%d]: %d\r\n", (int)id, (int)magnetic_field.AXIS_X, (int)id,
             (int)magnetic_field.AXIS_Y, (int)id, (int)magnetic_field.AXIS_Z );

    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_MAGNETO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_MAGNETO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &d1, &d2, 3 );
        sprintf( dataOut, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_MAGNETO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "FS[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( fullScale, &d1, &d2, 3 );
        sprintf( dataOut, "FS[%d]: %d.%03d Gauss\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the humidity data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Humidity_Sensor_Handler( void *handle )
{

  int32_t d1, d2;
  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float humidity;
  uint8_t status;

  BSP_HUMIDITY_Get_Instance( handle, &id );

  BSP_HUMIDITY_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_HUMIDITY_Get_Hum( handle, &humidity ) == COMPONENT_ERROR )
    {
      humidity = 0.0f;
    }

    floatToInt( humidity, &d1, &d2, 2 );
    sprintf( dataOut, "\r\nHUM[%d]: %d.%02d\r\n", (int)id, (int)d1, (int)d2 );
    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_HUMIDITY_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_HUMIDITY_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &d1, &d2, 3 );
        sprintf( dataOut, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the temperature data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Temperature_Sensor_Handler( void *handle )
{

  int32_t d1, d2;
  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float temperature;
  uint8_t status;

  BSP_TEMPERATURE_Get_Instance( handle, &id );

  BSP_TEMPERATURE_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_TEMPERATURE_Get_Temp( handle, &temperature ) == COMPONENT_ERROR )
    {
      temperature = 0.0f;
    }

    floatToInt( temperature, &d1, &d2, 2 );
    sprintf( dataOut, "\r\nTEMP[%d]: %d.%02d\r\n", (int)id, (int)d1, (int)d2 );
    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_TEMPERATURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_TEMPERATURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &d1, &d2, 3 );
        sprintf( dataOut, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the pressure sensor data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Pressure_Sensor_Handler( void *handle )
{

  int32_t d1, d2;
  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float pressure;
  uint8_t status;

  BSP_PRESSURE_Get_Instance( handle, &id );

  BSP_PRESSURE_IsInitialized( handle, &status );

  if( status == 1 )
  {
    if ( BSP_PRESSURE_Get_Press( handle, &pressure ) == COMPONENT_ERROR )
    {
      pressure = 0.0f;
    }

    floatToInt( pressure, &d1, &d2, 2 );
    sprintf(dataOut, "\r\nPRESS[%d]: %d.%02d\r\n", (int)id, (int)d1, (int)d2);
    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_PRESSURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_PRESSURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        sprintf( dataOut, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &d1, &d2, 3 );
        sprintf( dataOut, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)d1, (int)d2 );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{

  BSP_ACCELERO_Init( LSM6DSL_X_0, &LSM6DSL_X_0_handle );
  BSP_GYRO_Init( LSM6DSL_G_0, &LSM6DSL_G_0_handle );
  BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle );
  BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle );
  BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle );
  BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle );
  BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle );
  BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle );
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{

  BSP_ACCELERO_Sensor_Enable( LSM6DSL_X_0_handle );
  BSP_GYRO_Sensor_Enable( LSM6DSL_G_0_handle );
  BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
  BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
  BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
  BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
  BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
}



/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
static void disableAllSensors( void )
{

  BSP_ACCELERO_Sensor_Disable( LSM6DSL_X_0_handle );
  BSP_GYRO_Sensor_Disable( LSM6DSL_G_0_handle );
  BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
  BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
  BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
  BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}



/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config( void )
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
  }
  else
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
static void RTC_TimeStampConfig( void )
{

  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-3- Configure the Date using BCD format ################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Configure the Time using BCD format#################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
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
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss )
{

  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BIN ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  /* User button. */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  if(GPIO_Pin == KEY_BUTTON_PIN)
#elif (defined (USE_STM32L1XX_NUCLEO))
  if(GPIO_Pin == USER_BUTTON_PIN)
#endif
  {
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
    if ( BSP_PB_GetState( BUTTON_KEY ) == GPIO_PIN_RESET )
#elif (defined (USE_STM32L1XX_NUCLEO))
    if ( BSP_PB_GetState( BUTTON_USER ) == GPIO_PIN_RESET )
#elif (defined (USE_STM32L0XX_NUCLEO))
    if ( BSP_PB_GetState( BUTTON_KEY ) == GPIO_PIN_SET )
#endif
    {
      if ( acquire_data_enabled == 0 )
      {
        acquire_data_enable_request = 1;
      }
      else
      {
        acquire_data_disable_request = 1;
      }
    }
  }
}



/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler( void )
{

  while (1)
  {}
}



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
