/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A2/LSM6DSL_FIFOMode/Src/main.c
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
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A2_Examples
 * @{
 */

/** @addtogroup FIFO_MODE
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Handle DEMO State Machine
 */
typedef enum
{
  STATUS_IDLE,
  STATUS_SET_FIFO_MODE,
  STATUS_FIFO_RUN,
  STATUS_FIFO_DOWNLOAD,
  STATUS_SET_BYPASS_MODE
} DEMO_FIFO_STATUS;



/* Private define ------------------------------------------------------------*/
#define FIFO_INDICATION_DELAY  100 /*!< When FIFO event ocurs, LED is ON for at least this period [ms] */

#define FIFO_WATERMARK  31 /*!< FIFO size limit */

#define SAMPLE_ODR            ODR_LOW /*!< Sample Output Data Rate */
#define LSM6DSL_FIFO_MAX_ODR  6600    /*!< LSM6DSL FIFO maximum ODR */

#define PATTERN_GYR_X_AXIS  0 /*!< Pattern of gyro X axis */
#define PATTERN_GYR_Y_AXIS  1 /*!< Pattern of gyro Y axis */
#define PATTERN_GYR_Z_AXIS  2 /*!< Pattern of gyro Z axis */

#define UART_TRANSMIT_TIMEOUT  5000



/* Private variables ---------------------------------------------------------*/
/* This variable MUST be volatile because it could change into a ISR */
static volatile uint8_t memsIntDetected = 0;

static char dataOut[256];
static void *LSM6DSL_G_0_handle = NULL;

/* This variable MUST be volatile because it could change into a ISR */
static volatile DEMO_FIFO_STATUS demoFifoStatus = STATUS_SET_BYPASS_MODE;



/* Private function prototypes -----------------------------------------------*/
static DrvStatusTypeDef Init_All_Sensors(void);
static DrvStatusTypeDef Enable_All_Sensors(void);
static DrvStatusTypeDef LSM6DSL_FIFO_Set_Bypass_Mode(void);
static DrvStatusTypeDef LSM6DSL_FIFO_Set_FIFO_Mode(void);
static DrvStatusTypeDef LSM6DSL_Read_All_FIFO_Data(void);
static DrvStatusTypeDef LSM6DSL_Read_Single_FIFO_Pattern_Cycle(uint16_t sampleIndex);
static DrvStatusTypeDef LSM6DSL_FIFO_Demo_Config(void);



/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use sensor expansion board to run the
 *         LSM6DSL FIFO in FIFO Mode
 * @param  None
 * @retval Integer
 */
int main(void)
{
  uint8_t fifo_full_status = 0;
  uint16_t samplesInFIFO = 0;
  uint16_t oldSamplesInFIFO = 0;

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
  BSP_LED_Init(LED2);

  /* Initialize button */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#elif (defined (USE_STM32L1XX_NUCLEO))
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
#endif

  /* Initialize UART */
  USARTConfig();

  if (Init_All_Sensors() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }

  if (Enable_All_Sensors() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }

  /* Configure LSM6DSL Sensor for the DEMO application */
  if (LSM6DSL_FIFO_Demo_Config() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }

  sprintf(dataOut, "\r\n------ LSM6DSL FIFO Mode DEMO ------\r\n\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  while (1)
  {

    /* Handle DEMO State Machine */
    switch (demoFifoStatus)
    {
      case STATUS_IDLE:
        break;

      case STATUS_SET_FIFO_MODE:

        if (LSM6DSL_FIFO_Set_FIFO_Mode() == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }
        demoFifoStatus = STATUS_FIFO_RUN;
        break;

      case STATUS_FIFO_RUN:

        /* Get num of unread FIFO samples before reading data */
        if (BSP_GYRO_FIFO_Get_Num_Of_Samples_Ext(LSM6DSL_G_0_handle, &samplesInFIFO) == COMPONENT_ERROR)
        {
          return COMPONENT_ERROR;
        }

        /* Print dot realtime when each new data is stored in FIFO */
        if (samplesInFIFO != oldSamplesInFIFO)
        {
          oldSamplesInFIFO = samplesInFIFO;
          sprintf(dataOut, ".");
          HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
        }

        if (memsIntDetected)
        {
          demoFifoStatus = STATUS_FIFO_DOWNLOAD;
        }
        break;

      case STATUS_FIFO_DOWNLOAD:

        /* Print data if FIFO is full */
        if (BSP_GYRO_FIFO_Get_Full_Status_Ext(LSM6DSL_G_0_handle, &fifo_full_status) == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        if (fifo_full_status == 1)
        {
          BSP_LED_On(LED2);

          if (LSM6DSL_Read_All_FIFO_Data() == COMPONENT_ERROR)
          {
            Error_Handler(__func__);
          }

          HAL_Delay(FIFO_INDICATION_DELAY);
          BSP_LED_Off(LED2);

          demoFifoStatus = STATUS_SET_BYPASS_MODE;
        }
        break;

      case STATUS_SET_BYPASS_MODE:

        if (LSM6DSL_FIFO_Set_Bypass_Mode() == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        memsIntDetected = 0;
        samplesInFIFO = 0;
        oldSamplesInFIFO = 0;
        demoFifoStatus = STATUS_IDLE;
        break;

      default:
        Error_Handler(__func__);
        break;
    }
  }
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Init_All_Sensors(void)
{
  return BSP_GYRO_Init(LSM6DSL_G_0, &LSM6DSL_G_0_handle);
}



/**
 * @brief  Enable all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Enable_All_Sensors(void)
{
  return BSP_GYRO_Sensor_Enable(LSM6DSL_G_0_handle);
}



/**
 * @brief  Configure FIFO
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_FIFO_Demo_Config(void)
{
  if (BSP_GYRO_Set_ODR(LSM6DSL_G_0_handle, SAMPLE_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set gyro FIFO decimation */
  if (BSP_GYRO_FIFO_Set_Decimation_Ext(LSM6DSL_G_0_handle, LSM6DSL_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO ODR to highest value */
  if (BSP_GYRO_FIFO_Set_ODR_Value_Ext(LSM6DSL_G_0_handle, LSM6DSL_FIFO_MAX_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO_FULL on INT1 */
  if (BSP_GYRO_FIFO_Set_INT1_FIFO_Full_Ext(LSM6DSL_G_0_handle,
      LSM6DSL_ACC_GYRO_INT1_FULL_FLAG_ENABLED) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO watermark */
  if (BSP_GYRO_FIFO_Set_Watermark_Level_Ext(LSM6DSL_G_0_handle, FIFO_WATERMARK) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO depth to be limited to watermark threshold level  */
  if (BSP_GYRO_FIFO_Set_Stop_On_Fth_Ext(LSM6DSL_G_0_handle, LSM6DSL_ACC_GYRO_STOP_ON_FTH_ENABLED) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Set FIFO bypass mode
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_FIFO_Set_Bypass_Mode(void)
{
  if (BSP_GYRO_FIFO_Set_Mode_Ext(LSM6DSL_G_0_handle, LSM6DSL_ACC_GYRO_FIFO_MODE_BYPASS) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  sprintf(dataOut, "Press USER button to start the DEMO...\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  return COMPONENT_OK;
}



/**
 * @brief  Set FIFO mode
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_FIFO_Set_FIFO_Mode(void)
{
  sprintf(dataOut, "\r\nLSM6DSL starts to store the data into FIFO...\r\n\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  HAL_Delay(1000);

  /* Set FIFO mode to FIFO */
  if (BSP_GYRO_FIFO_Set_Mode_Ext(LSM6DSL_G_0_handle, LSM6DSL_ACC_GYRO_FIFO_MODE_FIFO) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Read all unread FIFO data in cycle
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_Read_All_FIFO_Data(void)
{
  uint16_t samplesToRead = 0;
  int i = 0;

  /* Get num of unread FIFO samples before reading data */
  if (BSP_GYRO_FIFO_Get_Num_Of_Samples_Ext(LSM6DSL_G_0_handle, &samplesToRead) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* 'samplesToRead' actually contains number of words in FIFO but each FIFO sample (data set) consists of 3 words
  so the 'samplesToRead' has to be divided by 3 */
  samplesToRead /= 3;

  sprintf(dataOut, "\r\n\r\n%d samples in FIFO.\r\n\r\nStarted downloading data from FIFO...\r\n\r\n", samplesToRead);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  HAL_Delay(1000);

  sprintf(dataOut, "[DATA ##]     GYR_X     GYR_Y     GYR_Z\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  for (i = 0; i < samplesToRead; i++)
  {
    if (LSM6DSL_Read_Single_FIFO_Pattern_Cycle(i) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  sprintf(dataOut, "\r\nFIFO download completed.\r\n\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  return COMPONENT_OK;
}



/**
 * @brief  Read single FIFO pattern cycle
 * @param  sampleIndex Current sample index.
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_Read_Single_FIFO_Pattern_Cycle(uint16_t sampleIndex)
{
  uint16_t pattern = 0;
  int32_t angular_velocity = 0;
  int32_t gyr_x = 0, gyr_y = 0, gyr_z = 0;
  int i = 0;

  /* Read one whole FIFO pattern cycle. Pattern: Gx, Gy, Gz */
  for (i = 0; i <= 2; i++)
  {
    /* Read FIFO pattern number */
    if (BSP_GYRO_FIFO_Get_Pattern_Ext(LSM6DSL_G_0_handle, &pattern) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }

    /* Read single FIFO data (angular velocity in one axis) */
    if (BSP_GYRO_FIFO_Get_Axis_Ext(LSM6DSL_G_0_handle, &angular_velocity) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }

    /* Decide which axis has been read from FIFO based on pattern number */
    switch(pattern)
    {
      case PATTERN_GYR_X_AXIS:
        gyr_x = angular_velocity;
        break;

      case PATTERN_GYR_Y_AXIS:
        gyr_y = angular_velocity;
        break;

      case PATTERN_GYR_Z_AXIS:
        gyr_z = angular_velocity;
        break;

      default:
        return COMPONENT_ERROR;
    }
  }

  sprintf(dataOut, "[DATA %02d]  %8ld  %8ld  %8ld\r\n", sampleIndex + 1, gyr_x, gyr_y, gyr_z);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  return COMPONENT_OK;
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* User button pressed */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  if(GPIO_Pin == KEY_BUTTON_PIN)
#elif (defined (USE_STM32L1XX_NUCLEO))
  if(GPIO_Pin == USER_BUTTON_PIN)
#endif
  {
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
#elif (defined (USE_STM32L1XX_NUCLEO))
    if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET)
#elif (defined (USE_STM32L0XX_NUCLEO))
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_SET)
#endif
    {
      /* Change this variable only if demoFifoStatus is STATUS_IDLE */
      if (demoFifoStatus == STATUS_IDLE)
      {
        demoFifoStatus = STATUS_SET_FIFO_MODE;
      }
    }
  }

  /* FIFO full (available only for LSM6DSL sensor) */
  else if (GPIO_Pin == LSM6DSL_INT1_O_PIN)
  {
    memsIntDetected = 1;
  }

  /* ERROR */
  else
  {
    Error_Handler(__func__);
  }
}



/**
 * @brief  This function is executed in case of error occurrence, turns LED2 ON and ends in infinite loop
 * @param  None
 * @retval None
 */
void Error_Handler(const char *function_name)
{
  sprintf(dataOut, "\r\nError in '%s' function.\r\n", function_name);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  while (1)
  {
    BSP_LED_On(LED2);
    HAL_Delay(FIFO_INDICATION_DELAY);
    BSP_LED_Off(LED2);
    HAL_Delay(FIFO_INDICATION_DELAY);
  }
}



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
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
