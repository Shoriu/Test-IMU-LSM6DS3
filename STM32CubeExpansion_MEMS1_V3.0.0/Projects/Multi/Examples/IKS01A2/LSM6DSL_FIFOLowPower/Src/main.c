/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A2/LSM6DSL_FIFOLowPower/Src/main.c
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

/** @addtogroup FIFO_LOW_POWER
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Handle DEMO State Machine
 */
typedef enum
{
  STATUS_MEMS_INT1_DETECTED,
  STATUS_FIFO_DOWNLOAD,
  STATUS_SLEEP
} DEMO_FIFO_STATUS;



/* Private define ------------------------------------------------------------*/
#define FIFO_INDICATION_DELAY  100 /*!< When FIFO event ocurs, LED is ON for at least this period [ms] */

#define FIFO_WATERMARK  301 /*!< FIFO size limit */
#define SAMPLE_LIST_MAX  10 /*!< Max. number of acceleration values (X,Y,Z) to be printed to UART */

#define SAMPLE_ODR            ODR_LOW /*!< Accelerometer samples Output Data Rate */
#define LSM6DSL_FIFO_MAX_ODR  6600    /*!< LSM6DSL FIFO maximum ODR */

#define PATTERN_ACC_X_AXIS  0 /*!< Pattern of accelero X axis */
#define PATTERN_ACC_Y_AXIS  1 /*!< Pattern of accelero Y axis */
#define PATTERN_ACC_Z_AXIS  2 /*!< Pattern of accelero Z axis */

#define UART_TRANSMIT_TIMEOUT  5000



/* Private variables ---------------------------------------------------------*/
static char dataOut[256];
static void *LSM6DSL_X_0_handle = NULL;

/* This variable MUST be volatile because it could change into a ISR */
static volatile DEMO_FIFO_STATUS demoFifoStatus = STATUS_SLEEP;
static volatile uint8_t mems_int1_detected = 0;
static volatile uint8_t button_pressed = 0;



/* Private function prototypes -----------------------------------------------*/
static DrvStatusTypeDef Init_All_Sensors(void);
static DrvStatusTypeDef Enable_All_Sensors(void);
static DrvStatusTypeDef LSM6DSL_FIFO_Demo_Config(void);
static DrvStatusTypeDef LSM6DSL_Read_All_FIFO_Data(void);
static DrvStatusTypeDef LSM6DSL_Read_Single_FIFO_Pattern_Cycle(uint16_t sampleIndex);
static void Sleep_Mode(void);



/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use sensor expansion board to run the
 *         LSM6DSL FIFO in conjunction with MCU sleep mode to decrease consumption
 * @param  None
 * @retval Integer
 */
int main(void)
{
  uint8_t fifo_full_status = 0;

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

  sprintf(dataOut, "\r\n------ LSM6DSL FIFO Low Power DEMO ------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  sprintf(dataOut, "\r\nPress USER button to start the DEMO ...\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  /* Wait for USER BUTTON push */
  Sleep_Mode();

  while (1)
  {
    if(button_pressed)
    {
      button_pressed = 0;
      /* _NOTE_: Pushing button creates interrupt/event and wakes up MCU from sleep mode */
      sprintf(dataOut, "\r\nNucleo processor is waking up ...\r\n");
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
    }

    if(mems_int1_detected)
    {
      mems_int1_detected = 0;
      sprintf(dataOut, "\r\nReceived FIFO Threshold Interrupt on INT1 pin ...\r\n\r\nNucleo processor is waking up ...\r\n");
      HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
      demoFifoStatus = STATUS_MEMS_INT1_DETECTED;
    }

    /* Handle DEMO State Machine */
    switch (demoFifoStatus)
    {
      case STATUS_MEMS_INT1_DETECTED:

        /* Check if FIFO is full */
        if (BSP_ACCELERO_FIFO_Get_Full_Status_Ext(LSM6DSL_X_0_handle, &fifo_full_status) == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        if (fifo_full_status == 1)
        {
          demoFifoStatus = STATUS_FIFO_DOWNLOAD;
        }
        else
        {
          demoFifoStatus = STATUS_SLEEP;
        }
        break;

      case STATUS_FIFO_DOWNLOAD:

        BSP_LED_On(LED2);

        if (LSM6DSL_Read_All_FIFO_Data() == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        HAL_Delay(FIFO_INDICATION_DELAY);
        BSP_LED_Off(LED2);

        /* Reset FIFO by setting FIFO mode to Bypass */
        if (BSP_ACCELERO_FIFO_Set_Mode_Ext(LSM6DSL_X_0_handle, LSM6DSL_ACC_GYRO_FIFO_MODE_BYPASS) == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }

        demoFifoStatus = STATUS_SLEEP;
        break;

      case STATUS_SLEEP:

        sprintf(dataOut, "\r\nNucleo processor is entering sleep mode while LSM6DSL is storing data into FIFO ...\r\n");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

        /* Set FIFO mode to FIFO */
        if (BSP_ACCELERO_FIFO_Set_Mode_Ext(LSM6DSL_X_0_handle, LSM6DSL_ACC_GYRO_FIFO_MODE_FIFO) == COMPONENT_ERROR)
        {
          return COMPONENT_ERROR;
        }

        /* Enter sleep mode */
        Sleep_Mode();
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
  return BSP_ACCELERO_Init(LSM6DSL_X_0, &LSM6DSL_X_0_handle);
}



/**
 * @brief  Enable all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Enable_All_Sensors(void)
{
  return BSP_ACCELERO_Sensor_Enable(LSM6DSL_X_0_handle);
}



/**
 * @brief  Enter sleep mode and wait for interrupt
 * @param  None
 * @retval None
 * @retval None
 */
static void Sleep_Mode(void)
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; /* Systick IRQ OFF */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; /* Systick IRQ ON */
}



/**
 * @brief  Configure FIFO
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LSM6DSL_FIFO_Demo_Config(void)
{
  if (BSP_ACCELERO_Set_ODR(LSM6DSL_X_0_handle, SAMPLE_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set accelero FIFO decimation */
  if (BSP_ACCELERO_FIFO_Set_Decimation_Ext(LSM6DSL_X_0_handle,
      LSM6DSL_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO ODR to highest value */
  if (BSP_ACCELERO_FIFO_Set_ODR_Value_Ext(LSM6DSL_X_0_handle, LSM6DSL_FIFO_MAX_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO_FULL on INT1 */
  if (BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext(LSM6DSL_X_0_handle,
      LSM6DSL_ACC_GYRO_INT1_FULL_FLAG_ENABLED) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO watermark */
  if (BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext( LSM6DSL_X_0_handle, FIFO_WATERMARK ) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO depth to be limited to watermark threshold level  */
  if (BSP_ACCELERO_FIFO_Set_Stop_On_Fth_Ext( LSM6DSL_X_0_handle,
      LSM6DSL_ACC_GYRO_STOP_ON_FTH_ENABLED ) == COMPONENT_ERROR)
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
  if (BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext(LSM6DSL_X_0_handle, &samplesToRead) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  sprintf(dataOut, "\r\n%d samples in FIFO.\r\n\r\nStarted downloading data from FIFO ...\r\n", samplesToRead);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  HAL_Delay(1000);

  sprintf(dataOut, "\r\n[DATA ##]  ACC_X  ACC_Y  ACC_Z  [mg]\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  /* 'samplesToRead' actually contains number of words in FIFO but each FIFO sample (data set) consists of 3 words
  so the 'samplesToRead' has to be divided by 3 */
  samplesToRead /= 3;

  for (i = 0; i < samplesToRead; i++)
  {
    if (LSM6DSL_Read_Single_FIFO_Pattern_Cycle(i) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  if ( samplesToRead > SAMPLE_LIST_MAX )
  {
    sprintf(dataOut, "\r\nSample list limited to: %d\r\n", SAMPLE_LIST_MAX);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  }

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
  int32_t acceleration = 0;
  int32_t acc_x = 0, acc_y = 0, acc_z = 0;
  int i = 0;

  /* Read one whole FIFO pattern cycle. Pattern: XLx, XLy, XLz */
  for (i = 0; i <= 2; i++)
  {
    /* Read FIFO pattern number */
    if (BSP_ACCELERO_FIFO_Get_Pattern_Ext(LSM6DSL_X_0_handle, &pattern) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }

    /* Read single FIFO data (acceleration in one axis) */
    if (BSP_ACCELERO_FIFO_Get_Axis_Ext(LSM6DSL_X_0_handle, &acceleration) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }

    /* Decide which axis has been read from FIFO based on pattern number */
    switch(pattern)
    {
      case PATTERN_ACC_X_AXIS:
        acc_x = acceleration;
        break;

      case PATTERN_ACC_Y_AXIS:
        acc_y = acceleration;
        break;

      case PATTERN_ACC_Z_AXIS:
        acc_z = acceleration;
        break;

      default:
        return COMPONENT_ERROR;
    }
  }

  if ( sampleIndex < SAMPLE_LIST_MAX )
  {
    sprintf(dataOut, "[DATA %02d]  %5ld  %5ld  %5ld\r\n", sampleIndex + 1, acc_x, acc_y, acc_z);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  }

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
      button_pressed = 1;
    }
  }

  /* FIFO full (available only for LSM6DSL sensor) */
  else if (GPIO_Pin == LSM6DSL_INT1_O_PIN)
  {
    mems_int1_detected = 1;
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
