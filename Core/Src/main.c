/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "M2006.h"
#include "M2006_Speed.h"
#include "imu_task.h"
#include "oled_task.h"
#include "maixcam_task.h"
#include "reset_reason.h"
#include "actuator_task.h"
#include "robot_config.h"
#include "boot_init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  ResetReason_Capture();
  BootInit_Begin();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  if (HAL_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Init */
  /* HSI is already running: publish progress before HSE/CAN/servo startup.
     USART2 is configured again after the APB1 clock changes. */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  BootInit_SetUartReady(true);
  {
    ResetReasonSnapshot snapshot;
    if (ResetReason_GetSnapshot(&snapshot))
    {
      BootInit_Logf("[BOOT-EARLY] RESET flags=0x%08lX HSI, UART2=115200\r\n",
                    (unsigned long)snapshot.raw_flags);
    }
  }
  if (BootInit_CheckHalTick() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if (BootInit_CheckHalTick() != HAL_OK)
  {
    Error_Handler();
  }
  BootInit_SetStage("PERIPHERALS");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  BootInit_SetStage("GPIO");
  MX_GPIO_Init();
  BootInit_SetStage("ADC1");
  MX_ADC1_Init();
  MX_CAN1_Init();
  BootInit_SetStage("TIM2");
  MX_TIM2_Init();
  BootInit_SetStage("TIM3");
  MX_TIM3_Init();
  BootInit_SetStage("UART4");
  MX_UART4_Init();
  BootInit_SetStage("UART5");
  MX_UART5_Init();
  BootInit_SetStage("USART1");
  MX_USART1_UART_Init();
  BootInit_SetStage("USART2");
  MX_USART2_UART_Init();
  BootInit_SetStage("USART3");
  MX_USART3_UART_Init();
  BootInit_SetStage("USART6");
  MX_USART6_UART_Init();
  BootInit_SetStage("I2C2");
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  BootInit_SetStage("ACTUATOR");
  if (Actuator_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (BootInit_InitCan(&hcan1, true) != HAL_OK)
  {
    Error_Handler();
  }
  SpeedLoop_Init();
  BootInit_SetStage("IMU_AUTO_RX");
  HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  if (IMU_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  BootInit_SetStage("MAIXCAM_RX");
  HAL_NVIC_SetPriority(USART3_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  if (MaixCam_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
#if ROBOT_OLED_ENABLED
  BootInit_SetStage("OLED_INIT");
  (void)OLED_TaskInit(&hi2c2);
#else
  BootInit_SetStage("OLED_OFF");
  /* Also blank a still-powered display when only the MCU was reset. */
  (void)OLED_DisplayOff(&hi2c2);
#endif
  /* M2006 motor 1 rotates at 10 rpm at the output shaft. */
  SpeedLoop_SetMotorTarget(4U, 0.0f, 1000);
  BootInit_SetStage("KERNEL_INIT");
  /* USER CODE END 2 */

  /* Init scheduler */
  if (osKernelInitialize() != osOK)
  {
    Error_Handler();
  }

  /* Call init function for freertos objects (in cmsis_os2.c) */
  BootInit_SetStage("CREATE_TASKS");
  MX_FREERTOS_Init();

  /* Start scheduler */
  BootInit_SetStage("KERNEL_START");
  BootInit_Logf("[BOOT-INIT] READY, START FREERTOS tick=%lu\r\n",
                (unsigned long)HAL_GetTick());
  BootInit_SetUartReady(false);
  osKernelStart();
  /* A successful scheduler start never returns. */
  BootInit_SetUartReady(true);
  Error_Handler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (BootInit_ConfigOscillators(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (BootInit_ConfigClock(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  if (huart2.Instance == USART2) BootInit_SetUartReady(true);
  SpeedLoop_EmergencyStop();
  if ((hcan1.Instance == CAN1) && (hcan1.State == HAL_CAN_STATE_LISTENING))
  {
    (void)set_moto_current(&hcan1, 0, 0, 0, 0);
  }
  BootInit_FaultLoop();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
