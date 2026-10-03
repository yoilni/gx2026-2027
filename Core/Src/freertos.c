/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_task.h"
#include "M2006_task.h"
#include "oled_task.h"
#include "mission_task.h"
#include "debug_uart_task.h"
#include "start_button_task.h"
#include "robot_config.h"
#include "servo_test_task.h"
#include "motor_test_task.h"

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
/* USER CODE BEGIN Variables */
#if !ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED
static osThreadId_t motorTaskHandle;
#endif
#if ROBOT_OLED_ENABLED
static osThreadId_t oledTaskHandle;
#endif

#if !ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED
static const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityHigh,
};
#endif

#if ROBOT_OLED_ENABLED
static const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityLow,
};
#endif

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
#if !ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
#endif
#if ROBOT_OLED_ENABLED
  oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);
  if (oledTaskHandle == NULL)
  {
    Error_Handler();
  }
#endif
  if ((defaultTaskHandle == NULL) || !DebugUartTask_Create(&huart2))
  {
    Error_Handler();
  }
#if ROBOT_MG90_SPEED_TEST_ENABLED
  if (!ServoTestTask_Create())
  {
    Error_Handler();
  }
#elif ROBOT_STRAIGHT_TEST_ENABLED
  if ((motorTaskHandle == NULL) || !MotorTestTask_Create())
  {
    Error_Handler();
  }
#elif !ROBOT_FRAME_DOWN_TEST_ENABLED
  if ((motorTaskHandle == NULL) || !MissionTask_Create() || !StartButtonTask_Create())
  {
    Error_Handler();
  }
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  LED_Task(argument);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

