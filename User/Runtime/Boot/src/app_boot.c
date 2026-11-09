#include "app_boot.h"

#include "M2006.h"
#include "M2006_Speed.h"
#include "actuator_task.h"
#include "boot_init.h"
#include "can.h"
#include "gpio.h"
#include "i2c.h"
#include "imu_task.h"
#include "maixcam_task.h"
#include "oled_task.h"
#include "reset_reason.h"
#include "robot_config.h"
#include "usart.h"

void AppBoot_BeforeHal(void)
{
  ResetReason_Capture();
  BootInit_Begin();
}

void AppBoot_AfterHal(void)
{
  /* HSI is already running: publish progress before HSE/CAN/servo startup.
     USART2 is configured again after the APB1 clock changes. */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  BootInit_SetUartReady(true);
  {
    ResetReasonSnapshot snapshot;
    if (ResetReason_GetSnapshot(&snapshot))
    {
      BootInit_Logf("[BOOT] rst=%08lX HSI UART2=115200\r\n",
                    (unsigned long)snapshot.raw_flags);
    }
  }
  if (BootInit_CheckHalTick() != HAL_OK)
  {
    Error_Handler();
  }
}

void AppBoot_AfterClock(void)
{
  if (BootInit_CheckHalTick() != HAL_OK)
  {
    Error_Handler();
  }
  BootInit_SetStage("PERIPHERALS");
}

void AppBoot_AfterPeripherals(void)
{
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
  BootInit_SetStage("MAIXCAM_RESET_TX");
  {
    /* main runs once per MCU boot, before any mission command can be sent.
       Keep a failed notification bounded; no retry or camera ACK wait. */
    HAL_StatusTypeDef status = MaixCam_SendCommand(MAIXCAM_COMMAND_MCU_RESET);
    BootInit_Logf("[BOOT] TXFE hal=%u\r\n", (unsigned int)status);
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
}

void AppBoot_BeforeScheduler(void)
{
  BootInit_SetStage("KERNEL_START");
  BootInit_Logf("[BOOT] RTOS ms=%lu\r\n",
                (unsigned long)HAL_GetTick());
  BootInit_SetUartReady(false);
}

void AppBoot_Fault(void)
{
  __disable_irq();
  if (huart2.Instance == USART2) BootInit_SetUartReady(true);
  SpeedLoop_EmergencyStop();
  if ((hcan1.Instance == CAN1) && (hcan1.State == HAL_CAN_STATE_LISTENING))
  {
    (void)set_moto_current(&hcan1, 0, 0, 0, 0);
  }
  BootInit_FaultLoop();
}
