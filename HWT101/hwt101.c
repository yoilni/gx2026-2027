#include "hwt101.h"
#include "debug_uart_task.h"
#include "robot_config.h"

#include <string.h>

#define HWT101_FRAME_SIZE          11U
#define HWT101_FRAME_HEAD          0x55U
#define HWT101_FRAME_ANGLE         0x53U
#define HWT101_TX_TIMEOUT_MS       20U
#define HWT101_RX_RETRY_MS        100U

static UART_HandleTypeDef *hwt101_uart;
static uint8_t hwt101_rx_byte;
static uint8_t hwt101_frame[HWT101_FRAME_SIZE];
static uint8_t hwt101_frame_index;
static volatile HWT101_Yaw hwt101_yaw;
static volatile bool hwt101_has_yaw;
static volatile bool hwt101_rx_armed;
static volatile HWT101_StartupState hwt101_startup;
static uint32_t hwt101_phase_tick;
static uint32_t hwt101_rx_retry_tick;
static uint32_t hwt101_next_debug_tick;
static volatile uint32_t hwt101_checksum_errors;
static volatile uint32_t hwt101_uart_errors;

static void HWT101_InvalidateYaw(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  hwt101_has_yaw = false;
  /* Discard an in-progress pre-zero frame as well as a cached old angle. */
  hwt101_frame_index = 0U;
  if (primask == 0U) __enable_irq();
}

static void HWT101_ProcessByte(uint8_t byte)
{
  uint8_t checksum = 0U;

  if (hwt101_frame_index == 0U && byte != HWT101_FRAME_HEAD) return;
  hwt101_frame[hwt101_frame_index++] = byte;
  if (hwt101_frame_index == 2U &&
      (byte < 0x50U || byte > 0x5FU))
  {
    hwt101_frame_index = byte == HWT101_FRAME_HEAD ? 1U : 0U;
    hwt101_frame[0] = HWT101_FRAME_HEAD;
    return;
  }
  if (hwt101_frame_index < HWT101_FRAME_SIZE) return;

  for (uint8_t index = 0U; index < HWT101_FRAME_SIZE - 1U; ++index)
    checksum = (uint8_t)(checksum + hwt101_frame[index]);

  if (checksum == hwt101_frame[HWT101_FRAME_SIZE - 1U])
  {
    if (hwt101_frame[1] == HWT101_FRAME_ANGLE &&
        hwt101_startup == HWT101_READY)
    {
      int16_t raw = (int16_t)((uint16_t)hwt101_frame[6] |
                             ((uint16_t)hwt101_frame[7] << 8U));
      hwt101_yaw.yaw_cdeg = ((int32_t)raw * 18000L) / 32768L;
      hwt101_yaw.version = (uint16_t)hwt101_frame[8] |
                          ((uint16_t)hwt101_frame[9] << 8U);
      hwt101_yaw.update_tick = HAL_GetTick();
      ++hwt101_yaw.sequence;
      hwt101_has_yaw = true;
    }
    /* A valid checksum equal to55 belongs to this frame, not the next one. */
    hwt101_frame_index = 0U;
    return;
  }

  ++hwt101_checksum_errors;
  /* Sliding resynchronization after dropped/inserted bytes: retain the next
     possible header, even when it is embedded in the rejected candidate. */
  for (uint8_t index = 1U; index < HWT101_FRAME_SIZE; ++index)
  {
    if (hwt101_frame[index] == HWT101_FRAME_HEAD &&
        (index == HWT101_FRAME_SIZE - 1U ||
         (hwt101_frame[index + 1U] >= 0x50U &&
          hwt101_frame[index + 1U] <= 0x5FU)))
    {
      hwt101_frame_index = HWT101_FRAME_SIZE - index;
      memmove(hwt101_frame, &hwt101_frame[index], hwt101_frame_index);
      return;
    }
  }
  hwt101_frame_index = 0U;
}

HAL_StatusTypeDef HWT101_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance == NULL ||
      huart->Init.BaudRate != HWT101_BAUD_RATE) return HAL_ERROR;
  /* Do not re-zero a running mission through a duplicate initialization. */
  if (hwt101_uart != NULL) return HAL_BUSY;

  hwt101_uart = huart;
  hwt101_frame_index = 0U;
  hwt101_has_yaw = false;
  hwt101_yaw.sequence = 0U;
  hwt101_yaw.update_tick = 0U;
  hwt101_checksum_errors = 0U;
  hwt101_uart_errors = 0U;
  hwt101_phase_tick = HAL_GetTick();
  hwt101_rx_retry_tick = hwt101_phase_tick;
  hwt101_next_debug_tick = hwt101_phase_tick;
  hwt101_startup = HWT101_WAIT_POWER;
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart, &hwt101_rx_byte, 1U);
  hwt101_rx_armed = status == HAL_OK;
  if (status != HAL_OK) hwt101_startup = HWT101_INIT_FAILED;
  return status;
}

void HWT101_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart != hwt101_uart) return;
  HWT101_ProcessByte(hwt101_rx_byte);
  hwt101_rx_armed = HAL_UART_Receive_IT(huart, &hwt101_rx_byte, 1U) == HAL_OK;
}

void HWT101_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart != hwt101_uart) return;
  ++hwt101_uart_errors;
  HWT101_InvalidateYaw();
  (void)HAL_UART_AbortReceive(huart);
  hwt101_rx_armed = HAL_UART_Receive_IT(huart, &hwt101_rx_byte, 1U) == HAL_OK;
}

static bool HWT101_SendRegister(uint8_t address, uint16_t value)
{
  uint8_t command[5] = {0xFFU, 0xAAU, address,
                        (uint8_t)value, (uint8_t)(value >> 8U)};
  HAL_StatusTypeDef status = HAL_UART_Transmit(hwt101_uart, command,
                                              sizeof(command), HWT101_TX_TIMEOUT_MS);
  (void)DebugUart_Logf("[HWT] TX %02X/%02X/%02X hal=%u\r\n",
      (unsigned)address, (unsigned)command[3], (unsigned)command[4], (unsigned)status);
  if (status != HAL_OK)
  {
    hwt101_startup = HWT101_INIT_FAILED;
    HWT101_InvalidateYaw();
  }
  return status == HAL_OK;
}

void HWT101_Service(uint32_t now)
{
  if (hwt101_uart == NULL) return;
  if (!hwt101_rx_armed && (int32_t)(now - hwt101_rx_retry_tick) >= 0)
  {
    hwt101_rx_retry_tick = now + HWT101_RX_RETRY_MS;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!hwt101_rx_armed)
      hwt101_rx_armed = HAL_UART_Receive_IT(hwt101_uart, &hwt101_rx_byte, 1U) == HAL_OK;
    if (primask == 0U) __enable_irq();
  }

  uint32_t elapsed = (uint32_t)(now - hwt101_phase_tick);
  switch (hwt101_startup)
  {
    case HWT101_WAIT_POWER:
      if (elapsed >= HWT101_POWER_SETTLE_MS && HWT101_SendRegister(0x69U, 0xB588U))
      {
        hwt101_phase_tick = HAL_GetTick();
        hwt101_startup = HWT101_WAIT_UNLOCK;
      }
      break;
    case HWT101_WAIT_UNLOCK:
      if (elapsed >= HWT101_UNLOCK_WAIT_MS && HWT101_SendRegister(0x76U, 0U))
      {
        HWT101_InvalidateYaw();
        hwt101_phase_tick = HAL_GetTick();
        hwt101_startup = HWT101_WAIT_ZERO;
      }
      break;
    case HWT101_WAIT_ZERO:
      if (elapsed >= HWT101_ZERO_WAIT_MS && HWT101_SendRegister(0x00U, 0U))
      {
        hwt101_phase_tick = HAL_GetTick();
        hwt101_startup = HWT101_WAIT_SAVE;
      }
      break;
    case HWT101_WAIT_SAVE:
      if (elapsed >= HWT101_SAVE_WAIT_MS)
      {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        hwt101_has_yaw = false;
        hwt101_frame_index = 0U;
        hwt101_startup = HWT101_READY;
        if (primask == 0U) __enable_irq();
        (void)DebugUart_Log("[HWT] ZERO/SAVE TX OK; NO ACK\r\n");
      }
      break;
    default:
      break;
  }
}

HWT101_StartupState HWT101_GetStartupState(void)
{
  return hwt101_startup;
}

bool HWT101_GetYaw(HWT101_Yaw *yaw)
{
  if (yaw == NULL) return false;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool valid = hwt101_has_yaw && hwt101_startup == HWT101_READY && hwt101_rx_armed;
  if (valid) *yaw = hwt101_yaw;
  if (primask == 0U) __enable_irq();
  return valid && (uint32_t)(HAL_GetTick() - yaw->update_tick) <= HWT101_YAW_TIMEOUT_MS;
}

void HWT101_DebugYaw(uint32_t now)
{
  if ((int32_t)(now - hwt101_next_debug_tick) < 0) return;
  hwt101_next_debug_tick = now + ROBOT_DEBUG_PERIOD_MS;
  HWT101_Yaw yaw;
  if (!HWT101_GetYaw(&yaw))
  {
    (void)DebugUart_Logf("[HWT] s=%u NO DATA rx=%u crc=%lu uart=%lu\r\n",
        (unsigned)hwt101_startup, hwt101_rx_armed ? 1U : 0U,
        (unsigned long)hwt101_checksum_errors, (unsigned long)hwt101_uart_errors);
    return;
  }
  int32_t wrapped = yaw.yaw_cdeg < 0 ? yaw.yaw_cdeg + 36000L : yaw.yaw_cdeg;
  (void)DebugUart_Logf("[HWT] yaw=%ld.%02ld q=%lu age=%lu ver=%04X\r\n",
      (long)(wrapped / 100L), (long)(wrapped % 100L), (unsigned long)yaw.sequence,
      (unsigned long)(now - yaw.update_tick), (unsigned)yaw.version);
}
