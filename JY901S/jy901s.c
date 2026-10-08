#include "jy901s.h"
#include <string.h>

#define JY901S_FRAME_SIZE 11U
#define JY901S_FRAME_HEAD 0x55U
#define JY901S_FRAME_ANGLE 0x53U

static UART_HandleTypeDef *jy901s_uart;
static uint8_t jy901s_rx_byte;
static uint8_t jy901s_frame[JY901S_FRAME_SIZE];
static uint8_t jy901s_frame_index;
static volatile JY901S_Attitude jy901s_attitude;
static volatile uint8_t jy901s_has_attitude;
static volatile bool jy901s_rx_armed;
static uint32_t jy901s_rx_retry_tick;

static int16_t JY901S_ReadS16(const uint8_t *bytes)
{
  return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

static int32_t JY901S_RawToCentiDegrees(int16_t raw)
{
  return ((int32_t)raw * 18000L) / 32768L;
}

static void JY901S_ProcessByte(uint8_t byte)
{
  uint8_t checksum = 0U;
  uint8_t index;

  if (jy901s_frame_index == 0U && byte != JY901S_FRAME_HEAD) return;
  jy901s_frame[jy901s_frame_index++] = byte;
  if (jy901s_frame_index == 2U && (byte < 0x50U || byte > 0x5FU))
  {
    jy901s_frame_index = byte == JY901S_FRAME_HEAD ? 1U : 0U;
    jy901s_frame[0] = JY901S_FRAME_HEAD;
    return;
  }
  if (jy901s_frame_index < JY901S_FRAME_SIZE)
  {
    return;
  }

  for (index = 0U; index < (JY901S_FRAME_SIZE - 1U); ++index)
  {
    checksum = (uint8_t)(checksum + jy901s_frame[index]);
  }

  if (checksum == jy901s_frame[JY901S_FRAME_SIZE - 1U])
  {
    if (jy901s_frame[1] == JY901S_FRAME_ANGLE)
    {
      jy901s_attitude.roll_cdeg = JY901S_RawToCentiDegrees(JY901S_ReadS16(&jy901s_frame[2]));
      jy901s_attitude.pitch_cdeg = JY901S_RawToCentiDegrees(JY901S_ReadS16(&jy901s_frame[4]));
      jy901s_attitude.yaw_cdeg = JY901S_RawToCentiDegrees(JY901S_ReadS16(&jy901s_frame[6]));
      jy901s_attitude.update_tick = HAL_GetTick();
      ++jy901s_attitude.sequence;
      jy901s_has_attitude = 1U;
    }
    jy901s_frame_index = 0U;
    return;
  }
  /* Preserve a new possible frame embedded in a corrupted candidate. */
  for (index = 1U; index < JY901S_FRAME_SIZE; ++index)
  {
    if (jy901s_frame[index] == JY901S_FRAME_HEAD &&
        (index == JY901S_FRAME_SIZE - 1U ||
         (jy901s_frame[index + 1U] >= 0x50U && jy901s_frame[index + 1U] <= 0x5FU)))
    {
      jy901s_frame_index = JY901S_FRAME_SIZE - index;
      memmove(jy901s_frame, &jy901s_frame[index], jy901s_frame_index);
      return;
    }
  }
  jy901s_frame_index = 0U;
}

HAL_StatusTypeDef JY901S_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance == NULL || huart->Init.BaudRate != 115200U)
  {
    return HAL_ERROR;
  }
  if (jy901s_uart != NULL) return HAL_BUSY;

  jy901s_uart = huart;
  jy901s_frame_index = 0U;
  jy901s_has_attitude = 0U;
  jy901s_attitude.sequence = 0U;
  jy901s_attitude.update_tick = 0U;
  jy901s_rx_retry_tick = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(jy901s_uart, &jy901s_rx_byte, 1U);
  jy901s_rx_armed = status == HAL_OK;
  return status;
}

void JY901S_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != jy901s_uart))
  {
    return;
  }

  JY901S_ProcessByte(jy901s_rx_byte);
  jy901s_rx_armed = HAL_UART_Receive_IT(jy901s_uart, &jy901s_rx_byte, 1U) == HAL_OK;
}

void JY901S_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart == jy901s_uart))
  {
    jy901s_frame_index = 0U;
    jy901s_has_attitude = 0U;
    (void)HAL_UART_AbortReceive(jy901s_uart);
    jy901s_rx_armed = HAL_UART_Receive_IT(jy901s_uart, &jy901s_rx_byte, 1U) == HAL_OK;
  }
}

void JY901S_Service(uint32_t now)
{
  if (jy901s_uart == NULL || jy901s_rx_armed ||
      (int32_t)(now - jy901s_rx_retry_tick) < 0) return;
  jy901s_rx_retry_tick = now + 100U;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!jy901s_rx_armed)
    jy901s_rx_armed = HAL_UART_Receive_IT(jy901s_uart, &jy901s_rx_byte, 1U) == HAL_OK;
  if (primask == 0U) __enable_irq();
}

bool JY901S_GetAttitude(JY901S_Attitude *attitude)
{
  uint32_t primask;

  if (attitude == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (jy901s_has_attitude == 0U || !jy901s_rx_armed)
  {
    if (primask == 0U) __enable_irq();
    return false;
  }
  *attitude = jy901s_attitude;
  if (primask == 0U) __enable_irq();

  return ((uint32_t)(HAL_GetTick() - attitude->update_tick) <= JY901S_ATTITUDE_TIMEOUT_MS);
}
