#include "imu_task.h"
#include "hwt101.h"
#include "jy901s.h"
#include "debug_uart_task.h"
#include "robot_config.h"

#include <string.h>

#define IMU_FRAME_SIZE 11U
#define IMU_TX_TIMEOUT_MS 20U
#define IMU_RX_RETRY_MS 100U
#if ROBOT_IMU_PROBE_ATTEMPTS < 1U || ROBOT_IMU_PROBE_ATTEMPTS > 255U
#error "IMU probe attempt count must fit a nonzero byte"
#endif

static UART_HandleTypeDef *imu_uart;
static volatile IMU_Source imu_source;
static uint8_t imu_probe_rx_byte;
static uint8_t imu_probe_frame[IMU_FRAME_SIZE];
static uint8_t imu_probe_index;
static volatile bool imu_probe_rx_armed;
static volatile bool imu_query_pending;
static volatile bool imu_has_version;
static volatile uint32_t imu_packed_version;
static volatile bool imu_hwt_identified;
static volatile bool imu_standard_sensor_seen;
static uint8_t imu_probe_attempts;
static uint32_t imu_next_probe_tick;
static uint32_t imu_next_rx_retry_tick;
static uint32_t imu_next_debug_tick;

static bool IMU_IsHwtVersion(uint32_t packed)
{
  uint32_t product = (packed >> 14U) & 0x1FFFFUL;
  /* Official HWT101 documentation names1003X.*.* and gives10173.1.1
     (89EF4101) as its packed-version example. Do not classify arbitrary
     55/53 angle frames or a legacy VERSION+CCBW pair as HWT101. */
  return (packed & 0x80000000UL) != 0U &&
      (product == 10173U || (product >= 10030U && product <= 10039U) ||
       (ROBOT_IMU_HWT_EXTRA_PRODUCT_ID != 0U &&
        product == ROBOT_IMU_HWT_EXTRA_PRODUCT_ID));
}

static void IMU_ProcessProbeByte(uint8_t byte)
{
  if (imu_probe_index == 0U && byte != 0x55U) return;
  imu_probe_frame[imu_probe_index++] = byte;
  if (imu_probe_index == 2U && (byte < 0x50U || byte > 0x5FU))
  {
    imu_probe_index = byte == 0x55U ? 1U : 0U;
    imu_probe_frame[0] = 0x55U;
    return;
  }
  if (imu_probe_index < IMU_FRAME_SIZE) return;
  uint8_t checksum = 0U;
  for (uint8_t index = 0U; index < IMU_FRAME_SIZE - 1U; ++index)
    checksum = (uint8_t)(checksum + imu_probe_frame[index]);
  if (checksum == imu_probe_frame[IMU_FRAME_SIZE - 1U])
  {
    uint8_t type = imu_probe_frame[1];
    if (type == 0x5FU && imu_query_pending)
    {
      uint32_t packed = (uint32_t)imu_probe_frame[2] |
          ((uint32_t)imu_probe_frame[3] << 8U) |
          ((uint32_t)imu_probe_frame[4] << 16U) |
          ((uint32_t)imu_probe_frame[5] << 24U);
      imu_has_version = true;
      if (IMU_IsHwtVersion(packed))
      {
        imu_packed_version = packed;
        imu_hwt_identified = true;
      }
      else if (!imu_hwt_identified) imu_packed_version = packed;
    }
    /* HWT101 does not output acceleration/magnetic frames or roll/pitch.
       Conflicting evidence must never trigger an HWT-only write command. */
    if (type == 0x51U || type == 0x54U ||
        (type == 0x53U && (imu_probe_frame[2] != 0U ||
         imu_probe_frame[3] != 0U || imu_probe_frame[4] != 0U ||
         imu_probe_frame[5] != 0U))) imu_standard_sensor_seen = true;
    imu_probe_index = 0U;
    return;
  }
  for (uint8_t index = 1U; index < IMU_FRAME_SIZE; ++index)
  {
    if (imu_probe_frame[index] == 0x55U &&
        (index == IMU_FRAME_SIZE - 1U ||
         (imu_probe_frame[index + 1U] >= 0x50U &&
          imu_probe_frame[index + 1U] <= 0x5FU)))
    {
      imu_probe_index = IMU_FRAME_SIZE - index;
      memmove(imu_probe_frame, &imu_probe_frame[index], imu_probe_index);
      return;
    }
  }
  imu_probe_index = 0U;
}

HAL_StatusTypeDef IMU_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance == NULL ||
      huart->Init.BaudRate != HWT101_BAUD_RATE) return HAL_ERROR;
  if (imu_uart != NULL) return HAL_BUSY;
  imu_uart = huart;
  imu_probe_index = 0U;
  imu_probe_attempts = 0U;
  imu_query_pending = false;
  imu_has_version = false;
  imu_hwt_identified = false;
  imu_standard_sensor_seen = false;
  imu_packed_version = 0U;
  uint32_t now = HAL_GetTick();
  imu_next_probe_tick = now + ROBOT_IMU_POWER_SETTLE_MS;
  imu_next_rx_retry_tick = now;
  imu_next_debug_tick = now;
  imu_source = IMU_PROBING;
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart, &imu_probe_rx_byte, 1U);
  imu_probe_rx_armed = status == HAL_OK;
  if (status != HAL_OK) imu_source = IMU_INIT_FAILED;
  return status;
}

void IMU_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart != imu_uart) return;
  switch (imu_source)
  {
    case IMU_HWT101: HWT101_UART_RxCpltCallback(huart); break;
    case IMU_JY901S: JY901S_UART_RxCpltCallback(huart); break;
    case IMU_PROBING:
      IMU_ProcessProbeByte(imu_probe_rx_byte);
      imu_probe_rx_armed = HAL_UART_Receive_IT(huart, &imu_probe_rx_byte, 1U) == HAL_OK;
      break;
    default: break;
  }
}

void IMU_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart != imu_uart) return;
  switch (imu_source)
  {
    case IMU_HWT101: HWT101_UART_ErrorCallback(huart); break;
    case IMU_JY901S: JY901S_UART_ErrorCallback(huart); break;
    case IMU_PROBING:
      imu_probe_index = 0U;
      imu_has_version = false;
      imu_hwt_identified = false;
      imu_query_pending = false;
      (void)HAL_UART_AbortReceive(huart);
      imu_probe_rx_armed = HAL_UART_Receive_IT(huart, &imu_probe_rx_byte, 1U) == HAL_OK;
      break;
    default: break;
  }
}

static void IMU_SelectDriver(bool select_hwt)
{
  HAL_StatusTypeDef status;
  uint32_t version;
  bool version_seen;
  uint32_t primask = __get_PRIMASK();
  /* End the probe's in-flight RX before installing one driver's RX buffer.
     UART4 uses IT, not DMA, so AbortReceive has no DMA blocking wait here. */
  __disable_irq();
  /* A valid HWT reply may arrive exactly at the fallback deadline. */
  if (imu_hwt_identified) select_hwt = true;
  version = imu_packed_version;
  version_seen = imu_has_version;
  if (select_hwt && imu_standard_sensor_seen)
  {
    imu_source = IMU_INIT_FAILED;
    if (primask == 0U) __enable_irq();
    (void)DebugUart_Log("[IMU] CONFLICT STOP NO ZERO\r\n");
    return;
  }
  status = HAL_UART_AbortReceive(imu_uart);
  imu_query_pending = false;
  imu_probe_index = 0U;
  if (status == HAL_OK)
    status = select_hwt ? HWT101_Init(imu_uart) : JY901S_Init(imu_uart);
  imu_source = status == HAL_OK
      ? (select_hwt ? IMU_HWT101 : IMU_JY901S) : IMU_INIT_FAILED;
  if (primask == 0U) __enable_irq();
  imu_next_debug_tick = HAL_GetTick();
  (void)DebugUart_Logf("[IMU] src=%s hal=%u seen=%u ver=%08lX pid=%lu\r\n",
      select_hwt ? "HWT101" : "JY901S", (unsigned)status, version_seen ? 1U : 0U,
      (unsigned long)version, (unsigned long)((version >> 14U) & 0x1FFFFUL));
  if (!select_hwt && status == HAL_OK)
    (void)DebugUart_Log("[IMU] JY COMPAT; HWT UNKNOWN\r\n");
}

void IMU_Service(uint32_t now)
{
  if (imu_source == IMU_HWT101) { HWT101_Service(now); return; }
  if (imu_source == IMU_JY901S) { JY901S_Service(now); return; }
  if (imu_source != IMU_PROBING) return;
  if (!imu_probe_rx_armed && (int32_t)(now - imu_next_rx_retry_tick) >= 0)
  {
    imu_next_rx_retry_tick = now + IMU_RX_RETRY_MS;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!imu_probe_rx_armed)
      imu_probe_rx_armed = HAL_UART_Receive_IT(imu_uart, &imu_probe_rx_byte, 1U) == HAL_OK;
    if (primask == 0U) __enable_irq();
  }
  if (imu_hwt_identified)
  {
    IMU_SelectDriver(true);
    return;
  }
  if ((int32_t)(now - imu_next_probe_tick) < 0) return;
  if (imu_probe_attempts >= ROBOT_IMU_PROBE_ATTEMPTS)
  {
    IMU_SelectDriver(false);
    return;
  }
  /* Read-only VERSIONL/VERSIONH query. Never send the HWT76 zero command
     simply because a JY901S produced the same common53 angle frame. */
  uint8_t query[5] = {0xFFU, 0xAAU, 0x27U, 0x2EU, 0x00U};
  ++imu_probe_attempts;
  imu_query_pending = true; /* Arm before TX: a response may arrive in TX. */
  HAL_StatusTypeDef status = HAL_UART_Transmit(imu_uart, query, sizeof(query), IMU_TX_TIMEOUT_MS);
  if (status != HAL_OK) imu_query_pending = false;
  imu_next_probe_tick = HAL_GetTick() + ROBOT_IMU_PROBE_INTERVAL_MS;
  (void)DebugUart_Logf("[IMU] PROBE=%u/%u TX27/2E hal=%u\r\n",
      (unsigned)imu_probe_attempts, (unsigned)ROBOT_IMU_PROBE_ATTEMPTS, (unsigned)status);
}

IMU_Source IMU_GetSource(void) { return imu_source; }

bool IMU_GetYaw(IMU_Yaw *yaw)
{
  if (yaw == NULL) return false;
  if (imu_source == IMU_HWT101)
  {
    HWT101_Yaw sample;
    if (!HWT101_GetYaw(&sample)) return false;
    yaw->yaw_cdeg = sample.yaw_cdeg;
    yaw->sequence = sample.sequence;
    yaw->update_tick = sample.update_tick;
    return true;
  }
  if (imu_source == IMU_JY901S)
  {
    JY901S_Attitude sample;
    if (!JY901S_GetAttitude(&sample)) return false;
    yaw->yaw_cdeg = sample.yaw_cdeg;
    yaw->sequence = sample.sequence;
    yaw->update_tick = sample.update_tick;
    return true;
  }
  return false;
}

void IMU_DebugYaw(uint32_t now)
{
  if (imu_source == IMU_HWT101) { HWT101_DebugYaw(now); return; }
  if ((int32_t)(now - imu_next_debug_tick) < 0) return;
  imu_next_debug_tick = now + ROBOT_DEBUG_PERIOD_MS;
  IMU_Yaw sample;
  const char *source = imu_source == IMU_JY901S ? "JY901S" :
      (imu_source == IMU_PROBING ? "PROBING" : "INIT_FAILED");
  if (!IMU_GetYaw(&sample))
  {
    (void)DebugUart_Logf("[IMU] src=%s NO DATA pid=%lu seen=%u\r\n",
        source, (unsigned long)((imu_packed_version >> 14U) & 0x1FFFFUL), imu_has_version ? 1U : 0U);
    return;
  }
  int32_t wrapped = sample.yaw_cdeg < 0 ? sample.yaw_cdeg + 36000L : sample.yaw_cdeg;
  (void)DebugUart_Logf("[IMU] src=%s yaw=%ld.%02ld q=%lu age=%lu\r\n",
      source, (long)(wrapped / 100L), (long)(wrapped % 100L),
      (unsigned long)sample.sequence, (unsigned long)(now - sample.update_tick));
}
