#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "hwt101.c"
#include "jy901s.c"
#include "imu_task.c"

/* Exercise the real selector and both drivers, including the single shared
   HAL receive buffer. No model-specific driver is stubbed out. */
static uint32_t now_ms, test_primask;
static UART_HandleTypeDef sensor_uart = {(void *)1, {115200}};
static UART_HandleTypeDef other_uart = {(void *)2, {115200}};
static uint8_t tx_packets[32][5];
static uint32_t tx_ticks[32];
static unsigned tx_count, rx_count, abort_count, log_count;
static uint8_t *rx_destination;
static HAL_StatusTypeDef tx_result, rx_result, abort_result;
static bool reply_in_tx, reply_at_critical_entry;
static char last_log[200];

static void version_frame(uint32_t packed);
uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return test_primask; }
void __disable_irq(void)
{
  /* Model a completed version RX interrupt immediately before selection's
     critical section. The fallback must recheck HWT priority inside it. */
  if (reply_at_critical_entry && test_primask == 0U)
  {
    reply_at_critical_entry = false;
    version_frame(0x89EF4101UL);
  }
  test_primask = 1U;
}
void __enable_irq(void) { test_primask = 0U; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
  assert(u == &sensor_uart && n == 1U);
  ++rx_count;
  if (rx_result != HAL_OK) return rx_result;
  /* Rearming before the old RX has completed/aborted would contend. */
  assert(rx_destination == NULL);
  rx_destination = b;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{
  assert(u == &sensor_uart);
  ++abort_count;
  if (abort_result == HAL_OK) rx_destination = NULL;
  return abort_result;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b,
                                   uint16_t n, uint32_t timeout)
{
  assert(u == &sensor_uart && n == 5U && timeout == 20U && tx_count < 32U);
  memcpy(tx_packets[tx_count], b, 5U);
  tx_ticks[tx_count++] = now_ms;
  /* A fast module may respond while polling TX is still completing. */
  if (reply_in_tx && b[2] == 0x27U) version_frame(0x89EF4101UL);
  return tx_result;
}
bool DebugUart_Log(const char *text)
{
  ++log_count;
  snprintf(last_log, sizeof last_log, "%s", text);
  return true;
}
bool DebugUart_Logf(const char *format, ...)
{
  ++log_count;
  va_list args;
  va_start(args, format);
  vsnprintf(last_log, sizeof last_log, format, args);
  va_end(args);
  return true;
}
static void reset(uint32_t start)
{
  imu_uart = hwt101_uart = jy901s_uart = NULL;
  imu_source = IMU_UNINITIALIZED;
  hwt101_startup = HWT101_UNINITIALIZED;
  now_ms = start;
  test_primask = 0U;
  tx_count = rx_count = abort_count = log_count = 0U;
  tx_result = rx_result = abort_result = HAL_OK;
  rx_destination = NULL;
  reply_in_tx = reply_at_critical_entry = false;
  sensor_uart.Init.BaudRate = 115200U;
}
static void setup(uint32_t start)
{
  reset(start);
  assert(IMU_Init(&sensor_uart) == HAL_OK);
  assert(IMU_GetSource() == IMU_PROBING);
}
static void service(uint32_t at) { now_ms = at; IMU_Service(at); }
static void rx(uint8_t byte)
{
  assert(rx_destination != NULL);
  *rx_destination = byte;
  rx_destination = NULL;
  unsigned before = rx_count;
  IMU_UART_RxCpltCallback(&sensor_uart);
  assert(rx_count == before + 1U);
}
static void send_frame(uint8_t *bytes)
{
  bytes[10] = 0U;
  for (unsigned i = 0U; i < 10U; ++i) bytes[10] += bytes[i];
  for (unsigned i = 0U; i < 11U; ++i) rx(bytes[i]);
}
static void version_frame(uint32_t packed)
{
  uint8_t bytes[11] = {0x55U, 0x5FU};
  for (unsigned i = 0U; i < 4U; ++i) bytes[2U + i] = packed >> (8U * i);
  send_frame(bytes);
}
static void angle_frame(int16_t raw, uint16_t version)
{
  uint8_t bytes[11] = {0x55U, 0x53U};
  bytes[6] = (uint16_t)raw;
  bytes[7] = (uint16_t)raw >> 8U;
  bytes[8] = version;
  bytes[9] = version >> 8U;
  send_frame(bytes);
}
static void other_frame(uint8_t type)
{
  uint8_t bytes[11] = {0x55U, type};
  send_frame(bytes);
}
static void fallback(uint32_t start)
{
  setup(start);
  IMU_Yaw yaw;
  /* Unsolicited versions before a query are not identification evidence. */
  version_frame(0x89EF4101UL);
  angle_frame(16384, 1U);
  assert(!IMU_GetYaw(&yaw) && !imu_hwt_identified);
  service(start + 499U); assert(tx_count == 0U);
  service(start + 500U); assert(tx_count == 1U);
  service(start + 799U); assert(tx_count == 1U);
  service(start + 800U); assert(tx_count == 2U);
  service(start + 1099U); assert(tx_count == 2U);
  service(start + 1100U); assert(tx_count == 3U);
  service(start + 1399U); assert(IMU_GetSource() == IMU_PROBING);
  // A partial probe angle may not become a fresh JY sample after handoff.
  rx(0x55U); rx(0x53U); rx(0U);
  service(start + 1400U);
  assert(IMU_GetSource() == IMU_JY901S && tx_count == 3U && abort_count == 1U);
  assert(hwt101_uart == NULL && jy901s_uart == &sensor_uart);
  assert(rx_destination == &jy901s_rx_byte && !IMU_GetYaw(&yaw));
  const uint8_t expected[5] = {0xFFU, 0xAAU, 0x27U, 0x2EU, 0U};
  for (unsigned i = 0U; i < 3U; ++i)
  {
    assert(memcmp(tx_packets[i], expected, 5U) == 0);
    assert(tx_ticks[i] == start + 500U + 300U * i);
  }
  for (unsigned i = 0U; i < 8U; ++i) rx(0U);
  assert(!IMU_GetYaw(&yaw));
  angle_frame(-16384, 2U);
  assert(IMU_GetYaw(&yaw) && yaw.yaw_cdeg == -9000 && yaw.sequence == 1U);
  service(start + 1900U); assert(IMU_GetYaw(&yaw) && tx_count == 3U);
  service(start + 1901U); assert(!IMU_GetYaw(&yaw) && IMU_GetSource() == IMU_JY901S);
  // Runtime loss/late version cannot cause a sensor swap or hardware zero.
  version_frame(0x89EF4101UL);
  service(start + 10000U);
  assert(IMU_GetSource() == IMU_JY901S && tx_count == 3U);
}
static void hwt_boot(void)
{
  setup(0U);
  service(500U);
  version_frame(0x89EF4101UL);
  service(500U);
  assert(IMU_GetSource() == IMU_HWT101 && tx_count == 1U);
  assert(hwt101_uart == &sensor_uart && jy901s_uart == NULL);
  assert(rx_destination == &hwt101_rx_byte && test_primask == 0U);
  IMU_Yaw yaw;
  angle_frame(16384, 3U); assert(!IMU_GetYaw(&yaw));
  service(999U); assert(tx_count == 1U);
  service(1000U); assert(tx_count == 2U);
  service(1200U); assert(tx_count == 3U);
  service(1700U); assert(tx_count == 4U);
  const uint8_t expected[3][5] = {
    {0xFFU,0xAAU,0x69U,0x88U,0xB5U},
    {0xFFU,0xAAU,0x76U,0U,0U},
    {0xFFU,0xAAU,0U,0U,0U}};
  assert(memcmp(expected, &tx_packets[1], sizeof expected) == 0);
  angle_frame(16384, 4U); assert(!IMU_GetYaw(&yaw));
  rx(0x55U); rx(0x53U); rx(0U);
  service(1900U); assert(HWT101_GetStartupState() == HWT101_READY);
  for (unsigned i = 0U; i < 8U; ++i) rx(0U);
  assert(!IMU_GetYaw(&yaw));
  angle_frame(0, 5U); assert(IMU_GetYaw(&yaw) && yaw.yaw_cdeg == 0);
}
static void parser_and_error_checks(bool hwt)
{
  IMU_Yaw yaw;
  if (hwt) hwt_boot(); else fallback(0U);
  angle_frame(16384, 6U); assert(IMU_GetYaw(&yaw) && yaw.yaw_cdeg == 9000);
  uint32_t sequence = yaw.sequence;
  other_frame(0x52U); assert(IMU_GetYaw(&yaw) && yaw.sequence == sequence);
  uint8_t bad[11] = {0x55U,0x53U,0U,0U,0U,0U,0U,0x40U,0U,0U,0U};
  for (unsigned i = 0U; i < 11U; ++i) rx(bad[i]);
  assert(IMU_GetYaw(&yaw) && yaw.sequence == sequence);
  // Truncated candidate followed by a full frame, then checksum55 boundary.
  rx(0x55U); rx(0x53U); rx(0U); angle_frame(-16384, 7U);
  assert(IMU_GetYaw(&yaw) && yaw.yaw_cdeg == -9000);
  angle_frame(0, 0xADU); angle_frame(32767, 8U);
  assert(IMU_GetYaw(&yaw) && yaw.yaw_cdeg == 17999);
  angle_frame(-32768, 9U); assert(IMU_GetYaw(&yaw) && yaw.yaw_cdeg == -18000);
  test_primask = 1U; assert(IMU_GetYaw(&yaw) && test_primask == 1U);
  test_primask = 0U;
  unsigned count = rx_count, aborts = abort_count;
  IMU_UART_RxCpltCallback(&other_uart); IMU_UART_ErrorCallback(&other_uart);
  assert(rx_count == count && abort_count == aborts);
  IMU_UART_ErrorCallback(&sensor_uart);
  assert(!IMU_GetYaw(&yaw) && abort_count == aborts + 1U);
  angle_frame(0, 10U); assert(IMU_GetYaw(&yaw));
  // A failed ISR rearm makes cached yaw invalid; the task retries RX.
  rx_result = HAL_BUSY; rx(0x99U); assert(!IMU_GetYaw(&yaw));
  service(now_ms); assert(rx_destination == NULL);
  count = rx_count; rx_result = HAL_OK; service(now_ms + 99U);
  assert(rx_count == count); service(now_ms + 1U);
  assert(rx_destination != NULL);
  angle_frame(0, 11U); assert(IMU_GetYaw(&yaw));
  assert(IMU_GetSource() == (hwt ? IMU_HWT101 : IMU_JY901S));
  assert(tx_count == (hwt ? 4U : 3U));
  IMU_DebugYaw(now_ms); count = log_count;
  IMU_DebugYaw(now_ms + 749U); assert(log_count == count);
  now_ms += 750U; angle_frame(-16384, 12U); IMU_DebugYaw(now_ms);
  assert(log_count == count + 1U && strstr(last_log, "yaw=270.00"));
}
int main(void)
{
  IMU_Yaw yaw;
  reset(0U);
  assert(!IMU_GetYaw(NULL) && !IMU_GetYaw(&yaw));
  assert(IMU_Init(NULL) == HAL_ERROR);
  sensor_uart.Init.BaudRate = 9600U;
  assert(IMU_Init(&sensor_uart) == HAL_ERROR);
  setup(0U); assert(IMU_Init(&sensor_uart) == HAL_BUSY);
  fallback(0U);
  fallback(UINT32_MAX - 1000U);
  // All documented HWT product variants, not arbitrary legacy versions.
  for (uint32_t id = 10030U; id <= 10039U; ++id)
  {
    setup(0U); service(500U);
    version_frame(0x80000000UL | (id << 14U) | 0x108U);
    service(501U); assert(IMU_GetSource() == IMU_HWT101 && tx_count == 1U);
  }
  assert(IMU_IsHwtVersion(0x89EF4101UL));
  assert(!IMU_IsHwtVersion(0x09EF4101UL) && !IMU_IsHwtVersion(0x80001001UL));
  // Unknown product/version, ordinary roll-pitch frames: JY fallback, no76.
  setup(0U); service(500U); version_frame(0x80001001UL); other_frame(0x51U);
  service(800U); service(1100U); service(1400U);
  assert(IMU_GetSource() == IMU_JY901S && tx_count == 3U);
  // Garbage and invalid checksum may not select HWT; resync must recover.
  setup(0U); service(500U);
  const uint8_t corrupt[11] = {0x55U,0x5FU,1U,0x41U,0xEFU,0x89U,0U,0U,0U,0U,0U};
  for (unsigned i = 0U; i < 11U; ++i) rx(corrupt[i]);
  service(501U); assert(IMU_GetSource() == IMU_PROBING);
  rx(0x55U); rx(0x5FU); rx(0U); version_frame(0x89EF4101UL);
  service(502U); assert(IMU_GetSource() == IMU_HWT101);
  // Immediate response during query TX and late response at fallback entry.
  setup(0U); reply_in_tx = true; service(500U); service(501U);
  assert(IMU_GetSource() == IMU_HWT101 && tx_count == 1U);
  setup(0U); service(500U); service(800U); service(1100U);
  reply_at_critical_entry = true; service(1400U);
  assert(IMU_GetSource() == IMU_HWT101 && tx_count == 3U);
  // No HWT writes if the identity conflicts with observed standard frames.
  setup(0U); service(500U); version_frame(0x89EF4101UL); other_frame(0x54U);
  service(501U); assert(IMU_GetSource() == IMU_INIT_FAILED);
  assert(!IMU_GetYaw(&yaw) && tx_count == 1U && hwt101_uart == NULL);
  // No sensor/failed queries: bounded selection, but no valid motion angle.
  setup(0U); tx_result = HAL_TIMEOUT;
  service(500U); service(800U); service(1100U); service(1400U);
  assert(IMU_GetSource() == IMU_JY901S && !IMU_GetYaw(&yaw));
  service(10000U); assert(tx_count == 3U);
  // Initial RX, handoff abort, and selected driver RX failures stay safe.
  reset(0U); rx_result = HAL_ERROR; assert(IMU_Init(&sensor_uart) == HAL_ERROR);
  assert(IMU_GetSource() == IMU_INIT_FAILED && !IMU_GetYaw(&yaw));
  setup(0U); service(500U); service(800U); service(1100U);
  abort_result = HAL_ERROR; service(1400U);
  assert(IMU_GetSource() == IMU_INIT_FAILED && !IMU_GetYaw(&yaw));
  setup(0U); service(500U); service(800U); service(1100U);
  rx_result = HAL_ERROR; service(1400U);
  assert(IMU_GetSource() == IMU_INIT_FAILED && !IMU_GetYaw(&yaw));
  // A selected HWT zero failure is not evidence for swapping to a JY.
  setup(0U); service(500U); version_frame(0x89EF4101UL); service(501U);
  tx_result = HAL_TIMEOUT; service(1001U); service(10000U);
  assert(IMU_GetSource() == IMU_HWT101 && HWT101_GetStartupState() == HWT101_INIT_FAILED);
  assert(!IMU_GetYaw(&yaw) && jy901s_uart == NULL && tx_count == 2U);
  // Probe error/rearm does not extend the probe window or use cached evidence.
  setup(0U); service(500U); version_frame(0x89EF4101UL);
  IMU_UART_ErrorCallback(&sensor_uart); assert(!imu_hwt_identified);
  rx_result = HAL_BUSY; rx(0x99U); service(600U);
  rx_result = HAL_OK; service(699U); assert(rx_destination == NULL);
  service(700U); assert(rx_destination != NULL);
  service(800U); service(1100U); service(1400U);
  assert(IMU_GetSource() == IMU_JY901S);
  parser_and_error_checks(false);
  parser_and_error_checks(true);
  puts("PASS: IMU1400ms probing/wrap, HWT version priority, read-only JY fallback, no shared-RX contention, fresh-frame handoff, zero/failure safety, parser/UART recovery, locked source and750ms debug");
  return 0;
}
