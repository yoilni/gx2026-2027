#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "hwt101.c"

static uint32_t now_ms, primask;
static UART_HandleTypeDef imu_uart = {(void *)1, {115200}};
static UART_HandleTypeDef other_uart = {(void *)2, {115200}};
static uint8_t tx_packets[4][5];
static uint32_t tx_ticks[4];
static unsigned tx_count, rx_count, abort_count, log_count;
static uint8_t *rx_destination;
static HAL_StatusTypeDef tx_status, rx_status;
static char last_log[160];

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __enable_irq(void) { primask = 0; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
  assert(u == &imu_uart && n == 1); ++rx_count; rx_destination = b; return rx_status;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t n, uint32_t timeout)
{
  assert(u == &imu_uart && n == 5 && timeout == 20 && tx_count < 4);
  memcpy(tx_packets[tx_count], b, 5); tx_ticks[tx_count++] = now_ms;
  return tx_status;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ assert(u == &imu_uart); ++abort_count; return HAL_OK; }
bool DebugUart_Log(const char *text)
{ ++log_count; snprintf(last_log, sizeof last_log, "%s", text); return true; }
bool DebugUart_Logf(const char *format, ...)
{
  ++log_count; va_list args; va_start(args, format);
  vsnprintf(last_log, sizeof last_log, format, args); va_end(args); return true;
}
static void setup(uint32_t initial_tick)
{
  hwt101_uart = NULL; hwt101_startup = HWT101_UNINITIALIZED;
  now_ms = initial_tick; primask = 0; tx_count = rx_count = abort_count = log_count = 0;
  tx_status = rx_status = HAL_OK; imu_uart.Init.BaudRate = 115200;
  assert(HWT101_Init(&imu_uart) == HAL_OK);
}
static void service(uint32_t at) { now_ms = at; HWT101_Service(at); }
static void rx(uint8_t byte)
{
  *rx_destination = byte; unsigned before = rx_count;
  HWT101_UART_RxCpltCallback(&imu_uart); assert(rx_count == before + 1);
}
static void make_frame(uint8_t *frame, uint8_t type, int16_t raw, uint16_t version)
{
  memset(frame, 0, 11); frame[0] = 0x55; frame[1] = type;
  frame[6] = (uint8_t)raw; frame[7] = (uint16_t)raw >> 8;
  frame[8] = version; frame[9] = version >> 8;
  for (unsigned i = 0; i < 10; ++i) frame[10] += frame[i];
}
static void frame(uint8_t type, int16_t raw, uint16_t version)
{ uint8_t bytes[11]; make_frame(bytes, type, raw, version); for (unsigned i = 0; i < 11; ++i) rx(bytes[i]); }
static void boot(uint32_t start)
{
  setup(start); HWT101_Yaw yaw;
  assert(!HWT101_GetYaw(&yaw));
  frame(0x53, 16384, 123); assert(!HWT101_GetYaw(&yaw));
  service(start + 499); assert(tx_count == 0);
  service(start + 500); assert(tx_count == 1 && HWT101_GetStartupState() == HWT101_WAIT_UNLOCK);
  service(start + 699); assert(tx_count == 1);
  service(start + 700); assert(tx_count == 2 && HWT101_GetStartupState() == HWT101_WAIT_ZERO);
  service(start + 1199); assert(tx_count == 2);
  service(start + 1200); assert(tx_count == 3 && HWT101_GetStartupState() == HWT101_WAIT_SAVE);
  const uint8_t expected[3][5] = {{0xFF,0xAA,0x69,0x88,0xB5},{0xFF,0xAA,0x76,0,0},{0xFF,0xAA,0,0,0}};
  assert(memcmp(expected, tx_packets, sizeof expected) == 0);
  assert(tx_ticks[1] - tx_ticks[0] == 200 && tx_ticks[2] - tx_ticks[1] == 500);
  // A partial angle from before READY must not be published afterwards.
  rx(0x55); rx(0x53); rx(0); rx(0); rx(0); rx(0);
  service(start + 1399); assert(!HWT101_GetYaw(&yaw));
  service(start + 1400); assert(HWT101_GetStartupState() == HWT101_READY);
  for (unsigned i = 0; i < 5; ++i) rx(0);
  assert(!HWT101_GetYaw(&yaw));
  frame(0x53, 0, 0x1234); assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == 0 && yaw.version == 0x1234);
  assert(HWT101_Init(&imu_uart) == HAL_BUSY && tx_count == 3);
}
int main(void)
{
  HWT101_Yaw yaw;
  assert(HWT101_Init(NULL) == HAL_ERROR);
  imu_uart.Init.BaudRate = 9600; assert(HWT101_Init(&imu_uart) == HAL_ERROR);
  boot(0);
  frame(0x53, 16384, 3); assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == 9000);
  frame(0x53, -16384, 4); assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == -9000);
  frame(0x53, -32768, 5); assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == -18000);
  frame(0x53, 32767, 6); assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == 17999);
  uint32_t sequence = yaw.sequence, tick = yaw.update_tick;
  now_ms += 100; frame(0x52, 16384, 7);
  assert(HWT101_GetYaw(&yaw) && yaw.sequence == sequence && yaw.update_tick == tick);
  uint8_t bytes[11]; make_frame(bytes, 0x53, 1000, 8); bytes[10] ^= 1;
  for (unsigned i = 0; i < 11; ++i) rx(bytes[i]);
  assert(HWT101_GetYaw(&yaw) && yaw.sequence == sequence && hwt101_checksum_errors == 1);
  rx(0x99); rx(0x55); rx(0x01); frame(0x53, 16384, 8);
  assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == 9000);
  // Header inside a truncated candidate: the next complete frame still wins.
  rx(0x55); rx(0x53); rx(0); frame(0x53, -16384, 9);
  assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == -9000 && yaw.version == 9);
  // A valid checksum equal to55 must not act as a phantom next header.
  make_frame(bytes, 0x53, 0, 0xAD); assert(bytes[10] == 0x55);
  for (unsigned i = 0; i < 11; ++i) rx(bytes[i]);
  frame(0x53, 16384, 10); assert(HWT101_GetYaw(&yaw) && yaw.yaw_cdeg == 9000);
  tick = yaw.update_tick; now_ms = tick + 500; assert(HWT101_GetYaw(&yaw));
  now_ms = tick + 501; assert(!HWT101_GetYaw(&yaw) && !HWT101_GetYaw(NULL));
  frame(0x53, 0, 11); primask = 1; assert(HWT101_GetYaw(&yaw) && primask == 1); primask = 0;
  unsigned count = rx_count; HWT101_UART_RxCpltCallback(&other_uart);
  HWT101_UART_ErrorCallback(&other_uart); assert(rx_count == count && abort_count == 0);
  HWT101_UART_ErrorCallback(&imu_uart); assert(abort_count == 1 && !HWT101_GetYaw(&yaw));
  frame(0x53, 0, 12); assert(HWT101_GetYaw(&yaw) && tx_count == 3);
  rx_status = HAL_BUSY; frame(0x53, 0, 13); assert(!HWT101_GetYaw(&yaw));
  rx_status = HAL_OK; service(now_ms); frame(0x53, 0, 14); assert(HWT101_GetYaw(&yaw));
  HWT101_DebugYaw(now_ms); count = log_count; HWT101_DebugYaw(now_ms + 749);
  assert(log_count == count); now_ms += 750; frame(0x53, -16384, 15);
  HWT101_DebugYaw(now_ms); assert(log_count == count + 1 && strstr(last_log, "yaw=270.00"));
  boot(UINT32_MAX - 1000); // All command waits/freshness work across tick wrap.
  frame(0x53, 0, 16); now_ms += 500; assert(HWT101_GetYaw(&yaw));
  ++now_ms; assert(!HWT101_GetYaw(&yaw));
  for (unsigned failed_leg = 0; failed_leg < 3; ++failed_leg)
  {
    setup(0); if (failed_leg > 0) service(500); if (failed_leg > 1) service(700);
    tx_status = HAL_TIMEOUT; service(failed_leg == 0 ? 500 : failed_leg == 1 ? 700 : 1200);
    assert(HWT101_GetStartupState() == HWT101_INIT_FAILED && !HWT101_GetYaw(&yaw));
    count = tx_count; service(5000); assert(tx_count == count);
    frame(0x53, 0, 17); assert(!HWT101_GetYaw(&yaw));
  }
  hwt101_uart = NULL; rx_status = HAL_ERROR; assert(HWT101_Init(&imu_uart) == HAL_ERROR);
  assert(HWT101_GetStartupState() == HWT101_INIT_FAILED);
  puts("PASS: HWT101 real parser, exact boot-zero timing/TX bytes, fresh-frame fence, signed angles/version/checksum/resync,500ms/wrap,UART recovery,750ms debug,duplicate init and TX failure safety");
  return 0;
}
