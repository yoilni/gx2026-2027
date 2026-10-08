param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/reset_notify_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$source = Get-Content (Join-Path $projectRoot 'Core/Src/main.c') -Raw
$notify = [regex]::Match($source, '(?s)  BootInit_SetStage\("MAIXCAM_RESET_TX"\);\r?\n  \{.*?\r?\n  \}')
if (-not $notify.Success) { throw 'Missing actual startup reset notification' }
if ([regex]::Matches($source, 'MaixCam_SendCommand\(MAIXCAM_COMMAND_MCU_RESET\)').Count -ne 1) {
  throw 'Reset notification must have one startup call site'
}
$initIndex = $source.IndexOf('if (MaixCam_Init(&huart3) != HAL_OK)')
$kernelIndex = $source.IndexOf('osKernelInitialize()')
if ($initIndex -lt 0 -or $notify.Index -le $initIndex -or $notify.Index -ge $kernelIndex) {
  throw 'Reset notification must follow vision RX init and precede RTOS startup'
}
$prefix = @'
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "maixcam_task.c"
static UART_HandleTypeDef vision_uart={(void *)3,{115200}};
static unsigned tx_count,rx_count,stage_count,log_count;
static uint8_t packet[5],*rx_destination;
static HAL_StatusTypeDef tx_status;
static char boot_log[64];
uint32_t HAL_GetTick(void) { return 100; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __enable_irq(void) {}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n) {
  assert(u==&vision_uart && n==1); ++rx_count; rx_destination=b; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *b,uint16_t n,uint32_t t) {
  assert(u==&vision_uart && n==5 && t==20);
  ++tx_count; memcpy(packet,b,5); return tx_status;
}
static void BootInit_SetStage(const char *stage) {
  assert(!strcmp(stage,"MAIXCAM_RESET_TX")); ++stage_count;
}
static void BootInit_Logf(const char *fmt,...) {
  va_list args; va_start(args,fmt); vsnprintf(boot_log,sizeof boot_log,fmt,args); va_end(args); ++log_count;
}
static void notify_reset(void) {
'@
$cases = @'
}
static void rx_bytes(const uint8_t *p,unsigned n) {
  for(unsigned i=0;i<n;++i) {
    *rx_destination=p[i]; MaixCam_UART_RxCpltCallback(&vision_uart);
  }
}
int main(void) {
  const uint8_t expected[]={0xE1,0xE2,0xFE,0x1E,0x2E};
  assert(MAIXCAM_COMMAND_MCU_RESET==0xFE);
  assert(MaixCam_SendCommand(MAIXCAM_COMMAND_MCU_RESET)==HAL_ERROR && tx_count==0);
  assert(MaixCam_Init(&vision_uart)==HAL_OK && rx_count==1 && tx_count==0);
  tx_status=HAL_OK; notify_reset();
  assert(tx_count==1 && stage_count==1 && log_count==1 && !memcmp(packet,expected,5));
  assert(!strcmp(boot_log,"[BOOT] TXFE hal=0\r\n"));
  //RX remains live after the reset notice; normal event and coordinate parsing unchanged.
  const uint8_t event[]={0xF1,0xF2,0x04,0x1F,0x2F};
  rx_bytes(event,sizeof event); uint8_t cmd;
  assert(MaixCam_TakeEvent(&cmd) && cmd==0x04 && tx_count==1);
  const uint8_t xy[]={0xF1,0xF2,5,0,20,1,0,30,2,0x1F,0x2F};
  rx_bytes(xy,sizeof xy); MaixCam_Object object;
  assert(MaixCam_GetObject(&object) && object.object_id==5 && object.x_error_px==20 && object.y_error_px==-30);
  assert(tx_count==1 && rx_count==1+sizeof event+sizeof xy);
  //A new boot makes one new attempt, including timeout/error; there is no retry/ACK loop.
  for(unsigned status=HAL_OK;status<=HAL_TIMEOUT;++status) {
    unsigned previous=tx_count;
    assert(MaixCam_Init(&vision_uart)==HAL_OK && tx_count==previous);
    tx_status=(HAL_StatusTypeDef)status; notify_reset();
    assert(tx_count==previous+1 && stage_count==tx_count && log_count==tx_count);
    assert(!memcmp(packet,expected,5));
    char expected_log[64]; snprintf(expected_log,sizeof expected_log,"[BOOT] TXFE hal=%u\r\n",status);
    assert(!strcmp(boot_log,expected_log));
  }
  puts("PASS:startup FE after RX/before RTOS, exact USART3 frame, once per boot, bounded failure/no retry/ACK, RX event/coordinate parsing preserved");
}
'@
$testSource = Join-Path $outputDir 'reset_notify_cases.c'
[IO.File]::WriteAllText($testSource,$prefix+"`n"+$notify.Value+"`n"+$cases,[Text.UTF8Encoding]::new($false))
$executable = Join-Path $outputDir 'reset_notify_cases.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-I' (Join-Path $projectRoot 'tests/host/hwt101_hal') `
    '-I' (Join-Path $projectRoot 'maixcam') $testSource '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'Reset notification host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Reset notification checks failed' }
