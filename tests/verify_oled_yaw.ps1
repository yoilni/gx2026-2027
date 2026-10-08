param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/oled_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$source = Get-Content (Join-Path $projectRoot 'OLED/oled_task.c') -Raw

function Extract-Function([string]$name) {
  $match = [regex]::Match($source, '(?m)^static [^\r\n]+\b' + $name + '\([^;]*?\)\r?\n\{')
  if (-not $match.Success) { throw "Missing function $name" }
  $depth = 1
  $cursor = $match.Index + $match.Length
  while ($depth -gt 0) {
    if ($source[$cursor] -eq '{') { ++$depth }
    if ($source[$cursor] -eq '}') { --$depth }
    ++$cursor
  }
  $source.Substring($match.Index, $cursor - $match.Index)
}

$preamble = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "imu_task.h"
#include "mission_task.h"
#include "robot_config.h"
#if ROBOT_OLED_ENABLED != 1U
#error OLED must be enabled for this restored display
#endif
static IMU_Yaw fake_attitude;
static MissionSnapshot fake_snapshot;
static bool fake_fresh, fake_mission_available;
static char rows[8][17];
bool IMU_GetYaw(IMU_Yaw *attitude) {
  if (!fake_fresh) return false;
  *attitude = fake_attitude;
  return true;
}
bool MissionTask_GetSnapshot(MissionSnapshot *snapshot) {
  if (!fake_mission_available) return false;
  *snapshot = fake_snapshot;
  return true;
}
static HAL_StatusTypeDef OLED_WriteString(uint8_t column, uint8_t page, const char *text) {
  assert(column == 0 && (page == 1 || page == 3));
  assert(strlen(text) == 16);
  memcpy(rows[page], text, 17);
  return HAL_OK;
}
'@
$cases = @'
static void check(int32_t raw, int32_t zero, const char *yaw, const char *field) {
  fake_attitude.yaw_cdeg = raw;
  fake_snapshot.yaw_start_cdeg = zero;
  OLED_TaskShowYaw();
  assert(strcmp(rows[1], yaw) == 0);
  assert(strcmp(rows[3], field) == 0);
}
int main(void) {
  assert(OLED_TaskWrapYaw(-1) == 35999);
  assert(OLED_TaskWrapYaw(-36000) == 0);
  assert(OLED_TaskWrapYaw(36000) == 0);
  fake_fresh = false;
  OLED_TaskShowYaw();
  assert(strcmp(rows[1], "YAW:--- NO DATA ") == 0);
  assert(strcmp(rows[3], "FIELD:---       ") == 0);
  fake_fresh = true;
  fake_mission_available = false; /* Actuator-only tests / scheduler not started. */
  check(12345, 0, "YAW:123.45 DEG  ", "FIELD:WAIT START");
  fake_mission_available = true;
  fake_snapshot.start_zone_locked = false; /* Waiting for PE12. */
  check(-1, 0, "YAW:359.99 DEG  ", "FIELD:WAIT START");
  fake_snapshot.start_zone_locked = true;
  check(0, 0, "YAW:  0.00 DEG  ", "FIELD:  0.00 DEG");
  check(-4500, 0, "YAW:315.00 DEG  ", "FIELD:315.00 DEG");
  check(-18000, 31500, "YAW:180.00 DEG  ", "FIELD:225.00 DEG");
  check(100, 35900, "YAW:  1.00 DEG  ", "FIELD:  2.00 DEG");
  check(-1, 100, "YAW:359.99 DEG  ", "FIELD:358.99 DEG");
  fake_fresh = false; /* Expired data must replace the previous angle. */
  OLED_TaskShowYaw();
  assert(strcmp(rows[1], "YAW:--- NO DATA ") == 0);
  assert(strcmp(rows[3], "FIELD:---       ") == 0);
  puts("PASS: OLED yaw, zero reference, wraparound, test-mode and stale-data display");
  return 0;
}
'@
$code = $preamble + "`n" + (Extract-Function 'OLED_TaskWrapYaw') + "`n" +
    (Extract-Function 'OLED_TaskShowYaw') + "`n" + $cases
$executable = Join-Path $outputDir 'oled_yaw_cases.exe'
# Compile the actual display functions from stdin; do not rewrite firmware files.
$code | & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-x' 'c' '-' `
    '-I' (Join-Path $projectRoot 'tests/host') `
    '-I' (Join-Path $projectRoot 'HWT101') `
    '-I' (Join-Path $projectRoot 'Task') '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'OLED yaw host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'OLED yaw checks failed' }
