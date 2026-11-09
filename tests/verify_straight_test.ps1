param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/straight_test_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$motorSource = Get-Content (Join-Path $projectRoot 'Task/motor_test_task.c') -Raw
$rtosSource = Get-Content (Join-Path $projectRoot 'Core/Src/freertos.c') -Raw
function Extract-Function([string]$source, [string]$name) {
  $match = [regex]::Match($source, '(?m)^(?:static )?[^\r\n]+\b' + $name + '\([^;]*?\)\s*\{')
  if (-not $match.Success) { throw "Missing function $name" }
  $depth = 1; $cursor = $match.Index + $match.Length
  while ($depth -gt 0) {
    if ($source[$cursor] -eq '{') { ++$depth }
    if ($source[$cursor] -eq '}') { --$depth }
    ++$cursor
  }
  $source.Substring($match.Index, $cursor - $match.Index)
}
$motorDefines = ([regex]::Matches($motorSource, '(?m)^#define MOTOR_TEST_[^\r\n]+') | ForEach-Object Value) -join "`n"
$speedHeader = Get-Content (Join-Path $projectRoot 'User/Motion/WheelSpeed/inc/M2006_Speed.h') -Raw
$feedbackDefine = [regex]::Match($speedHeader, '(?m)^#define M2006_FEEDBACK_TIMEOUT_MS[^\r\n]+').Value
if (-not $feedbackDefine) { throw 'Missing actual feedback timeout' }
$begin = $motorSource.IndexOf('static osThreadId_t motor_test_task_handle;')
$end = $motorSource.IndexOf('static bool MotorTest_FeedbackFresh')
if ($begin -lt 0 -or $end -le $begin) { throw 'Missing motor-test globals' }
$motorGlobals = $motorSource.Substring($begin, $end - $begin)
$rtosGlobals = [regex]::Match($rtosSource, '(?s)/\* USER CODE BEGIN Variables \*/(.*?)/\* USER CODE END Variables \*/').Groups[1].Value
if (-not $rtosGlobals) { throw 'Missing RTOS task declarations' }
$prefix = @'
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "robot_config.h"
#include "motor_test_task.h"
typedef void *osThreadId_t;
typedef int osPriority_t;
typedef struct { const char *name; unsigned stack_size; int priority; } osThreadAttr_t;
enum { osPriorityLow, osPriorityBelowNormal, osPriorityNormal, osPriorityHigh };
typedef struct { uint32_t msg_cnt, last_rx_tick; } moto_measure_t;
static moto_measure_t moto_chassis[4];
static uint32_t fake_now, run_started;
static unsigned run_mode, wait_samples, targets, stops, emergency_stops;
static unsigned thread_count, motor_threads, wheel_test_threads, servo_calls, mission_calls, button_calls;
static unsigned debug_calls;
static float wheel_targets[4];
static jmp_buf task_return;
static int huart2;
static osThreadId_t defaultTaskHandle;
static const osThreadAttr_t defaultTask_attributes = {.name="defaultTask"};
static void StartDefaultTask(void *p) { (void)p; }
void StartMotorTask(void *p) { (void)p; }
static void StartOledTask(void *p) { (void)p; }
static void StartImuTask(void *p) { (void)p; }
static osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr) {
  (void)fn; (void)arg; ++thread_count;
  if (!strcmp(attr->name,"motorTask")) ++motor_threads;
  if (!strcmp(attr->name,"motorTestTask")) ++wheel_test_threads;
  return (osThreadId_t)(uintptr_t)thread_count;
}
static uint32_t osKernelGetTickCount(void) { return fake_now; }
static uint32_t HAL_GetTick(void) { return fake_now; }
static int DebugUart_Log(const char *s) { (void)s; return 0; }
static int DebugUart_Logf(const char *s, ...) { (void)s; return 0; }
bool DebugUartTask_Create(void *u) { assert(u==&huart2); ++debug_calls; return true; }
bool ServoTestTask_Create(void) { ++servo_calls; return true; }
bool MissionTask_Create(void) { ++mission_calls; return true; }
bool StartButtonTask_Create(void) { ++button_calls; return true; }
static void Error_Handler(void) { assert(!"unexpected startup fault"); }
static void SpeedLoop_SetMotorTarget(uint8_t id, float speed, int16_t limit) {
  assert(id==ROBOT_LEFT_WHEEL_MOTOR_ID || id==ROBOT_RIGHT_WHEEL_MOTOR_ID);
  assert(limit==ROBOT_S1_CURRENT_LIMIT); wheel_targets[id-1U]=speed; ++targets;
}
static void brake(void) { ++stops; memset(wheel_targets,0,sizeof wheel_targets); }
static void SpeedLoop_EmergencyStop(void) { ++emergency_stops; memset(wheel_targets,0,sizeof wheel_targets); }
static void feed_feedback(void) {
  uint32_t elapsed=fake_now-run_started;
  bool left=run_mode!=1 && run_mode!=3 && !(run_mode==5 && elapsed>=1000U);
  bool right=run_mode!=1 && run_mode!=2 && !(run_mode==4 && elapsed>=1000U);
  if (left) moto_chassis[ROBOT_LEFT_WHEEL_MOTOR_ID-1U]=(moto_measure_t){1,fake_now};
  if (right) moto_chassis[ROBOT_RIGHT_WHEEL_MOTOR_ID-1U]=(moto_measure_t){1,fake_now};
}
static int osDelayUntil(uint32_t next);
'@
$cases = @'
static int osDelayUntil(uint32_t next) {
  uint32_t elapsed=fake_now-run_started; ++wait_samples;
  if (run_mode==0) {
    assert(targets==2 && !emergency_stops);
    if (elapsed<3000U) {
      assert(motor_test_status==MOTOR_TEST_RUNNING && stops==1);
      assert(wheel_targets[ROBOT_LEFT_WHEEL_MOTOR_ID-1U]==100.0f*ROBOT_LEFT_WHEEL_FORWARD_SIGN);
      assert(wheel_targets[ROBOT_RIGHT_WHEEL_MOTOR_ID-1U]==100.0f*ROBOT_RIGHT_WHEEL_FORWARD_SIGN);
    } else {
      assert(motor_test_status==MOTOR_TEST_DONE && stops==2);
      assert(!wheel_targets[ROBOT_LEFT_WHEEL_MOTOR_ID-1U] && !wheel_targets[ROBOT_RIGHT_WHEEL_MOTOR_ID-1U]);
    }
  } else if (run_mode<=3) {
    assert(!targets && stops==1);
    assert(motor_test_status==(elapsed<2000U?MOTOR_TEST_WAIT_FEEDBACK:MOTOR_TEST_FEEDBACK_FAULT));
    assert(emergency_stops==(elapsed<2000U?0U:1U));
  } else {
    assert(targets==2 && stops==1);
    if (elapsed>=1100U) assert(motor_test_status==MOTOR_TEST_FEEDBACK_FAULT && emergency_stops==1);
  }
  uint32_t end=run_mode==0?3100U:(run_mode<=3?2100U:1200U);
  if (elapsed>=end) longjmp(task_return,1);
  fake_now=next; feed_feedback(); return 0;
}
int main(void) {
  assert(ROBOT_STRAIGHT_TEST_ENABLED<=1 && ROBOT_STRAIGHT_TEST_RPM==100 && ROBOT_STRAIGHT_TEST_MS==3000U);
  assert(ROBOT_CAMERA_WIDE_ANGLE_DEG==140 && ROBOT_BAFFLE_TEST_ANGLE_DEG==155 && ROBOT_CLAW_TEST_ANGLE_DEG==63);
  assert(ROBOT_BAFFLE_TEST_ANGLE_DEG==ROBOT_BAFFLE_DOWN_DEG);
  assert(ROBOT_LEFT_WHEEL_MOTOR_ID==2 && ROBOT_RIGHT_WHEEL_MOTOR_ID==4);
  assert(M2006_FEEDBACK_TIMEOUT_MS==100U);
  MX_FREERTOS_Init();
#if ROBOT_STRAIGHT_TEST_ENABLED
  assert(motorTaskHandle && motor_threads==1 && wheel_test_threads==1 && debug_calls==1);
  assert(servo_calls==ROBOT_MG90_SPEED_TEST_ENABLED && !mission_calls && !button_calls);
  assert(MotorTestTask_Create() && wheel_test_threads==1); //No duplicated test task.
#elif !ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED
  assert(motorTaskHandle && motor_threads==1 && !wheel_test_threads && !servo_calls && debug_calls==1);
  assert(mission_calls==1 && button_calls==1 && !motor_test_task_handle);
  assert(!targets && !stops && !emergency_stops);
  puts("PASS:normal startup creates wheel-control, mission and PE12 tasks without either test task or a drive command");
#else
  assert(!motor_threads && !wheel_test_threads && servo_calls==ROBOT_MG90_SPEED_TEST_ENABLED && debug_calls==1);
  assert(!mission_calls && !button_calls);
  assert(!motor_test_task_handle && motor_test_status==MOTOR_TEST_WAIT_FEEDBACK);
  assert(!targets && !stops && !emergency_stops);
  puts("PASS:real RTOS servo-only startup creates NO wheel-control/test/mission/start-button tasks;disabled wheel test cannot move motors");
#endif
  //The following directly invokes the handler only in this hardware-free host
  //simulation; it is not scheduled by the disabled production test entry.
  for (unsigned mode=0;mode<6;++mode) for (unsigned wrap=0;wrap<2;++wrap) {
    run_mode=mode; run_started=fake_now=wrap?UINT32_MAX-500U:100U;
    targets=stops=emergency_stops=wait_samples=0;
    motor_test_status=MOTOR_TEST_WAIT_FEEDBACK;
    memset(moto_chassis,0,sizeof moto_chassis); memset(wheel_targets,0,sizeof wheel_targets);
    feed_feedback();
    if (!setjmp(task_return)) StartMotorTestTask(0);
    assert(wait_samples>100 && !wheel_targets[ROBOT_LEFT_WHEEL_MOTOR_ID-1U] && !wheel_targets[ROBOT_RIGHT_WHEEL_MOTOR_ID-1U]);
  }
  puts("PASS:real RTOS respects straight-test enable;isolated host simulation checks100rpm/3000ms stop/no restart,both motor IDs/signs,2s missing/one-sided feedback timeout,100ms feedback-loss stop and tick wrap");
}
'@
$names = @('MotorTest_FeedbackFresh','MotorTest_BothFeedbackFresh','MotorTest_Stop',
  'StartMotorTestTask','MotorTestTask_Create','MotorTest_GetStatus')
$functions = ($names | ForEach-Object { Extract-Function $motorSource $_ }) -join "`n"
$code = $prefix + "`n" + $motorDefines + "`n" + $feedbackDefine + "`n" + $motorGlobals +
  "`n" + $rtosGlobals + "`n" + $functions + "`n" +
  (Extract-Function $rtosSource 'MX_FREERTOS_Init') + "`n" + $cases
$executable = Join-Path $outputDir 'straight_test_cases.exe'
$code | & $Compiler -std=c11 -Wall -Wextra -Werror -x c - '-I' (Join-Path $projectRoot 'Task') -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Straight-test host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Straight-test checks failed' }
