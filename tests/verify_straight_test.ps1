param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/straight_test_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$motorSource = Get-Content (Join-Path $projectRoot 'Task/motor_test_task.c') -Raw
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
$prefix = @'
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "app_rtos_task_config.h"
#include "debug_uart_task.h"
#include "imu_task.h"
#include "robot_config.h"
#include "motor_test_task.h"
#include "usart.h"
typedef struct { uint32_t msg_cnt, last_rx_tick; } moto_measure_t;
static moto_measure_t moto_chassis[4];
static uint32_t fake_now, run_started;
static unsigned run_mode, wait_samples, targets, stops, emergency_stops;
static float wheel_targets[4];
static jmp_buf task_return;
static char created[16], fail_at;
static size_t created_count;
static unsigned startup_faults, heartbeat_calls, frame_logs, imu_services, imu_logs;
static osThreadFunc_t imu_entry;
UART_HandleTypeDef huart2;
void MX_FREERTOS_Init(void);
void StartDefaultTask(void *p);
void StartMotorTask(void *p) { (void)p; }
void StartOledTask(void *p) { (void)p; }
static void DummyTask(void *p) { (void)p; }
osThreadId_t osThreadNew(osThreadFunc_t fn, void *arg, const osThreadAttr_t *attr) {
  const struct { const char *name; char id; uint32_t stack; osPriority_t priority; } threads[] = {
    {"defaultTask",     'D',  512, osPriorityNormal},
    {"motorTask",       'M', 1024, osPriorityHigh},
    {"oledTask",        'O', 1024, osPriorityLow},
    {"debugUartTask",   'U', 1024, osPriorityLow},
    {"imuTask",         'I', 1024, osPriorityBelowNormal},
    {"servoTestTask",   'S', 1024, osPriorityLow},
    {"motorTestTask",   'W', 1024, osPriorityNormal},
    {"missionTask",     'Q', 2048, osPriorityAboveNormal},
    {"startButtonTask", 'B', 1024, osPriorityNormal},
  };
  assert(fn && !arg && attr && attr->name);
  for (size_t i=0; i<sizeof threads/sizeof threads[0]; ++i) {
    if (strcmp(attr->name,threads[i].name)) continue;
    char id=threads[i].id;
    assert(attr->stack_size==threads[i].stack && attr->priority==threads[i].priority);
    if (id=='D') assert(fn==StartDefaultTask);
    if (id=='M') assert(fn==StartMotorTask);
    if (id=='O') assert(fn==StartOledTask);
    if (id=='I') imu_entry=fn;
    assert(created_count+1<sizeof created);
    created[created_count++]=id;
    created[created_count]='\0';
    return id==fail_at ? NULL : (osThreadId_t)(uintptr_t)created_count;
  }
  assert(!"unexpected thread");
  return NULL;
}
static uint32_t osKernelGetTickCount(void) { return fake_now; }
uint32_t HAL_GetTick(void) { return fake_now; }
bool DebugUart_Log(const char *s) { (void)s; return true; }
bool DebugUart_Logf(const char *s, ...) {
  if (!strcmp(s,"[TEST] FRAME lift=%u cam=%u MOTOR OFF\r\n")) ++frame_logs;
  return true;
}
bool DebugUartTask_Create(UART_HandleTypeDef *u) {
  assert(u==&huart2);
  return osThreadNew(DummyTask,NULL,&AppRtos_DebugUartTaskAttr)!=NULL;
}
bool ServoTestTask_Create(void) { return osThreadNew(DummyTask,NULL,&AppRtos_ServoTestTaskAttr)!=NULL; }
bool MissionTask_Create(void) { return osThreadNew(DummyTask,NULL,&AppRtos_MissionTaskAttr)!=NULL; }
bool StartButtonTask_Create(void) { return osThreadNew(DummyTask,NULL,&AppRtos_StartButtonTaskAttr)!=NULL; }
void Error_Handler(void) { ++startup_faults; }
void LED_Task(void *p) { assert(p==&heartbeat_calls); ++heartbeat_calls; }
void IMU_Service(uint32_t now) { assert(now==fake_now); ++imu_services; ++fake_now; }
void IMU_DebugYaw(uint32_t now) { assert(imu_services==imu_logs+1 && now==fake_now); ++imu_logs; }
int osDelay(uint32_t ticks) {
  assert(ticks==20 && imu_services==1 && imu_logs==1);
  longjmp(task_return,1);
}
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
static void verify_startup(void) {
  const char *expected="D"
#if ROBOT_STRAIGHT_TEST_ENABLED || (!ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED)
    "M"
#endif
#if ROBOT_OLED_ENABLED
    "O"
#endif
    "UI"
#if ROBOT_MG90_SPEED_TEST_ENABLED
    "S"
#endif
#if ROBOT_STRAIGHT_TEST_ENABLED
    "W"
#elif !ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED
    "QB"
#endif
  ;
  for (size_t i=0; expected[i]; ++i) {
    created_count=0;
    startup_faults=0;
    fail_at=expected[i];
    size_t prefix=i+1;
    /* The existing order checks default and motor creation later. */
    if (fail_at=='D') prefix=strcspn(expected,"U");
    if (fail_at=='M') prefix=strcspn(expected,"WQ");
    MX_FREERTOS_Init();
    assert(startup_faults==1 && created_count==prefix && !strncmp(created,expected,prefix));
  }
  created_count=startup_faults=0;
  fail_at=0;
  MX_FREERTOS_Init();
  assert(!startup_faults && !strcmp(created,expected));
  assert(!targets && !stops && !emergency_stops);
#if ROBOT_STRAIGHT_TEST_ENABLED
  assert(MotorTestTask_Create() && !strcmp(created,expected)); //No duplicated test task.
#else
  assert(!motor_test_task_handle && motor_test_status==MOTOR_TEST_WAIT_FEEDBACK);
#endif
  StartDefaultTask(&heartbeat_calls);
  assert(heartbeat_calls==1 && frame_logs==ROBOT_FRAME_DOWN_TEST_ENABLED);
  assert(imu_entry);
  if (!setjmp(task_return)) imu_entry(NULL);
  assert(imu_services==1 && imu_logs==1);
  printf("PASS: startup profile %d order=%s, stacks/priorities, failure stops, heartbeat and IMU polling\n",
         RTOS_TEST_PROFILE,expected);
}

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
  verify_startup();
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
  "`n" + $functions + "`n" + $cases
$includeDirs = @('tests/host/rtos_hal', 'User/Runtime/Rtos/inc', 'User/Mission/State/inc',
  'User/Hardware/Imu/inc', 'User/Hardware/Display/inc', 'Core/Inc', 'M2006', 'OLED', 'Task', 'Debug')
$includeArgs = @($includeDirs | ForEach-Object { "-I$(Join-Path $projectRoot $_)" })
$sources = @('Core/Src/freertos.c', 'User/Runtime/Rtos/src/app_rtos.c',
  'User/Runtime/Rtos/src/app_rtos_task_config.c') | ForEach-Object { Join-Path $projectRoot $_ }
foreach ($profile in 0..4) {
  $executable = Join-Path $outputDir "straight_test_cases_$profile.exe"
  $code | & $Compiler -std=c11 -Wall -Wextra -Werror "-DRTOS_TEST_PROFILE=$profile" @includeArgs -x c - @sources -o $executable
  if ($LASTEXITCODE -ne 0) { throw "Straight-test host build failed for profile $profile" }
  & $executable
  if ($LASTEXITCODE -ne 0) { throw "Straight-test checks failed for profile $profile" }
}
