param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/servo_outputs_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$source = Get-Content (Join-Path $projectRoot 'User/Motion/Servo/src/actuator_task.c') -Raw
$servoSource = Get-Content (Join-Path $projectRoot 'User/Runtime/Bench/src/servo_test_task.c') -Raw
$pwmRoot = Join-Path $projectRoot 'User/Hardware/Servo'
$servoMotionRoot = Join-Path $projectRoot 'User/Motion/Servo'
function Extract-Function([string]$name, [string]$body = $source) {
  $match = [regex]::Match($body, '(?m)^(?:static )?[^\r\n]+\b' + $name + '\([^;]*?\)\r?\n\{')
  if (-not $match.Success) { throw "Missing function $name" }
  $depth = 1; $cursor = $match.Index + $match.Length
  while ($depth -gt 0) {
    if ($body[$cursor] -eq '{') { ++$depth }
    if ($body[$cursor] -eq '}') { --$depth }
    ++$cursor
  }
  $body.Substring($match.Index, $cursor - $match.Index)
}
$prefix = @'
#include <assert.h>
#include <stdbool.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "robot_config.h"
#include "servo_pwm.h"
#include "servo_motion.h"
#include "tim.h"
TIM_TypeDef test_tim2;
TIM_HandleTypeDef htim2;
#define SERVO_TEST_TASK_PERIOD_TICKS 10U
static uint8_t baffle_pwm_suspended;
static uint8_t claw_close_guard_active, claw_close_pending;
static uint32_t claw_close_guard_tick;
static uint16_t compare[4];
static unsigned started[4], writes[4], delay_calls, baffle_stops;
static unsigned fail_channel;
static jmp_buf bench_exit;
static uint32_t bench_now;
static volatile uint16_t servo_test_pulse_us=ROBOT_MG90_TEST_START_PULSE_US;
void ServoPwmTest_SetCompare(TIM_HandleTypeDef *timer, unsigned channel, uint16_t pulse) {
  assert(timer==&htim2 && channel<4);
  compare[channel]=pulse; ++writes[channel];
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
  assert(timer==&htim2 && channel<4); ++started[channel];
  assert(writes[channel]);
  if(channel==fail_channel) return HAL_ERROR;
  assert(!(timer->Instance->CCER & (1U << (4U * channel))));
  timer->Instance->CCER |= 1U << (4U * channel);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel) {
  assert(timer==&htim2 && channel==TIM_CHANNEL_3);
  ++baffle_stops; timer->Instance->CCER &= ~TIM_CCER_CC3E; return HAL_OK;
}
static void BootInit_DelayMs(uint32_t delay) {
  assert(delay==ROBOT_CAMERA_POWERUP_SETTLE_MS); ++delay_calls;
  assert(ServoPwm_IsEnabled(SERVO_PWM_CAMERA));
  assert(!ServoPwm_IsEnabled(SERVO_PWM_BAFFLE) && !ServoPwm_IsEnabled(SERVO_PWM_CLAW));
}
static void reset(void) {
  memset(compare,0,sizeof compare); memset(started,0,sizeof started);
  memset(writes,0,sizeof writes); delay_calls=baffle_stops=0; fail_channel=4;
  baffle_pwm_suspended=0;
  claw_close_guard_active=claw_close_pending=0;
  test_tim2.CCER=0;
  htim2=(TIM_HandleTypeDef){TIM2,{83,19999}};
}
static void DebugUart_Logf(const char *format,...) { (void)format; }
static void osThreadExit(void) { assert(!"unexpected invalid bench timer"); }
static uint32_t osKernelGetTickCount(void) { return bench_now; }
#if ROBOT_MG90_TEST_WIDE_HOLD_MS > 0U
static int osDelay(uint32_t ms) {
  assert(ms==ROBOT_MG90_TEST_WIDE_HOLD_MS);
  assert(compare[TIM_CHANNEL_1]==ROBOT_MG90_TEST_START_PULSE_US);
  assert(servo_test_pulse_us==ROBOT_MG90_TEST_START_PULSE_US);
  bench_now+=ms;
  return 0;
}
#endif
static int osDelayUntil(uint32_t tick) {
  assert(tick==bench_now+SERVO_TEST_TASK_PERIOD_TICKS);
  longjmp(bench_exit,1);
}
'@
$cases = @'
static void CheckRawPwm(void) {
  const ServoPwm_Channel invalid = (ServoPwm_Channel)-1;
  reset();
  ServoPwm_WritePulseUs(SERVO_PWM_BAFFLE, 1234U);
  assert(compare[TIM_CHANNEL_3]==1234U && !ServoPwm_IsEnabled(SERVO_PWM_BAFFLE));
  assert(writes[TIM_CHANNEL_3]==1 && !started[TIM_CHANNEL_3]);
  assert(!writes[TIM_CHANNEL_1] && !writes[TIM_CHANNEL_4]);
  assert(ServoPwm_Start(SERVO_PWM_BAFFLE)==HAL_OK && ServoPwm_IsEnabled(SERVO_PWM_BAFFLE));
  assert(ServoPwm_Stop(SERVO_PWM_BAFFLE)==HAL_OK && !ServoPwm_IsEnabled(SERVO_PWM_BAFFLE));

  reset();
  ServoPwm_WritePulseUs(invalid, 1234U);
  assert(!ServoPwm_IsEnabled(invalid));
  assert(ServoPwm_Start(invalid)==HAL_ERROR && ServoPwm_Stop(invalid)==HAL_ERROR);
  htim2.Instance=0;
  ServoPwm_WritePulseUs(SERVO_PWM_CAMERA, 1234U);
  assert(!ServoPwm_IsEnabled(SERVO_PWM_CAMERA));
  assert(ServoPwm_Start(SERVO_PWM_CAMERA)==HAL_ERROR && ServoPwm_Stop(SERVO_PWM_BAFFLE)==HAL_ERROR);
  for(unsigned channel=0;channel<4;++channel) assert(!writes[channel] && !started[channel]);
  assert(!baffle_stops);
  puts("PASS:raw PWM write never enables output; invalid channels and timer are rejected");
}
int main(void) {
  CheckRawPwm();
  if(ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_BAFFLE_TEST_ENABLED && !ROBOT_CLAW_TEST_ENABLED) {
    assert(!ROBOT_FRAME_DOWN_TEST_ENABLED && !ROBOT_STRAIGHT_TEST_ENABLED);
    assert(ROBOT_CAMERA_WIDE_ANGLE_DEG==140U && ROBOT_CAMERA_NEAR_ANGLE_DEG==115U);
    reset();assert(Actuator_Init()==HAL_OK);
    assert(compare[TIM_CHANNEL_1]==ROBOT_MG90_TEST_START_PULSE_US && started[TIM_CHANNEL_1]==1);
    assert(test_tim2.CCER==1U && delay_calls==1);
    if(setjmp(bench_exit)==0) StartServoTestTask(NULL);
    assert(bench_now==ROBOT_MG90_TEST_WIDE_HOLD_MS);
    assert(compare[TIM_CHANNEL_1]==ROBOT_MG90_TEST_PULSE_US && servo_test_pulse_us==ROBOT_MG90_TEST_PULSE_US);
    assert(started[TIM_CHANNEL_1]==1 && test_tim2.CCER==1U);
    for(unsigned channel=1;channel<4;++channel) assert(!started[channel] && !writes[channel]);
    puts("PASS:camera-only bench uses the configured start and hold angles; PA2 and PA3 remain off");
    return 0;
  }
  assert(ROBOT_BAFFLE_TEST_ENABLED==ROBOT_MG90_SPEED_TEST_ENABLED);
  assert(ROBOT_CLAW_TEST_ENABLED==ROBOT_MG90_SPEED_TEST_ENABLED);
  const uint16_t camera_pulse = ROBOT_MG90_SPEED_TEST_ENABLED ? ROBOT_MG90_TEST_START_PULSE_US : 2055U;
  const uint16_t closed_claw_pulse = 757U, material_open_pulse = 942U;
  const uint16_t startup_claw_pulse = ROBOT_CLAW_TEST_ENABLED ? 950U : closed_claw_pulse;
  assert(ROBOT_BAFFLE_TEST_ANGLE_DEG==155);
  assert(ROBOT_BAFFLE_TEST_ANGLE_DEG==ROBOT_BAFFLE_DOWN_DEG);
  assert(ROBOT_BAFFLE_DOWN_DEG==155 && ROBOT_BAFFLE_UP_DEG==80);
  assert(ROBOT_CLAW_TEST_ANGLE_DEG==63);
  assert(ROBOT_CLAW_CLOSED_DEG==36);
  assert(ROBOT_CLAW_MATERIAL_OPEN_DEG==62);
  assert(ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG==64U);
  assert(ROBOT_CLAW_CASUALTY_OPEN_DEG==63);
  assert(ROBOT_CLAW_CASUALTY_CAPTURE_OPEN_DEG==68 && ROBOT_CLAW_ARRANGE02_OPEN_DEG==74);
  assert(!ROBOT_FRAME_DOWN_TEST_ENABLED && !ROBOT_STRAIGHT_TEST_ENABLED);
  assert(ROBOT_STRAIGHT_TEST_RPM==100 && ROBOT_STRAIGHT_TEST_MS==3000);
  reset(); assert(Actuator_Init()==HAL_OK);
  assert(compare[TIM_CHANNEL_1]==camera_pulse && compare[TIM_CHANNEL_3]==1607);
  assert(started[TIM_CHANNEL_1]==1 && started[TIM_CHANNEL_3]==1 && delay_calls==1);
  assert(writes[TIM_CHANNEL_1]==1 && writes[TIM_CHANNEL_3]==1);
  assert(compare[TIM_CHANNEL_4]==startup_claw_pulse && writes[TIM_CHANNEL_4]==1 && started[TIM_CHANNEL_4]==1);
  assert(!writes[1] && !started[1]); //PA1 unused; the independent channels must not alias.
  assert(Actuator_SetClawClosed()==HAL_OK);
  assert(compare[TIM_CHANNEL_4]==closed_claw_pulse && writes[TIM_CHANNEL_4]==2 && started[TIM_CHANNEL_4]==1);
  assert(compare[TIM_CHANNEL_1]==camera_pulse && compare[TIM_CHANNEL_3]==1607);
  assert(writes[TIM_CHANNEL_1]==1 && writes[TIM_CHANNEL_3]==1 && delay_calls==1);
  assert(test_tim2.CCER&TIM_CCER_CC4E); //Change only CH4 compare, never restart PWM.
  assert(Actuator_SetClawAngle(ROBOT_CLAW_MATERIAL_OPEN_DEG)==HAL_OK && compare[TIM_CHANNEL_4]==material_open_pulse);
  assert(Actuator_SetClawAngle(ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG)==HAL_OK && compare[TIM_CHANNEL_4]==957); //PA3 64.
  assert(Actuator_SetClawAngle(ROBOT_CLAW_CASUALTY_OPEN_DEG)==HAL_OK && compare[TIM_CHANNEL_4]==950);
  assert(writes[TIM_CHANNEL_4]==5 && started[TIM_CHANNEL_4]==1);
  assert(writes[TIM_CHANNEL_1]==1 && writes[TIM_CHANNEL_3]==1 && delay_calls==1);
  assert(Actuator_StopBafflePwm()==HAL_OK && Actuator_StopBafflePwm()==HAL_OK);
  assert(baffle_stops==1 && !(test_tim2.CCER&TIM_CCER_CC3E));
  assert(test_tim2.CCER==(1U|TIM_CCER_CC4E)); //PA5 and PA3 still run.
  assert(Actuator_SetBaffleUpClawOpen(ROBOT_CLAW_CASUALTY_OPEN_DEG)==HAL_OK);
  assert(!(test_tim2.CCER&TIM_CCER_CC3E) && writes[TIM_CHANNEL_3]==1 && started[TIM_CHANNEL_3]==1);
  uint16_t old_claw=compare[TIM_CHANNEL_4];
  assert(Actuator_SetBaffleAngle(ROBOT_BAFFLE_DOWN_DEG)==HAL_OK);
  assert((test_tim2.CCER&TIM_CCER_CC3E) && compare[TIM_CHANNEL_3]==1607 && started[TIM_CHANNEL_3]==2);
  assert(compare[TIM_CHANNEL_4]==old_claw && compare[TIM_CHANNEL_1]==camera_pulse);
  assert(Actuator_SetBaffleUpClawOpen(ROBOT_CLAW_MATERIAL_OPEN_DEG)==HAL_OK);
  assert(compare[TIM_CHANNEL_3]==1071 && compare[TIM_CHANNEL_4]==material_open_pulse);
  assert(Actuator_SetFrameLowered()==HAL_OK);
  assert(compare[TIM_CHANNEL_3]==1607 && compare[TIM_CHANNEL_4]==closed_claw_pulse);
  assert(Actuator_SetCameraNearView()==HAL_OK && compare[TIM_CHANNEL_1]==1777); //PA5 115.
  assert(ROBOT_CAMERA_FINAL_TRACK_ANGLE_DEG==140U);
  assert(Actuator_SetCameraFinalTrackView()==HAL_OK && compare[TIM_CHANNEL_1]==2055); //PA5 140.
  assert(ROBOT_CAMERA_OBJECT_TRACK_ANGLE_DEG==140U);
  assert(Actuator_SetCameraObjectTrackView()==HAL_OK && compare[TIM_CHANNEL_1]==2055); //PA5 140.
  assert(ROBOT_CAMERA_SAFE_ZONE_ANGLE_DEG==145U);
  assert(Actuator_SetCameraSafeZoneView()==HAL_OK && compare[TIM_CHANNEL_1]==2111); //PA5 145.
  assert(Actuator_SetCameraWideView()==HAL_OK && compare[TIM_CHANNEL_1]==2055); //PA5 140.
  assert(compare[TIM_CHANNEL_3]==1607 && compare[TIM_CHANNEL_4]==closed_claw_pulse);
  assert(started[TIM_CHANNEL_1]==1 && started[TIM_CHANNEL_3]==2 && started[TIM_CHANNEL_4]==1);
  reset(); assert(Actuator_SetClawClosed()==HAL_ERROR && !writes[TIM_CHANNEL_4]);
  for(unsigned i=0;i<3;++i) {
    reset();
    if(i==0) htim2.Instance=0;
    if(i==1) htim2.Init.Prescaler=167;
    if(i==2) htim2.Init.Period=9999;
    assert(Actuator_Init()==HAL_ERROR);
    assert(Actuator_SetClawClosed()==HAL_ERROR);
    for(unsigned channel=0;channel<4;++channel) assert(!writes[channel] && !started[channel]);
    assert(!delay_calls);
  }
  reset(); fail_channel=TIM_CHANNEL_1; assert(Actuator_Init()==HAL_ERROR);
  assert(!started[TIM_CHANNEL_3] && !started[TIM_CHANNEL_4] && !delay_calls);
  reset(); fail_channel=TIM_CHANNEL_3; assert(Actuator_Init()==HAL_ERROR);
  assert(!started[TIM_CHANNEL_4] && writes[TIM_CHANNEL_4]==!ROBOT_MG90_SPEED_TEST_ENABLED);
  reset(); fail_channel=TIM_CHANNEL_4; assert(Actuator_Init()==HAL_ERROR);
  assert(started[TIM_CHANNEL_1]==1 && started[TIM_CHANNEL_3]==1 && started[TIM_CHANNEL_4]==1);
  assert(compare[TIM_CHANNEL_1]==camera_pulse && compare[TIM_CHANNEL_3]==1607 && compare[TIM_CHANNEL_4]==startup_claw_pulse);
  assert(Actuator_SetClawClosed()==HAL_ERROR && writes[TIM_CHANNEL_4]==1);
  assert(ROBOT_S6_EXIT_BAFFLE_SETTLE_MS==500U);
  for(unsigned wrap=0;wrap<2;++wrap) {
    uint32_t start=wrap?UINT32_MAX-100U:100U;
    reset();assert(Actuator_Init()==HAL_OK);
    assert(Actuator_SetBaffleUpClawOpen(ROBOT_CLAW_MATERIAL_OPEN_DEG)==HAL_OK);
    assert(Actuator_SetBaffleAngle(ROBOT_BAFFLE_DOWN_DEG)==HAL_OK);
    Actuator_DeferClawClose(start);
    unsigned claw_writes=writes[TIM_CHANNEL_4];
    assert(Actuator_SetFrameLowered()==HAL_OK && Actuator_SetClawClosed()==HAL_OK);
    assert(Actuator_ServiceDeferredClawClose(start+499U)==HAL_OK && Actuator_ClawCloseDeferred());
    assert(compare[TIM_CHANNEL_3]==1607 && compare[TIM_CHANNEL_4]==material_open_pulse);
    assert(writes[TIM_CHANNEL_4]==claw_writes);
    assert(Actuator_ServiceDeferredClawClose(start+500U)==HAL_OK && !Actuator_ClawCloseDeferred());
    assert(compare[TIM_CHANNEL_4]==closed_claw_pulse && writes[TIM_CHANNEL_4]==claw_writes+1);
    assert(Actuator_ServiceDeferredClawClose(start+520U)==HAL_OK && writes[TIM_CHANNEL_4]==claw_writes+1);
  }
  Actuator_DeferClawClose(100);
  assert(Actuator_SetClawAngle(ROBOT_CLAW_CASUALTY_OPEN_DEG)==HAL_OK);
  assert(Actuator_ClawCloseDeferred());
  assert(Actuator_ServiceDeferredClawClose(600)==HAL_OK && compare[TIM_CHANNEL_4]==950);
  Actuator_DeferClawClose(700);
  Actuator_CancelDeferredClawClose();
  assert(Actuator_ServiceDeferredClawClose(1200)==HAL_OK && compare[TIM_CHANNEL_4]==950);
  assert(Actuator_SetClawAngle(ROBOT_CLAW_CASUALTY_CAPTURE_OPEN_DEG)==HAL_OK && compare[TIM_CHANNEL_4]==985);
  puts("PASS:claw deferred500ms from PA2 down,paired close cannot bypass,new open cancels auto-close,tick wrap,one-shot,cancel");
  puts("PASS:PA5 wide140/all object tracking140/near115;PA2 down155/up80,PA3 closed36;PA2 suspension survives paired poses until explicit resume;independent PWM and startup faults checked");
}
'@
$code = $prefix + "`n" +
  (Extract-Function 'Actuator_DeferClawCommand') + "`n" +
  (Extract-Function 'Actuator_SetFrameAngles') + "`n" +
  (Extract-Function 'Actuator_Init') + "`n" + (Extract-Function 'Actuator_SetClawAngle') + "`n" +
  (Extract-Function 'Actuator_SetClawClosed') + "`n" + (Extract-Function 'Actuator_SetBaffleAngle') + "`n" +
  (Extract-Function 'Actuator_DeferClawClose') + "`n" + (Extract-Function 'Actuator_ClawCloseDeferred') + "`n" +
  (Extract-Function 'Actuator_CancelDeferredClawClose') + "`n" + (Extract-Function 'Actuator_ServiceDeferredClawClose') + "`n" +
  (Extract-Function 'Actuator_StopBafflePwm') + "`n" +
  (Extract-Function 'Actuator_SetBaffleUpClawOpen') + "`n" + (Extract-Function 'Actuator_SetFrameLowered') + "`n" +
  (Extract-Function 'Actuator_SetCameraAngle') + "`n" + (Extract-Function 'Actuator_SetCameraNearView') + "`n" +
  (Extract-Function 'Actuator_SetCameraWideView') + "`n" +
  (Extract-Function 'Actuator_SetCameraObjectTrackView') + "`n" +
  (Extract-Function 'Actuator_SetCameraSafeZoneView') + "`n" +
  (Extract-Function 'Actuator_SetCameraFinalTrackView') + "`n" +
  (Extract-Function 'ServoTest_SetMg90Pulse' $servoSource) + "`n" +
  (Extract-Function 'StartServoTestTask' $servoSource) + "`n" + $cases
$executable = Join-Path $outputDir 'servo_outputs_cases.exe'
$code | & $Compiler -std=c11 -Wall -Wextra -Werror -x c - (Join-Path $pwmRoot 'src/servo_pwm.c') `
  (Join-Path $servoMotionRoot 'src/servo_motion.c') '-I' (Join-Path $servoMotionRoot 'inc') `
  '-I' (Join-Path $pwmRoot 'inc') '-I' (Join-Path $PSScriptRoot 'host/servo_pwm_hal') `
  '-I' (Join-Path $PSScriptRoot 'host') '-I' (Join-Path $projectRoot 'User/Config/inc') -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Servo output test build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Servo output checks failed' }
