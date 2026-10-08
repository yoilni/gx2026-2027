param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/material_e4_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$source = Get-Content (Join-Path $projectRoot 'Task/mission_task.c') -Raw
function Extract-Function([string]$name) {
  $match = [regex]::Match($source, '(?m)^static [^\r\n]+\b' + $name + '\([^;]*?\)\r?\n\{')
  if (-not $match.Success) { throw "Missing function $name" }
  $depth = 1; $cursor = $match.Index + $match.Length
  while ($depth -gt 0) {
    if ($source[$cursor] -eq '{') { ++$depth }
    if ($source[$cursor] -eq '}') { --$depth }
    ++$cursor
  }
  $source.Substring($match.Index, $cursor - $match.Index)
}
$names = @('MissionTask_WrapYaw', 'MissionTask_YawError', 'MissionTask_Abs32',
  'MissionTask_ClampFloat', 'MissionTask_ActiveRecoveryLossEvent',
  'MissionTask_DropDuplicateRecoveryEvent', 'MissionTask_CanPauseForLoadCheck',
  'MissionTask_IsObjectTrackingPhase', 'MissionTask_RedCoordinateIsFresh',
  'MissionTask_UpdateVisionInput', 'MissionTask_CalculateS6YawTurn',
  'MissionTask_ReturnToS4TrackCenter', 'MissionTask_AckS4Center',
  'MissionTask_ResumeS4AfterE4', 'MissionTask_StartS4E4Spin',
  'MissionTask_RunS4E4Spin', 'MissionTask_StartS4E4Recovery',
  'MissionTask_RunS4E4Recovery', 'MissionTask_StartS3Align',
  'MissionTask_BeginS3Vision', 'MissionTask_StartS7Decision',
  'MissionTask_RunS6FinalReverse', 'MissionTask_RunS6ExitTurn180', 'MissionTask_CheckRunningHealth')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$prototypes = ($functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).TrimEnd() + ';' }) -join "`n"
$start = $source.IndexOf('static osThreadId_t mission_task_handle;')
$globals = $source.Substring($start, $source.IndexOf('static const osThreadAttr_t mission_task_attributes') - $start)
$arrange = Extract-Function 'MissionTask_RunS4Arrange'
$waitBegin = $arrange.IndexOf('    case MISSION_STATE_S4_WAIT_ARRANGE_READY:')
$waitEnd = $arrange.IndexOf('    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:', $waitBegin)
if ($waitBegin -lt 0 -or $waitEnd -lt 0) { throw 'Missing real E4 handshake wait case' }
$arrangeWait = "static void run_arrange_wait(uint32_t now) { uint32_t elapsed=now-mission_snapshot.state_entry_tick; uint8_t event_code; switch(mission_snapshot.state) {`n" +
  $arrange.Substring($waitBegin,$waitEnd-$waitBegin) + "default: break; } }`n"
$preamble = @'
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include "mission_task.h"
#include "robot_config.h"
#include "maixcam_task.h"
#define assert(c) do { if (!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while (0)
#define MISSION_VISION_TARGET_TIMEOUT_MS MAIXCAM_DATA_TIMEOUT_MS
typedef void *osThreadId_t;
typedef void *osEventFlagsId_t;
typedef void *osMutexId_t;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
uint32_t fake_now;
unsigned fake_rx_arms;
'@
$stubs = @'
static UART_HandleTypeDef uart;
static unsigned tx_count, rx_bytes, center_entries, acknowledgements;
static uint8_t tx_commands[128];
static uint32_t health_faults;
static float selected_turn_limit;
static bool turn_aligned;
static bool tx_ok;
static uint8_t inject_stage_on_tx;
static bool frame_down, frame_lower_ok;
static unsigned frame_lower_calls;
static void byte(uint8_t b) {
  ++rx_bytes;maixcam_rx_byte=b;MaixCam_UART_RxCpltCallback(maixcam_uart);
}
static void rx(uint8_t cmd) {
  const uint8_t f[]={0xF1,0xF2,cmd,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static void coords(uint8_t id) {
  const uint8_t f[]={0xF1,0xF2,id,0,20,1,0,30,2,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static bool DebugUart_Log(const char *s) { (void)s;return true; }
static bool DebugUart_Logf(const char *s,...) { (void)s;return true; }
static void MissionTask_StopWheels(void) {
  mission_snapshot.left_target_rpm=mission_snapshot.right_target_rpm=0;
}
static void MissionTask_SetWheelTargets(int16_t l,int16_t r) {
  mission_snapshot.left_target_rpm=l;mission_snapshot.right_target_rpm=r;
}
static void MissionTask_ResetVisionControllerState(void) {}
static void MissionTask_ResetVisionPid(void) {}
static void MissionTask_ResetS6ControllerState(void) {}
static void MissionTask_ResetSupplement(void) { supplement_capture_active=supplement_budget_started=false; }
static uint32_t MissionTask_GetRequiredFaults(void) { return health_faults; }
static void MissionTask_EnterState(MissionState s,uint32_t t) {
  if(mission_snapshot.state==s) return;
  mission_snapshot.state=s;mission_snapshot.state_entry_tick=t;
  MaixCam_SetRecoveryLossFilter(MissionTask_ActiveRecoveryLossEvent());
  MaixCam_SetLoadCheckInterruptEnabled(MissionTask_CanPauseForLoadCheck());
  if(s==MISSION_STATE_FAULT || s==MISSION_STATE_STOPPED) MissionTask_StopWheels();
}
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t cmd) {
  //Ordinary03 lowers before TX;07's preparatory03 during exit180 is exempt.
  if(cmd==0x03 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN) assert(frame_down);
  assert(tx_count<sizeof(tx_commands));tx_commands[tx_count++]=cmd;
  if(cmd==0x14) ++acknowledgements;
  if(inject_stage_on_tx==cmd) { inject_stage_on_tx=0;rx(0x14);rx(0x24); }
  return tx_ok?HAL_OK:HAL_ERROR;
}
static bool MissionTask_HandleSupplementFinish(uint32_t t) { (void)t;return false; }
static bool MissionTask_LoadCheckPausesMotion(void) { return false; }
static void MissionTask_SkipToS4FinalTrack(uint32_t t) { MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME,t); }
static void MissionTask_StartS4TrackState(MissionState s,uint32_t t) { MissionTask_EnterState(s,t); }
static void verify_e4_event_buffering(void);
static void verify_03_frame_down(void);
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) {
  ++frame_lower_calls;
  if(!frame_lower_ok) return HAL_ERROR;
  frame_down=true;return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetCameraWideView(void) { return HAL_OK; }
static uint32_t MissionTask_S4ScalePushDuration(uint32_t t,bool right) { (void)right;return t; }
static uint32_t MissionTask_S4ReverseDuration(bool right) { (void)right;return 500; }
static void MissionTask_CommandS4CornerPush(uint32_t t,bool right) { (void)t;MissionTask_SetWheelTargets(right?90:70,right?70:90); }
static void MissionTask_CommandS4CornerReverse(uint32_t t,bool right) { (void)t;MissionTask_SetWheelTargets(right?-90:-70,right?-70:-90); }
static bool MissionTask_RunS6TurnToHeading(uint32_t t,int32_t yaw) {
  MissionTask_CalculateS6YawTurn(t,yaw);return turn_aligned;
}
static void MissionTask_CommandS6Straight(uint32_t t,int32_t yaw,int16_t rpm,int16_t max) {
  (void)t;(void)yaw;(void)max;MissionTask_SetWheelTargets(rpm,rpm);
}
static int16_t MissionTask_CalculateS6YawPid(uint32_t t,int32_t yaw,float min,float max) {
  (void)t;(void)yaw;(void)min;selected_turn_limit=max;return (int16_t)max;
}
'@
$cases = @'
static void setup(uint8_t target) {
  fake_now=100;assert(MaixCam_Init(&uart)==HAL_OK);
  memset(&mission_snapshot,0,sizeof(mission_snapshot));
  mission_snapshot.state=MISSION_STATE_S4_TRACK_CENTER;
  mission_snapshot.yaw_start_cdeg=35000;mission_snapshot.yaw_cdeg=100;
  mission_snapshot.safe_zone_yaw_target_cdeg=17000;
  s7_target_switch_enabled=true;s3_target_select_command=target;
  s4_material_e4_recovery_count=0;s4_green_e4_push_cycles=0;
  s4_green_e4_spin_exhausted=false;s4_e4_recovery_used=false;
  s7_red_switch_sync_pending=s7_red_coordinate_fence_active=false;
  supplement_capture_active=supplement_budget_started=s5_empty_recovery_active=false;
  health_faults=tx_count=center_entries=acknowledgements=0;turn_aligned=false;
  tx_ok=true;inject_stage_on_tx=0;s4_e4_protocol_active=false;
  frame_down=false;frame_lower_ok=true;frame_lower_calls=0;
}
static void tick(uint32_t t) {
  fake_now=t;MissionTask_UpdateVisionInput(t);MissionTask_CheckRunningHealth(t);
  MissionTask_DropDuplicateRecoveryEvent(t);run_arrange_wait(t);MissionTask_RunS4E4Recovery(t);
}
static void second_loss(void) {
  setup(0x51);s4_e4_recovery_used=true; // Prior red recovery must not consume black/green's first attempt.
  MissionTask_StartS4E4Recovery(120);
  assert(s4_material_e4_recovery_count==1 && mission_snapshot.state==MISSION_STATE_S4_E4_TURN_AWAY);
  assert(s4_e4_away_yaw_cdeg==35000 && s4_e4_left_yaw_cdeg==3500 && s4_e4_right_yaw_cdeg==30500);
  tick(140);assert(selected_turn_limit==50);
  fake_now=200;rx(0xE4);tick(200);
  assert(s4_material_e4_recovery_count==1 && mission_snapshot.state_entry_tick==120);
  MissionTask_StartS4E4Recovery(220); // Active recovery guard also handles direct duplicate calls.
  assert(s4_material_e4_recovery_count==1 && mission_snapshot.state_entry_tick==120);
  turn_aligned=true;tick(240);assert(mission_snapshot.state==MISSION_STATE_S4_E4_FORWARD);
  tick(839);assert(mission_snapshot.left_target_rpm==80 && mission_snapshot.state==MISSION_STATE_S4_E4_FORWARD);
  tick(840);assert(mission_snapshot.state==MISSION_STATE_S4_E4_TURN_LEFT);
  tick(860);assert(mission_snapshot.state==MISSION_STATE_S4_E4_TURN_RIGHT && selected_turn_limit==50);
  tick(880);assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && tx_count==1 && tx_commands[0]==0x04);
  MissionTask_StartS4E4Recovery(900);
  assert(s4_material_e4_recovery_count==2 && mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
  tick(920);assert(mission_snapshot.left_target_rpm==ROBOT_YAW_LEFT_SIGN*40);
  assert(mission_snapshot.right_target_rpm==-ROBOT_YAW_LEFT_SIGN*40);
  fake_now=940;rx(0xE4);tick(940);
  assert(mission_snapshot.state_entry_tick==900 && s4_material_e4_recovery_count==2);
}
int main(void) {
  verify_03_frame_down();
  verify_e4_event_buffering();
  assert(MISSION_STATE_S3_E3_TURN_AWAY==95 && MISSION_STATE_SUPPLEMENT_WAIT_RECHECK==96);
  assert(MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360==97);
  second_loss();
  // Gyro progress crosses0 correctly and follows clockwise, not counterclockwise.
  for(unsigned i=1;i<=12;++i) {
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(100-(int32_t)i*3000*ROBOT_YAW_LEFT_SIGN);
    tick(940+i*20);
  }
  assert(s4_e4_spin_progress_cdeg==36000 && mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER);
  assert(s4_material_e4_recovery_count==2 && s3_target_select_command==0x51);
  assert(tx_count==2 && tx_commands[1]==0x04);
  MissionTask_StartS4E4Recovery(1300);assert(s4_material_e4_recovery_count==2);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
  fake_now=1320;coords(4);tick(1320);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360); // Wrong class cannot interrupt.
  fake_now=1340;coords(6);tick(1340);
  assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && s4_material_e4_recovery_count==2);
  // Coordinate-only resume also preserves the first recovery count.
  setup(0x51);MissionTask_StartS4E4Recovery(120);fake_now=140;coords(5);tick(140);
  assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && s4_material_e4_recovery_count==1);
  MissionTask_StartS4E4Recovery(160);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
  fake_now=180;coords(6);tick(681); //501ms-old frame must not interrupt.
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
  fake_now=700;rx(0x14);tick(700);assert(acknowledgements==1);
  // Black31 has the same two-stage policy, without being forced to51.
  setup(0x31);MissionTask_StartS4E4Recovery(120);fake_now=140;coords(6);tick(140);
  MissionTask_StartS4E4Recovery(160);assert(mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
  assert(s3_target_select_command==0x31);
  fake_now=170;coords(5);tick(170);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
  //05 remains able to preempt the new spin; health faults still stop it.
  assert(MissionTask_CanPauseForLoadCheck());
  health_faults=MISSION_FAULT_IMU;tick(180);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  second_loss();tick(900+ROBOT_S4_E4_MATERIAL_SPIN_TIMEOUT_MS);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.right_target_rpm);
  // Independent new-target and new07 resets, leaving red/initial-green E4 unchanged.
  setup(0x51);s4_material_e4_recovery_count=2;
  assert(MissionTask_StartS3Align(120,0x11) && !s4_material_e4_recovery_count);
  setup(0x51);s4_material_e4_recovery_count=2;MissionTask_StartS7Decision(120);
  assert(!s4_material_e4_recovery_count && tx_count==2 && tx_commands[0]==0x11 && tx_commands[1]==0x03);
  setup(0x11);MissionTask_StartS4E4Recovery(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_RED_SPIN_360 && !s4_material_e4_recovery_count);
  tick(140);assert(mission_snapshot.left_target_rpm==-ROBOT_YAW_LEFT_SIGN*20);
  setup(0x21);s7_target_switch_enabled=false;MissionTask_StartS4E4Recovery(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT && s4_green_e4_push_cycles==1 && !s4_material_e4_recovery_count);
  s4_green_e4_push_cycles=2;MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,140);
  MissionTask_StartS4E4Recovery(160);tick(180);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_GREEN_SPIN_360);
  assert(mission_snapshot.left_target_rpm==-ROBOT_YAW_LEFT_SIGN*20);
  assert(rx_bytes>0 && fake_rx_arms>=rx_bytes);
  puts("PASS:after07 black/green E4 first50rpm sweep,second gyro CW360,cross-zero/duplicates/fresh/stale/class/04/14,independent counts/reset,safety and legacy red/first-green unchanged");
}
'@
$parserPath = (Join-Path $projectRoot 'maixcam/maixcam_task.c').Replace('\','/')
$parserInclude = "`n#define MaixCam_SendCommand Parser_SendCommand`n#include `"$parserPath`"`n#undef MaixCam_SendCommand`n"
$testExe = Join-Path $outputDir 'verify_s4_material_e4.exe'
$eventCases = Get-Content (Join-Path $PSScriptRoot 'host/e4_event_cases.c') -Raw
$frameCases = Get-Content (Join-Path $PSScriptRoot 'host/03_frame_cases.c') -Raw
$code = $preamble+"`n"+$globals+"`n"+$prototypes+$parserInclude+$stubs+"`n"+($functions -join "`n")+"`n"+$arrangeWait+"`n"+$cases+"`n"+$eventCases+"`n"+$frameCases
$code | & $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/Task" "-I$projectRoot/maixcam" -x c - -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
