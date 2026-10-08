param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/supplement_host'
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
$names = @('MissionTask_WrapYaw', 'MissionTask_YawError', 'MissionTask_Abs32', 'MissionTask_ClampFloat',
  'MissionTask_HeadingTurnExpired',
  'MissionTask_ResetVisionControllerState', 'MissionTask_ResetVisionPid', 'MissionTask_ResetS6ControllerState',
  'MissionTask_ActiveRecoveryLossEvent', 'MissionTask_IsObjectTrackingPhase', 'MissionTask_RedCoordinateIsFresh',
  'MissionTask_UpdateVisionInput', 'MissionTask_GetSelectedObjectId', 'MissionTask_GetCarriedObjectId',
  'MissionTask_UpdateS5CarriedObject', 'MissionTask_EnterS5', 'MissionTask_RunS5WaitSingleGreen',
  'MissionTask_StartS6FromLoad', 'MissionTask_S6IsSupplyTarget', 'MissionTask_S6IsCasualtyTarget',
  'MissionTask_ResetSupplement', 'MissionTask_HandleSupplementFinish', 'MissionTask_FenceSupplementCoordinates',
  'MissionTask_SendSupplementCommand', 'MissionTask_StartSupplement', 'MissionTask_HandleSupplementControl',
  'MissionTask_StartSupplementRecovery', 'MissionTask_ResumeSupplement', 'MissionTask_RunSupplement',
  'MissionTask_GuardVisionForward', 'MissionTask_CalculateVisionForward', 'MissionTask_CalculateVisionTurn', 'MissionTask_SetTrackingTargets',
  'MissionTask_VisionAgeScale', 'MissionTask_SetVisionTrackingTargets', 'MissionTask_CommandS4VisionTracking',
  'MissionTask_CalculateS6YawPid', 'MissionTask_CalculateS6YawTurn', 'MissionTask_RunS6TurnToHeading',
  'MissionTask_CheckRunningHealth', 'MissionTask_HandleRedPriorityRequest')
$names += @('MissionTask_ReportS6SearchSoftTimeout', 'MissionTask_RunS6SearchSafeZone',
  'MissionTask_StartS6Recovery', 'MissionTask_TryFinishS6Recovery', 'MissionTask_RunS6Recovery',
  'MissionTask_ResetS6PreDecisionTracking', 'MissionTask_StartS6VisionApproach',
  'MissionTask_EnterS6TrackingFrom16', 'MissionTask_CommandS6Straight', 'MissionTask_RunS5PrepareArrange',
  'MissionTask_StartS6FinalFrom26', 'MissionTask_HandleS6AlignRequest')
$names += @('MissionTask_StartS6LoadRecheck', 'MissionTask_RunS6LoadRecheck')
$names += @('MissionTask_CanPauseForLoadCheck', 'MissionTask_HandleLoadCheckRequest',
  'MissionTask_WaitFinalLoadCheck', 'MissionTask_LoadCheckPausesMotion')
$names += @('MissionTask_StartS4TrackCenterFrom04', 'MissionTask_AckS4Center',
  'MissionTask_SkipToS4FinalTrack', 'MissionTask_EnterS4FinalCenter', 'MissionTask_S4FinalFrameLowerSettleMs',
  'MissionTask_StartBlackFinalTrack')
$names += @('MissionTask_RunS3Track')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$prototypes = ($functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).TrimEnd() + ';' }) -join "`n"
$start = $source.IndexOf('static osThreadId_t mission_task_handle;')
$globals = $source.Substring($start, $source.IndexOf('static const osThreadAttr_t mission_task_attributes') - $start)
$arrange = Extract-Function 'MissionTask_RunS4Arrange'
function Extract-Cases([string]$first, [string]$after) {
  $begin = $arrange.IndexOf($first)
  $end = $arrange.IndexOf($after, $begin)
  if ($begin -lt 0 -or $end -lt 0) { throw "Missing S4 cases $first" }
  $arrange.Substring($begin, $end - $begin)
}
$normalS4 = "static void run_normal_s4(uint32_t now) { uint32_t elapsed=now-mission_snapshot.state_entry_tick; uint8_t event_code; bool xy_in_tolerance,camera_already_in_final_mode; switch(mission_snapshot.state) {`n" +
  (Extract-Cases '    case MISSION_STATE_S4_TRACK_CENTER:' '    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:') +
  (Extract-Cases '    case MISSION_STATE_S4_RAISE_FRAME:' '    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:') +
  (Extract-Cases '    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:' '    default:') +
  " default: break; } }`n"
$preamble = @'
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition); exit(1); } } while(0)
#include "mission_task.h"
#include "robot_config.h"
#include "maixcam_task.h"
typedef void *osThreadId_t;
typedef void *osEventFlagsId_t;
typedef void *osMutexId_t;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define MISSION_TASK_PERIOD_TICKS 20U
#define MISSION_VISION_TARGET_TIMEOUT_MS MAIXCAM_DATA_TIMEOUT_MS
uint32_t fake_now;
unsigned fake_rx_arms;
'@
$stubs = @'
static UART_HandleTypeDef uart;
static unsigned stops, wide_calls, near_calls, frame_raise_calls, frame_partial_calls, frame_lower_calls, rx_bytes, tx_count, uart_inits;
static unsigned turn_soft_logs, search_soft_logs;
static uint16_t frame_lift;
static uint8_t tx_commands[64], inject06_on_tx;
static uint8_t inject05_result_on_tx, inject05_result_on_stop;
static bool wide_ok, near_ok, frame_lower_ok, tx_ok, expire_on_tx;
static uint32_t health_faults;
static void byte(uint8_t b) {
  ++rx_bytes; maixcam_rx_byte=b; MaixCam_UART_RxCpltCallback(maixcam_uart);
}
static void rx(uint8_t cmd) {
  const uint8_t f[]={0xF1,0xF2,cmd,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static void coords(uint8_t id,uint16_t x,uint16_t y) {
  const uint8_t f[]={0xF1,0xF2,id,x>>8,x&255,1,y>>8,y&255,2,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static void review_reply(uint8_t result) {
  rx(0x05); coords(5,0,0); rx(result);
  if(result==0x06) rx(0x16);
}
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t cmd) {
  if(cmd==0x03) assert(frame_lift==0);
  if(cmd==0x05) assert(frame_lift==0 && !mission_snapshot.left_target_rpm &&
      !mission_snapshot.right_target_rpm && maixcam_load_recheck_enabled);
  assert(tx_count<sizeof(tx_commands)); tx_commands[tx_count++]=cmd;
  if(inject06_on_tx==cmd) {
    inject06_on_tx=0; review_reply(0x06);
  }
  if(cmd==0x05 && inject05_result_on_tx) {
    uint8_t result=inject05_result_on_tx; inject05_result_on_tx=0;
    review_reply(result);
  }
  if(expire_on_tx) { expire_on_tx=false; fake_now=supplement_first_request_tick+15000U; }
  return tx_ok?HAL_OK:HAL_ERROR;
}
static bool DebugUart_Log(const char *s) { (void)s; return true; }
static bool DebugUart_Logf(const char *s,...) {
  char line[512]; va_list a; va_start(a,s); vsnprintf(line,sizeof(line),s,a); va_end(a);
  if(strstr(line,"[SUPP]")) assert(strlen(line)<128);
  if(strstr(line,"[S6-CHECK]")) assert(strlen(line)<128);
  if(strstr(line,"[LOAD05]")) assert(strlen(line)<128);
  if(strstr(line,"[S6-TMO]")) {
    assert(strlen(line)<128);
    if(strstr(line,"TURN SOFT")) ++turn_soft_logs;
    if(strstr(line,"SEARCH SOFT")) ++search_soft_logs;
  }
  if(strstr(line,"[SUPP] STOP")==line && inject05_result_on_stop) {
    uint8_t result=inject05_result_on_stop; inject05_result_on_stop=0;
    review_reply(result);
  }
  return true;
}
static void MissionTask_StopWheels(void) {
  ++stops; mission_snapshot.left_target_rpm=mission_snapshot.right_target_rpm=0;
}
static void MissionTask_SetWheelTargets(int16_t l,int16_t r) {
  mission_snapshot.left_target_rpm=l; mission_snapshot.right_target_rpm=r;
}
static void MissionTask_ResetS6XConfirmation(void) {}
static void MissionTask_DebugS6(uint32_t now) { (void)now; }
static HAL_StatusTypeDef Actuator_SetCameraWideView(void) { ++wide_calls; return wide_ok?HAL_OK:HAL_ERROR; }
static HAL_StatusTypeDef Actuator_SetCameraNearView(void) { ++near_calls; return near_ok?HAL_OK:HAL_ERROR; }
static HAL_StatusTypeDef Actuator_SetFrameRaised(void) { ++frame_raise_calls; frame_lift=88; return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) {
  ++frame_lower_calls;
  if(!frame_lower_ok) return HAL_ERROR;
  frame_lift=0; return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetFramePartiallyRaised(uint16_t lift) { ++frame_partial_calls; frame_lift=lift; return HAL_OK; }
static void MissionTask_EnterState(MissionState state,uint32_t now) {
  if(mission_snapshot.state==state) return;
  mission_snapshot.state=state; mission_snapshot.state_entry_tick=now;
  MaixCam_SetLoadCheckInterruptEnabled(MissionTask_CanPauseForLoadCheck());
  MaixCam_SetLoadEmptyRecoveryEnabled((state==MISSION_STATE_S5_WAIT_SINGLE_GREEN ||
      state==MISSION_STATE_S6_RECHECK_WAIT_RESULT) && !s5_recheck_resume_safe_tracking &&
      !s7_target_switch_enabled && s3_target_select_command==0x21);
  MaixCam_SetSupplementLoadCheckEnabled(
      (state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && s5_close_view_active) ||
      state==MISSION_STATE_SUPPLEMENT_LOAD_CHECK);
  MaixCam_SetRecoveryLossFilter(MissionTask_ActiveRecoveryLossEvent());
  if(state==MISSION_STATE_FAULT || state==MISSION_STATE_STOPPED || state==MISSION_STATE_WAIT_START)
    MissionTask_ResetSupplement();
  MissionTask_StopWheels();
}
static uint32_t MissionTask_GetRequiredFaults(void) { return health_faults; }
static void MissionTask_StartS4TrackState(MissionState state,uint32_t now) {
  MaixCam_ClearObject(); MissionTask_ResetVisionPid(); MissionTask_EnterState(state,now);
}
static void MissionTask_StartS4FinalRecovery(uint32_t now) {
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_RECOVERY_REVERSE,now);
}
static void MissionTask_StartS4E4Recovery(uint32_t now) {
  MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_AWAY,now);
}
static void MissionTask_StartS6ObstacleAvoidance(uint32_t now) {
  (void)now; assert(!"obstacle36 is covered by its separate regression test");
}
static void MissionTask_StartVisionSearchRecovery(uint32_t now,MissionState resume,int32_t yaw,bool green) {
  (void)yaw; (void)green; vision_search_resume_state=resume;
  MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CW,now);
}
static bool MissionTask_TryStartS3Capture(uint32_t now) { (void)now; return false; }
static void MissionTask_DebugS3(uint32_t now) { (void)now; }
static bool MissionTask_StartS3Align(uint32_t now,uint8_t cmd) {
  (void)now; (void)cmd; assert(!"supplement must not use ordinary search/red switch"); return false;
}
'@
$cases = (Get-Content (Join-Path $PSScriptRoot 'host/vision_approach_cases.c') -Raw) + "`n" +
  (Get-Content (Join-Path $PSScriptRoot 'host/load05_cases.c') -Raw)
$parserPath = (Join-Path $projectRoot 'maixcam/maixcam_task.c').Replace('\','/')
$parserInclude = "`n#define MaixCam_SendCommand Parser_SendCommand`n#include `"$parserPath`"`n#undef MaixCam_SendCommand`n"
$testSource = Join-Path $outputDir 'verify_supplement_capture.c'
[IO.File]::WriteAllText($testSource, $preamble+$parserInclude+$globals+"`n"+$prototypes+"`n"+$stubs+"`n"+($functions -join "`n")+"`n"+$normalS4+"`n"+$cases,[Text.UTF8Encoding]::new($false))
$testExe = Join-Path $outputDir 'verify_supplement_capture.exe'
& $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/Task" "-I$projectRoot/maixcam" $testSource -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
