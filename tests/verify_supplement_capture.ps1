param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/supplement_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
. (Join-Path $PSScriptRoot 'host/mission_sources.ps1')
$missionFixture = Get-MissionHostFixture $projectRoot
$source = $missionFixture.Source
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
$names = @('MissionTask_GetVisionMotionProfile', 'MissionTask_S7MaterialOnly', 'MissionTask_WrapYaw', 'MissionTask_YawError', 'MissionTask_Abs32', 'MissionTask_ClampFloat',
  'MissionTask_HeadingTurnExpired',
  'MissionTask_ResetVisionControllerState', 'MissionTask_ResetVisionPid', 'MissionTask_ResetS6ControllerState',
  'MissionTask_ActiveRecoveryLossEvent', 'MissionTask_DropDuplicateRecoveryEvent',
  'MissionTask_IsObjectTrackingPhase', 'MissionTask_RedCoordinateIsFresh',
  'MissionTask_UpdateVisionInput', 'MissionTask_GetSelectedObjectId', 'MissionTask_GetCarriedObjectId',
  'MissionTask_UpdateS5CarriedObject', 'MissionTask_EnterS5', 'MissionTask_RunS5WaitSingleGreen',
  'MissionTask_StartS6FromLoad', 'MissionTask_S6IsSupplyTarget', 'MissionTask_S6IsCasualtyTarget',
  'MissionTask_ResetSupplement', 'MissionTask_HandleSupplementFinish', 'MissionTask_FenceSupplementCoordinates',
  'MissionTask_SendSupplementCommand', 'MissionTask_StartSupplement', 'MissionTask_HandleSupplementControl',
  'MissionTask_StartSupplementRecovery', 'MissionTask_ResumeSupplement', 'MissionTask_RunSupplement',
  'MissionTask_GuardVisionForward', 'MissionTask_CalculateVisionForward', 'MissionTask_CalculateVisionTurn', 'MissionTask_SetTrackingTargets',
  'MissionTask_VisionAgeScale', 'MissionTask_SetVisionTrackingTargets', 'MissionTask_CommandClawApproach', 'MissionTask_CommandS4VisionTracking',
  'MissionTask_CalculateS6YawPid', 'MissionTask_CalculateS6YawTurn', 'MissionTask_RunS6TurnToHeading',
  'MissionTask_CheckRunningHealth', 'MissionTask_HandleRedPriorityRequest')
$names += @('MissionTask_ReportS6SearchSoftTimeout', 'MissionTask_RunS6SearchSafeZone',
  'MissionTask_StartS6Recovery', 'MissionTask_TryFinishS6Recovery', 'MissionTask_RunS6Recovery',
  'MissionTask_ResetS6PreDecisionTracking', 'MissionTask_StartS6VisionApproach',
  'MissionTask_EnterS6TrackingFrom16', 'MissionTask_StartS6FoundBafflePulse',
  'MissionTask_CommandS6Straight', 'MissionTask_RunS5PrepareArrange',
  'MissionTask_StartS6FinalFrom26', 'MissionTask_HandleS6AlignRequest')
$names += @('MissionTask_StartS6LoadRecheck', 'MissionTask_RunS5CasualtyAdjust', 'MissionTask_RunS6LoadRecheck')
$names += @('MissionTask_GetS6CornerHeading', 'MissionTask_StartS6CornerEvacuation')
$names += @('MissionTask_CanPauseForLoadCheck', 'MissionTask_HandleLoadCheckRequest',
  'MissionTask_WaitFinalLoadCheck', 'MissionTask_LoadCheckPausesMotion', 'MissionTask_CanLatchCasualtyRegrip')
$names += @('MissionTask_StartS4TrackCenterFrom04', 'MissionTask_AckS4Center',
  'MissionTask_SkipToS4FinalTrack', 'MissionTask_CaptureS4From34', 'MissionTask_FinishS4Capture',
  'MissionTask_PrepareS4CapturePose', 'MissionTask_OpenS4CaptureClaw', 'MissionTask_S4ClawIsOpening',
  'MissionTask_HoldS4ClawOpening',
  'MissionTask_BeginS4Close',
  'MissionTask_TryResumeS4FinalRecovery', 'MissionTask_EndSupplementRun',
  'MissionTask_RetryS3AfterS4', 'MissionTask_StartS5Arrange02',
  'MissionTask_HandleS4FinalTrackTimeout',
  'MissionTask_StartBlackFinalTrack')
$names += @('MissionTask_RunS3Track', 'MissionTask_StartS3Align',
  'MissionTask_BeginS3Vision', 'MissionTask_RunS3LowerFrame')
$names += @('MissionTask_RecordInitialGreenPush', 'MissionTask_RecoverS4FineArrange', 'MissionTask_HandleS4FineArrangeRequest',
  'MissionTask_CompleteS4FineArrange', 'MissionTask_RunS4FineArrange',
  'MissionTask_RecoveryScanLeg',
  'MissionTask_StartS4PushTrial', 'MissionTask_StartS4TrackState', 'MissionTask_TryS4ArrangeEvent',
  'MissionTask_StartS4GreenScatter')
$names += @('MissionTask_StartS4Post22Recovery', 'MissionTask_ResumeS3FromPost22Search',
  'MissionTask_StartPost22BlackGreenSearch', 'MissionTask_HandlePost22E3')
$names += @('MissionTask_CanAvoidStartZone', 'MissionTask_HandleStartZoneAvoidanceRequest',
  'MissionTask_RunStartZoneAvoidance')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$prototypes = ($functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).TrimEnd() + ';' }) -join "`n"
$globals = $missionFixture.Globals
$ruleTypes = [regex]::Match($source, '(?m)^typedef struct\r?\n\{\r?\n  uint8_t start_zone;[\s\S]*?^static const MissionSideRule s6_corner_rules\[\].*;').Value
if (-not $ruleTypes) { throw 'Missing real side/corner tables' }
$arrange = Extract-Function 'MissionTask_RunS4Arrange'
function Extract-Cases([string]$first, [string]$after) {
  $begin = $arrange.IndexOf($first)
  $end = $arrange.IndexOf($after, $begin)
  if ($begin -lt 0 -or $end -lt 0) { throw "Missing S4 cases $first" }
  $arrange.Substring($begin, $end - $begin)
}
$normalS4 = "static void run_normal_s4(uint32_t now) { uint32_t elapsed=now-mission_snapshot.state_entry_tick; uint8_t event_code; bool xy_in_tolerance,camera_already_in_final_mode; switch(mission_snapshot.state) {`n" +
  (Extract-Cases '    case MISSION_STATE_S4_TRACK_CENTER:' '    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:') +
  (Extract-Cases '    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:' '    case MISSION_STATE_S4_REVERSE_RIGHT_BLOCK:') +
  (Extract-Cases '    case MISSION_STATE_S4_WAIT_LEFT_TARGET:' '    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:') +
  (Extract-Cases '    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:' '    case MISSION_STATE_S4_REVERSE_LEFT_BLOCK:') +
  (Extract-Cases '    case MISSION_STATE_S4_WAIT_FINAL_READY:' '    case MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE:') +
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
#include "chassis_motion.h"
#include "heading_control.h"
#include "chassis_actions.h"
#include "flow_modules.h"
#include "vision_motion.h"
#include "robot_config.h"
#include "maixcam_task.h"
typedef void *osThreadId_t;
typedef void *osEventFlagsId_t;
typedef void *osMutexId_t;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define MISSION_TASK_PERIOD_TICKS 20U
#define MISSION_VISION_TARGET_TIMEOUT_MS MAIXCAM_DATA_TIMEOUT_MS
#define MISSION_LEFT_MOTOR_INDEX (ROBOT_LEFT_WHEEL_MOTOR_ID - 1U)
#define MISSION_RIGHT_MOTOR_INDEX (ROBOT_RIGHT_WHEEL_MOTOR_ID - 1U)
uint32_t fake_now;
unsigned fake_rx_arms;
'@
$stubs = @'
static UART_HandleTypeDef uart;
static struct { int16_t speed_rpm; } moto_chassis[4];
static unsigned stops, wide_calls, track_view_calls, safe_view_calls, near_calls, frame_raise_calls, frame_partial_calls, frame_lower_calls, rx_bytes, tx_count, uart_inits;
static unsigned claw_close_calls, nonzero_wheel_requests;
static unsigned baffle_stop_calls;
static unsigned final_view_calls;
static unsigned claw_open_calls;
static uint16_t claw_angle;
static unsigned turn_soft_logs, search_soft_logs, red_switch_success_logs;
static uint16_t frame_lift;
static uint8_t tx_commands[64], inject06_on_tx;
static uint32_t first34_ack_delay_ms;
static uint8_t inject_retry03_id;
static bool inject06_on45;
static bool inject_e3_on13_tx;
static uint8_t inject_push_reply_on_tx;
static uint8_t inject05_result_on_tx, inject05_result_on_stop;
static uint8_t inject_ae_coord_id, inject_ae_event;
static bool inject_ae_before_coordinate;
static uint32_t inject_ae_tx_delay;
static bool wide_ok, near_ok, frame_lower_ok, claw_close_ok, claw_open_ok, tx_ok, expire_on_tx;
static bool final_view_ok;
static uint32_t health_faults;
static void byte(uint8_t b) {
  ++rx_bytes; maixcam_rx_byte=b; MaixCam_UART_RxCpltCallback(maixcam_uart);
}
static void rx_wire(uint8_t cmd) {
  const uint8_t f[]={0xF1,0xF2,cmd,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static void rx(uint8_t cmd) {
  rx_wire(cmd==0x34 ? MAIXCAM_EVENT_GEOMETRY_CAPTURE : cmd);
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
  if(cmd==0x15 || cmd==0x25)
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  if(cmd==0x05) assert(frame_lift==0 && !mission_snapshot.left_target_rpm &&
      !mission_snapshot.right_target_rpm && maixcam_load_recheck_enabled);
  assert(tx_count<sizeof(tx_commands)); tx_commands[tx_count++]=cmd;
  if(cmd==0x34) {
    assert(claw_angle==ROBOT_CLAW_CLOSED_DEG && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    fake_now+=first34_ack_delay_ms;
    if(first34_ack_delay_ms) rx(0x34); // Old first-capture retry while its ACK is in flight.
  }
  if(cmd==0x45) {
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && claw_angle==36);
    if(inject06_on45) { inject06_on45=false;coords(4,0,0);rx(0x06);rx(0x16); }
  }
  if(cmd==0x03 && inject_retry03_id) {
    assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    coords(inject_retry03_id,15,95);inject_retry03_id=0;
    rx(0x04);
  }
  if(cmd==0xAE) {
    assert((!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm) || ae_ack_completed);
    fake_now+=inject_ae_tx_delay;inject_ae_tx_delay=0;
    if(inject_ae_before_coordinate) { inject_ae_before_coordinate=false;rx(0xAE); }
    if(inject_ae_coord_id) { uint8_t id=inject_ae_coord_id;inject_ae_coord_id=0;coords(id,0,50); }
    if(inject_ae_event==0x05) { inject_ae_event=0;review_reply(0x06); }
    else if(inject_ae_event) { uint8_t event=inject_ae_event;inject_ae_event=0;rx(event); }
  }
  if((cmd==0x12 || cmd==0x14 || cmd==0x22) && inject_push_reply_on_tx) {
    uint8_t next=inject_push_reply_on_tx;inject_push_reply_on_tx=0;rx(next);
  }
  if(cmd==0x13 && inject_e3_on13_tx) { inject_e3_on13_tx=false;rx(0xE3); }
  if(inject06_on_tx==cmd) {
    inject06_on_tx=0; review_reply(0x06);
  }
  if(cmd==0x05 && inject05_result_on_tx) {
    uint8_t result=inject05_result_on_tx; inject05_result_on_tx=0;
    review_reply(result);
  }
  if(expire_on_tx) { expire_on_tx=false; fake_now=supplement_first_request_tick+ROBOT_SUPPLEMENT_TOTAL_MS; }
  return tx_ok?HAL_OK:HAL_ERROR;
}
static bool DebugUart_Log(const char *s) { (void)s; return true; }
static bool DebugUart_Logf(const char *s,...) {
  char line[512]; va_list a; va_start(a,s); vsnprintf(line,sizeof(line),s,a); va_end(a);
  if(strstr(line,"[07-R] TX11/03 OK WAIT ID4")) ++red_switch_success_logs;
  if(strstr(line,"[SUPP]")) assert(strlen(line)<128);
  if(strstr(line,"[CHECK05]")) assert(strlen(line)<128);
  if(strstr(line,"[LOAD05]")) assert(strlen(line)<128);
  if(strstr(line,"[6E-TMO]")) {
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
  if(l || r) ++nonzero_wheel_requests;
  mission_snapshot.left_target_rpm=l; mission_snapshot.right_target_rpm=r;
}
static void MissionTask_ResetS6XConfirmation(void) {}
static void MissionTask_DebugS6(uint32_t now) { (void)now; }
static HAL_StatusTypeDef Actuator_SetCameraWideView(void) { ++wide_calls; return wide_ok?HAL_OK:HAL_ERROR; }
static HAL_StatusTypeDef Actuator_SetCameraObjectTrackView(void) { ++track_view_calls; return wide_ok?HAL_OK:HAL_ERROR; }
static HAL_StatusTypeDef Actuator_SetCameraSafeZoneView(void) { ++safe_view_calls; return wide_ok?HAL_OK:HAL_ERROR; }
static HAL_StatusTypeDef Actuator_SetCameraFinalTrackView(void) {
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  ++final_view_calls;return final_view_ok?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Actuator_SetCameraNearView(void) {
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  ++near_calls; return near_ok?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Actuator_StopBafflePwm(void) { ++baffle_stop_calls; return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetBaffleAngle(uint16_t angle_deg) {
  assert(angle_deg==ROBOT_BAFFLE_DOWN_DEG);
  frame_lift=0;return frame_lower_ok?HAL_OK:HAL_ERROR;
}
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) {
  ++frame_lower_calls;
  if(!frame_lower_ok) return HAL_ERROR;
  frame_lift=0; claw_angle=ROBOT_CLAW_CLOSED_DEG; return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetClawClosed(void) {
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  ++claw_close_calls;
  if(!claw_close_ok) return HAL_ERROR;
  claw_angle=ROBOT_CLAW_CLOSED_DEG; return HAL_OK;
}
static uint8_t claw_close_inhibited;
static uint8_t Actuator_ClawCloseDeferred(void) { return claw_close_inhibited; }
static HAL_StatusTypeDef Actuator_SetClawAngle(uint16_t angle_deg) {
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  ++claw_open_calls;
  if(!claw_open_ok) return HAL_ERROR;
  claw_angle=angle_deg; return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetBaffleUpClawOpen(uint16_t angle_deg) {
  if(Actuator_SetClawAngle(angle_deg)!=HAL_OK) return HAL_ERROR;
  ++frame_raise_calls; frame_lift=ROBOT_BAFFLE_DOWN_DEG-ROBOT_BAFFLE_UP_DEG;
  return HAL_OK;
}
static void MissionTask_EnterState(MissionState state,uint32_t now) {
  if(mission_snapshot.state==state) return;
  if(state!=MISSION_STATE_S6_RECHECK_WAIT_RESULT) {
    if(s5_casualty_adjust_active) MaixCam_SetLoadRecheckEnabled(false);
    s5_casualty_adjust_active=s5_casualty_adjust_done=false;
  }
  mission_snapshot.state=state; mission_snapshot.state_entry_tick=now;
  if(state!=MISSION_STATE_S4_RAISE_FRAME && state!=MISSION_STATE_S4_TRACK_FINAL_BLOCK &&
      state!=MISSION_STATE_S4_FINAL_CENTER_OBJECT) s4_capture_claw_open=false;
  if(state!=MISSION_STATE_S4_FINAL_CENTER_OBJECT && state!=MISSION_STATE_S5_WAIT_SINGLE_GREEN &&
      state!=MISSION_STATE_S6_RECHECK_WAIT_RESULT) s5_casualty_timeout_check=false;
  MaixCam_SetCasualtyRegripEnabled(MissionTask_CanLatchCasualtyRegrip(state));
  if(state<MISSION_STATE_S4_FINE32_TRACK || state>MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT)
    s4_initial_green02_active=false;
  if(state!=MISSION_STATE_S4_WAIT_ARRANGE_READY) s4_fine_waiting_decision=s4_direct_capture_wait24=false;
  if(state!=MISSION_STATE_S4_E4_PUSH_RIGHT) s4_green_e4_half_turn_active=false;
  if(state!=MISSION_STATE_S6_RECOVERY_START) s6_recovery_first_forward_active=false;
  MaixCam_SetFineArrangeActive(state==MISSION_STATE_S4_TRACK_RIGHT_BLOCK ||
      (state>=MISSION_STATE_S4_FINE32_TRACK && state<=MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT));
  MaixCam_SetArrange02Active(state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE ||
      state==MISSION_STATE_S4_TRACK_RIGHT_BLOCK ||
      (state>=MISSION_STATE_S4_FINE32_TRACK && state<=MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT &&
       s4_fixed_motion_ack==MAIXCAM_COMMAND_RIGHT_DONE_ACK));
  MaixCam_SetStartZoneAvoidanceActive(state==MISSION_STATE_VISION_AVOID_START_TURN ||
      state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK);
  if(state!=MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW) supplement_e3_first_hold_active=false;
  MaixCam_SetSupplementE3TurnActive(state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW &&
      supplement_e3_spin_goal_cdeg==ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG &&
      !supplement_e3_first_hold_active);
  if(state!=MISSION_STATE_VISION_AVOID_START_TURN &&
      state!=MISSION_STATE_VISION_AVOID_START_WAIT_ACK && !MissionTask_CanAvoidStartZone())
    ae_ack_completed=ae_reply_pending=false;
  MaixCam_SetCornerEvacuationActive(state>=MISSION_STATE_S6_CORNER_TURN_AWAY &&
      state<=MISSION_STATE_S6_CORNER_TURN_SAFE);
  MaixCam_SetSupplementDirectSearch(supplement_capture_active && !supplement_budget_exhausted &&
      (state==MISSION_STATE_SUPPLEMENT_WIDE_SETTLE || state==MISSION_STATE_SUPPLEMENT_TRACK ||
       state==MISSION_STATE_SUPPLEMENT_TURN_LEFT || state==MISSION_STATE_SUPPLEMENT_TURN_RIGHT ||
       state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW ||
       ((state==MISSION_STATE_VISION_AVOID_START_TURN || state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK) &&
        ae_resume_state==MISSION_STATE_SUPPLEMENT_TRACK)));
  MaixCam_SetLoadCheckInterruptEnabled(MissionTask_CanPauseForLoadCheck());
  MaixCam_SetLoadEmptyRecoveryEnabled((state==MISSION_STATE_S5_WAIT_SINGLE_GREEN ||
      state==MISSION_STATE_S6_RECHECK_WAIT_RESULT) && !s5_recheck_resume_safe_tracking &&
      !s7_target_switch_enabled && s3_target_select_command==0x21);
  MaixCam_SetSupplementLoadCheckEnabled(
      (state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && s5_close_view_active) ||
      state==MISSION_STATE_SUPPLEMENT_LOAD_CHECK);
  MaixCam_SetRecoveryLossFilter(MissionTask_ActiveRecoveryLossEvent());
  if(state==MISSION_STATE_FAULT || state==MISSION_STATE_STOPPED || state==MISSION_STATE_WAIT_START) {
    MissionTask_ResetSupplement();
    ae_ack_completed=ae_reply_pending=false;
  }
  MissionTask_StopWheels();
}
static uint32_t MissionTask_GetRequiredFaults(void) { return health_faults; }
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
/* The ordinary post22 E3 recovery is covered by verify_s51_e3_stages.
   Keep the real post22 dispatcher here for12 ->22 ->24/04/14 handshakes. */
static bool MissionTask_HandleS3TargetLost(uint32_t now) {
  (void)now; assert(!"ordinary post22 E3 is covered by its separate regression test"); return false;
}
static void MissionTask_DebugS3(uint32_t now) { (void)now; }
'@
$cases = (Get-Content (Join-Path $PSScriptRoot 'host/vision_approach_cases.c') -Raw) + "`n" +
  "static void verify_fine_arrange_integration(void);`n" +
  "static void verify_start_zone_avoidance(void);`n" +
  (Get-Content (Join-Path $PSScriptRoot 'host/load05_cases.c') -Raw) + "`n" +
  (Get-Content (Join-Path $PSScriptRoot 'host/s4_fine_integration_cases.c') -Raw) + "`n" +
  (Get-Content (Join-Path $PSScriptRoot 'host/start_zone_avoidance_cases.c') -Raw)
$parserPath = (Join-Path $projectRoot 'User/Runtime/Comms/src/maixcam_task.c').Replace('\','/')
$parserInclude = "`n#define MaixCam_SendCommand Parser_SendCommand`n#include `"$parserPath`"`n#undef MaixCam_SendCommand`n"
$testSource = Join-Path $outputDir 'verify_supplement_capture.c'
[IO.File]::WriteAllText($testSource, $preamble+$parserInclude+$globals+"`n"+$ruleTypes+"`n"+$prototypes+"`n"+$stubs+"`n"+($functions -join "`n")+"`n"+$normalS4+"`n"+$cases,[Text.UTF8Encoding]::new($false))
$testExe = Join-Path $outputDir 'verify_supplement_capture.exe'
. (Join-Path $PSScriptRoot 'host/flow_build.ps1') $projectRoot
& $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/User/Config/inc" "-I$projectRoot/User/Mission/State/inc" "-I$projectRoot/User/Runtime/Comms/inc" "-I$projectRoot/User/Hardware/Serial/inc" "-I$projectRoot/User/Motion/Chassis/inc" $testSource (Join-Path $projectRoot 'User/Motion/Chassis/src/heading_control.c') "-I$projectRoot/User/Motion/Vision/inc" (Join-Path $projectRoot 'User/Motion/Chassis/src/chassis_actions.c') (Join-Path $projectRoot 'User/Motion/Vision/src/vision_motion.c') @flowBuildArgs -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
