param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/s51_e3_host'
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
  'MissionTask_HeadingTurnExpired', 'MissionTask_RunS3GreenE3Recovery',
  'MissionTask_ClampFloat', 'MissionTask_GetSafeZoneYawOffset',
  'MissionTask_ResetVisionControllerState', 'MissionTask_ResetVisionPid',
  'MissionTask_ResetS6ControllerState', 'MissionTask_ActiveRecoveryLossEvent',
  'MissionTask_DropDuplicateRecoveryEvent', 'MissionTask_IsObjectTrackingPhase',
  'MissionTask_RedCoordinateIsFresh', 'MissionTask_UpdateVisionInput',
  'MissionTask_BeginS3Vision', 'MissionTask_HandleS3TargetLost', 'MissionTask_TryStartS3Capture',
  'MissionTask_StartVisionSearchRecovery', 'MissionTask_RunS3SearchTurn',
  'MissionTask_CalculateS6YawPid', 'MissionTask_CalculateS6YawTurn',
  'MissionTask_ResumeRedTracking', 'MissionTask_CompleteS7RedRecovery',
  'MissionTask_CheckRunningHealth', 'MissionTask_HandleRedPriorityRequest')
$names += @('MissionTask_StartS4Post22Recovery', 'MissionTask_ResumeS3FromPost22Search',
  'MissionTask_StartPost22BlackGreenSearch', 'MissionTask_HandlePost22E3',
  'MissionTask_EscalatePost22Search', 'MissionTask_RunS4Post22Recovery',
  'MissionTask_RunS6TurnToHeading', 'MissionTask_RunS6ExitTurn180',
  'MissionTask_RunS7Decision', 'MissionTask_DebugYaw',
  'MissionTask_DebugVisionAge', 'MissionTask_DebugAngles',
  'MissionTask_S4FinalFrameLowerSettleMs')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$prototypes = ($functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).TrimEnd() + ';' }) -join "`n"
$start = $source.IndexOf('static osThreadId_t mission_task_handle;')
$globals = $source.Substring($start, $source.IndexOf('static const osThreadAttr_t mission_task_attributes') - $start)
$arrange = Extract-Function 'MissionTask_RunS4Arrange'
$waitBegin = $arrange.IndexOf('    case MISSION_STATE_S4_WAIT_FINAL_READY:')
$waitEnd = $arrange.IndexOf('    case MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE:', $waitBegin)
if ($waitBegin -lt 0 -or $waitEnd -lt 0) { throw 'Missing real post22 wait case' }
$post22Wait = "static void run_post22_wait(uint32_t now) { uint32_t elapsed=now-mission_snapshot.state_entry_tick; uint8_t event_code; switch(mission_snapshot.state) {`n" +
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
#define MISSION_TASK_PERIOD_TICKS 20U
typedef void *osThreadId_t;
typedef void *osEventFlagsId_t;
typedef void *osMutexId_t;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
uint32_t fake_now;
unsigned fake_rx_arms;
'@
$stubs = @'
static UART_HandleTypeDef uart;
static unsigned rx_bytes, tx_count, red_switches, center_entries;
static unsigned yaw_prints, age_prints, angle_prints;
static uint8_t tx_commands[64];
static uint8_t inject_red_on_tx, inject_red_event, inject_red_frame_kind;
static bool frame_down;
static uint32_t health_faults;
static void byte(uint8_t b) {
  ++rx_bytes; maixcam_rx_byte=b; MaixCam_UART_RxCpltCallback(maixcam_uart);
}
static void rx(uint8_t cmd) {
  const uint8_t f[]={0xF1,0xF2,cmd,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static void coords(uint8_t id) {
  const uint8_t f[]={0xF1,0xF2,id,0,20,1,0,30,2,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(f);++i) byte(f[i]);
}
static bool DebugUart_Log(const char *s) { (void)s; return true; }
static bool DebugUart_Logf(const char *s,...) {
  char line[256]; va_list a; va_start(a,s); vsnprintf(line,sizeof(line),s,a); va_end(a);
  if(strstr(line,"[S51-E3]")) assert(strlen(line)<128);
  if(strstr(line,"[YAW]")) ++yaw_prints;
  if(strstr(line,"[VDBG]")) ++age_prints;
  if(strstr(line,"[ANGLE]")) ++angle_prints;
  return true;
}
static void MissionTask_StopWheels(void) {
  mission_snapshot.left_target_rpm=mission_snapshot.right_target_rpm=0;
}
static void MissionTask_SetWheelTargets(int16_t l,int16_t r) {
  mission_snapshot.left_target_rpm=l; mission_snapshot.right_target_rpm=r;
}
static void MissionTask_DebugS6(uint32_t now) { (void)now; }
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) { frame_down=true;return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetCameraWideView(void) { return HAL_OK; }
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t cmd) {
  if(cmd==0x03) assert(frame_down);
  assert(tx_count<sizeof(tx_commands)); tx_commands[tx_count++]=cmd;
  if(inject_red_on_tx==cmd) {
    inject_red_on_tx=0;
    if(inject_red_frame_kind==4) {
      // Complete the old coordinate that started before11/03.
      byte(30); byte(2); byte(0x1F); byte(0x2F);
    }
    coords(6); coords(4); // Reference isolation must reject even an old ID4.
    if(inject_red_frame_kind==1 || inject_red_frame_kind==4) rx(inject_red_event);
    if(inject_red_frame_kind==2) {
      byte(0xF1); byte(0xF2); byte(inject_red_event); byte(0x1F);
    }
    if(inject_red_frame_kind==3) {
      byte(0xF1); byte(0xF2); byte(4); byte(0); byte(20); byte(1); byte(0);
    }
  }
  return HAL_OK;
}
static void MissionTask_EnterState(MissionState s,uint32_t t) {
  if(mission_snapshot.state==s) return;
  mission_snapshot.state=s; mission_snapshot.state_entry_tick=t;
  MaixCam_SetRecoveryLossFilter(MissionTask_ActiveRecoveryLossEvent());
  if(s==MISSION_STATE_S7_SEARCH_RED || s==MISSION_STATE_S7_WAIT_SECOND_E3 ||
      s==MISSION_STATE_S7_SEARCH_BLACK) {
    s7_last_message_tick=t; s7_last_coordinate_sequence=mission_snapshot.target_sequence;
  }
  if(s==MISSION_STATE_FAULT || s==MISSION_STATE_STOPPED || s==MISSION_STATE_WAIT_START) {
    s51_e3_recovery_count=0; s51_second_e3_pending=false; MissionTask_StopWheels();
  }
}
static void MissionTask_SwitchS7ToBlackGreen(uint32_t t) {
  s3_target_select_command=0x51; assert(MissionTask_BeginS3Vision(t));
}
static bool MissionTask_ReturnToS4TrackCenter(uint32_t t) {
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,t); return true;
}
static void MissionTask_StartS4TrackCenterFrom04(uint32_t t) {
  ++center_entries; MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,t);
}
static void MissionTask_SkipToS4FinalTrack(uint32_t t) {
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME,t);
}
static void MissionTask_AckS4Center(uint32_t t) {
  MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY,t);
}
static void MissionTask_StartS4E4Recovery(uint32_t t) {
  MissionTask_EnterState(MISSION_STATE_S4_E4_PUSH_RIGHT,t);
}
static void MissionTask_StartBlackFinalTrack(uint32_t t) {
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME,t);
}
static void verify_post22_search(void);
static void verify_exit180_events(void);
static void verify_debug_period(void);
static void verify_red_switch_event_fence(void);
static uint32_t MissionTask_GetRequiredFaults(void) { return health_faults; }
static bool MissionTask_HandleSupplementFinish(uint32_t t) { (void)t; return false; }
static bool MissionTask_LoadCheckPausesMotion(void) { return false; }
static bool MissionTask_StartS3Align(uint32_t t,uint8_t cmd) {
  ++red_switches; s3_target_select_command=cmd; return MissionTask_BeginS3Vision(t);
}
'@
$cases = @'
static void tick(uint32_t now) {
  fake_now=now; MissionTask_UpdateVisionInput(now);
  MissionTask_CheckRunningHealth(now); MissionTask_DropDuplicateRecoveryEvent(now);
  MissionTask_RunS3SearchTurn(now);
  if(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN) MissionTask_TryStartS3Capture(now);
  run_post22_wait(now); MissionTask_RunS4Post22Recovery(now);
}
static void setup(uint8_t zone,uint8_t team,int32_t zero) {
  assert(MaixCam_Init(&uart)==HAL_OK); memset(&mission_snapshot,0,sizeof(mission_snapshot));
  mission_snapshot.state=MISSION_STATE_WAIT_START;
  mission_snapshot.start_zone=zone; mission_snapshot.team=team;
  mission_snapshot.yaw_start_cdeg=zero;
  mission_snapshot.safe_zone_yaw_target_cdeg=MissionTask_WrapYaw(zero+MissionTask_GetSafeZoneYawOffset(zone,team));
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(zero+22000);
  s7_target_switch_enabled=true; s3_target_select_command=0x51;
  s7_red_switch_sync_pending=s7_red_coordinate_fence_active=false;
  supplement_budget_started=supplement_capture_active=false;
  health_faults=0; tx_count=red_switches=center_entries=0; fake_now=100;
  inject_red_on_tx=inject_red_event=inject_red_frame_kind=0;
  frame_down=false;
  assert(MissionTask_BeginS3Vision(fake_now)); assert(!s51_e3_recovery_count);
  assert(tx_count==2 && tx_commands[0]==0x51 && tx_commands[1]==0x03);
}
static void first_and_second(void) {
  rx(0xE3); tick(120);
  assert(s51_e3_recovery_count==1 && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  tick(140); assert(mission_snapshot.left_target_rpm==-80 && vision_search_forward_ms==800);
  rx(0xE3); tick(620);
  assert(s51_e3_recovery_count==1 && mission_snapshot.state_entry_tick==120);
  coords(6); tick(640);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s51_e3_recovery_count==1);
  assert(tx_count==2); // Coordinate-only resume preserves the camera target lock.
  rx(0xE3); tick(660);
  assert(s51_e3_recovery_count==2 && !s51_second_e3_pending);
  assert(mission_snapshot.state==MISSION_STATE_S3_E3_TURN_AWAY);
  assert(s3_search_yaw_target_cdeg==MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg+18000));
  assert(vision_search_forward_rpm==100 && vision_search_forward_ms==1200);
}
static uint32_t align_away(void) {
  tick(680);
  assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm);
  assert(mission_snapshot.safe_zone_yaw_command_cdeg==s3_search_yaw_target_cdeg);
  rx(0xE3); tick(1160);
  assert(s51_e3_recovery_count==2 && mission_snapshot.state_entry_tick==660);
  mission_snapshot.yaw_cdeg=s3_search_yaw_target_cdeg;
  tick(1180); tick(1379);
  assert(mission_snapshot.state==MISSION_STATE_S3_E3_TURN_AWAY);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  tick(1380);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  assert(mission_snapshot.state_entry_tick==1380);
  return 1380;
}
int main(void) {
  verify_red_switch_event_fence();
  verify_debug_period();
  verify_exit180_events();
  verify_post22_search();
  const uint32_t turn_timeout=ROBOT_HEADING_TURN_TIMEOUT_MS;
  assert(turn_timeout==2000U);
  assert(MISSION_STATE_S6_RECHECK_WIDE_SETTLE==94 && MISSION_STATE_S3_E3_TURN_AWAY==95);
  // All eight zone/team configurations and a nonzero IMU boot/start heading.
  for(uint8_t zone=1;zone<=4;++zone) for(uint8_t team=ROBOT_TEAM_RED;team<=ROBOT_TEAM_BLUE;++team) {
    setup(zone,team,12300); first_and_second(); uint32_t start=align_away();
    tick(start+20); assert(mission_snapshot.left_target_rpm==100 && mission_snapshot.right_target_rpm==100);
    rx(0xE3); tick(start+500);
    assert(s51_e3_recovery_count==2 && mission_snapshot.state_entry_tick==start);
    tick(start+1199); assert(mission_snapshot.left_target_rpm==100 && vision_search_forward_active);
    tick(start+1200); assert(!vision_search_forward_active && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
    assert(mission_snapshot.state_entry_tick==start+1200 && !mission_snapshot.left_target_rpm);
    tick(start+1220); assert(mission_snapshot.left_target_rpm==-ROBOT_YAW_LEFT_SIGN*25);
    rx(0xE3); tick(start+1240); assert(s51_e3_recovery_count==2);
    for(unsigned i=1;i<=12;++i) {
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+ROBOT_YAW_LEFT_SIGN*3000);
      tick(start+1240+i*20);
    }
    assert(e3_spin_progress_cdeg==36000 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
    assert(s51_e3_recovery_count==2 && s3_search_sweep_completed && tx_count==2);
    // Only E3#2 changed; E3#3 retains the existing generic recovery.
    rx(0xE3); tick(4000); tick(4020);
    assert(s51_e3_recovery_count==3 && mission_snapshot.left_target_rpm==-80 && vision_search_forward_ms==800);
    MissionTask_EnterState(MISSION_STATE_S3_LOWER_FRAME,4500);
    assert(MissionTask_BeginS3Vision(4520) && !s51_e3_recovery_count);
  }
  // Coordinates interrupt each second-recovery leg; wrong/stale class cannot.
  for(unsigned leg=0;leg<3;++leg) {
    setup(3,ROBOT_TEAM_BLUE,35000); first_and_second();
    uint32_t now=680;
    if(leg) { uint32_t start=align_away(); now=start+20; if(leg==2) { tick(start+1200); now=start+1220; } }
    fake_now=now; coords(4); tick(now);
    assert(mission_snapshot.state!=MISSION_STATE_S3_TRACK_GREEN);
    coords(5); rx(0xE3); tick(now+20); // Active-recovery E3 does not invalidate new coordinates.
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s51_e3_recovery_count==2);
    assert(tx_count==2);
  }
  setup(3,ROBOT_TEAM_BLUE,0); first_and_second();
  fake_now=680; coords(6); tick(1181); //501ms-old packet must not resume.
  assert(mission_snapshot.state==MISSION_STATE_S3_E3_TURN_AWAY);
  // A fresh coordinate accompanying an E3 is not another loss request.
  setup(3,ROBOT_TEAM_BLUE,0); rx(0xE3); coords(6); tick(120);
  assert(!s51_e3_recovery_count && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  // Silent recovery is not counted, including after an interrupted second E3.
  setup(3,ROBOT_TEAM_BLUE,0); first_and_second(); coords(6); tick(680);
  mission_snapshot.vision_target_valid=false;
  MissionTask_StartVisionSearchRecovery(1300,MISSION_STATE_S3_TRACK_GREEN,0,false);
  assert(s51_e3_recovery_count==2 && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  assert(vision_search_forward_rpm==-80 && vision_search_forward_ms==800);
  // Preserve fault-stop coverage in the new phase and both heading/spin deadlines.
  setup(3,ROBOT_TEAM_BLUE,0); first_and_second(); health_faults=MISSION_FAULT_IMU; tick(680);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  setup(3,ROBOT_TEAM_BLUE,0); first_and_second(); tick(660U+turn_timeout-1U);
  assert(mission_snapshot.state==MISSION_STATE_S3_E3_TURN_AWAY);
  int32_t original_yaw=mission_snapshot.yaw_cdeg;
  tick(660U+turn_timeout);assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  assert(mission_snapshot.state_entry_tick==660U+turn_timeout && mission_snapshot.yaw_cdeg==original_yaw && !mission_snapshot.fault_flags);
  tick(680U+turn_timeout);assert(mission_snapshot.left_target_rpm==100);
  setup(3,ROBOT_TEAM_BLUE,0); first_and_second(); uint32_t start=align_away(); tick(start+1200); tick(start+21200);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  // Supplement budget and initial-green/red/04 recovery profiles are unchanged.
  setup(3,ROBOT_TEAM_BLUE,0); supplement_budget_started=supplement_capture_active=true;
  rx(0xE3); tick(120); assert(!s51_e3_recovery_count && vision_search_forward_rpm==-80);
  setup(3,ROBOT_TEAM_BLUE,0); s7_target_switch_enabled=false; s3_target_select_command=0x21;
  rx(0xE3); tick(120); assert(!s51_e3_recovery_count && vision_search_green_e3_mode && vision_search_forward_rpm==80);
  tick(920);assert(s3_green_e3_phase==S3_GREEN_E3_TURN_LEFT && mission_snapshot.state_entry_tick==920);
  tick(920U+turn_timeout-1U);assert(s3_green_e3_phase==S3_GREEN_E3_TURN_LEFT);
  tick(920U+turn_timeout);assert(s3_green_e3_phase==S3_GREEN_E3_TURN_RIGHT && mission_snapshot.state_entry_tick==920U+turn_timeout);
  tick(920U+2U*turn_timeout-1U);assert(s3_green_e3_phase==S3_GREEN_E3_TURN_RIGHT);
  tick(920U+2U*turn_timeout);assert(s3_green_e3_phase==S3_GREEN_E3_TURN_BACK && mission_snapshot.state_entry_tick==920U+2U*turn_timeout);
  tick(920U+3U*turn_timeout);assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !mission_snapshot.fault_flags);
  setup(3,ROBOT_TEAM_BLUE,0); s3_target_select_command=0x11; s7_red_recovery_used=false;
  rx(0xE3); tick(120); assert(!s51_e3_recovery_count && s7_red_recovery_used && vision_search_forward_rpm==80);
  setup(3,ROBOT_TEAM_BLUE,0); MissionTask_StartVisionSearchRecovery(120,MISSION_STATE_S4_TRACK_CENTER,0,false);
  assert(!s51_e3_recovery_count && !vision_search_e3_mode && vision_search_forward_rpm==60);
  tick(620);tick(620U+turn_timeout-1U);assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  tick(620U+turn_timeout);assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW && mission_snapshot.state_entry_tick==620U+turn_timeout);
  tick(620U+2U*turn_timeout);assert(s3_search_ccw_midpoint_done && mission_snapshot.state_entry_tick==620U+2U*turn_timeout);
  tick(620U+3U*turn_timeout-1U);assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
  tick(620U+3U*turn_timeout);assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && !mission_snapshot.fault_flags);
  // Red priority during the new phase retains the existing black-only switch gate.
  setup(3,ROBOT_TEAM_BLUE,0); first_and_second(); s51_active_object_id=6;
  rx(0x11); MissionTask_HandleRedPriorityRequest(680);
  assert(red_switches==1 && s3_target_select_command==0x11 && !s51_e3_recovery_count);
  // Saturate without wrapping back to the second profile.
  setup(3,ROBOT_TEAM_BLUE,0); s51_e3_recovery_count=UINT16_MAX;
  rx(0xE3); tick(120); assert(s51_e3_recovery_count==UINT16_MAX && vision_search_forward_rpm==-80);
  assert(rx_bytes>0 && fake_rx_arms>=rx_bytes);
  puts("PASS: ordinary51 E3 count, second mapped-away turn +100rpm/1200ms +gyro CCW360; all zone/team headings, duplicate/fresh/stale gates, interruptions, reset/saturation, health/timeouts and legacy profiles");
}
'@
$parserPath = (Join-Path $projectRoot 'maixcam/maixcam_task.c').Replace('\','/')
$parserInclude = "`n#define MaixCam_SendCommand Parser_SendCommand`n#include `"$parserPath`"`n#undef MaixCam_SendCommand`n"
$post22Cases = Get-Content (Join-Path $PSScriptRoot 'host/post22_search_cases.c') -Raw
$exit180Cases = Get-Content (Join-Path $PSScriptRoot 'host/exit180_event_cases.c') -Raw
$debugCases = Get-Content (Join-Path $PSScriptRoot 'host/debug_period_cases.c') -Raw
$redFenceCases = Get-Content (Join-Path $PSScriptRoot 'host/red_switch_event_cases.c') -Raw
$testSource = Join-Path $outputDir 'verify_s51_e3_stages.c'
[IO.File]::WriteAllText($testSource,$preamble+"`n"+$globals+"`n"+$prototypes+$parserInclude+$stubs+"`n"+($functions -join "`n")+"`n"+$post22Wait+"`n"+$cases+"`n"+$post22Cases+"`n"+$exit180Cases+"`n"+$debugCases+"`n"+$redFenceCases,[Text.UTF8Encoding]::new($false))
$testExe = Join-Path $outputDir 'verify_s51_e3_stages.exe'
& $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/Task" "-I$projectRoot/maixcam" $testSource -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
