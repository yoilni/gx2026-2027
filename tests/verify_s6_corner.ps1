param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/corner_host'
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
  'MissionTask_HeadingTurnExpired', 'MissionTask_RunS1', 'MissionTask_RunS4ForcedLeftTurn',
  'MissionTask_GetSafeZoneYawOffset', 'MissionTask_GetSelectedObjectId', 'MissionTask_GetCarriedObjectId',
  'MissionTask_S6IsSupplyTarget', 'MissionTask_S6IsCasualtyTarget', 'MissionTask_GetS6SideRule',
  'MissionTask_GetS6CornerHeading', 'MissionTask_S6NeedsSideReposition',
  'MissionTask_S6SideForwardDuration', 'MissionTask_StartS6SideReposition',
  'MissionTask_StartS6CornerEvacuation', 'MissionTask_TryS6AngleManeuver',
  'MissionTask_ResetS6PreDecisionTracking', 'MissionTask_S6PreDecisionTrackingReady',
  'MissionTask_UpdateS6PreDecisionTracking', 'MissionTask_ResetS6XConfirmation',
  'MissionTask_S6SideXConfirmed', 'MissionTask_S6XConfirmed', 'MissionTask_UpdateS6XConfirmation',
  'MissionTask_ResetVisionControllerState', 'MissionTask_ResetVisionPid', 'MissionTask_ResetS6ControllerState',
  'MissionTask_StartS6VisionApproach', 'MissionTask_EnterS6TrackingFrom16', 'MissionTask_DecideS6Side',
  'MissionTask_RunS6TrackSafeZone', 'MissionTask_RunS6CornerEvacuation', 'MissionTask_RunS6SideReposition',
  'MissionTask_CalculateS6YawPid', 'MissionTask_CalculateS6YawTurn', 'MissionTask_RunS6TurnToHeading',
  'MissionTask_CommandS6Straight', 'MissionTask_CalculateS2YawCorrection', 'MissionTask_StartS6ObstacleAvoidance',
  'MissionTask_RunS6ObstacleAvoidance', 'MissionTask_SetTrackingTargets', 'MissionTask_VisionAgeScale',
  'MissionTask_SetVisionTrackingTargets', 'MissionTask_GuardVisionForward', 'MissionTask_CalculateVisionForward', 'MissionTask_CalculateVisionTurn',
  'MissionTask_IsObjectTrackingPhase', 'MissionTask_RedCoordinateIsFresh', 'MissionTask_UpdateVisionInput',
  'MissionTask_StartS6FinalFrom26', 'MissionTask_HandleS6AlignRequest', 'MissionTask_CheckRunningHealth')
$names += @('MissionTask_StartS6LoadRecheck', 'MissionTask_RunS6LoadRecheck', 'MissionTask_StartS6FromLoad',
  'MissionTask_ReportS6SearchSoftTimeout', 'MissionTask_RunS6SearchSafeZone')
$names += @('MissionTask_CanPauseForLoadCheck', 'MissionTask_HandleLoadCheckRequest')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$prototypes = ($functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).TrimEnd() + ';' }) -join "`n"
$start = $source.IndexOf('static osThreadId_t mission_task_handle;')
$globals = $source.Substring($start, $source.IndexOf('static const osThreadAttr_t mission_task_attributes') - $start)
$ruleTypes = [regex]::Match($source, '(?m)^typedef struct\r?\n\{\r?\n  uint8_t start_zone;[\s\S]*?^static const MissionSideRule s6_corner_rules\[\].*;').Value
if (-not $ruleTypes) { throw 'Missing real side/corner tables' }
$preamble = @'
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
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
static uint32_t health_faults;
static unsigned tx_calls, rx_bytes, uart_inits, recovery_calls;
static uint8_t last_tx_command;
static bool inject26_before_corner_run;
static void rx(uint8_t command) {
  const uint8_t frame[]={0xF1,0xF2,command,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(frame);++i) {
    ++rx_bytes;maixcam_rx_byte=frame[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
  }
}
static void coords(void) {
  const uint8_t frame[]={0xF1,0xF2,2,0,80,1,0,70,2,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(frame);++i) {
    ++rx_bytes;maixcam_rx_byte=frame[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
  }
}
static void aligned_coords(void) {
  const uint8_t frame[]={0xF1,0xF2,2,0,0,1,1,0xC0,2,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(frame);++i) {
    ++rx_bytes;maixcam_rx_byte=frame[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
  }
}
static void cargo_report(uint8_t id) {
  const uint8_t frame[]={0xF1,0xF2,id,0,0,1,0,0,1,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(frame);++i) {
    ++rx_bytes;maixcam_rx_byte=frame[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
  }
}
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t command) { last_tx_command=command;++tx_calls;return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) { return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetCameraNearView(void) { return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetCameraWideView(void) { return HAL_OK; }
static bool DebugUart_Log(const char *line) { (void)line;return true; }
static bool DebugUart_Logf(const char *format,...) {
  char line[512];va_list args;va_start(args,format);vsnprintf(line,sizeof(line),format,args);va_end(args);
  if(strstr(line,"[CORNER]")==line) assert(strlen(line)<128);
  return true;
}
static void MissionTask_StopWheels(void) {
  mission_snapshot.left_target_rpm=mission_snapshot.right_target_rpm=0;
}
static void MissionTask_SetWheelTargets(int16_t left,int16_t right) {
  mission_snapshot.left_target_rpm=left;mission_snapshot.right_target_rpm=right;
}
static void MissionTask_EnterState(MissionState state,uint32_t now) {
  if(mission_snapshot.state==state) return;
  mission_snapshot.state=state;mission_snapshot.state_entry_tick=now;
  MaixCam_SetLoadCheckInterruptEnabled(MissionTask_CanPauseForLoadCheck());
  MaixCam_SetCornerEvacuationActive(state>=MISSION_STATE_S6_CORNER_TURN_AWAY &&
      state<=MISSION_STATE_S6_CORNER_TURN_SAFE);
  if(state!=MISSION_STATE_S6_TRACK_SAFE_ZONE) {
    s6_pre_reposition_track_active=false;s6_pre_reposition_track_tick=now;
  }
  if(state==MISSION_STATE_FAULT || state==MISSION_STATE_STOPPED) MissionTask_StopWheels();
}
static void MissionTask_DebugS6(uint32_t now) { (void)now; }
static uint32_t MissionTask_GetRequiredFaults(void) { return health_faults; }
static void MissionTask_StartS6Recovery(uint32_t now,bool tracking) {
  (void)tracking;++recovery_calls;MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_START,now);
}
static void MissionTask_DebugS1(uint32_t now) { (void)now; }
static void MissionTask_SetS1Phase(MissionS1Phase phase,uint32_t now) {
  (void)now;mission_snapshot.s1_phase=phase;
}
static void MissionTask_StartS2CrossBump(uint32_t now) {
  MissionTask_SetS1Phase(MISSION_S1_DONE,now);MissionTask_EnterState(MISSION_STATE_S2_CROSS_BUMP,now);
}
'@
$cases = @'
static void setup(uint8_t zone,uint8_t team,uint8_t cargo,int32_t zero,int32_t field) {
  assert(MaixCam_Init(&uart)==HAL_OK);++uart_inits;
  memset(&mission_snapshot,0,sizeof(mission_snapshot));
  mission_snapshot.state=MISSION_STATE_S6_SEARCH_SAFE_ZONE;
  mission_snapshot.start_zone=zone;mission_snapshot.team=team;
  mission_snapshot.yaw_start_cdeg=zero;
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(zero+field);
  mission_snapshot.safe_zone_yaw_target_cdeg=MissionTask_WrapYaw(zero+MissionTask_GetSafeZoneYawOffset(zone,team));
  s6_carried_object_id=cargo;s3_target_select_command=cargo==4?0x51:0x11;
  s7_target_switch_enabled=true;s7_red_switch_sync_pending=false;s7_red_coordinate_fence_active=false;
  s6_obstacle_side_required=s6_obstacle_side_decision_pending=s6_obstacle_side_decision_done=false;
  s6_align26_pending=false;s6_supply_y_started=false;s6_track_elapsed_ms=0;s6_reposition_count=0;
  s6_corner_completed=false;
  s5_recheck_resume_safe_tracking=false;
  fake_now=100;health_faults=0;tx_calls=0;recovery_calls=0;inject26_before_corner_run=false;
  MissionTask_EnterS6TrackingFrom16(fake_now);
}
static void tick(uint32_t now,bool new_frame) {
  fake_now=now;if(new_frame) coords();
  MissionTask_UpdateVisionInput(now);MissionTask_CheckRunningHealth(now);
  (void)MissionTask_HandleLoadCheckRequest(now);
  MissionTask_HandleS6AlignRequest(now);
  if(inject26_before_corner_run) { inject26_before_corner_run=false;rx(0x26); }
  MissionTask_RunS6LoadRecheck(now);MissionTask_RunS6SearchSafeZone(now);
  MissionTask_RunS6SideReposition(now);MissionTask_RunS6CornerEvacuation(now);
  MissionTask_RunS6TrackSafeZone(now);
}
static bool side_expected(int32_t base) {
  return (base>=13500 && base<=15700) || (base>19000 && base<=22400);
}
static bool casualty_side_expected(int32_t base) {
  return (base>=9000 && base<=16500) || (base>19000 && base<=22400);
}
static bool corner_expected(int32_t base) {
  return (base>=9000 && base<13500) || (base>22400 && base<=27000);
}
static void verify_ranges(void) {
  const uint8_t teams[]={ROBOT_TEAM_RED,ROBOT_TEAM_BLUE};
  const int32_t zeroes[]={0,35000,17900};
  const int32_t edges[]={8999,9000,9001,13499,13500,13501,15699,15700,15701,
    16499,16500,16501,18999,19000,19001,22399,22400,22401,
    22999,23000,23001,26999,27000,27001};
  for(unsigned z=1;z<=4;++z) for(unsigned t=0;t<2;++t) for(unsigned n=0;n<3;++n) {
    setup(z,teams[t],5,zeroes[n],0);
    int32_t safe=MissionTask_GetSafeZoneYawOffset(z,teams[t]);
    for(unsigned id=3;id<=6;++id) {
      s6_carried_object_id=id;
      for(int deg=0;deg<360;++deg) {
        int32_t field=deg*100,base=MissionTask_WrapYaw(field-safe+18000),heading;
        int32_t yaw=MissionTask_WrapYaw(zeroes[n]+field);
        bool side=MissionTask_S6NeedsSideReposition(yaw);
        bool corner=MissionTask_GetS6CornerHeading(yaw,&heading);
        assert(side==(id==4 ? casualty_side_expected(base) : side_expected(base)));
        assert(corner==corner_expected(base));
        assert(!(id!=4 && side && corner));
        if(side) {
          bool calibrated;
          const MissionSideRule *rule=MissionTask_GetS6SideRule(yaw,&calibrated);
          assert(calibrated && rule);
          assert(rule->side_heading_cdeg==MissionTask_WrapYaw(safe-18000+(base<18000?9000:27000)));
        }
        if(corner) assert(heading==MissionTask_WrapYaw(zeroes[n]+safe-18000+(base<18000?9000:27000)));
      }
      for(unsigned e=0;e<sizeof(edges)/sizeof(edges[0]);++e) {
        int32_t base=edges[e],field=MissionTask_WrapYaw(base+safe-18000),heading;
        int32_t yaw=MissionTask_WrapYaw(zeroes[n]+field);
        bool side=MissionTask_S6NeedsSideReposition(yaw);
        bool corner=MissionTask_GetS6CornerHeading(yaw,&heading);
        assert(side==(id==4 ? casualty_side_expected(base) : side_expected(base)));
        assert(corner==corner_expected(base));
        assert(!(id!=4 && side && corner));
      }
    }
  }
}
static void track_before_angle_decision(void) {
  const uint32_t decision_tick=120+ROBOT_S6_PRE_REPOSITION_TRACK_MS;
  tick(120,true);
  for(unsigned now=320;now<decision_tick;now+=200) tick(now,true);
  tick(decision_tick-1,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  assert(s6_pre_reposition_track_elapsed_ms==ROBOT_S6_PRE_REPOSITION_TRACK_MS-1);
  tick(decision_tick,true); // X=80px must not block the time/yaw-only decision
}
static void verify_corner_route(uint8_t zone,uint8_t team,uint8_t cargo,int32_t field) {
  setup(zone,team,cargo,35000,field);
  track_before_angle_decision();
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY && tx_calls==0);
  int32_t safe=mission_snapshot.safe_zone_yaw_target_cdeg;
  int32_t away=MissionTask_WrapYaw(safe+18000),side;
  assert(MissionTask_GetS6CornerHeading(MissionTask_WrapYaw(35000+field),&side));
  assert(s6_corner_away_yaw_cdeg==away && s6_corner_side_yaw_cdeg==side);
  assert(mission_snapshot.left_target_rpm==0);
  fake_now=1140;rx(0x26);
  mission_snapshot.yaw_cdeg=away;tick(1140,false);tick(1340,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_AWAY);
  fake_now=1360;rx(0x26);rx(0x26);
  tick(1360,true);assert(mission_snapshot.left_target_rpm==100 && mission_snapshot.right_target_rpm==100);
  tick(3339,true);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_AWAY);
  assert(mission_snapshot.safe_zone_yaw_command_cdeg==away);
  tick(3340,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_SIDE);
  assert(mission_snapshot.left_target_rpm==0);
  fake_now=3360;rx(0x26);
  mission_snapshot.yaw_cdeg=side;tick(3360,false);tick(3560,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_SIDE);
  fake_now=3580;rx(0x26);
  tick(3580,true);assert(mission_snapshot.left_target_rpm==100 && mission_snapshot.right_target_rpm==100);
  tick(4759,true);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_SIDE);
  assert(mission_snapshot.safe_zone_yaw_command_cdeg==side);
  tick(4760,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_SAFE);
  fake_now=4780;rx(0x26);
  mission_snapshot.yaw_cdeg=safe;tick(4780,true);
  inject26_before_corner_run=true;tick(4980,false); //ISR after task handler but before exit must still be ignored
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_ACK);
  assert(mission_snapshot.left_target_rpm==0 && !mission_snapshot.vision_target_valid);
  assert(s6_carried_object_id==cargo && tx_calls==1 && last_tx_command==0x05 && recovery_calls==0);
  tick(5000,false);assert(!mission_snapshot.vision_target_valid); // cached maneuver coordinates were cleared
  fake_now=5020;rx(0x05);cargo_report(cargo);rx(0x06);rx(0x26);
  tick(5020,true);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  tick(5519,true);assert(!mission_snapshot.left_target_rpm && !mission_snapshot.vision_target_valid);
  tick(5520,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE);
  assert(s6_carried_object_id==cargo && tx_calls==1 && s5_load_class_confirmed);
  tick(6300,true);assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  tick(6319,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE);
  tick(6320,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed);
  assert(s6_pre_reposition_track_elapsed_ms==0 && s6_pre_reposition_decision_pending);
  tick(6340,true);assert(mission_snapshot.vision_target_valid);
  assert(mission_snapshot.left_target_rpm!=0); // new frames restart XY PID
  //Even if tracking revisits the same angle, do not repeat this load's corner.
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(35000+field);
  for(unsigned now=6540;now<=7540;now+=200) tick(now,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed && tx_calls==1);
  //The same guard survives recovery re-entry and saved post36 decisions.
  MissionTask_StartS6VisionApproach(7560);
  s6_obstacle_side_decision_pending=true;s6_obstacle_decision_yaw_cdeg=mission_snapshot.yaw_cdeg;
  for(unsigned now=7560;now<=8560;now+=200) tick(now,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed && tx_calls==1);
  fake_now=8580;rx(0x26);tick(8580,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN); // only a new post-return26 starts delivery
}
static void verify_yaw_deadbands(void) {
  assert(ROBOT_S1_TURN_TOLERANCE_CDEG==400 && ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG==400);
  assert(ROBOT_S2_YAW_HOLD_TOLERANCE_CDEG==300 && ROBOT_S6_YAW_TOLERANCE_CDEG==300);
  assert(ROBOT_S6_TURN_TOLERANCE_CDEG==400 && ROBOT_S5_ARRANGE_TURN_RELEASE_CDEG==600);
  assert(ROBOT_S6_SEARCH_HOLD_TOLERANCE_CDEG==600);
  for(int sign=-1;sign<=1;sign+=2) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    int32_t goal=mission_snapshot.safe_zone_yaw_target_cdeg;
    mission_snapshot.yaw_target_cdeg=goal;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*300);
    assert(MissionTask_CalculateS2YawCorrection()==0);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*301);
    assert(MissionTask_CalculateS2YawCorrection()!=0);
    MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_FORWARD,120);
    MissionTask_ResetS6ControllerState();
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*300);
    MissionTask_CommandS6Straight(140,goal,70,80);
    assert(mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*301);
    MissionTask_CommandS6Straight(160,goal,70,80);
    assert(mission_snapshot.left_target_rpm!=mission_snapshot.right_target_rpm);
    MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_LEFT,180);
    MissionTask_ResetS6ControllerState();s6_phase_stable_since=0;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*400);
    assert(MissionTask_RunS6TurnToHeading(200,goal));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*401);
    assert(!MissionTask_RunS6TurnToHeading(220,goal));
    assert(mission_snapshot.left_target_rpm && mission_snapshot.right_target_rpm);
    MissionTask_EnterState(MISSION_STATE_S5_ALIGN_FOR_ARRANGE,240);
    MissionTask_ResetS6ControllerState();s6_phase_stable_since=0;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*400);
    assert(!MissionTask_RunS6TurnToHeading(260,goal));
    assert(s6_phase_stable_since==260 && !mission_snapshot.left_target_rpm);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*600);
    assert(!MissionTask_RunS6TurnToHeading(280,goal));
    assert(s6_phase_stable_since==260 && !mission_snapshot.left_target_rpm);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*601);
    assert(!MissionTask_RunS6TurnToHeading(300,goal));
    assert(!s6_phase_stable_since && mission_snapshot.left_target_rpm);
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE,120);
    MissionTask_ResetS6ControllerState();s6_missing_since=0;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*401);
    MissionTask_RunS6SearchSafeZone(140);
    assert(!recovery_calls && mission_snapshot.left_target_rpm);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goal+sign*400);
    MissionTask_RunS6SearchSafeZone(160);
    assert(recovery_calls==1 && !mission_snapshot.left_target_rpm);
  }
  puts("PASS:yaw deadbands +/-3deg straight,+/-4deg turn,inclusive boundaries and6deg release hysteresis");
}
static void verify_heading_turn_timeout(void) {
  assert(ROBOT_HEADING_TURN_TIMEOUT_MS==2000);
  const MissionState phases[]={MISSION_STATE_S5_ALIGN_FOR_ARRANGE,
    MISSION_STATE_S4_E4_TURN_AWAY,MISSION_STATE_S4_E4_TURN_LEFT,MISSION_STATE_S4_E4_TURN_RIGHT,
    MISSION_STATE_S4_POST22_TURN_LEFT,MISSION_STATE_S4_POST22_TURN_RIGHT,
    MISSION_STATE_S6_RECOVERY_START,MISSION_STATE_S6_RECOVERY_TURN_LEFT,
    MISSION_STATE_S6_RECOVERY_TURN_BACK,MISSION_STATE_S6_RECOVERY_RETURN_SAFE,
    MISSION_STATE_S6_REPOSITION_TURN_SIDE,MISSION_STATE_S6_REPOSITION_TURN_SAFE,
    MISSION_STATE_S6_CORNER_TURN_AWAY,MISSION_STATE_S6_CORNER_TURN_SIDE,MISSION_STATE_S6_CORNER_TURN_SAFE,
    MISSION_STATE_S6_OBSTACLE_TURN_OUT,MISSION_STATE_S6_OBSTACLE_TURN_BACK,MISSION_STATE_S6_OBSTACLE_FACE_SAFE,
    MISSION_STATE_S6_FINAL_ALIGN,MISSION_STATE_S6_EXIT_TURN_180,
    MISSION_STATE_S7_SILENT_TURN_LEFT,MISSION_STATE_S7_SILENT_TURN_BACK,
    MISSION_STATE_SUPPLEMENT_TURN_LEFT,MISSION_STATE_SUPPLEMENT_TURN_RIGHT};
  for(unsigned i=0;i<sizeof(phases)/sizeof(phases[0]);++i) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
    MissionTask_EnterState(phases[i],120);MissionTask_ResetS6ControllerState();s6_phase_stable_since=0;
    int32_t original=mission_snapshot.yaw_cdeg,goal=mission_snapshot.safe_zone_yaw_target_cdeg;
    assert(!MissionTask_RunS6TurnToHeading(2119,goal));
    assert(mission_snapshot.left_target_rpm && mission_snapshot.right_target_rpm);
    assert(MissionTask_RunS6TurnToHeading(2120,goal));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!mission_snapshot.vision_turn_rpm && !mission_snapshot.vision_forward_rpm);
    assert(mission_snapshot.yaw_cdeg==original && mission_snapshot.safe_zone_yaw_error_cdeg==9000);
    assert(!mission_snapshot.fault_flags && mission_snapshot.state==phases[i]);
  }
  setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
  MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE,120);
  MissionTask_RunS6SearchSafeZone(2119);assert(!recovery_calls);
  MissionTask_RunS6SearchSafeZone(2120);
  assert(recovery_calls==1 && mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
  assert(!mission_snapshot.vision_target_valid && !mission_snapshot.fault_flags);
  setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_RECOVERY_TURN_270,120);
  int32_t goal=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+9000);
  assert(!MissionTask_RunS4ForcedLeftTurn(2119,goal));
  assert(MissionTask_RunS4ForcedLeftTurn(2120,goal));
  assert(!mission_snapshot.fault_flags && !mission_snapshot.left_target_rpm);
  setup(3,ROBOT_TEAM_BLUE,5,35000,0);
  MissionTask_EnterState(MISSION_STATE_S1_DEPART,120);
  mission_snapshot.s1_phase=MISSION_S1_TURN;
  mission_snapshot.yaw_target_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+4500);
  MissionTask_RunS1(2119);assert(mission_snapshot.state==MISSION_STATE_S1_DEPART);
  MissionTask_RunS1(2120);assert(mission_snapshot.state==MISSION_STATE_S2_CROSS_BUMP);
  assert(!mission_snapshot.fault_flags && !mission_snapshot.left_target_rpm);
  // Each new state owns a fresh budget, including timer wrap.
  setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
  uint32_t entry=UINT32_MAX-400U;
  MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_LEFT,entry);
  assert(!MissionTask_RunS6TurnToHeading(entry+1999U,mission_snapshot.safe_zone_yaw_target_cdeg));
  assert(MissionTask_RunS6TurnToHeading(entry+2000U,mission_snapshot.safe_zone_yaw_target_cdeg));
  MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_BACK,entry+2000U);
  assert(!MissionTask_RunS6TurnToHeading(entry+3999U,mission_snapshot.safe_zone_yaw_target_cdeg));
  assert(MissionTask_RunS6TurnToHeading(entry+4000U,mission_snapshot.safe_zone_yaw_target_cdeg));
  puts("PASS:all fixed-heading turn deadlines1999/2000ms,initial06 recovery without fake16,EE forced-left,S1 next action,fresh per-leg2s budgets/timer wrap,no fake yaw/no timeout FAULT");
}
int main(void) {
  verify_yaw_deadbands();
  verify_heading_turn_timeout();
  assert(ROBOT_S6_PRE_REPOSITION_TRACK_MS==500);
  assert(MISSION_STATE_SUPPLEMENT_CENTER_OBJECT==85 && MISSION_STATE_S6_CORNER_TURN_AWAY==86);
  assert(ROBOT_S6_CORNER_FORWARD_RPM==100 && ROBOT_S6_CORNER_AWAY_FORWARD_MS==2000 && ROBOT_S6_CORNER_SIDE_FORWARD_MS==1200);
  assert(ROBOT_S6_REPOSITION_FORWARD_RPM==120 && ROBOT_S6_CORNER_WHEEL_MAX_RPM==110);
  // Initial06 safe-heading turn is20rpm faster; near-heading speed stays low.
  setup(3,ROBOT_TEAM_BLUE,5,35000,8000);
  MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE,120);
  MissionTask_ResetS6ControllerState();
  MissionTask_RunS6SearchSafeZone(140);
  assert(mission_snapshot.left_target_rpm==-85 && mission_snapshot.right_target_rpm==85);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(35000+17500);
  MissionTask_ResetS6ControllerState();
  MissionTask_RunS6SearchSafeZone(160);
  assert(mission_snapshot.left_target_rpm==-10 && mission_snapshot.right_target_rpm==10);
  assert(ROBOT_S6_OBSTACLE_TURN_MAX_RPM==85 && ROBOT_S6_REPOSITION_TURN_MAX_RPM==55);
  assert(ROBOT_S6_RECOVERY_SAFE_MAX_RPM==50 && ROBOT_S6_YAW_MAX_TURN_RPM==30);
  //36 has no200ms turn confirmation; supplies left/casualties right, unchanged travel.
  assert(ROBOT_S6_OBSTACLE_TURN_STABLE_MS==0 && ROBOT_S6_OBSTACLE_DRIVE_RPM==140);
  assert(ROBOT_S6_OBSTACLE_FORWARD_MS==357);
  assert(140*357>=120*417-140 && 140*357<=120*417+140);
  for(uint8_t cargo=4;cargo<=6;++cargo) {
    setup(3,ROBOT_TEAM_BLUE,cargo,35000,18000);
    MissionTask_StartS6ObstacleAvoidance(120);
    int32_t expected_out=MissionTask_WrapYaw(35000+18000+(cargo==4?-4500:4500));
    assert(s6_obstacle_out_yaw_cdeg==expected_out && s6_obstacle_turn_right==(cargo==4));
    MissionTask_RunS6ObstacleAvoidance(140);
    assert(mission_snapshot.left_target_rpm==(cargo==4?83:-83));
    assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    mission_snapshot.yaw_cdeg=expected_out;
    MissionTask_RunS6ObstacleAvoidance(160);
    assert(mission_snapshot.state==MISSION_STATE_S6_OBSTACLE_FORWARD);
    MissionTask_RunS6ObstacleAvoidance(180);
    assert(mission_snapshot.left_target_rpm==140 && mission_snapshot.right_target_rpm==140);
    MissionTask_RunS6ObstacleAvoidance(516);
    assert(mission_snapshot.state==MISSION_STATE_S6_OBSTACLE_FORWARD);
    MissionTask_RunS6ObstacleAvoidance(517);
    assert(mission_snapshot.state==MISSION_STATE_S6_OBSTACLE_TURN_BACK);
    mission_snapshot.yaw_cdeg=s6_obstacle_back_yaw_cdeg;
    MissionTask_RunS6ObstacleAvoidance(537);
    assert(mission_snapshot.state==MISSION_STATE_S6_OBSTACLE_FACE_SAFE);
    mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
    fake_now=557;MissionTask_RunS6ObstacleAvoidance(557);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && last_tx_command==0x36 && tx_calls==1);
  }
  // Linear nominal travel from the decision angle; symmetry and nonzero start yaw.
  for(uint8_t zone=1;zone<=4;++zone) for(uint8_t team=ROBOT_TEAM_RED;team<=ROBOT_TEAM_BLUE;++team)
    for(uint8_t cargo=3;cargo<=6;++cargo) {
      int32_t safe=MissionTask_GetSafeZoneYawOffset(zone,team);
      const int32_t far=13500, near=cargo==4?16500:15700;
      const int32_t angles[]={far,(far+near)/2,near};
      const uint32_t times[]={1225,817,408};
      for(unsigned i=0;i<3;++i) {
        setup(zone,team,cargo,35000,MissionTask_WrapYaw(angles[i]+safe-18000));
        assert(MissionTask_TryS6AngleManeuver(120,mission_snapshot.yaw_cdeg));
        assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
        if(s6_reposition_forward_ms!=times[i]) {
          bool found; const MissionSideRule *r=MissionTask_GetS6SideRule(mission_snapshot.yaw_cdeg,&found);
          fprintf(stderr,"SIDE duration mismatch zone=%u team=%u cargo=%u angle=%ld got=%lu expected=%lu yaw=%ld safe=%ld rule=%ld..%ld\n",
            zone,team,cargo,(long)angles[i],(unsigned long)s6_reposition_forward_ms,(unsigned long)times[i],
            (long)mission_snapshot.yaw_cdeg,(long)mission_snapshot.safe_zone_yaw_target_cdeg,
            (long)(r?r->sector_start_cdeg:-1),(long)(r?r->sector_end_cdeg:-1));
        }
        assert(s6_reposition_forward_ms==times[i]);
      }
      {
        const int32_t second_angles[]={19001,20700,22400};
        const uint32_t second_times[]={409,817,1225};
        for(unsigned i=0;i<3;++i) {
          setup(zone,team,cargo,35000,MissionTask_WrapYaw(second_angles[i]+safe-18000));
          assert(MissionTask_TryS6AngleManeuver(120,mission_snapshot.yaw_cdeg));
          assert(s6_reposition_forward_ms==second_times[i]);
        }
      }
  }
  // Forward duration is frozen before turning and unaffected by new gyro samples.
  setup(3,ROBOT_TEAM_BLUE,5,35000,13500);
  assert(MissionTask_TryS6AngleManeuver(120,mission_snapshot.yaw_cdeg));
  assert(s6_reposition_forward_ms==1225);
  mission_snapshot.yaw_cdeg=s6_reposition_side_yaw_cdeg;
  MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_FORWARD,700);
  fake_now=720;MissionTask_RunS6SideReposition(fake_now);
  assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
  fake_now=1924;MissionTask_RunS6SideReposition(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD && s6_reposition_forward_ms==1225);
  fake_now=1925;MissionTask_RunS6SideReposition(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE);
  // Keep the30rpm cap until three new, stable X-aligned coordinate frames.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
  fake_now=120;aligned_coords();tick(120,false);
  assert(mission_snapshot.vision_forward_rpm==30);
  fake_now=320;aligned_coords();tick(320,false);
  assert(mission_snapshot.vision_forward_rpm==30);
  fake_now=520;aligned_coords();tick(520,false);
  assert(mission_snapshot.vision_forward_rpm==150 && mission_snapshot.left_target_rpm==150 && mission_snapshot.right_target_rpm==150);
  verify_ranges();
  const uint8_t teams[]={ROBOT_TEAM_RED,ROBOT_TEAM_BLUE};
  const uint8_t cargos[]={3,4,5,6};
  for(unsigned z=1;z<=4;++z) for(unsigned t=0;t<2;++t) for(unsigned c=0;c<4;++c) {
    int32_t safe=MissionTask_GetSafeZoneYawOffset(z,teams[t]);
    verify_corner_route(z,teams[t],cargos[c],MissionTask_WrapYaw(10000+safe-18000));
    // Newly added sector begins strictly after224deg (44deg for safe=0).
    verify_corner_route(z,teams[t],cargos[c],MissionTask_WrapYaw(22401+safe-18000));
    verify_corner_route(z,teams[t],cargos[c],MissionTask_WrapYaw(25000+safe-18000));
  }
  // Shared135/224 boundaries select ordinary side motion, not evacuation.
  const int32_t side_angles[]={13500,15700,19001,22400};
  for(unsigned i=0;i<4;++i) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,side_angles[i]);track_before_angle_decision();
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
  }
  // Casualty evacuation wins in the old overlapping side sector; its
  // remaining side range, corner endpoints and mapped headings are preserved.
  const int32_t casualty_edges[]={8999,9000,13499,13500,16500,16501,
    18999,19000,19001,19492,22399,22400,22401,27000,27001};
  for(unsigned z=1;z<=4;++z) for(unsigned t=0;t<2;++t) {
    int32_t safe=MissionTask_GetSafeZoneYawOffset(z,teams[t]);
    for(unsigned i=0;i<sizeof(casualty_edges)/sizeof(casualty_edges[0]);++i) {
      int32_t base=casualty_edges[i];
      setup(z,teams[t],4,35000,MissionTask_WrapYaw(base+safe-18000));
      track_before_angle_decision();
      MissionState expected=corner_expected(base)?MISSION_STATE_S6_CORNER_TURN_AWAY:
        (casualty_side_expected(base)?MISSION_STATE_S6_REPOSITION_TURN_SIDE:
         MISSION_STATE_S6_TRACK_SAFE_ZONE);
      assert(mission_snapshot.state==expected);
      if(expected==MISSION_STATE_S6_REPOSITION_TURN_SIDE)
        assert(s6_reposition_side_yaw_cdeg==MissionTask_WrapYaw(35000+safe-18000+(base<18000?9000:27000)));
    }
    // Saved post36 angles must use the same casualty corner-first selector.
    setup(z,teams[t],4,35000,safe);
    s6_obstacle_side_decision_pending=true;
    s6_obstacle_decision_yaw_cdeg=MissionTask_WrapYaw(35000+10000+safe-18000);
    track_before_angle_decision();assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
    // The added casualty sector also applies to the saved post36 angle.
    setup(z,teams[t],4,35000,safe);
    s6_obstacle_side_decision_pending=true;
    s6_obstacle_decision_yaw_cdeg=MissionTask_WrapYaw(35000+19492+safe-18000);
    track_before_angle_decision();
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
    assert(s6_reposition_side_yaw_cdeg==MissionTask_WrapYaw(35000+safe+9000));
    // A pending26 still bypasses the new sector and enters final alignment.
    setup(z,teams[t],4,35000,MissionTask_WrapYaw(19492+safe-18000));
    fake_now=120;rx(0x26);tick(120,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && s6_reposition_count==0);
  }
  // Post36 uses the saved field100 angle even after the robot faces safe180.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
  s6_obstacle_side_decision_pending=true;s6_obstacle_decision_yaw_cdeg=MissionTask_WrapYaw(35000+10000);
  track_before_angle_decision();assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
  assert(s6_corner_side_yaw_cdeg==MissionTask_WrapYaw(35000+9000));
  // Each side-return restarts the full500ms interval, not the old elapsed time.
  setup(3,ROBOT_TEAM_BLUE,5,35000,13500);track_before_angle_decision();
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
  MissionTask_StartS6VisionApproach(700);
  assert(!s6_pre_reposition_track_elapsed_ms && s6_pre_reposition_decision_pending);
  tick(720,true);tick(1219,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_pre_reposition_track_elapsed_ms==499);
  tick(1220,true);assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
  // A brief missing frame pauses, but does not clear the accumulated500ms gate.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);tick(120,true);tick(520,true);
  assert(s6_pre_reposition_track_elapsed_ms==400);
  fake_now=540;MaixCam_ClearObject();tick(540,false);tick(560,true);
  assert(s6_pre_reposition_track_elapsed_ms==400);
  tick(659,true);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_pre_reposition_track_elapsed_ms==499);
  tick(660,true);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
  // Every corner step ignores26 without latching or postponing delivery.
  for(unsigned state=MISSION_STATE_S6_CORNER_TURN_AWAY;state<=MISSION_STATE_S6_CORNER_TURN_SAFE;++state) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,10000);
    MissionTask_EnterState((MissionState)state,100);s6_approach_start_tick=100;
    s6_corner_away_yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+9000);
    s6_corner_side_yaw_cdeg=s6_corner_away_yaw_cdeg;
    mission_snapshot.left_target_rpm=mission_snapshot.right_target_rpm=100;
    fake_now=120;rx(0x26);tick(120,true);
    assert(mission_snapshot.state==(MissionState)state && !s6_align26_pending && tx_calls==0);
    fake_now=140;rx(0x26);tick(140,true);
    assert(mission_snapshot.state==(MissionState)state && !s6_align26_pending);
    MissionTask_StartS6VisionApproach(160);tick(180,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
    fake_now=200;rx(0x26);tick(200,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
    setup(3,ROBOT_TEAM_BLUE,5,35000,10000);
    mission_snapshot.state=(MissionState)state;mission_snapshot.state_entry_tick=100;
    s6_approach_start_tick=100;health_faults=MISSION_FAULT_IMU;tick(120,false);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && mission_snapshot.left_target_rpm==0);
    setup(3,ROBOT_TEAM_BLUE,5,35000,10000);
    mission_snapshot.state=(MissionState)state;mission_snapshot.state_entry_tick=100;
    s6_approach_start_tick=100;tick(15100,false);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags & MISSION_FAULT_S6_TIMEOUT));
  }
  //A26 already pending at corner entry is consumed without executing it.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);rx(0x26);
  MissionTask_EnterState(MISSION_STATE_S6_CORNER_TURN_AWAY,100);
  MissionTask_HandleS6AlignRequest(120);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY && !s6_align26_pending);
  assert(!MaixCam_TakeSafeZoneAlignRequest());
  //Old26 half-frame starts during evacuation and completes after return: discard, don't replay.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
  MissionTask_EnterState(MISSION_STATE_S6_CORNER_TURN_SAFE,100);s6_approach_start_tick=100;
  tick(120,false);
  const uint8_t prefix[]={0xF1,0xF2,0x26},tail[]={0x1F,0x2F};
  for(unsigned i=0;i<sizeof(prefix);++i) {
    ++rx_bytes;maixcam_rx_byte=prefix[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
  }
  tick(320,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_ACK);
  for(unsigned i=0;i<sizeof(tail);++i) {
    ++rx_bytes;maixcam_rx_byte=tail[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
  }
  tick(340,true);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_ACK);
  fake_now=360;rx(0x26);tick(360,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_ACK);
  fake_now=380;rx(0x05);cargo_report(5);rx(0x06);tick(380,false);tick(880,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE);
  tick(1660,true);tick(1680,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  fake_now=1700;rx(0x26);tick(1700,false);assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
  //A fresh26 at the800ms resume edge waits for the camera, then still delivers.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);MissionTask_StartS6LoadRecheck(100);
  rx(0x05);cargo_report(5);rx(0x06);tick(120,false);tick(620,false);
  fake_now=1400;rx(0x26);tick(1400,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE && s6_align26_pending && !mission_snapshot.left_target_rpm);
  tick(1420,false);assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
  //No coordinates after the hold:1s loss countdown starts at resume, not at06.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);MissionTask_StartS6LoadRecheck(100);
  rx(0x05);cargo_report(5);rx(0x06);tick(120,false);tick(620,false);tick(1420,false);
  tick(2419,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && recovery_calls==0);
  tick(2420,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && recovery_calls==1);
  //A new capture06 resets completion so later loads may evacuate normally.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);s6_corner_completed=true;
  MissionTask_StartS6FromLoad(100);assert(!s6_corner_completed);
  MissionTask_EnterS6TrackingFrom16(100);track_before_angle_decision();
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);track_before_angle_decision();
  MissionTask_EnterState(MISSION_STATE_STOPPED,1200);tick(1220,true);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && mission_snapshot.left_target_rpm==0);
  assert(fake_rx_arms==rx_bytes+uart_inits);
  puts("PASS:all8 zone/team rules,shared supply/casualty corners,casualty corner-first priority and added(190,224]/mapped(10,44] side sector with heading/distance/post36/26 priority,centidegree edges/wrapped zero;500ms XY tracking gate/side-return reset;both corners100rpm/2000+1200ms;ignore26 all steps/ISR exit race/old half-frame;TX05/RX05/near count/wide800ms/direct XY,no new16;no repeated corner via tracking/recovery/36;new load resets guard;fresh26 resume edge;loss countdown/IMU/timeout/stop safety;continuous UART RX");
}
'@
$parserPath = (Join-Path $projectRoot 'maixcam/maixcam_task.c').Replace('\','/')
$parserInclude = "`n#define MaixCam_SendCommand Parser_SendCommand`n#include `"$parserPath`"`n#undef MaixCam_SendCommand`n"
$testSource = Join-Path $outputDir 'verify_s6_corner.c'
[IO.File]::WriteAllText($testSource,$preamble+$parserInclude+$globals+"`n"+$ruleTypes+"`n"+$prototypes+"`n"+$stubs+"`n"+($functions -join "`n")+"`n"+$cases,[Text.UTF8Encoding]::new($false))
$testExe = Join-Path $outputDir 'verify_s6_corner.exe'
& $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/Task" "-I$projectRoot/maixcam" $testSource -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
