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
  'MissionTask_HeadingTurnExpired', 'MissionTask_SetS1Phase', 'MissionTask_StartS1ClawOpenWait',
  'MissionTask_StartS2CrossBump', 'MissionTask_RunS1', 'MissionTask_RunS2CrossBump', 'MissionTask_RunS4ForcedLeftTurn',
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
  'MissionTask_StartS6FoundBafflePulse', 'MissionTask_StopS6DeliveryBafflePwm', 'MissionTask_ServiceS6FoundBafflePulse',
  'MissionTask_RunS6TrackSafeZone', 'MissionTask_RunS6CornerEvacuation', 'MissionTask_RunS6SideReposition',
  'MissionTask_CalculateS6YawPid', 'MissionTask_CalculateS6YawTurn', 'MissionTask_RunS6TurnToHeading',
  'MissionTask_CommandS6Straight', 'MissionTask_CalculateS2YawCorrection', 'MissionTask_StartS6ObstacleAvoidance',
  'MissionTask_ResetS6BorderProgress',
  'MissionTask_RunS6BorderLocate', 'MissionTask_RunS6FinalReverse', 'MissionTask_S7MaterialOnly',
  'MissionTask_RunS6ObstacleAvoidance', 'MissionTask_SetTrackingTargets', 'MissionTask_VisionAgeScale',
  'MissionTask_SetVisionTrackingTargets', 'MissionTask_GuardVisionForward', 'MissionTask_CalculateVisionForward', 'MissionTask_CalculateVisionTurn',
  'MissionTask_IsObjectTrackingPhase', 'MissionTask_RedCoordinateIsFresh', 'MissionTask_UpdateVisionInput',
  'MissionTask_StartS6FinalFrom26', 'MissionTask_HandleS6AlignRequest', 'MissionTask_CheckRunningHealth')
$names += @('MissionTask_StartS6LoadRecheck', 'MissionTask_RunS5CasualtyAdjust', 'MissionTask_RunS6LoadRecheck', 'MissionTask_StartS6FromLoad',
  'MissionTask_ReportS6SearchSoftTimeout', 'MissionTask_RunS6SearchSafeZone')
$names += @('MissionTask_CanPauseForLoadCheck', 'MissionTask_HandleLoadCheckRequest')
$names += @('MissionTask_S6CanStartBarShift', 'MissionTask_HandleS6BarShiftRequest', 'MissionTask_RunS6BarShift')
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
static bool fail46_tx;
static unsigned inject46_tx_response;
static uint32_t inject46_tx_delay_ms;
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
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t command) {
  last_tx_command=command;++tx_calls;
  if(command==0x46) {
    assert((!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm) || s6_bar_shift_completed);
    fake_now+=inject46_tx_delay_ms;inject46_tx_delay_ms=0;
    if(fail46_tx) return HAL_ERROR;
    unsigned response=inject46_tx_response;inject46_tx_response=0;
    if(response==1 || response==2) coords();
    if(response==2) rx(0x26);
    if(response==3) { rx(0x05);cargo_report(5);rx(0x06); }
    if(response==4) rx(0x46);
    if(response==7) rx(0x16);
    if(response==5 || response==6 || response==7) {
      const uint8_t frame[]={0xF1,0xF2,1,0,0,1,0,128,2,0x1F,0x2F};
      for(unsigned i=0;i<sizeof(frame);++i) {
        ++rx_bytes;maixcam_rx_byte=frame[i];MaixCam_UART_RxCpltCallback(maixcam_uart);
      }
    }
    if(response==6) rx(0x26);
  }
  return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) { return HAL_OK; }
static uint16_t departure_baffle, departure_claw;
static unsigned departure_baffle_calls, departure_claw_calls, baffle_stop_calls;
static bool departure_claw_fail;
static bool departure_servo_ok, release_baffle_fail, baffle_stop_fail, baffle_pwm_active;
static HAL_StatusTypeDef Actuator_SetBaffleAngle(uint16_t degrees) {
  ++departure_baffle_calls;
  if(mission_snapshot.state==MISSION_STATE_S6_BORDER_BACKOFF) {
    assert(degrees==ROBOT_S6_PUSH_BAFFLE_DEG);
    assert(departure_claw_calls==1 && (departure_claw==ROBOT_CLAW_CASUALTY_RELEASE_DEG ||
        departure_claw==ROBOT_CLAW_MATERIAL_RELEASE_DEG));
    assert(baffle_pwm_active && departure_baffle==ROBOT_BAFFLE_DOWN_DEG);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    if(release_baffle_fail) return HAL_ERROR;
  }
  if(mission_snapshot.state==MISSION_STATE_S6_FINAL_PUSH) {
    assert(degrees==ROBOT_BAFFLE_UP_DEG && departure_baffle==ROBOT_S6_PUSH_BAFFLE_DEG);
    assert((departure_claw_calls==1 || departure_claw_calls==2) && (departure_claw==ROBOT_CLAW_CASUALTY_RELEASE_DEG ||
        departure_claw==ROBOT_CLAW_MATERIAL_RELEASE_DEG));
    assert(baffle_pwm_active && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    if(release_baffle_fail) return HAL_ERROR;
  }
  if(mission_snapshot.state==MISSION_STATE_S6_BORDER_LOCATE && degrees==ROBOT_BAFFLE_UP_DEG) {
    assert(departure_claw_calls==1 && departure_claw==ROBOT_CLAW_MATERIAL_RELEASE_DEG);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    if(release_baffle_fail) return HAL_ERROR;
  }
  if(!departure_servo_ok) return HAL_ERROR;
  departure_baffle=degrees;baffle_pwm_active=true;return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetClawAngle(uint16_t degrees) {
  ++departure_claw_calls;if(!departure_servo_ok || departure_claw_fail) return HAL_ERROR;
  departure_claw=degrees;return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetClawClosed(void) { return Actuator_SetClawAngle(ROBOT_CLAW_CLOSED_DEG); }
static HAL_StatusTypeDef Actuator_StopBafflePwm(void) {
  ++baffle_stop_calls;
  if(baffle_stop_fail) return HAL_ERROR;
  baffle_pwm_active=false;return HAL_OK;
}
static HAL_StatusTypeDef Actuator_SetCameraNearView(void) { return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetCameraSafeZoneView(void) { return HAL_OK; }
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
  bool keep=state==MISSION_STATE_S6_BORDER_BACKOFF || state==MISSION_STATE_S6_FINAL_PUSH;
  if(((s6_border_baffle_hold_active && !keep) || state==MISSION_STATE_FAULT || state==MISSION_STATE_STOPPED ||
      state==MISSION_STATE_WAIT_START || state==MISSION_STATE_S6_RECHECK_WAIT_ACK ||
      state==MISSION_STATE_S6_RECHECK_WAIT_RESULT) &&
      !MissionTask_StopS6DeliveryBafflePwm()) state=MISSION_STATE_FAULT;
  mission_snapshot.state=state;mission_snapshot.state_entry_tick=now;
  if(state==MISSION_STATE_S6_BORDER_LOCATE || state==MISSION_STATE_S6_FINAL_PUSH) MissionTask_ResetS6BorderProgress(now);
  MaixCam_SetLoadCheckInterruptEnabled(MissionTask_CanPauseForLoadCheck());
  MaixCam_SetCornerEvacuationActive(state>=MISSION_STATE_S6_CORNER_TURN_AWAY &&
      state<=MISSION_STATE_S6_CORNER_TURN_SAFE);
  MaixCam_SetBarShiftActive(state>=MISSION_STATE_S6_BAR_REVERSE && state<=MISSION_STATE_S6_BAR_WAIT_ACK);
  if(state!=MISSION_STATE_S6_TRACK_SAFE_ZONE) {
    s6_pre_reposition_track_active=false;s6_pre_reposition_track_tick=now;
  }
  if(state==MISSION_STATE_FAULT || state==MISSION_STATE_STOPPED) {
    s6_bar_shift_completed=false;s6_bar_shift_ack_pending=false;
    s6_bar_initial_align_pending=false;s6_align26_pending=false;MissionTask_StopWheels();
  }
}
static void MissionTask_DebugS6(uint32_t now) { (void)now; }
static uint32_t MissionTask_GetRequiredFaults(void) { return health_faults; }
static void MissionTask_StartS6Recovery(uint32_t now,bool tracking) {
  (void)tracking;++recovery_calls;MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_START,now);
}
static void MissionTask_DebugS1(uint32_t now) { (void)now; }
static void MissionTask_DebugS2(uint32_t now,int16_t correction) { (void)now;(void)correction; }
static bool MissionTask_StartS3Align(uint32_t now,uint8_t command) {
  assert(command==0x21 && departure_baffle==155 && departure_claw==ROBOT_CLAW_CLOSED_DEG);
  MissionTask_EnterState(MISSION_STATE_S3_LOWER_FRAME,now);return true;
}
'@
$cases = @'
static void setup(uint8_t zone,uint8_t team,uint8_t cargo,int32_t zero,int32_t field) {
  departure_baffle=155;departure_claw=ROBOT_CLAW_CLOSED_DEG;departure_baffle_calls=departure_claw_calls=0;
  departure_claw_fail=false;
  departure_servo_ok=true;release_baffle_fail=false;
  baffle_stop_calls=0;baffle_stop_fail=false;baffle_pwm_active=false;
  s6_found_baffle_pwm_active=s6_border_baffle_hold_active=false;
  s7_delivery_counted=false;
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
  s6_corner_completed=false;s6_recovery_count=0;
  s6_bar_shift_completed=s6_bar_shift_ack_pending=false;
  s6_bar_initial_align_pending=false;
  s5_recheck_resume_safe_tracking=false;
  fake_now=100;health_faults=0;tx_calls=0;recovery_calls=0;inject26_before_corner_run=false;
  fail46_tx=false;inject46_tx_response=0;inject46_tx_delay_ms=0;
  MissionTask_StartS6VisionApproach(fake_now);
}
static void tick(uint32_t now,bool new_frame) {
  fake_now=now;if(new_frame) coords();
  MissionTask_UpdateVisionInput(now);MissionTask_CheckRunningHealth(now);
  (void)MissionTask_HandleLoadCheckRequest(now);
  MissionTask_HandleS6BarShiftRequest(now);
  MissionTask_HandleS6AlignRequest(now);
  if(inject26_before_corner_run) { inject26_before_corner_run=false;rx(0x26); }
  MissionTask_RunS6LoadRecheck(now);MissionTask_RunS6BarShift(now);MissionTask_RunS6SearchSafeZone(now);
  MissionTask_RunS6SideReposition(now);MissionTask_RunS6CornerEvacuation(now);
  MissionTask_RunS6TrackSafeZone(now);
}
static void verify_s6_unaligned_speed(void) {
  const int16_t xs[]={-61,-60,-26,-25,0,25,26,60,61};
  const uint16_t ys[]={251,400,448};
  const int16_t aligned_speeds[]={150,160,179};
  assert(ROBOT_S6_APPROACH_UNALIGNED_RPM==120 && ROBOT_S6_SIDE_X_TOLERANCE_PX==60);
  assert(ROBOT_S6_APPROACH_MIN_FORWARD_RPM==150 && ROBOT_S6_APPROACH_RPM==180);
  assert(ROBOT_S6_APPROACH_MIN_REVERSE_RPM==100 && ROBOT_S6_APPROACH_MAX_REVERSE_RPM==150);
  assert(ROBOT_S6_APPROACH_UNALIGNED_REVERSE_RPM==100 && ROBOT_S6_APPROACH_WHEEL_MAX_RPM==180);
  assert(ROBOT_S6_X_ALIGN_MIN_FRAMES==1 && ROBOT_S6_X_ALIGN_STABLE_MS==0);
  for(uint8_t cargo=4;cargo<=6;++cargo) for(unsigned e=0;e<sizeof(xs)/sizeof(xs[0]);++e)
    for(unsigned y=0;y<sizeof(ys)/sizeof(ys[0]);++y) {
    setup(3,ROBOT_TEAM_BLUE,cargo,35000,18000);
    uint16_t ax=(uint16_t)abs(xs[e]);
    const uint8_t frame[]={0xF1,0xF2,2,0,(uint8_t)ax,xs[e]<0?2:1,(uint8_t)(ys[y]>>8),(uint8_t)(ys[y]&255),2,0x1F,0x2F};
    for(unsigned n=0;n<3;++n) {
      fake_now=120U+n*150U;
      for(unsigned b=0;b<sizeof(frame);++b) {
        ++rx_bytes;maixcam_rx_byte=frame[b];MaixCam_UART_RxCpltCallback(maixcam_uart);
      }
      tick(fake_now,false);
      if(mission_snapshot.vision_forward_rpm!=(ax<=60?aligned_speeds[y]:120))
        fprintf(stderr,"S6 speed mismatch cargo=%u x=%d y=%u frame=%u actual=%d expected=%d age=%lu aligned=%u\n",
            cargo,xs[e],ys[y],n,mission_snapshot.vision_forward_rpm,ax<=60?aligned_speeds[y]:120,
            (unsigned long)mission_snapshot.target_age_ms,MissionTask_S6SideXConfirmed());
      assert(mission_snapshot.vision_forward_rpm==(ax<=60?aligned_speeds[y]:120));
      assert(MissionTask_S6SideXConfirmed()==(ax<=60));
      assert(MissionTask_S6XConfirmed()==(ax<=25));
      assert(abs(mission_snapshot.vision_turn_rpm)<=35);
      assert(abs(mission_snapshot.left_target_rpm)<=180 && abs(mission_snapshot.right_target_rpm)<=180);
      if(!ax) assert(mission_snapshot.left_target_rpm==aligned_speeds[y] && mission_snapshot.right_target_rpm==aligned_speeds[y]);
    }
    // A375ms-old frame still fades both channels, never reapplying a floor.
    tick(795,false);assert(mission_snapshot.vision_forward_rpm==60);
    tick(920,false);assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
  // One new frame is sufficient after an out-of-range frame, long gap or
  // wrong-class interruption. Cached/old coordinates must not keep high speed.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
  const uint8_t edge_xs[]={61,60,61,60};
  for(unsigned i=0;i<sizeof(edge_xs);++i) {
    fake_now=120U+i*20U;
    const uint8_t frame[]={0xF1,0xF2,2,0,edge_xs[i],1,1,192,2,0x1F,0x2F};
    for(unsigned b=0;b<sizeof(frame);++b) { ++rx_bytes;maixcam_rx_byte=frame[b];MaixCam_UART_RxCpltCallback(maixcam_uart); }
    tick(fake_now,false);assert(mission_snapshot.vision_forward_rpm==(edge_xs[i]<=60?179:120));
  }
  tick(200,false);assert(mission_snapshot.vision_forward_rpm==179);
  tick(555,false);assert(mission_snapshot.vision_forward_rpm==60);
  tick(681,false);assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm);
  fake_now=700;aligned_coords();tick(700,false);
  assert(mission_snapshot.vision_forward_rpm==179 && MissionTask_S6SideXConfirmed());
  fake_now=720;
  const uint8_t wrong_class[]={0xF1,0xF2,5,0,0,1,1,192,2,0x1F,0x2F};
  for(unsigned b=0;b<sizeof(wrong_class);++b) { ++rx_bytes;maixcam_rx_byte=wrong_class[b];MaixCam_UART_RxCpltCallback(maixcam_uart); }
  tick(720,false);assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm);
  fake_now=740;aligned_coords();tick(740,false);assert(mission_snapshot.vision_forward_rpm==179);
  puts("PASS:06 unaligned forward cap120rpm,threshold inclusive+/-60px,ONE fresh frame restores150..180,wheel cap180;reverse100..150 unchanged,red/green/black,symmetric X,35rpm steering and250..500ms coordinate fade preserved");
}
static void verify_s6_near_y_speed(void) {
  const int16_t xs[]={-61,-60,-31,-30,-26,-25,0,25,26,30,31,60,61};
  const uint16_t ys[]={0,12,13,29,30,31,39,40,41,50,149,150,151,199,200,201,249,250,251,400,448};
  const int16_t forward_speeds[]={0,0,0,0,0,60,60,60,60,60,60,60,60,60,60,60,60,60,150,160,179};
  const int16_t reverse_speeds[]={0,0,0,0,0,60,60,60,60,60,60,60,60,60,60,60,60,60,100,150,150};
  assert(ROBOT_S6_APPROACH_STOP_Y_PX==30 && ROBOT_S6_APPROACH_NEAR_Y_PX==250 && ROBOT_S6_APPROACH_NEAR_RPM==60);
  assert(ROBOT_S6_APPROACH_STOP_X_PX==25 && ROBOT_S6_SIDE_X_TOLERANCE_PX==60);
  assert(ROBOT_VISION_Y_TOLERANCE_PX==12 && ROBOT_VISION_X_TOLERANCE_PX==15);
  for(uint8_t cargo=4;cargo<=6;++cargo) for(unsigned x=0;x<sizeof(xs)/sizeof(xs[0]);++x)
    for(unsigned y=0;y<sizeof(ys)/sizeof(ys[0]);++y) for(uint8_t sign=1;sign<=2;++sign) {
    setup(3,ROBOT_TEAM_BLUE,cargo,35000,18000);
    uint16_t ax=(uint16_t)abs(xs[x]);
    const uint8_t frame[]={0xF1,0xF2,2,0,(uint8_t)ax,xs[x]<0?2:1,
        (uint8_t)(ys[y]>>8),(uint8_t)(ys[y]&255),sign,0x1F,0x2F};
    int16_t magnitude=sign==2?forward_speeds[y]:reverse_speeds[y];
    int16_t unaligned_cap=sign==2?120:100;
    if(ax>60 && magnitude>unaligned_cap) magnitude=unaligned_cap;
    int16_t expected=sign==2?magnitude:-magnitude;
    for(unsigned n=0;n<3;++n) {
      fake_now=120U+n*150U;
      for(unsigned b=0;b<sizeof(frame);++b) {
        ++rx_bytes;maixcam_rx_byte=frame[b];MaixCam_UART_RxCpltCallback(maixcam_uart);
      }
      tick(fake_now,false);
      assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
      assert(mission_snapshot.vision_forward_rpm==expected);
      assert(abs(mission_snapshot.vision_turn_rpm)<=35);
      if(ax<=25) assert(!mission_snapshot.vision_turn_rpm);
      else assert(mission_snapshot.vision_turn_rpm);
      assert(MissionTask_S6SideXConfirmed()==(ax<=60));
      assert(MissionTask_S6XConfirmed()==(ax<=25));
      assert(abs(mission_snapshot.left_target_rpm)<=180 && abs(mission_snapshot.right_target_rpm)<=180);
      if(sign==1) assert(abs(mission_snapshot.left_target_rpm)<=150 && abs(mission_snapshot.right_target_rpm)<=150);
      if(ax==0) assert(mission_snapshot.left_target_rpm==expected && mission_snapshot.right_target_rpm==expected);
    }
    // Stale input fades the near cap too; it cannot reapply the150rpm floor.
    tick(795,false);
    int16_t faded=ys[y]<=30?0:(ys[y]<=250?30:(sign==2?60:50));
    assert(mission_snapshot.vision_forward_rpm==(sign==2?faded:-faded));
    tick(921,false);
    assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
  puts("PASS:06 |Y|31..250 capped60 both directions,<=30 stops Y only;251+ forward150..180/unaligned120,reverse100..150/unaligned100;red/green/black,Y30/31+250/251,X25/26+60/61 boundaries,object X15/Y12 unchanged,180 wheel cap,35 steering and fade preserved");
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
  tick(1360,true);assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
  tick(1939,true);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_AWAY);
  assert(mission_snapshot.safe_zone_yaw_command_cdeg==away);
  tick(1940,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_SIDE);
  assert(mission_snapshot.left_target_rpm==0);
  fake_now=1960;rx(0x26);
  mission_snapshot.yaw_cdeg=side;tick(1960,false);tick(2160,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_SIDE);
  fake_now=2180;rx(0x26);
  tick(2180,true);assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
  tick(2759,true);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_SIDE);
  assert(mission_snapshot.safe_zone_yaw_command_cdeg==side);
  tick(2760,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_SAFE);
  fake_now=2780;rx(0x26);
  mission_snapshot.yaw_cdeg=safe;tick(2780,true);
  inject26_before_corner_run=true;tick(2980,false); //ISR after task handler but before exit must still be ignored
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_ACK);
  assert(mission_snapshot.left_target_rpm==0 && !mission_snapshot.vision_target_valid);
  assert(s6_carried_object_id==cargo && tx_calls==1 && last_tx_command==0x05 && recovery_calls==0);
  tick(5000,false);assert(!mission_snapshot.vision_target_valid); // cached maneuver coordinates were cleared
  fake_now=5020;rx(0x05);cargo_report(cargo);rx(0x06);rx(0x26);
  tick(5020,true);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  tick(5020U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS-1U,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm && !mission_snapshot.vision_target_valid);
  tick(5020U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE);
  assert(s6_carried_object_id==cargo && tx_calls==1 && s5_load_class_confirmed);
  uint32_t wide_done=5020U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS+ROBOT_S6_RECHECK_WIDE_SETTLE_MS;
  tick(wide_done-20U,true);assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  tick(wide_done-1U,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE);
  tick(wide_done,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed);
  assert(s6_pre_reposition_track_elapsed_ms==0 && s6_pre_reposition_decision_pending);
  tick(wide_done+20U,true);assert(mission_snapshot.vision_target_valid);
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
    MissionTask_EnterState(MISSION_STATE_S6_CORNER_FORWARD_AWAY,120);
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
  MissionTask_RunS1(2120);assert(mission_snapshot.state==MISSION_STATE_S1_DEPART);
  assert(mission_snapshot.s1_phase==MISSION_S1_CLAW_OPEN_WAIT && departure_claw==ROBOT_CLAW_MATERIAL_OPEN_DEG);
  MissionTask_RunS1(2319);assert(mission_snapshot.state==MISSION_STATE_S1_DEPART);
  MissionTask_RunS1(2320);assert(mission_snapshot.state==MISSION_STATE_S2_CROSS_BUMP && departure_baffle==80);
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
static void verify_departure_mechanism(void) {
  assert(ROBOT_S1_CLAW_OPEN_SETTLE_MS==200U && ROBOT_S2_CROSS_BUMP_SETTLE_MS==300U);
  assert(ROBOT_S2_CROSS_BUMP_RPM==110 && ROBOT_S2_CROSS_BUMP_DRIVE_MS==3000U);
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned fail=0;fail<5;++fail) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,0);
    uint32_t start=wrap?UINT32_MAX-100U:100U;
    MissionTask_EnterState(MISSION_STATE_S1_DEPART,start);
    MissionTask_SetS1Phase(MISSION_S1_TURN,start);
    mission_snapshot.yaw_target_cdeg=mission_snapshot.yaw_cdeg;
    MissionTask_RunS1(start);MissionTask_RunS1(start+299U);
    assert(mission_snapshot.s1_phase==MISSION_S1_TURN_STABLE && !departure_claw_calls);
    uint32_t opened=start+300U;departure_servo_ok=fail!=1;
    MissionTask_RunS1(opened);
    if(fail==1) { assert(mission_snapshot.state==MISSION_STATE_FAULT && !departure_baffle_calls);continue; }
    assert(mission_snapshot.s1_phase==MISSION_S1_CLAW_OPEN_WAIT && departure_claw==ROBOT_CLAW_MATERIAL_OPEN_DEG && departure_baffle==155);
    MissionTask_RunS1(opened+199U);
    assert(mission_snapshot.state==MISSION_STATE_S1_DEPART && !mission_snapshot.left_target_rpm && !departure_baffle_calls);
    uint32_t drive=opened+200U;departure_servo_ok=fail!=2;MissionTask_RunS1(drive);
    if(fail==2) { assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);continue; }
    assert(mission_snapshot.state==MISSION_STATE_S2_CROSS_BUMP && departure_baffle==80 && departure_claw==ROBOT_CLAW_MATERIAL_OPEN_DEG);
    MissionTask_RunS2CrossBump(drive);MissionTask_RunS2CrossBump(drive+2999U);
    assert(mission_snapshot.left_target_rpm==110 && mission_snapshot.right_target_rpm==110);
    //The wait begins at the actual down command, even if the task ran late.
    uint32_t down=drive+3013U;departure_servo_ok=fail!=3;MissionTask_RunS2CrossBump(down);
    if(fail==3) { assert(mission_snapshot.state==MISSION_STATE_FAULT && departure_claw==ROBOT_CLAW_MATERIAL_OPEN_DEG);continue; }
    assert(departure_baffle==155 && departure_claw==ROBOT_CLAW_MATERIAL_OPEN_DEG && !mission_snapshot.left_target_rpm);
    MissionTask_RunS2CrossBump(down+299U);
    assert(mission_snapshot.state==MISSION_STATE_S2_CROSS_BUMP && departure_claw_calls==1);
    departure_servo_ok=fail!=4;MissionTask_RunS2CrossBump(down+300U);
    if(fail==4) { assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);continue; }
    assert(mission_snapshot.state==MISSION_STATE_S3_LOWER_FRAME && departure_claw==ROBOT_CLAW_CLOSED_DEG);
    assert(departure_baffle_calls==2 && departure_claw_calls==2 && !mission_snapshot.right_target_rpm);
  }
  puts("PASS:departure turn then claw open/wait200/PA2 up80,110rpm crossing3000,PA2 down155/wait300/claw closed before03;real command-time waits,tick wrap and actuator faults checked");
}

static void verify_16_baffle_pulse(void) {
  assert(ROBOT_S6_FOUND_BAFFLE_PWM_MS==150U);
  for(unsigned wrap=0;wrap<2;++wrap) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    uint32_t start=wrap?UINT32_MAX-100U:120U;
    MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE,start);
    assert(!departure_baffle_calls && !baffle_pwm_active);
    fake_now=start;rx(0x16);tick(start,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
    assert(baffle_pwm_active && departure_baffle==ROBOT_BAFFLE_DOWN_DEG && departure_baffle_calls==1);
    assert(!departure_claw_calls && !baffle_stop_calls && s6_found_baffle_pwm_tick==start);
    fake_now=start+20U;aligned_coords();tick(fake_now,false);
    assert(mission_snapshot.vision_forward_rpm>0 && baffle_pwm_active);
    fake_now=start+40U;rx(0x16);tick(fake_now,false);
    assert(s6_found_baffle_pwm_tick==start && departure_baffle_calls==1);
    tick(start+149U,false);assert(baffle_pwm_active && !baffle_stop_calls);
    tick(start+150U,false);
    assert(!baffle_pwm_active && baffle_stop_calls==1 && mission_snapshot.vision_forward_rpm>0);
    tick(start+170U,false);assert(baffle_stop_calls==1 && !departure_claw_calls);
  }
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);MissionTask_EnterS6TrackingFrom16(120);
  fake_now=140;rx(0x26);tick(140,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && baffle_pwm_active);
  tick(270,false);
  assert(!baffle_pwm_active && departure_baffle_calls==1 && baffle_stop_calls==1 && !departure_claw_calls);
  //If26 actually starts its push before the old16 cutoff, it owns PA2.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);MissionTask_EnterS6TrackingFrom16(120);
  s6_push_yaw_target_cdeg=mission_snapshot.yaw_cdeg;
  MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,140);
  MissionTask_RunS6BorderLocate(140);
  assert(s6_border_baffle_hold_active && !s6_found_baffle_pwm_active && baffle_pwm_active);
  tick(270,false);MissionTask_RunS6BorderLocate(270);
  assert(baffle_pwm_active && !baffle_stop_calls && !departure_claw_calls && departure_claw==ROBOT_CLAW_CLOSED_DEG);
  assert(mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);

  for(unsigned mode=0;mode<5;++mode) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    departure_servo_ok=mode!=2;
    MissionTask_EnterS6TrackingFrom16(120);
    if(mode==0) MissionTask_EnterState(MISSION_STATE_STOPPED,180);
    if(mode==1) { health_faults=MISSION_FAULT_IMU; MissionTask_CheckRunningHealth(180); }
    if(mode==3) { baffle_stop_fail=true; tick(270,false); }
    if(mode==4) { fake_now=180;rx(0x05);tick(180,false); }
    assert(mission_snapshot.state==(mode==0?MISSION_STATE_STOPPED:
        mode==4?MISSION_STATE_S6_RECHECK_WAIT_RESULT:MISSION_STATE_FAULT));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!departure_claw_calls && !s7_delivery_counted);
    if(mode<2 || mode==4) assert(!baffle_pwm_active && baffle_stop_calls==1);
    if(mode==2 || mode==3) assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  }
  puts("PASS:RX16 starts PA2 DOWN PWM150 in parallel with XY motion;149/150,wrap,duplicate16,no claw,early26 handoff,05/stop/health/actuator faults");
}
static void verify_final26_motion_parameters(void) {
  verify_departure_mechanism();
  assert(ROBOT_S6_BORDER_LOCATE_RPM==70 && ROBOT_S6_BORDER_LOCATE_MS==750U);
  assert(ROBOT_S6_BORDER_BACKOFF_MS==250U && ROBOT_S6_FINAL_PUSH_MS==700U);
  assert(ROBOT_S6_PUSH_BAFFLE_DEG==130U && ROBOT_BAFFLE_UP_DEG==80U);
  assert(ROBOT_S6_BORDER_LOCATE_WHEEL_MAX_RPM==170);
  assert(ROBOT_S6_REVERSE_RPM==100 && ROBOT_S6_REVERSE_DRIVE_MS==700U);
  assert(ROBOT_S6_REVERSE_WHEEL_MAX_RPM==200 && ROBOT_S6_EXIT_TURN_MAX_RPM==80);
  assert(ROBOT_S6_EXIT_TURN_MIN_RPM==40.0f);
  assert(ROBOT_CLAW_MATERIAL_RELEASE_DEG==74U && ROBOT_CLAW_CASUALTY_OPEN_DEG==63U);
  assert(ROBOT_CLAW_CASUALTY_RELEASE_DEG==73U);
  for(unsigned wrap=0;wrap<2;++wrap) for(uint8_t cargo=3;cargo<=6;++cargo) {
    setup(3,ROBOT_TEAM_BLUE,cargo,35000,18000);
    uint32_t start=wrap?UINT32_MAX-100U:700U;
    uint32_t backoff_start=start+ROBOT_S6_BORDER_LOCATE_MS;
    uint32_t push_start=backoff_start+ROBOT_S6_BORDER_BACKOFF_MS;
    uint32_t reverse_start=push_start+ROBOT_S6_FINAL_PUSH_MS;
    uint32_t turn_start=reverse_start+ROBOT_S6_REVERSE_DRIVE_MS;
    const uint16_t opening=cargo==4?73U:74U;
    s6_push_yaw_target_cdeg=mission_snapshot.yaw_cdeg;
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,start);
    MissionTask_ResetS6ControllerState();
    MissionTask_RunS6BorderLocate(start);
    assert(departure_claw==ROBOT_CLAW_CLOSED_DEG && !departure_claw_calls && !s7_delivery_counted);
    assert(mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);
    MissionTask_RunS6BorderLocate(backoff_start-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_LOCATE);
    assert(mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);
    assert(departure_baffle_calls==1 && !baffle_stop_calls && baffle_pwm_active && !departure_claw_calls);
    assert(departure_claw==ROBOT_CLAW_CLOSED_DEG && departure_baffle==ROBOT_BAFFLE_DOWN_DEG);
    MissionTask_RunS6BorderLocate(backoff_start);
    assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_BACKOFF);
    assert(mission_snapshot.left_target_rpm==-70 && mission_snapshot.right_target_rpm==-70);
    assert(departure_claw==opening);
    assert(!s7_delivery_counted && departure_claw_calls==1 && departure_baffle_calls==1 && baffle_pwm_active);
    MissionTask_RunS6BorderLocate(push_start-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_BACKOFF && mission_snapshot.left_target_rpm==-70);
    assert(departure_baffle==ROBOT_BAFFLE_DOWN_DEG && departure_claw_calls==1);
    MissionTask_RunS6BorderLocate(push_start);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_PUSH);
    assert(mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);
    assert(departure_baffle==ROBOT_S6_PUSH_BAFFLE_DEG && departure_baffle_calls==2 && baffle_pwm_active);
    MissionTask_RunS6BorderLocate(reverse_start-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_PUSH && mission_snapshot.left_target_rpm==70);
    assert(departure_claw_calls==1 && departure_claw==opening && !s7_delivery_counted && !baffle_stop_calls);
    MissionTask_RunS6BorderLocate(reverse_start);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE);
    assert(departure_claw==opening && departure_claw_calls==1);
    assert(departure_baffle==ROBOT_BAFFLE_UP_DEG && departure_baffle_calls==3 && baffle_pwm_active);
    assert(!baffle_stop_calls && !s6_border_baffle_hold_active);
    MissionTask_RunS6FinalReverse(reverse_start);
    assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
    MissionTask_RunS6FinalReverse(turn_start-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE && mission_snapshot.left_target_rpm==-100);
    assert(departure_claw==opening && departure_claw_calls==1);
    MissionTask_RunS6FinalReverse(turn_start);
    assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180 && !tx_calls);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(departure_claw==ROBOT_CLAW_CLOSED_DEG && departure_claw_calls==2 && departure_baffle_calls==3);
    assert(s6_exit_yaw_target_cdeg==MissionTask_WrapYaw(s6_push_yaw_target_cdeg+ROBOT_YAW_LEFT_SIGN*18000L));
    assert(!MissionTask_RunS6TurnToHeading(turn_start+20U,s6_exit_yaw_target_cdeg));
    assert(abs(mission_snapshot.left_target_rpm)==80 && mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    assert(departure_claw==ROBOT_CLAW_CLOSED_DEG && departure_baffle==ROBOT_BAFFLE_UP_DEG && !tx_calls);
  }
  for(int sign=-1;sign<=1;sign+=2) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    s6_push_yaw_target_cdeg=mission_snapshot.yaw_cdeg;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(s6_push_yaw_target_cdeg+sign*900L);
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,120);MissionTask_ResetS6ControllerState();
    MissionTask_RunS6BorderLocate(140);
    assert(mission_snapshot.left_target_rpm+mission_snapshot.right_target_rpm==140);
    assert(abs(mission_snapshot.left_target_rpm)<=170 && abs(mission_snapshot.right_target_rpm)<=170);
    MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE,160);MissionTask_ResetS6ControllerState();
    MissionTask_RunS6FinalReverse(180);
    assert(mission_snapshot.left_target_rpm+mission_snapshot.right_target_rpm==-200);
    assert(abs(mission_snapshot.left_target_rpm)<=200 && abs(mission_snapshot.right_target_rpm)<=200);
  }
  for(unsigned failure=0;failure<3;++failure) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    s6_push_yaw_target_cdeg=mission_snapshot.yaw_cdeg;
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,120);
    departure_claw_fail=failure==0;
    MissionTask_RunS6BorderLocate(120);
    uint32_t back=120U+ROBOT_S6_BORDER_LOCATE_MS;
    MissionTask_RunS6BorderLocate(back);
    if(failure!=0) {
      uint32_t push=back+ROBOT_S6_BORDER_BACKOFF_MS;
      release_baffle_fail=failure==1;
      MissionTask_RunS6BorderLocate(push);
      if(failure==2) {
        release_baffle_fail=true;
        MissionTask_RunS6BorderLocate(push+ROBOT_S6_FINAL_PUSH_MS);
      }
    }
    assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(departure_claw_calls==1 && departure_baffle_calls==failure+1 && !s7_delivery_counted);
    assert(!baffle_pwm_active && baffle_stop_calls==1);
  }
  for(unsigned mode=0;mode<3;++mode) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    s6_push_yaw_target_cdeg=mission_snapshot.yaw_cdeg;
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,120);
    departure_servo_ok=mode!=0;
    MissionTask_RunS6BorderLocate(120);
    if(mode==1) MissionTask_EnterState(MISSION_STATE_STOPPED,180);
    if(mode==2) { health_faults=MISSION_FAULT_IMU; MissionTask_CheckRunningHealth(180); }
    assert(mission_snapshot.state==(mode==1?MISSION_STATE_STOPPED:MISSION_STATE_FAULT));
    assert(!baffle_pwm_active && baffle_stop_calls==(mode==0?0U:1U));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!departure_claw_calls);
    if(mode==0) assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  }
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE,120);
  departure_claw=ROBOT_CLAW_MATERIAL_OPEN_DEG;departure_servo_ok=false;
  MissionTask_RunS6FinalReverse(820);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR));
  assert(departure_claw_calls==1 && !departure_baffle_calls && !tx_calls);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  puts("PASS:26 closed during F70/750; opens once at B70/250 material74/casualty73; F70/700,full80 and B100/700; closes before exit180; no early counts, actuator faults");
}
static void verify_border_effective_advance(void) {
  for(unsigned wrap=0;wrap<2;++wrap) for(int sign=-1;sign<=1;sign+=2) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    uint32_t start=wrap?UINT32_MAX-100U:700U;
    s6_push_yaw_target_cdeg=100L;
    const int32_t off=MissionTask_WrapYaw(100L+sign*1001L);
    const int32_t boundary=MissionTask_WrapYaw(100L+sign*1000L);
    const int32_t perpendicular=MissionTask_WrapYaw(100L+sign*9000L);
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,start);
    mission_snapshot.yaw_cdeg=off;MissionTask_RunS6BorderLocate(start);
    assert(s6_border_advancing && !s6_border_forward_ms && !departure_claw_calls && departure_claw==ROBOT_CLAW_CLOSED_DEG);
    assert(mission_snapshot.vision_turn_rpm*sign*ROBOT_YAW_LEFT_SIGN>0);
    assert(mission_snapshot.left_target_rpm+mission_snapshot.right_target_rpm==140);
    assert(baffle_pwm_active && departure_baffle_calls==1 && !baffle_stop_calls);
    mission_snapshot.yaw_cdeg=boundary;MissionTask_RunS6BorderLocate(start+300U);
    assert(s6_border_advancing && s6_border_forward_ms==300U);
    assert(baffle_pwm_active && departure_baffle_calls==1 && !baffle_stop_calls);
    mission_snapshot.yaw_cdeg=perpendicular;MissionTask_RunS6BorderLocate(start+600U);
    assert(s6_border_forward_ms==600U && !s6_border_advancing && mission_snapshot.vision_turn_rpm!=0);
    assert(baffle_pwm_active && !baffle_stop_calls);
    MissionTask_RunS6BorderLocate(start+800U);
    assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_LOCATE && !departure_claw_calls && departure_baffle_calls==1);
    assert(s6_border_forward_ms==600U && !mission_snapshot.vision_forward_rpm);
    mission_snapshot.yaw_cdeg=100L;MissionTask_RunS6BorderLocate(start+1000U);
    uint32_t first_end=start+1000U+ROBOT_S6_BORDER_LOCATE_MS-600U;
    MissionTask_RunS6BorderLocate(first_end-1U);
    assert(s6_border_forward_ms==749U && s6_border_advancing && !s7_delivery_counted);
    mission_snapshot.yaw_cdeg=off;MissionTask_RunS6BorderLocate(first_end);
    assert(s6_border_forward_ms==750U && !s6_border_advancing && !s7_delivery_counted);
    assert(!departure_claw_calls);
    assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_LOCATE && baffle_pwm_active && !baffle_stop_calls);
    mission_snapshot.yaw_cdeg=boundary;MissionTask_RunS6BorderLocate(first_end+100U);
    assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_BACKOFF && departure_claw_calls==1);
    uint32_t push=first_end+100U+ROBOT_S6_BORDER_BACKOFF_MS;
    mission_snapshot.yaw_cdeg=off;MissionTask_RunS6BorderLocate(push);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_PUSH && s6_border_advancing && !s6_border_forward_ms);
    assert(mission_snapshot.vision_forward_rpm==70 && mission_snapshot.vision_turn_rpm*sign*ROBOT_YAW_LEFT_SIGN>0);
    assert(departure_baffle==ROBOT_S6_PUSH_BAFFLE_DEG && departure_claw_calls==1 && baffle_pwm_active);
    MissionTask_RunS6BorderLocate(push+699U);
    assert(s6_border_forward_ms==699U && !s7_delivery_counted);
    MissionTask_RunS6BorderLocate(push+700U);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_PUSH && s6_border_forward_ms==700U);
    assert(!mission_snapshot.vision_forward_rpm && mission_snapshot.vision_turn_rpm!=0 && !s7_delivery_counted);
    mission_snapshot.yaw_cdeg=boundary;MissionTask_RunS6BorderLocate(push+800U);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE && s6_border_forward_ms==700U);
    assert(departure_claw_calls==1 && departure_baffle_calls==3 && departure_baffle==ROBOT_BAFFLE_UP_DEG);
  }
  for(unsigned leg=0;leg<2;++leg) for(unsigned failure=0;failure<3;++failure) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    uint32_t start=UINT32_MAX-1000U;
    s6_push_yaw_target_cdeg=100L;mission_snapshot.yaw_cdeg=100L;
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,start);
    if(leg) {
      MissionTask_RunS6BorderLocate(start);
      start+=ROBOT_S6_BORDER_LOCATE_MS;
      MissionTask_RunS6BorderLocate(start);
      start+=ROBOT_S6_BORDER_BACKOFF_MS;
    }
    mission_snapshot.yaw_cdeg=8100L; //80deg still drives and corrects.
    MissionTask_RunS6BorderLocate(start);
    assert(mission_snapshot.vision_forward_rpm==70 && mission_snapshot.vision_turn_rpm!=0);
    MissionTask_RunS6BorderLocate(start+2999U);
    assert(mission_snapshot.state==(leg?MISSION_STATE_S6_FINAL_PUSH:MISSION_STATE_S6_BORDER_LOCATE));
    assert(departure_claw_calls==leg && !s7_delivery_counted && !mission_snapshot.vision_forward_rpm);
    release_baffle_fail=failure==1;
    departure_claw_fail=failure==2;
    MissionTask_RunS6BorderLocate(start+3000U);
    assert(departure_claw_calls==leg+1 && !s7_delivery_counted);
    if(failure) {
      assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags&MISSION_FAULT_ACTUATOR));
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
      assert(!baffle_pwm_active && baffle_stop_calls==1);
      continue;
    }
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE && !mission_snapshot.fault_flags);
    assert(departure_claw==74U);
    assert(departure_baffle==ROBOT_BAFFLE_UP_DEG && baffle_pwm_active && !baffle_stop_calls);
    MissionTask_RunS6FinalReverse(start+3000U);
    assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
    MissionTask_RunS6FinalReverse(start+3700U);
    assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180 && !tx_calls);
    assert(s6_exit_yaw_target_cdeg==26100L && departure_claw==ROBOT_CLAW_CLOSED_DEG);
  }
  puts("PASS:26 both legs correct in motion over10deg,90deg steers in place;2999/3000 opens claw before forced reverse without count,wrap,180deg exit and actuator safety");
}

static void verify_exit180_speed_and_corner_cruise(void) {
  const int32_t errors[]={18000,9000,4500,3000,1001,1000,900,400};
  const int speeds[]={80,80,54,40,40,0,0,0};
  assert(ROBOT_S6_EXIT_TURN_TOLERANCE_CDEG==1000L && ROBOT_S6_TURN_TOLERANCE_CDEG==400L);
  for(int sign=-1;sign<=1;sign+=2) for(unsigned e=0;e<sizeof(errors)/sizeof(errors[0]);++e) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);MissionTask_EnterState(MISSION_STATE_S6_EXIT_TURN_180,120);
    MissionTask_ResetS6ControllerState();s6_phase_stable_since=0;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(100L-sign*errors[e]);
    assert(!MissionTask_RunS6TurnToHeading(140,100));
    assert(abs(mission_snapshot.left_target_rpm)==speeds[e] && mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    if(errors[e]<=1000) {
      assert(!MissionTask_RunS6TurnToHeading(339,100));
      assert(MissionTask_RunS6TurnToHeading(340,100));
    }
  }
  const MissionState stages[]={MISSION_STATE_S6_CORNER_FORWARD_AWAY,MISSION_STATE_S6_CORNER_FORWARD_SIDE};
  const MissionState next[]={MISSION_STATE_S6_CORNER_TURN_SIDE,MISSION_STATE_S6_CORNER_TURN_SAFE};
  const uint32_t durations[]={600,600};
  const int32_t yaw_errors[]={0,900,-900,1001};
  for(unsigned p=0;p<2;++p) for(unsigned wrap=0;wrap<2;++wrap)
    for(unsigned e=0;e<sizeof(yaw_errors)/sizeof(yaw_errors[0]);++e) {
      setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
      uint32_t start=wrap?UINT32_MAX-100U:100U;
      s6_corner_away_yaw_cdeg=s6_corner_side_yaw_cdeg=100;
      s6_approach_start_tick=start;MissionTask_EnterState(stages[p],start);MissionTask_ResetS6ControllerState();
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(100L-yaw_errors[e]);
      MissionTask_RunS6CornerEvacuation(start+durations[p]-1U);
      assert(mission_snapshot.state==stages[p] && !tx_calls && s6_yaw_pid_initialized);
      if(e<3) assert(mission_snapshot.left_target_rpm+mission_snapshot.right_target_rpm==240);
      else assert(mission_snapshot.left_target_rpm+mission_snapshot.right_target_rpm==0); //Existing>10deg heading gate.
      assert(abs(mission_snapshot.left_target_rpm)<=200 && abs(mission_snapshot.right_target_rpm)<=200);
      MissionTask_RunS6CornerEvacuation(start+durations[p]);
      assert(mission_snapshot.state==next[p] && !mission_snapshot.left_target_rpm && !tx_calls);
    }
  puts("PASS:exit180 only40..80rpm taper80/54/40,both directions across0,10deg/200ms arrival and10.01deg still turns;other turns4deg;corner120rpm600/600 with yaw PID and200cap retains cruise,>10deg gate/boundaries/wrap");
}
static void verify_side_turn_pid80(void) {
  const MissionState phases[]={MISSION_STATE_S6_REPOSITION_TURN_SIDE,MISSION_STATE_S6_REPOSITION_TURN_SAFE};
  const int32_t goals[]={100L,35900L};
  const int32_t errors[]={9000,4500,4000,2000,1500,1000,800,500,401,400};
  const int expected[]={80,79,70,35,22,12,10,10,10,0};
  assert(ROBOT_S6_REPOSITION_TURN_MAX_RPM==80 && ROBOT_S6_CORNER_TURN_MAX_RPM==55);
  assert(ROBOT_S6_REPOSITION_YAW_KI==0.0f && ROBOT_S6_REPOSITION_YAW_KD==0.0f);
  assert(ROBOT_S6_REPOSITION_SLOW_BEGIN_CDEG==2000 && ROBOT_S6_REPOSITION_SLOW_END_CDEG==1000);
  for(unsigned p=0;p<2;++p) for(unsigned g=0;g<2;++g) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned e=0;e<sizeof(errors)/sizeof(errors[0]);++e) {
      setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
      MissionTask_EnterState(phases[p],120);MissionTask_ResetS6ControllerState();s6_phase_stable_since=0;
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(goals[g]-sign*errors[e]);
      assert(!MissionTask_RunS6TurnToHeading(140,goals[g]));
      assert(mission_snapshot.left_target_rpm==-sign*expected[e]*ROBOT_YAW_LEFT_SIGN);
      assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
      assert(mission_snapshot.safe_zone_yaw_error_cdeg==sign*errors[e]);
      if(errors[e]==400) {
        assert(!mission_snapshot.left_target_rpm && s6_phase_stable_since==140);
        assert(!MissionTask_RunS6TurnToHeading(339,goals[g]));
        assert(MissionTask_RunS6TurnToHeading(340,goals[g]));
      }
    }
  // Corner turns retain their old0.012 gain and55 cap in both directions.
  const MissionState corners[]={MISSION_STATE_S6_CORNER_TURN_AWAY,MISSION_STATE_S6_CORNER_TURN_SIDE,MISSION_STATE_S6_CORNER_TURN_SAFE};
  for(unsigned p=0;p<3;++p) for(int sign=-1;sign<=1;sign+=2) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    MissionTask_EnterState(corners[p],120);MissionTask_ResetS6ControllerState();s6_phase_stable_since=0;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(100L-sign*4500L);
    assert(!MissionTask_RunS6TurnToHeading(140,100));
    assert(mission_snapshot.left_target_rpm==-sign*54*ROBOT_YAW_LEFT_SIGN);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(100L-sign*9000L);
    assert(!MissionTask_RunS6TurnToHeading(160,100));
    assert(mission_snapshot.left_target_rpm==-sign*55*ROBOT_YAW_LEFT_SIGN);
  }
  puts("PASS:both side turns80rpm cap,Kp0.0175 at>=20deg,smooth10..20deg blend and old0.012 gain within10deg;90/45/40/20/15/10/8/5/4.01/4deg errors in both directions across0,4deg/200ms arrival;all corner turns retain0.012/55rpm");
}
static void verify_side_forward_without_yaw(void) {
  const int32_t errors[]={-18000,-1001,-301,0,301,1001,17900};
  const uint32_t durations[]={408,817,1225};
  for(unsigned side=0;side<2;++side) for(unsigned wrap=0;wrap<2;++wrap)
    for(unsigned d=0;d<3;++d) for(unsigned e=0;e<sizeof(errors)/sizeof(errors[0]);++e) {
      setup(3,ROBOT_TEAM_BLUE,side?4:5,35000,18000);
      uint32_t start=wrap?UINT32_MAX-200U:700U;
      s6_approach_start_tick=start;s6_reposition_forward_ms=durations[d];
      s6_reposition_side_yaw_cdeg=MissionTask_WrapYaw(35000+(side?27000:9000));
      MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_FORWARD,start);
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(s6_reposition_side_yaw_cdeg-errors[e]);
      s6_yaw_pid_update_tick=start-10U;s6_yaw_pid_integral=123.0f;
      fake_now=start+20U;coords();MissionTask_UpdateVisionInput(fake_now);
      MissionTask_RunS6SideReposition(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD);
      assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
      assert(mission_snapshot.vision_forward_rpm==120 && !mission_snapshot.vision_turn_rpm);
      assert(!mission_snapshot.safe_zone_fixed_heading_active);
      assert(s6_yaw_pid_update_tick==start-10U && s6_yaw_pid_integral==123.0f);
      fake_now=start+durations[d]-1U;MissionTask_RunS6SideReposition(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD && s6_reposition_forward_ms==durations[d]);
      assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
      assert(s6_yaw_pid_update_tick==start-10U && s6_yaw_pid_integral==123.0f);
      fake_now=start+durations[d];MissionTask_RunS6SideReposition(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE);
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg+9000);
      fake_now+=20U;MissionTask_RunS6SideReposition(fake_now);
      assert(mission_snapshot.left_target_rpm==80 && mission_snapshot.right_target_rpm==-80);
      mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
      fake_now+=20U;MissionTask_RunS6SideReposition(fake_now);
      fake_now+=ROBOT_S6_TURN_STABLE_MS;MissionTask_RunS6SideReposition(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !s6_pre_reposition_track_elapsed_ms);
    }
  puts("PASS:side forward120/120 NO yaw PID/gate or pixel steering even180deg error,408/817/1225ms/tick wrap;80rpm return-turn PID and fresh1s tracking restart preserved;corner/46 straight yaw loops unchanged");
}
int main(void) {
  verify_s6_unaligned_speed();
  verify_s6_near_y_speed();
  verify_s6_bar_shift();
  verify_yaw_deadbands();
  verify_heading_turn_timeout();
  verify_side_turn_pid80();
  verify_16_baffle_pulse();
  verify_final26_motion_parameters();
  verify_border_effective_advance();
  verify_exit180_speed_and_corner_cruise();
  verify_side_forward_without_yaw();
  // Real tracking must not erase6E history on every valid-coordinate tick.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);s6_recovery_count=1;
  tick(120,true);tick(320,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_recovery_count==1);
  MissionTask_StartS6VisionApproach(340);assert(s6_recovery_count==1);
  MissionTask_EnterS6TrackingFrom16(360);tick(380,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_recovery_count==1);
  s6_recovery_count=3;MissionTask_StartS6FromLoad(400);
  assert(s6_recovery_count==0);
  puts("PASS:real06 tracking/XY re-entry/16 preserve6E count;new load06 resets");
  assert(ROBOT_S6_PRE_REPOSITION_TRACK_MS==1000);
  assert(ROBOT_S6_RECOVERY_SCAN_MAX_RPM==20 && ROBOT_S6_RECOVERY_SCAN_MIN_RPM==20);
  // Both6E sweep directions use20rpm; reaching the heading adds no wait.
  const MissionState recovery_scan_states[]={MISSION_STATE_S6_RECOVERY_TURN_LEFT,
                                             MISSION_STATE_S6_RECOVERY_TURN_BACK};
  for(unsigned i=0;i<2;++i) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    MissionTask_EnterState(recovery_scan_states[i],120);
    MissionTask_ResetS6ControllerState();
    int32_t scan_goal=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+(i?-9000:4500));
    assert(!MissionTask_RunS6TurnToHeading(140,scan_goal));
    assert(mission_snapshot.left_target_rpm==(i?20:-20) &&
           mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    mission_snapshot.yaw_cdeg=scan_goal;
    assert(MissionTask_RunS6TurnToHeading(160,scan_goal));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
  assert(MISSION_STATE_SUPPLEMENT_CENTER_OBJECT==85 && MISSION_STATE_S6_CORNER_TURN_AWAY==86);
  assert(ROBOT_S6_CORNER_FORWARD_RPM==120 && ROBOT_S6_CORNER_AWAY_FORWARD_MS==600 && ROBOT_S6_CORNER_SIDE_FORWARD_MS==600);
  assert(ROBOT_S6_REPOSITION_FORWARD_RPM==120 && ROBOT_S6_CORNER_WHEEL_MAX_RPM==200);
  assert(ROBOT_S6_REPOSITION_WHEEL_MAX_RPM==170 && ROBOT_S6_REPOSITION_TURN_MAX_RPM==80.0f);
  assert(ROBOT_S6_CORNER_TURN_MAX_RPM==55.0f);
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
  assert(ROBOT_S6_OBSTACLE_TURN_MAX_RPM==85 && ROBOT_S6_REPOSITION_TURN_MAX_RPM==80);
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
        int32_t travel_error=(int32_t)(s6_reposition_forward_ms*ROBOT_S6_REPOSITION_FORWARD_RPM)-
            (int32_t)(49000U*(3U-i));
        assert(abs(travel_error)<=ROBOT_S6_REPOSITION_FORWARD_RPM/2+1);
      }
      {
        const int32_t second_angles[]={19001,20700,22400};
        const uint32_t second_times[]={409,817,1225};
        for(unsigned i=0;i<3;++i) {
          setup(zone,team,cargo,35000,MissionTask_WrapYaw(second_angles[i]+safe-18000));
          assert(MissionTask_TryS6AngleManeuver(120,mission_snapshot.yaw_cdeg));
          assert(s6_reposition_forward_ms==second_times[i]);
          float expected_travel=49000.0f*(1.0f+2.0f*(float)(second_angles[i]-19000L)/3400.0f);
          float distance_error=(float)(s6_reposition_forward_ms*ROBOT_S6_REPOSITION_FORWARD_RPM)-expected_travel;
          if(distance_error<0.0f) distance_error=-distance_error;
          assert(distance_error<=(float)ROBOT_S6_REPOSITION_FORWARD_RPM*0.5f+1.0f);
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
  puts("PASS:ordinary side equal120rpm NO yaw PID,408/817/1225ms preserves1x/2x/3x nominal distance within0.5ms rounding in all zone/team/cargo rules;80rpm side turns and independent corner routes");
  // A single fresh X-aligned frame immediately enables full approach speed.
  setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
  fake_now=120;aligned_coords();tick(120,false);
  assert(mission_snapshot.vision_forward_rpm==179);
  fake_now=320;aligned_coords();tick(320,false);
  assert(mission_snapshot.vision_forward_rpm==179);
  fake_now=520;aligned_coords();tick(520,false);
  assert(mission_snapshot.vision_forward_rpm==179 && mission_snapshot.left_target_rpm==179 && mission_snapshot.right_target_rpm==179);
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
  // Each side-return restarts the full1000ms interval, not the old elapsed time.
  setup(3,ROBOT_TEAM_BLUE,5,35000,13500);track_before_angle_decision();
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
  const uint32_t side_restart=120+ROBOT_S6_PRE_REPOSITION_TRACK_MS+80;
  MissionTask_StartS6VisionApproach(side_restart);
  assert(!s6_pre_reposition_track_elapsed_ms && s6_pre_reposition_decision_pending);
  const uint32_t side_new_coords=side_restart+20;
  tick(side_new_coords,true);
  for(uint32_t now=side_new_coords+200;now<side_new_coords+ROBOT_S6_PRE_REPOSITION_TRACK_MS;now+=200)
    tick(now,true);
  tick(side_new_coords+ROBOT_S6_PRE_REPOSITION_TRACK_MS-1,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE &&
         s6_pre_reposition_track_elapsed_ms==ROBOT_S6_PRE_REPOSITION_TRACK_MS-1);
  tick(side_new_coords+ROBOT_S6_PRE_REPOSITION_TRACK_MS,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
  // A brief missing frame pauses, but does not clear the accumulated1000ms gate.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);tick(120,true);tick(520,true);
  assert(s6_pre_reposition_track_elapsed_ms==400);
  fake_now=540;MaixCam_ClearObject();tick(540,false);tick(560,true);
  assert(s6_pre_reposition_track_elapsed_ms==400);
  tick(960,true);assert(s6_pre_reposition_track_elapsed_ms==800);
  const uint32_t paused_decision=560+ROBOT_S6_PRE_REPOSITION_TRACK_MS-400;
  tick(paused_decision-1,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE &&
         s6_pre_reposition_track_elapsed_ms==ROBOT_S6_PRE_REPOSITION_TRACK_MS-1);
  tick(paused_decision,true);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
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
    assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
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
  puts("PASS:all8 zone/team rules,shared supply/casualty corners,casualty corner-first priority and added(190,224]/mapped(10,44] side sector with heading/distance/post36/26 priority,centidegree edges/wrapped zero;1000ms XY tracking gate/side-return reset;both corners120rpm/600+600ms;ignore26 all steps/ISR exit race/old half-frame;TX05/RX05/near count/wide800ms/direct XY,no new16;no repeated corner via tracking/recovery/36;new load resets guard;fresh26 resume edge;loss countdown/IMU/timeout/stop safety;continuous UART RX");
}
'@
# Compile the actual post46 clock-refresh block from the production loop.
# A hand-written fresh clock here would hide a stale-clock firmware regression.
$handoff = [regex]::Match($source, 'MissionTask_RunS6BarShift\(now\);(?<clock>[\s\S]*?)MissionTask_RunS6SearchSafeZone\(now\);')
if (-not $handoff.Success -or
    -not $handoff.Groups['clock'].Value.Contains('now = HAL_GetTick();') -or
    -not $handoff.Groups['clock'].Value.Contains('MissionTask_UpdateVisionInput(now);') -or
    -not $handoff.Groups['clock'].Value.Contains('MissionTask_ServiceS6FoundBafflePulse(now);')) {
  throw 'Production loop must refresh clock and coordinate age after46'
}
$clockHook = 'MissionTask_RunS6BarShift(now);MissionTask_RunS6SearchSafeZone(now);'
if (-not $cases.Contains($clockHook)) { throw 'Missing host46 handoff clock hook' }
$cases = $cases.Replace($clockHook,
  'MissionTask_RunS6BarShift(now);' + $handoff.Groups['clock'].Value + 'MissionTask_RunS6SearchSafeZone(now);')
$parserPath = (Join-Path $projectRoot 'maixcam/maixcam_task.c').Replace('\','/')
$parserInclude = "`n#define MaixCam_SendCommand Parser_SendCommand`n#include `"$parserPath`"`n#undef MaixCam_SendCommand`n"
$barCases = Get-Content (Join-Path $PSScriptRoot 'host/s6_bar_shift_cases.c') -Raw
$align26Cases = Get-Content (Join-Path $PSScriptRoot 'host/s6_align26_corner_cases.c') -Raw
$cases = $cases.Replace('int main(void) {',
  $barCases + "`n" + $align26Cases + "`nint main(void) {`n  verify_s6_align26_corner();")
$testSource = Join-Path $outputDir 'verify_s6_corner.c'
[IO.File]::WriteAllText($testSource,$preamble+$parserInclude+$globals+"`n"+$ruleTypes+"`n"+$prototypes+"`n"+$stubs+"`n"+($functions -join "`n")+"`n"+$cases,[Text.UTF8Encoding]::new($false))
$testExe = Join-Path $outputDir 'verify_s6_corner.exe'
& $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/Task" "-I$projectRoot/maixcam" "-I$projectRoot/User/Hardware/Serial/inc" $testSource -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
