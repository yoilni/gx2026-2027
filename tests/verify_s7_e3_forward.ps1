param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/s7_e3_host'
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
  'MissionTask_HeadingTurnExpired',
  'MissionTask_ActiveRecoveryLossEvent', 'MissionTask_RedCoordinateIsFresh',
  'MissionTask_StartVisionSearchRecovery', 'MissionTask_RunRedSearchSweep',
  'MissionTask_RunS7RedE3Recovery',
  'MissionTask_RunS3SearchTurn',
  'MissionTask_ResumeRedTracking', 'MissionTask_CompleteS7RedRecovery')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$prototypes = ($functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).TrimEnd() + ';' }) -join "`n"
$start = $source.IndexOf('static osThreadId_t mission_task_handle;')
$globals = $source.Substring($start, $source.IndexOf('static const osThreadAttr_t mission_task_attributes') - $start)
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
static unsigned switches, tx13, clears;
static bool forward_log;
static HAL_StatusTypeDef tx_status;
static bool DebugUart_Log(const char *s) { (void)s; return true; }
static bool DebugUart_Logf(const char *s,...) {
  char text[256]; va_list a; va_start(a,s); vsnprintf(text,sizeof(text),s,a); va_end(a);
  if (strstr(text,"[07-E3] FWD=80/800")) forward_log=true;
  return true;
}
static void MissionTask_StopWheels(void) {
  mission_snapshot.left_target_rpm=mission_snapshot.right_target_rpm=0;
}
static void MissionTask_SetWheelTargets(int16_t l,int16_t r) {
  mission_snapshot.left_target_rpm=l; mission_snapshot.right_target_rpm=r;
}
static void MissionTask_ResetVisionControllerState(void) {}
static void MissionTask_ResetS6ControllerState(void) {}
static int16_t MissionTask_CalculateS6YawTurn(uint32_t t,int32_t yaw) {
  (void)t; mission_snapshot.safe_zone_yaw_error_cdeg=yaw-mission_snapshot.yaw_cdeg;
  return 0;
}
static HAL_StatusTypeDef Actuator_SetFrameLowered(void) { return HAL_OK; }
static HAL_StatusTypeDef Actuator_SetCameraWideView(void) { return HAL_OK; }
void MaixCam_ClearObject(void) { ++clears; }
void MaixCam_ClearEvent(void) { ++clears; }
HAL_StatusTypeDef MaixCam_SendCommand(uint8_t c) {
  assert(c==MAIXCAM_COMMAND_RED_SEARCH_DONE); ++tx13; return tx_status;
}
static void MissionTask_EnterState(MissionState s,uint32_t t) {
  mission_snapshot.state=s; mission_snapshot.state_entry_tick=t;
}
static void MissionTask_SwitchS7ToBlackGreen(uint32_t t) {
  (void)t; ++switches; s3_target_select_command=MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
}
static bool MissionTask_TryStartS3Capture(uint32_t t) { (void)t; return false; }
static bool MissionTask_ReturnToS4TrackCenter(uint32_t t) { (void)t; return true; }
static void MissionTask_RunS3GreenE3Recovery(uint32_t t) { (void)t; }
'@
$cases = @'
static void setup(void) {
  memset(&mission_snapshot,0,sizeof(mission_snapshot));
  mission_snapshot.state=MISSION_STATE_S7_SEARCH_RED;
  mission_snapshot.yaw_cdeg=35000;
  s7_target_switch_enabled=true; s7_red_recovery_used=false;
  s3_target_select_command=MAIXCAM_COMMAND_SELECT_RED;
  switches=tx13=clears=0; forward_log=false;
  tx_status=HAL_OK;
}
static void fresh_red(void) {
  mission_snapshot.vision_target_valid=true; mission_snapshot.target_id=4;
  mission_snapshot.target_age_ms=0; mission_snapshot.target_sequence=123;
}
static uint32_t reach_phase(uint32_t start,S7RedE3Phase phase) {
  MissionTask_StartVisionSearchRecovery(start,MISSION_STATE_S7_SEARCH_RED,
                                        mission_snapshot.yaw_cdeg,false);
  uint32_t now=start;
  if(phase==S7_RED_E3_FORWARD) return now;
  now+=800; MissionTask_RunS3SearchTurn(now);
  if(phase==S7_RED_E3_TURN_LEFT) return now;
  mission_snapshot.yaw_cdeg=s3_search_yaw_target_cdeg;
  now+=20; MissionTask_RunS3SearchTurn(now);
  if(phase==S7_RED_E3_HOLD_LEFT) return now;
  now+=500; MissionTask_RunS3SearchTurn(now);
  if(phase==S7_RED_E3_TURN_RIGHT) return now;
  mission_snapshot.yaw_cdeg=s3_search_yaw_target_cdeg;
  now+=20; MissionTask_RunS3SearchTurn(now);
  if(phase==S7_RED_E3_HOLD_RIGHT) return now;
  now+=500; MissionTask_RunS3SearchTurn(now);
  assert(phase==S7_RED_E3_TURN_HOME);
  return now;
}
int main(void) {
  setup();
  MissionTask_StartVisionSearchRecovery(100,MISSION_STATE_S7_SEARCH_RED,35000,false);
  assert(s7_red_recovery_used && forward_log && vision_search_e3_mode);
  assert(vision_search_forward_rpm==80 && vision_search_forward_ms==800);
  MissionTask_RunS3SearchTurn(101);
  assert(mission_snapshot.left_target_rpm==80 && mission_snapshot.right_target_rpm==80);
  MissionTask_StartVisionSearchRecovery(300,MISSION_STATE_S7_SEARCH_RED,35000,false);
  assert(mission_snapshot.state_entry_tick==100 && !switches);
  MissionTask_RunS3SearchTurn(899);
  assert(mission_snapshot.left_target_rpm==80 && vision_search_forward_active);
  MissionTask_RunS3SearchTurn(900);
  assert(!vision_search_forward_active && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
  assert(mission_snapshot.left_target_rpm==0 && !tx13);
  assert(s7_red_e3_phase==S7_RED_E3_TURN_LEFT && s7_red_e3_origin_yaw_cdeg==35000);
  assert(s3_search_yaw_target_cdeg==5000);
  MissionTask_RunS3SearchTurn(920);
  assert(mission_snapshot.left_target_rpm==-45 && mission_snapshot.right_target_rpm==45);
  mission_snapshot.yaw_cdeg=4200; MissionTask_RunS3SearchTurn(940);
  assert(mission_snapshot.left_target_rpm==-15 && mission_snapshot.right_target_rpm==15);
  mission_snapshot.yaw_cdeg=5000; MissionTask_RunS3SearchTurn(960);
  assert(s7_red_e3_phase==S7_RED_E3_HOLD_LEFT && !mission_snapshot.left_target_rpm);
  MissionTask_StartVisionSearchRecovery(1100,MISSION_STATE_S7_SEARCH_RED,5000,false);
  assert(mission_snapshot.state_entry_tick==960 && !switches);
  MissionTask_RunS3SearchTurn(1459);
  assert(s7_red_e3_phase==S7_RED_E3_HOLD_LEFT && !mission_snapshot.left_target_rpm);
  MissionTask_RunS3SearchTurn(1460);
  assert(s7_red_e3_phase==S7_RED_E3_TURN_RIGHT && s3_search_yaw_target_cdeg==29000);
  assert(mission_snapshot.state_entry_tick==1460);
  MissionTask_RunS3SearchTurn(1480);
  assert(mission_snapshot.left_target_rpm==45 && mission_snapshot.right_target_rpm==-45);
  mission_snapshot.yaw_cdeg=29800; MissionTask_RunS3SearchTurn(1500);
  assert(mission_snapshot.left_target_rpm==15 && mission_snapshot.right_target_rpm==-15);
  mission_snapshot.yaw_cdeg=29000; MissionTask_RunS3SearchTurn(1520);
  assert(s7_red_e3_phase==S7_RED_E3_HOLD_RIGHT && !mission_snapshot.left_target_rpm);
  MissionTask_RunS3SearchTurn(2019);
  assert(s7_red_e3_phase==S7_RED_E3_HOLD_RIGHT && !mission_snapshot.left_target_rpm);
  MissionTask_RunS3SearchTurn(2020);
  assert(s7_red_e3_phase==S7_RED_E3_TURN_HOME && s3_search_yaw_target_cdeg==35000);
  MissionTask_RunS3SearchTurn(2040);
  assert(mission_snapshot.left_target_rpm==-45 && mission_snapshot.right_target_rpm==45);
  mission_snapshot.yaw_cdeg=35000; MissionTask_RunS3SearchTurn(2060);
  assert(tx13==1 && !mission_snapshot.left_target_rpm);
  assert(mission_snapshot.state==MISSION_STATE_S7_WAIT_SECOND_E3);

  // Reacquisition interrupts every motion/hold leg without TX11/03 or TX13.
  for(unsigned p=S7_RED_E3_FORWARD;p<=S7_RED_E3_TURN_HOME;++p) {
    setup(); uint32_t now=reach_phase(3000,(S7RedE3Phase)p);
    assert(s7_red_e3_phase==(S7RedE3Phase)p);
    MissionTask_StartVisionSearchRecovery(now+1,MISSION_STATE_S7_SEARCH_RED,0,false);
    assert(mission_snapshot.state_entry_tick==now && !switches);
    fresh_red(); MissionTask_RunS3SearchTurn(now+20);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s7_red_recovery_used);
    assert(mission_snapshot.target_sequence==123 && !clears && !tx13);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }

  // Fresh wrong-class coordinates never interrupt a red sweep/hold.
  for(unsigned p=S7_RED_E3_TURN_LEFT;p<=S7_RED_E3_TURN_HOME;++p) {
    setup(); uint32_t now=reach_phase(5000,(S7RedE3Phase)p);
    mission_snapshot.vision_target_valid=true; mission_snapshot.target_id=6;
    MissionTask_RunS3SearchTurn(now+20);
    assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW && !tx13);
  }

  // Heading wrap and forward drift: every sweep returns to its own origin.
  const int32_t headings[]={0,35000,6000,18000,35999};
  for(unsigned h=0;h<sizeof(headings)/sizeof(headings[0]);++h) {
    setup();
    MissionTask_StartVisionSearchRecovery(6000,MISSION_STATE_S7_SEARCH_RED,0,false);
    mission_snapshot.yaw_cdeg=headings[h]; MissionTask_RunS3SearchTurn(6800);
    assert(s7_red_e3_origin_yaw_cdeg==headings[h]);
    assert(s3_search_yaw_target_cdeg==MissionTask_WrapYaw(headings[h]+6000));
    mission_snapshot.yaw_cdeg=s3_search_yaw_target_cdeg; MissionTask_RunS3SearchTurn(6820);
    MissionTask_RunS3SearchTurn(7320);
    assert(s3_search_yaw_target_cdeg==MissionTask_WrapYaw(headings[h]-6000));
    mission_snapshot.yaw_cdeg=s3_search_yaw_target_cdeg; MissionTask_RunS3SearchTurn(7340);
    MissionTask_RunS3SearchTurn(7840);
    assert(s3_search_yaw_target_cdeg==headings[h]);
    mission_snapshot.yaw_cdeg=headings[h]; MissionTask_RunS3SearchTurn(7860);
    assert(tx13==1 && mission_snapshot.state==MISSION_STATE_S7_WAIT_SECOND_E3);
  }

  // Fresh red coordinates interrupt the advance; recovery stays consumed.
  setup();
  MissionTask_StartVisionSearchRecovery(2000,MISSION_STATE_S7_SEARCH_RED,35000,false);
  fresh_red(); MissionTask_RunS3SearchTurn(2020);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s7_red_recovery_used);
  assert(mission_snapshot.target_sequence==123 && !clears && !tx13);
  mission_snapshot.vision_target_valid=false;
  MissionTask_StartVisionSearchRecovery(3000,MISSION_STATE_S7_SEARCH_RED,35000,false);
  assert(switches==1 && s3_target_select_command==MAIXCAM_COMMAND_SELECT_BLACK_GREEN);

  // Wrong class must not interrupt the first red recovery.
  setup(); MissionTask_StartVisionSearchRecovery(4000,MISSION_STATE_S7_SEARCH_RED,35000,false);
  mission_snapshot.vision_target_valid=true; mission_snapshot.target_id=6;
  MissionTask_RunS3SearchTurn(4020);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  assert(mission_snapshot.left_target_rpm==80);

  // Generic03, initial green E3, and04 are not changed by the red-only setting.
  setup(); mission_snapshot.state=MISSION_STATE_S3_TRACK_GREEN;
  s3_target_select_command=MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
  MissionTask_StartVisionSearchRecovery(5000,MISSION_STATE_S3_TRACK_GREEN,35000,false);
  MissionTask_RunS3SearchTurn(5020);
  assert(mission_snapshot.left_target_rpm==-80 && mission_snapshot.right_target_rpm==-80);
  assert(vision_search_forward_ms==800 && !s7_red_recovery_used);
  setup(); mission_snapshot.state=MISSION_STATE_S3_TRACK_GREEN;
  s7_target_switch_enabled=false; s3_target_select_command=MAIXCAM_COMMAND_SELECT_GREEN;
  MissionTask_StartVisionSearchRecovery(6000,MISSION_STATE_S3_TRACK_GREEN,35000,true);
  assert(vision_search_green_e3_mode && !vision_search_e3_mode);
  assert(vision_search_forward_rpm==80 && vision_search_forward_ms==800);
  setup(); mission_snapshot.state=MISSION_STATE_S4_TRACK_CENTER;
  MissionTask_StartVisionSearchRecovery(7000,MISSION_STATE_S4_TRACK_CENTER,35000,false);
  MissionTask_RunS3SearchTurn(7020);
  assert(!vision_search_e3_mode && mission_snapshot.left_target_rpm==60);
  assert(vision_search_forward_ms==500);

  // Fixed-heading legs keep separate2s soft deadlines, not a20s spin fault.
  setup(); MissionTask_StartVisionSearchRecovery(8000,MISSION_STATE_S7_SEARCH_RED,35000,false);
  MissionTask_RunS3SearchTurn(8800);
  MissionTask_RunS3SearchTurn(10799); assert(s7_red_e3_phase==S7_RED_E3_TURN_LEFT);
  MissionTask_RunS3SearchTurn(10800); assert(s7_red_e3_phase==S7_RED_E3_HOLD_LEFT);
  MissionTask_RunS3SearchTurn(11299); assert(s7_red_e3_phase==S7_RED_E3_HOLD_LEFT);
  MissionTask_RunS3SearchTurn(11300); assert(s7_red_e3_phase==S7_RED_E3_TURN_RIGHT);
  MissionTask_RunS3SearchTurn(13299); assert(s7_red_e3_phase==S7_RED_E3_TURN_RIGHT);
  MissionTask_RunS3SearchTurn(13300); assert(s7_red_e3_phase==S7_RED_E3_HOLD_RIGHT);
  MissionTask_RunS3SearchTurn(13800); assert(s7_red_e3_phase==S7_RED_E3_TURN_HOME);
  mission_snapshot.yaw_cdeg=29000; // Home leg still has an actual heading error.
  MissionTask_RunS3SearchTurn(15799); assert(!tx13);
  MissionTask_RunS3SearchTurn(15800);
  assert(tx13==1 && mission_snapshot.state==MISSION_STATE_S7_WAIT_SECOND_E3 && !mission_snapshot.fault_flags);

  // Millisecond wrap cannot shorten either500ms hold.
  setup(); uint32_t wrap_start=UINT32_MAX-900U;
  uint32_t wrap_now=reach_phase(wrap_start,S7_RED_E3_HOLD_LEFT);
  MissionTask_RunS3SearchTurn(wrap_now+499U); assert(s7_red_e3_phase==S7_RED_E3_HOLD_LEFT);
  MissionTask_RunS3SearchTurn(wrap_now+500U); assert(s7_red_e3_phase==S7_RED_E3_TURN_RIGHT);
  // The final13 transmit failure still enters a stopped fault.
  setup(); uint32_t end=reach_phase(18000,S7_RED_E3_TURN_HOME); tx_status=HAL_ERROR;
  mission_snapshot.yaw_cdeg=s3_search_yaw_target_cdeg; MissionTask_RunS3SearchTurn(end+20);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags&MISSION_FAULT_VISION_TX));
  puts("PASS:after07 red E3 forward80/800 +fast L60 hold500 R120 hold500 L60 home; gyro/tick wrap, drift, duplicates, all-leg coordinate interruptions, TX13/budget/2s deadlines; other03/04/green unchanged");
}
'@
$testSource = Join-Path $outputDir 'verify_s7_e3_forward.c'
[IO.File]::WriteAllText($testSource,$preamble+"`n"+$globals+"`n"+$prototypes+"`n"+$stubs+"`n"+($functions -join "`n")+"`n"+$cases,[Text.UTF8Encoding]::new($false))
$testExe = Join-Path $outputDir 'verify_s7_e3_forward.exe'
& $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Wno-unused-variable "-I$projectRoot/tests/host" "-I$projectRoot/Task" "-I$projectRoot/maixcam" $testSource -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host assertions failed' }
