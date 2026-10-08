param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/s4_motion_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$source = Get-Content (Join-Path $projectRoot 'Task/mission_task.c') -Raw
function Extract-Function([string]$name) {
  $match = [regex]::Match($source, '(?m)^static [^\r\n]+\b' + $name + '\([^;]*?\)\r?\n\{')
  if (-not $match.Success) { throw "Missing function $name" }
  $depth = 1
  $cursor = $match.Index + $match.Length
  while ($depth -gt 0) {
    if ($source[$cursor] -eq '{') { ++$depth }
    if ($source[$cursor] -eq '}') { --$depth }
    ++$cursor
  }
  $source.Substring($match.Index, $cursor - $match.Index)
}
$names = @('MissionTask_S4SmoothStep', 'MissionTask_S4MotionScale',
  'MissionTask_CalculateS4CornerProfile', 'MissionTask_S4ScalePushDuration',
  'MissionTask_S4PushDuration', 'MissionTask_S4ReverseDuration',
  'MissionTask_CommandS4CornerReverse')
$functions = @($names | ForEach-Object { Extract-Function $_ })
$preamble = @'
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "mission_task.h"
#include "robot_config.h"
static MissionSnapshot mission_snapshot;
static uint32_t s4_next_debug_tick;
static int16_t wheel_left, wheel_right;
static bool DebugUart_Logf(const char *format, ...) { (void)format; return true; }
static void MissionTask_SetWheelTargets(int16_t left, int16_t right) {
  wheel_left = left; wheel_right = right;
}
/* The pre-adjustment differential is the reference: retain center travel and
   durations while reducing right curvature and slightly increasing left. */
static uint32_t baseline_duration(bool right) {
  uint32_t duration = right ? ROBOT_S4_RIGHT_CORNER_FOLLOW_MS : ROBOT_S4_LEFT_CORNER_FOLLOW_MS;
  float area = 70.0f * ((float)duration - ROBOT_S4_PUSH_RAMP_MS);
  if (!right) area -= 10.0f * (ROBOT_S4_LEFT_CORNER_INITIAL_MS +
      ROBOT_S4_LEFT_SPEED_BLEND_MS * 0.5f - ROBOT_S4_PUSH_RAMP_MS * 0.5f);
  area *= right ? 1.45f * 1.30f : 1.45f;
  if (!right) area += 10.0f * (ROBOT_S4_LEFT_CORNER_INITIAL_MS +
      ROBOT_S4_LEFT_SPEED_BLEND_MS * 0.5f - ROBOT_S4_PUSH_RAMP_MS * 0.5f);
  return (uint32_t)(area / 90.0f + ROBOT_S4_PUSH_RAMP_MS + 0.5f);
}
'@
$baseline = @'
static void baseline_profile(uint32_t elapsed, uint32_t duration, bool right,
                             int16_t *left, int16_t *right_wheel) {
  float scale = MissionTask_S4MotionScale(elapsed, duration, ROBOT_S4_PUSH_RAMP_MS);
  float speed = 90.0f;
  if (!right) {
    float blend = elapsed <= ROBOT_S4_LEFT_CORNER_INITIAL_MS ? 0.0f :
        MissionTask_S4SmoothStep((float)(elapsed - ROBOT_S4_LEFT_CORNER_INITIAL_MS) /
                                ROBOT_S4_LEFT_SPEED_BLEND_MS);
    speed = 80.0f + 10.0f * blend;
  }
  float progress = (float)elapsed / duration;
  float middle = MissionTask_S4SmoothStep((progress - ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN) /
      (ROBOT_S4_CORNER_DIFF_MIDDLE_END - ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN));
  float late = MissionTask_S4SmoothStep((progress - ROBOT_S4_CORNER_DIFF_LATE_BEGIN) /
      (ROBOT_S4_CORNER_DIFF_LATE_END - ROBOT_S4_CORNER_DIFF_LATE_BEGIN));
  float early_diff = right ? 33.0f : 11.0f;
  float middle_diff = right ? 41.0f : 12.0f;
  float late_diff = right ? 48.0f : 14.0f;
  int16_t forward = (int16_t)(speed * scale + 0.5f);
  int16_t diff = (int16_t)((early_diff + (middle_diff - early_diff) * middle +
                          (late_diff - middle_diff) * late) * scale + 0.5f);
  int16_t faster = forward + (diff + 1) / 2;
  int16_t slower = forward - diff / 2;
  *left = right ? faster : slower;
  *right_wheel = right ? slower : faster;
}
'@
$cases = @'
static double relative_error(double value, double baseline) {
  return fabs(value / baseline - 1.0);
}
static void verify(bool right) {
  uint32_t old_push = baseline_duration(right);
  uint32_t push = MissionTask_S4PushDuration(right);
  uint32_t reverse = MissionTask_S4ReverseDuration(right);
  float return_fraction = right ? 1.0f : ROBOT_S4_LEFT_REVERSE_DISTANCE_PERCENT / 100.0f;
  uint32_t old_reverse = (uint32_t)(old_push * 90.0f / 120.0f * return_fraction + 0.5f);
  double old_area = 0, old_turn = 0, area = 0, turn = 0, reverse_area = 0;
  double old_reverse_area = 0;
  double peak_forward = 0, peak_reverse = 0;
  int16_t left, r;
  assert(ROBOT_S4_CORNER_TRAVEL_PERCENT == 100);
  assert(push == old_push && reverse == old_reverse);
  assert(push == (right ? 643U : 1014U));
  assert(reverse == (right ? 482U : 608U));
  assert(ROBOT_S4_CORNER_DIFF_EARLY_RPM == 30 &&
         ROBOT_S4_CORNER_DIFF_MIDDLE_RPM == 38 &&
         ROBOT_S4_CORNER_DIFF_LATE_RPM == 45);
  assert(ROBOT_S4_LEFT_DIFF_EARLY_RPM == 13 &&
         ROBOT_S4_LEFT_DIFF_MIDDLE_RPM == 14 &&
         ROBOT_S4_LEFT_DIFF_LATE_RPM == 16);
  for (uint32_t t = 0; t < old_push; ++t) {
    baseline_profile(t, old_push, right, &left, &r);
    old_area += (left + r) * 0.5;
    old_turn += right ? left - r : r - left;
  }
  for (uint32_t t = 0; t < old_reverse; ++t) {
    uint32_t replay = (uint32_t)(((uint64_t)(old_reverse - t) * old_push + old_reverse / 2U) / old_reverse);
    baseline_profile(replay, old_push, right, &left, &r);
    float speed_scale = (float)old_push / old_reverse * return_fraction;
    old_reverse_area += ((int16_t)(left * speed_scale + 0.5f) +
                         (int16_t)(r * speed_scale + 0.5f)) * 0.5;
  }
  for (uint32_t t = 0; t <= push; ++t) {
    MissionTask_CalculateS4CornerProfile(t, push, right, &left, &r);
    assert(left >= 0 && r >= 0);
    assert(right ? left >= r : r >= left);
    if (t == 0 || t == push) assert(left == 0 && r == 0);
    area += (left + r) * 0.5;
    turn += right ? left - r : r - left;
    if ((left + r) * 0.5 > peak_forward) peak_forward = (left + r) * 0.5;
  }
  mission_snapshot.state_entry_tick = 10000;
  s4_next_debug_tick = 1000000;
  for (uint32_t t = 0; t <= reverse; ++t) {
    MissionTask_CommandS4CornerReverse(10000 + t, right);
    assert(wheel_left <= 0 && wheel_right <= 0);
    assert(right ? wheel_left <= wheel_right : wheel_right <= wheel_left);
    if (t == 0 || t == reverse) assert(wheel_left == 0 && wheel_right == 0);
    double speed = -(wheel_left + wheel_right) * 0.5;
    reverse_area += speed;
    if (speed > peak_reverse) peak_reverse = speed;
  }
  assert(relative_error(area, old_area) < 0.01);
  assert(relative_error(reverse_area, old_reverse_area) < 0.01);
  assert(relative_error(reverse_area, area * return_fraction) < 0.01);
  assert(peak_forward >= 89.0 && peak_reverse >= 119.0);
  double curvature_ratio = (turn / area) / (old_turn / old_area);
  assert(right ? curvature_ratio > 0.90 && curvature_ratio < 0.96
               : curvature_ratio > 1.10 && curvature_ratio < 1.20);
  printf("PASS %s: forward %u -> %ums, back %u -> %ums, distance %.2f%%/%.2f%%, curvature %.1f%%, total turn %.1f%%\n",
      right ? "02" : "12", old_push, push, old_reverse, reverse,
      area / old_area * 100.0, reverse_area / old_reverse_area * 100.0,
      curvature_ratio * 100.0, turn / old_turn * 100.0);
}
int main(void) {
  verify(true);
  verify(false);
  return 0;
}
'@
$code = $preamble + "`n" + ($functions -join "`n") + "`n" + $baseline + "`n" + $cases
$executable = Join-Path $outputDir 's4_motion_cases.exe'
$code | & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-x' 'c' '-' `
    '-I' (Join-Path $projectRoot 'Task') '-o' $executable '-lm'
if ($LASTEXITCODE -ne 0) { throw 'S4 motion host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'S4 motion distance/curve checks failed' }
