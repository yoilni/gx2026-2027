#include "flow_modules.h"
#include "robot_config.h"

#include <assert.h>
#include <stdio.h>

static int16_t left, right, effective_forward;
static int32_t last_heading;
static uint16_t last_baffle, last_claw;
static unsigned stops, turns, tracks, closes, baffle_stops;
static bool arrived, pivot_only, claw_waiting;

static void wheels(int16_t l, int16_t r) { left = l; right = r; }
static void stop(void) { ++stops; wheels(0, 0); }
static bool turn(uint32_t now, int32_t heading)
{ (void)now; ++turns; last_heading = heading; return arrived; }
static void straight(uint32_t now, int32_t heading, int16_t rpm, int16_t cap)
{
  (void)now; (void)cap; last_heading = heading;
  effective_forward = pivot_only ? 0 : rpm;
  wheels(pivot_only ? -10 : rpm, pivot_only ? 10 : rpm);
}
static HAL_StatusTypeDef baffle(uint16_t angle) { last_baffle = angle; return HAL_OK; }
static HAL_StatusTypeDef claw(uint16_t angle) { last_claw = angle; return HAL_OK; }
static HAL_StatusTypeDef close_claw(void) { ++closes; return claw(ROBOT_CLAW_CLOSED_DEG); }
static HAL_StatusTypeDef pwm_stop(void) { ++baffle_stops; return HAL_OK; }
static HAL_StatusTypeDef camera(void) { return HAL_OK; }
static uint8_t deferred(void) { return claw_waiting; }
static bool track(uint32_t now, bool *in_tolerance)
{ (void)now; assert(in_tolerance == NULL); ++tracks; return true; }
static bool never_expired(uint32_t now, int32_t target) { (void)now; (void)target; return false; }
static int16_t correction(void) { return 7; }

static const FlowMotion io = { .stop = stop, .wheels = wheels, .turn = turn, .straight = straight };

static void check_delivery(void)
{
  const DeliveryFlow_Io port = { .motion = io, .baffle = baffle, .claw = claw, .close_claw = close_claw };
  DeliveryFlow_State flow = {0};
  const uint32_t start = UINT32_MAX - 99U;
  arrived = true;
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_ALIGN, start, start, 100, 100, 0U,
      &effective_forward) == DELIVERY_ALIGNED);
  DeliveryFlow_ResetProgress(&flow, start);
  pivot_only = true;
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_LOCATE, start, start, 100, 100, 74U,
      &effective_forward) == DELIVERY_RUNNING);
  assert(flow.baffle_held && !flow.advancing && last_baffle == ROBOT_BAFFLE_DOWN_DEG);
  pivot_only = false;
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_LOCATE, 100U, start, 100, 100, 74U,
      &effective_forward) == DELIVERY_RUNNING);
  assert(flow.forward_ms == 0U && flow.advancing);
  const uint32_t located = 100U + ROBOT_S6_BORDER_LOCATE_MS;
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_LOCATE, located, start, 100, 100, 74U,
      &effective_forward) == DELIVERY_LOCATED);
  assert(last_claw == 74U && flow.baffle_held);
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_BACKOFF, located, located, 100, 100, 74U,
      &effective_forward) == DELIVERY_RUNNING);
  assert(left == -ROBOT_S6_BORDER_LOCATE_RPM && right == left);
  const uint32_t push = located + ROBOT_S6_BORDER_BACKOFF_MS;
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_BACKOFF, push, located, 100, 100, 74U,
      &effective_forward) == DELIVERY_BACKED_OFF);
  assert(last_baffle == ROBOT_S6_PUSH_BAFFLE_DEG);
  DeliveryFlow_ResetProgress(&flow, push);
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_PUSH, push, push, 100, 100, 74U,
      &effective_forward) == DELIVERY_RUNNING);
  const uint32_t placed = push + ROBOT_S6_FINAL_PUSH_MS;
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_PUSH, placed, push, 100, 100, 74U,
      &effective_forward) == DELIVERY_PLACED);
  assert(last_baffle == ROBOT_BAFFLE_UP_DEG && !flow.baffle_held);
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_REVERSE, placed + ROBOT_S6_REVERSE_DRIVE_MS,
      placed, 100, 100, 74U, &effective_forward) == DELIVERY_RETREATED);
  assert(last_claw == ROBOT_CLAW_CLOSED_DEG);

  flow.baffle_held = true;
  assert(DeliveryFlow_ReleasePwm(&flow, false, pwm_stop) && !flow.baffle_held && baffle_stops == 1U);
  assert(DeliveryFlow_ReleasePwm(&flow, false, pwm_stop) && baffle_stops == 1U);
  DeliveryFlow_ResetProgress(&flow, 100U);
  assert(DeliveryFlow_Step(&flow, &port, DELIVERY_LOCATE, 100U + ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS,
      100U, 100, 100, 73U, &effective_forward) == DELIVERY_FORCED_RETREAT);
  assert(last_claw == 73U && last_baffle == ROBOT_BAFFLE_UP_DEG && left == 0);
}

static void check_routes(void)
{
  arrived = true;
  assert(BarShiftFlow_Step(&io, BAR_SHIFT_ALIGN, 10U, 10U, 0, 9000, 0) == BAR_SHIFT_BACK);
  assert(BarShiftFlow_Step(&io, BAR_SHIFT_BACK, 10U, 10U, 0, 9000, 0) == BAR_SHIFT_BACK);
  assert(left == -ROBOT_S6_BAR_REVERSE_RPM && last_heading == 0);
  assert(BarShiftFlow_Step(&io, BAR_SHIFT_BACK, 10U + ROBOT_S6_BAR_REVERSE_MS,
      10U, 0, 9000, 0) == BAR_SHIFT_TURN_SIDE && left == 0);
  assert(BarShiftFlow_Step(&io, BAR_SHIFT_TURN_SIDE, 20U, 20U, 0, 9000, 0) == BAR_SHIFT_FORWARD);
  assert(BarShiftFlow_Step(&io, BAR_SHIFT_FACE_SAFE, 50U, 50U, 0, 9000, 0) == BAR_SHIFT_DONE);
  assert(ObstacleFlow_Step(&io, OBSTACLE_TURN_OUT, 10U, 10U, 4500, 0, 0) == OBSTACLE_FORWARD);
  assert(ObstacleFlow_Step(&io, OBSTACLE_FORWARD, 10U, 10U, 4500, 0, 0) == OBSTACLE_FORWARD);
  assert(left == ROBOT_S6_OBSTACLE_DRIVE_RPM && left == right);
  assert(ObstacleFlow_Step(&io, OBSTACLE_FORWARD, 10U + ROBOT_S6_OBSTACLE_FORWARD_MS,
      10U, 4500, 0, 0) == OBSTACLE_TURN_BACK);
  assert(RepositionFlow_StepSide(&io, SIDE_FORWARD, 210U, 10U, 200U, 9000, 0) == SIDE_FACE_SAFE);
  assert(RepositionFlow_StepCorner(&io, CORNER_TURN_AWAY, 10U, 10U, 18000, 9000, 0) == CORNER_FORWARD_AWAY);
  assert(RepositionFlow_StepCorner(&io, CORNER_FORWARD_AWAY, 10U + ROBOT_S6_CORNER_AWAY_FORWARD_MS,
      10U, 18000, 9000, 0) == CORNER_TURN_SIDE);
  assert(RepositionFlow_StepCorner(&io, CORNER_FACE_SAFE, 10U, 10U, 18000, 9000, 0) == CORNER_DONE);
}

static void check_capture_arrange_departure(void)
{
  ArrangeFlow_State arrange = {0};
  const ArrangeFlow_Io port = { .motion = io, .track = track };
  const ArrangeFlow_Profile profile = { .side_push = true, .casualty = true, .turn_left = true };
  assert(ArrangeFlow_Step(&arrange, &port, ARRANGE_TRACK, profile, ROBOT_S4_FINE42_TRACK_MS,
      0U, 0).next == ARRANGE_TURN);
  assert(ArrangeFlow_Step(&arrange, &port, ARRANGE_TURN, profile, 0U, 0U, 0).next == ARRANGE_PUSH);
  assert(ArrangeFlow_Step(&arrange, &port, ARRANGE_PUSH, profile, ROBOT_S4_FINE42_FORWARD_MS,
      0U, 0).next == ARRANGE_COUNTER);
  assert(ArrangeFlow_Step(&arrange, &port, ARRANGE_COUNTER, profile, 0U, 0U, 0).next == ARRANGE_DONE);
  int32_t target_yaw = 1000;
  assert(ArrangeFlow_StepScatter(&io, SCATTER_TURN_IN, 10U, 10U, 1000, &target_yaw) == SCATTER_FORWARD);
  assert(ArrangeFlow_StepScatter(&io, SCATTER_FORWARD, 10U + ROBOT_S5_E5_FORWARD_MS,
      10U, 1000, &target_yaw) == SCATTER_COUNTER);
  assert(ArrangeFlow_StepScatter(&io, SCATTER_COUNTER, 20U, 20U, target_yaw, &target_yaw) == SCATTER_FINISH);
  assert(CaptureFlow_Open(camera, claw, 64U) == HAL_OK && last_claw == 64U);
  claw_waiting = true;
  assert(CaptureFlow_Close(stop, deferred, close_claw) == CAPTURE_RUNNING && last_claw == 64U);
  claw_waiting = false;
  assert(CaptureFlow_Close(stop, deferred, close_claw) == CAPTURE_READY && last_claw == ROBOT_CLAW_CLOSED_DEG);
  uint32_t missing = 0U;
  assert(CaptureFlow_Track(199U, 100U, true, &missing, track) == CAPTURE_TRACK_ACTIVE);
  const unsigned before = tracks;
  assert(CaptureFlow_Track(100U + ROBOT_S4_FINAL_TRACK_RETRY_MS, 100U, true, &missing, track) == CAPTURE_TRACK_TIMEOUT);
  assert(tracks == before);
  assert(DepartureFlow_OpenClaw(stop, claw) == HAL_OK && last_claw == ROBOT_CLAW_MATERIAL_OPEN_DEG);
  assert(DepartureFlow_LiftAfterClaw(100U + ROBOT_S1_CLAW_OPEN_SETTLE_MS - 1U, 100U,
      stop, baffle) == DEPARTURE_WAITING);
  assert(DepartureFlow_LiftAfterClaw(100U + ROBOT_S1_CLAW_OPEN_SETTLE_MS, 100U,
      stop, baffle) == DEPARTURE_READY && last_baffle == ROBOT_BAFFLE_UP_DEG);
  bool finished = false;
  uint32_t down = 0U;
  assert(DepartureFlow_CrossBump(&io, baffle, close_claw, correction, 100U, 100U,
      &finished, &down).status == DEPARTURE_DRIVING);
  assert(left == ROBOT_S2_CROSS_BUMP_RPM - 7 && right == ROBOT_S2_CROSS_BUMP_RPM);
  const uint32_t end = 100U + ROBOT_S2_CROSS_BUMP_DRIVE_MS;
  assert(DepartureFlow_CrossBump(&io, baffle, close_claw, correction, end, 100U,
      &finished, &down).just_lowered && last_baffle == ROBOT_BAFFLE_DOWN_DEG);
  assert(DepartureFlow_CrossBump(&io, baffle, close_claw, correction, end + ROBOT_S2_CROSS_BUMP_SETTLE_MS,
      100U, &finished, &down).status == DEPARTURE_READY && last_claw == ROBOT_CLAW_CLOSED_DEG);
}

static void check_recovery(void)
{
  int32_t origin = 0, target = 0, error = 0, previous = 35900, progress = 0;
  int16_t turn_cmd = 0;
  RecoveryFlow_ObserveSpin(&previous, &progress, 100, 1);
  assert(previous == 100 && progress == 200);
  assert(RecoveryFlow_StepRed(&io, RECOVERY_RED_FORWARD, never_expired, 150U, 100U,
      50U, 80, 100, &origin, &target, &turn_cmd, &error) == RECOVERY_RED_LEFT);
  assert(RecoveryFlow_StepRed(&io, RECOVERY_RED_LEFT, never_expired, 200U, 150U,
      50U, 80, target, &origin, &target, &turn_cmd, &error) == RECOVERY_RED_HOLD_LEFT);
  assert(RecoveryFlow_StepRed(&io, RECOVERY_RED_HOLD_LEFT, never_expired, 200U + ROBOT_S7_E3_SEARCH_HOLD_MS,
      200U, 50U, 80, target, &origin, &target, &turn_cmd, &error) == RECOVERY_RED_RIGHT);
  assert(RecoveryFlow_StepZone(&io, ZONE_RECOVERY_FIRST_DRIVE, 10U + ROBOT_S6_RECOVERY_FIRST_FORWARD_MS,
      10U, 4500, 31500, 0) == ZONE_RECOVERY_FACE);
  assert(RecoveryFlow_StepScan(&io, RECOVERY_CAPTURE_LOSS, RECOVERY_SCAN_BACK,
      10U + ROBOT_S4_FINAL_RECOVERY_REVERSE_MS, 10U, 0) == RECOVERY_SCAN_FIRST);
}

int main(void)
{
  check_delivery();
  check_routes();
  check_capture_arrange_departure();
  check_recovery();
  puts("PASS: eight flow modules run without Mission or vision protocol; physical sequences and tick boundaries checked");
  return 0;
}
