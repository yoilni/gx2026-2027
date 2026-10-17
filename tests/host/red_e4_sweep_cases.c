/* Real UART RX, red angle PID and E4 handlers; legacy turn stubs stay intact. */
static uint32_t red_e4_reach_phase(int phase)
{
  setup(0x11);
  s7_red_recovery_used = true; /* E4 must not reset or consume E3's allowance. */
  MissionTask_StartS4E4Recovery(120);
  assert(s4_e4_recovery_used && s4_red_e4_align_away_active);
  assert(s4_e4_away_yaw_cdeg == 35000);
  if (phase < 0) return 120;
  mission_snapshot.yaw_cdeg = s4_e4_away_yaw_cdeg;
  tick(140);
  assert(!s4_red_e4_align_away_active && vision_search_forward_active);
  if (phase == S7_RED_E3_FORWARD) return 140;
  tick(940);
  if (phase == S7_RED_E3_TURN_LEFT) return 940;
  mission_snapshot.yaw_cdeg = s3_search_yaw_target_cdeg;
  tick(960);
  if (phase == S7_RED_E3_HOLD_LEFT) return 960;
  tick(1460);
  if (phase == S7_RED_E3_TURN_RIGHT) return 1460;
  mission_snapshot.yaw_cdeg = s3_search_yaw_target_cdeg;
  tick(1480);
  if (phase == S7_RED_E3_HOLD_RIGHT) return 1480;
  tick(1980);
  assert(phase == S7_RED_E3_TURN_HOME);
  return 1980;
}

static void verify_red_e4_sweep(void)
{
  // Complete trajectory: mapped away first, then the same red E3 motion.
  uint32_t now = red_e4_reach_phase(S7_RED_E3_FORWARD);
  assert(now == 140 && mission_snapshot.state_entry_tick == 140);
  tick(160);
  assert(mission_snapshot.left_target_rpm == 80 && mission_snapshot.right_target_rpm == 80);
  fake_now = 620; rx(0xE4); tick(620);
  assert(mission_snapshot.state_entry_tick == 140 && !tx_count);
  tick(939);
  assert(s7_red_e3_phase == S7_RED_E3_FORWARD && mission_snapshot.left_target_rpm == 80);
  tick(940);
  assert(s7_red_e3_phase == S7_RED_E3_TURN_LEFT && s3_search_yaw_target_cdeg == 5000);
  assert(mission_snapshot.state == MISSION_STATE_S4_E4_RED_SPIN_360);
  tick(960);
  assert(mission_snapshot.left_target_rpm == -45 && mission_snapshot.right_target_rpm == 45);
  mission_snapshot.yaw_cdeg = 4200; tick(980);
  assert(mission_snapshot.left_target_rpm == -15);
  mission_snapshot.yaw_cdeg = 5000; tick(1000);
  assert(s7_red_e3_phase == S7_RED_E3_HOLD_LEFT && !mission_snapshot.left_target_rpm);
  tick(1499); assert(s7_red_e3_phase == S7_RED_E3_HOLD_LEFT);
  tick(1500);
  assert(s7_red_e3_phase == S7_RED_E3_TURN_RIGHT && s3_search_yaw_target_cdeg == 29000);
  tick(1520); assert(mission_snapshot.left_target_rpm == 45);
  mission_snapshot.yaw_cdeg = 29000; tick(1540);
  assert(s7_red_e3_phase == S7_RED_E3_HOLD_RIGHT && !mission_snapshot.left_target_rpm);
  tick(2039); assert(s7_red_e3_phase == S7_RED_E3_HOLD_RIGHT);
  tick(2040);
  assert(s7_red_e3_phase == S7_RED_E3_TURN_HOME && s3_search_yaw_target_cdeg == 35000);
  tick(2060); assert(mission_snapshot.left_target_rpm == -45);
  mission_snapshot.yaw_cdeg = 35000; tick(2080);
  assert(tx_count == 2 && tx_commands[0] == 0x51 && tx_commands[1] == 0x03);
  assert(s3_target_select_command == 0x51 && !mission_snapshot.left_target_rpm);

  // All phases accept fresh ID4, retain the consumed allowance and send04.
  for (int p = -1; p <= S7_RED_E3_TURN_HOME; ++p)
  {
    now = red_e4_reach_phase(p);
    uint32_t entry = mission_snapshot.state_entry_tick;
    fake_now = now + 20; rx(0xE4); tick(now + 20);
    assert(mission_snapshot.state_entry_tick == entry && !tx_count);
    fake_now = now + 40; coords(4); tick(now + 40);
    assert(mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER);
    assert(s4_e4_recovery_used && s7_red_recovery_used);
    assert(!s4_red_e4_align_away_active && !vision_search_forward_active);
    assert(tx_count == 1 && tx_commands[0] == 0x04);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    MissionTask_StartS4E4Recovery(now + 60);
    assert(tx_count == 3 && tx_commands[1] == 0x51 && tx_commands[2] == 0x03);

    now = red_e4_reach_phase(p);
    fake_now = now + 20; coords(6); tick(now + 20);
    assert(mission_snapshot.state == MISSION_STATE_S4_E4_RED_SPIN_360 && !tx_count);
    fake_now = now + 40; coords(4); tick(now + 541); /* 501ms old. */
    assert(mission_snapshot.state == MISSION_STATE_S4_E4_RED_SPIN_360 && !tx_count);
    fake_now = now + 561; coords(4); tick(now + 561);
    assert(mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER && tx_commands[0] == 0x04);

    now = red_e4_reach_phase(p);
    health_faults = MISSION_FAULT_IMU; tick(now + 20);
    assert(mission_snapshot.state == MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  }

  // One-shot phase messages interrupt each leg and survive the return04 sync.
  const uint8_t stage_messages[] = {0x14, 0x02, 0x24};
  for (int p = -1; p <= S7_RED_E3_TURN_HOME; ++p)
  {
    for (unsigned e = 0; e < sizeof(stage_messages); ++e)
    {
      now = red_e4_reach_phase(p);
      fake_now = now + 20; rx(stage_messages[e]); rx(0xE4); tick(now + 20);
      assert(tx_count >= 1 && tx_commands[0] == 0x04);
      assert(mission_snapshot.state != MISSION_STATE_S4_E4_RED_SPIN_360);
    }
  }

  // Every start zone/team uses the already-converted safety heading +180.
  for (uint8_t zone = 1; zone <= 4; ++zone)
  {
    for (uint8_t team = ROBOT_TEAM_RED; team <= ROBOT_TEAM_BLUE; ++team)
    {
      setup(0x11);
      mission_snapshot.start_zone = zone; mission_snapshot.team = team;
      mission_snapshot.yaw_start_cdeg = 12300;
      int32_t offset = ((zone <= 2 && team == ROBOT_TEAM_BLUE) ||
                        (zone >= 3 && team == ROBOT_TEAM_RED)) ? 0 : 18000;
      mission_snapshot.safe_zone_yaw_target_cdeg = MissionTask_WrapYaw(12300 + offset);
      MissionTask_StartS4E4Recovery(120);
      assert(s4_e4_away_yaw_cdeg == MissionTask_WrapYaw(12300 + offset + 18000));
      tick(140); assert(selected_turn_limit == 45 && s4_red_e4_align_away_active);
    }
  }

  // Production angle PID commands the wrapped shortest turn, within45rpm.
  setup(0x11); MissionTask_StartS4E4Recovery(120);
  mission_snapshot.yaw_cdeg = MissionTask_WrapYaw(s4_e4_away_yaw_cdeg - 12000);
  tick(140);
  assert(mission_snapshot.left_target_rpm == -45 && mission_snapshot.right_target_rpm == 45);
  mission_snapshot.yaw_cdeg = MissionTask_WrapYaw(s4_e4_away_yaw_cdeg + 12000);
  tick(160);
  assert(mission_snapshot.left_target_rpm == 45 && mission_snapshot.right_target_rpm == -45);

  // Away and all three sweep turns each own2s; holds consume none of it.
  setup(0x11); MissionTask_StartS4E4Recovery(120);
  tick(2119); assert(s4_red_e4_align_away_active && !vision_search_forward_active);
  tick(2120); assert(!s4_red_e4_align_away_active && mission_snapshot.state_entry_tick == 2120);
  tick(2140); assert(mission_snapshot.left_target_rpm == 80);
  tick(2920); assert(s7_red_e3_phase == S7_RED_E3_TURN_LEFT);
  tick(4919); assert(s7_red_e3_phase == S7_RED_E3_TURN_LEFT);
  tick(4920); assert(s7_red_e3_phase == S7_RED_E3_HOLD_LEFT);
  tick(5420); assert(s7_red_e3_phase == S7_RED_E3_TURN_RIGHT);
  tick(7419); assert(s7_red_e3_phase == S7_RED_E3_TURN_RIGHT);
  tick(7420); assert(s7_red_e3_phase == S7_RED_E3_HOLD_RIGHT);
  tick(7920); assert(s7_red_e3_phase == S7_RED_E3_TURN_HOME);
  mission_snapshot.yaw_cdeg = 10000; /* Still not home after the mocked stall. */
  tick(9919); assert(!tx_count);
  tick(9920); assert(tx_count == 2 && !mission_snapshot.fault_flags);

  // Final switch failures stop; E4 never transmits E3's13 or green44.
  now = red_e4_reach_phase(S7_RED_E3_TURN_HOME);
  frame_lower_ok = false; mission_snapshot.yaw_cdeg = s3_search_yaw_target_cdeg;
  tick(now + 20);
  assert(mission_snapshot.state == MISSION_STATE_FAULT && !tx_count);
  now = red_e4_reach_phase(S7_RED_E3_TURN_HOME);
  tx_ok = false; mission_snapshot.yaw_cdeg = s3_search_yaw_target_cdeg;
  tick(now + 20);
  assert(mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 1 && tx_commands[0] == 0x51);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  puts("PASS:after07 red E4 mapped-away turn +shared80/800 L60/500 R120/500 L60; all8 headings, all-phase fresh/stale/ID/duplicate/event gates,04 sync/budget, per-leg2s and failure safety; miss TX51/03 only");
}
