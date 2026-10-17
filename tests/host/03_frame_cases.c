/* Real03 initializer,07 entry and final26 reverse; actuator spy, real parser. */
static void verify_03_frame_down(void)
{
  const uint8_t targets[] = {0x21, 0x11, 0x31, 0x51};
  for (unsigned i = 0; i < sizeof(targets); ++i)
  {
    setup(targets[i]);
    assert(!frame_down && !mission_snapshot.vision_target_valid);
    assert(MissionTask_BeginS3Vision(120));
    assert(frame_down && frame_lower_calls == 1);
    assert(mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN);
    assert(tx_count == 2 && tx_commands[0] == targets[i] && tx_commands[1] == 0x03);
    assert(!mission_snapshot.vision_target_valid); // No coordinate is needed.
  }

  //07 lowers before11/03, which is now sent only after the exit180 motion ends.
  setup(0x51); MissionTask_StartS7Decision(120);
  assert(frame_down && frame_lower_calls == 1);
  assert(tx_count == 2 && tx_commands[0] == 0x11 && tx_commands[1] == 0x03);
  assert(mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED);

  //26 keeps the fully raised frame during the300ms reverse AND the180 turn.
  assert(ROBOT_S6_REVERSE_DRIVE_MS == 300 && ROBOT_S6_REVERSE_RPM == 100);
  setup(0x21); MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE, 100);
  s6_push_yaw_target_cdeg = 17000;
  fake_now = 399; MissionTask_RunS6FinalReverse(fake_now);
  assert(!frame_down && tx_count == 0);
  assert(mission_snapshot.state == MISSION_STATE_S6_FINAL_REVERSE);
  assert(mission_snapshot.left_target_rpm == -100);
  fake_now = 400; MissionTask_RunS6FinalReverse(fake_now);
  assert(!frame_down && frame_lower_calls == 0);
  assert(tx_count == 0); //No search request before/during the rotation.
  assert(mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180);
  assert(s6_exit_yaw_target_cdeg == MissionTask_WrapYaw(17000 + ROBOT_YAW_LEFT_SIGN * 18000));
  fake_now = 420; MissionTask_RunS6ExitTurn180(fake_now);
  assert(!frame_down && frame_lower_calls == 0);
  assert(mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180);
  fake_now = 430; coords(4); MissionTask_UpdateVisionInput(fake_now);
  MissionTask_RunS6ExitTurn180(fake_now);
  assert(!mission_snapshot.vision_target_valid);
  assert(!frame_down && tx_count == 0);
  assert(mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180);
  fake_now = 440; rx(0xE3); turn_aligned = true;
  MissionTask_RunS6ExitTurn180(fake_now);
  assert(frame_down && frame_lower_calls == 1);
  assert(mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED && tx_count == 2);
  assert(tx_commands[0] == 0x11 && tx_commands[1] == 0x03);
  uint8_t event;
  assert(!MaixCam_PeekEvent(&event)); //Old motion-era input fenced BEFORE new TX.

  //Coordinates cannot interrupt an unfinished turn or lower the frame early.
  setup(0x21); MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE, 100);
  fake_now = 400; MissionTask_RunS6FinalReverse(fake_now);
  fake_now = 420; coords(4); MissionTask_UpdateVisionInput(fake_now);
  MissionTask_RunS6ExitTurn180(fake_now);
  assert(!frame_down && frame_lower_calls == 0 && tx_count == 0);
  assert(!mission_snapshot.vision_target_valid);
  assert(mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180);
  turn_aligned = true; fake_now = 440; MissionTask_RunS6ExitTurn180(fake_now);
  assert(frame_down && frame_lower_calls == 1 && tx_count == 2);
  assert(mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED && !mission_snapshot.vision_target_valid);

  // Actuator errors abort the03 handshake/exit turn and remain safely stopped.
  setup(0x51); frame_lower_ok = false;
  assert(!MissionTask_BeginS3Vision(120));
  assert(mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 0);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  setup(0x21); frame_lower_ok = false;
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE, 100);
  fake_now = 400; MissionTask_RunS6FinalReverse(fake_now);
  assert(mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180 && tx_count == 0);
  turn_aligned = true; fake_now = 420; MissionTask_RunS6ExitTurn180(fake_now);
  assert(mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 0);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  setup(0x21); MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE, 100);
  fake_now = 400; MissionTask_RunS6FinalReverse(fake_now);
  frame_lower_ok = false; fake_now = 420; coords(4); MissionTask_UpdateVisionInput(fake_now);
  MissionTask_RunS6ExitTurn180(fake_now);
  assert(mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180 && tx_count == 0);
  turn_aligned = true; fake_now = 440; MissionTask_RunS6ExitTurn180(fake_now);
  assert(mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 0);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  setup(0x21); MissionTask_EnterState(MISSION_STATE_S6_EXIT_TURN_180, 100);
  turn_aligned = true; tx_ok = false; fake_now = 420; MissionTask_RunS6ExitTurn180(fake_now);
  assert(frame_down && mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 1);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_VISION_TX);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  puts("PASS:ordinary03 lowers before search;07 keeps UP through300ms reverse+exit180, no premature11/03 or coordinate interruption, post-turn lowering/search and actuator/TX-failure safety");
}
