/* Real parser + TURN180 +07 handlers: motion completion is not a protocol reset. */
static void exit180_setup(void) {
  setup(3,ROBOT_TEAM_BLUE,0);
  s3_target_select_command=0x11;
  s7_red_recovery_used=false;
  s6_exit_yaw_target_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+18000);
  MissionTask_ResetS6ControllerState(); s6_phase_stable_since=0;
  MissionTask_EnterState(MISSION_STATE_S6_EXIT_TURN_180,100);
  frame_down=false; // Simulate the raised frame left by final26 delivery.
}
static void exit180_tick(uint32_t now) {
  fake_now=now; MissionTask_UpdateVisionInput(now);
  MissionTask_CheckRunningHealth(now);
  MissionTask_RunS6ExitTurn180(now); MissionTask_RunS7Decision(now);
}
static void verify_exit180_events(void) {
  //An E3 received during the turn survives both normal alignment and timeout.
  for(unsigned aligned=0;aligned<2;++aligned) {
    exit180_setup(); fake_now=200; rx(0xE3);
    if(aligned) mission_snapshot.yaw_cdeg=s6_exit_yaw_target_cdeg;
    exit180_tick(200); assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180);
    assert(!frame_down);
    assert(maixcam_has_event && maixcam_event_code==0xE3);
    uint32_t done=aligned?200U+ROBOT_S6_TURN_STABLE_MS:100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
    exit180_tick(done);
    assert(frame_down); //Normal arrival AND2s fallback lower without coordinates.
    assert(s7_red_recovery_used && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
    assert(vision_search_resume_state==MISSION_STATE_S7_SEARCH_RED && vision_search_e3_mode);
    assert(vision_search_forward_rpm==80 && vision_search_forward_ms==800);
    assert(tx_count==2 && s3_target_select_command==0x11); //No silent51/03 switch.
    fake_now=done+20; MissionTask_RunS3SearchTurn(fake_now);
    assert(mission_snapshot.left_target_rpm==80 && mission_snapshot.right_target_rpm==80);
  }
  //Packet may straddle the turn-complete boundary; parsing must keep running.
  exit180_setup(); fake_now=200; byte(0xF1); byte(0xF2); byte(0xE3);
  exit180_tick(100U+ROBOT_HEADING_TURN_TIMEOUT_MS);
  assert(frame_down);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && !s7_red_recovery_used);
  fake_now+=20; byte(0x1F); byte(0x2F); exit180_tick(fake_now);
  assert(s7_red_recovery_used && vision_search_forward_rpm==80);
  //Same-task fresh red coordinates win over E3; preserve04 for the03 handler.
  for(unsigned cmd=0;cmd<2;++cmd) {
    exit180_setup(); fake_now=200; rx(cmd?0x04:0xE3); coords(4);
    exit180_tick(200);
    assert(frame_down);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !s7_red_recovery_used);
    assert(maixcam_has_event && maixcam_event_code==(cmd?0x04:0xE3));
    bool handled=MissionTask_TryStartS3Capture(220);
    assert(handled==(cmd!=0));
    assert(mission_snapshot.state==(cmd?MISSION_STATE_S4_TRACK_CENTER:MISSION_STATE_S3_TRACK_GREEN));
    assert(tx_count==2);
  }
  //No E3 or coordinates really is silence: the original2s policy is unchanged.
  exit180_setup(); uint32_t done=100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
  exit180_tick(done); assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED);
  exit180_tick(done+ROBOT_S7_SILENT_TIMEOUT_MS-1U);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && tx_count==2);
  exit180_tick(done+ROBOT_S7_SILENT_TIMEOUT_MS);
  assert(s3_target_select_command==0x51 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  assert(tx_count==4 && tx_commands[2]==0x51 && tx_commands[3]==0x03);
  //Safety wins even with E3 pending; no search motion can restart from FAULT.
  exit180_setup(); fake_now=200; rx(0xE3); health_faults=MISSION_FAULT_IMU;
  exit180_tick(200); assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  assert(!frame_down); //Health fault does not execute the normal turn-end action.
  puts("PASS:TURN180 preserves pending E3/04 and partial frames, normal/timeout turn completion, first red recovery/fresh coordinate priority, genuine2s silence and safety");
}
