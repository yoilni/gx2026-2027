/* Real parser + TURN180 +07: request the next target only AFTER rotation. */
static void exit180_setup(void) {
  setup(3,ROBOT_TEAM_BLUE,0);
  tx_count=0; //Previous capture's commands are not part of the exit-turn test.
  s3_target_select_command=0x21;
  s7_target_switch_enabled=false;
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
  //Old red coordinates/E3/04 cannot interrupt, whether arriving at the goal or timeout.
  for(unsigned aligned=0;aligned<2;++aligned) {
    exit180_setup(); fake_now=200; rx(aligned?0x04:0xE3); coords(4);
    if(aligned) mission_snapshot.yaw_cdeg=s6_exit_yaw_target_cdeg;
    exit180_tick(200); assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180);
    assert(!frame_down && !mission_snapshot.vision_target_valid && tx_count==0);
    assert(maixcam_has_event); //Parser runs; these old replies are not consumed in motion.
    uint32_t done=aligned?200U+ROBOT_S6_TURN_STABLE_MS:100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
    exit180_tick(done);
    assert(frame_down && mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED);
    assert(!mission_snapshot.vision_target_valid && !s7_red_recovery_used && !maixcam_has_event);
    assert(tx_count==2 && tx_commands[0]==0x11 && tx_commands[1]==0x03);
    fake_now=done+20; coords(4); exit180_tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !s7_red_recovery_used);
    assert(tx_count==2); //New red coordinates resume without another11/03.
  }

  //Partial old short/coordinate frames must not become a new-task response after TX.
  for(unsigned coordinate=0;coordinate<2;++coordinate) {
    exit180_setup(); fake_now=200;
    byte(0xF1); byte(0xF2); byte(coordinate?4:0xE3);
    if(coordinate) { byte(0); byte(20); byte(1); byte(0); }
    else byte(0x1F);
    uint32_t done=100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
    exit180_tick(done);
    assert(frame_down && mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && tx_count==2);
    fake_now=done+20;
    if(coordinate) { byte(30); byte(2); byte(0x1F); }
    byte(0x2F); exit180_tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && !s7_red_recovery_used);
    assert(!mission_snapshot.vision_target_valid && !maixcam_has_event);
    coords(4); rx(0x04); exit180_tick(done+40);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !s7_red_recovery_used);
    assert(maixcam_has_event && maixcam_event_code==0x04);
    assert(MissionTask_TryStartS3Capture(done+60));
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && tx_count==2);
  }

  //Replies received inside the new03 TX are retained and refreshed before07 runs.
  exit180_setup(); inject_exit_on_tx=0x04;
  exit180_tick(100U+ROBOT_HEADING_TURN_TIMEOUT_MS);
  assert(frame_down && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==2);
  assert(maixcam_has_event && maixcam_event_code==0x04);
  assert(MissionTask_TryStartS3Capture(fake_now+20));
  assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && tx_count==2);
  exit180_setup(); inject_exit_on_tx=0xE3;
  uint32_t loss_done=100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
  exit180_tick(loss_done);
  assert(frame_down && s7_red_recovery_used && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  assert(vision_search_resume_state==MISSION_STATE_S7_SEARCH_RED && vision_search_e3_mode);
  assert(vision_search_forward_rpm==80 && vision_search_forward_ms==800 && tx_count==2);

  //Only new fresh ID4 coordinates can start the red task after rotation.
  exit180_setup(); uint32_t ready=100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
  exit180_tick(ready); fake_now=ready+20; coords(5); exit180_tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED);
  coords(4); exit180_tick(fake_now+501);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED);
  fake_now+=521; coords(4); exit180_tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==2);

  //No E3 or coordinates really is silence: the original2s policy is unchanged.
  exit180_setup(); uint32_t done=100U+ROBOT_HEADING_TURN_TIMEOUT_MS;
  exit180_tick(done); assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED);
  exit180_tick(done+ROBOT_S7_SILENT_TIMEOUT_MS-1U);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && tx_count==2);
  exit180_tick(done+ROBOT_S7_SILENT_TIMEOUT_MS);
  assert(s3_target_select_command==0x51 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  assert(tx_count==4 && tx_commands[2]==0x51 && tx_commands[3]==0x03);
  //The existing2s heading fallback still works across timer wrap; not a coordinate break.
  exit180_setup(); uint32_t start=UINT32_MAX-1000U;
  mission_snapshot.state_entry_tick=start;
  exit180_tick(start+1999U);
  assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180 && tx_count==0);
  exit180_tick(start+2000U);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && frame_down && tx_count==2);
  //Safety wins even with E3 pending; no search motion can restart from FAULT.
  exit180_setup(); fake_now=200; rx(0xE3); health_faults=MISSION_FAULT_IMU;
  exit180_tick(200); assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  assert(!frame_down && tx_count==0); //No turn-end handshake under a real fault.
  puts("PASS:TURN180 finishes before11/03; live RX cannot interrupt; old complete/partial frames isolated before TX, new in-TX04/E3 retained, fresh ID4, normal/2s timeout/wrap, silence and safety");
}
