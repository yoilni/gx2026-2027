/* Real UART parser + post22 WAIT/recovery handlers + ordinary03 E3 policy. */
static void post22_setup(void) {
  setup(3,ROBOT_TEAM_BLUE,0);
  s4_post22_search_active=false;
  s4_post22_failed_sweeps=0;
  MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY,100);
  frame_down=false; //The51/03 recovery entry must explicitly lower again.
  fake_now=120; rx(0xE2); tick(120);
  assert(frame_down);
  assert(s4_post22_search_active && s3_target_select_command==0x51);
  assert(mission_snapshot.state==MISSION_STATE_S4_POST22_REVERSE);
  assert(tx_count==4 && tx_commands[2]==0x51 && tx_commands[3]==0x03);
}
static uint32_t complete_post22_sweep(uint32_t start) {
  assert(mission_snapshot.state==MISSION_STATE_S4_POST22_REVERSE);
  tick(start+ROBOT_S4_POST22_REVERSE_MS);
  assert(mission_snapshot.state==MISSION_STATE_S4_POST22_TURN_LEFT);
  //Keep measured yaw unchanged to exercise the real per-leg turn upper bound.
  tick(start+ROBOT_S4_POST22_REVERSE_MS+ROBOT_HEADING_TURN_TIMEOUT_MS);
  assert(mission_snapshot.state==MISSION_STATE_S4_POST22_TURN_RIGHT);
  tick(start+ROBOT_S4_POST22_REVERSE_MS+2U*ROBOT_HEADING_TURN_TIMEOUT_MS);
  return start+ROBOT_S4_POST22_REVERSE_MS+2U*ROBOT_HEADING_TURN_TIMEOUT_MS;
}
static void verify_post22_search(void) {
  const MissionState states[]={MISSION_STATE_S4_POST22_REVERSE,
    MISSION_STATE_S4_POST22_TURN_LEFT,MISSION_STATE_S4_POST22_TURN_RIGHT,
    MISSION_STATE_S4_WAIT_FINAL_READY};
  for (unsigned i=0;i<sizeof(states)/sizeof(states[0]);++i) {
    post22_setup(); MissionTask_EnterState(states[i],130);
    fake_now=140; rx(0xE3); tick(140);
    assert(!s4_post22_search_active && s51_e3_recovery_count==1);
    assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
    assert(vision_search_e3_mode && vision_search_forward_rpm==-80 && vision_search_forward_ms==800);
    assert(tx_count==4); //No51/03 reinitialization at E3 handoff.
    rx(0xE3); tick(640);
    assert(s51_e3_recovery_count==1 && mission_snapshot.state_entry_tick==140);
    coords(6); tick(660);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s51_e3_recovery_count==1);
    rx(0xE3); tick(680);
    assert(s51_e3_recovery_count==2 && mission_snapshot.state==MISSION_STATE_S3_E3_TURN_AWAY);
    assert(vision_search_forward_rpm==100 && vision_search_forward_ms==1200);
    assert(tx_count==4);
  }
  //Fresh valid coordinates win over an old E3 in the same control cycle.
  post22_setup(); fake_now=140; rx(0xE3); coords(5); tick(140);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !s4_post22_search_active);
  assert(!s51_e3_recovery_count && tx_count==4);
  //Wrong classes and expired frames cannot mask E3 or restart PID.
  post22_setup(); fake_now=140; coords(4); rx(0xE3); tick(140);
  assert(s51_e3_recovery_count==1 && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  post22_setup(); fake_now=140; coords(6); rx(0xE3); tick(641);
  assert(s51_e3_recovery_count==1 && !s4_post22_search_active);
  //Silence/repeated E2 cannot keep cycling short sweeps indefinitely.
  post22_setup(); rx(0xE2); tick(140);
  assert(mission_snapshot.state_entry_tick==120 && tx_count==4);
  uint32_t done=complete_post22_sweep(120);
  assert(s4_post22_failed_sweeps==1 && mission_snapshot.state==MISSION_STATE_S4_WAIT_FINAL_READY);
  tick(done+ROBOT_S4_HANDSHAKE_TIMEOUT_MS-1);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_FINAL_READY);
  tick(done+ROBOT_S4_HANDSHAKE_TIMEOUT_MS);
  assert(s4_post22_failed_sweeps==1 && mission_snapshot.state==MISSION_STATE_S4_POST22_REVERSE);
  uint32_t start=mission_snapshot.state_entry_tick;
  complete_post22_sweep(start);
  assert(s4_post22_failed_sweeps==2 && !s4_post22_search_active);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW && vision_search_e3_mode);
  assert(!s51_e3_recovery_count && tx_count==4 && !mission_snapshot.fault_flags);
  tick(fake_now+ROBOT_E3_SEARCH_REVERSE_MS);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
  //New coordinates interrupt the escalated360 and resume without reinit.
  fake_now+=20; coords(6); tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==4);
  //04/24 preempt either the wait or any short sweep just as before.
  for (unsigned i=0;i<sizeof(states)/sizeof(states[0]);++i) {
    for (unsigned cmd=0;cmd<2;++cmd) {
      post22_setup(); MissionTask_EnterState(states[i],130);
      fake_now=140; rx(cmd?0x24:0x04); tick(140);
      assert(mission_snapshot.state==(cmd?MISSION_STATE_S4_RAISE_FRAME:MISSION_STATE_S4_TRACK_CENTER));
      assert(!s4_post22_search_active && !s51_e3_recovery_count);
    }
  }
  //Before07 no51/03 switch or reinterpretation of the green E2 handshake.
  setup(3,ROBOT_TEAM_BLUE,0); s7_target_switch_enabled=false;
  s3_target_select_command=0x21; s4_post22_search_active=false;
  MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY,100);
  unsigned sent=tx_count; fake_now=120; rx(0xE2); tick(120);
  assert(!s4_post22_search_active && s3_target_select_command==0x21 && tx_count==sent);
  //Hardware faults and operator stop still outrank any new recovery.
  post22_setup(); health_faults=MISSION_FAULT_IMU; rx(0xE3); tick(140);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  post22_setup(); MissionTask_EnterState(MISSION_STATE_STOPPED,140); rx(0xE3); tick(160);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !mission_snapshot.right_target_rpm);
  puts("PASS:post22 TX51/03 E3 handoff in all wait/sweep phases, retained ordinary E3 count, fresh/stale/class/duplicate gates; two failed sweeps escalate360 without resend;04/24 and safety priority");
}
