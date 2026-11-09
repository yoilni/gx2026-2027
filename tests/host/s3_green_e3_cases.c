/* Initial21/green E3, using the real parser, class filter and search runner. */
static uint32_t green_e3_begin(uint32_t start,int32_t yaw) {
  setup(3,ROBOT_TEAM_BLUE,0);
  s7_target_switch_enabled=false;s3_target_select_command=0x21;
  mission_snapshot.yaw_cdeg=yaw;tx_count=0;
  fake_now=start;rx(0xE3);tick(start);
  assert(vision_search_green_e3_mode && !vision_search_e3_mode && !s51_e3_recovery_count);
  assert(s3_green_e3_phase==S3_GREEN_E3_FORWARD && mission_snapshot.state_entry_tick==start);
  tick(start+20U);
  assert(mission_snapshot.left_target_rpm==80 && mission_snapshot.right_target_rpm==80);
  tick(start+799U);assert(s3_green_e3_phase==S3_GREEN_E3_FORWARD);
  tick(start+800U);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW && mission_snapshot.state_entry_tick==start+800U);
  assert(s3_green_e3_phase==S3_GREEN_E3_SPIN_CCW && e3_spin_progress_cdeg==0);
  assert(!vision_search_forward_active && !tx_count && frame_down);
  return start+800U;
}

static void verify_initial_green_e3_spin(void) {
  assert(ROBOT_S3_GREEN_E3_SPIN_CDEG==36000L);
  const int32_t origins[]={0L,35000L,17900L};
  const int32_t progress[]={9000L,18000L,27000L,34999L,35000L,35999L,36000L};
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned y=0;y<3;++y) {
    uint32_t start=wrap?UINT32_MAX-1000U:1000U;
    uint32_t spin=green_e3_begin(start,origins[y]);
    tick(spin+20U);
    assert(mission_snapshot.left_target_rpm==-25 && mission_snapshot.right_target_rpm==25);
    tick(spin+2000U); // continuous360 never uses a2s fixed-heading shortcut
    assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW && !mission_snapshot.fault_flags);
    for(unsigned i=0;i<sizeof(progress)/sizeof(progress[0]);++i) {
      fake_now=spin+2020U+i*20U;rx(0xE3); // must not restart the angle/timer
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(origins[y]+ROBOT_YAW_LEFT_SIGN*progress[i]);
      tick(fake_now);
      assert(e3_spin_progress_cdeg==progress[i] && !tx_count && !s51_e3_recovery_count);
      assert(!s7_target_switch_enabled && s3_target_select_command==0x21 && frame_down);
      if(i+1<sizeof(progress)/sizeof(progress[0])) {
        assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW && mission_snapshot.state_entry_tick==spin);
        int16_t rpm=progress[i]>=35000L?12:25;
        assert(mission_snapshot.left_target_rpm==-rpm && mission_snapshot.right_target_rpm==rpm);
      }
    }
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !vision_search_green_e3_mode);
    assert(s3_search_sweep_completed && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    // A later E3 repeats the same green policy; it never selects51 or sends13.
    fake_now+=20;rx(0xE3);tick(fake_now);
    assert(s3_green_e3_phase==S3_GREEN_E3_FORWARD && !tx_count && s3_target_select_command==0x21);
  }
  // Only a fresh green target can interrupt either forward or spin.
  for(unsigned spinning=0;spinning<2;++spinning) {
    uint32_t spin;
    if(spinning) spin=green_e3_begin(1000,35000);
    else {
      setup(3,ROBOT_TEAM_BLUE,0);s7_target_switch_enabled=false;s3_target_select_command=0x21;tx_count=0;
      fake_now=1000;rx(0xE3);tick(fake_now);spin=1000;
    }
    for(unsigned id=3;id<=6;++id) {
      if(id==5) continue;
      fake_now=spin+20U;coords(id);tick(fake_now);
      assert(vision_search_green_e3_mode && !mission_snapshot.vision_target_valid);
    }
    fake_now=spin+40U;coords(5);tick(spin+541U);
    assert(vision_search_green_e3_mode && !mission_snapshot.vision_target_valid);
    fake_now=spin+561U;coords(5);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !vision_search_green_e3_mode && !tx_count);
    assert(mission_snapshot.vision_target_valid && mission_snapshot.target_id==5);
  }
  //20s timeout starts after the full800ms forward leg, including tick wrap.
  for(unsigned wrap=0;wrap<2;++wrap) {
    uint32_t spin=green_e3_begin(wrap?UINT32_MAX-300U:1000U,35000);
    tick(spin+19999U);assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
    tick(spin+20000U);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags & MISSION_FAULT_S3_TIMEOUT));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !tx_count);
  }
  const uint32_t faults[]={MISSION_FAULT_IMU,MISSION_FAULT_LEFT_MOTOR,MISSION_FAULT_RIGHT_MOTOR};
  for(unsigned i=0;i<3;++i) {
    uint32_t spin=green_e3_begin(1000,35000);health_faults=faults[i];tick(spin+20U);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm && !tx_count);
  }
  uint32_t spin=green_e3_begin(1000,35000);
  MissionTask_EnterState(MISSION_STATE_STOPPED,spin+20U);tick(spin+40U);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !mission_snapshot.left_target_rpm && !tx_count);
  puts("PASS:first-green E3 equal80/800 then directed gyro CCW360 at25/12rpm,359.99/360 boundary and yaw/tick wrap;ID5 fresh-only interrupts both legs,duplicates keep timer/progress,no13/no51,repeat remains21,20s spin timeout independent from2s turns,health/stop protection");
}
